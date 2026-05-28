# A thin cursor + watermark over a contiguous byte block (`Memory{UInt8}` or
# `Vector{UInt8}`) so the reader/writer fast paths can target a pre-allocated
# buffer without an IOBuffer's bookkeeping. The backing store is a type
# parameter `S` so the field stays concrete — both storage types expose
# identical `pointer(_, i)` / `length` so every hot-path body is shared.
# The cursor semantics match IOBuffer: `pos` is the 1-based index of the next
# byte to read/write; `written` is the count of bytes containing real data
# (0-based watermark, equal to position after the last write).
mutable struct MemBuf{S <: DenseVector{UInt8}} <: IO
    mem::S
    pos::Int
    written::Int

    MemBuf(mem::S, pos::Int=1, written::Int=length(mem)) where {S <: DenseVector{UInt8}} =
        new{S}(mem, pos, written)
end

const _CDRBufLike = Union{IOBuffer, MemBuf}

# Field accessor helpers. The reader/writer hot paths call these instead of
# touching `.data`/`.ptr`/`.size` directly, so MemBuf can use whatever field
# names it likes without coupling to IOBuffer's internals.
@inline _buf_data(b::IOBuffer) = b.data
@inline _buf_data(b::MemBuf)   = b.mem

@inline _buf_pos(b::IOBuffer)  = b.ptr
@inline _buf_pos(b::MemBuf)    = b.pos
@inline _buf_pos!(b::IOBuffer, p::Int) = (b.ptr = p)
@inline _buf_pos!(b::MemBuf,   p::Int) = (b.pos = p)

@inline _buf_size(b::IOBuffer) = b.size
@inline _buf_size(b::MemBuf)   = b.written
@inline _buf_size!(b::IOBuffer, s::Int) = (b.size = s)
@inline _buf_size!(b::MemBuf,   s::Int) = (b.written = s)

# IOBuffer grows; MemBuf is fixed-size and throws if a write would overrun.
@inline _ensureroom!(b::IOBuffer, n::Integer) = (Base.ensureroom(b, n); nothing)
@inline function _ensureroom!(b::MemBuf, n::Integer)
    needed = b.pos + Int(n) - 1
    needed > length(b.mem) && throw(BoundsError(b.mem, needed))
    return nothing
end

# After advancing the cursor, bump the watermark if we wrote past it.
@inline function _advance_written!(b::_CDRBufLike, new_pos::Int)
    new_pos - 1 > _buf_size(b) && _buf_size!(b, new_pos - 1)
    _buf_pos!(b, new_pos)
    return nothing
end

# IO interface for MemBuf — covers the slow paths that fall through to
# `Base.read(io, T)` / `Base.write(io, v)` (e.g. the writer's preamble setup,
# the reader's `String` byte-block grab, the per-element BE swap loops).

Base.position(b::MemBuf) = b.pos - 1
Base.eof(b::MemBuf)      = b.pos > b.written
Base.bytesavailable(b::MemBuf) = max(0, b.written - (b.pos - 1))

function Base.seek(b::MemBuf, n::Integer)
    n < 0 && throw(ArgumentError("MemBuf seek to negative position $n"))
    b.pos = Int(n) + 1
    return b
end
Base.seekstart(b::MemBuf) = seek(b, 0)

function Base.skip(b::MemBuf, n::Integer)
    b.pos += Int(n)
    return b
end

# `read(::IO, ::Type{UInt8})` is the one byte-read method Base mandates.
# Base's generic typed reads (UInt32 for the preamble, etc.) decompose
# through `unsafe_read` below; the CDRReader fast paths bypass all of this
# and load from the buffer directly via `_read_prim`.
@inline function Base.read(b::MemBuf, ::Type{UInt8})
    _ensureroom_read!(b, 1)
    mem = b.mem
    p = b.pos
    v = GC.@preserve mem unsafe_load(pointer(mem, p))
    b.pos = p + 1
    return v
end

# Read range error mirrors IOBuffer's behavior — short read raises EOFError.
@inline function _ensureroom_read!(b::MemBuf, n::Int)
    b.pos + n - 1 > b.written && throw(EOFError())
    return nothing
end

function Base.read(b::MemBuf, n::Integer)
    nb = Int(n)
    _ensureroom_read!(b, nb)
    out = Vector{UInt8}(undef, nb)
    mem = b.mem
    p = b.pos
    GC.@preserve mem out unsafe_copyto!(pointer(out), pointer(mem, p), nb)
    b.pos = p + nb
    return out
end

function Base.unsafe_read(b::MemBuf, dst::Ptr{UInt8}, nb::UInt)
    n = Int(nb)
    _ensureroom_read!(b, n)
    mem = b.mem
    p = b.pos
    GC.@preserve mem unsafe_copyto!(dst, pointer(mem, p), n)
    b.pos = p + n
    return nothing
end

# `write(::IO, ::UInt8)` is the one byte-write method Base mandates every IO
# define. Base's generic numeric `write` methods (Int16…Float64) decompose
# through `unsafe_write` below, so no per-type methods are needed — and
# defining them would collide with Base's own union overloads. These slow
# paths only carry the preamble + String terminators; the CDRWriter fast
# paths store into the buffer directly, never through these.
@inline function Base.write(b::MemBuf, v::UInt8)
    _ensureroom!(b, 1)
    mem = b.mem
    p = b.pos
    GC.@preserve mem unsafe_store!(pointer(mem, p), v)
    _advance_written!(b, p + 1)
    return 1
end

function Base.write(b::MemBuf, s::String)
    n = sizeof(s)
    _ensureroom!(b, n)
    mem = b.mem
    p = b.pos
    GC.@preserve mem s unsafe_copyto!(pointer(mem, p), pointer(s), n)
    _advance_written!(b, p + n)
    return n
end

function Base.unsafe_write(b::MemBuf, src::Ptr{UInt8}, nb::UInt)
    n = Int(nb)
    _ensureroom!(b, n)
    mem = b.mem
    p = b.pos
    GC.@preserve mem unsafe_copyto!(pointer(mem, p), src, n)
    _advance_written!(b, p + n)
    return n
end
