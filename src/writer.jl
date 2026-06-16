# `IsCDR2` and `LE` are type parameters so kind- and endian-specific paths
# dispatch at compile time. `B` is the backing buffer type (IOBuffer or
# MemBuf) — kept last so `CDRWriter{IsCDR2}` / `CDRWriter{<:Any, LE}`
# dispatch elsewhere is unaffected.
mutable struct CDRWriter{IsCDR2, LE, B <: _CDRBufLike}
    buf::B

    usesDelimiterHeader::Bool
    usesMemberHeader::Bool

    origin::Int
    kind::EncapsulationKind

    function CDRWriter(buf::B = IOBuffer(), kind::EncapsulationKind=CDR_LE) where B <: _CDRBufLike
        isCDR2, littleEndian, usesDelimiterHeader, usesMemberHeader = getEncapsulationKind(UInt8(kind))
        write(buf, UInt8(0))
        write(buf, UInt8(kind))
        write(buf, UInt16(0))
        new{isCDR2, littleEndian, B}(buf, usesDelimiterHeader, usesMemberHeader, 4, kind)
    end
end

# Write into a pre-allocated raw byte block — a `Memory{UInt8}` or a
# `Vector{UInt8}` — the caller being responsible for sizing it (e.g. via
# CDRSizeCalculator). The 4-byte preamble is written first, exactly as for
# an IOBuffer.
CDRWriter(mem::DenseVector{UInt8}, kind::EncapsulationKind=CDR_LE) =
    CDRWriter(MemBuf(mem, 1, 0), kind)

# Kind-explicit construction, mirroring `CDRReader(buf, Val(CDR_LE))`. The runtime-kind
# `CDRWriter(buf[, kind])` derives `IsCDR2`/`LE` from a value, so its type — and thus the
# `write` dispatched on it — is runtime-determined; inference can't pin it, so a caller
# encoding into a fresh buffer pays a dynamic dispatch (and box) on `write`. Declaring the
# kind up front fixes the type at the call site. @generated so kind → (IsCDR2, LE) splits
# at expansion; routes through the runtime-kind ctor (which writes + validates the preamble)
# then asserts the now-known concrete type.
@generated function CDRWriter(buf::B, ::Val{K}) where {B <: _CDRBufLike, K}
    K isa EncapsulationKind ||
        return :(throw(ArgumentError(string("CDRWriter: expected an EncapsulationKind, got ",
                                            $(QuoteNode(K))))))
    isCDR2, le, _, _ = getEncapsulationKind(UInt8(K))
    return :(CDRWriter(buf, $K)::CDRWriter{$isCDR2, $le, B})
end
CDRWriter(mem::DenseVector{UInt8}, v::Val) = CDRWriter(MemBuf(mem, 1, 0), v)

@inline isCDR2(::CDRWriter{B}) where B = B
@inline littleEndian(::CDRWriter{<:Any, L}) where L = L

@inline _maybe_swap(::CDRWriter{<:Any, true},  v) = v
@inline _maybe_swap(::CDRWriter{<:Any, false}, v) = sizeof(v) == 1 ? v : ntoh(v)

function resetOrigin(c::CDRWriter)
    c.origin = position(c.buf)
end

@inline function _emit_padding!(buf::_CDRBufLike, padding)
    _ensureroom!(buf, padding)
    data = _buf_data(buf)
    base = _buf_pos(buf)
    GC.@preserve data begin
        for i in 0:padding-1
            unsafe_store!(pointer(data, base + i), UInt8(0))
        end
    end
    new_ptr = base + padding
    if new_ptr - 1 > _buf_size(buf)
        _buf_size!(buf, new_ptr - 1)
    end
    _buf_pos!(buf, new_ptr)
    return
end

# CDR alignment sizes are powers of two; the bitmask form folds when `size`
# is constant-propagated from a literal or `sizeof(T)`.
@inline function align(c::CDRWriter, size::Int)
    buf = c.buf
    alignment = (_buf_pos(buf) - 1 - c.origin) & (size - 1)
    alignment == 0 && return
    _emit_padding!(buf, size - alignment)
end

# 8-byte primitives align to 4 on CDR2, 8 on CDR1.
@inline align8(c::CDRWriter{true})  = align(c, 4)
@inline align8(c::CDRWriter{false}) = align(c, 8)

# A typed `unsafe_store!` directly into the backing buffer — bypasses
# `Base.unsafe_write`'s flag checks and per-byte copy loop.
@inline function _write_prim(buf::_CDRBufLike, v::T) where T
    _ensureroom!(buf, sizeof(T))
    _write_prim_unchecked!(buf, v)
end

@inline function _write_prim_unchecked!(buf::_CDRBufLike, v::T) where T
    n = sizeof(T)
    data = _buf_data(buf)
    ptr = _buf_pos(buf)
    GC.@preserve data unsafe_store!(Ptr{T}(pointer(data, ptr)), v)
    new_ptr = ptr + n
    if new_ptr - 1 > _buf_size(buf)
        _buf_size!(buf, new_ptr - 1)
    end
    _buf_pos!(buf, new_ptr)
    return n
end

function Base.write(c::CDRWriter, v::Union{Int8, UInt8, Bool})
    align(c, 1)
    _write_prim(c.buf, v)
end

function Base.write(c::CDRWriter, v::T) where T <: Union{Int16, UInt16, Int32, UInt32, Float32}
    align(c, sizeof(T))
    _write_prim(c.buf, _maybe_swap(c, v))
end

Base.write(c::CDRWriter, v::Char) = write(c, UInt8(v))

presentFlag(::CDRWriter{false}, ::Bool) = throw(ArgumentError("presentFlag is only valid for CDR2 streams"))
presentFlag(c::CDRWriter{true}, value::Bool) = write(c, UInt8(value ? 1 : 0))

function uintBE(c::CDRWriter, v::T) where T <: Union{UInt16, UInt32}
    align(c, sizeof(T))
    _write_prim(c.buf, hton(v))
end
function uintBE(c::CDRWriter, v::UInt64)
    align8(c)
    _write_prim(c.buf, hton(v))
end

uint16BE(c::CDRWriter, v::UInt16) = uintBE(c, v)
uint32BE(c::CDRWriter, v::UInt32) = uintBE(c, v)
uint64BE(c::CDRWriter, v::UInt64) = uintBE(c, v)

Base.position(c::CDRWriter) = position(c.buf)
data(c::CDRWriter) = view(_buf_data(c.buf), 1:position(c.buf))

function Base.write(c::CDRWriter, v::T) where T <: Union{Float64, UInt64, Int64}
    align8(c)
    _write_prim(c.buf, _maybe_swap(c, v))
end

function Base.write(c::CDRWriter, v::String, writeLength=true)
    if writeLength
        write(c, UInt32(sizeof(v) + 1))
    end
    write(c.buf, v)
    write(c.buf, UInt8('\0'))
end

dHeader(c::CDRWriter, objectSize) = write(c, UInt32(objectSize))

emHeader(c::CDRWriter{true},  mustUnderstand::Bool, id::Int, objectSize::Int, lengthCode::Union{Nothing, Int}) =
    memberHeaderV2(c, mustUnderstand, id, objectSize, lengthCode)
emHeader(c::CDRWriter{false}, mustUnderstand::Bool, id::Int, objectSize::Int, lengthCode::Union{Nothing, Int}) =
    memberHeaderV1(c, mustUnderstand, id, objectSize, lengthCode)
function memberHeaderV1(c::CDRWriter, mustUnderstand::Bool, id::Int, objectSize::Int, lengthCode::Union{Nothing, Int})
    align(c, 4)
    mustUnderstandFlag = mustUnderstand ? 1 << 14 : 0
    shouldUseExtendedPID = id > 0x3f00 || objectSize > 0xffff

    if !shouldUseExtendedPID
      idHeader = mustUnderstandFlag | id
      write(c, UInt16(idHeader))
      objectSizeHeader = objectSize & 0xffff
      write(c, UInt16(objectSizeHeader))
    else
      extendedHeader = mustUnderstandFlag | EXTENDED_PID;
      write(c, UInt16(extendedHeader))
      write(c, UInt16(8)) # size of next two parameters
      write(c, UInt32(id))
      write(c, UInt32(objectSize))
    end
    resetOrigin(c)
end

sentinelHeader(::CDRWriter{true}) = nothing
function sentinelHeader(c::CDRWriter{false})
    align(c, 4)
    write(c, UInt16(SENTINEL_PID))
    write(c, UInt16(0))
end

function getLengthCodeForObjectSize(objectSize)
    if objectSize == 1
        return 0
    elseif objectSize == 2
        return 1
    elseif objectSize == 4
        return 2
    elseif objectSize == 8
        return 3
    end
    if objectSize > 0xffffffff
        throw(ArgumentError("Object size $objectSize is too large; max value is $(0xffffffff)"))
    end
    return 4
end

function memberHeaderV2(c::CDRWriter, mustUnderstand::Bool, id::Int, objectSize::Int, lengthCode::Union{Nothing, Int})
    if id > 0x0fffffff
        throw(ArgumentError("Member ID $id is too large; max value is $(0x0fffffff)"))
    end
    # EMHEADER wire layout: M_FLAG<<31 | LC<<28 | id
    mustUnderstandFlag = mustUnderstand ? 1 << 31 : 0
    finalLengthCode = !isnothing(lengthCode) ? lengthCode : getLengthCodeForObjectSize(objectSize)

    header = mustUnderstandFlag | (finalLengthCode << 28) | id

    write(c, UInt32(header))

    if finalLengthCode == 0 || finalLengthCode == 1 ||
        finalLengthCode == 2 || finalLengthCode == 3
        shouldBeSize = lengthCodeToObjectSize(finalLengthCode)
        if objectSize != shouldBeSize
            throw(ArgumentError("Cannot write a length code $(finalLengthCode) header with an object size not equal to $(shouldBeSize)"))
        end
    elseif finalLengthCode == 4 || finalLengthCode == 5
        return write(c, UInt32(objectSize))
    elseif finalLengthCode == 6
        if objectSize % 4 !== 0
            throw(ArgumentError("Cannot write a length code 6 header with an object size that is not a multiple of 4"))
        end
        return write(c, UInt32(objectSize >> 2))
    elseif finalLengthCode == 7
        if objectSize % 8 !== 0
            throw(ArgumentError("Cannot write a length code 7 header with an object size that is not a multiple of 8"))
        end
        return write(c, UInt32(objectSize >> 3))
    else 
        error("Unexpected length code $finalLengthCode")
    end
end

sequenceLength(w::CDRWriter, len) = write(w, UInt32(len))

@inline function _bulk_copy_into!(buf::_CDRBufLike, src::Ptr{UInt8}, nb::Int)
    _ensureroom!(buf, nb)
    data = _buf_data(buf)
    ptr = _buf_pos(buf)
    GC.@preserve data Base.unsafe_copyto!(pointer(data, ptr), src, nb)
    new_ptr = ptr + nb
    if new_ptr - 1 > _buf_size(buf)
        _buf_size!(buf, new_ptr - 1)
    end
    _buf_pos!(buf, new_ptr)
    return
end

@inline function _writeArrayBody!(w::CDRWriter{<:Any, true}, a::AbstractArray{T}) where T
    GC.@preserve a _bulk_copy_into!(w.buf, Ptr{UInt8}(pointer(a)), length(a) * sizeof(T))
    return nothing
end

@inline function _writeArrayBody!(w::CDRWriter{<:Any, false}, a::AbstractArray{T}) where T
    if sizeof(T) == 1
        GC.@preserve a _bulk_copy_into!(w.buf, Ptr{UInt8}(pointer(a)), length(a))
    else
        for v in a
            _write_prim(w.buf, ntoh(v))
        end
    end
    return nothing
end

# `unsafe_store!` of the SArray value avoids a `Ref(a)` indirection that
# would heap-allocate under Julia 1.11.
@inline function _writeStaticArrayBody!(w::CDRWriter{<:Any, true}, a::SArray{S, T, N, L}) where {S, T, N, L}
    nb = L * sizeof(T)
    buf = w.buf
    _ensureroom!(buf, nb)
    data = _buf_data(buf)
    ptr = _buf_pos(buf)
    GC.@preserve data unsafe_store!(Ptr{SArray{S, T, N, L}}(pointer(data, ptr)), a)
    new_ptr = ptr + nb
    if new_ptr - 1 > _buf_size(buf)
        _buf_size!(buf, new_ptr - 1)
    end
    _buf_pos!(buf, new_ptr)
    return nothing
end

@inline function _writeStaticArrayBody!(w::CDRWriter{<:Any, false}, a::SArray{S, T, N, L}) where {S, T, N, L}
    if sizeof(T) == 1
        nb = L
        buf = w.buf
        _ensureroom!(buf, nb)
        data = _buf_data(buf)
        ptr = _buf_pos(buf)
        GC.@preserve data unsafe_store!(Ptr{SArray{S, T, N, L}}(pointer(data, ptr)), a)
        new_ptr = ptr + nb
        if new_ptr - 1 > _buf_size(buf)
            _buf_size!(buf, new_ptr - 1)
        end
        _buf_pos!(buf, new_ptr)
    else
        for v in a
            _write_prim(w.buf, ntoh(v))
        end
    end
    return nothing
end

# Dynamic sequence: `writeLength` defaults to `true` — the u32 length prefix is
# part of a CDR sequence and the reader (`read(r, Vector{T})`) and size
# calculator both assume it. Pass `false` only when emitting a bare element run
# whose count the caller writes separately. (The fixed-length `SArray` methods
# below default to `false`: an `SArray` is a CDR array, which carries no prefix.)
function Base.write(w::CDRWriter, a::AbstractArray{T}, writeLength=true) where T <: Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32}
    if writeLength
        sequenceLength(w, length(a))
    end
    isempty(a) && return
    align(w, sizeof(T))
    _writeArrayBody!(w, a)
end

function Base.write(w::CDRWriter, a::A, writeLength=false) where {T <: Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32}, S, N, L, A<:SArray{S, T, N, L}}
    if writeLength
        sequenceLength(w, L)
    end
    L == 0 && return nothing
    align(w, sizeof(T))
    _writeStaticArrayBody!(w, a)
    return nothing
end

function Base.write(w::CDRWriter, a::AbstractArray{T}, writeLength=true) where T <: Union{Int64, UInt64, Float64}
    if writeLength
        sequenceLength(w, length(a))
    end
    isempty(a) && return
    align8(w)
    _writeArrayBody!(w, a)
end

function Base.write(w::CDRWriter, a::A, writeLength=false) where {T <: Union{Int64, UInt64, Float64}, S, N, L, A<:SArray{S, T, N, L}}
    if writeLength
        sequenceLength(w, L)
    end
    L == 0 && return nothing
    align8(w)
    _writeStaticArrayBody!(w, a)
    return nothing
end

function Base.write(w::CDRWriter, a::A, writeLength=true) where A<:AbstractArray{String}
    if writeLength
        sequenceLength(w, length(a))
    end
    for s in a
        write(w, s)
    end
end

# --- Generic struct / sequence writes ---------------------------------------
#
# These are the exact inverse of the generic reads (`_read_value` /
# `_read_user_struct` and `read(r, Vector{T})` / `read(r, ::Type{SArray})`):
# each struct field is written at its own CDR alignment relative to the message
# origin — NOT packed to the struct's widest member the way a single `write_all!`
# run does. That field-independent alignment is what guarantees a flat-but-non-
# compact struct embedded at an arbitrary offset round-trips through the field-
# walk reader. A genuinely compact struct still takes the single-`unsafe_store!`
# fast path, which is bit-identical to the field walk.

# Compact-struct fast path: one wide store of the whole value (mirror of
# `_read_struct_compact`). The caller guarantees compactness via
# `_is_compact_struct`.
@generated function _write_struct_compact(c::C, x::T) where {C <: CDRWriter, T}
    isCDR2 = C.parameters[1]
    max_align = _wa_align_for(T, isCDR2)
    sz = sizeof(T)
    return quote
        align(c, $max_align)
        buf = c.buf
        _ensureroom!(buf, $sz)
        data = _buf_data(buf)
        ptr = _buf_pos(buf)
        GC.@preserve data unsafe_store!(Ptr{$T}(pointer(data, ptr)), x)
        new_ptr = ptr + $sz
        if new_ptr - 1 > _buf_size(buf)
            _buf_size!(buf, new_ptr - 1)
        end
        _buf_pos!(buf, new_ptr)
        return nothing
    end
end

# Field-walk write: each field through its own `write`, the inverse of
# `_read_user_struct`. Array fields carry the same length-prefix convention the
# reader assumes — dynamic sequences (`Vector`) are length-prefixed, fixed-length
# `SArray`s are not — so a field whose `write` defaults to no prefix (the
# primitive/string array methods) is still given one here.
@generated function _write_user_struct(c::CDRWriter, x::T) where T
    if isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0
        stmts = Expr[]
        for i in 1:fieldcount(T)
            FT = fieldtype(T, i)
            fv = :(getfield(x, $i))
            if FT <: SArray
                push!(stmts, :(write(c, $fv)))        # fixed-length, no prefix
            elseif FT <: AbstractArray
                push!(stmts, :(write(c, $fv, true)))  # dynamic sequence, length prefix
            else
                push!(stmts, :(write(c, $fv)))        # scalar / String / nested struct
            end
        end
        push!(stmts, :(return nothing))
        return Expr(:block, stmts...)
    end
    return :(throw(ArgumentError(string("write: unsupported type ", $T))))
end

@generated function _write_value(c::C, x::T) where {C <: CDRWriter, T}
    isCDR2 = C.parameters[1]
    LE     = C.parameters[2]
    if _is_compact_struct(T, isCDR2, LE)
        return :(_write_struct_compact(c, x))
    end
    return :(_write_user_struct(c, x))
end

# Generic single-value catch-all (mirror of `read(r, ::Type{T})`). Returns the
# byte count — including alignment padding — as `Base.write` callers expect. The
# primitive / String / array / SArray methods above are all more specific, so
# this only fires for user structs (and anything else falls through to the
# `_write_user_struct` error).
function Base.write(c::CDRWriter, x::T) where T
    p0 = position(c)
    _write_value(c, x)
    return position(c) - p0
end

# Sequence of non-primitive elements: u32 length prefix then each element through
# its own `write` (mirror of `read(r, Vector{T})` for non-primitive `T`). Unlike
# the primitive-element array methods — whose `writeLength` defaults to `false`
# because `write_all!` manages their prefixes — this defaults to `true` so a bare
# `write(c, vec_of_struct)` is the inverse of the reader, which reads the prefix
# by default.
function Base.write(c::CDRWriter, a::AbstractArray{T}, writeLength=true) where T
    writeLength && sequenceLength(c, length(a))
    for elt in a
        write(c, elt)
    end
    return nothing
end

# SArray of non-primitive elements: fixed-length, no prefix (mirror of
# `read(r, ::Type{SArray})` for non-primitive elements).
function Base.write(c::CDRWriter, a::SArray{S, T, N, L}) where {S, T, N, L}
    for elt in a
        write(c, elt)
    end
    return nothing
end

# Unchecked variants used by `write_all!` after a single upfront ensureroom.
# All caller-side writes go through these so the body has no internal
# room-check branches.

@inline function _align_unchecked!(c::CDRWriter, size::Int)
    buf = c.buf
    alignment = (_buf_pos(buf) - 1 - c.origin) & (size - 1)
    alignment == 0 && return
    padding = size - alignment
    data = _buf_data(buf)
    base = _buf_pos(buf)
    GC.@preserve data begin
        for i in 0:padding-1
            unsafe_store!(pointer(data, base + i), UInt8(0))
        end
    end
    new_ptr = base + padding
    if new_ptr - 1 > _buf_size(buf)
        _buf_size!(buf, new_ptr - 1)
    end
    _buf_pos!(buf, new_ptr)
    return
end

@inline _align8_unchecked!(c::CDRWriter{true})  = _align_unchecked!(c, 4)
@inline _align8_unchecked!(c::CDRWriter{false}) = _align_unchecked!(c, 8)

@inline function _write_string_unchecked!(c::CDRWriter, s::String)
    buf = c.buf
    _align_unchecked!(c, 4)
    _write_prim_unchecked!(buf, _maybe_swap(c, UInt32(sizeof(s) + 1)))
    n = sizeof(s)
    data = _buf_data(buf)
    ptr = _buf_pos(buf)
    GC.@preserve s data Base.unsafe_copyto!(pointer(data, ptr), pointer(s), n)
    new_ptr = ptr + n
    GC.@preserve data unsafe_store!(pointer(data, new_ptr), UInt8(0))
    new_ptr += 1
    if new_ptr - 1 > _buf_size(buf)
        _buf_size!(buf, new_ptr - 1)
    end
    _buf_pos!(buf, new_ptr)
    return nothing
end

@inline function _write_array_unchecked!(c::CDRWriter, a::AbstractArray{T}) where T <: Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32}
    buf = c.buf
    _align_unchecked!(c, 4)
    _write_prim_unchecked!(buf, _maybe_swap(c, UInt32(length(a))))
    isempty(a) && return nothing
    _align_unchecked!(c, sizeof(T))
    nb = length(a) * sizeof(T)
    data = _buf_data(buf)
    ptr = _buf_pos(buf)
    if littleEndian(c) || sizeof(T) == 1
        GC.@preserve a data Base.unsafe_copyto!(pointer(data, ptr), Ptr{UInt8}(pointer(a)), nb)
        new_ptr = ptr + nb
        if new_ptr - 1 > _buf_size(buf)
            _buf_size!(buf, new_ptr - 1)
        end
        _buf_pos!(buf, new_ptr)
    else
        for v in a
            _write_prim_unchecked!(buf, ntoh(v))
        end
    end
    return nothing
end

@inline function _write_array_unchecked!(c::CDRWriter, a::AbstractArray{T}) where T <: Union{Int64, UInt64, Float64}
    buf = c.buf
    _align_unchecked!(c, 4)
    _write_prim_unchecked!(buf, _maybe_swap(c, UInt32(length(a))))
    isempty(a) && return nothing
    _align8_unchecked!(c)
    nb = length(a) * sizeof(T)
    data = _buf_data(buf)
    ptr = _buf_pos(buf)
    if littleEndian(c)
        GC.@preserve a data Base.unsafe_copyto!(pointer(data, ptr), Ptr{UInt8}(pointer(a)), nb)
        new_ptr = ptr + nb
        if new_ptr - 1 > _buf_size(buf)
            _buf_size!(buf, new_ptr - 1)
        end
        _buf_pos!(buf, new_ptr)
    else
        for v in a
            _write_prim_unchecked!(buf, ntoh(v))
        end
    end
    return nothing
end

@inline function _write_array_unchecked!(c::CDRWriter, a::AbstractArray{String})
    buf = c.buf
    _align_unchecked!(c, 4)
    _write_prim_unchecked!(buf, _maybe_swap(c, UInt32(length(a))))
    for s in a
        _write_string_unchecked!(c, s)
    end
    return nothing
end

# Vectors of user structs: length prefix then each element through the
# normal write path (which handles struct expansion). Each element's
# internal ensureroom hits the fast path because the outer write_all!
# budget already covered the worst-case size.
@inline function _write_array_unchecked!(c::CDRWriter, a::AbstractArray{T}) where T
    buf = c.buf
    _align_unchecked!(c, 4)
    _write_prim_unchecked!(buf, _maybe_swap(c, UInt32(length(a))))
    for elt in a
        write_all!(c, elt)
    end
    return nothing
end

# SArrays of structs or of other SArrays: no length prefix (SArray is the
# fixed-length array form in CDR IDL). Each element flows through
# `write_all!`, which dispatches to the right path per element type.
@inline function _write_array_unchecked!(c::CDRWriter, a::SArray{S, T, N, L}) where {S, T, N, L}
    L == 0 && return nothing
    for elt in a
        write_all!(c, elt)
    end
    return nothing
end

# Walk a user struct down to its leaf members. CDR encodes nested structs
# inline with no per-struct headers; the flat leaf stream IS the wire
# format. Compact structs (where Julia's layout matches CDR's) bypass the
# per-field expansion and go through a single `unsafe_store!` of the whole
# struct value.
function _expand_schema!(out_types::Vector{Any}, out_exprs::Vector{Any}, ::Type{T}, val_expr; isCDR2::Bool=false, LE::Bool=true) where T
    if _is_packed_leaf(T) || T <: AbstractString || T <: AbstractArray
        push!(out_types, T)
        push!(out_exprs, val_expr)
        return
    end
    if _is_compact_struct(T, isCDR2, LE)
        push!(out_types, T)
        push!(out_exprs, val_expr)
        return
    end
    if isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0
        for i in 1:fieldcount(T)
            FT = fieldtype(T, i)
            fname = fieldname(T, i)
            _expand_schema!(out_types, out_exprs, FT,
                            :(getfield($val_expr, $(QuoteNode(fname))));
                            isCDR2=isCDR2, LE=LE)
        end
        return
    end
    error("write_all!: unsupported type $T")
end

"""
    write_all!(c::CDRWriter, vs...)
    write_all!(c::CDRWriter, ::Type{Tuple{T1, T2, …}}, vs...)

Write multiple values to `c`. Contiguous runs of primitives and SArrays-of-
primitive are emitted as a single packed operation with offsets and padding
resolved at compile time; strings, vectors, and other dynamic-length values
are dispatched to their per-type `write(c, v)` method. The schema overload
type-asserts each value against its declared slot.

# Examples
```julia
write_all!(c, UInt8(1), Int16(2), Float64(3.14))
write_all!(c, UInt32(len), "header", SVector(1.0, 2.0, 3.0))
write_all!(c, Tuple{UInt8, Int16, Float64}, 1, 2, 3.14)
```
"""
function write_all! end

# One packed run: inline alignment + typed stores at constant offsets,
# wrapped in a `let` so multiple runs in the same function don't collide on
# local names. The caller is responsible for ensureroom (the outer
# `write_all!` budget covers the whole batch).
function _wa_packed_run_expr(types::Vector, value_exprs::Vector, isCDR2::Bool, LE::Bool)
    max_align = maximum(_wa_align_for(T, isCDR2) for T in types)

    offsets = Int[]
    cur = 0
    for T in types
        a = _wa_align_for(T, isCDR2)
        rem = cur % a
        rem != 0 && (cur += a - rem)
        push!(offsets, cur)
        cur += _wa_size_for(T)
    end
    total_size = cur

    stmts = Expr[]
    push!(stmts, :(buf = c.buf))
    push!(stmts, :(data = _buf_data(buf)))
    # `pointer(data)` cached once; LLVM otherwise reloads `data.ptr` before
    # every typed store because TBAA can't rule out aliasing.
    push!(stmts, :(base_ptr = pointer(data)))

    # A single `max_align`-sized zero store covers all 0..max_align-1
    # padding cases; trailing zero bytes sit at the start of the data
    # region and are overwritten by the value stores.
    # `-offset & (max_align - 1)` is the branchless padding count.
    if max_align == 1
        push!(stmts, :(local_ptr = _buf_pos(buf)))
    else
        zero_t = max_align == 2 ? UInt16 :
                 max_align == 4 ? UInt32 :
                                  UInt64
        push!(stmts, quote
            _old_ptr = _buf_pos(buf)
            _pad = (-((_old_ptr - 1 - c.origin) & $(max_align - 1))) & $(max_align - 1)
            GC.@preserve data unsafe_store!(Ptr{$zero_t}(base_ptr + (_old_ptr - 1)),
                                            $zero_t(0))
            local_ptr = _old_ptr + _pad
        end)
    end

    store_exprs = Expr[:(_base = base_ptr + (local_ptr - 1))]

    # Zero inter-element alignment padding: bytes inside the run that no value
    # store covers. The leading-pad zero-store above only covers the dynamic
    # run-start padding, not gaps between fields (e.g. the 3 bytes between a
    # `UInt8` at offset 8 and a `UInt32` at offset 12). Left unzeroed these
    # carry whatever the buffer held — diverging from standard CDR (which zeros
    # pad) and leaking uninitialized heap memory onto the wire. This matches the
    # per-field path's `_emit_padding!`/`_align_unchecked!`, which zero pad.
    covered = falses(total_size)
    for (i, T) in enumerate(types)
        sz = _wa_size_for(T)
        for b in offsets[i]:(offsets[i] + sz - 1)
            covered[b + 1] = true
        end
    end
    for g in 0:(total_size - 1)
        covered[g + 1] && continue
        push!(store_exprs, :(unsafe_store!(Ptr{UInt8}(_base + $g), 0x00)))
    end

    for (i, T) in enumerate(types)
        off = offsets[i]
        v = value_exprs[i]
        if T <: SArray || _is_compact_struct(T, isCDR2, LE)
            # Both SArray and compact structs go through a single
            # `unsafe_store!` of the whole value, which LLVM lowers to a
            # wide SIMD memcpy.
            push!(store_exprs, :(unsafe_store!(Ptr{$T}(_base + $off), $v)))
        elseif T <: Union{Int8, UInt8, Bool}
            push!(store_exprs, :(unsafe_store!(Ptr{$T}(_base + $off), $v)))
        elseif T === Char
            push!(store_exprs, :(unsafe_store!(Ptr{UInt8}(_base + $off), UInt8($v))))
        else
            push!(store_exprs, :(unsafe_store!(Ptr{$T}(_base + $off), _maybe_swap(c, $v))))
        end
    end

    push!(stmts, Expr(:macrocall, GlobalRef(Base.GC, Symbol("@preserve")),
                     LineNumberNode(0, :write_all!),
                     :data, Expr(:block, store_exprs...)))

    push!(stmts, :(new_ptr = local_ptr + $total_size))
    push!(stmts, :(new_ptr - 1 > _buf_size(buf) && _buf_size!(buf, new_ptr - 1)))
    push!(stmts, :(_buf_pos!(buf, new_ptr)))

    return Expr(:let, Expr(:block), Expr(:block, stmts...))
end

# Conservative upper bound on the bytes a leaf value will consume on the
# wire — generous on alignment padding. Operates only on expanded leaves
# (primitives, SArrays, Strings, Arrays); the caller handles struct
# expansion upstream.
function _wa_value_bytes_expr(::Type{T}, isCDR2::Bool, LE::Bool, val_expr) where T
    T <: Union{Int8, UInt8, Bool, Char}              && return :(1)
    T <: Union{Int16, UInt16}                        && return :(3)
    T <: Union{Int32, UInt32, Float32}               && return :(7)
    T <: Union{Int64, UInt64, Float64}               && return isCDR2 ? :(11) : :(15)

    if T <: SArray
        ET = T.parameters[2]
        L  = T.parameters[4]
        if ET <: _PrimitivePacked
            pad = sizeof(ET) == 1 ? 0 :
                  ET <: Union{Int64, UInt64, Float64} ? (isCDR2 ? 3 : 7) :
                  sizeof(ET) - 1
            return :($(pad + L * sizeof(ET)))
        elseif _is_packed_type(ET)
            return :($(L * _wa_packed_bytes(ET, isCDR2)))
        else
            return _runtime_value_bytes_expr(val_expr)
        end
    end

    # Compact struct passed through as an opaque blob.
    if _is_compact_struct(T, isCDR2, LE)
        max_a = _wa_align_for(T, isCDR2)
        return :($((max_a - 1) + sizeof(T)))
    end

    if T <: AbstractArray{String}
        # A for-loop, not `sum(s -> …)`: a closure in a @generated body is
        # rejected as non-pure.
        return quote
            local _sbytes = 7
            for _s in $val_expr
                _sbytes += sizeof(_s) + 8
            end
            _sbytes
        end
    end

    if T <: AbstractArray
        ET = eltype(T)
        if ET <: _PrimitivePacked
            elt_pad = ET <: Union{Int64, UInt64, Float64} ? (isCDR2 ? 3 : 7) :
                      sizeof(ET) == 1 ? 0 : sizeof(ET) - 1
            return :(7 + $elt_pad + length($val_expr) * sizeof($ET))
        elseif _is_packed_type(ET)
            return :(7 + length($val_expr) * $(_wa_packed_bytes(ET, isCDR2)))
        else
            return :(7 + $(_runtime_value_bytes_expr(val_expr)))
        end
    end

    if T <: AbstractString
        return :(8 + sizeof($val_expr))
    end
    error("write_all!: unsupported leaf type $T")
end

# Sum of leaf bytes for a packed type (recurses into struct fields).
function _wa_packed_bytes(::Type{T}, isCDR2::Bool) where T
    leaves = Any[]
    _flatten_packed_types!(leaves, T)
    return sum(_wa_leaf_bytes_int(L, isCDR2) for L in leaves; init=0)
end

# Walk a container at runtime via the size calculator. Used when the
# per-element byte count can't be resolved at the type level (struct fields
# include Strings or Vectors). The for-loop avoids closures — closures in
# @generated bodies are rejected as "non-pure".
function _runtime_value_bytes_expr(val_expr)
    return quote
        local _total = 0
        local _calc = $(CDRSizeCalculator)()
        for _elt in $val_expr
            _calc.offset = 4
            $(addValue!)(_calc, _elt)
            _total += _calc.offset - 4
        end
        _total
    end
end

# Compile-time byte budget for a packed leaf (used when computing element
# bounds for arrays of packed structs).
function _wa_leaf_bytes_int(::Type{T}, isCDR2::Bool) where T
    T <: Union{Int8, UInt8, Bool, Char}              && return 1
    T <: Union{Int16, UInt16}                        && return 3
    T <: Union{Int32, UInt32, Float32}               && return 7
    T <: Union{Int64, UInt64, Float64}               && return isCDR2 ? 11 : 15
    if T <: SArray
        ET = T.parameters[2]
        L  = T.parameters[4]
        pad = sizeof(ET) == 1 ? 0 :
              ET <: Union{Int64, UInt64, Float64} ? (isCDR2 ? 3 : 7) :
              sizeof(ET) - 1
        return pad + L * sizeof(ET)
    end
    error("write_all!: not a packed leaf: $T")
end

# Helper for `_flatten_packed_types!` referenced above.
function _flatten_packed_types!(out_types::Vector{Any}, ::Type{T}) where T
    if _is_packed_leaf(T)
        push!(out_types, T)
        return
    end
    if isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0
        for i in 1:fieldcount(T)
            _flatten_packed_types!(out_types, fieldtype(T, i))
        end
        return
    end
    error("write_all!: cannot flatten packed type $T")
end

# Expand user structs into leaves, partition into maximal packed runs and
# singleton dynamic writes. Arrays are written length-prefixed (matches the
# reader's default). A single upfront ensureroom covers the worst-case bytes
# for the whole batch; every downstream write goes through unchecked paths.
function _wa_mixed_body(types::Vector, isCDR2::Bool, LE::Bool, value_expr::Function)
    # Schema expansion: replace each user struct with its leaf fields,
    # keeping compact structs intact for single-store codegen.
    exp_types = Any[]
    exp_exprs = Any[]
    for (i, T) in enumerate(types)
        _expand_schema!(exp_types, exp_exprs, T, value_expr(i); isCDR2=isCDR2, LE=LE)
    end

    n = length(exp_types)
    body = Expr[]

    size_terms = Any[_wa_value_bytes_expr(T, isCDR2, LE, exp_exprs[i])
                     for (i, T) in enumerate(exp_types)]
    total = length(size_terms) == 0 ? :(0) :
            length(size_terms) == 1 ? size_terms[1] :
            Expr(:call, :+, size_terms...)
    push!(body, :(_ensureroom!(c.buf, $total)))

    i = 1
    while i <= n
        T = exp_types[i]
        # Pack leaves and compact structs into a single run — both flow
        # through `_wa_packed_run_expr` with constant-offset stores. A run
        # aligns its start to its widest member; the field-walk reader instead
        # aligns the run's *first* leaf to its own alignment. Those agree only
        # when the first leaf carries the run's max alignment — so a run may
        # not extend across a leaf wider-aligned than its head (that leaf
        # starts a new run). Otherwise a run placed at a non-max-aligned offset
        # (e.g. right after a string) would over-pad its head and desync the
        # reader. At the message origin (offset 0, aligned to everything) the
        # grouping is immaterial; this only changes non-origin runs.
        if _is_packed_leaf(T) || _is_compact_struct(T, isCDR2, LE)
            run_align = _wa_align_for(T, isCDR2)
            j = i + 1
            while j <= n && (_is_packed_leaf(exp_types[j]) || _is_compact_struct(exp_types[j], isCDR2, LE)) &&
                  _wa_align_for(exp_types[j], isCDR2) <= run_align
                j += 1
            end
            push!(body, _wa_packed_run_expr(exp_types[i:j-1], exp_exprs[i:j-1], isCDR2, LE))
            i = j
        elseif T <: AbstractArray
            push!(body, :(_write_array_unchecked!(c, $(exp_exprs[i]))))
            i += 1
        elseif T <: AbstractString
            push!(body, :(_write_string_unchecked!(c, $(exp_exprs[i]))))
            i += 1
        else
            push!(body, :(write(c, $(exp_exprs[i]))))
            i += 1
        end
    end
    push!(body, :(return nothing))
    return Expr(:block, body...)
end

@generated function write_all!(c::CDRWriter{IsCDR2, LE},
                                vs::Vararg{Any, K}) where {IsCDR2, LE, K}
    K == 0 && return :(return nothing)
    types = collect(vs)
    return _wa_mixed_body(types, IsCDR2, LE, i -> :(vs[$i]))
end

@generated function write_all!(c::CDRWriter{IsCDR2, LE}, ::Type{Schema},
                                vs::Vararg{Any, K}) where {IsCDR2, LE, Schema <: Tuple, K}
    decl = collect(Schema.parameters)
    if length(decl) != K
        return :(throw(ArgumentError(string("write_all!: schema has ",
                                            $(length(decl)),
                                            " slots but received ",
                                            $K, " values"))))
    end
    K == 0 && return :(return nothing)
    return _wa_mixed_body(decl, IsCDR2, LE, i -> :(vs[$i]::$(decl[i])))
end