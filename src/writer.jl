# Both `IsCDR2` and `LE` (little-endian) are hoisted into the type so per-kind
# and per-endian dispatch resolves at compile time — no field loads, no
# branches in the inner write loop.
mutable struct CDRWriter{IsCDR2, LE}
    buf::IOBuffer

    usesDelimiterHeader::Bool
    usesMemberHeader::Bool

    origin::Int
    kind::EncapsulationKind

    function CDRWriter(buf::IOBuffer = IOBuffer(), kind::EncapsulationKind=CDR_LE)
        isCDR2, littleEndian, usesDelimiterHeader, usesMemberHeader = getEncapsulationKind(UInt8(kind))
        write(buf, UInt8(0))
        write(buf, UInt8(kind))
        write(buf, UInt16(0))
        new{isCDR2, littleEndian}(buf, usesDelimiterHeader, usesMemberHeader, 4, kind)
    end
end

@inline isCDR2(::CDRWriter{B}) where B = B
@inline littleEndian(::CDRWriter{<:Any, L}) where L = L

# Endianness branch resolved at compile time via the LE type parameter.
# Single-byte values pass through unchanged regardless of endianness.
@inline _maybe_swap(::CDRWriter{<:Any, true},  v) = v
@inline _maybe_swap(::CDRWriter{<:Any, false}, v) = sizeof(v) == 1 ? v : ntoh(v)

function resetOrigin(c::CDRWriter)
    c.origin = position(c.buf)
end

# Internal: emit `padding` zero bytes at the buffer's current write head.
# Keeping the byte-loop here (rather than the wider zero store used in the
# packed `write_all!` path) because callers of this checked variant don't
# advertise their max alignment to us, so we can't safely write past
# `padding` bytes.
@inline function _emit_padding!(buf::IOBuffer, padding)
    Base.ensureroom(buf, padding)
    data = buf.data
    base = buf.ptr
    GC.@preserve data begin
        for i in 0:padding-1
            unsafe_store!(pointer(data, base + i), UInt8(0))
        end
    end
    new_ptr = base + padding
    if new_ptr - 1 > buf.size
        buf.size = new_ptr - 1
    end
    buf.ptr = new_ptr
    return
end

# All CDR alignment sizes are powers of two (1/2/4/8), so we use a bitmask
# instead of `% size` (which would compile to an idiv when `size` isn't a
# compile-time constant). `@inline` lets LLVM constant-fold the mask at any
# call site that passes a literal or a constant-propagated `sizeof(T)`.
@inline function align(c::CDRWriter, size::Int)
    buf = c.buf
    alignment = (buf.ptr - 1 - c.origin) & (size - 1)
    alignment == 0 && return
    _emit_padding!(buf, size - alignment)
end

# 8-byte alignment is 4 on CDR2, 8 on CDR1. With `IsCDR2` lifted into the
# type, each call site dispatches to the right specialization at compile
# time — no runtime branch, no field load.
@inline align8(c::CDRWriter{true})  = align(c, 4)
@inline align8(c::CDRWriter{false}) = align(c, 8)

# Write a primitive value directly into the buffer's backing memory.
#
# Going through Base.unsafe_write here adds ~30 instructions per call
# (writable/reinit/append flag checks, room math, byte-copy loop), which
# dominates for small types. We do the room check ourselves, then a single
# typed `unsafe_store!` — measured at ~4× faster on Float64.
#
# Assumes the buffer was constructed with the writer's defaults
# (append=false, writable, seekable) — i.e. created by CDRWriter.
@inline function _write_prim(buf::IOBuffer, v::T) where T
    n = sizeof(T)
    Base.ensureroom(buf, n)
    data = buf.data
    ptr = buf.ptr
    GC.@preserve data unsafe_store!(Ptr{T}(pointer(data, ptr)), v)
    new_ptr = ptr + n
    if new_ptr - 1 > buf.size
        buf.size = new_ptr - 1
    end
    buf.ptr = new_ptr
    return n
end

function Base.write(c::CDRWriter, v::Union{Int8, UInt8, Bool})
    align(c, 1)
    _write_prim(c.buf, v)
end

# Julia specializes parametric methods per concrete `T`, so `sizeof(T)` is a
# compile-time constant inside each specialization and `align` (which is
# `@inline`) folds the mask down to a single AND.
function Base.write(c::CDRWriter, v::T) where T <: Union{Int16, UInt16, Int32, UInt32, Float32}
    align(c, sizeof(T))
    _write_prim(c.buf, _maybe_swap(c, v))
end

Base.write(c::CDRWriter, v::Char) = write(c, UInt8(v))

presentFlag(::CDRWriter{false}, ::Bool) = throw("presentFlag is only valid for CDR2 streams")
presentFlag(c::CDRWriter{true}, value::Bool) = write(c, UInt8(value ? 1 : 0))

# Big-endian helpers: one method per width-class. UInt16/UInt32 share an
# alignment-from-sizeof method; UInt64 needs align8 because the alignment
# depends on the encapsulation kind.
function uintBE(c::CDRWriter, v::T) where T <: Union{UInt16, UInt32}
    align(c, sizeof(T))
    _write_prim(c.buf, hton(v))
end
function uintBE(c::CDRWriter, v::UInt64)
    align8(c)
    _write_prim(c.buf, hton(v))
end

# Width-specific names kept for backward compatibility with existing callers.
uint16BE(c::CDRWriter, v::UInt16) = uintBE(c, v)
uint32BE(c::CDRWriter, v::UInt32) = uintBE(c, v)
uint64BE(c::CDRWriter, v::UInt64) = uintBE(c, v)

Base.position(c::CDRWriter) = position(c.buf)
data(c::CDRWriter) = view(c.buf.data, 1:position(c.buf))

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
        throw("Object size $objectSize is too large; max value is $(0xffffffff)")
    end
    return 4
end

function memberHeaderV2(c::CDRWriter, mustUnderstand::Bool, id::Int, objectSize::Int, lengthCode::Union{Nothing, Int})
    if id > 0x0fffffff
        throw("Member ID $id is too large; max value is $(0x0fffffff)")
    end
    # EMHEADER = (M_FLAG<<31) + (LC<<28) + M.id
    # M is the member of a structure
    # M_FLAG is the value of the Must Understand option for the member
    mustUnderstandFlag = mustUnderstand ? 1 << 31 : 0
    # LC is the value of the Length Code for the member.
    finalLengthCode = !isnothing(lengthCode) ? lengthCode : getLengthCodeForObjectSize(objectSize)

    header = mustUnderstandFlag | (finalLengthCode << 28) | id

    write(c, UInt32(header))

    if finalLengthCode == 0 || finalLengthCode == 1 ||
        finalLengthCode == 2 || finalLengthCode == 3
        shouldBeSize = lengthCodeToObjectSize(finalLengthCode)
        if objectSize != shouldBeSize
            throw("Cannot write a length code $(finalLengthCode) header with an object size not equal to $(shouldBeSize)")
        end
    elseif finalLengthCode == 4 || finalLengthCode == 5
        return write(c, UInt32(objectSize))
    elseif finalLengthCode == 6
        if objectSize % 4 !== 0
            throw("Cannot write a length code 6 header with an object size that is not a multiple of 4")
        end
        return write(c, UInt32(objectSize >> 2))
    elseif finalLengthCode == 7
        if objectSize % 8 !== 0
            throw("Cannot write a length code 7 header with an object size that is not a multiple of 4")
        end
        return write(c, UInt32(objectSize >> 3))
    else 
        throw("Unexpected length code $finalLengthCode")
    end
end

sequenceLength(w::CDRWriter, len) = write(w, UInt32(len))

# Internal: bulk memcpy from src into the buffer at the current write head.
@inline function _bulk_copy_into!(buf::IOBuffer, src::Ptr{UInt8}, nb::Int)
    Base.ensureroom(buf, nb)
    data = buf.data
    ptr = buf.ptr
    GC.@preserve data Base.unsafe_copyto!(pointer(data, ptr), src, nb)
    new_ptr = ptr + nb
    if new_ptr - 1 > buf.size
        buf.size = new_ptr - 1
    end
    buf.ptr = new_ptr
    return
end

# LE writer: always bulk copy regardless of element width.
@inline function _writeArrayBody!(w::CDRWriter{<:Any, true}, a::AbstractArray{T}) where T
    GC.@preserve a _bulk_copy_into!(w.buf, Ptr{UInt8}(pointer(a)), length(a) * sizeof(T))
    return nothing
end

# BE writer: single-byte stays bulk; multi-byte goes through ntoh per element.
# `sizeof(T) == 1` is a compile-time constant in each specialization, so only
# one branch survives codegen.
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

# SArray equivalents: take a Ref to get a stable pointer to the immutable.
@inline function _writeStaticArrayBody!(w::CDRWriter{<:Any, true}, a::SArray{S, T, N, L}) where {S, T, N, L}
    ref = Ref(a)
    GC.@preserve ref _bulk_copy_into!(w.buf, Ptr{UInt8}(pointer_from_objref(ref)), L * sizeof(T))
    return nothing
end

@inline function _writeStaticArrayBody!(w::CDRWriter{<:Any, false}, a::SArray{S, T, N, L}) where {S, T, N, L}
    if sizeof(T) == 1
        ref = Ref(a)
        GC.@preserve ref _bulk_copy_into!(w.buf, Ptr{UInt8}(pointer_from_objref(ref)), L)
    else
        for v in a
            _write_prim(w.buf, ntoh(v))
        end
    end
    return nothing
end

# 1/2/4-byte element types: alignment derives from `sizeof(T)`, which is a
# constant in each parametric specialization and folds inside `align`.
function Base.write(w::CDRWriter, a::AbstractArray{T}, writeLength=false) where T <: Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32}
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

# 8-byte element types: alignment depends on encapsulation kind (CDR2→4, CDR1→8).
function Base.write(w::CDRWriter, a::AbstractArray{T}, writeLength=false) where T <: Union{Int64, UInt64, Float64}
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

function Base.write(w::CDRWriter, a::A, writeLength=false) where A<:AbstractArray{String}
    if writeLength
        sequenceLength(w, length(a))
    end
    for s in a
        write(w, s)
    end
end

# ---------------------------------------------------------------------------
# Multi-element write: batched ensureroom + unchecked inner writes.
#
# `write(w, v1, v2, vs...)` ensures the buffer has room for the worst-case
# byte count of all arguments in one shot, then writes each value without
# the per-call `Base.ensureroom` overhead. The work is unrolled at compile
# time via `@generated`, so each value lands on its concrete-typed write.
# ---------------------------------------------------------------------------

# Worst-case byte budget for writing a single value of type T:
#   primitives:  (sizeof(T) - 1) max padding + sizeof(T) value
#   8-byte ints: 7 max padding (CDR1) + 8 value = 15
#   SArrays:     worst alignment for T + L*sizeof(T)
@inline _worst_case_bytes(::Type{T}) where T <: Union{Int8, UInt8, Bool, Char} = 1
@inline _worst_case_bytes(::Type{T}) where T <: Union{Int16, UInt16, Int32, UInt32, Float32} = 2 * sizeof(T) - 1
@inline _worst_case_bytes(::Type{T}) where T <: Union{Int64, UInt64, Float64} = 15
@inline function _worst_case_bytes(::Type{SA}) where {S, T, N, L, SA<:SArray{S, T, N, L}}
    pad = sizeof(T) == 1 ? 0 :
          T <: Union{Int64, UInt64, Float64} ? 7 :
          sizeof(T) - 1
    return pad + L * sizeof(T)
end

# Types accepted by the multi-element / bulk write API. (Strings and
# Vector{T} are excluded — they have runtime-dependent lengths.)
const _BulkWritable = Union{Int8, UInt8, Bool, Char,
                            Int16, UInt16, Int32, UInt32, Float32,
                            Int64, UInt64, Float64, SArray}


"""
    write_all!(c::CDRWriter, vs...)
    write_all!(c::CDRWriter, ::Type{Tuple{T1, T2, …}}, vs...)

Write multiple values to `c` with the full CDR layout (offsets and padding)
computed at compile time.

The implementation builds the destination layout from the argument types,
issues one `ensureroom` covering the worst case, aligns the buffer once to
the strongest alignment in the workload, then emits direct `unsafe_store!`s
at each value's pre-computed offset inside a single `GC.@preserve` block.
This eliminates the per-value `align` / `ensureroom` overhead that a
sequence of `write(c, v)` calls would pay, and gives LLVM contiguous typed
stores at constant offsets — which it tends to coalesce.

Accepted element types are primitives (Int8…Float64, Bool, Char) and
`StaticArrays.SArray` of those. Strings and `Vector{T}` are not accepted
because their byte budget can't be computed at the type level.

The schema overload takes the type list as a `Tuple{…}` type and asserts
each value against its declared slot.

# Examples
```julia
write_all!(c, UInt8(1), Int16(2), Float64(3.14))
write_all!(c, Tuple{UInt8, Int16, Float64}, 1, 2, 3.14)
```
"""
function write_all! end

# Layout helpers used at generation time. NOT for runtime dispatch.
function _wa_align_for(::Type{T}, isCDR2::Bool) where T
    T <: Union{Int8, UInt8, Bool, Char}            && return 1
    T <: Union{Int16, UInt16}                      && return 2
    T <: Union{Int32, UInt32, Float32}             && return 4
    T <: Union{Int64, UInt64, Float64}             && return isCDR2 ? 4 : 8
    if T <: SArray
        return _wa_align_for(T.parameters[2], isCDR2)
    end
    error("write_all!: unsupported type $T")
end

function _wa_size_for(::Type{T}) where T
    T <: Union{Int8, UInt8, Bool, Char} && return 1
    T <: Union{Int16, UInt16, Int32, UInt32, Float32,
               Int64, UInt64, Float64}  && return sizeof(T)
    if T <: SArray
        return T.parameters[4] * sizeof(T.parameters[2])
    end
    error("write_all!: unsupported type $T")
end

# Build the body of a packed write: ensureroom, pre-align, then direct
# typed stores at compile-time-known offsets.
function _wa_build_body(types::Vector, isCDR2::Bool, value_expr::Function)
    isempty(types) && return :(return nothing)

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

    body = Expr[]
    # One ensureroom that covers worst-case pre-align padding + payload.
    push!(body, :(Base.ensureroom(c.buf, $(total_size + max_align - 1))))
    push!(body, :(buf = c.buf))
    push!(body, :(data = buf.data))
    # Hoist the Memory's inner data pointer ONCE. LLVM otherwise reloads
    # `data.ptr` before every typed store because TBAA can't prove our
    # stores don't alias the Memory's metadata. Caching it as a raw Ptr in a
    # local lets LLVM hold it in a register across the entire store block.
    push!(body, :(base_ptr = pointer(data)))

    # Inline alignment: a single `max_align`-sized typed zero store covers
    # up to `max_align - 1` bytes of padding in one instruction. The bytes
    # beyond actual padding are at the start of the data region and get
    # overwritten by subsequent value stores, so a wider zero write is safe
    # as long as ensureroom gave us at least `max_align` bytes (which it
    # did: `total_size + max_align - 1 >= max_align` when `total_size >= 1`).
    # `-offset & (max_align - 1)` is the branchless padding count.
    if max_align == 1
        push!(body, :(local_ptr = buf.ptr))
    else
        zero_t = max_align == 2 ? UInt16 :
                 max_align == 4 ? UInt32 :
                                  UInt64
        push!(body, quote
            _old_ptr = buf.ptr
            _pad = (-((_old_ptr - 1 - c.origin) & $(max_align - 1))) & $(max_align - 1)
            GC.@preserve data unsafe_store!(Ptr{$zero_t}(base_ptr + (_old_ptr - 1)),
                                            $zero_t(0))
            local_ptr = _old_ptr + _pad
        end)
    end

    # All store addresses are derived from `base_ptr` + a fully constant
    # offset (`local_ptr - 1 + off`), so LLVM sees them as offsets off a
    # single register-held base.
    ref_setup = Expr[]
    preserve_syms = Symbol[:data]
    store_exprs = Expr[:(_base = base_ptr + (local_ptr - 1))]

    for (i, T) in enumerate(types)
        off = offsets[i]
        v = value_expr(i)
        if T <: SArray
            refsym = Symbol("ref", i)
            push!(ref_setup, :($refsym = Ref($v)))
            push!(preserve_syms, refsym)
            push!(store_exprs,
                  :(Base.unsafe_copyto!(_base + $off,
                                        Ptr{UInt8}(pointer_from_objref($refsym)),
                                        $(_wa_size_for(T)))))
        elseif T <: Union{Int8, UInt8, Bool}
            push!(store_exprs,
                  :(unsafe_store!(Ptr{$T}(_base + $off), $v)))
        elseif T === Char
            push!(store_exprs,
                  :(unsafe_store!(Ptr{UInt8}(_base + $off), UInt8($v))))
        else
            push!(store_exprs,
                  :(unsafe_store!(Ptr{$T}(_base + $off), _maybe_swap(c, $v))))
        end
    end

    append!(body, ref_setup)
    push!(body, Expr(:macrocall, GlobalRef(Base.GC, Symbol("@preserve")),
                     LineNumberNode(0, :write_all!),
                     preserve_syms..., Expr(:block, store_exprs...)))

    push!(body, :(new_ptr = local_ptr + $total_size))
    push!(body, :(new_ptr - 1 > buf.size && (buf.size = new_ptr - 1)))
    push!(body, :(buf.ptr = new_ptr))
    push!(body, :(return nothing))
    return Expr(:block, body...)
end

@generated function write_all!(c::CDRWriter{IsCDR2, LE},
                                vs::Vararg{_BulkWritable, K}) where {IsCDR2, LE, K}
    types = collect(vs)
    return _wa_build_body(types, IsCDR2, i -> :(vs[$i]))
end

@generated function write_all!(c::CDRWriter{IsCDR2, LE}, ::Type{Schema},
                                vs::Vararg{_BulkWritable, K}) where {IsCDR2, LE, Schema <: Tuple, K}
    decl = collect(Schema.parameters)
    if length(decl) != K
        return :(throw(ArgumentError(string("write_all!: schema has ",
                                            $(length(decl)),
                                            " slots but received ",
                                            $K, " values"))))
    end
    return _wa_build_body(decl, IsCDR2, i -> :(vs[$i]::$(decl[i])))
end