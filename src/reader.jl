@enum EncapsulationKind begin
    # Both RTPS and ENC_HEADER enum values
    # Plain CDR, big-endian
    CDR_BE = 0x00
    # Plain CDR, little-endian
    CDR_LE = 0x01
    # Parameter List CDR, big-endian
    PL_CDR_BE = 0x02
    # Parameter List CDR, little-endian
    PL_CDR_LE = 0x03
    
    # ENC_HEADER enum values
    # Plain CDR2, big-endian
    CDR2_BE = 0x10
    # Plain CDR2, little-endian
    CDR2_LE = 0x11
    # Parameter List CDR2, big-endian
    PL_CDR2_BE = 0x12
    # Parameter List CDR2, little-endian
    PL_CDR2_LE = 0x13
    # Delimited CDR, big-endian
    DELIMITED_CDR2_BE = 0x14
    # Delimited CDR, little-endian
    DELIMITED_CDR2_LE = 0x15
    
    # RTPS enum values
    # Plain CDR2, big-endian
    RTPS_CDR2_BE = 0x06
    # Plain CDR2, little-endian
    RTPS_CDR2_LE = 0x07
    # Delimited CDR, big-endian
    RTPS_DELIMITED_CDR2_BE = 0x08
    # Delimited CDR, little-endian
    RTPS_DELIMITED_CDR2_LE = 0x09
    # Parameter List CDR2, big-endian
    RTPS_PL_CDR2_BE = 0x0a
    # Parameter List CDR2, little-endian
    RTPS_PL_CDR2_LE = 0x0b
end

# Both `IsCDR2` and `LE` (little-endian) are hoisted into the type so per-kind
# and per-endian dispatch resolves at compile time — no field loads, no
# branches in the inner read/write loop.
mutable struct CDRReader{B <: IO, IsCDR2, LE}
    src::B

    usesDelimiterHeader::Bool
    usesMemberHeader::Bool

    origin::Int

    kind::EncapsulationKind

    function CDRReader(buf::A) where A <: IO
        preamble = ntoh(read(buf, UInt32))
        kind = UInt8((preamble >> 16) & 0xFF)
        isCDR2, littleEndian, usesDelimiterHeader, usesMemberHeader = getEncapsulationKind(kind)
        new{A, isCDR2, littleEndian}(buf, usesDelimiterHeader, usesMemberHeader, 4, EncapsulationKind(kind))
    end

    function CDRReader{A, IsCDR2, LE}(src::A,
                                      usesDelimiterHeader::Bool, usesMemberHeader::Bool, origin::Int,
                                      kind::EncapsulationKind) where {A <: IO, IsCDR2, LE}
        new{A, IsCDR2, LE}(src, usesDelimiterHeader, usesMemberHeader, origin, kind)
    end
end

@inline isCDR2(::CDRReader{<:Any, B}) where B = B
@inline littleEndian(::CDRReader{<:Any, <:Any, L}) where L = L

# `r.littleEndian` is no longer a field — provide a method so existing
# external callers continue to compile. `_maybe_swap` is the fast path used
# internally; it dispatches on the type parameter, no branch at runtime.
@inline _maybe_swap(::CDRReader{<:Any, <:Any, true},  v) = v
@inline _maybe_swap(::CDRReader{<:Any, <:Any, false}, v) = sizeof(v) == 1 ? v : ntoh(v)

Base.eof(r::CDRReader) = eof(r.src)
Base.position(r::CDRReader) = position(r.src)
Base.seek(r::CDRReader, abs::Integer) = (seek(r.src, abs); r)
Base.skip(r::CDRReader, n::Integer) = (skip(r.src, n); r)
isAtEnd(r::CDRReader) = eof(r.src)
decodedBytes(r::CDRReader) = position(r.src) - 4
byteLength(r::CDRReader) = position(r.src) + bytesavailable(r.src)

function limit!(r::CDRReader, length::Integer)
    r.src isa IOBuffer || throw("limit! only supported for IOBuffer-backed readers")
    r.src.size = position(r.src) + Int(length)
    return r
end

function Base.copy(r::CDRReader{<:Any, IsCDR2, LE}) where {IsCDR2, LE}
    r.src isa IOBuffer || throw("copy only supported for IOBuffer-backed readers")
    newio = IOBuffer(r.src.data; read=true, write=false)
    newio.size = r.src.size
    seek(newio, position(r.src))
    return CDRReader{typeof(newio), IsCDR2, LE}(newio,
                                     r.usesDelimiterHeader, r.usesMemberHeader, r.origin, r.kind)
end

isPresentFlag(::CDRReader{<:Any, false}) = throw("isPresentFlag is only valid for CDR2 streams")
isPresentFlag(r::CDRReader{<:Any, true}) = read(r, UInt8) != 0

@inline function align(r::CDRReader, size::Int)
    src = r.src
    # All CDR alignment sizes are powers of two; `& (size-1)` avoids the
    # `idiv` that `% size` produces with a non-constant size.
    alignment = (position(src) - r.origin) & (size - 1)
    if alignment > 0
        skip(src, size - alignment)
    end
end

# 8-byte alignment is 4 on CDR2, 8 on CDR1. Dispatching on the type param
# resolves at compile time — no field load, no branch.
@inline align8(r::CDRReader{<:Any, true})  = align(r, 4)
@inline align8(r::CDRReader{<:Any, false}) = align(r, 8)

# Big-endian reads: one parametric method for UInt16/UInt32, one for UInt64.
function uintBE(r::CDRReader, ::Type{T}) where T <: Union{UInt16, UInt32}
    align(r, sizeof(T))
    return ntoh(_read_prim(r.src, T))
end
function uintBE(r::CDRReader, ::Type{UInt64})
    align8(r)
    return ntoh(_read_prim(r.src, UInt64))
end

uint16BE(r::CDRReader) = uintBE(r, UInt16)
uint32BE(r::CDRReader) = uintBE(r, UInt32)
uint64BE(r::CDRReader) = uintBE(r, UInt64)

function getEncapsulationKind(kind::UInt8)
    isCDR2 = kind > UInt8(PL_CDR_LE)
    littleEndian = 
        kind === UInt8(CDR_LE) ||
        kind === UInt8(PL_CDR_LE) ||
        kind === UInt8(CDR2_LE) ||
        kind === UInt8(PL_CDR2_LE) ||
        kind === UInt8(DELIMITED_CDR2_LE) ||
        kind === UInt8(RTPS_CDR2_LE) ||
        kind === UInt8(RTPS_PL_CDR2_LE) ||
        kind === UInt8(RTPS_DELIMITED_CDR2_LE)
        
    isDelimitedCDR2 =
        kind === UInt8(DELIMITED_CDR2_BE) ||
        kind === UInt8(DELIMITED_CDR2_LE) ||
        kind === UInt8(RTPS_DELIMITED_CDR2_BE) ||
        kind === UInt8(RTPS_DELIMITED_CDR2_LE)

    isPLCDR2 =
        kind === UInt8(PL_CDR2_BE) ||
        kind === UInt8(PL_CDR2_LE) ||
        kind === UInt8(RTPS_PL_CDR2_BE) ||
        kind === UInt8(RTPS_PL_CDR2_LE)

    isPLCDR1 = kind === UInt8(PL_CDR_BE) || kind === UInt8(PL_CDR_LE)
    
    usesDelimiterHeader = isDelimitedCDR2 || isPLCDR2
    usesMemberHeader = isPLCDR2 || isPLCDR1

    return isCDR2, littleEndian, usesDelimiterHeader, usesMemberHeader
end

# Read a primitive value directly out of the buffer's backing memory. Mirrors
# `_write_prim` on the writer side — one typed `unsafe_load!` instead of
# `Base.read(::IOBuffer, ::Type{T})`'s peek+skip+Ref dance.
@inline function _read_prim(buf::IOBuffer, ::Type{T}) where T
    n = sizeof(T)
    data = buf.data
    ptr = buf.ptr
    v = GC.@preserve data unsafe_load(Ptr{T}(pointer(data, ptr)))
    buf.ptr = ptr + n
    return v
end
# Generic IO fallback for non-IOBuffer-backed readers.
@inline _read_prim(src::IO, ::Type{T}) where T = read(src, T)

function Base.read(r::CDRReader, ::Type{T}) where T <: Union{Int8, UInt8, Bool}
    align(r, sizeof(T))
    return _read_prim(r.src, T)
end

function Base.read(r::CDRReader, ::Type{Char})
    return Char(read(r, UInt8))
end

function Base.read(r::CDRReader, ::Type{T}) where T <: Union{Int16, UInt16, Int32, UInt32, Float32}
    align(r, sizeof(T))
    return _maybe_swap(r, _read_prim(r.src, T))
end

function Base.read(r::CDRReader, ::Type{T}) where T <: Union{Int64, UInt64, Float64}
    align8(r)
    return _maybe_swap(r, _read_prim(r.src, T))
end

Base.read(r::CDRReader, ::Type{String}) = read(r, String, read(r, UInt32))
function Base.read(r::CDRReader, ::Type{String}, len::Integer)
    if len <= 1
        skip(r.src, len)
        return ""
    end
    bytes = read(r.src, len)
    if bytes[end] == 0x00
        bytes = @view bytes[1:end-1]
    end
    return String(bytes)
end

dHeader(r::CDRReader) = read(r, UInt32)

emHeader(r::CDRReader{<:Any, true})  = memberHeaderV2(r)
emHeader(r::CDRReader{<:Any, false}) = memberHeaderV1(r)

resetOrigin(r::CDRReader) = r.origin = position(r.src)

const EXTENDED_PID = 0x3f01
const SENTINEL_PID = 0x3f02

function lengthCodeToObjectSize(lengthCode)
    if lengthCode == 0
        return 1
    elseif lengthCode == 1
        return 2
    elseif lengthCode == 2
        return 4
    elseif lengthCode == 3
        return 8
    end
    throw("Invalid length code $lengthCode")
end
function memberHeaderV1(r::CDRReader)
    align(r, 4)
    idHeader = read(r, UInt16)
    mustUnderstandFlag = (idHeader & 0x4000) >> 14 == 1
    # indicates that the parameter has a implementation-specific interpretation
    implementationSpecificFlag = (idHeader & 0x8000) >> 15 == 1

    # Allows the specification of large member ID and/or data length values
    # requires the reading in of two uint32's for ID and size
    extendedPIDFlag = (idHeader & 0x3fff) === EXTENDED_PID

    # Indicates the end of the parameter list structure
    sentinelPIDFlag = (idHeader & 0x3fff) === SENTINEL_PID
    if sentinelPIDFlag
      # Return that we have read the sentinel header when we expected to read an emHeader.
      # This can happen for absent optional members at the end of a struct.
      return (id=UInt32(SENTINEL_PID), objectSize=UInt32(0), mustUnderstand=false,
              implementationSpecific=false, ignore=false,
              readSentinelHeader=true, lengthCode=UInt32(0))
    end

    # Indicates that the parameter ID should be ignored by the consumer,
    # but the size field still needs to be consumed so we can skip the value.
    ignorePIDFlag = (idHeader & 0x3fff) == 0x3f03

    if (extendedPIDFlag)
      # Need to consume last part of header (is just an 8 in this case)
      # Alignment could take care of this, but I want to be explicit
      read(r, UInt16)
    end

    id = extendedPIDFlag ? read(r, UInt32) : UInt32(idHeader & 0x3fff)
    objectSize = extendedPIDFlag ? read(r, UInt32) : UInt32(read(r, UInt16))
    resetOrigin(r)
    return (; id, objectSize, mustUnderstand=mustUnderstandFlag,
            implementationSpecific=implementationSpecificFlag, ignore=ignorePIDFlag,
            readSentinelHeader=false, lengthCode=UInt32(0))
end

sentinelHeader(::CDRReader{<:Any, true}) = nothing
function sentinelHeader(r::CDRReader{<:Any, false})
    align(r, 4)
    header = read(r, UInt16)
    sentinelPIDFlag = (header & 0x3fff) === SENTINEL_PID
    if !sentinelPIDFlag
        throw("Expected SENTINEL_PID flag but got $header")
    end
    read(r, UInt16)
end

function memberHeaderV2(r::CDRReader)
    header = read(r, UInt32)
    mustUnderstand = (header & 0x80000000) >> 31 == 1
    # LC is the value of the Length Code for the member.
    lengthCode = (header & 0x70000000) >> 28
    id = header & 0x0fffffff

    objectSize = emHeaderObjectSize(r, lengthCode);

    return (; id, objectSize, mustUnderstand,
            implementationSpecific=false, ignore=false,
            readSentinelHeader=false, lengthCode)
end

function emHeaderObjectSize(r::CDRReader, lengthCode)
    if lengthCode == 0 || lengthCode == 1 ||
        lengthCode == 2 || lengthCode == 3
        return UInt32(lengthCodeToObjectSize(lengthCode))
    elseif lengthCode == 4 || lengthCode == 5
        return read(r, UInt32)
    elseif lengthCode == 6
        return read(r, UInt32) * UInt32(4)
    elseif lengthCode == 7
        return read(r, UInt32) * UInt32(8)
    else
        throw("Invalid length code $lengthCode in EMHEADER at offset $(position(r.src) - 4)")
    end
end

sequenceLength(r::CDRReader) = read(r, UInt32)

# Element-bulk read body — alignment is the caller's responsibility.
# Split by LE so the byte-swap branch goes away at compile time. For BE the
# `sizeof(T) == 1` check is also a compile-time constant per specialization.
@inline function _readArrayBody!(r::CDRReader{<:Any, <:Any, true}, ::Type{T}, count) where T
    array = Vector{T}(undef, count)
    GC.@preserve array unsafe_read(r.src, Ptr{UInt8}(pointer(array)), count * sizeof(T))
    return array
end

@inline function _readArrayBody!(r::CDRReader{<:Any, <:Any, false}, ::Type{T}, count) where T
    array = Vector{T}(undef, count)
    GC.@preserve array unsafe_read(r.src, Ptr{UInt8}(pointer(array)), count * sizeof(T))
    if sizeof(T) > 1
        array .= ntoh.(array)
    end
    return array
end

# 1/2/4-byte element type — alignment from constant `sizeof(T)`.
function Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where {T <:Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32}, A<:AbstractArray{T}}
    num == 0 && return T[]
    align(r, sizeof(T))
    return _readArrayBody!(r, T, num)
end

# 8-byte element type — alignment via align8.
function Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where {T <:Union{Int64, UInt64, Float64}, A<:AbstractArray{T}}
    num == 0 && return T[]
    align8(r)
    return _readArrayBody!(r, T, num)
end

Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where A<:AbstractArray{String} = [read(r, String) for i=1:num]

@inline function _readStaticArrayBody(r::CDRReader{<:Any, <:Any, true}, ::Type{SA}) where {S, T, N, L, SA<:SArray{S, T, N, L}}
    ref = Ref{NTuple{L, T}}()
    GC.@preserve ref unsafe_read(r.src, Ptr{UInt8}(pointer_from_objref(ref)), L * sizeof(T))
    return SA(ref[])
end

@inline function _readStaticArrayBody(r::CDRReader{<:Any, <:Any, false}, ::Type{SA}) where {S, T, N, L, SA<:SArray{S, T, N, L}}
    ref = Ref{NTuple{L, T}}()
    GC.@preserve ref unsafe_read(r.src, Ptr{UInt8}(pointer_from_objref(ref)), L * sizeof(T))
    if sizeof(T) > 1
        return SA(ntoh.(ref[]))
    end
    return SA(ref[])
end

function Base.read(r::CDRReader, ::Type{SA}) where {T<:Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32}, S, N, L, SA<:SArray{S, T, N, L}}
    L == 0 && return SA(ntuple(_ -> zero(T), Val(L)))
    align(r, sizeof(T))
    return _readStaticArrayBody(r, SA)
end

function Base.read(r::CDRReader, ::Type{SA}) where {T<:Union{Int64, UInt64, Float64}, S, N, L, SA<:SArray{S, T, N, L}}
    L == 0 && return SA(ntuple(_ -> zero(T), Val(L)))
    align8(r)
    return _readStaticArrayBody(r, SA)
end

# ---------------------------------------------------------------------------
# Multi-element read: layout-aware bulk reader, mirror of `write_all!`.
#
# `read_all!(r, Tuple{T1, T2, …})` computes each field's byte offset at
# compile time, pre-aligns the buffer once to the strongest alignment in the
# schema, and emits direct `unsafe_load!`s inside one `GC.@preserve` block.
# Returns a `Tuple{T1, T2, …}` of values.
#
# Restricted to IOBuffer-backed readers because we pull values straight off
# `buf.data` — other IO types go through the per-value `read` path.
# ---------------------------------------------------------------------------

# Endian-swap an NTuple element-wise (used for SArray loads on BE streams).
@inline _maybe_swap_each(::CDRReader{<:Any, <:Any, true}, nt::NTuple) = nt
@inline _maybe_swap_each(::CDRReader{<:Any, <:Any, false}, nt::NTuple{L, T}) where {L, T} =
    sizeof(T) == 1 ? nt : map(ntoh, nt)

function _ra_build_body(types::Vector, isCDR2::Bool)
    isempty(types) && return :(return ())

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
    push!(body, :(align(r, $max_align)))
    push!(body, :(src = r.src))
    push!(body, :(local_ptr = src.ptr))
    push!(body, :(data = src.data))

    value_syms = Symbol[]
    load_exprs = Expr[]

    for (i, T) in enumerate(types)
        off = offsets[i]
        vsym = Symbol("v", i)
        push!(value_syms, vsym)

        if T <: SArray
            ET = T.parameters[2]
            L  = T.parameters[4]
            NT = :(NTuple{$L, $ET})
            push!(load_exprs,
                  :($vsym = $T(_maybe_swap_each(r,
                      unsafe_load(Ptr{$NT}(pointer(data, local_ptr + $off)))))))
        elseif T <: Union{Int8, UInt8, Bool}
            push!(load_exprs,
                  :($vsym = unsafe_load(Ptr{$T}(pointer(data, local_ptr + $off)))))
        elseif T === Char
            push!(load_exprs,
                  :($vsym = Char(unsafe_load(Ptr{UInt8}(pointer(data, local_ptr + $off))))))
        else
            push!(load_exprs,
                  :($vsym = _maybe_swap(r,
                      unsafe_load(Ptr{$T}(pointer(data, local_ptr + $off))))))
        end
    end

    push!(body, Expr(:macrocall, GlobalRef(Base.GC, Symbol("@preserve")),
                     LineNumberNode(0, :read_all!),
                     :data, Expr(:block, load_exprs...)))

    push!(body, :(src.ptr = local_ptr + $total_size))
    push!(body, Expr(:return, Expr(:tuple, value_syms...)))
    return Expr(:block, body...)
end

"""
    read_all!(r::CDRReader, ::Type{Tuple{T1, T2, …}}) -> Tuple{T1, T2, …}

Read multiple values from `r` according to the schema declared as a `Tuple{…}`
type, returning them as a Tuple. The schema determines offsets, alignment,
and the value types pulled from the buffer. Accepted element types match
those of [`write_all!`](@ref): primitives (Int8…Float64, Bool, Char) and
`StaticArrays.SArray` of those.

The work is unrolled at compile time, with a single pre-alignment and direct
`unsafe_load!`s at constant offsets inside one `GC.@preserve` block. Use this
to balance a `write_all!(c, …)` on the encode side.
"""
@generated function read_all!(r::CDRReader{IOBuffer, IsCDR2, LE},
                              ::Type{Schema}) where {IsCDR2, LE, Schema <: Tuple}
    decl = collect(Schema.parameters)
    return _ra_build_body(decl, IsCDR2)
end