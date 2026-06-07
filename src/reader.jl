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

# `IsCDR2` and `LE` are type parameters so kind- and endian-specific paths
# dispatch at compile time. Immutable: the read cursor lives in the (mutable)
# `src` buffer, and the only field that ever changes during a parse — `origin`,
# reset at each XCDR1 parameter-list member boundary — is handled functionally
# by `resetOrigin` returning a fresh reader (see `memberHeaderV1`). Keeping the
# reader itself immutable lets it stack-allocate, so per-message and
# per-element (CDRArrayView/CDRView) readers cost no heap allocation.
struct CDRReader{B <: IO, IsCDR2, LE}
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

# Read directly from a pre-filled raw byte block — a `Memory{UInt8}` or a
# `Vector{UInt8}` — with no IOBuffer wrapper. Same wire layout: 4-byte
# preamble at the start, then payload.
CDRReader(mem::DenseVector{UInt8}) = CDRReader(MemBuf(mem))

# --- Kind-explicit construction --------------------------------------------
#
# `CDRReader(buf)` reads the encapsulation kind from the preamble at runtime, so
# its `IsCDR2`/`LE` type parameters — and the type of every view derived from it
# — are runtime-determined; inference can't pin them down, so a view built and
# consumed in the same scope has `Any`-typed fields unless a dispatch barrier
# (`read_view(f, buf, T)`) re-specialises them.
#
# When the wire format is fixed and known up front (a given topic/middleware),
# declare it instead: `CDRReader(buf, Val(isCDR2), Val(LE))` (or
# `CDRReader(buf, Val(CDR_LE))`). The reader type is then concrete at the call
# site, so `read_view`/views stay type-stable and allocation-free with no
# barrier. The preamble is still consumed and validated against the declaration.
# `@inline` + the inlinable `_read_prim` (rather than the generic `read(io, T)`)
# matter for allocation: with both, the transient `MemBuf` cursor never escapes
# into a non-inlined call, so escape analysis stack-promotes it and the whole
# `CDRReader(bytes, Val…)` + `read_view` pipeline runs with zero heap allocation
# — not even the cursor. (The runtime-kind `CDRReader(buf)` can't do this: its
# type isn't known at the call site, so the reader is materialised on the heap.)
@inline function CDRReader(buf::A, ::Val{IsCDR2}, ::Val{LE}) where {A <: IO, IsCDR2, LE}
    (IsCDR2 isa Bool && LE isa Bool) ||
        throw(ArgumentError("CDRReader: IsCDR2 and LE must be Bool"))
    preamble = ntoh(_read_prim(buf, UInt32))
    kind = UInt8((preamble >> 16) & 0xFF)
    aCDR2, aLE, usesDelimiterHeader, usesMemberHeader = getEncapsulationKind(kind)
    (aCDR2 === IsCDR2 && aLE === LE) ||
        throw(ArgumentError(string(
            "CDRReader: buffer encapsulation (kind byte ", repr(kind), ", CDR2=",
            aCDR2, ", LE=", aLE, ") does not match declared CDR2=", IsCDR2,
            ", LE=", LE)))
    # `EncapsulationKind(kind)` additionally rejects an unrecognised kind byte.
    return CDRReader{A, IsCDR2, LE}(buf, usesDelimiterHeader, usesMemberHeader,
                                    4, EncapsulationKind(kind))
end

@inline CDRReader(mem::DenseVector{UInt8}, c2::Val, le::Val) = CDRReader(MemBuf(mem), c2, le)

# Sugar: declare the exact `EncapsulationKind` rather than the two flags.
# Generated so the kind → (IsCDR2, LE) split happens at expansion time and the
# call lowers to the concrete-typed `Val`/`Val` constructor above.
@generated function CDRReader(buf::Union{IO, DenseVector{UInt8}}, ::Val{K}) where {K}
    K isa EncapsulationKind ||
        return :(throw(ArgumentError(string("CDRReader: expected an EncapsulationKind, got ",
                                            $(QuoteNode(K))))))
    isCDR2, le, _, _ = getEncapsulationKind(UInt8(K))
    return :(CDRReader(buf, Val($isCDR2), Val($le)))
end

@inline isCDR2(::CDRReader{<:Any, B}) where B = B
@inline littleEndian(::CDRReader{<:Any, <:Any, L}) where L = L

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
    r.src isa _CDRBufLike || throw(ArgumentError("limit! only supported for IOBuffer-/MemBuf-backed readers"))
    _buf_size!(r.src, position(r.src) + Int(length))
    return r
end

function Base.copy(r::CDRReader{IOBuffer, IsCDR2, LE}) where {IsCDR2, LE}
    newio = IOBuffer(r.src.data; read=true, write=false)
    newio.size = r.src.size
    seek(newio, position(r.src))
    return CDRReader{IOBuffer, IsCDR2, LE}(newio,
                                     r.usesDelimiterHeader, r.usesMemberHeader, r.origin, r.kind)
end

function Base.copy(r::CDRReader{B, IsCDR2, LE}) where {B <: MemBuf, IsCDR2, LE}
    # MemBuf clones share the underlying storage: branching readers must not
    # mutate the bytes themselves, only their own cursors.
    newbuf = MemBuf(r.src.mem, r.src.pos, r.src.written)
    return CDRReader{B, IsCDR2, LE}(newbuf,
                                     r.usesDelimiterHeader, r.usesMemberHeader, r.origin, r.kind)
end

isPresentFlag(::CDRReader{<:Any, false}) = throw(ArgumentError("isPresentFlag is only valid for CDR2 streams"))
isPresentFlag(r::CDRReader{<:Any, true}) = read(r, UInt8) != 0

# CDR alignment sizes are powers of two; the bitmask form avoids an `idiv`.
@inline function align(r::CDRReader, size::Int)
    src = r.src
    alignment = (position(src) - r.origin) & (size - 1)
    if alignment > 0
        skip(src, size - alignment)
    end
end

# 8-byte primitives align to 4 on CDR2, 8 on CDR1.
@inline align8(r::CDRReader{<:Any, true})  = align(r, 4)
@inline align8(r::CDRReader{<:Any, false}) = align(r, 8)

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

# Typed `unsafe_load!` directly off the backing buffer — bypasses the
# peek+skip path in `Base.read(::IOBuffer, ::Type{T})`.
@inline function _read_prim(buf::IOBuffer, ::Type{T}) where T
    n = sizeof(T)
    data = _buf_data(buf)
    ptr = _buf_pos(buf)
    v = GC.@preserve data unsafe_load(Ptr{T}(pointer(data, ptr)))
    _buf_pos!(buf, ptr + n)
    return v
end
# Cold throw kept out-of-line: constructing the BoundsError boxes the index, and
# leaving that in an inlined hot read body defeats the escape analysis that keeps
# the decode zero-alloc. Mirrors Base's `@noinline throw_boundserror`.
@noinline _throw_buf_bounds(data, idx) = throw(BoundsError(data, idx))

# MemBuf cursor + length come straight off the wire, so validate the load span
# against the watermark before dereferencing — a truncated/oversized payload must
# raise rather than read past the buffer. A single compare on the hot path.
@inline function _read_prim(buf::MemBuf, ::Type{T}) where T
    n = sizeof(T)
    data = _buf_data(buf)
    ptr = _buf_pos(buf)
    ptr + n - 1 <= _buf_size(buf) || _throw_buf_bounds(data, ptr + n - 1)
    v = GC.@preserve data unsafe_load(Ptr{T}(pointer(data, ptr)))
    _buf_pos!(buf, ptr + n)
    return v
end
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

# The reader is immutable, so resetting the alignment origin (done at each XCDR1
# parameter-list member boundary, where a member's value re-aligns from its own
# start) yields a *new* reader sharing the same `src` cursor. Member-header
# readers therefore return `(reader, header)`: the caller threads the returned
# reader so subsequent reads align against the member's origin.
resetOrigin(r::CDRReader{B, IsCDR2, LE}) where {B, IsCDR2, LE} =
    CDRReader{B, IsCDR2, LE}(r.src, r.usesDelimiterHeader, r.usesMemberHeader,
                             position(r.src), r.kind)

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
    error("Invalid length code $lengthCode")
end
function memberHeaderV1(r::CDRReader)
    align(r, 4)
    idHeader = read(r, UInt16)
    mustUnderstandFlag = (idHeader & 0x4000) >> 14 == 1
    implementationSpecificFlag = (idHeader & 0x8000) >> 15 == 1
    # Extended PID expands id and size to UInt32s appended after the header.
    extendedPIDFlag = (idHeader & 0x3fff) === EXTENDED_PID

    sentinelPIDFlag = (idHeader & 0x3fff) === SENTINEL_PID
    if sentinelPIDFlag
      # Caller saw a sentinel where it expected another member: an absent
      # optional member at the end of a struct. No member body follows, so the
      # origin is left as-is (reader returned unchanged).
      return (r, (id=UInt32(SENTINEL_PID), objectSize=UInt32(0), mustUnderstand=false,
              implementationSpecific=false, ignore=false,
              readSentinelHeader=true, lengthCode=UInt32(0)))
    end

    # Ignore-PID 0x3f03: skip the id, still consume the size + value.
    ignorePIDFlag = (idHeader & 0x3fff) == 0x3f03

    if (extendedPIDFlag)
      # Consume the trailing UInt16 sentinel that always reads as 8.
      read(r, UInt16)
    end

    id = extendedPIDFlag ? read(r, UInt32) : UInt32(idHeader & 0x3fff)
    objectSize = extendedPIDFlag ? read(r, UInt32) : UInt32(read(r, UInt16))
    r = resetOrigin(r)
    return (r, (; id, objectSize, mustUnderstand=mustUnderstandFlag,
            implementationSpecific=implementationSpecificFlag, ignore=ignorePIDFlag,
            readSentinelHeader=false, lengthCode=UInt32(0)))
end

sentinelHeader(::CDRReader{<:Any, true}) = nothing
function sentinelHeader(r::CDRReader{<:Any, false})
    align(r, 4)
    header = read(r, UInt16)
    sentinelPIDFlag = (header & 0x3fff) === SENTINEL_PID
    if !sentinelPIDFlag
        error("Expected SENTINEL_PID flag but got $header")
    end
    read(r, UInt16)
end

function memberHeaderV2(r::CDRReader)
    header = read(r, UInt32)
    mustUnderstand = (header & 0x80000000) >> 31 == 1
    lengthCode = (header & 0x70000000) >> 28
    id = header & 0x0fffffff

    objectSize = emHeaderObjectSize(r, lengthCode);

    # XCDR2 measures member alignment from the encapsulation start, so there is
    # no per-member origin reset; the reader is returned unchanged to match
    # `memberHeaderV1`'s `(reader, header)` contract.
    return (r, (; id, objectSize, mustUnderstand,
            implementationSpecific=false, ignore=false,
            readSentinelHeader=false, lengthCode))
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
        error("Invalid length code $lengthCode in EMHEADER at offset $(position(r.src) - 4)")
    end
end

sequenceLength(r::CDRReader) = read(r, UInt32)

# Caller aligns before; this is the raw element bulk read.
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

function Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where {T <:Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32}, A<:AbstractArray{T}}
    num == 0 && return T[]
    align(r, sizeof(T))
    return _readArrayBody!(r, T, num)
end

function Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where {T <:Union{Int64, UInt64, Float64}, A<:AbstractArray{T}}
    num == 0 && return T[]
    align8(r)
    return _readArrayBody!(r, T, num)
end

Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where A<:AbstractArray{String} = [read(r, String) for i=1:num]

# Vector of user structs (or any other non-primitive element). Reads the
# length prefix then constructs N inline values. `num` is wire-supplied; since
# every non-primitive element occupies at least one byte, a count exceeding the
# remaining bytes is malformed — reject it up front so a bogus length can't drive
# a huge allocation or an out-of-bounds field-walk.
function Base.read(r::CDRReader, ::Type{V}; num=sequenceLength(r)) where {T, V<:AbstractArray{T}}
    num <= bytesavailable(r.src) || _throw_buf_bounds(r.src, num)
    return [read(r, T) for _ in 1:num]
end

# SArray of non-primitive element (struct or nested SArray): no length
# prefix on the wire (SArray represents a fixed-length array in CDR IDL).
# Primitive-element SArrays use more specific methods above.
function Base.read(r::CDRReader, ::Type{SA}) where {S, T, N, L, SA<:SArray{S, T, N, L}}
    return SA(ntuple(_ -> read(r, T), Val(L)))
end

# Read a user struct by walking its declared fields. CDR encodes nested
# structs inline (no per-struct headers), so each field reads independently.
@generated function _read_user_struct(r::CDRReader, ::Type{T}) where T
    if isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0
        field_exprs = Expr[:(read(r, $(fieldtype(T, i)))) for i in 1:fieldcount(T)]
        return Expr(:call, T, field_exprs...)
    end
    return :(throw(ArgumentError(string("read: unsupported type ", $T))))
end

# Compact-struct fast path: when Julia's in-memory layout matches CDR
# encoding (isbits + no trailing pad + matching endianness + CDR1), one
# `unsafe_load` of the whole struct is bit-equivalent to per-field reads
# and LLVM lowers it to a single wide SIMD load. The caller guarantees
# compactness via `_is_compact_struct` before invoking.
@generated function _read_struct_compact(r::R, ::Type{T}) where {R <: CDRReader{<:_CDRBufLike}, T}
    isCDR2 = R.parameters[2]
    max_align = _wa_align_for(T, isCDR2)
    sz = sizeof(T)
    return quote
        align(r, $max_align)
        src = r.src
        data = _buf_data(src)
        ptr = _buf_pos(src)
        _check_load_span(src, ptr, $sz)
        v = GC.@preserve data unsafe_load(Ptr{$T}(pointer(data, ptr)))
        _buf_pos!(src, ptr + $sz)
        return v
    end
end

@generated function _read_value(r::R, ::Type{T}) where {R <: CDRReader, T}
    isCDR2 = R.parameters[2]
    LE     = R.parameters[3]
    if R <: CDRReader{<:_CDRBufLike} && _is_compact_struct(T, isCDR2, LE)
        return :(_read_struct_compact(r, T))
    end
    return :(_read_user_struct(r, T))
end

Base.read(r::CDRReader, ::Type{T}) where T = _read_value(r, T)

# A raw `unsafe_load` of `nbytes` at the (1-based) cursor must stay within the
# data watermark. IOBuffer-backed reads reach here only through the size-checked
# owned paths, so the guard is a no-op there; a MemBuf is fed directly from the
# wire, so its span is validated before the dereference (a single compare).
@inline _check_load_span(::IOBuffer, ptr::Int, nbytes::Int) = nothing
@inline function _check_load_span(buf::MemBuf, ptr::Int, nbytes::Int)
    ptr + nbytes - 1 <= _buf_size(buf) || _throw_buf_bounds(_buf_data(buf), ptr + nbytes - 1)
    return nothing
end

# Raw-buffer fast path: pull the tuple straight out with `unsafe_load`. The
# `Ref{NTuple}` form below allocates on Julia 1.11 (escape analysis on 1.12+
# stack-promotes it, but 1.11 doesn't).
@inline function _readStaticArrayBody(r::CDRReader{<:_CDRBufLike, <:Any, true}, ::Type{SA}) where {S, T, N, L, SA<:SArray{S, T, N, L}}
    src = r.src
    data = _buf_data(src)
    ptr = _buf_pos(src)
    _check_load_span(src, ptr, L * sizeof(T))
    nt = GC.@preserve data unsafe_load(Ptr{NTuple{L, T}}(pointer(data, ptr)))
    _buf_pos!(src, ptr + L * sizeof(T))
    return SA(nt)
end

@inline function _readStaticArrayBody(r::CDRReader{<:_CDRBufLike, <:Any, false}, ::Type{SA}) where {S, T, N, L, SA<:SArray{S, T, N, L}}
    src = r.src
    data = _buf_data(src)
    ptr = _buf_pos(src)
    _check_load_span(src, ptr, L * sizeof(T))
    nt = GC.@preserve data unsafe_load(Ptr{NTuple{L, T}}(pointer(data, ptr)))
    _buf_pos!(src, ptr + L * sizeof(T))
    if sizeof(T) > 1
        return SA(ntoh.(nt))
    end
    return SA(nt)
end

# Generic IO fallback: `unsafe_read` needs a destination pointer.
@inline function _readStaticArrayBody(r::CDRReader{<:IO, <:Any, true}, ::Type{SA}) where {S, T, N, L, SA<:SArray{S, T, N, L}}
    ref = Ref{NTuple{L, T}}()
    GC.@preserve ref unsafe_read(r.src, Ptr{UInt8}(pointer_from_objref(ref)), L * sizeof(T))
    return SA(ref[])
end

@inline function _readStaticArrayBody(r::CDRReader{<:IO, <:Any, false}, ::Type{SA}) where {S, T, N, L, SA<:SArray{S, T, N, L}}
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
    push!(body, :(local_ptr = _buf_pos(src)))
    push!(body, :(data = _buf_data(src)))

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

    push!(body, :(_buf_pos!(src, local_ptr + $total_size)))
    push!(body, Expr(:return, Expr(:tuple, value_syms...)))
    return Expr(:block, body...)
end

"""
    read_all!(r::CDRReader, ::Type{Tuple{T1, T2, …}}) -> Tuple{T1, T2, …}

Read a schema's worth of values from `r` in a single packed operation, with
offsets and padding resolved at compile time. Accepted element types match
[`write_all!`](@ref): primitives (Int8…Float64, Bool, Char) and
`StaticArrays.SArray` of those. Only IOBuffer-/MemBuf-backed readers are supported.
"""
@generated function read_all!(r::CDRReader{<:_CDRBufLike, IsCDR2, LE},
                              ::Type{Schema}) where {IsCDR2, LE, Schema <: Tuple}
    decl = collect(Schema.parameters)
    return _ra_build_body(decl, IsCDR2)
end