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

mutable struct CDRReader{A <: AbstractArray{UInt8}}
    buf::A

    littleEndian::Bool
    eightByteAlignment::Int
    isCDR2::Bool
    usesDelimiterHeader::Bool
    usesMemberHeader::Bool

    origin::Int
    offset::Int

    kind::EncapsulationKind

    function CDRReader(buf::A) where A <: AbstractArray{UInt8}
        if (length(buf) < 4)
            throw("Invalid CDR data size, must have at least 4 bytes")
        end
        kind = buf[2]
        isCDR2, littleEndian, usesDelimiterHeader, usesMemberHeader = getEncapsulationKind(kind)
        new{A}(buf, littleEndian, isCDR2 ? 4 : 8, isCDR2, usesDelimiterHeader, usesMemberHeader, 5, 5, EncapsulationKind(kind)) # julia is 1-indexed
    end
end

function align(r::CDRReader, size::Int)
    alignment = (r.offset - r.origin) % size
    if alignment > 0
        r.offset += size - alignment
    end
end

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

function Base.read(r::CDRReader, ::Type{T}) where T <: Union{Int8, UInt8, Char}
    align(r, sizeof(T))
    value = reinterpret(T, @view r.buf[r.offset:end])[1]
    r.offset += sizeof(T)
    return value
end

function Base.read(r::CDRReader, ::Type{T}) where T <: Union{Int16, UInt16, Int32, UInt32, Float32, Int64, UInt64, Float64}
    if sizeof(T) == 8
        align(r, r.eightByteAlignment)
    else
        align(r, sizeof(T))
    end

    value = reinterpret(T, @view r.buf[r.offset:r.offset+sizeof(T)-1])[1] 
    
    if !r.littleEndian
        value = ntoh(value)
    end
    
    r.offset += sizeof(T)
    return value
end

Base.read(r::CDRReader, ::Type{String}) = read(r, String, read(r, UInt32))
function Base.read(r::CDRReader, ::Type{String}, len::Integer)
    if len <= 1
        r.offset += len
        return ""
    end
    if r.buf[r.offset + len - 1] == 0x00 # null terminated
        value = String(@view r.buf[r.offset:r.offset + len - 2])
    else 
        value = String(@view r.buf[r.offset:r.offset + len - 1])
    end
    r.offset += len
    return value
end

dHeader(r::CDRReader) = read(r, UInt32)

function emHeader(r::CDRReader)
    if r.isCDR2
        memberHeaderV2(r)
    else
        memberHeaderV1(r)
    end
end

resetOrigin(r::CDRReader) = r.origin = r.offset

const EXTENDED_PID = 0x3f01
const SENTINEL_PID = 0x3f02
function memberHeaderV1(r::CDRReader)
    align(r, 4)
    idHeader = read(r, UInt16)
    mustUnderstandFlag = (idHeader & 0x4000) >> 14 === 1
    # indicates that the parameter has a implementation-specific interpretation
    implementationSpecificFlag = (idHeader & 0x8000) >> 15 === 1

    # Allows the specification of large member ID and/or data length values
    # requires the reading in of two uint32's for ID and size
    extendedPIDFlag = (idHeader & 0x3fff) === EXTENDED_PID

    # Indicates the end of the parameter list structure
    sentinelPIDFlag = (idHeader & 0x3fff) === SENTINEL_PID
    if sentinelPIDFlag
      # Return that we have read the sentinel header when we expected to read an emHeader.
      # This can happen for absent optional members at the end of a struct.
      return (id=SENTINEL_PID, objectSize=0, mustUnderstand=false, readSentinelHeader=true)
    end

    # Indicates that the ID should be ignored
    # ignorePIDFlag = (idHeader & 0x3fff) === 0x3f03;

    usesReservedParameterId = (idHeader & 0x3fff) > SENTINEL_PID;

    # Not trying to support right now if we don't need to
    if (usesReservedParameterId || implementationSpecificFlag)
      throw("Unsupported parameter ID header $(idHeader)")
    end

    if (extendedPIDFlag)
      # Need to consume last part of header (is just an 8 in this case)
      # Alignment could take care of this, but I want to be explicit
      read(r, UInt16)
    end

    id = extendedPIDFlag ? read(r, UInt32) : idHeader & 0x3fff
    objectSize = extendedPIDFlag ? read(r, UInt32) : read(r, UInt16)
    resetOrigin(r)
    return (; id, objectSize, mustUnderstand=mustUnderstandFlag)
end

function sentinelHeader(r::CDRReader)
    if r.isCDR2
        return
    end
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
    mustUnderstand = abs((header & 0x80000000) >> 31) === 1
    # LC is the value of the Length Code for the member.
    lengthCode = (header & 0x70000000) >> 28
    id = header & 0x0fffffff

    objectSize = emHeaderObjectSize(r, lengthCode);

    return (; mustUnderstand, id, objectSize, lengthCode);
end

function emHeaderObjectSize(r::CDRReader, lengthCode)
    if lengthCode == 0 || lengthCode == 1 ||
        lengthCode == 2 || lengthCode == 3
        return lengthCodeToObjectSizes[lengthCode]
    elseif lengthCode == 4 || lengthCode == 5
        return read(r, UInt32)
    elseif lengthCode == 6
        return read(r, UInt32) * 4
    elseif lengthCode == 7
        return read(r, UInt32) * 8
    else 
        throw("Invalid length code $lengthCode in EMHEADER at offset $(r.offset - 4)")
    end
end

sequenceLength(r::CDRReader) = read(r, UInt32)
Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where {T <:Union{Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32}, A<:AbstractArray{T}} = readArray(r, T, num, sizeof(T))
Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where {T <:Union{UInt64, Int64, Float64}, A<:AbstractArray{T}} = readArray(r, T, num, r.eightByteAlignment)
Base.read(r::CDRReader, ::Type{A}; num=sequenceLength(r)) where A<:AbstractArray{String} = [read(r, String) for i=1:num]

function readArray(r::CDRReader, ::Type{T}, count, alignment) where T
    if count == 0
        return T[]
    end
    align(r, alignment)
    if !r.littleEndian
        array = copy(reinterpret(T, @view r.buf[r.offset:r.offset + count*sizeof(T) - 1]))
        r.offset += count * sizeof(T)
        array .= ntoh.(array)
        return array
    elseif r.offset % sizeof(T) === 0
        array = reinterpret(T, @view r.buf[r.offset:r.offset + count*sizeof(T) - 1])
        r.offset += count * sizeof(T)
        return array
    else
        array = Vector{T}(undef, count)
        for i=1:count
            if !r.littleEndian
                array[i] = ntoh(reinterpret(T, @view r.buf[r.offset:end])[1])
            else
                array[i] = reinterpret(T, @view r.buf[r.offset:end])[1]
            end
            r.offset += sizeof(T)
        end
        return array
    end
end