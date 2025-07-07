mutable struct CDRWriter
    buf::IOBuffer

    littleEndian::Bool
    eightByteAlignment::Int
    isCDR2::Bool
    usesDelimiterHeader::Bool
    usesMemberHeader::Bool
    
    origin::Int
    kind::EncapsulationKind

    function CDRWriter(buf::IOBuffer = IOBuffer(), kind::EncapsulationKind=CDR_LE)
        isCDR2, littleEndian, usesDelimiterHeader, usesMemberHeader = getEncapsulationKind(UInt8(kind))
        write(buf, UInt8(0))
        write(buf, UInt8(kind))
        write(buf, UInt16(0))
        new(buf, littleEndian, isCDR2 ? 4 : 8, isCDR2, usesDelimiterHeader, usesMemberHeader, 4, kind) # julia is 1-indexed
    end
end

function resetOrigin(c::CDRWriter)
    c.origin = position(c.buf)
end

function align(c::CDRWriter, size)
    alignment = (position(c.buf) - c.origin) % size
    padding = alignment > 0 ? size - alignment : 0
    if alignment > 0
        for i=1:padding
            write(c.buf, UInt8(0))
        end
    end
end

function Base.write(c::CDRWriter, v::Union{Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Bool}) 
    align(c, sizeof(typeof(v)))
    write(c.buf, c.littleEndian ? v : ntoh(v))
end

function Base.write(c::CDRWriter, v::Union{Float64, UInt64, Int64}) 
    align(c, c.eightByteAlignment)
    write(c.buf, c.littleEndian ? v : ntoh(v))
end

function Base.write(c::CDRWriter, v::String, writeLength=true) 
    if writeLength
        align(c, 4)
        write(c.buf, UInt32(length(v) + 1))
    end
    write(c.buf, v)
    write(c.buf, UInt8('\0'))
end

dHeader(c::CDRWriter, objectSize) = write(c, UInt32(objectSize))

function emHeader(c::CDRWriter, mustUnderstand::Bool, id::Int, objectSize::Int, lengthCode::Union{Nothing, Int})
    if c.isCDR2
        memberHeaderV2(c, mustUnderstand, id, objectSize, lengthCode)
    else
        memberHeaderV1(c, mustUnderstand, id, objectSize, lengthCode)
    end
end
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

function sentinelHeader(c::CDRWriter)
    if !c.isCDR2
        align(c, 4)
        write(c, UInt16(SENTINEL_PID))
        write(c, UInt16(0))
    end
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
        shouldBeSize = lengthCodeToObjectSizes[lengthCode]
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
Base.write(w::CDRWriter, a::A, writeLength=false) where {T <:Union{Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32}, A<:AbstractArray{T}} = writeArray(w, a, sizeof(T), writeLength)
Base.write(w::CDRWriter, a::A, writeLength=false) where {T <:Union{UInt64, Int64, Float64}, A<:AbstractArray{T}} = writeArray(w, a, r.eightByteAlignment, writeLength)
function writeArray(w::CDRWriter, a::A, alignment, writeLength=false) where A
    if writeLength
        sequenceLength(w, length(a))
    end
    if w.littleEndian
        align(w, alignment)
        write(w.buf, a)
    else 
        for v in a 
            write(w, ntoh(v))
        end
    end
end

function Base.write(w::CDRWriter, a::A, writeLength=false) where A<:AbstractArray{String}
    if writeLength
        sequenceLength(w, length(a))
    end
    for s in a
        write(w, s)
    end
end
Base.write(w::CDRWriter, a::A, writeLength=false) where {T<:Union{Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Bool}, D, A<:SArray{Tuple{D}, T}} = writeStaticArray(w, a, sizeof(T))
Base.write(w::CDRWriter, a::A, writeLength=false) where {T <:Union{UInt64, Int64, Float64}, D, A<:SArray{Tuple{D}, T}} = writeStaticArray(w, a, w.eightByteAlignment)
function writeStaticArray(w::CDRWriter, a::A, alignment, writeLength=false) where A
    if writeLength
        sequenceLength(w, length(a))
    end
    if w.littleEndian
        align(w, alignment)
        write(w.buf, a)
    else 
        for v in a 
            write(w, ntoh(v))
        end
    end
    return nothing
end