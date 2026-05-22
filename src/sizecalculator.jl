mutable struct CDRSizeCalculator
    offset::Int
    eightByteAlignment::Int
end

CDRSizeCalculator(; isCDR2::Bool=false) = CDRSizeCalculator(4, isCDR2 ? 4 : 8)

Base.position(c::CDRSizeCalculator) = c.offset

function align!(c::CDRSizeCalculator, size)
    alignment = (c.offset - 4) % size
    if alignment > 0
        c.offset += size - alignment
    end
end

function add!(c::CDRSizeCalculator, ::Type{T}) where T <: Union{Int8, UInt8, Bool}
    c.offset += sizeof(T)
    return c
end

add!(c::CDRSizeCalculator, ::Type{Char}) = add!(c, UInt8)

function add!(c::CDRSizeCalculator, ::Type{T}) where T <: Union{Int16, UInt16, Int32, UInt32, Float32}
    align!(c, sizeof(T))
    c.offset += sizeof(T)
    return c
end

function add!(c::CDRSizeCalculator, ::Type{T}) where T <: Union{Int64, UInt64, Float64}
    align!(c, c.eightByteAlignment)
    c.offset += sizeof(T)
    return c
end

sequenceLength!(c::CDRSizeCalculator) = add!(c, UInt32)

function add!(c::CDRSizeCalculator, ::Type{String}, bytes::Integer)
    sequenceLength!(c)
    c.offset += Int(bytes) + 1
    return c
end

const _ArrayElt = Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32, Int64, UInt64, Float64}

function add!(c::CDRSizeCalculator, ::Type{Vector{T}}, count::Integer;
              writeLength::Bool=true) where T <: _ArrayElt
    if writeLength
        sequenceLength!(c)
    end
    if count > 0
        alignment = T <: Union{Int64, UInt64, Float64} ? c.eightByteAlignment : sizeof(T)
        align!(c, alignment)
        c.offset += Int(count) * sizeof(T)
    end
    return c
end
