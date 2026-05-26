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

# Value-based API: walks user structs by their fields, so callers can
# size-compute a nested struct in one call. Strings and Arrays carry the
# runtime length-dependent contribution.
@inline addValue!(c::CDRSizeCalculator, v::Union{Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32, Int64, UInt64, Float64}) =
    add!(c, typeof(v))
@inline addValue!(c::CDRSizeCalculator, ::Char) = add!(c, Char)
@inline addValue!(c::CDRSizeCalculator, v::AbstractString) = add!(c, String, sizeof(v))

function addValue!(c::CDRSizeCalculator, v::AbstractArray{T}; writeLength::Bool=true) where T <: _ArrayElt
    add!(c, Vector{T}, length(v); writeLength=writeLength)
end

function addValue!(c::CDRSizeCalculator, v::AbstractArray{String}; writeLength::Bool=true)
    if writeLength
        sequenceLength!(c)
    end
    for s in v
        add!(c, String, sizeof(s))
    end
    return c
end

# Vector of non-primitive elements (structs, nested vectors, …)
function addValue!(c::CDRSizeCalculator, v::AbstractArray{T}; writeLength::Bool=true) where T
    if writeLength
        sequenceLength!(c)
    end
    for elt in v
        addValue!(c, elt)
    end
    return c
end

# SArray of primitive: same alignment as the element type, then L * sizeof(T) bytes.
function addValue!(c::CDRSizeCalculator, v::SArray{S, T, N, L}) where {S, T <: _ArrayElt, N, L}
    L == 0 && return c
    alignment = T <: Union{Int64, UInt64, Float64} ? c.eightByteAlignment : sizeof(T)
    align!(c, alignment)
    c.offset += L * sizeof(T)
    return c
end

# SArray of non-primitive: fixed length, no prefix, each element walked.
function addValue!(c::CDRSizeCalculator, v::SArray{S, T, N, L}) where {S, T, N, L}
    L == 0 && return c
    for elt in v
        addValue!(c, elt)
    end
    return c
end

# User struct fallback: unroll field accesses at compile time so the walk
# is fully type-stable for each concrete struct.
@generated function addValue!(c::CDRSizeCalculator, v::T) where T
    if isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0
        body = Expr[]
        for i in 1:fieldcount(T)
            fname = fieldname(T, i)
            push!(body, :(addValue!(c, getfield(v, $(QuoteNode(fname))))))
        end
        push!(body, :(return c))
        return Expr(:block, body...)
    end
    return :(throw(ArgumentError(string("CDRSizeCalculator: unsupported value of type ", $T))))
end
