module CDRSerialization

using StaticArrays

include("reader.jl")
include("writer.jl")
include("sizecalculator.jl")
export CDRReader, CDRWriter, CDRSizeCalculator

end # module CDR
