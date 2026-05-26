module CDRSerialization

using StaticArrays

include("layout.jl")
include("reader.jl")
include("writer.jl")
include("sizecalculator.jl")
export CDRReader, CDRWriter, CDRSizeCalculator, @cdr_compact

end # module CDR
