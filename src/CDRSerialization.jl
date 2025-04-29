module CDRSerialization

using StaticArrays

include("reader.jl")
include("writer.jl")
export CDRReader, CDRWriter

end # module CDR
