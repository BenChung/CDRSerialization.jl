module CDRSerialization

using StaticArrays

include("membuf.jl")
include("layout.jl")
include("reader.jl")
include("writer.jl")
include("sizecalculator.jl")
include("reinterpret.jl")
export CDRReader, CDRWriter, CDRSizeCalculator, @cdr_compact, @cdr1_compat,
       reinterpret_struct, reinterpret_array, reinterpret_string, read_view,
       canview, iscompact, cdr_layout, CDRLayout, CDRArray, CDRArrayView, CDRString, CDRView,
       materialize

end # module CDR
