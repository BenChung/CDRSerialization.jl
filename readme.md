# CDRSerialization.jl

A port of Foxglove's [CDR library for Javascript](https://github.com/foxglove/cdr) to Julia.
Implments `CDRReader` and `CDRWriter` which consume an `IO` and implement `read(r::CDRReader, t)` and `write(w::CDRWriter, v)` where `t::Type{T}` or `v::T` and `T` is one of

* Primitives `Int8`, `UInt8`, `Char`, `Bool Int16`, `UInt16`, `Int32`, `UInt32`, `Float32`, `Int64`, `UInt64`, `Float64`, or `String`
* `Vector{T}` (where `T` is one of the above primitive types)
* `SArray{T, N}` (where `N` is a known dimension of the array and `T` is a primitive type)

See `test/reader.jl` and `test/writer.jl` for examples.