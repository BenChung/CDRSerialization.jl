# CDRSerialization.jl

A port of Foxglove's [CDR library for JavaScript](https://github.com/foxglove/cdr) to Julia.

Implements `CDRReader` and `CDRWriter` over any `IO`, plus `CDRSizeCalculator` for sizing a buffer before writing. `read(r::CDRReader, T)` and `write(w::CDRWriter, v::T)` are defined for:

* Primitives: `Int8`, `UInt8`, `Bool`, `Char`, `Int16`, `UInt16`, `Int32`, `UInt32`, `Float32`, `Int64`, `UInt64`, `Float64`, `String`
* `Vector{T}` where `T` is a primitive type
* `SArray{S, T, N, L}` of any shape (`SVector`, `SMatrix`, higher-dim).

Multi-dim `SArray` round-trips in **column-major** order — Julia's native storage layout. Callers coming from row-major sources (C, NumPy, ROS conventions) should transpose the result or read into the transposed shape.

## Reader navigation

`Base.position`, `Base.seek`, `Base.skip`, `Base.eof`, plus `isAtEnd`, `decodedBytes`, `limit!(r, n)`, and `Base.copy(r)` (clone). `limit!` and `copy` require an `IOBuffer`-backed reader.

## Sizing a buffer ahead of time

```julia
calc = CDRSizeCalculator()
add!(calc, UInt32)
add!(calc, String, sizeof("base_link"))
add!(calc, Vector{Float64}, 7)
position(calc)  # exact byte count the writer will emit
```
