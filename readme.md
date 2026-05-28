# CDRSerialization.jl

A port of Foxglove's [CDR library for JavaScript](https://github.com/foxglove/cdr) to Julia.

Implements `CDRReader` and `CDRWriter` over any `IO` (or a raw `Memory{UInt8}` / `Vector{UInt8}`, see below), plus `CDRSizeCalculator` for sizing a buffer before writing. `read(r::CDRReader, T)` and `write(w::CDRWriter, v::T)` are defined for:

* Primitives: `Int8`, `UInt8`, `Bool`, `Char`, `Int16`, `UInt16`, `Int32`, `UInt32`, `Float32`, `Int64`, `UInt64`, `Float64`, `String`
* `Vector{T}` where `T` is a primitive type
* `SArray{S, T, N, L}` of any shape (`SVector`, `SMatrix`, higher-dim).

Multi-dim `SArray` round-trips in **column-major** order — Julia's native storage layout. Callers coming from row-major sources (C, NumPy, ROS conventions) should transpose the result or read into the transposed shape.

## Reader navigation

`Base.position`, `Base.seek`, `Base.skip`, `Base.eof`, plus `isAtEnd`, `decodedBytes`, `limit!(r, n)`, and `Base.copy(r)` (clone). `limit!` and `copy` require an `IOBuffer`-backed reader.

## Reading and writing a raw `Memory{UInt8}` / `Vector{UInt8}`

Besides any `IO`, both ends accept a pre-allocated `Memory{UInt8}` or
`Vector{UInt8}` directly — no `IOBuffer` wrapper. The wire format is identical
(4-byte preamble followed by the payload); only the backing store differs. All
the single-`unsafe_load`/`unsafe_store!` fast paths (compact structs, SArrays,
`read_all!`/`write_all!`) fire exactly as they do for an `IOBuffer`.

```julia
mem = Memory{UInt8}(undef, 256)   # or Vector{UInt8}(undef, 256)
w = CDRWriter(mem)                # or CDRWriter(mem, CDR2_LE)
write(w, UInt32(1))
write(w, SVector(1.0, 2.0, 3.0))

r = CDRReader(mem)
read(r, UInt32)               # 1
read(r, SVector{3, Float64})  # [1.0, 2.0, 3.0]
```

Such a buffer is **fixed-size**: a write that would overrun it throws a
`BoundsError` (an `IOBuffer` grows instead). The per-value `write` API
reserves exactly, so a buffer sized with `CDRSizeCalculator` fits precisely.
`write_all!` reserves a conservative upper bound up front (its whole speed
model is one reservation followed by unchecked stores), so a buffer used with
`write_all!` must be sized to that worst case, not the exact byte count — size
generously, or reuse one max-sized scratch buffer.

## Sizing a buffer ahead of time

```julia
calc = CDRSizeCalculator()
add!(calc, UInt32)
add!(calc, String, sizeof("base_link"))
add!(calc, Vector{Float64}, 7)
position(calc)  # exact byte count the writer will emit
```
