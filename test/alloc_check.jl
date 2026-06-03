using AllocCheck

# IOBuffer growth (ensureroom / _resize!) is amortized and unavoidable for a
# growable buffer; filter it so the test only flags per-call allocations.
_is_iobuffer_growth(alloc) = any(frame -> begin
    s = string(frame)
    occursin("_resize!", s) || occursin("ensureroom", s) || occursin("_similar_data", s)
end, alloc.backtrace)

_allocs(f, types) = filter(!_is_iobuffer_growth, check_allocs(f, types))

# `T` stays in dispatch (vs. a closure capturing a local) so each check_allocs
# sees a fully concrete signature.
_read_typed(r, ::Type{T}) where T = read(r, T)
_write_val(w, v) = write(w, v)
_read_sarray(r, ::Type{SA}) where SA = read(r, SA)
_write_sarray(w, a) = write(w, a)

const _PRIM_READ_TYPES = (
    Int8, UInt8, Bool, Char,
    Int16, UInt16,
    Int32, UInt32, Float32,
    Int64, UInt64, Float64,
)

const _PRIM_WRITE_VALUES = (
    Int8(1), UInt8(1), true, 'A',
    Int16(1), UInt16(1),
    Int32(1), UInt32(1), Float32(1),
    Int64(1), UInt64(1), Float64(1),
)

const _SARRAY_TYPES = (
    SArray{Tuple{3}, UInt8, 1, 3},
    SArray{Tuple{3}, Int32, 1, 3},
    SArray{Tuple{3}, Float64, 1, 3},
    SArray{Tuple{2, 3}, Float64, 2, 6},
    SArray{Tuple{2, 2, 2}, Int32, 3, 8},
)

# Concrete types for the default CDR1 + LE encapsulation.
const _ReaderT = CDRSerialization.CDRReader{IOBuffer, false, true}
const _WriterT = CDRSerialization.CDRWriter{false, true, IOBuffer}
const _CalcT   = CDRSerialization.CDRSizeCalculator

@testset "AllocCheck: CDRReader primitives" begin
    for T in _PRIM_READ_TYPES
        @test isempty(_allocs(_read_typed, (_ReaderT, Type{T})))
    end
end

@testset "AllocCheck: CDRWriter primitives" begin
    for v in _PRIM_WRITE_VALUES
        @test isempty(_allocs(_write_val, (_WriterT, typeof(v))))
    end
end

@testset "AllocCheck: BE helpers" begin
    @test isempty(_allocs(CDRSerialization.uint16BE, (_ReaderT,)))
    @test isempty(_allocs(CDRSerialization.uint32BE, (_ReaderT,)))
    @test isempty(_allocs(CDRSerialization.uint64BE, (_ReaderT,)))
    @test isempty(_allocs(CDRSerialization.uint16BE, (_WriterT, UInt16)))
    @test isempty(_allocs(CDRSerialization.uint32BE, (_WriterT, UInt32)))
    @test isempty(_allocs(CDRSerialization.uint64BE, (_WriterT, UInt64)))
end

@testset "AllocCheck: member / delimiter / sentinel headers" begin
    @test isempty(_allocs(CDRSerialization.memberHeaderV1, (_ReaderT,)))
    @test isempty(_allocs(CDRSerialization.memberHeaderV2, (_ReaderT,)))
    @test isempty(_allocs(CDRSerialization.dHeader, (_ReaderT,)))
    @test isempty(_allocs(CDRSerialization.dHeader, (_WriterT, Int)))
    @test isempty(_allocs(CDRSerialization.sentinelHeader, (_WriterT,)))
    @test isempty(_allocs(CDRSerialization.sentinelHeader, (_ReaderT,)))
    @test isempty(_allocs(CDRSerialization.emHeader, (_ReaderT,)))
end

@testset "AllocCheck: presentFlag / isPresentFlag" begin
    @test isempty(_allocs(CDRSerialization.isPresentFlag, (_ReaderT,)))
    @test isempty(_allocs(CDRSerialization.presentFlag, (_WriterT, Bool)))
end

@testset "AllocCheck: SArray reads / writes" begin
    for SA in _SARRAY_TYPES
        @test isempty(_allocs(_read_sarray, (_ReaderT, Type{SA})))
        @test isempty(_allocs(_write_sarray, (_WriterT, SA)))
    end
end

_add_calc(c, ::Type{T}) where T = CDRSerialization.add!(c, T)
_add_calc_vec(c, ::Type{V}, n) where V = CDRSerialization.add!(c, V, n)
_add_calc_str(c, n) = CDRSerialization.add!(c, String, n)

@testset "AllocCheck: CDRSizeCalculator" begin
    for T in _PRIM_READ_TYPES
        @test isempty(_allocs(_add_calc, (_CalcT, Type{T})))
    end
    for T in (Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32, Int64, UInt64, Float64)
        @test isempty(_allocs(_add_calc_vec, (_CalcT, Type{Vector{T}}, Int)))
    end
    @test isempty(_allocs(_add_calc_str, (_CalcT, Int)))
    @test isempty(_allocs(CDRSerialization.sequenceLength!, (_CalcT,)))
end

# Test structs spanning the patterns we care about: pure-primitive,
# SArray-of-primitive field, and one with a String (forces the dynamic path).
struct _AllocPoint
    x::Float64
    y::Float64
    z::Float64
end

struct _AllocPose
    position::_AllocPoint
    orientation::SVector{4, Float64}
end

struct _AllocNamedPose
    name::String
    pose::_AllocPose
end

_write_pose(w, p)        = CDRSerialization.write_all!(w, p)
_write_named(w, n)       = CDRSerialization.write_all!(w, n)
_write_pose_vec(w, v)    = CDRSerialization.write_all!(w, v)
_calc_value(c, v)        = CDRSerialization.addValue!(c, v)
_read_pose(r)            = read(r, _AllocPose)
_gwrite_pose(w, p)       = write(w, p)
_gwrite_named(w, n)      = write(w, n)
_gwrite_pose_vec(w, v)   = write(w, v)

@testset "AllocCheck: nested struct writes" begin
    # Packed nested struct → fully flat unsafe_store chain, no allocations.
    @test isempty(_allocs(_write_pose, (_WriterT, _AllocPose)))

    # Struct with String field uses the dynamic path. String content itself
    # gets memcpy'd from the existing String object — the wire format
    # doesn't allocate a new string.
    @test isempty(_allocs(_write_named, (_WriterT, _AllocNamedPose)))

    # Vector{Struct}: writes length prefix + N inline structs.
    @test isempty(_allocs(_write_pose_vec, (_WriterT, Vector{_AllocPose})))
end

@testset "AllocCheck: generic write(c, struct) / write(c, Vector)" begin
    # Compact struct → single-store fast path; non-compact-with-String → field
    # walk; Vector{Struct} → length prefix + per-element walk. All alloc-free.
    @test isempty(_allocs(_gwrite_pose, (_WriterT, _AllocPose)))
    @test isempty(_allocs(_gwrite_named, (_WriterT, _AllocNamedPose)))
    @test isempty(_allocs(_gwrite_pose_vec, (_WriterT, Vector{_AllocPose})))
end

@testset "AllocCheck: compact struct read" begin
    # AllocPose contains Point + SVector{4, Float64}: layout-compatible
    # → single `unsafe_load` of the whole struct, no allocations.
    @test isempty(_allocs(_read_pose, (_ReaderT,)))
end

@testset "AllocCheck: addValue! on structs" begin
    @test isempty(_allocs(_calc_value, (_CalcT, _AllocPose)))
    @test isempty(_allocs(_calc_value, (_CalcT, _AllocNamedPose)))
    @test isempty(_allocs(_calc_value, (_CalcT, Vector{_AllocPose})))
end

# Flat but NOT compact (trailing pad after `flag`): a sequence of these views as
# a `CDRArrayView`, whose `getindex` decodes cursor-free (no per-element reader).
struct _AllocTail
    a::Float64
    b::Float64
    flag::UInt8
end

# String + Vector{flat-non-compact} fields → the view carries a `CDRString` and a
# `CDRArrayView`, the two paths that previously boxed.
struct _AllocFrame
    id::UInt32
    pts::Vector{_AllocTail}
    label::String
end

const _CdrStrT = CDRSerialization.CDRString{Vector{UInt8}}
const _CdrAVT  = CDRSerialization.CDRArrayView{_AllocTail, false, true, Vector{UInt8}}

_read_view_typed(r, ::Type{T}) where T = read_view(r, T)
_cdrav_get(v, i) = @inbounds v[i]
_str_len(s) = ncodeunits(s)
_str_cu(s)  = codeunit(s, 1)

@testset "AllocCheck: zero-copy views (read_view / CDRArrayView / CDRString)" begin
    # On a concrete reader type (the state after a dispatch barrier), the
    # generated straight-line decode yields a fully concrete `CDRView` returned
    # by value — including its `CDRString` and `CDRArrayView` fields.
    @test isempty(_allocs(_read_view_typed, (_ReaderT, Type{_AllocFrame})))
    # `CDRArrayView` element decode threads a plain `Int` cursor — no `MemBuf`.
    @test isempty(_allocs(_cdrav_get, (_CdrAVT, Int)))
    # `CDRString` aliases the buffer; reading code units copies nothing.
    @test isempty(_allocs(_str_len, (_CdrStrT,)))
    @test isempty(_allocs(_str_cu, (_CdrStrT,)))
end

# End-to-end from a raw byte buffer: a kind-explicit reader plus `read_view`
# plus consuming the `CDRString`/`CDRArrayView` fields. The transient cursor is
# stack-promoted, so the whole decode is heap-free. A runtime `@allocated`
# check (not AllocCheck) because the guarantee depends on escape analysis
# promoting the mutable `MemBuf` — a property of the optimized code, not the
# static call graph.
function _decode_frame_explicit(buf)
    r = CDRSerialization.CDRReader(buf, Val(false), Val(true))
    v = read_view(r, _AllocFrame)
    s = 0.0
    for p in v.pts
        s += p.a
    end
    return s + ncodeunits(v.label) + Int(v.id)
end

@testset "zero-alloc kind-explicit decode (raw buffer, no barrier)" begin
    io = IOBuffer()
    CDRSerialization.write_all!(CDRSerialization.CDRWriter(io, CDRSerialization.CDR_LE),
                                _AllocFrame(UInt32(7),
                                            [_AllocTail(Float64(i), 2.0, UInt8(i % 256)) for i in 1:16],
                                            "base_link_frame"))
    buf = take!(io)
    _decode_frame_explicit(buf)                 # warm up / compile
    @test @allocated(_decode_frame_explicit(buf)) == 0
end
