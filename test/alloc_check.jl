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
const _WriterT = CDRSerialization.CDRWriter{false, true}
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
