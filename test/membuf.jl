# Memory{UInt8}-backed reader/writer: same wire format as the IOBuffer path,
# just targeting a pre-allocated raw block instead of a growable Vector.

using CDRSerialization: CDRReader, CDRWriter, write_all!, read_all!, data

# Write a message twice — once to an IOBuffer, once to a Memory of the same
# size — and assert the bytes are identical.
function _assert_membuf_matches(build!; kind=CDRSerialization.CDR_LE)
    io = IOBuffer()
    wio = CDRWriter(io, kind)
    build!(wio)
    expected = copy(take!(wio.buf))

    mem = Memory{UInt8}(undef, length(expected))
    wmem = CDRWriter(mem, kind)
    build!(wmem)
    got = collect(data(wmem))
    @test got == expected
    return mem, length(expected)
end

@testset "MemBuf writer matches IOBuffer bytes" begin
    _assert_membuf_matches() do w
        CDRSerialization.sequenceLength(w, 1)
        write(w, UInt32(1490149580))
        write(w, UInt32(117017840))
        write(w, "base_link")
        write(w, "radar")
        write(w, Float64(3.835))
        write(w, Float64(0))
        write(w, SVector(0.0, 1.0, 3.0))
        write(w, SVector(UInt8(1), UInt8(2), UInt8(3)))
    end
end

@testset "MemBuf primitive round-trip" begin
    mem = Memory{UInt8}(undef, 64)
    w = CDRWriter(mem)
    write(w, UInt32(0xDEADBEEF))
    write(w, Float64(3.14159))
    write(w, Int16(-7))
    write(w, 'Q')

    r = CDRReader(mem)
    @test read(r, UInt32) == 0xDEADBEEF
    @test read(r, Float64) == 3.14159
    @test read(r, Int16) == -7
    @test read(r, Char) == 'Q'
end

@testset "MemBuf string + vector round-trip" begin
    mem = Memory{UInt8}(undef, 128)
    w = CDRWriter(mem)
    write(w, "héllo")
    write(w, [1.0, 2.0, 3.0], true)
    write(w, ["a", "bb", "ccc"], true)

    r = CDRReader(mem)
    @test read(r, String) == "héllo"
    @test read(r, Vector{Float64}) == [1.0, 2.0, 3.0]
    @test read(r, Vector{String}) == ["a", "bb", "ccc"]
end

@testset "MemBuf SArray round-trip" begin
    mem = Memory{UInt8}(undef, 256)
    w = CDRWriter(mem)
    m = SMatrix{2, 3, Float64}(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    a = SArray{Tuple{2, 2, 2}, Int32, 3, 8}(Int32.(1:8))
    write(w, m)
    write(w, a)

    r = CDRReader(mem)
    @test read(r, SMatrix{2, 3, Float64, 6}) === m
    @test read(r, SArray{Tuple{2, 2, 2}, Int32, 3, 8}) === a
end

@testset "MemBuf write_all! / read_all! round-trip" begin
    # write_all! reserves a worst-case upper bound, so size generously.
    mem = Memory{UInt8}(undef, 256)
    w = CDRWriter(mem)
    write_all!(w,
        UInt8(42), Int16(-3), Float32(2.5), Float64(3.14),
        SVector{3, Float64}(1.0, 2.0, 3.0))

    r = CDRReader(mem)
    got = read_all!(r, Tuple{UInt8, Int16, Float32, Float64, SVector{3, Float64}})
    @test got == (UInt8(42), Int16(-3), Float32(2.5), Float64(3.14),
                  SVector{3, Float64}(1.0, 2.0, 3.0))
end

struct _MemPoint
    x::Float64
    y::Float64
    z::Float64
end
struct _MemQuat
    x::Float64; y::Float64; z::Float64; w::Float64
end
struct _MemPose
    position::_MemPoint
    orientation::_MemQuat
end

@testset "MemBuf compact struct fast path round-trip" begin
    mem = Memory{UInt8}(undef, 256)
    w = CDRWriter(mem)
    pose = _MemPose(_MemPoint(1.0, 2.0, 3.0), _MemQuat(0.1, 0.2, 0.3, 0.4))
    write_all!(w, pose)

    r = CDRReader(mem)
    @test read(r, _MemPose) == _MemPose(_MemPoint(1.0, 2.0, 3.0), _MemQuat(0.1, 0.2, 0.3, 0.4))
end

@testset "MemBuf bounds check throws on overflow" begin
    mem = Memory{UInt8}(undef, 8)   # 4 preamble + room for one UInt32
    w = CDRWriter(mem)
    write(w, UInt32(1))
    @test_throws BoundsError write(w, UInt32(2))
end

@testset "MemBuf reader clone branches independently" begin
    mem = Memory{UInt8}(undef, 64)
    w = CDRWriter(mem)
    write(w, UInt32(42))
    write(w, UInt32(99))

    r = CDRReader(mem)
    @test read(r, UInt32) == 42
    r2 = copy(r)
    @test read(r, UInt32) == 99
    @test read(r2, UInt32) == 99
end

# AllocCheck: the MemBuf fast paths must be allocation-free just like the
# IOBuffer ones. MemBuf is fixed-size so there is no growth to filter out.
const _MemReaderT = CDRSerialization.CDRReader{CDRSerialization.MemBuf{Memory{UInt8}}, false, true}
const _MemWriterT = CDRSerialization.CDRWriter{false, true, CDRSerialization.MemBuf{Memory{UInt8}}}

@testset "AllocCheck: MemBuf primitives" begin
    for T in _PRIM_READ_TYPES
        @test isempty(check_allocs(_read_typed, (_MemReaderT, Type{T})))
    end
    for v in _PRIM_WRITE_VALUES
        @test isempty(check_allocs(_write_val, (_MemWriterT, typeof(v))))
    end
end

@testset "AllocCheck: MemBuf SArray reads / writes" begin
    for SA in _SARRAY_TYPES
        @test isempty(check_allocs(_read_sarray, (_MemReaderT, Type{SA})))
        @test isempty(check_allocs(_write_sarray, (_MemWriterT, SA)))
    end
end

@testset "AllocCheck: MemBuf compact struct read" begin
    @test isempty(check_allocs(_read_pose, (_MemReaderT,)))
end

# Vector{UInt8} backing: the same MemBuf machinery, just a different
# (equally contiguous) storage type.
@testset "Vector{UInt8}-backed round-trip" begin
    buf = Vector{UInt8}(undef, 256)
    w = CDRWriter(buf)
    write_all!(w,
        UInt8(42), Int16(-3), Float32(2.5), Float64(3.14),
        SVector{3, Float64}(1.0, 2.0, 3.0))
    write(w, "trailing")

    r = CDRReader(buf)
    got = read_all!(r, Tuple{UInt8, Int16, Float32, Float64, SVector{3, Float64}})
    @test got == (UInt8(42), Int16(-3), Float32(2.5), Float64(3.14),
                  SVector{3, Float64}(1.0, 2.0, 3.0))
    @test read(r, String) == "trailing"
end

@testset "Vector{UInt8} backing matches Memory + IOBuffer bytes" begin
    build! = w -> begin
        write(w, UInt32(7))
        write(w, "base_link")
        write(w, SVector(1.0, 2.0, 3.0))
    end

    io = IOBuffer(); build!(CDRWriter(io)); expected = copy(take!(io))

    vec = Vector{UInt8}(undef, length(expected))
    build!(CDRWriter(vec))
    @test vec[1:length(expected)] == expected
end

const _VecReaderT = CDRSerialization.CDRReader{CDRSerialization.MemBuf{Vector{UInt8}}, false, true}
const _VecWriterT = CDRSerialization.CDRWriter{false, true, CDRSerialization.MemBuf{Vector{UInt8}}}

@testset "AllocCheck: Vector-backed MemBuf primitives + compact read" begin
    for T in _PRIM_READ_TYPES
        @test isempty(check_allocs(_read_typed, (_VecReaderT, Type{T})))
    end
    for v in _PRIM_WRITE_VALUES
        @test isempty(check_allocs(_write_val, (_VecWriterT, typeof(v))))
    end
    @test isempty(check_allocs(_read_pose, (_VecReaderT,)))
end
