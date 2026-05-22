using CDRSerialization, Test
using StaticArrays

@testset "Reader" begin 
    include("reader.jl")
end
@testset "Writer" begin
    include("writer.jl")
end

@testset "readwrite" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    # geometry_msgs/TransformStamped[] transforms
    CDRSerialization.sequenceLength(w, 1)
    # std_msgs/Header header
    # time stamp
    write(w, UInt32(1490149580)) # uint32 sec
    write(w, UInt32(117017840)) # uint32 nsec
    write(w, "base_link") # string frame_id
    write(w, "radar") # string child_frame_id
    # geometry_msgs/Transform transform
    # geometry_msgs/Vector3 translation
    write(w, Float64(3.835)) # float64 x
    write(w, Float64(0)) # float64 y
    write(w, Float64(0)) # float64 z
    # geometry_msgs/Quaternion rotation
    write(w, Float64(0)) # float64 x
    write(w, Float64(0)) # float64 y
    write(w, Float64(0)) # float64 z
    write(w, Float64(1)) # float64 w
    write(w, SVector(0.0, 1.0, 3.0))
    write(w, SVector(-2.0, -1.0, 4.0))
    write(w, SVector(3.0, 1.0, 0.0))
    write(w, SVector(UInt8(0), UInt8(1), UInt8(2)))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, UInt32) == UInt32(1)
    @test read(r, UInt32) == UInt32(1490149580)
    @test read(r, UInt32) == UInt32(117017840)
    @test read(r, String) == "base_link"
    @test read(r, String) == "radar"
    @test read(r, Float64) == 3.835
    @test read(r, Float64) == 0
    @test read(r, Float64) == 0
    @test read(r, Float64) == 0
    @test read(r, Float64) == 0
    @test read(r, Float64) == 0
    @test read(r, Float64) == 1
    @test (read(r, SVector{3, Float64}), read(r, SVector{3, Float64}), read(r, SVector{3, Float64})) == ([0.0, 1.0, 3.0], [-2.0, -1.0, 4.0], [3.0, 1.0, 0.0])
    @test read(r, SVector{3, UInt8}) == [0, 1, 2]
end

@testset "non-ASCII string round-trip" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, "héllo")
    # follow with a UInt8 so any mismatch between declared length and
    # actual UTF-8 byte count is not absorbed by 4-byte alignment padding.
    write(w, UInt8(42))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, String) == "héllo"
    @test read(r, UInt8) == 42
end

@testset "Vector{Float64} round-trip" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, [1.0, 2.0, 3.0], true)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, Vector{Float64}) == [1.0, 2.0, 3.0]
end

@testset "memberHeader V1 mustUnderstand round-trip" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR_LE)
    CDRSerialization.emHeader(w, true, 1, 4, nothing)
    write(w, UInt32(42))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    h = CDRSerialization.memberHeaderV1(r)
    @test h.mustUnderstand == true
    @test h.id == 1
    @test h.objectSize == 4
    @test read(r, UInt32) == 42
end

@testset "V2 emHeader length code 2" begin
    # CDR2_LE encapsulation header, then EMHEADER with mustUnderstand=1, lengthCode=2, id=5
    buf = UInt8[0x00, 0x11, 0x00, 0x00]
    append!(buf, reinterpret(UInt8, UInt32[0xA0000005]))
    append!(buf, reinterpret(UInt8, UInt32[42]))
    r = CDRSerialization.CDRReader(IOBuffer(buf))
    h = CDRSerialization.memberHeaderV2(r)
    @test h.mustUnderstand == true
    @test h.id == 5
    @test h.objectSize == 4
    @test h.lengthCode == 2
    @test read(r, UInt32) == 42
end

@testset "Char round-trip" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, 'A')
    write(w, 'z')
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, Char) == 'A'
    @test read(r, Char) == 'z'
end

