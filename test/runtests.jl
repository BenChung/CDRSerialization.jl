using CDR, Test
using StaticArrays

@testset "Reader" begin 
    include("reader.jl")
end
@testset "Writer" begin 
    include("reader.jl")
end

@testset "readwrite" begin
    data = IOBuffer()
    w = CDR.CDRWriter(data)
    # geometry_msgs/TransformStamped[] transforms
    CDR.sequenceLength(w, 1)
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
    r = CDR.CDRReader(data)
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

