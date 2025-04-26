
@testset "TFMessage" begin
    tf2_msg__TFMessage =
    hex2bytes("0001000001000000cce0d158f08cf9060a000000626173655f6c696e6b000000060000007261646172000000ae47e17a14ae0e4000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000f03f")

    reader = CDR.CDRReader(IOBuffer(tf2_msg__TFMessage))
    @test reader.kind == CDR.CDR_LE
    #@test reader.offset == 5
    @test CDR.sequenceLength(reader) == 1 # geometry_msgs/TransformStamped[] transforms
    @test read(reader, UInt32) == 1490149580 # uint32 sec
    @test read(reader, UInt32) == 117017840 # uint32 nsec
    @test read(reader, String) == "base_link" # string frame_id
    @test read(reader, String) == "radar" # string child_frame_id

    # geometry_msgs/Transform transform
    # geometry_msgs/Vector3 translation
    @test read(reader, Float64) ≈ 3.835 # float64 x
    @test read(reader, Float64) ≈ 0 # float64 y
    @test read(reader, Float64) ≈ 0 # float64 z
    # geometry_msgs/Quaternion rotation
    @test read(reader, Float64) ≈ 0 # float64 x
    @test read(reader, Float64) ≈ 0 # float64 y
    @test read(reader, Float64) ≈ 0 # float64 z
    @test read(reader, Float64) ≈ 1 # float64 w

end

@testset "rcl_interfaces/ParameterEvent" begin 
    data = hex2bytes("00010000a9b71561a570ea01110000002f5f726f7332636c695f33373833363300000000010000000d0000007573655f73696d5f74696d650001000000000000000000000000000000000000010000000000000000000000000000000000000000000000000000000000000000000000")
    reader = CDR.CDRReader(IOBuffer(data))
    # builtin_interfaces/Time stamp
    @test read(reader, UInt32) == 1628813225 # uint32 sec
    @test read(reader, UInt32) == 32141477 # uint32 nsec
    # string node
    @test read(reader, String) == "/_ros2cli_378363"

    # Parameter[] new_parameters
    @test CDR.sequenceLength(reader) == 1
    @test read(reader, String) == "use_sim_time" # string name
    # ParameterValue value
    @test read(reader, UInt8) == 1 # uint8 type
    @test read(reader, Int8) == 0 # bool bool_value
    @test read(reader, Int64) == 0 # int64 integer_value
    @test read(reader, Float64) == 0 # float64 double_value
    @test read(reader, String) == "" # string string_value

    @test read(reader, Vector{Int8}) == Int8[] # byte[] byte_array_value
    @test read(reader, Vector{UInt8}) == UInt8[] # bool[] bool_array_value
    @test read(reader, Vector{Int64}) == Int64[] # int64[] integer_array_value
    @test read(reader, Vector{Float64}) == Float64[] # float64[] double_array_value
    @test read(reader, Vector{String}) == String[] # string[] string_array_value

    # Parameter[] changed_parameters
    @test CDR.sequenceLength(reader) == 0

    # Parameter[] deleted_parameters
    @test CDR.sequenceLength(reader) == 0
end