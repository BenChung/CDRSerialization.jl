function writeExampleMessage(w::CDRWriter)
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
end

encapsulationkinds=[
    CDR.CDR2_BE,
    CDR.CDR2_LE,
    CDR.PL_CDR2_BE,
    CDR.PL_CDR2_LE,
    CDR.DELIMITED_CDR2_BE,
    CDR.DELIMITED_CDR2_LE,
    CDR.RTPS_CDR2_BE,
    CDR.RTPS_CDR2_LE,
    CDR.RTPS_PL_CDR2_BE,
    CDR.RTPS_PL_CDR2_LE,
    CDR.RTPS_DELIMITED_CDR2_BE,
    CDR.RTPS_DELIMITED_CDR2_LE,
]

tf2_msg__TFMessage =
    hex2bytes("0001000001000000cce0d158f08cf9060a000000626173655f6c696e6b000000060000007261646172000000ae47e17a14ae0e4000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000f03f")
msg_without_header = tf2_msg__TFMessage[3:end]

for writer in [
    CDR.CDRWriter(IOBuffer()),
    CDR.CDRWriter(IOBuffer(), CDR.CDR_LE)
]
    tf2msg = [hex2bytes("00"); UInt8(writer.kind); msg_without_header]
    writeExampleMessage(writer)
    @test writer.buf.size == 100
    @test bytes2hex(take!(writer.buf)) == bytes2hex(tf2msg)
end
