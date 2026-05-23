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

@testset "CDRSizeCalculator matches writer output" begin
    # Build the example message via the writer
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.sequenceLength(w, 1)
    write(w, UInt32(1490149580))
    write(w, UInt32(117017840))
    write(w, "base_link")
    write(w, "radar")
    write(w, Float64(3.835))
    write(w, Float64(0))
    write(w, Float64(0))
    write(w, Float64(0))
    write(w, Float64(0))
    write(w, Float64(0))
    write(w, Float64(1))

    # Compute the same size via the calculator
    calc = CDRSizeCalculator()
    CDRSerialization.sequenceLength!(calc)
    CDRSerialization.add!(calc, UInt32)
    CDRSerialization.add!(calc, UInt32)
    CDRSerialization.add!(calc, String, sizeof("base_link"))
    CDRSerialization.add!(calc, String, sizeof("radar"))
    for _ in 1:7
        CDRSerialization.add!(calc, Float64)
    end

    @test position(calc) == position(data)
end

@testset "reader seek/skip/isAtEnd" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, UInt32(1))
    write(w, UInt32(2))
    write(w, UInt32(3))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test !CDRSerialization.isAtEnd(r)
    @test CDRSerialization.byteLength(r) == 16  # 4 preamble + 3 * UInt32
    skip(r, 4)
    @test read(r, UInt32) == 2
    @test CDRSerialization.decodedBytes(r) == 8
    @test CDRSerialization.byteLength(r) == 16  # unchanged by reads
    @test read(r, UInt32) == 3
    @test CDRSerialization.isAtEnd(r)
end

@testset "presentFlag round-trip (CDR2)" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR2_LE)
    CDRSerialization.presentFlag(w, true)
    CDRSerialization.presentFlag(w, false)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test CDRSerialization.isPresentFlag(r) == true
    @test CDRSerialization.isPresentFlag(r) == false
end

@testset "reader clone branches independently" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, UInt32(42))
    write(w, UInt32(99))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, UInt32) == 42
    r2 = copy(r)
    @test read(r, UInt32) == 99
    @test read(r2, UInt32) == 99
end

@testset "reader limit truncates remaining bytes" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, UInt32(1))
    write(w, UInt32(2))
    write(w, UInt32(3))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    CDRSerialization.limit!(r, 8)
    @test read(r, UInt32) == 1
    @test read(r, UInt32) == 2
    @test CDRSerialization.isAtEnd(r)
end

@testset "V1 implementationSpecific PID does not throw" begin
    # Construct CDR_LE buffer with a member header that has bit 15 set,
    # followed by a 4-byte payload, and verify we can step past it.
    raw = UInt8[0x00, 0x01, 0x00, 0x00]
    # idHeader: implementationSpecific=1, mustUnderstand=0, id=1
    append!(raw, reinterpret(UInt8, UInt16[(UInt16(1) << 15) | UInt16(1)]))
    append!(raw, reinterpret(UInt8, UInt16[4]))           # objectSize = 4
    append!(raw, reinterpret(UInt8, UInt32[42]))           # payload
    r = CDRSerialization.CDRReader(IOBuffer(raw))
    h = CDRSerialization.memberHeaderV1(r)
    @test h.implementationSpecific == true
    @test h.id == 1
    @test h.objectSize == 4
    @test read(r, UInt32) == 42
end

@testset "uint16BE / uint32BE / uint64BE round-trip on LE stream" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR_LE)
    CDRSerialization.uint16BE(w, UInt16(0x1234))
    CDRSerialization.uint32BE(w, UInt32(0xDEADBEEF))
    CDRSerialization.uint64BE(w, UInt64(0x0102030405060708))
    # confirm the bytes are big-endian regardless of the stream's LE kind
    bytes = CDRSerialization.data(w)
    # After preamble (4 bytes): UInt16 at 5-6, then 2 bytes alignment padding,
    # UInt32 at 9-12 (already 8-aligned for the UInt64 that follows).
    @test bytes[5:6] == UInt8[0x12, 0x34]
    @test bytes[9:12] == UInt8[0xDE, 0xAD, 0xBE, 0xEF]
    @test bytes[13:20] == UInt8[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test CDRSerialization.uint16BE(r) == 0x1234
    @test CDRSerialization.uint32BE(r) == 0xDEADBEEF
    @test CDRSerialization.uint64BE(r) == 0x0102030405060708
end

@testset "writer data / position accessors" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, UInt32(0xCAFEBABE))
    @test position(w) == 8  # 4 preamble + 4 UInt32
    bytes = CDRSerialization.data(w)
    @test length(bytes) == 8
    @test bytes[5:8] == UInt8[0xBE, 0xBA, 0xFE, 0xCA]
    # non-destructive: buffer still has the data
    @test position(w) == 8
end

@testset "Vector{String} round-trip" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    arr = ["foo", "barbaz", "héllo"]
    write(w, arr, true)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, Vector{String}) == arr
end

@testset "SMatrix round-trip" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    m = SMatrix{2, 3, Float64}(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    write(w, m)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, SMatrix{2, 3, Float64, 6}) === m
end

@testset "3-D SArray round-trip" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    a = SArray{Tuple{2, 2, 2}, Int32, 3, 8}(Int32.(1:8))
    write(w, a)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, SArray{Tuple{2, 2, 2}, Int32, 3, 8}) === a
end

@testset "SArray read is non-allocating after warmup" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    a = SMatrix{3, 4, Int32}(Int32.(1:12))
    write(w, a)
    payload_pos = 4  # right after preamble; payload starts here (sizeof(Int32)==4)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    # warmup
    read(r, SMatrix{3, 4, Int32, 12})
    seek(r, payload_pos)
    allocs = @allocated read(r, SMatrix{3, 4, Int32, 12})
    @test allocs == 0
end

@testset "V1 ignore-PID is surfaced (0x3f03)" begin
    raw = UInt8[0x00, 0x01, 0x00, 0x00]
    append!(raw, reinterpret(UInt8, UInt16[0x3f03]))       # ignore-PID, no flags
    append!(raw, reinterpret(UInt8, UInt16[4]))            # objectSize
    append!(raw, reinterpret(UInt8, UInt32[0]))            # payload (to be skipped)
    r = CDRSerialization.CDRReader(IOBuffer(raw))
    h = CDRSerialization.memberHeaderV1(r)
    @test h.ignore == true
    @test h.objectSize == 4
    skip(r, Int(h.objectSize))
    @test CDRSerialization.isAtEnd(r)
end

include("fuzz.jl")

