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

@testset "write_all! / read_all! round-trip" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w,
        UInt8(42), Int16(-3), Float32(2.5), Float64(3.14),
        SVector{3, Float64}(1.0, 2.0, 3.0))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    got = CDRSerialization.read_all!(r,
        Tuple{UInt8, Int16, Float32, Float64, SVector{3, Float64}})
    @test got == (UInt8(42), Int16(-3), Float32(2.5), Float64(3.14),
                  SVector{3, Float64}(1.0, 2.0, 3.0))
end

struct _TestPoint
    x::Float64
    y::Float64
    z::Float64
end

struct _TestQuat
    x::Float64; y::Float64; z::Float64; w::Float64
end

struct _TestPose
    position::_TestPoint
    orientation::_TestQuat
end

struct _TestNamedPose
    name::String
    pose::_TestPose
end

@testset "write_all! flattens packed nested structs" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    pose = _TestPose(_TestPoint(1.0, 2.0, 3.0), _TestQuat(0.1, 0.2, 0.3, 0.4))
    CDRSerialization.write_all!(w, UInt32(42), pose, UInt16(7))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, UInt32) == 42
    @test read(r, Float64) == 1.0  # position.x
    @test read(r, Float64) == 2.0  # position.y
    @test read(r, Float64) == 3.0  # position.z
    @test read(r, Float64) == 0.1  # orientation.x
    @test read(r, Float64) == 0.2
    @test read(r, Float64) == 0.3
    @test read(r, Float64) == 0.4  # orientation.w
    @test read(r, UInt16) == 7
end

@testset "write_all! walks structs containing dynamic fields" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    named = _TestNamedPose("base_link",
                           _TestPose(_TestPoint(3.835, 0.0, 0.0), _TestQuat(0.0, 0.0, 0.0, 1.0)))
    CDRSerialization.write_all!(w, named)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, String) == "base_link"
    @test read(r, Float64) == 3.835
    @test read(r, Float64) == 0.0
    @test read(r, Float64) == 0.0
    @test read(r, Float64) == 0.0
    @test read(r, Float64) == 0.0
    @test read(r, Float64) == 0.0
    @test read(r, Float64) == 1.0
end

@testset "CDRSizeCalculator walks structs via addValue!" begin
    pose = _TestPose(_TestPoint(1.0, 2.0, 3.0), _TestQuat(0.1, 0.2, 0.3, 0.4))
    named = _TestNamedPose("base_link", pose)
    calc = CDRSizeCalculator()
    CDRSerialization.addValue!(calc, named)

    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, named)
    @test position(calc) == position(data)
end

@testset "Vector{Struct} round-trip" begin
    poses = [
        _TestPose(_TestPoint(1.0, 2.0, 3.0), _TestQuat(0.0, 0.0, 0.0, 1.0)),
        _TestPose(_TestPoint(-4.5, 6.5, 7.25), _TestQuat(0.1, 0.2, 0.3, 0.4)),
        _TestPose(_TestPoint(0.0, 0.0, 0.0), _TestQuat(1.0, 0.0, 0.0, 0.0)),
    ]
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, poses)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, Vector{_TestPose}) == poses

    # Verify the size calculator matches.
    calc = CDRSizeCalculator()
    CDRSerialization.addValue!(calc, poses)
    @test position(calc) == position(data)
end

@testset "Vector{Struct} with dynamic field round-trip" begin
    items = [
        _TestNamedPose("left", _TestPose(_TestPoint(1.0, 2.0, 3.0), _TestQuat(0.0, 0.0, 0.0, 1.0))),
        _TestNamedPose("right", _TestPose(_TestPoint(-1.0, 2.0, 3.0), _TestQuat(0.0, 0.0, 0.0, 1.0))),
    ]
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, items)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, Vector{_TestNamedPose}) == items
end

@testset "Vector{Struct} inside a struct round-trip" begin
    # Common ROS pattern: a struct whose body is a length-prefixed sequence
    # of inner structs (e.g. `tf2_msgs/TFMessage`).
    poses = [
        _TestPose(_TestPoint(1.0, 2.0, 3.0), _TestQuat(0.0, 0.0, 0.0, 1.0)),
        _TestPose(_TestPoint(4.0, 5.0, 6.0), _TestQuat(0.7, 0.0, 0.7, 0.0)),
    ]
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, UInt32(0xDEADBEEF), poses, UInt8(7))
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, UInt32) == 0xDEADBEEF
    @test read(r, Vector{_TestPose}) == poses
    @test read(r, UInt8) == 7
end

# Recursive case: container struct holding a Vector of inner structs that
# themselves contain a Vector. Exercises the round-trip through every
# dispatch layer (struct → array → struct → array).
struct _TestInner
    label::String
    samples::Vector{Float64}
end

struct _TestOuter
    name::String
    inners::Vector{_TestInner}
end

# Default struct `==` falls back to `===` on non-isbits fields; provide
# structural equality so the round-trip assertions work.
Base.:(==)(a::_TestInner, b::_TestInner) = a.label == b.label && a.samples == b.samples
Base.:(==)(a::_TestOuter, b::_TestOuter) = a.name == b.name && a.inners == b.inners

@testset "recursively nested arrays of structs round-trip" begin
    outer = _TestOuter("frame_0",
                       [_TestInner("a", [1.0, 2.0, 3.0]),
                        _TestInner("b", Float64[]),
                        _TestInner("c", [10.0, 20.0])])
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, outer)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, _TestOuter) == outer

    calc = CDRSizeCalculator()
    CDRSerialization.addValue!(calc, outer)
    @test position(calc) == position(data)
end

# A naturally-non-compact struct (Float64 then UInt8 → 7 trailing pad bytes
# in Julia, only 9 bytes in standard CDR). @cdr_compact emits two inner
# storage types — one each for CDR1 (Float64 aligns to 8 → 7 trailing pad)
# and XCDR2 (Float64 aligns to 4 → 3 trailing pad) — and a public wrapper
# `_PadCompact{V}` that dispatches on variant.
@cdr_compact struct _PadCompact
    x::Float64
    y::UInt8
end

@testset "@cdr_compact: CDR1 inner layout + compact path" begin
    Inner = var"__PadCompact_CDR1"
    @test sizeof(Inner) == 16
    @test CDRSerialization.iscompact(Inner)            # CDR1, host endianness
    @test CDRSerialization._struct_cdr_size(Inner, false) == 16

    val = _PadCompact(3.14, 0x42)             # default → CDR1 variant
    @test val isa _PadCompact{Inner}
    @test val.x == 3.14
    @test val.y == 0x42

    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)     # default CDR_LE → CDR1
    CDRSerialization.write_all!(w, val)
    @test position(w) == 4 + 16              # preamble + 16 bytes

    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    got = read(r, _PadCompact)
    @test got isa _PadCompact{Inner}
    @test got.x == 3.14
    @test got.y == 0x42
end

@testset "@cdr_compact: CDR2 inner layout + compact path" begin
    Inner = var"__PadCompact_CDR2"
    @test sizeof(Inner) == 12                # 8 (Float64-as-2xF32) + 1 + 3 pad
    @test CDRSerialization.iscompact(Inner; cdr2=true)
    @test CDRSerialization._struct_cdr_size(Inner, true) == 12

    val = _PadCompact(3.14, 0x42; xcdr2=true)
    @test val isa _PadCompact{Inner}
    @test val.x == 3.14                       # transparent via getproperty
    @test val.y == 0x42

    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR2_LE)
    CDRSerialization.write_all!(w, val)
    @test position(w) == 4 + 12

    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    got = read(r, _PadCompact)
    @test got isa _PadCompact{Inner}
    @test got.x == 3.14
    @test got.y == 0x42
end

@cdr_compact struct _PoseCompact
    position::SVector{3, Float64}
    orientation::SVector{4, Float64}
end

@testset "@cdr_compact: SArray of Float64 round-trips under both variants" begin
    pos  = SVector{3, Float64}(1.0, 2.0, 3.0)
    quat = SVector{4, Float64}(0.0, 0.0, 0.0, 1.0)

    # CDR1: SArray fields kept as-is; layout naturally compact.
    val1 = _PoseCompact(pos, quat)
    @test val1 isa _PoseCompact{var"__PoseCompact_CDR1"}
    @test val1.position == pos
    @test val1.orientation == quat

    buf1 = IOBuffer()
    w1 = CDRSerialization.CDRWriter(buf1)
    CDRSerialization.write_all!(w1, val1)
    seekstart(buf1)
    r1 = CDRSerialization.CDRReader(buf1)
    got1 = read(r1, _PoseCompact)
    @test got1.position == pos
    @test got1.orientation == quat

    # CDR2: SArray fields substituted to SVector{N, Float32} for storage,
    # transparently reinterpreted back on access.
    val2 = _PoseCompact(pos, quat; xcdr2=true)
    @test val2 isa _PoseCompact{var"__PoseCompact_CDR2"}
    @test val2.position == pos
    @test val2.orientation == quat
    inner_pos_storage = getfield(getfield(val2, :inner), :position)
    @test inner_pos_storage isa SVector{6, Float32}

    buf2 = IOBuffer()
    w2 = CDRSerialization.CDRWriter(buf2, CDRSerialization.CDR2_LE)
    CDRSerialization.write_all!(w2, val2)
    seekstart(buf2)
    r2 = CDRSerialization.CDRReader(buf2)
    got2 = read(r2, _PoseCompact)
    @test got2.position == pos
    @test got2.orientation == quat
end

@testset "@cdr_compact: variant mismatch is rejected" begin
    cdr1_val = _PadCompact(1.0, 0x01)
    cdr2_val = _PadCompact(1.0, 0x01; xcdr2=true)

    # CDR2 writer can't write the CDR1 wrapper, and vice versa.
    data = IOBuffer()
    w_cdr2 = CDRSerialization.CDRWriter(data, CDRSerialization.CDR2_LE)
    @test_throws ArgumentError CDRSerialization.write_all!(w_cdr2, cdr1_val)

    data2 = IOBuffer()
    w_cdr1 = CDRSerialization.CDRWriter(data2)
    @test_throws ArgumentError CDRSerialization.write_all!(w_cdr1, cdr2_val)
end

@testset "SVector{Struct} round-trip (no length prefix)" begin
    triplet = SVector(
        _TestPose(_TestPoint(1.0, 2.0, 3.0), _TestQuat(0.0, 0.0, 0.0, 1.0)),
        _TestPose(_TestPoint(4.0, 5.0, 6.0), _TestQuat(0.7, 0.0, 0.7, 0.0)),
        _TestPose(_TestPoint(7.0, 8.0, 9.0), _TestQuat(0.0, 1.0, 0.0, 0.0)),
    )
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, UInt32(99), triplet)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, UInt32) == 99
    @test read(r, typeof(triplet)) == triplet

    calc = CDRSizeCalculator()
    CDRSerialization.addValue!(calc, UInt32(99))
    CDRSerialization.addValue!(calc, triplet)
    @test position(calc) == position(data)
end

@testset "write_all! with a Vector{String} field/arg" begin
    # Regression: the byte-budget for a string sequence once used a closure,
    # which made the @generated write_all! body non-pure.
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, UInt32(3), ["foo", "barbaz", "héllo"])
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, UInt32) == 3
    @test read(r, Vector{String}) == ["foo", "barbaz", "héllo"]
end

@testset "write_all! mixed packed / dynamic types" begin
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w,
        UInt8(7), Int32(-99),                         # packed run
        "hello world",                                # dynamic
        Float64(3.14), SVector{2, Int32}(10, 20),     # packed run
        [1.0, 2.0, 3.0],                              # dynamic
        UInt16(123))                                  # packed run
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, UInt8)   == 7
    @test read(r, Int32)   == -99
    @test read(r, String)  == "hello world"
    @test read(r, Float64) == 3.14
    @test read(r, SVector{2, Int32}) == SVector{2, Int32}(10, 20)
    @test read(r, Vector{Float64}) == [1.0, 2.0, 3.0]
    @test read(r, UInt16)  == 123
end

@testset "AllocCheck" begin
    include("alloc_check.jl")
end

@testset "MemBuf (Memory-backed)" begin
    include("membuf.jl")
end

@testset "reinterpret_struct (zero-copy load)" begin
    include("reinterpret.jl")
end

include("fuzz.jl")

