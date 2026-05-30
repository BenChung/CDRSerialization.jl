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
# storage types: CDR1 is a flat plain struct that serializes as standard CDR
# (9 bytes, no trailing pad), XCDR2 keeps trailing pad for its single-store
# path (Float64-as-2xF32 aligns to 4 → 3 trailing pad). A public wrapper
# `_PadCompact{V}` dispatches on variant.
@cdr_compact struct _PadCompact
    x::Float64
    y::UInt8
end

@testset "@cdr_compact: CDR1 inner layout is wire-compatible" begin
    Inner = var"__PadCompact_CDR1"
    @test sizeof(Inner) == 16                          # Julia pads to 8-align
    @test !CDRSerialization.iscompact(Inner)           # has trailing pad → not single-store
    @test CDRSerialization._struct_cdr_size(Inner, false) == 9   # CDR1 wire size, no trailing pad

    val = _PadCompact(3.14, 0x42)             # default → CDR1 variant
    @test val isa _PadCompact{Inner}
    @test val.x == 3.14
    @test val.y == 0x42

    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)     # default CDR_LE → CDR1
    CDRSerialization.write_all!(w, val)
    @test position(w) == 4 + 9               # preamble + 9 wire bytes (no trailing pad)

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

# @cdr1_compat: a single flat plain struct, standard-CDR (no trailing pad),
# safe to nest/embed. _C1Pad has trailing pad in Julia (9 wire bytes), _C1Vec
# is naturally compact, _C1Nest/_C1Outer exercise nesting.
@cdr1_compat struct _C1Pad
    x::Float64
    y::UInt8
end
@cdr1_compat struct _C1Vec
    v::SVector{3, Float64}
end
@cdr1_compat struct _C1Nest
    a::_C1Pad
    b::UInt32
end
@cdr1_compat struct _C1Outer
    head::_C1Pad
    tail::UInt16
end

@testset "@cdr1_compat: single concrete type, standard-CDR wire format" begin
    # One plain concrete type — not a {V} variant union.
    @test isconcretetype(_C1Pad)
    @test !(_C1Pad isa UnionAll)
    @test fieldnames(_C1Pad) == (:x, :y)

    val = _C1Pad(3.14, 0x42)
    @test val.x == 3.14 && val.y == 0x42
    @test propertynames(val) == (:x, :y)

    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, val)
    @test position(w) == 4 + 9                          # 9 wire bytes, no trailing pad
    @test bytes2hex(collect(CDRSerialization.data(w)[5:end])) == "1f85eb51b81e094042"  # 3.14 LE + 0x42

    seekstart(data)
    got = read(CDRSerialization.CDRReader(data), _C1Pad)
    @test got == val

    # Single-value `write` fallback forwards to write_all!, returns byte count.
    d3 = IOBuffer(); w3 = CDRSerialization.CDRWriter(d3)
    @test write(w3, val) == 9
    @test collect(CDRSerialization.data(w3)) == collect(CDRSerialization.data(w))

    # Identical wire bytes to @cdr_compact's CDR1 variant of the same fields.
    d2 = IOBuffer(); CDRSerialization.write_all!(CDRSerialization.CDRWriter(d2), _PadCompact(3.14, 0x42))
    @test collect(CDRSerialization.data(w)) == collect(d2.data[1:position(d2)])
end

@testset "@cdr1_compat: naturally-compact struct takes the fast path" begin
    @test CDRSerialization.iscompact(_C1Vec)            # 24 bytes, no trailing pad
    v = _C1Vec(SVector{3, Float64}(1.0, 2.0, 3.0))
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, v)
    @test position(w) == 4 + 24
    seekstart(data)
    @test read(CDRSerialization.CDRReader(data), _C1Vec) == v
end

@testset "@cdr1_compat: nesting and embedding stay wire-compatible" begin
    nest = _C1Nest(_C1Pad(1.0, 0xAB), UInt32(0x11223344))
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    CDRSerialization.write_all!(w, nest)
    # _C1Pad's 9 data bytes, then b aligns to offset 12 — exactly standard CDR,
    # with the alignment gap zeroed.
    @test bytes2hex(collect(CDRSerialization.data(w)[5:end])) ==
          "000000000000f03fab00000044332211"
    seekstart(data)
    @test read(CDRSerialization.CDRReader(data), _C1Nest) == nest

    outer = _C1Outer(_C1Pad(2.5, 0x07), UInt16(0xBEEF))
    d2 = IOBuffer()
    w2 = CDRSerialization.CDRWriter(d2)
    CDRSerialization.write_all!(w2, outer)
    seekstart(d2)
    @test read(CDRSerialization.CDRReader(d2), _C1Outer) == outer

    # Size calculator agrees with the writer for the nested value.
    calc = CDRSizeCalculator()
    CDRSerialization.addValue!(calc, nest)
    @test position(calc) == position(w)
end

@testset "@cdr1_compat: variable-length fields are rejected" begin
    @test_throws LoadError @eval @cdr1_compat struct _C1Bad
        s::String
    end
    @test_throws LoadError @eval @cdr1_compat struct _C1BadVec
        v::Vector{Float64}
    end
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

# A flat-but-non-compact struct: UInt16, then Float64 (8-aligned), then UInt16.
# Julia pads it to 24 bytes; standard CDR is 18 with the field at offset 0.
struct _AwkwardMsg
    a::UInt16
    b::Float64
    c::UInt16
end
Base.:(==)(x::_AwkwardMsg, y::_AwkwardMsg) = x.a == y.a && x.b == y.b && x.c == y.c

@testset "generic write(c, struct) is the inverse of read(r, T)" begin
    # Owned struct write — no write_all!, no hand-rolled field statements.
    pose = _TestPose(_TestPoint(1.0, 2.0, 3.0), _TestQuat(0.1, 0.2, 0.3, 0.4))
    named = _TestNamedPose("base_link", pose)
    for (v, T) in ((pose, _TestPose), (named, _TestNamedPose))
        data = IOBuffer()
        w = CDRSerialization.CDRWriter(data)
        n = write(w, v)
        @test n == position(w) - 4          # byte count includes padding
        seekstart(data)
        @test read(CDRSerialization.CDRReader(data), T) == v
    end

    # Generic struct write produces the same bytes as write_all! when the value
    # starts at the (max-aligned) message origin.
    d1 = IOBuffer(); CDRSerialization.write_all!(CDRSerialization.CDRWriter(d1), named)
    d2 = IOBuffer(); write(CDRSerialization.CDRWriter(d2), named)
    @test take!(d1) == take!(d2)
end

@testset "generic write aligns each field independently (no over-pad)" begin
    # Written right after a string the struct lands at a 4-aligned (not
    # 8-aligned) offset. The field-walk writer pads `b` to the next 8 *within*
    # the struct — exactly what the field-walk reader expects. (A single
    # write_all! packed run would instead pad the whole struct to 8 up front,
    # desyncing the reader — the latent bug this path avoids.)
    msg = _AwkwardMsg(0x1111, 3.14, 0x2222)
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, "hi")        # leaves the cursor at a non-8-aligned offset
    write(w, msg)
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, String) == "hi"
    @test read(r, _AwkwardMsg) == msg

    # Size calculator agrees with the field-walk writer for the whole message.
    calc = CDRSizeCalculator()
    CDRSerialization.addValue!(calc, "hi")
    CDRSerialization.addValue!(calc, msg)
    @test position(calc) == position(w)
end

# A struct leading with a smaller-aligned field than its max. Its CDR data size
# equals its Julia sizeof (no trailing pad), so the old compactness test would
# have single-blob it — but the blob's internal padding is baked for an
# 8-aligned base, so it can't sit where a field-walk reader puts it at a
# 2-aligned offset. It must field-walk; `iscompact` must report that.
struct _LeadSmall
    a::UInt16
    b::Float64
end
Base.:(==)(x::_LeadSmall, y::_LeadSmall) = x.a == y.a && x.b == y.b

@testset "compact fast path is restricted to standard-CDR-safe structs" begin
    # Not single-blob compact, despite sizeof == cdr-data-size, because it leads
    # with a 2-aligned field under an 8-aligned struct.
    @test !CDRSerialization.iscompact(_LeadSmall)
    @test CDRSerialization._struct_cdr_size(_LeadSmall, false) == sizeof(_LeadSmall)

    # Reading hand-written STANDARD-CDR bytes (a at +2, b at +8) matches a
    # field-walk reader — the blob path would have skipped a to +8.
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, UInt16(0x1234))     # prefix @0
    write(w, UInt16(0xAAAA))     # _LeadSmall.a @2 (standard placement)
    write(w, Float64(3.14))      # _LeadSmall.b @8
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, UInt16) == 0x1234
    @test read(r, _LeadSmall) == _LeadSmall(0xAAAA, 3.14)

    # And our own generic write lands it at the standard offsets (a@2, b@8).
    d2 = IOBuffer()
    w2 = CDRSerialization.CDRWriter(d2)
    write(w2, UInt16(0x1234))
    write(w2, _LeadSmall(0xAAAA, 3.14))
    @test position(w2) == 4 + 16     # u16@0, a@2, b@8, ends @16
    seekstart(d2)
    r2 = CDRSerialization.CDRReader(d2)
    @test read(r2, UInt16) == 0x1234
    @test read(r2, _LeadSmall) == _LeadSmall(0xAAAA, 3.14)

    # A struct that leads with its max-aligned field is still single-blob.
    @test CDRSerialization.iscompact(_TestPoint)   # {Float64,Float64,Float64}
end

@testset "generic write(c, Vector{Struct}) length-prefixes (inverse of read)" begin
    poses = [_TestPose(_TestPoint(1.0, 2.0, 3.0), _TestQuat(0.0, 0.0, 0.0, 1.0)),
             _TestPose(_TestPoint(4.0, 5.0, 6.0), _TestQuat(0.7, 0.0, 0.7, 0.0))]
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, poses)                          # default: writes u32 length prefix
    seekstart(data)
    r = CDRSerialization.CDRReader(data)
    @test read(r, Vector{_TestPose}) == poses

    # SVector of structs: fixed length, no prefix (inverse of the SArray read).
    triplet = SVector(poses[1], poses[2], poses[1])
    d2 = IOBuffer()
    w2 = CDRSerialization.CDRWriter(d2)
    write(w2, triplet)
    seekstart(d2)
    @test read(CDRSerialization.CDRReader(d2), typeof(triplet)) == triplet
end

@testset "write_all! packed runs stay field-walk-correct at non-max-aligned offsets" begin
    # A run is aligned to its widest member, but the reader aligns the run's
    # first leaf to its own alignment. When a run leads with a narrower-aligned
    # leaf and starts off a max-aligned boundary (e.g. right after a string),
    # the two must still agree — the grouping splits the run so they do.
    rt(args, reads) = begin
        data = IOBuffer(); CDRSerialization.write_all!(CDRSerialization.CDRWriter(data), args...)
        seekstart(data); r = CDRSerialization.CDRReader(data)
        Tuple(rd(r) for rd in reads)
    end
    @test rt((UInt16(0xAAAA), 3.14),
             (r->read(r,UInt16), r->read(r,Float64))) == (0xAAAA, 3.14)
    @test rt(("s", UInt16(0xAAAA), 3.14),
             (r->read(r,String), r->read(r,UInt16), r->read(r,Float64))) == ("s", 0xAAAA, 3.14)
    @test rt(("s", UInt8(1), UInt32(2), UInt16(3)),
             (r->read(r,String), r->read(r,UInt8), r->read(r,UInt32), r->read(r,UInt16))) == ("s", 0x01, UInt32(2), 0x0003)

    # write_all! and the generic field-walk write must emit identical bytes.
    d1 = IOBuffer(); CDRSerialization.write_all!(CDRSerialization.CDRWriter(d1), "s", _LeadSmall(0xAAAA, 3.14))
    d2 = IOBuffer(); w = CDRSerialization.CDRWriter(d2); write(w, "s"); write(w, _LeadSmall(0xAAAA, 3.14))
    @test take!(d1) == take!(d2)

    # Vector of a non-compact (lead-with-small) struct round-trips through
    # write_all! + the field-walk reader.
    msgs = [_LeadSmall(0x1111, 1.0), _LeadSmall(0x2222, 2.0), _LeadSmall(0x3333, 3.0)]
    data = IOBuffer(); CDRSerialization.write_all!(CDRSerialization.CDRWriter(data), msgs)
    seekstart(data)
    @test read(CDRSerialization.CDRReader(data), Vector{_LeadSmall}) == msgs
end

@testset "cdr_layout surfaces the capability tier + reason" begin
    # compact (leads with max-align field, no trailing pad): all capabilities.
    lc = cdr_layout(_TestPoint)
    @test lc.fixed_size && lc.single_op && lc.viewable
    @test lc isa CDRLayout

    # flat but trailing pad → fixed-size, but not single-op / viewable.
    lp = cdr_layout(_C1Pad)                       # {Float64; UInt8}, from earlier testset
    @test lp.fixed_size && !lp.single_op && !lp.viewable
    @test occursin("trailing padding", lp.why)

    # flat but leads with a narrow field → fixed-size, not single-op / viewable.
    ln = cdr_layout(_LeadSmall)                    # {UInt16; Float64}
    @test ln.fixed_size && !ln.single_op && !ln.viewable
    @test occursin("leads with", ln.why)

    # dynamic (has a String/Vector) → none of the three.
    ld = cdr_layout(_TestInner)                    # {String; Vector{Float64}}
    @test !ld.fixed_size && !ld.single_op && !ld.viewable
    @test occursin("variable-length", ld.why)

    # the nesting holds: viewable ⟹ single_op ⟹ fixed_size for each.
    for T in (_TestPoint, _C1Pad, _LeadSmall, _TestInner, Float64)
        l = cdr_layout(T)
        @test !l.viewable || l.single_op
        @test !l.single_op || l.fixed_size
    end

    # cdr_layout agrees with the iscompact predicate.
    @test cdr_layout(_TestPoint).single_op == CDRSerialization.iscompact(_TestPoint)
    @test cdr_layout(_LeadSmall).single_op == CDRSerialization.iscompact(_LeadSmall)
end

@testset "view error message explains why and points to the owned read" begin
    mem = Memory{UInt8}(undef, 128)
    CDRSerialization.write_all!(CDRSerialization.CDRWriter(mem), [_LeadSmall(0x1, 1.0)])
    r = CDRSerialization.CDRReader(mem)
    err = try; view(r, CDRArray{_LeadSmall}); catch e; e; end
    @test err isa ArgumentError
    @test occursin("leads with", err.msg)            # the reason
    @test occursin("read(r, Vector{", err.msg)       # the suggested alternative
end

@testset "CDRArray view of a @cdr1_compat element: only when compact" begin
    # _LeadSmall is standard-CDR1 but NOT compact (leads with UInt16 < max 8):
    # its sequence elements aren't uniformly strided, so they can't be aliased.
    mem = Memory{UInt8}(undef, 256)
    CDRSerialization.write_all!(CDRSerialization.CDRWriter(mem),
                               [_LeadSmall(0x1111, 1.0), _LeadSmall(0x2222, 2.0)])
    r = CDRSerialization.CDRReader(mem)
    @test !CDRSerialization.canview(r, CDRArray{_LeadSmall})
    @test_throws ArgumentError view(r, CDRArray{_LeadSmall})
    # The owned read still works.
    @test read(CDRSerialization.CDRReader(mem), Vector{_LeadSmall}) ==
          [_LeadSmall(0x1111, 1.0), _LeadSmall(0x2222, 2.0)]

    # A compact element (leads with max-align field) IS viewable.
    mem2 = Memory{UInt8}(undef, 256)
    pts = [_TestPoint(1.0, 2.0, 3.0), _TestPoint(4.0, 5.0, 6.0)]
    CDRSerialization.write_all!(CDRSerialization.CDRWriter(mem2), pts)
    r2 = CDRSerialization.CDRReader(mem2)
    @test CDRSerialization.canview(r2, CDRArray{_TestPoint})
    @test view(r2, CDRArray{_TestPoint}) == pts
end

@testset "write(c, vector) default length-prefix is symmetric with read" begin
    # A bare `write(c, vec)` (no explicit writeLength) writes the u32 length
    # prefix, exactly as `read(r, Vector{T})` consumes it and the size
    # calculator counts it — for every dynamic sequence element type.
    for v in (Float64[1.0, 2.0, 3.0], UInt8[1, 2, 3], Int16[-1, 2, -3],
              ["foo", "barbaz"], [_TestPoint(1.0, 2.0, 3.0), _TestPoint(4.0, 5.0, 6.0)])
        data = IOBuffer()
        w = CDRSerialization.CDRWriter(data)
        write(w, v)                                   # default → writes prefix
        seekstart(data)
        r = CDRSerialization.CDRReader(data)
        @test read(r, Vector{eltype(v)}) == v

        calc = CDRSizeCalculator()
        CDRSerialization.addValue!(calc, v)           # default → counts prefix
        @test position(calc) == position(w)
    end

    # A fixed-length SArray still carries NO prefix by default (both ends agree).
    data = IOBuffer()
    w = CDRSerialization.CDRWriter(data)
    write(w, SVector(1.0, 2.0, 3.0))
    seekstart(data)
    @test read(CDRSerialization.CDRReader(data), SVector{3, Float64}) == [1.0, 2.0, 3.0]
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

