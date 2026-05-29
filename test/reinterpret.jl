using CDRSerialization: CDRReader, CDRWriter, write_all!, reinterpret_struct, reinterpret_array,
                        reinterpret_string, read_view, CDRString, canview, iscompact
using CDRSerialization
using StaticArrays
using AllocCheck

struct _RPoint
    x::Float64
    y::Float64
    z::Float64
end
struct _RQuat
    x::Float64; y::Float64; z::Float64; w::Float64
end
struct _RPose
    position::_RPoint
    orientation::_RQuat
end
struct _RHasString    # not isbits → never compact
    a::Int32
    s::String
end

Base.:(==)(a::_RPoint, b::_RPoint) = a.x == b.x && a.y == b.y && a.z == b.z
Base.:(==)(a::_RQuat, b::_RQuat) = a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w
Base.:(==)(a::_RPose, b::_RPose) = a.position == b.position && a.orientation == b.orientation

@testset "compact struct: read single-loads + iscompact reports it" begin
    mem = Memory{UInt8}(undef, 128)
    w = CDRWriter(mem)
    pose = _RPose(_RPoint(1.0, 2.0, 3.0), _RQuat(0.1, 0.2, 0.3, 0.4))
    write_all!(w, pose, UInt32(0xCAFE))

    r = CDRReader(mem)
    @test iscompact(r, _RPose)            # read(r, _RPose) is a single unsafe_load
    @test read(r, _RPose) == pose
    @test read(r, UInt32) == 0xCAFE       # cursor advanced exactly past the struct

    # The reader-less form agrees (host endianness, CDR1).
    @test iscompact(_RPose)
    @test !iscompact(_RHasString)
end

@testset "reinterpret_struct: standalone over Memory + Vector" begin
    # Lay a struct down with the writer, then reinterpret the raw payload.
    mem = Memory{UInt8}(undef, 64)
    w = CDRWriter(mem)
    write_all!(w, _RPoint(1.5, 2.5, 3.5))
    # payload starts at byte offset 4 (after the 4-byte preamble)
    @test reinterpret_struct(mem, _RPoint, 4) == _RPoint(1.5, 2.5, 3.5)

    vec = Vector{UInt8}(undef, 64)
    wv = CDRWriter(vec)
    write_all!(wv, _RPoint(-1.0, -2.0, -3.0))
    @test reinterpret_struct(vec, _RPoint, 4) == _RPoint(-1.0, -2.0, -3.0)
end

@testset "reinterpret_struct: rejects non-compact + out of bounds" begin
    buf = Vector{UInt8}(undef, 64)
    @test_throws ArgumentError reinterpret_struct(buf, _RHasString, 0)
    @test_throws BoundsError reinterpret_struct(buf, _RPoint, 60)
    @test_throws BoundsError reinterpret_struct(buf, _RPoint, -1)
end

@testset "reinterpret_array: length-prefixed sequence as a view" begin
    poses = [_RPoint(1.0, 2.0, 3.0), _RPoint(4.0, 5.0, 6.0), _RPoint(7.0, 8.0, 9.0)]
    mem = Memory{UInt8}(undef, 256)
    w = CDRWriter(mem)
    write_all!(w, poses)              # u32 length prefix + 3 packed structs

    r = CDRReader(mem)
    av = view(r, CDRArray{_RPoint})
    @test av isa CDRArray{_RPoint}
    @test av isa AbstractVector{_RPoint}
    @test length(av) == 3
    @test size(av) == (3,)
    @test av[2] == _RPoint(4.0, 5.0, 6.0)
    # Usable directly as an array — no collect needed.
    @test [p.x for p in av] == [1.0, 4.0, 7.0]
    @test sum(p -> p.x, av) == 12.0
    # Matches the owned-Vector read exactly.
    @test av == read(CDRReader(mem), Vector{_RPoint})
end

@testset "reinterpret_array: aliases the buffer, read + write through" begin
    poses = [_RPoint(1.0, 2.0, 3.0), _RPoint(4.0, 5.0, 6.0)]
    mem = Memory{UInt8}(undef, 128)
    write_all!(CDRWriter(mem), poses)

    av = view(CDRReader(mem), CDRArray{_RPoint})
    # Store an element back in place; a fresh view over the same bytes sees it.
    av[1] = _RPoint(-9.0, -8.0, -7.0)
    av2 = view(CDRReader(mem), CDRArray{_RPoint})
    @test av2[1] == _RPoint(-9.0, -8.0, -7.0)
    @test av2[2] == _RPoint(4.0, 5.0, 6.0)
    # And the plain reader path agrees.
    @test read(CDRReader(mem), Vector{_RPoint})[1] == _RPoint(-9.0, -8.0, -7.0)
end

@testset "view CDRArray: fixed length (no prefix) via internal _view_array" begin
    poses = [_RPoint(1.0, 2.0, 3.0), _RPoint(4.0, 5.0, 6.0), _RPoint(7.0, 8.0, 9.0)]
    mem = Memory{UInt8}(undef, 256)
    w = CDRWriter(mem)
    write_all!(w, SVector(poses...), UInt32(0xBEEF))   # SArray of struct → no length prefix

    r = CDRReader(mem)
    av = CDRSerialization._view_array(r, _RPoint; num=3)   # no-prefix path (internal)
    @test av == poses
    @test read(r, UInt32) == 0xBEEF   # cursor advanced past the elements
end

@testset "view CDRArray: empty sequence" begin
    mem = Memory{UInt8}(undef, 64)
    write_all!(CDRWriter(mem), _RPoint[])
    av = view(CDRReader(mem), CDRArray{_RPoint})
    @test length(av) == 0
    @test collect(av) == _RPoint[]
end

@testset "view CDRArray: standalone reinterpret + rejects non-compact / OOB" begin
    poses = [_RPoint(-1.0, -2.0, -3.0), _RPoint(0.0, 0.0, 0.0)]
    vec = Vector{UInt8}(undef, 128)
    write_all!(CDRWriter(vec), SVector(poses...))   # no prefix → elements at offset 4
    @test reinterpret_array(vec, _RPoint, 4, 2) == poses

    buf = Vector{UInt8}(undef, 64)
    @test_throws ArgumentError reinterpret_array(buf, _RHasString, 0, 1)
    @test_throws BoundsError reinterpret_array(buf, _RPoint, 0, 10)
    # view is strict: a non-compact element can't be aliased → error.
    @test_throws ArgumentError view(CDRReader(Memory{UInt8}(undef, 64)), CDRArray{_RHasString})
end

struct _RFrame
    id::UInt32
    poses::Vector{_RPoint}
    samples::Vector{Float64}
    timestamp::Float64
end

struct _ROuter
    name::Vector{UInt8}          # treat as a byte sequence
    frame::_RFrame               # nested struct containing sequences
    tail::UInt16
end

struct _RWithStrings
    n::UInt32
    labels::Vector{String}       # element type can't be aliased
end

struct _RNamed
    name::String                 # → CDRString view
    id::UInt32
    samples::Vector{Float64}     # → CDRArray view
end

@testset "read_view: struct with sequence fields → CDRArrays" begin
    frame = _RFrame(7, [_RPoint(1.0, 2.0, 3.0), _RPoint(4.0, 5.0, 6.0)],
                    [10.0, 20.0, 30.0], 99.5)
    mem = Memory{UInt8}(undef, 512)
    write_all!(CDRWriter(mem), frame)

    v = read_view(CDRReader(mem), _RFrame)
    @test v.id == 7
    @test v.poses isa CDRArray{_RPoint}
    @test v.samples isa CDRArray{Float64}
    @test v.poses == frame.poses          # value-equal without ever copying
    @test v.samples == frame.samples
    @test v.timestamp == 99.5

    # Scalars decode identically to the field-by-field reader; arrays match too.
    ref = read(CDRReader(mem), _RFrame)
    @test v.id == ref.id && v.timestamp == ref.timestamp
    @test collect(v.poses) == ref.poses && collect(v.samples) == ref.samples
end

@testset "read_view: nested struct recurses to nested view" begin
    frame = _RFrame(3, [_RPoint(1.0, 2.0, 3.0)], Float64[], 1.0)
    outer = _ROuter(UInt8[0x61, 0x62, 0x63], frame, 0xBEEF)
    mem = Memory{UInt8}(undef, 512)
    write_all!(CDRWriter(mem), outer)

    v = read_view(CDRReader(mem), _ROuter)
    @test v.name isa CDRArray{UInt8}
    @test collect(v.name) == UInt8[0x61, 0x62, 0x63]
    @test v.frame isa NamedTuple              # nested view
    @test v.frame.id == 3
    @test v.frame.poses isa CDRArray{_RPoint}
    @test v.frame.poses[1] == _RPoint(1.0, 2.0, 3.0)
    @test length(v.frame.samples) == 0
    @test v.tail == 0xBEEF
end

@testset "read_view: non-aliasable sequence falls back to decode" begin
    # Vector{String} elements can't be aliased → decoded to a normal Vector.
    val = _RWithStrings(5, ["foo", "bar"])
    mem = Memory{UInt8}(undef, 256)
    write_all!(CDRWriter(mem), val)

    v = read_view(CDRReader(mem), _RWithStrings)
    @test v.n == 5
    @test v.labels isa Vector{String}
    @test v.labels == ["foo", "bar"]
end

@testset "reinterpret_string: zero-copy UTF-8 view" begin
    str = "héllo wörld 你好"
    mem = Memory{UInt8}(undef, 128)
    w = CDRWriter(mem)
    write(w, str)
    write(w, UInt8(42))

    r = CDRReader(mem)
    s = view(r, CDRString)
    @test s isa CDRString
    @test s == str                         # value-equal, no copy
    @test length(s) == length(str)         # char count (multibyte aware)
    @test ncodeunits(s) == ncodeunits(str)
    @test collect(s) == collect(str)       # UTF-8 iteration
    @test String(s) == str                 # materialise an owned copy
    @test read(r, UInt8) == 42             # cursor advanced past content + null
end

@testset "reinterpret_string: empty + standalone" begin
    mem = Memory{UInt8}(undef, 64)
    w = CDRWriter(mem)
    write(w, "")
    write(w, UInt8(7))
    r = CDRReader(mem)
    s = view(r, CDRString)
    @test s == ""
    @test ncodeunits(s) == 0
    @test read(r, UInt8) == 7

    # standalone over raw content bytes
    buf = Vector{UInt8}(codeunits("abcdef"))
    @test reinterpret_string(buf, 1, 3) == "bcd"
    @test_throws BoundsError reinterpret_string(buf, 4, 5)
end

@testset "read_view: String field becomes a CDRString view" begin
    val = _RNamed("robot", 7, [1.0, 2.0, 3.0])
    mem = Memory{UInt8}(undef, 256)
    write_all!(CDRWriter(mem), val)

    v = read_view(CDRReader(mem), _RNamed)
    @test v.name isa CDRString
    @test v.name == "robot"
    @test v.id == 7
    @test v.samples isa CDRArray{Float64}
    @test collect(v.samples) == [1.0, 2.0, 3.0]
end

# @cdr_compact opt-in view mode: only fields declared CDRString / CDRArray{T}
# become zero-copy views; plain fields are read as owned values.
@cdr_compact struct _RFrameView
    name::CDRString                     # opt-in view
    id::UInt32                          # value
    poses::CDRArray{SVector{3, Float64}} # opt-in view
    owned::Vector{Float64}              # plain → owned, NOT a view
    flag::UInt8
end

# plain twin to author the wire bytes
struct _RFramePlain
    name::String
    id::UInt32
    poses::Vector{SVector{3, Float64}}
    owned::Vector{Float64}
    flag::UInt8
end

@testset "@cdr_compact: opt-in CDRString/CDRArray fields → view struct" begin
    val = _RFramePlain("robot", 7,
                       [SVector(1.0, 2.0, 3.0), SVector(4.0, 5.0, 6.0)],
                       [10.0, 20.0, 30.0], 0x09)
    mem = Memory{UInt8}(undef, 512)
    write_all!(CDRWriter(mem), val)

    v = read(CDRReader(mem), _RFrameView)
    @test v isa _RFrameView{Memory{UInt8}}
    @test v.name isa CDRString && v.name == "robot"
    @test v.id == 7
    @test v.poses isa CDRArray{SVector{3, Float64}}
    @test v.poses[2] == SVector(4.0, 5.0, 6.0)
    @test v.owned isa Vector{Float64}        # plain field is owned, not a view
    @test v.owned == [10.0, 20.0, 30.0]
    @test v.flag == 0x09
    @test propertynames(v) == (:name, :id, :poses, :owned, :flag)
    @test v == read(CDRReader(mem), _RFrameView)
    @test occursin("robot", repr(v))

    # `view(r, ViewStruct)` is an alias of `read(r, ViewStruct)`, for
    # consistency with view(r, CDRArray{T}) / view(r, CDRString).
    @test view(CDRReader(mem), _RFrameView) == v
    @test view(CDRReader(mem), _RFrameView) isa _RFrameView
end

# A view struct whose dynamic fields are *all* opt-in views (no owned Vector/
# String), so decoding + using it touches the buffer only — must be
# allocation-free regardless of how large `points` is.
@cdr_compact struct _RCloudView
    seq::UInt32
    points::CDRArray{_RPoint}
    frame::CDRString
end

# Decode then use fields without letting the view escape (the normal pattern).
function _use_cloud(r)
    v = read(r, _RCloudView)
    s = 0.0
    @inbounds for i in eachindex(v.points)
        s += v.points[i].x
    end
    return s + v.seq + ncodeunits(v.frame)
end

@testset "AllocCheck: @cdr_compact view decode of a large array is allocation-free" begin
    RT = CDRReader{CDRSerialization.MemBuf{Memory{UInt8}}, false, true}
    @test isempty(check_allocs(_use_cloud, (RT,)))

    # And it's genuinely O(1) in allocation as the array grows.
    for N in (4, 1024, 65536)
        io = IOBuffer()
        write_all!(CDRWriter(io), UInt32(7),
                   [_RPoint(Float64(i), 2.0, 3.0) for i in 1:N], "frame")
        r = CDRReader(take!(io))
        _use_cloud(r)                       # warmup / compile
        seek(r.src, 4); r.origin = 4
        @test (@allocated _use_cloud(r)) == 0
    end
end

@testset "@cdr_compact: view detection is syntactic + opt-in" begin
    @test CDRSerialization._cdr_has_view_fields(Any[:Float64, :UInt8]) == false
    @test CDRSerialization._cdr_has_view_fields(Any[:(Vector{Float64})]) == false   # plain, not a view
    @test CDRSerialization._cdr_has_view_fields(Any[:CDRString]) == true
    @test CDRSerialization._cdr_has_view_fields(Any[:(CDRArray{Float64})]) == true
    @test CDRSerialization._cdr_view_field_class(:(CDRArray{Int32})) == (:array, :Int32)
    @test CDRSerialization._cdr_view_field_class(:CDRString) == (:string, nothing)
    @test CDRSerialization._cdr_view_field_class(:(Vector{Int32}))[1] == :plain
end

# A plain variable-length field without opting in is rejected (not auto-viewed).
@testset "@cdr_compact: plain Vector field is rejected, not auto-converted" begin
    @test_throws LoadError @eval @cdr_compact struct _RBadCompact
        xs::Vector{Float64}
    end
end

@testset "view: strict zero-copy, errors when impossible" begin
    poses = [_RPoint(1.0, 2.0, 3.0), _RPoint(4.0, 5.0, 6.0)]
    mem = Memory{UInt8}(undef, 256)
    w = CDRWriter(mem)
    write_all!(w, poses)
    write(w, "hi")

    r = CDRReader(mem)
    av = view(r, CDRArray{_RPoint})
    @test av isa CDRArray{_RPoint}
    @test collect(av) == poses
    sv = view(r, CDRString)
    @test sv isa CDRString && sv == "hi"

    # On a big-endian stream, aliasing primitives/structs isn't possible → error.
    bmem = Memory{UInt8}(undef, 256)
    bw = CDRWriter(bmem, CDRSerialization.CDR_BE)
    write_all!(bw, [_RPoint(1.0, 2.0, 3.0)])
    br = CDRReader(bmem)
    @test_throws ArgumentError view(br, CDRArray{_RPoint})
end

# Top-level probe functions (not local closures) so inference/codegen reflect
# real call sites.
_cv_branch(r) = canview(r, CDRArray{_RPoint}) ? 1 : 2
_ic_branch(r) = iscompact(r, _RPose) ? 1 : 2
_pick(r)      = canview(r, CDRArray{_RPoint}) ? view(r, CDRArray{_RPoint}) : read(r, Vector{_RPoint})
function _folds_to_literal(f, RT)
    code = code_typed(f, (RT,); optimize=true)[1].first.code
    length(code) == 1 && code[1] isa Core.ReturnNode && code[1].val isa Int
end

@testset "canview / iscompact: compile-time-foldable predicates" begin
    RT_le = CDRReader{CDRSerialization.MemBuf{Memory{UInt8}}, false, true}
    RT_be = CDRReader{CDRSerialization.MemBuf{Memory{UInt8}}, false, false}

    # A reader over a properly-initialised LE buffer (the writer lays down the
    # CDR_LE preamble; a raw `undef` Memory would carry a garbage preamble).
    le_mem = Memory{UInt8}(undef, 64); CDRWriter(le_mem)
    le_reader() = CDRReader(le_mem)

    # Values agree with reality.
    @test canview(le_reader(), CDRArray{_RPoint}) == true
    @test canview(le_reader(), CDRString) == true
    @test iscompact(le_reader(), _RPose) == true

    # Each predicate folds to a literal — the surrounding branch specializes.
    @test _folds_to_literal(_cv_branch, RT_le)
    @test _folds_to_literal(_cv_branch, RT_be)
    @test _folds_to_literal(_ic_branch, RT_le)

    # The idiomatic fallback is type-stable per reader (dead branch pruned).
    @test Base.return_types(_pick, (RT_le,))[1] == CDRArray{_RPoint, Memory{UInt8}}
    @test Base.return_types(_pick, (RT_be,))[1] == Vector{_RPoint}
end

# AllocCheck: a checked single-load reinterpret must be allocation-free.
_ri_reader(r)        = reinterpret_struct(r, _RPose)
_ri_mem(m)           = reinterpret_struct(m, _RPose, 4)
_ca_index(a, i)      = @inbounds a[i]
_ca_store!(a, v, i)  = @inbounds (a[i] = v)

@testset "AllocCheck: reinterpret_struct + CDRArray indexing" begin
    RT = CDRReader{CDRSerialization.MemBuf{Memory{UInt8}}, false, true}
    @test isempty(check_allocs(_ri_reader, (RT,)))
    @test isempty(check_allocs(_ri_mem, (Memory{UInt8},)))
    @test isempty(check_allocs(_ri_mem, (Vector{UInt8},)))

    for S in (Memory{UInt8}, Vector{UInt8})
        AT = CDRArray{_RPose, S}
        @test isempty(check_allocs(_ca_index, (AT, Int)))
        @test isempty(check_allocs(_ca_store!, (AT, _RPose, Int)))
    end
end
