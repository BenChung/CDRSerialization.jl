# Malformed / hostile-input decode: a wire-supplied length or a truncated buffer
# must not drive an out-of-bounds `unsafe_load` or a giant allocation. Every
# MemBuf-backed read path that dereferences off a wire length is exercised with a
# corrupt length or a short buffer and must raise rather than read past the data.

using CDRSerialization: CDRReader, CDRWriter, write_all!, MemBuf, CDRArrayView, CDRArray
using StaticArrays

# A flat fixed-size but NOT compact element (leads with UInt16 < max align 8):
# its sequence decodes via CDRArrayView (the field-walk path), not a CDRArray alias.
struct _MLead
    a::UInt16
    b::Float64
end
Base.:(==)(x::_MLead, y::_MLead) = x.a == y.a && x.b == y.b

# A compact element (leads with its max-aligned field) → CDRArray-aliasable.
struct _MPoint
    x::Float64
    y::Float64
    z::Float64
end
Base.:(==)(x::_MPoint, y::_MPoint) = x.x == y.x && x.y == y.y && x.z == y.z

# Overwrite a top-level sequence's UInt32 length prefix (4 bytes, immediately
# after the 4-byte encapsulation preamble) with `n`, little-endian.
function _patch_seq_length!(mem, n::UInt32)
    mem[5] = UInt8(n & 0xFF)
    mem[6] = UInt8((n >> 8) & 0xFF)
    mem[7] = UInt8((n >> 16) & 0xFF)
    mem[8] = UInt8((n >> 24) & 0xFF)
    return mem
end

@testset "CDRArrayView: corrupt wire length is rejected, not OOB-read" begin
    leads = [_MLead(UInt16(i), Float64(i) * 1.5) for i in 1:6]
    mem = Memory{UInt8}(undef, 512)
    write_all!(CDRWriter(mem), leads)

    # A bogus enormous element count must not drive the view past the buffer.
    _patch_seq_length!(mem, 0xFFFFFFFF)
    @test_throws Union{BoundsError, EOFError} view(CDRReader(mem), CDRArrayView{_MLead})

    # A count that overruns by only a few elements is rejected just the same.
    _patch_seq_length!(mem, UInt32(6 + 10_000))
    @test_throws Union{BoundsError, EOFError} view(CDRReader(mem), CDRArrayView{_MLead})

    # The honest length still decodes correctly (guard does not perturb it).
    _patch_seq_length!(mem, UInt32(6))
    @test view(CDRReader(mem), CDRArrayView{_MLead}) == leads
end

@testset "CDRArrayView: truncated buffer is rejected mid-element" begin
    leads = [_MLead(UInt16(i), Float64(i)) for i in 1:6]
    mem = Memory{UInt8}(undef, 512)
    write_all!(CDRWriter(mem), leads)

    # Honest length, but the data watermark stops partway through the sequence:
    # the trial reads / span check must catch the short buffer.
    for short in (12, 24, 40)
        r = CDRReader(MemBuf(mem, 1, short))
        @test_throws Union{BoundsError, EOFError} view(r, CDRArrayView{_MLead})
    end
end

@testset "CDRArray (compact sibling): corrupt length already guards — control" begin
    pts = [_MPoint(1.0, 2.0, 3.0), _MPoint(4.0, 5.0, 6.0)]
    mem = Memory{UInt8}(undef, 512)
    write_all!(CDRWriter(mem), pts)

    _patch_seq_length!(mem, 0xFFFFFFFF)
    @test_throws BoundsError view(CDRReader(mem), CDRArray{_MPoint})

    # Honest length still aliases correctly.
    _patch_seq_length!(mem, UInt32(2))
    @test view(CDRReader(mem), CDRArray{_MPoint}) == pts
end

@testset "scalar / SArray MemBuf read: truncated buffer is rejected" begin
    # Lay down a scalar then an SArray; truncating the watermark before each
    # field's bytes must raise instead of loading past the data.
    mem = Memory{UInt8}(undef, 128)
    write_all!(CDRWriter(mem), UInt64(0x0102030405060708), SVector{4, Float64}(1, 2, 3, 4))

    # Scalar UInt64 occupies bytes [5, 12] (1-based): preamble(4), already 8-aligned.
    r = CDRReader(MemBuf(mem, 1, 8))       # watermark stops inside the UInt64
    @test_throws Union{BoundsError, EOFError} read(r, UInt64)

    # SArray bulk load past the watermark.
    r2 = CDRReader(MemBuf(mem, 1, 16))     # past the scalar but short of the SArray
    read(r2, UInt64)                       # ok (bytes [5, 12])
    @test_throws Union{BoundsError, EOFError} read(r2, SVector{4, Float64})

    # Full buffer reads both back.
    r3 = CDRReader(mem)
    @test read(r3, UInt64) == 0x0102030405060708
    @test read(r3, SVector{4, Float64}) == SVector{4, Float64}(1, 2, 3, 4)
end

@testset "owned read(r, Vector{E}): bogus length doesn't over-allocate" begin
    leads = [_MLead(UInt16(i), Float64(i)) for i in 1:4]
    mem = Memory{UInt8}(undef, 256)
    write_all!(CDRWriter(mem), leads)

    # 0xFFFFFFFF elements can't fit in the buffer — rejected before allocating.
    _patch_seq_length!(mem, 0xFFFFFFFF)
    @test_throws BoundsError read(CDRReader(mem), Vector{_MLead})

    # Honest length round-trips.
    _patch_seq_length!(mem, UInt32(4))
    @test read(CDRReader(mem), Vector{_MLead}) == leads
end
