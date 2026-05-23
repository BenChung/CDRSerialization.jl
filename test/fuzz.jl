using Random

const FUZZ_PRIMS = (Int8, UInt8, Bool, Int16, UInt16, Int32, UInt32, Float32, Int64, UInt64, Float64)

randprim(rng, ::Type{T}) where T <: Union{Int8, UInt8, Int16, UInt16, Int32, UInt32, Int64, UInt64} = rand(rng, T)
randprim(rng, ::Type{Bool}) = rand(rng, Bool)
# Use bit-pattern-random floats so we exercise denormals/NaN/Inf as well.
randprim(rng, ::Type{Float32}) = reinterpret(Float32, rand(rng, UInt32))
randprim(rng, ::Type{Float64}) = reinterpret(Float64, rand(rng, UInt64))
randprim(rng, ::Type{Char}) = Char(rand(rng, UInt8(0x20):UInt8(0x7E)))

# Bit-level equality so NaN round-trips compare true if the bit pattern matches.
bit_equal(a::Float32, b::Float32) = reinterpret(UInt32, a) == reinterpret(UInt32, b)
bit_equal(a::Float64, b::Float64) = reinterpret(UInt64, a) == reinterpret(UInt64, b)
bit_equal(a, b) = a == b
function bit_equal(a::AbstractArray, b::AbstractArray)
    length(a) == length(b) || return false
    for (x, y) in zip(a, b)
        bit_equal(x, y) || return false
    end
    return true
end

function randstring_safe(rng, maxlen=20)
    n = rand(rng, 0:maxlen)
    pool_ascii = collect(UInt32(0x21):UInt32(0x7E))   # printable ASCII (no space, no NUL)
    pool_multi = UInt32[0x00E9, 0x00F1, 0x2603, 0x4E2D, 0x1F600]
    cps = UInt32[]
    for _ in 1:n
        push!(cps, rand(rng) < 0.7 ? rand(rng, pool_ascii) : rand(rng, pool_multi))
    end
    return String([Char(c) for c in cps])
end

struct FuzzField
    descr::String
    writer::Function
    reader::Function
    value::Any
end

function gen_field(rng)
    op = rand(rng, 1:7)
    if op == 1
        T = rand(rng, FUZZ_PRIMS)
        v = randprim(rng, T)
        return FuzzField("prim{$T}", w -> write(w, v), r -> read(r, T), v)
    elseif op == 2
        v = randprim(rng, Char)
        return FuzzField("Char", w -> write(w, v), r -> read(r, Char), v)
    elseif op == 3
        v = randstring_safe(rng)
        return FuzzField("String($(sizeof(v)) bytes)", w -> write(w, v), r -> read(r, String), v)
    elseif op == 4
        T = rand(rng, FUZZ_PRIMS)
        n = rand(rng, 0:10)
        v = T[randprim(rng, T) for _ in 1:n]
        return FuzzField("Vector{$T}($n)", w -> write(w, v, true), r -> read(r, Vector{T}), v)
    elseif op == 5
        T = rand(rng, FUZZ_PRIMS)
        D = rand(rng, 1:5)
        v = SVector{D, T}(ntuple(_ -> randprim(rng, T), D))
        return FuzzField("SVector{$D,$T}", w -> write(w, v), r -> read(r, SVector{D, T}), v)
    elseif op == 6
        T = rand(rng, FUZZ_PRIMS)
        R = rand(rng, 1:4); C = rand(rng, 1:4); L = R * C
        SAtype = SArray{Tuple{R, C}, T, 2, L}
        v = SAtype(ntuple(_ -> randprim(rng, T), L))
        return FuzzField("SMatrix{$R,$C,$T}", w -> write(w, v), r -> read(r, SAtype), v)
    else
        n = rand(rng, 0:5)
        v = String[randstring_safe(rng, 10) for _ in 1:n]
        return FuzzField("Vector{String}($n)", w -> write(w, v, true), r -> read(r, Vector{String}), v)
    end
end

const FUZZ_KINDS = (CDRSerialization.CDR_LE, CDRSerialization.CDR_BE)

function fuzz_one(seed::UInt)
    rng = MersenneTwister(seed)
    n_fields = rand(rng, 1:8)
    fields = FuzzField[gen_field(rng) for _ in 1:n_fields]
    kind = rand(rng, FUZZ_KINDS)

    buf = IOBuffer()
    w = CDRSerialization.CDRWriter(buf, kind)
    write_err = nothing
    try
        for f in fields
            f.writer(w)
        end
    catch e
        write_err = e
    end
    if write_err !== nothing
        return (ok=false, phase=:write, err=write_err, fields=fields, kind=kind, field_index=0, got=nothing)
    end

    seekstart(buf)
    r = CDRSerialization.CDRReader(buf)
    for (i, f) in enumerate(fields)
        got = try
            f.reader(r)
        catch e
            return (ok=false, phase=:read, err=e, fields=fields, kind=kind, field_index=i, got=nothing)
        end
        if !bit_equal(got, f.value)
            return (ok=false, phase=:compare, err=nothing, fields=fields, kind=kind, field_index=i, got=got)
        end
    end
    return (ok=true, phase=:done, err=nothing, fields=fields, kind=kind, field_index=0, got=nothing)
end

@testset "fuzz writer vs reader" begin
    n_iters = parse(Int, get(ENV, "CDR_FUZZ_ITERS", "300"))
    base_seed = parse(UInt, get(ENV, "CDR_FUZZ_SEED", string(UInt(0xC0FFEE))))
    failures = 0
    for i in 1:n_iters
        seed = base_seed + UInt(i)
        res = fuzz_one(seed)
        if !res.ok
            failures += 1
            schema = join((f.descr for f in res.fields), " | ")
            if res.phase == :compare
                @error "fuzz: value mismatch" seed=seed kind=res.kind field_index=res.field_index field=res.fields[res.field_index].descr expected=res.fields[res.field_index].value got=res.got schema=schema
            else
                @error "fuzz: $(res.phase) error" seed=seed kind=res.kind field_index=res.field_index err=res.err schema=schema
            end
            failures >= 3 && break
        end
    end
    @test failures == 0
end
