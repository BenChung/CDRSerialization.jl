#!/usr/bin/env julia
# Reads test cases on stdin (emitted by gen.js) and verifies each one
# round-trips through CDRSerialization.jl's CDRReader.
#
# Exits 0 on full success, 1 if any case mismatches.

using CDRSerialization

# Parse a single case from `io`, returning a Dict from tag => Vector{SubString}
# of the rest of the tokens on that line, or `nothing` at EOF.
function parse_case(io)
    fields = Dict{String, Vector{SubString{String}}}()
    saw_any = false
    while !eof(io)
        line = strip(readline(io))
        isempty(line) && continue
        saw_any = true
        if line == "END"
            return fields
        end
        toks = split(line, ' ')
        fields[String(toks[1])] = length(toks) >= 2 ? toks[2:end] : SubString{String}[]
    end
    return saw_any ? fields : nothing
end

bit_equal_f32(a, b) = reinterpret(UInt32, Float32(a)) == reinterpret(UInt32, Float32(b))
bit_equal_f64(a, b) = reinterpret(UInt64, Float64(a)) == reinterpret(UInt64, Float64(b))

f32_from_hex(s) = reinterpret(Float32, parse(UInt32, s; base=16))
f64_from_hex(s) = reinterpret(Float64, parse(UInt64, s; base=16))

function check(label, got, expected)
    if got != expected
        error("$(label) mismatch: got=$got expected=$expected")
    end
end

function check_f32(label, got, expected)
    if !bit_equal_f32(got, expected)
        error("$(label) bit mismatch: got=0x$(string(reinterpret(UInt32, got), base=16, pad=8)) expected=0x$(string(reinterpret(UInt32, expected), base=16, pad=8))")
    end
end

function check_f64(label, got, expected)
    if !bit_equal_f64(got, expected)
        error("$(label) bit mismatch: got=0x$(string(reinterpret(UInt64, got), base=16, pad=16)) expected=0x$(string(reinterpret(UInt64, expected), base=16, pad=16))")
    end
end

function verify(c::Dict{String, Vector{SubString{String}}})
    hex = String(c["HEX"][1])
    bytes = hex2bytes(hex)
    r = CDRSerialization.CDRReader(IOBuffer(bytes))

    check("U8",   read(r, UInt8),  parse(UInt8,  c["U8"][1]))
    check("I8",   read(r, Int8),   parse(Int8,   c["I8"][1]))
    check("BOOL", read(r, Bool),   parse(Int, c["BOOL"][1]) != 0)
    check("U16",  read(r, UInt16), parse(UInt16, c["U16"][1]))
    check("I16",  read(r, Int16),  parse(Int16,  c["I16"][1]))
    check("U32",  read(r, UInt32), parse(UInt32, c["U32"][1]))
    check("I32",  read(r, Int32),  parse(Int32,  c["I32"][1]))
    check("U64",  read(r, UInt64), parse(UInt64, c["U64"][1]))
    check("I64",  read(r, Int64),  parse(Int64,  c["I64"][1]))

    check_f32("F32", read(r, Float32), f32_from_hex(c["F32"][1]))
    check_f64("F64", read(r, Float64), f64_from_hex(c["F64"][1]))

    # "-" is the sentinel for empty hex.
    decode_hex(tok) = tok == "-" ? UInt8[] : hex2bytes(String(tok))

    # STR: count hex
    expected_str = String(decode_hex(c["STR"][2]))
    got_str = read(r, String)
    check("STR", got_str, expected_str)

    # VU8: count hex
    expected_vu8 = decode_hex(c["VU8"][2])
    got_vu8 = read(r, Vector{UInt8})
    check("VU8", got_vu8, expected_vu8)

    # VF64: count hex hex ...
    vf64_n = parse(Int, c["VF64"][1])
    expected_vf64 = [f64_from_hex(String(c["VF64"][i + 1])) for i in 1:vf64_n]
    got_vf64 = read(r, Vector{Float64})
    length(got_vf64) == vf64_n || error("VF64 length: got=$(length(got_vf64)) expected=$vf64_n")
    for i in 1:vf64_n
        check_f64("VF64[$i]", got_vf64[i], expected_vf64[i])
    end

    # VSTR: count [bc1 hex1 bc2 hex2 ...]
    vstr_n = parse(Int, c["VSTR"][1])
    expected_vstr = String[]
    for i in 1:vstr_n
        # tokens at positions 2 + 2*(i-1) (bc) and 3 + 2*(i-1) (hex), 1-indexed
        hex_idx = 1 + 2 * (i - 1) + 2
        push!(expected_vstr, String(decode_hex(c["VSTR"][hex_idx])))
    end
    got_vstr = read(r, Vector{String})
    check("VSTR", got_vstr, expected_vstr)

    return true
end

function main()
    n_ok = 0
    n_fail = 0
    max_failures_shown = 5
    while true
        c = parse_case(stdin)
        c === nothing && break
        case_id = haskey(c, "CASE") ? String(c["CASE"][1]) : "?"
        kind = haskey(c, "KIND") ? String(c["KIND"][1]) : "?"
        try
            verify(c)
            n_ok += 1
        catch e
            n_fail += 1
            if n_fail <= max_failures_shown
                println(stderr, "FAIL case=$case_id kind=$kind: $(sprint(showerror, e))")
            end
        end
    end
    println(stderr, "verified=$n_ok failed=$n_fail")
    exit(n_fail == 0 ? 0 : 1)
end

main()
