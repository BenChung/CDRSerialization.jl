# Type-level layout helpers shared between the writer, reader, and size
# calculator. Knows about CDR alignment rules and how to recognise structs
# whose Julia in-memory layout matches their CDR wire encoding.

const _PrimitivePacked = Union{Int8, UInt8, Bool, Char,
                               Int16, UInt16, Int32, UInt32, Float32,
                               Int64, UInt64, Float64}

# Leaf types: primitives and SArrays of primitives. After schema expansion
# these are the atomic types `write_all!` / `read_all!` see.
function _is_packed_leaf(::Type{T}) where T
    T <: _PrimitivePacked && return true
    if T <: SArray
        ET = T.parameters[2]
        return ET <: Union{Int8, UInt8, Bool,
                           Int16, UInt16, Int32, UInt32, Float32,
                           Int64, UInt64, Float64}
    end
    return false
end

# True for any type whose total bytes is known at the type level: a packed
# leaf or a user struct whose leaves are all packed (transitively).
function _is_packed_type(::Type{T}) where T
    _is_packed_leaf(T) && return true
    (T <: AbstractString || T <: AbstractArray) && return false
    if isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0
        return all(_is_packed_type(fieldtype(T, i)) for i in 1:fieldcount(T))
    end
    return false
end

# CDR alignment of a value of type T (1, 2, 4, or 8 bytes).
function _wa_align_for(::Type{T}, isCDR2::Bool) where T
    T <: Union{Int8, UInt8, Bool, Char}            && return 1
    T <: Union{Int16, UInt16}                      && return 2
    T <: Union{Int32, UInt32, Float32}             && return 4
    T <: Union{Int64, UInt64, Float64}             && return isCDR2 ? 4 : 8
    if T <: SArray
        return _wa_align_for(T.parameters[2], isCDR2)
    end
    if isstructtype(T) && isbitstype(T) && fieldcount(T) > 0
        return maximum(_wa_align_for(fieldtype(T, i), isCDR2) for i in 1:fieldcount(T))
    end
    error("layout: unsupported type $T")
end

# CDR-encoded byte size of a packed value. For structs, walks fields with
# CDR alignment rules — does NOT include trailing pad.
function _wa_size_for(::Type{T}) where T
    T <: Union{Int8, UInt8, Bool, Char} && return 1
    T <: Union{Int16, UInt16, Int32, UInt32, Float32,
               Int64, UInt64, Float64}  && return sizeof(T)
    if T <: SArray
        return T.parameters[4] * sizeof(T.parameters[2])
    end
    if isstructtype(T) && isbitstype(T) && fieldcount(T) > 0
        return _struct_cdr_size(T)
    end
    error("layout: unsupported type $T")
end

function _struct_cdr_size(::Type{T}, isCDR2::Bool=false) where T
    T <: Union{Int8, UInt8, Bool, Char}              && return 1
    T <: Union{Int16, UInt16}                        && return 2
    T <: Union{Int32, UInt32, Float32}               && return 4
    T <: Union{Int64, UInt64, Float64}               && return sizeof(T)
    if T <: SArray
        return T.parameters[4] * sizeof(T.parameters[2])
    end
    if isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0
        total = 0
        for i in 1:fieldcount(T)
            FT = fieldtype(T, i)
            a = _wa_align_for(FT, isCDR2)
            rem = total % a
            rem != 0 && (total += a - rem)
            total += _struct_cdr_size(FT, isCDR2)
        end
        return total
    end
    return -1
end

# A "compact" struct can flow through a single `unsafe_store!`/`unsafe_load`
# instead of per-field stores. Requires:
#   * isbits with no trailing pad (Julia layout = CDR layout)
#   * CDR1 (CDR2 uses 4-byte align for 8-byte types; Julia uses 8)
#   * Wire endianness matches host (no per-field byte swap needed)
# `LE` is the writer/reader's little-endian type parameter.
function _is_compact_struct(::Type{T}, isCDR2::Bool, LE::Bool) where T
    LE == (ENDIAN_BOM == 0x04030201) || return false
    isstructtype(T) && isconcretetype(T) && isbitstype(T) || return false
    fieldcount(T) > 0 || return false
    # A struct that's compact under CDR1 may or may not be compact under
    # CDR2 depending on its fields. CDR2 aligns 8-byte primitives to 4 but
    # Julia aligns them to 8 — for that to still match the struct must
    # happen to lay out the same way (e.g. all 8-byte fields at multiples
    # of 8 anyway). `_struct_cdr_size(T, isCDR2)` handles the per-variant
    # comparison.
    return _struct_cdr_size(T, isCDR2) == sizeof(T)
end

# Map an original field type to the type used in the CDR2-laid-out inner
# struct. 8-byte primitives become `SVector{2, Float32}` so the Julia
# alignment drops to 4 (matching CDR2 wire alignment); SArrays of 8-byte
# primitives become `SVector{2L, Float32}` likewise. Everything else passes
# through unchanged because its native Julia alignment already matches
# CDR2's rule.
function _cdr2_substitute_type(::Type{T}) where T
    if T <: Union{Int64, UInt64, Float64}
        return SVector{2, Float32}
    elseif T <: SArray
        ET = T.parameters[2]
        if ET <: Union{Int64, UInt64, Float64}
            L = T.parameters[4]
            return SVector{2L, Float32}
        end
    end
    return T
end

# Convert a user-supplied value to the storage representation used by the
# CDR2 inner struct's matching field. Pure bit reinterpretation.
function _cdr2_pack(::Type{T}, x) where T <: Union{Int64, UInt64, Float64}
    SVector{2, Float32}(reinterpret(NTuple{2, Float32}, (x,)))
end
function _cdr2_pack(::Type{T}, x::SArray{S, ET, N, L}) where {S, ET <: Union{Int64, UInt64, Float64}, N, L, T <: SArray{S, ET, N, L}}
    SVector{2L, Float32}(reinterpret(NTuple{2L, Float32}, x.data))
end
_cdr2_pack(::Type{T}, x) where T = x

# Reverse of `_cdr2_pack`: convert from CDR2 storage back to the user-facing
# type.
function _cdr2_unpack(::Type{T}, x::SVector{2, Float32}) where T <: Union{Int64, UInt64, Float64}
    reinterpret(NTuple{1, T}, x.data)[1]
end
function _cdr2_unpack(::Type{T}, x::SArray) where {S, ET <: Union{Int64, UInt64, Float64}, N, L, T <: SArray{S, ET, N, L}}
    T(reinterpret(NTuple{L, ET}, x.data))
end
_cdr2_unpack(::Type{T}, x) where T = x

# Pre-compute layout (size + max_align) for a sequence of field types under
# a given CDR variant.
function _cdr_compact_layout(types::Vector{<:Type}, isCDR2::Bool)
    total = 0
    max_align = 1
    for T in types
        a = _wa_align_for(T, isCDR2)
        max_align = max(max_align, a)
        rem = total % a
        rem != 0 && (total += a - rem)
        total += sizeof(T)
    end
    julia_size = total % max_align == 0 ? total :
                 total + (max_align - total % max_align)
    return (total, julia_size, max_align)
end

"""
    @cdr_compact struct Name
        field1::T1
        ...
    end

Define a struct whose in-memory layout matches its CDR wire encoding —
both for **CDR1** (8-byte primitives align to 8) and **XCDR2** (8-byte
primitives align to 4) — by emitting two internal storage structs with
trailing padding plus a public wrapper `Name{V}` parameterised on variant.
Both variants qualify for the compact-struct single-store fast path on
`write_all!` and `read`.

The macro emits:

  * `_Name_CDR1` — fields use their natural Julia types plus trailing pad.
  * `_Name_CDR2` — 8-byte primitives (and SArrays thereof) substituted with
    `SVector{N, Float32}` so the field alignment drops from 8 to 4, plus
    trailing pad. The bytes are identical to the original on a host whose
    endianness matches the writer.
  * `Name{V}` — public wrapper holding one of the inner types. Field access
    through `.field` transparently reinterprets the CDR2 storage back to
    the user-facing type.

Default constructor `Name(args...)` produces a CDR1 value; passing
`xcdr2=true` produces the CDR2 variant. Library hooks dispatch on the
inner type so a CDR1 writer paired with a CDR2 wrapper is a `MethodError`,
not garbage on the wire.

**Wire format note:** the trailing pad fields are part of the wire format —
each value is `sizeof(_Name_CDR1)` / `sizeof(_Name_CDR2)` bytes, not
`sum-of-declared-field-sizes`. This is self-consistent for round-trip use
but **not** wire-compatible with standard CDR producers (e.g. ROS
publishers), which omit trailing pad.

**Endianness:** the fast path only fires when writer/reader endianness
matches the host. On a little-endian host (the common case) use the
default LE encapsulation.

Field types must be primitives or `SArray`s of primitives.
"""
macro cdr_compact(structdef)
    structdef isa Expr && structdef.head === :struct ||
        error("@cdr_compact: expected `struct …` definition")
    is_mutable = structdef.args[1]
    is_mutable && error("@cdr_compact: mutable structs are not supported")
    name_part  = structdef.args[2]
    body       = structdef.args[3]

    name_sym = name_part isa Symbol ? name_part :
               name_part isa Expr && name_part.head === :curly ? name_part.args[1] :
               name_part isa Expr && name_part.head === :(<:) ? name_part.args[1] :
               error("@cdr_compact: unsupported struct name form: $name_part")

    f_names = Symbol[]
    f_type_exprs = Any[]
    for ex in body.args
        ex isa LineNumberNode && continue
        ex isa Expr && ex.head === :(::) && length(ex.args) == 2 ||
            error("@cdr_compact: only `field::Type` declarations are supported (got $ex)")
        push!(f_names, ex.args[1])
        push!(f_type_exprs, ex.args[2])
    end
    isempty(f_names) && error("@cdr_compact: struct has no fields")

    f_types = Type[Base.eval(__module__, te) for te in f_type_exprs]

    for T in f_types
        ok = T <: _PrimitivePacked ||
             (T <: SArray && T.parameters[2] <: _PrimitivePacked)
        ok || error("@cdr_compact: field type $T is not compact-eligible " *
                    "(must be a primitive or an SArray of primitive)")
    end

    inner1 = Symbol("_", name_sym, "_CDR1")
    inner2 = Symbol("_", name_sym, "_CDR2")

    # CDR1: use natural field types.
    cdr1_raw, cdr1_julia_size, _ = _cdr_compact_layout(f_types, false)
    cdr1_pad = cdr1_julia_size - cdr1_raw

    # CDR2: substitute 8-byte primitives to drop alignment to 4.
    cdr2_subst_types = Type[_cdr2_substitute_type(T) for T in f_types]
    cdr2_raw, cdr2_julia_size, _ = _cdr_compact_layout(cdr2_subst_types, true)
    cdr2_pad = cdr2_julia_size - cdr2_raw

    # Field-declaration AST for each inner struct.
    cdr1_field_exprs = Expr[Expr(:(::), f_names[i], f_type_exprs[i]) for i in 1:length(f_names)]
    for i in 1:cdr1_pad
        push!(cdr1_field_exprs, Expr(:(::), Symbol("_cdr1_pad", i), :UInt8))
    end

    cdr2_field_exprs = Expr[]
    for (i, T) in enumerate(f_types)
        ST = cdr2_subst_types[i]
        type_expr = ST === T ? f_type_exprs[i] :
                    ST <: SArray && ST.parameters[2] === Float32 ?
                        :($(GlobalRef(StaticArrays, :SVector)){$(ST.parameters[4]), Float32}) :
                        :($(GlobalRef(StaticArrays, :SVector)){2, Float32})
        push!(cdr2_field_exprs, Expr(:(::), f_names[i], type_expr))
    end
    for i in 1:cdr2_pad
        push!(cdr2_field_exprs, Expr(:(::), Symbol("_cdr2_pad", i), :UInt8))
    end

    # Argument list for the smart constructor.
    arg_exprs = [Expr(:(::), f_names[i], f_type_exprs[i]) for i in 1:length(f_names)]

    # Pack/unpack expressions used inside the constructor and getproperty
    # bodies. These reference the module's `_cdr2_pack`/`_cdr2_unpack`
    # helpers via fully-qualified names so the esc'd block keeps resolving
    # them regardless of which module declares the struct.
    pack_qual   = GlobalRef(@__MODULE__, :_cdr2_pack)
    unpack_qual = GlobalRef(@__MODULE__, :_cdr2_unpack)
    write_all_qual = GlobalRef(@__MODULE__, :write_all!)
    cdrwriter_qual = GlobalRef(@__MODULE__, :CDRWriter)
    cdrreader_qual = GlobalRef(@__MODULE__, :CDRReader)
    struct_size_qual = GlobalRef(@__MODULE__, :_struct_cdr_size)

    cdr1_ctor_call = Expr(:call, inner1, f_names..., fill(:(UInt8(0)), cdr1_pad)...)

    cdr2_pack_args = [:($pack_qual($(f_type_exprs[i]), $(f_names[i]))) for i in 1:length(f_names)]
    cdr2_ctor_call = Expr(:call, inner2, cdr2_pack_args..., fill(:(UInt8(0)), cdr2_pad)...)

    # getproperty body for the CDR2 variant: each declared field becomes
    # an `if` branch that unpacks the storage representation back to the
    # user-facing type. Pad fields fall through to the default getfield.
    cdr2_getprop_branches = Expr[]
    for (i, fn) in enumerate(f_names)
        rhs = :($unpack_qual($(f_type_exprs[i]), getfield(_inner, $(QuoteNode(fn)))))
        push!(cdr2_getprop_branches,
              :(_name === $(QuoteNode(fn)) && return $rhs))
    end

    propnames_tuple = Expr(:tuple, [QuoteNode(n) for n in f_names]...)

    return esc(quote
        struct $inner1
            $(cdr1_field_exprs...)
        end
        struct $inner2
            $(cdr2_field_exprs...)
        end
        struct $name_sym{V}
            inner::V
        end

        function $name_sym($(arg_exprs...); xcdr2::Bool=false)
            if xcdr2
                $name_sym{$inner2}($cdr2_ctor_call)
            else
                $name_sym{$inner1}($cdr1_ctor_call)
            end
        end

        Base.propertynames(::$name_sym) = $propnames_tuple

        @inline function Base.getproperty(_obj::$name_sym{$inner1}, _name::Symbol)
            _inner = getfield(_obj, :inner)
            _name === :inner && return _inner
            return getfield(_inner, _name)
        end

        @inline function Base.getproperty(_obj::$name_sym{$inner2}, _name::Symbol)
            _inner = getfield(_obj, :inner)
            _name === :inner && return _inner
            $(cdr2_getprop_branches...)
            return getfield(_inner, _name)
        end

        function Base.:(==)(_a::$name_sym, _b::$name_sym)
            for _n in $propnames_tuple
                getproperty(_a, _n) == getproperty(_b, _n) || return false
            end
            return true
        end

        function Base.show(io::IO, _v::$name_sym)
            print(io, $(string(name_sym)), "(")
            _first = true
            for _n in $propnames_tuple
                _first || print(io, ", ")
                _first = false
                print(io, _n, "=", repr(getproperty(_v, _n)))
            end
            print(io, ")")
        end

        # Dispatch hooks: only matching (writer-variant, wrapper-variant)
        # pairs forward to the compact path; mismatches hit the explicit
        # fallback below, which beats the variadic `write_all!` because
        # `$name_sym` is more specific than `Vararg{Any}`.
        function $write_all_qual(_c::$cdrwriter_qual{false}, _x::$name_sym{$inner1})
            $write_all_qual(_c, getfield(_x, :inner))
        end
        function $write_all_qual(_c::$cdrwriter_qual{true}, _x::$name_sym{$inner2})
            $write_all_qual(_c, getfield(_x, :inner))
        end
        function $write_all_qual(_c::$cdrwriter_qual{IsCDR2}, _x::$name_sym) where IsCDR2
            throw(ArgumentError(string($(string(name_sym)),
                ": variant mismatch — writer is CDR", IsCDR2 ? 2 : 1,
                " but wrapper is ", typeof(_x))))
        end

        function Base.read(_r::$cdrreader_qual{S, false, LE}, ::Type{$name_sym}) where {S, LE}
            $name_sym{$inner1}(read(_r, $inner1))
        end
        function Base.read(_r::$cdrreader_qual{S, true, LE}, ::Type{$name_sym}) where {S, LE}
            $name_sym{$inner2}(read(_r, $inner2))
        end
        function Base.read(_r::$cdrreader_qual{S, false, LE}, ::Type{$name_sym{$inner1}}) where {S, LE}
            $name_sym{$inner1}(read(_r, $inner1))
        end
        function Base.read(_r::$cdrreader_qual{S, true, LE}, ::Type{$name_sym{$inner2}}) where {S, LE}
            $name_sym{$inner2}(read(_r, $inner2))
        end

        # Self-checks. `_struct_cdr_size` returns the same number under both
        # variants for a struct whose Julia layout already matches CDR for
        # that variant; this asserts that the layout we engineered actually
        # round-trips through the layout helpers.
        @assert sizeof($inner1) == $cdr1_julia_size "@cdr_compact: CDR1 sizeof mismatch for $($(QuoteNode(inner1)))"
        @assert sizeof($inner2) == $cdr2_julia_size "@cdr_compact: CDR2 sizeof mismatch for $($(QuoteNode(inner2)))"
        @assert $struct_size_qual($inner1, false) == sizeof($inner1) "@cdr_compact: CDR1 layout helpers disagree for $($(QuoteNode(inner1)))"
        @assert $struct_size_qual($inner2, true) == sizeof($inner2) "@cdr_compact: CDR2 layout helpers disagree for $($(QuoteNode(inner2)))"
    end)
end
