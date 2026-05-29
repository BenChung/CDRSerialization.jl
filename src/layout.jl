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

# Parse the `struct …` definition shared by `@cdr1_compat`/`@cdr_compact` into
# `(name_sym, field_names, field_type_exprs)`. Rejects mutable structs and any
# member that isn't a plain `field::Type` declaration. `macroname` is woven
# into the error messages.
function _cdr_parse_structdef(structdef, macroname::AbstractString)
    structdef isa Expr && structdef.head === :struct ||
        error("$macroname: expected `struct …` definition")
    structdef.args[1] && error("$macroname: mutable structs are not supported")
    name_part = structdef.args[2]
    body      = structdef.args[3]

    name_sym = name_part isa Symbol ? name_part :
               name_part isa Expr && name_part.head === :curly ? name_part.args[1] :
               name_part isa Expr && name_part.head === :(<:) ? name_part.args[1] :
               error("$macroname: unsupported struct name form: $name_part")

    f_names = Symbol[]
    f_type_exprs = Any[]
    for ex in body.args
        ex isa LineNumberNode && continue
        ex isa Expr && ex.head === :(::) && length(ex.args) == 2 ||
            error("$macroname: only `field::Type` declarations are supported (got $ex)")
        push!(f_names, ex.args[1])
        push!(f_type_exprs, ex.args[2])
    end
    isempty(f_names) && error("$macroname: struct has no fields")
    return name_sym, f_names, f_type_exprs
end

# True when `T` lays out identically in Julia memory and on the CDR1 wire,
# field for field — a primitive, an SArray of primitives, or an isbits struct
# built (transitively) from those. Such a type round-trips through the normal
# `write_all!`/`read` paths as standard CDR1, with no trailing pad on the wire:
# the leaf flattener (`_expand_schema!`) recomputes CDR offsets, and a struct
# that happens to end on its max alignment additionally qualifies for the
# single-store fast path. Variable-length fields (String/Vector/CDRArray/
# CDRString) and abstract/non-isbits fields are rejected — they aren't flat.
function _is_cdr1_compat_type(::Type{T}) where T
    T <: _PrimitivePacked && return true
    if T <: SArray
        return T.parameters[2] <: _PrimitivePacked
    end
    if isstructtype(T) && isconcretetype(T) && isbitstype(T) && fieldcount(T) > 0
        return all(_is_cdr1_compat_type(fieldtype(T, i)) for i in 1:fieldcount(T))
    end
    return false
end

# Shared companion-method ASTs (`propertynames`, field-wise `==`, `show`) for
# both `@cdr1_compat` (plain struct) and `@cdr_compact` (variant wrapper).
# Field access goes through `getproperty`, so the wrapper's transparent CDR2
# unpacking is honoured and a plain struct falls through to `getfield`. Spliced
# into the macros' `esc`'d blocks; `name_sym` resolves in the caller's module.
function _cdr1_emit_companions(name_sym::Symbol, f_names::Vector{Symbol})
    propnames_tuple = Expr(:tuple, [QuoteNode(n) for n in f_names]...)
    return Expr[
        :(Base.propertynames(::$name_sym) = $propnames_tuple),
        quote
            function Base.:(==)(_a::$name_sym, _b::$name_sym)
                for _n in $propnames_tuple
                    getproperty(_a, _n) == getproperty(_b, _n) || return false
                end
                return true
            end
        end,
        quote
            function Base.show(_io::IO, _v::$name_sym)
                print(_io, $(string(name_sym)), "(")
                _first = true
                for _n in $propnames_tuple
                    _first || print(_io, ", ")
                    _first = false
                    print(_io, _n, "=", repr(getproperty(_v, _n)))
                end
                print(_io, ")")
            end
        end,
    ]
end

"""
    @cdr1_compat struct Name
        field1::T1
        ...
    end

Define a plain, single concrete struct whose serialization is **standard CDR1 /
XCDR1** — byte-for-byte what a conventional CDR producer (e.g. a ROS 2
publisher) emits, including *omitting trailing pad*. Unlike [`@cdr_compact`]
this emits exactly one type (no `Name{V}` variant union, no padded inner
structs), so values nest and embed cleanly in other types.

Field types must be primitives, `SArray`s of primitives, or other flat
CDR1-compatible structs (nesting is allowed and validated recursively).
Variable-length fields (`String`, `Vector`, `CDRString`, `CDRArray`) are
rejected — they aren't flat; use [`@cdr_compact`]'s view mode or read them
through the generic path.

Aside from a `write(c, v)` forwarder to `write_all!`, no custom serialization
hooks are generated: the value flows through the normal `write_all!`/`read`
machinery, which flattens it to leaves and resolves CDR1
offsets at compile time. A struct that ends on its maximum alignment also
qualifies for the single-store fast path; one that doesn't is written as a
packed run of constant-offset stores. Either way the wire bytes are standard
CDR1 with no trailing pad.

The CDR1 wire-compatibility guarantee is for CDR1/XCDR1. The same type still
encodes correctly under XCDR2 (8-byte primitives are realigned to 4 via
per-field flattening), but XCDR2 is not this macro's promise.
"""
macro cdr1_compat(structdef)
    name_sym, f_names, f_type_exprs = _cdr_parse_structdef(structdef, "@cdr1_compat")

    f_types = Type[Base.eval(__module__, te) for te in f_type_exprs]
    for (i, T) in enumerate(f_types)
        _is_cdr1_compat_type(T) ||
            error("@cdr1_compat: field $(f_names[i])::$T is not CDR1-flat-compatible " *
                  "(must be a primitive, an SArray of primitives, or another flat " *
                  "CDR1-compatible struct). Variable-length fields are not supported; " *
                  "use @cdr_compact's CDRString/CDRArray view mode or read them through " *
                  "the generic path.")
    end

    field_exprs    = Expr[Expr(:(::), f_names[i], f_type_exprs[i]) for i in 1:length(f_names)]
    companions     = _cdr1_emit_companions(name_sym, f_names)
    compat_qual    = GlobalRef(@__MODULE__, :_is_cdr1_compat_type)
    write_all_qual = GlobalRef(@__MODULE__, :write_all!)
    cdrwriter_qual = GlobalRef(@__MODULE__, :CDRWriter)

    return esc(quote
        struct $name_sym
            $(field_exprs...)
        end
        $(companions...)

        # Single-value `write` fallback: forward to `write_all!` (the generic
        # path handles flattening + CDR1 offsets) and return the byte count, as
        # `Base.write` callers expect.
        function Base.write(_c::$cdrwriter_qual, _x::$name_sym)
            _p0 = position(_c)
            $write_all_qual(_c, _x)
            return position(_c) - _p0
        end

        @assert isbitstype($name_sym) "@cdr1_compat: $($(QuoteNode(name_sym))) is not isbits"
        @assert $compat_qual($name_sym) "@cdr1_compat: $($(QuoteNode(name_sym))) is not CDR1-flat-compatible"
    end)
end

"""
    @cdr_compact struct Name
        field1::T1
        ...
    end

Define a struct carrying **both** a CDR1 and an XCDR2 layout, behind a public
wrapper `Name{V}` parameterised on variant. Use this only when one value must
be serializable under either encoding; **if you only need CDR1/XCDR1, prefer
[`@cdr1_compat`]** — it emits a single plain concrete type that nests cleanly
and has no variant union.

The macro emits:

  * `_Name_CDR1` — a flat plain struct of the natural field types (identical to
    what [`@cdr1_compat`] produces). It serializes as **standard CDR1 with no
    trailing pad**; a struct that ends on its max alignment additionally takes
    the single-store fast path.
  * `_Name_CDR2` — 8-byte primitives (and SArrays thereof) substituted with
    `SVector{N, Float32}` so the field alignment drops from 8 to 4, plus
    trailing pad so its Julia `sizeof` matches the (padded) CDR2 size for the
    single-store path. The bytes are identical to the original on a host whose
    endianness matches the writer.
  * `Name{V}` — public wrapper holding one of the inner types. Field access
    through `.field` transparently reinterprets the CDR2 storage back to
    the user-facing type.

Default constructor `Name(args...)` produces a CDR1 value; passing
`xcdr2=true` produces the CDR2 variant. Library hooks dispatch on the
inner type so a CDR1 writer paired with a CDR2 wrapper is a `MethodError`,
not garbage on the wire.

**Wire format note:** the CDR1 variant is standard-CDR-compatible (no trailing
pad). The **CDR2** variant's trailing pad *is* on the wire — its value is
`sizeof(_Name_CDR2)` bytes, not `sum-of-declared-field-sizes` — so it is not
wire-compatible with standard CDR producers that omit trailing pad.

**Endianness:** the fast path only fires when writer/reader endianness
matches the host. On a little-endian host (the common case) use the
default LE encapsulation.

Field types must be primitives or `SArray`s of primitives.
"""
macro cdr_compact(structdef)
    name_sym, f_names, f_type_exprs = _cdr_parse_structdef(structdef, "@cdr_compact")

    # Opt-in view mode: if any field is declared `CDRString` or
    # `CDRArray{Element}`, emit a buffer-backed view struct read zero-copy.
    # Plain fields are never silently turned into views.
    if _cdr_has_view_fields(f_type_exprs)
        return _cdr_view_emit(name_sym, f_names, f_type_exprs)
    end

    f_types = Type[Base.eval(__module__, te) for te in f_type_exprs]

    for T in f_types
        ok = T <: _PrimitivePacked ||
             (T <: SArray && T.parameters[2] <: _PrimitivePacked)
        ok || error("@cdr_compact: field type $T is not compact-eligible " *
                    "(must be a primitive or an SArray of primitive). " *
                    "For a variable-length field, declare it `CDRArray{Element}` " *
                    "or `CDRString` to opt into a zero-copy view.")
    end

    inner1 = Symbol("_", name_sym, "_CDR1")
    inner2 = Symbol("_", name_sym, "_CDR2")

    # CDR1: a flat plain struct of the natural field types — no trailing pad,
    # so it serializes as standard CDR1 (same as `@cdr1_compat`). The CDR1
    # dispatch hooks below forward to the generic `write_all!`/`read`, which
    # flatten it to leaves and omit trailing pad on the wire.
    cdr1_raw, cdr1_julia_size, _ = _cdr_compact_layout(f_types, false)

    # CDR2: substitute 8-byte primitives to drop alignment to 4. This variant
    # keeps the trailing pad fields because its single-store fast path requires
    # the Julia layout to match the (padded) CDR2 size exactly.
    cdr2_subst_types = Type[_cdr2_substitute_type(T) for T in f_types]
    cdr2_raw, cdr2_julia_size, _ = _cdr_compact_layout(cdr2_subst_types, true)
    cdr2_pad = cdr2_julia_size - cdr2_raw

    # Field-declaration AST for each inner struct.
    cdr1_field_exprs = Expr[Expr(:(::), f_names[i], f_type_exprs[i]) for i in 1:length(f_names)]

    cdr2_field_exprs = Expr[]
    for (i, T) in enumerate(f_types)
        ST = cdr2_subst_types[i]
        # Interpolate the `Float32` *type object* (not a bare symbol): the
        # emitted expr is `esc`'d into the caller's module, and a caller that
        # defines a type named `Float32` (e.g. a `std_msgs/Float32` message
        # struct) would otherwise shadow `Base.Float32` here.
        type_expr = ST === T ? f_type_exprs[i] :
                    ST <: SArray && ST.parameters[2] === Float32 ?
                        :($(GlobalRef(StaticArrays, :SVector)){$(ST.parameters[4]), $Float32}) :
                        :($(GlobalRef(StaticArrays, :SVector)){2, $Float32})
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

    cdr1_ctor_call = Expr(:call, inner1, f_names...)

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

    companions  = _cdr1_emit_companions(name_sym, f_names)
    compat_qual = GlobalRef(@__MODULE__, :_is_cdr1_compat_type)

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

        $(companions...)

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

        # Self-checks. The CDR1 inner is a flat plain struct: its CDR1 wire
        # size is the data size (`cdr1_raw`, no trailing pad) and it must be
        # CDR1-flat-compatible. The CDR2 inner keeps trailing pad so its Julia
        # `sizeof` matches the (padded) CDR2 size for the single-store path.
        @assert sizeof($inner1) == $cdr1_julia_size "@cdr_compact: CDR1 sizeof mismatch for $($(QuoteNode(inner1)))"
        @assert $struct_size_qual($inner1, false) == $cdr1_raw "@cdr_compact: CDR1 wire size mismatch for $($(QuoteNode(inner1)))"
        @assert $compat_qual($inner1) "@cdr_compact: CDR1 inner $($(QuoteNode(inner1))) is not CDR1-flat-compatible"
        @assert sizeof($inner2) == $cdr2_julia_size "@cdr_compact: CDR2 sizeof mismatch for $($(QuoteNode(inner2)))"
        @assert $struct_size_qual($inner2, true) == sizeof($inner2) "@cdr_compact: CDR2 layout helpers disagree for $($(QuoteNode(inner2)))"
    end)
end
