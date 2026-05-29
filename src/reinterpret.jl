# Zero-copy reinterpretation of raw bytes as a struct value.
#
# When a struct is "compact" — its Julia in-memory layout is bit-identical to
# its CDR wire encoding (isbits, no trailing pad, CDR1 alignment, host
# endianness; see `_is_compact_struct`) — the bytes already sitting in the
# buffer *are* a valid `T`. `reinterpret_struct` verifies that layout, then
# pulls the value out with a single `unsafe_load`: no per-field decode, no
# allocation, the C `T v = *(T *)buf;` idiom.
#
# Like the compact read/write fast paths this uses an unaligned load — valid
# on x86/ARM64; on a platform that faults on unaligned access it carries the
# same caveat as the rest of the compact machinery (CDR aligns relative to the
# message origin at byte 4, so 8-byte fields are only 4-byte aligned in
# absolute terms).

# Host-endian CDR1 compactness — the variant a standalone reinterpret of raw
# memory assumes (no stream to carry endianness/version).
@inline _is_host_compact(::Type{T}) where T =
    _is_compact_struct(T, false, ENDIAN_BOM == 0x04030201)

# Can a CDR sequence of element type `ET` be aliased as a contiguous block of
# Julia `ET` (no per-element decode)? For a struct that's compactness; for a
# primitive or SArray-of-primitive it's just an endianness match (the element
# stride is `sizeof(ET)` either way). `Char` is excluded: Julia `Char` is 4
# bytes but a CDR char is 1, so the strides disagree.
@inline function _is_view_eltype(::Type{ET}, isCDR2::Bool, LE::Bool) where ET
    ET === Char && return false
    if _is_packed_leaf(ET)
        return LE == (ENDIAN_BOM == 0x04030201)
    end
    return _is_compact_struct(ET, isCDR2, LE)
end

"""
    reinterpret_struct(mem::DenseVector{UInt8}, ::Type{T}, byte_offset=0) -> T

Load a `T` directly from the bytes of `mem` (a `Memory{UInt8}` or
`Vector{UInt8}`) at `byte_offset` (0-based) with a single `unsafe_load` and no
allocation. Throws an `ArgumentError` unless `T` is a compact struct under
CDR1 + host endianness — i.e. its in-memory layout matches the CDR wire bytes
exactly (`isbits`, no trailing pad; see [`@cdr_compact`]) — and a `BoundsError`
if the `sizeof(T)` bytes at `byte_offset` do not fit within `mem`.
"""
function reinterpret_struct(mem::DenseVector{UInt8}, ::Type{T}, byte_offset::Integer=0) where T
    _is_host_compact(T) ||
        throw(ArgumentError(string("reinterpret_struct: ", T,
                                   " is not a compact struct (CDR1, host endianness)")))
    off = Int(byte_offset)
    off >= 0 && off + sizeof(T) <= length(mem) ||
        throw(BoundsError(mem, off + sizeof(T)))
    return GC.@preserve mem unsafe_load(Ptr{T}(pointer(mem, off + 1)))
end

"""
    reinterpret_struct(r::CDRReader, ::Type{T}) -> T

Load a `T` from the reader's buffer at the current position (after CDR
alignment) via a single `unsafe_load`, advancing the cursor by `sizeof(T)`.
Only IOBuffer-/MemBuf-backed readers are supported. Throws an `ArgumentError`
unless `T` is a compact struct for the reader's encapsulation variant.

Equivalent to `read(r, T)` for a compact struct, but *asserts* the
single-load layout rather than silently falling back to a field-by-field
decode — use it when zero-copy is a requirement, not just an optimisation.
"""
@generated function reinterpret_struct(r::R, ::Type{T}) where {R <: CDRReader{<:_CDRBufLike}, T}
    isCDR2 = R.parameters[2]
    LE     = R.parameters[3]
    if !_is_compact_struct(T, isCDR2, LE)
        return :(throw(ArgumentError(string("reinterpret_struct: ", $T,
            " is not a compact struct for this stream variant"))))
    end
    max_align = _wa_align_for(T, isCDR2)
    sz = sizeof(T)
    return quote
        align(r, $max_align)
        src = r.src
        mem = _buf_data(src)
        off = _buf_pos(src) - 1            # 0-based byte offset
        off + $sz <= _buf_size(src) || throw(BoundsError(mem, off + $sz))
        v = GC.@preserve mem unsafe_load(Ptr{$T}(pointer(mem, off + 1)))
        _buf_pos!(src, off + 1 + $sz)
        return v
    end
end

# A CDR sequence of a compact struct is bit-identical to a contiguous block of
# that struct: compact means no trailing pad, so consecutive elements sit
# exactly `sizeof(T)` apart with no inter-element padding — the same layout as
# a Julia array of `T`. So the bytes can be presented as `[T...]` with no copy.

"""
    CDRArray{T, S} <: AbstractVector{T}

A zero-copy, owned array view of compact structs `T` laid out contiguously in
a byte buffer `S` (`Memory{UInt8}` or `Vector{UInt8}`). The buffer is held as
a field, so it stays alive for the array's lifetime — no `collect` needed and
no dangling. Indexing loads an element straight from the bytes; assigning
stores one back in place. Construct via [`reinterpret_array`](@ref).
"""
struct CDRArray{T, S <: DenseVector{UInt8}} <: AbstractVector{T}
    mem::S
    offset::Int    # 0-based byte offset of element 1
    len::Int       # element count
end

CDRArray{T}(mem::S, offset::Integer, len::Integer) where {T, S <: DenseVector{UInt8}} =
    CDRArray{T, S}(mem, Int(offset), Int(len))

Base.size(a::CDRArray) = (getfield(a, :len),)
Base.IndexStyle(::Type{<:CDRArray}) = IndexLinear()

@inline function Base.getindex(a::CDRArray{T}, i::Int) where T
    @boundscheck checkbounds(a, i)
    mem = getfield(a, :mem)
    off = getfield(a, :offset) + (i - 1) * sizeof(T)
    return GC.@preserve mem unsafe_load(Ptr{T}(pointer(mem, off + 1)))
end

@inline function Base.setindex!(a::CDRArray{T}, v, i::Int) where T
    @boundscheck checkbounds(a, i)
    mem = getfield(a, :mem)
    off = getfield(a, :offset) + (i - 1) * sizeof(T)
    GC.@preserve mem unsafe_store!(Ptr{T}(pointer(mem, off + 1)), convert(T, v))
    return v
end

"""
    reinterpret_array(mem::DenseVector{UInt8}, ::Type{T}, byte_offset, count) -> CDRArray{T}

View `count` consecutive `T` values starting at `byte_offset` (0-based) in
`mem` as a [`CDRArray`](@ref), with no copy. `T` must be a compact struct
under CDR1 + host endianness (see [`reinterpret_struct`](@ref)). The result
aliases — and keeps alive — `mem`.
"""
function reinterpret_array(mem::DenseVector{UInt8}, ::Type{T},
                           byte_offset::Integer, count::Integer) where T
    _is_view_eltype(T, false, ENDIAN_BOM == 0x04030201) ||
        throw(ArgumentError(string("reinterpret_array: ", T,
                                   " is not a compact element type (CDR1, host endianness)")))
    off = Int(byte_offset)
    n = Int(count)
    off >= 0 && n >= 0 && off + n * sizeof(T) <= length(mem) ||
        throw(BoundsError(mem, off + n * sizeof(T)))
    return CDRArray{T}(mem, off, n)
end

"""
    reinterpret_array(r::CDRReader, ::Type{T}; num=<sequence length>) -> CDRArray{T}

Read a CDR sequence of compact structs `T` as a zero-copy [`CDRArray`](@ref)
over the reader's buffer, advancing the cursor past the elements. By default
the element count is taken from the stream's `UInt32` length prefix (as
`read(r, Vector{T})` does); pass `num` for a fixed-length array with no prefix.
`T` must be compact for the reader's encapsulation variant.
"""
function reinterpret_array(r::CDRReader{B, IsCDR2, LE}, ::Type{T};
                           num=sequenceLength(r)) where {B <: _CDRBufLike, IsCDR2, LE, T}
    _is_view_eltype(T, IsCDR2, LE) ||
        throw(ArgumentError(string("reinterpret_array: ", T,
                                   " is not a compact element type for this stream variant")))
    align(r, _wa_align_for(T, IsCDR2))
    src = r.src
    mem = _buf_data(src)
    start = _buf_pos(src)              # 1-based
    n = Int(num)
    nbytes = n * sizeof(T)
    start - 1 + nbytes <= _buf_size(src) || throw(BoundsError(mem, start - 1 + nbytes))
    _buf_pos!(src, start + nbytes)
    return CDRArray{T}(mem, start - 1, n)
end

# A CDR string is a `UInt32` length (including the null terminator) followed by
# that many UTF-8 bytes. The content bytes already sit in the buffer, so they
# can be presented as an `AbstractString` with no copy.

"""
    CDRString{S} <: AbstractString

A zero-copy view of a UTF-8 string living in a byte buffer `S` (`Memory{UInt8}`
or `Vector{UInt8}`). The buffer is held as a field, so it stays alive for the
string's lifetime. Implements the `AbstractString` interface, so it iterates,
compares, prints, and interpolates like any string; `String(s)` materialises
an owned copy. Construct via [`reinterpret_string`](@ref).
"""
struct CDRString{S <: DenseVector{UInt8}} <: AbstractString
    mem::S
    offset::Int     # 0-based byte offset of the first content byte
    nbytes::Int     # content length in bytes (excludes the CDR null terminator)
end

CDRString(mem::S, offset::Integer, nbytes::Integer) where {S <: DenseVector{UInt8}} =
    CDRString{S}(mem, Int(offset), Int(nbytes))

Base.ncodeunits(s::CDRString) = getfield(s, :nbytes)
Base.codeunit(::CDRString) = UInt8
@inline function Base.codeunit(s::CDRString, i::Integer)
    @boundscheck (1 <= i <= getfield(s, :nbytes)) || throw(BoundsError(s, i))
    mem = getfield(s, :mem)
    return GC.@preserve mem unsafe_load(pointer(mem, getfield(s, :offset) + Int(i)))
end

@inline function Base.isvalid(s::CDRString, i::Int)
    1 <= i <= ncodeunits(s) || return false
    @inbounds b = codeunit(s, i)
    return (b & 0xc0) != 0x80         # not a UTF-8 continuation byte → char start
end

# UTF-8 decode adapted from Base's `iterate(::String, ::Int)`, which is already
# written against `codeunit(s, i)` / `ncodeunits(s)`. On a malformed sequence
# it returns the partial char and the advanced index, exactly as Base does.
@inline _cdrstr_between(b::UInt8, lo::UInt8, hi::UInt8) = (lo <= b) & (b <= hi)

@inline function Base.iterate(s::CDRString, i::Int=1)
    i > ncodeunits(s) && return nothing
    @inbounds b = codeunit(s, i)
    u = UInt32(b) << 24
    _cdrstr_between(b, 0x80, 0xf7) || return reinterpret(Char, u), i + 1
    return _cdrstr_iterate_continued(s, i, u)
end

function _cdrstr_iterate_continued(s::CDRString, i::Int, u::UInt32)
    u < 0xc0000000 && (i += 1; return reinterpret(Char, u), i)
    n = ncodeunits(s)
    (i += 1) > n && return reinterpret(Char, u), i
    @inbounds b = codeunit(s, i); (b & 0xc0 == 0x80) || return reinterpret(Char, u), i
    u |= UInt32(b) << 16
    (((i += 1) > n) | (u < 0xe0000000)) && return reinterpret(Char, u), i
    @inbounds b = codeunit(s, i); (b & 0xc0 == 0x80) || return reinterpret(Char, u), i
    u |= UInt32(b) << 8
    (((i += 1) > n) | (u < 0xf0000000)) && return reinterpret(Char, u), i
    @inbounds b = codeunit(s, i); (b & 0xc0 == 0x80) || return reinterpret(Char, u), i
    u |= UInt32(b); i += 1
    return reinterpret(Char, u), i
end

# Direct byte copy beats the generic char-by-char `String(::AbstractString)`.
function Base.String(s::CDRString)
    n = getfield(s, :nbytes)
    out = Base.StringVector(n)
    mem = getfield(s, :mem)
    GC.@preserve mem out unsafe_copyto!(pointer(out), pointer(mem, getfield(s, :offset) + 1), n)
    return String(out)
end

"""
    reinterpret_string(mem::DenseVector{UInt8}, byte_offset, nbytes) -> CDRString

View `nbytes` UTF-8 content bytes starting at `byte_offset` (0-based) in `mem`
as a [`CDRString`](@ref), with no copy. `nbytes` is the content length and
excludes any CDR null terminator.
"""
function reinterpret_string(mem::DenseVector{UInt8}, byte_offset::Integer, nbytes::Integer)
    off = Int(byte_offset)
    n = Int(nbytes)
    off >= 0 && n >= 0 && off + n <= length(mem) || throw(BoundsError(mem, off + n))
    return CDRString(mem, off, n)
end

"""
    reinterpret_string(r::CDRReader) -> CDRString

Read a CDR string as a zero-copy [`CDRString`](@ref) over the reader's buffer,
advancing the cursor past the content and its null terminator. Mirrors
`read(r, String)` but aliases the buffer instead of copying.
"""
function reinterpret_string(r::CDRReader{B}) where {B <: _CDRBufLike}
    len = Int(sequenceLength(r))       # CDR length: content + null terminator
    src = r.src
    mem = _buf_data(src)
    start = _buf_pos(src)              # 1-based, first content byte
    start - 1 + len <= _buf_size(src) || throw(BoundsError(mem, start - 1 + len))
    _buf_pos!(src, start + len)        # skip content + null
    nbytes = len <= 1 ? 0 : len - 1
    return CDRString(mem, start - 1, nbytes)
end

# --- Strict views + capability predicates ---------------------------------
#
# The contracts are deliberately separate, never one call that silently
# chooses (which would hide an allocation cliff):
#   * `read(r, Vector{T})` / `read(r, String)` — owned, always works.
#   * `view(r, CDRArray{T})` / `view(r, CDRString)` — guaranteed alias, errors
#     early if impossible (mirrors Base `view`: a guarantee, not a hint).
#   * `canview(r, …)` / `iscompact(r, T)` — predicates for composing a smooth,
#     type-stable fallback yourself: `canview(r, V) ? view(r, V) : read(r, …)`.
#
# `canview`/`iscompact` are `@generated` so they reduce to a literal `Bool` at
# the call site — capability is a property of the reader's type parameters
# (endianness/variant) and the requested type, so the surrounding `?:` branch
# constant-folds and inference specializes each arm.

"""
    iscompact(r::CDRReader, ::Type{T}) -> Bool

Whether `read(r, T)` decodes the struct `T` as a single `unsafe_load` (its
layout matches the wire bytes for this reader's variant/endianness) rather
than walking it field by field. A compile-time constant.
"""
@generated function iscompact(r::CDRReader{B, IsCDR2, LE}, ::Type{T}) where {B, IsCDR2, LE, T}
    return :($(B <: _CDRBufLike && _is_compact_struct(T, IsCDR2, LE)))
end

"""
    canview(r::CDRReader, ::Type{CDRArray{T}}) -> Bool
    canview(r::CDRReader, ::Type{CDRString})   -> Bool

Whether [`view`](@ref) can alias the requested type for this reader (buffer
backing, and — for `CDRArray` — an element that's compact under the reader's
variant and host endianness). A compile-time constant, intended for
`canview(r, V) ? view(r, V) : read(r, …)`.
"""
@generated function canview(r::CDRReader{B, IsCDR2, LE},
                            ::Type{V}) where {B, IsCDR2, LE, T, V <: CDRArray{T}}
    return :($(B <: _CDRBufLike && _is_view_eltype(T, IsCDR2, LE)))
end
@generated function canview(r::CDRReader{B}, ::Type{CDRString}) where B
    return :($(B <: _CDRBufLike))
end

"""
    view(r::CDRReader, ::Type{CDRArray{T}}) -> CDRArray{T}
    view(r::CDRReader, ::Type{CDRString})   -> CDRString

Read the next value as a zero-copy view aliasing the reader's buffer,
advancing the cursor. Strict: it errors if the view isn't possible (use
[`canview`](@ref) to branch, or [`read`](@ref) for an owned copy). Mirrors
`Base.view` — asking for a view is a guarantee, not a hint that may silently
fall back to a copy.
"""
Base.view(r::CDRReader, ::Type{V}) where {T, V <: CDRArray{T}} = reinterpret_array(r, T)
Base.view(r::CDRReader, ::Type{CDRString}) = reinterpret_string(r)

# Read one field for `read_view`. Every branch condition is a compile-time
# constant (a property of `FT`/`IsCDR2`/`LE`), so the compiler prunes this to
# a single path per field type — no runtime dispatch. Fixed-size parts
# (primitives, SArrays, all-fixed structs) decode to ordinary values; dynamic
# CDR sequences become zero-copy `CDRArray` views; nested structs that
# themselves contain sequences recurse. Sequences whose elements can't be
# aliased (strings, jagged structs) fall back to a normal decode.
@inline function _read_view_field(r::CDRReader{B, IsCDR2, LE}, ::Type{FT}) where {B, IsCDR2, LE, FT}
    if FT <: StaticArray || _is_packed_type(FT)
        return read(r, FT)
    elseif FT <: AbstractString
        return reinterpret_string(r)        # zero-copy CDRString view
    elseif FT <: AbstractVector
        ET = eltype(FT)
        if _is_view_eltype(ET, IsCDR2, LE)
            return reinterpret_array(r, ET)
        else
            return read(r, FT)
        end
    elseif isstructtype(FT) && isconcretetype(FT) && fieldcount(FT) > 0
        return read_view(r, FT)
    else
        return read(r, FT)
    end
end

"""
    read_view(r::CDRReader, ::Type{T}) -> NamedTuple

Read a struct `T` as a lazy view: scalar and fixed-size fields are decoded to
ordinary values, but each variable-length sequence field becomes a zero-copy
[`CDRArray`](@ref) aliasing the reader's buffer — so the elements are never
touched or copied. Nested structs that contain sequences recurse to nested
views. The result is a `NamedTuple` with `T`'s field names, accessed the same
way (`v.field`, `v.seq[i]`); `T` itself is unchanged.

Sequence fields whose elements can't be aliased (strings, or jagged/dynamic
element structs) fall back to a normal decode. Only IOBuffer-/MemBuf-backed
readers are supported.
"""
@inline function read_view(r::CDRReader{B, IsCDR2, LE},
                           ::Type{T}) where {B <: _CDRBufLike, IsCDR2, LE, T}
    isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0 ||
        throw(ArgumentError(string("read_view: ", T,
                                   " is not a concrete struct with fields")))
    # `ntuple(…, Val(N))` unrolls, so each `fieldtype(T, i)` folds to a
    # concrete type and the tuple is built left-to-right (fields read in order).
    vals = ntuple(i -> _read_view_field(r, fieldtype(T, i)), Val(fieldcount(T)))
    return NamedTuple{fieldnames(T)}(vals)
end

# --- @cdr_compact view-struct support -------------------------------------
#
# A struct gets the buffer-backed *view* treatment when the user *opts in* by
# declaring a field as `CDRString` or `CDRArray{ElementType}`. Those fields
# become zero-copy views over the reader's buffer; every other field is read
# as an ordinary owned value. The detection below is purely syntactic (on the
# field-type ASTs), so `@cdr_compact` never silently turns a plain `Vector`
# into a view. These helpers run at macro-expansion time.

# Does the type expression `te` name `sym` — bare (`CDRString`) or qualified
# (`CDRSerialization.CDRString`)?
function _cdr_expr_is_name(te, sym::Symbol)
    te === sym && return true
    te isa Expr && te.head === :. && length(te.args) == 2 &&
        te.args[2] === QuoteNode(sym) && return true
    return false
end

# Classify a field's declared type expression:
#   (:string, nothing)        for `CDRString`
#   (:array,  element_expr)   for `CDRArray{Element}`
#   (:plain,  nothing)        for anything else (read as an owned value)
function _cdr_view_field_class(te)
    _cdr_expr_is_name(te, :CDRString) && return (:string, nothing)
    if te isa Expr && te.head === :curly && _cdr_expr_is_name(te.args[1], :CDRArray)
        length(te.args) == 2 ||
            error("@cdr_compact: a CDRArray field must be written `CDRArray{ElementType}`")
        return (:array, te.args[2])
    end
    return (:plain, nothing)
end

# True if any field opts into a view (`CDRString`/`CDRArray{…}`) — the signal
# `@cdr_compact` uses to pick the view path over the compact single-store path.
_cdr_has_view_fields(f_type_exprs) =
    any(te -> _cdr_view_field_class(te)[1] !== :plain, f_type_exprs)

# Build the escaped definition block for an opt-in view struct.
function _cdr_view_emit(name_sym::Symbol, f_names::Vector{Symbol}, f_type_exprs::Vector)
    cdrarray   = GlobalRef(@__MODULE__, :CDRArray)
    cdrstring  = GlobalRef(@__MODULE__, :CDRString)
    ri_string  = GlobalRef(@__MODULE__, :reinterpret_string)
    ri_array   = GlobalRef(@__MODULE__, :reinterpret_array)
    buf_data   = GlobalRef(@__MODULE__, :_buf_data)
    reader     = GlobalRef(@__MODULE__, :CDRReader)
    buflike    = GlobalRef(@__MODULE__, :_CDRBufLike)

    n = length(f_names)
    field_decls = Expr[]
    read_args = Any[]
    for i in 1:n
        kind, elt = _cdr_view_field_class(f_type_exprs[i])
        if kind === :string
            push!(field_decls, Expr(:(::), f_names[i], :($cdrstring{S})))
            push!(read_args, :($ri_string(_r)))
        elseif kind === :array
            push!(field_decls, Expr(:(::), f_names[i], :($cdrarray{$elt, S})))
            push!(read_args, :($ri_array(_r, $elt)))
        else
            # Owned field: keep the declared type, decode it normally.
            push!(field_decls, Expr(:(::), f_names[i], f_type_exprs[i]))
            push!(read_args, :(read(_r, $(f_type_exprs[i]))))
        end
    end

    propnames = Expr(:tuple, [QuoteNode(nm) for nm in f_names]...)

    show_stmts = Expr[]
    for (k, nm) in enumerate(f_names)
        k > 1 && push!(show_stmts, :(print(_io, ", ")))
        push!(show_stmts, :(print(_io, $(string(nm)), "=", repr(getfield(_v, $(QuoteNode(nm)))))))
    end

    return esc(quote
        struct $name_sym{S <: DenseVector{UInt8}}
            $(field_decls...)
        end

        # Arguments evaluate left-to-right, so fields are read in declaration
        # order; each read's return type matches its declared field type.
        function Base.read(_r::$reader{B}, ::Type{$name_sym}) where {B <: $buflike}
            _S = typeof($buf_data(_r.src))
            return $name_sym{_S}($(read_args...))
        end

        Base.propertynames(::$name_sym) = $propnames

        function Base.:(==)(_a::$name_sym, _b::$name_sym)
            for _n in $propnames
                getfield(_a, _n) == getfield(_b, _n) || return false
            end
            return true
        end

        function Base.show(_io::IO, _v::$name_sym)
            print(_io, $(string(name_sym)), "(")
            $(show_stmts...)
            print(_io, ")")
        end
    end)
end
