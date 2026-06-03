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
exactly (`isbits`, no trailing pad; see [`@cdr_fixed`]/[`iscompact`]) — and a `BoundsError`
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

# Note: there is no reader-form `reinterpret_struct(r, T)`. For a struct the
# zero-copy read is a single-load *value* (nothing to alias), which `read(r, T)`
# already produces for a compact struct; `iscompact(r, T)` asserts that path.

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
stores one back in place. Obtain one via `view(r, CDRArray{T})` from a reader,
or [`reinterpret_array`](@ref) over a raw buffer.
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

# Internal: view a CDR sequence of compact element `T` over the reader's
# buffer, advancing past the elements. The public reader spelling is
# `view(r, CDRArray{T})`; this is also used by `read_view` and the
# `@cdr_compact` view-struct decode. `num` defaults to the stream's UInt32
# length prefix; pass it for a fixed-length sequence with no prefix.
function _view_array(r::CDRReader{B, IsCDR2, LE}, ::Type{T};
                     num=sequenceLength(r)) where {B <: _CDRBufLike, IsCDR2, LE, T}
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

"""
    CDRArrayView{E} <: AbstractVector{E}

A *decoding* (non-aliasing) view of a CDR sequence of a **flat fixed-size**
element `E` (a primitive, `SArray` of primitive, or a struct built transitively
from those — anything `@cdr_fixed` accepts). Unlike [`CDRArray`](@ref), which
aliases the raw bytes and so requires `E` to be *compact*, `CDRArrayView`
field-walk-decodes element `i` on indexing, so it works for **every** flat `E`
(trailing pad, leading-narrow leaf, foreign endianness — all fine) and returns
an owned `E` value.

Indexing is O(1): a CDR sequence of a fixed-size struct has at most one
"phase-shifted" first element, after which the stride is constant (the element's
max-aligned field erases the start phase, so every element from the second on
begins at the same alignment). The view stores that head offset + steady stride,
no per-element table.
"""
struct CDRArrayView{E, IsCDR2, LE, S <: DenseVector{UInt8}} <: AbstractVector{E}
    mem::S
    origin::Int
    kind::EncapsulationKind
    first_pos::Int     # 1-based buffer position where element 1 begins
    second_pos::Int    # 1-based position where element 2 begins (first_pos + head span)
    stride::Int        # steady per-element stride (bytes) for elements ≥ 2
    len::Int
end

Base.size(a::CDRArrayView) = (getfield(a, :len),)
Base.IndexStyle(::Type{<:CDRArrayView}) = IndexLinear()

# Element i's start position. Only element 1 can sit at a different phase than
# the rest; element i ≥ 2 is at a uniform stride. (See the type docstring: the
# phase map is constant after one step, so this two-segment form is exact for
# every flat fixed-size element — no offset table needed.)
@inline _cdrav_pos(a::CDRArrayView, i::Int) =
    i == 1 ? getfield(a, :first_pos) :
             getfield(a, :second_pos) + (i - 2) * getfield(a, :stride)

@inline function Base.getindex(a::CDRArrayView{E, IsCDR2, LE, S}, i::Int) where {E, IsCDR2, LE, S}
    @boundscheck checkbounds(a, i)
    mem = getfield(a, :mem)
    mb = MemBuf(mem, _cdrav_pos(a, i), length(mem))
    r = CDRReader{MemBuf{S}, IsCDR2, LE}(mb, false, false, getfield(a, :origin), getfield(a, :kind))
    return read(r, E)
end

# Internal: view a CDR sequence of flat element `E` as a lazy decoding view,
# advancing the reader past the whole sequence. Measures the head span (element
# 1) and steady stride (element 2) with two trial reads — exact because the
# stride is constant from the second element on (see `CDRArrayView`).
function _seq_view(r::CDRReader{B, IsCDR2, LE}, ::Type{E}) where {B <: _CDRBufLike, IsCDR2, LE, E}
    n = Int(sequenceLength(r))
    src = r.src
    mem = _buf_data(src)
    S = typeof(mem)
    origin = r.origin
    kind = r.kind
    first_pos = _buf_pos(src)
    n == 0 && return CDRArrayView{E, IsCDR2, LE, S}(mem, origin, kind, first_pos, first_pos, 0, 0)
    read(r, E)                                   # consume element 1
    second_pos = _buf_pos(src)
    n == 1 && return CDRArrayView{E, IsCDR2, LE, S}(mem, origin, kind, first_pos, second_pos, second_pos - first_pos, 1)
    read(r, E)                                   # consume element 2 → measure steady stride
    stride = _buf_pos(src) - second_pos
    _buf_pos!(src, second_pos + (n - 1) * stride) # advance past the remaining elements
    return CDRArrayView{E, IsCDR2, LE, S}(mem, origin, kind, first_pos, second_pos, stride, n)
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
an owned copy. Obtain one via `view(r, CDRString)` from a reader, or
[`reinterpret_string`](@ref) over a raw buffer.
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

# Internal: view a CDR string over the reader's buffer, advancing past the
# content and null terminator. Public reader spelling is `view(r, CDRString)`.
function _view_string(r::CDRReader{B}) where {B <: _CDRBufLike}
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
    iscompact(::Type{T}; cdr2=false) -> Bool

Reader-less form: whether `T`'s in-memory layout matches its CDR wire encoding
under the given variant (host endianness assumed), i.e. whether it can be read
or written as a single `unsafe_load`/`unsafe_store!`. A compile-time constant.
"""
@inline iscompact(::Type{T}; cdr2::Bool=false) where T = _iscompact_host(T, Val(cdr2))
@generated function _iscompact_host(::Type{T}, ::Val{C}) where {T, C}
    return :($(_is_compact_struct(T, C, ENDIAN_BOM == 0x04030201)))
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

# --- Layout introspection: make the capability hierarchy visible -----------
#
# Serializability, single-load/store, and zero-copy viewability are three
# *nested* properties, and which ones a type has is otherwise invisible at the
# call site — especially when the type came out of `@cdr_fixed`/`@cdr_compact`.
# `cdr_layout(T)` reports all three plus the *reason* a struct is not compact
# (trailing pad vs. leading-narrow leaf), so the distinction is discoverable in
# the REPL instead of surfacing as a `view` that throws at runtime.

"""
    CDRLayout

The result of [`cdr_layout`](@ref): a readable report of which CDR capabilities
a type has, beyond plain serialization (every concrete struct reads/writes as
standard CDR via field-walk). The capabilities, which **nest**
`viewable ⟹ compact ⟹ fixed` — and which mirror the macros
`@cdr_fixed` / `@cdr_compact`:

  * `fixed`     — no variable-length fields, so a constant compile-time wire size
    (the [`@cdr_fixed`](@ref) tier; a sequence of these is `CDRArrayView`-able).
  * `compact`   — `read`/`write` is a single `unsafe_load`/`unsafe_store!`
    (i.e. [`iscompact`](@ref)).
  * `viewable`  — `view(r, CDRArray{T})` can zero-copy alias a sequence.

`why` explains the layout — in particular *why* a `fixed` struct is not
compact (trailing padding vs. a leading field narrower than its max alignment).
"""
struct CDRLayout
    type::Any
    cdr2::Bool
    fixed::Bool
    compact::Bool
    viewable::Bool
    why::String
end

function _cdr_layout_why(::Type{T}, cdr2::Bool, host::Bool, single::Bool) where T
    if _is_packed_leaf(T)
        return T === Char ? "primitive leaf (Char is 4 bytes in Julia, 1 on the wire — not array-viewable)" :
                            "primitive / SArray-of-primitive leaf — its own bytes are the wire bytes"
    end
    (T <: AbstractString || T <: AbstractArray) &&
        return "variable-length sequence — length-prefixed, decoded element-by-element"
    isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0 ||
        return "not a concrete serializable type"
    single && return "compact: Julia in-memory layout is bit-identical to the CDR wire layout"
    _is_cdr1_compat_type(T) || return "has variable-length or non-isbits fields — field-walk only, never a single blob or view"
    # Flat (standard CDR1) but not compact: say which condition failed.
    if !host
        return "flat standard CDR1; would be compact but the host is not little-endian"
    elseif _struct_cdr_size(T, cdr2) != sizeof(T)
        return string("flat standard CDR1; field-walked because Julia adds trailing padding ",
                      "(sizeof ", sizeof(T), " ≠ CDR data size ", _struct_cdr_size(T, cdr2),
                      ") — not a single blob, not array-viewable")
    elseif _first_leaf_align(T, cdr2) != _wa_align_for(T, cdr2)
        return string("flat standard CDR1; field-walked because it leads with a field aligned to ",
                      _first_leaf_align(T, cdr2), " < the struct's max alignment ", _wa_align_for(T, cdr2),
                      " — its sequence elements aren't uniformly strided, so not array-viewable")
    else
        return "flat standard CDR1, field-walked"
    end
end

"""
    cdr_layout(::Type{T}; cdr2=false) -> CDRLayout

Report `T`'s CDR capabilities — does it encode as standard CDR1, read/write as a
single `unsafe_load`/`unsafe_store!`, and can `view(r, CDRArray{T})` alias it —
and, for a struct that *isn't* compact, *why* (trailing padding vs. a
leading field narrower than the struct's max alignment). Host endianness is
assumed (as for the reader-less [`iscompact`](@ref)).

Use it to see, at a glance, which tier a type produced by `@cdr_fixed` /
`@cdr_compact` lands in: the capabilities nest `viewable ⟹ compact ⟹ fixed`,
and being fixed-size (or merely serializable) does **not** imply compact or
viewable.

```julia
julia> cdr_layout(MyMsg)   # prints a capability summary + the reason
```
"""
function cdr_layout(::Type{T}; cdr2::Bool=false) where T
    host = (ENDIAN_BOM == 0x04030201)
    compact = _is_packed_leaf(T) || (host && _is_compact_struct(T, cdr2, host))
    viewable = _is_view_eltype(T, cdr2, host)
    fixed = _is_packed_leaf(T) || _is_cdr1_compat_type(T)
    return CDRLayout(T, cdr2, fixed, compact, viewable,
                     _cdr_layout_why(T, cdr2, host, compact))
end

function Base.show(io::IO, ::MIME"text/plain", l::CDRLayout)
    println(io, "CDRLayout(", l.type, l.cdr2 ? "; XCDR2)" : "; CDR1)")
    _yn(b) = b ? "yes" : "no "
    println(io, "  fixed (@cdr_fixed) : ", _yn(l.fixed),    "   (no var-length fields; O(1) CDRArrayView decoding)")
    println(io, "  compact (iscompact): ", _yn(l.compact),  "   (read/write is one unsafe op)")
    println(io, "  CDRArray viewable  : ", _yn(l.viewable), "   (view(r, CDRArray{T}) can zero-copy alias)")
    print(io,   "  → ", l.why)
end

Base.show(io::IO, l::CDRLayout) =
    print(io, "CDRLayout(", l.type, ": fixed=", l.fixed,
          ", compact=", l.compact, ", viewable=", l.viewable, ")")

"""
    view(r::CDRReader, ::Type{CDRArray{T}}) -> CDRArray{T}
    view(r::CDRReader, ::Type{CDRString})   -> CDRString
    view(r::CDRReader, ::Type{ViewStruct})  -> ViewStruct   # a @cdr_view struct

Read the next value as a zero-copy view aliasing the reader's buffer,
advancing the cursor. Strict: it errors if the view isn't possible (use
[`canview`](@ref) to branch, or [`read`](@ref) for an owned copy). Mirrors
`Base.view` — asking for a view is a guarantee, not a hint that may silently
fall back to a copy. For a `@cdr_view` struct, `view(r, T)` is an alias
of `read(r, T)` (the struct's view fields already alias the buffer).
"""
function Base.view(r::CDRReader, ::Type{V}) where {T, V <: CDRArray{T}}
    canview(r, V) || throw(ArgumentError(string("view: cannot alias ", V,
        " for this reader — element ", T, " ",
        cdr_layout(T; cdr2=isCDR2(r)).why,
        ". Use read(r, Vector{", T, "}) for an owned copy, or check canview(r, ", V, ") first.")))
    return _view_array(r, T)
end
function Base.view(r::CDRReader, ::Type{CDRString})
    canview(r, CDRString) || throw(ArgumentError(
        "view: cannot alias CDRString (reader is not buffer-backed)"))
    return _view_string(r)
end

# `view(r, CDRArrayView{E})`: lazy *decoding* sequence view. Unlike
# `view(r, CDRArray{E})` it doesn't alias, so it works for any flat fixed-size
# `E` (not just compact) — `canview` here only asks that `E` be flat.
@generated function canview(r::CDRReader{B}, ::Type{V}) where {B, E, V <: CDRArrayView{E}}
    return :($(B <: _CDRBufLike && _is_cdr1_compat_type(E)))
end
function Base.view(r::CDRReader, ::Type{V}) where {E, V <: CDRArrayView{E}}
    canview(r, V) || throw(ArgumentError(string("view: ", V,
        " needs a flat fixed-size element and a buffer-backed reader — ", E, " ",
        cdr_layout(E; cdr2=isCDR2(r)).why, ". Use read(r, Vector{", E, "}) instead.")))
    return _seq_view(r, E)
end

# --- Nominal struct view ----------------------------------------------------
#
# `read_view(r, T)` / `view(r, T)` return a `CDRView{T}` — a nominal wrapper that
# derives `propertynames`/`getproperty`/`==`/`show` from `T` — rather than a bare
# NamedTuple. Being nominal it is `isa CDRView{T}`-dispatchable and nests for free
# (a nested struct field becomes a `CDRView{Nested}`). Each variable-length field
# is substituted with its zero-copy view (`String` → `CDRString`, `Vector{E}` →
# `CDRArray{E}`); scalar and fixed-size fields decode to ordinary values; fields
# whose elements can't be aliased fall back to an owned decode. The owned struct
# `T` keeps its `String`/`Vector` fields for construction and publishing.

"""
    CDRView{T} <: Any

A lazy, nominal view of a struct `T` over a reader's buffer, produced by
[`read_view`](@ref) / `view(r, T)`. Field access mirrors `T`
(`v.field`, `propertynames`, `==`, `show`), but variable-length fields are
zero-copy views: a `String` field reads as a [`CDRString`](@ref) and a
`Vector{E}` field as a [`CDRArray{E}`](@ref). Nested struct fields are
themselves `CDRView`s. Compares equal to another `CDRView{T}` and to an owned
`T` of the same field values.
"""
struct CDRView{T, NT <: NamedTuple}
    fields::NT
end

CDRView{T}(fields::NT) where {T, NT <: NamedTuple} = CDRView{T, NT}(fields)

Base.propertynames(::CDRView{T}) where T = fieldnames(T)
@inline Base.getproperty(v::CDRView, name::Symbol) = getfield(getfield(v, :fields), name)

# Two views of the same struct compare field-wise; a view also compares equal to
# an owned `T` (CDRString == String and CDRArray == Vector both compare by value,
# and a nested CDRView vs nested owned struct recurses through these methods).
function Base.:(==)(a::CDRView{T}, b::CDRView{T}) where T
    af = getfield(a, :fields); bf = getfield(b, :fields)
    for n in fieldnames(T)
        getfield(af, n) == getfield(bf, n) || return false
    end
    return true
end
function Base.:(==)(a::CDRView{T}, b::T) where T
    af = getfield(a, :fields)
    for n in fieldnames(T)
        getfield(af, n) == getfield(b, n) || return false
    end
    return true
end
Base.:(==)(a::T, b::CDRView{T}) where T = b == a

function Base.show(io::IO, v::CDRView{T}) where T
    print(io, "CDRView{", T, "}(")
    first = true
    for n in fieldnames(T)
        first || print(io, ", ")
        first = false
        print(io, n, "=", repr(getfield(getfield(v, :fields), n)))
    end
    print(io, ")")
end

# --- materialize: copy a view out of the buffer into a fully-owned value ----
#
# A `CDRView{T}` and its `CDRString`/`CDRArray`/`CDRArrayView` fields alias the
# reader's buffer. `materialize` walks the view and rebuilds an owned `T` whose
# tree holds no view wrappers and no reference to the buffer, so the value can
# outlive the bytes it was read from (e.g. a borrowed Zenoh `Sample`). Leaves
# copy by value: `collect` loads each array element out with `unsafe_load`,
# `String` copies the UTF-8 bytes. The owned struct is rebuilt through `T`'s
# positional constructor, which coerces each materialized field to its declared
# type. The catch-all leaves already-owned fields (primitives, SArrays, owned
# `Vector`s of strings/dynamic structs) untouched — they alias nothing.
"""
    materialize(v::CDRView{T}) -> T

Copy a [`CDRView`](@ref) out of the reader's buffer into a fully-owned `T`,
recursively: `CDRString` → `String`, `CDRArray`/`CDRArrayView` → `Vector`,
nested `CDRView` → owned struct. The result aliases nothing, so it stays valid
after the source buffer is freed or overwritten.
"""
materialize(v::CDRView{T}) where T =
    T(map(materialize, values(getfield(v, :fields)))...)
materialize(s::CDRString)    = String(s)
materialize(a::CDRArray)     = collect(a)
materialize(a::CDRArrayView) = collect(a)
materialize(x) = x

# Read one field for `read_view`. Generated rather than a plain `if`-chain
# because the branch predicates (`_is_view_eltype`/`_is_cdr1_compat_type`/
# `_is_packed_type`) return a runtime `Bool` that inference does NOT
# constant-fold, even though their inputs (`FT`/`IsCDR2`/`LE`) are all known at
# compile time. A value-level chain would therefore infer as the *union* of
# every branch's result, degrading the enclosing `CDRView`'s field types to a
# `Union` and forcing the whole view onto the heap. Deciding the branch here at
# expansion time emits a single concrete call per field type. Fixed-size parts
# (primitives, SArrays, all-fixed structs) decode to ordinary values; dynamic
# CDR sequences become zero-copy `CDRArray` views; nested structs that
# themselves contain sequences recurse. Sequences whose elements can't be
# aliased (strings, jagged structs) fall back to a normal decode.
@generated function _read_view_field(r::CDRReader{B, IsCDR2, LE}, ::Type{FT}) where {B, IsCDR2, LE, FT}
    if FT <: StaticArray || _is_packed_type(FT)
        return :(read(r, $FT))
    elseif FT <: AbstractString
        return :(_view_string(r))           # zero-copy CDRString view
    elseif FT <: AbstractVector
        ET = eltype(FT)
        if _is_view_eltype(ET, IsCDR2, LE)
            return :(_view_array(r, $ET))   # compact element → zero-copy CDRArray alias
        elseif _is_cdr1_compat_type(ET)
            return :(_seq_view(r, $ET))     # flat (but not compact) element → lazy decoding CDRArrayView
        else
            return :(read(r, $FT))          # variable-length element → owned decode
        end
    elseif isstructtype(FT) && isconcretetype(FT) && fieldcount(FT) > 0
        return :(read_view(r, $FT))
    else
        return :(read(r, $FT))
    end
end

"""
    read_view(r::CDRReader, ::Type{T}) -> CDRView{T}

Read a struct `T` as a lazy view: scalar and fixed-size fields are decoded to
ordinary values, but each variable-length sequence field becomes a zero-copy
[`CDRArray`](@ref) aliasing the reader's buffer — so the elements are never
touched or copied — and each `String` field a zero-copy [`CDRString`](@ref).
Nested structs recurse to nested views. The result is a nominal
[`CDRView{T}`](@ref) carrying `T`'s field names, accessed the same way
(`v.field`, `v.seq[i]`) and `isa CDRView{T}`-dispatchable; `T` itself is
unchanged.

Sequence fields whose elements can't be aliased (strings, or jagged/dynamic
element structs) fall back to a normal decode. Only IOBuffer-/MemBuf-backed
readers are supported.
"""
# Generated so the fields lower to a straight-line, left-to-right (in wire
# order) sequence of `_read_view_field` calls on each concrete `fieldtype(T,i)`
# — giving the `CDRView` a fully concrete `NamedTuple` type, so it returns by
# value and allocates nothing (see `_read_view_field` for why a value-level
# loop would not).
@generated function read_view(r::CDRReader{B, IsCDR2, LE},
                              ::Type{T}) where {B <: _CDRBufLike, IsCDR2, LE, T}
    (isstructtype(T) && isconcretetype(T) && fieldcount(T) > 0) ||
        return :(throw(ArgumentError(string("read_view: ", $T,
                                            " is not a concrete struct with fields"))))
    calls = [:(_read_view_field(r, $(fieldtype(T, i)))) for i in 1:fieldcount(T)]
    return :(CDRView{$T}(NamedTuple{$(fieldnames(T))}(tuple($(calls...)))))
end

# `view(r, T)` for a plain struct is an alias of `read_view(r, T)`. The reader
# is left unconstrained to match the `CDRArray`/`CDRString` view methods (so
# this stays strictly less specific than all of them — no dispatch ambiguity);
# a non-buffer-backed reader errors in `read_view`, which requires one.
Base.view(r::CDRReader, ::Type{T}) where T = read_view(r, T)

# --- @cdr_view view-struct support -----------------------------------------
#
# A `@cdr_view` struct gets the buffer-backed *view* treatment per field: a
# field declared `CDRString` or `CDRArray{ElementType}` becomes a zero-copy view
# over the reader's buffer; every other field is read as an ordinary owned
# value. The detection below is purely syntactic (on the field-type ASTs), so a
# plain `Vector` is never silently turned into a view. (`@cdr_compact`'s
# deprecated view mode reuses these helpers.) Run at macro-expansion time.

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
            error("@cdr_view: a CDRArray field must be written `CDRArray{ElementType}`")
        return (:array, te.args[2])
    end
    return (:plain, nothing)
end

# True if any field opts into a view (`CDRString`/`CDRArray{…}`) — the signal
# `@cdr_compact` uses to route its deprecated view mode to `_cdr_view_emit`.
_cdr_has_view_fields(f_type_exprs) =
    any(te -> _cdr_view_field_class(te)[1] !== :plain, f_type_exprs)

# Build the escaped definition block for an opt-in view struct.
function _cdr_view_emit(name_sym::Symbol, f_names::Vector{Symbol}, f_type_exprs::Vector)
    cdrarray   = GlobalRef(@__MODULE__, :CDRArray)
    cdrstring  = GlobalRef(@__MODULE__, :CDRString)
    ri_string  = GlobalRef(@__MODULE__, :_view_string)
    ri_array   = GlobalRef(@__MODULE__, :_view_array)
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

        # `view` alias for consistency with `view(r, CDRArray{T})` /
        # `view(r, CDRString)`: a view struct's view fields alias the buffer.
        Base.view(_r::$reader{B}, ::Type{$name_sym}) where {B <: $buflike} =
            Base.read(_r, $name_sym)

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
