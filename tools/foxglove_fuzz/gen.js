#!/usr/bin/env node
// Random-case generator: writes CDR bytes via @foxglove/cdr and emits
// each case as a line-based text format that verify.jl can parse.
//
// Schema (per case, in this order):
//   uint8, int8, bool-as-u8, uint16, int16, uint32, int32, uint64, int64,
//   float32, float64, string, Vector{u8}, Vector{f64}, Vector{string}
//
// Float bit patterns are emitted as hex so NaN/denormals round-trip.

const { CdrWriter, EncapsulationKind } = require("@foxglove/cdr");

// Mulberry32: deterministic, seedable, 32-bit PRNG.
function mulberry32(seed) {
  let s = seed >>> 0;
  return function () {
    s = (s + 0x6D2B79F5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function randInt(rng, lo, hi) {
  return Math.floor(rng() * (hi - lo + 1)) + lo;
}

function randU32(rng) {
  return Math.floor(rng() * 0x100000000) >>> 0;
}

function randBigU64(rng) {
  return (BigInt(randU32(rng)) << 32n) | BigInt(randU32(rng));
}

function f32FromBits(bits) {
  const buf = new ArrayBuffer(4);
  new Uint32Array(buf)[0] = bits >>> 0;
  return new Float32Array(buf)[0];
}

function f64FromBits(bits) {
  const buf = new ArrayBuffer(8);
  new DataView(buf).setBigUint64(0, bits, true);
  return new DataView(buf).getFloat64(0, true);
}

function f32Bits(f) {
  const buf = new ArrayBuffer(4);
  new Float32Array(buf)[0] = f;
  return new Uint32Array(buf)[0] >>> 0;
}

function f64Bits(f) {
  const buf = new ArrayBuffer(8);
  new DataView(buf).setFloat64(0, f, true);
  return new DataView(buf).getBigUint64(0, true);
}

// ECMA-262 permits implementations to substitute any NaN value when reading
// or writing floats, so a signaling NaN payload may not survive a round-trip
// through DataView.setFloat32 / setFloat64. To keep cross-impl bit patterns
// stable, force the "quiet" mantissa bit so we never generate signaling NaNs.
function canonicalF32Bits(bits) {
  bits = bits >>> 0;
  const exp = (bits >>> 23) & 0xFF;
  const mantissa = bits & 0x7FFFFF;
  if (exp === 0xFF && mantissa !== 0) {
    bits = (bits | (1 << 22)) >>> 0;
  }
  return bits;
}

function canonicalF64Bits(bits) {
  const exp = (bits >> 52n) & 0x7FFn;
  const mantissa = bits & ((1n << 52n) - 1n);
  if (exp === 0x7FFn && mantissa !== 0n) {
    bits = bits | (1n << 51n);
  }
  return bits;
}

function randString(rng, maxLen) {
  const n = randInt(rng, 0, maxLen);
  const ascii = [];
  for (let c = 0x21; c <= 0x7E; c++) ascii.push(c);
  const multi = [0xE9, 0xF1, 0x2603, 0x4E2D, 0x1F600];
  const cps = [];
  for (let i = 0; i < n; i++) {
    if (rng() < 0.7) cps.push(ascii[randInt(rng, 0, ascii.length - 1)]);
    else cps.push(multi[randInt(rng, 0, multi.length - 1)]);
  }
  return String.fromCodePoint(...cps);
}

function hex(buf) {
  return Buffer.from(buf).toString("hex");
}

function utf8(s) {
  return Buffer.from(s, "utf-8");
}

function genCase(seed) {
  const rng = mulberry32(seed);
  const le = rng() < 0.5;
  const kind = le ? EncapsulationKind.CDR_LE : EncapsulationKind.CDR_BE;
  const kindStr = le ? "CDR_LE" : "CDR_BE";

  const u8 = randInt(rng, 0, 255);
  const i8 = randInt(rng, -128, 127);
  const bool = randInt(rng, 0, 1);
  const u16 = randInt(rng, 0, 65535);
  const i16 = randInt(rng, -32768, 32767);
  const u32 = randU32(rng);
  const i32 = u32 | 0;
  const u64 = randBigU64(rng);
  const i64 = BigInt.asIntN(64, randBigU64(rng));
  const f32bits = canonicalF32Bits(randU32(rng));
  const f32val = f32FromBits(f32bits);
  const f64bits = canonicalF64Bits(randBigU64(rng));
  const f64val = f64FromBits(f64bits);
  const str = randString(rng, 20);
  const vu8N = randInt(rng, 0, 8);
  const vu8 = new Uint8Array(vu8N);
  for (let i = 0; i < vu8N; i++) vu8[i] = randInt(rng, 0, 255);
  const vf64N = randInt(rng, 0, 5);
  const vf64bits = [];
  const vf64 = new Float64Array(vf64N);
  for (let i = 0; i < vf64N; i++) {
    const b = canonicalF64Bits(randBigU64(rng));
    vf64bits.push(b);
    vf64[i] = f64FromBits(b);
  }
  const vstrN = randInt(rng, 0, 4);
  const vstr = [];
  for (let i = 0; i < vstrN; i++) vstr.push(randString(rng, 10));

  // Encode.
  // NOTE: @foxglove/cdr 3.5.0 has a bug in .string() — it uses value.length
  // (UTF-16 code units) as the byte count, truncating any multi-byte UTF-8
  // payload. We bypass it by writing strings via sequenceLength + uint8Array.
  function writeCDRString(w, s) {
    const b = Buffer.from(s, "utf-8");
    w.sequenceLength(b.length + 1);
    w.uint8Array(b, false);
    w.uint8(0);
  }

  const w = new CdrWriter({ kind });
  w.uint8(u8);
  w.int8(i8);
  w.uint8(bool);
  w.uint16(u16);
  w.int16(i16);
  w.uint32(u32);
  w.int32(i32);
  w.uint64(u64);
  w.int64(i64);
  w.float32(f32val);
  w.float64(f64val);
  writeCDRString(w, str);
  w.uint8Array(vu8, true);
  w.float64Array(vf64, true);
  w.sequenceLength(vstrN);
  for (const s of vstr) writeCDRString(w, s);

  // Emit
  const lines = [];
  lines.push(`CASE ${seed}`);
  lines.push(`KIND ${kindStr}`);
  lines.push(`U8 ${u8}`);
  lines.push(`I8 ${i8}`);
  lines.push(`BOOL ${bool}`);
  lines.push(`U16 ${u16}`);
  lines.push(`I16 ${i16}`);
  lines.push(`U32 ${u32}`);
  lines.push(`I32 ${i32}`);
  lines.push(`U64 ${u64.toString()}`);
  lines.push(`I64 ${i64.toString()}`);
  lines.push(`F32 ${f32bits.toString(16).padStart(8, "0")}`);
  lines.push(`F64 ${f64bits.toString(16).padStart(16, "0")}`);
  // Use "-" as a sentinel for empty hex so trimming whitespace never collapses
  // the surrounding tokens.
  const hexOrDash = (b) => b.length === 0 ? "-" : hex(b);

  const sb = utf8(str);
  lines.push(`STR ${sb.length} ${hexOrDash(sb)}`);
  lines.push(`VU8 ${vu8N} ${hexOrDash(vu8)}`);
  const vfHex = vf64bits.map(b => b.toString(16).padStart(16, "0")).join(" ");
  lines.push(`VF64 ${vf64N}${vf64N > 0 ? " " + vfHex : ""}`);
  const vstrTokens = vstr.flatMap(s => {
    const b = utf8(s);
    return [b.length.toString(), hexOrDash(b)];
  });
  lines.push(`VSTR ${vstrN}${vstrN > 0 ? " " + vstrTokens.join(" ") : ""}`);
  lines.push(`HEX ${hex(w.data)}`);
  lines.push(`END`);
  return lines.join("\n");
}

const N = parseInt(process.env.FUZZ_N || "300", 10);
const baseSeed = parseInt(process.env.FUZZ_SEED || "1", 10);

const out = [];
for (let i = 0; i < N; i++) {
  out.push(genCase(baseSeed + i));
}
process.stdout.write(out.join("\n") + "\n");
