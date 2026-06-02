//===-- AArch64LFI.td - LFI rewriter metadata --------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Per-opcode metadata consumed by AArch64MCLFIRewriter.
//
//===----------------------------------------------------------------------===//


//===----------------------------------------------------------------------===//
// Searchable Tables
//===----------------------------------------------------------------------===//

def LFIVariantTable : GenericTable {
  let FilterClass    = "LFIVariantEntry";
  let CppTypeName    = "LFIVariantEntry";
  let Fields         = ["LFIInst", "AddrMode", "Log2Size", "RoWInst"];
  let PrimaryKey     = ["LFIInst"];
  let PrimaryKeyName = "lookupLFIVariantByOpcode";
}

def PairVariantTable : GenericTable {
  let FilterClass    = "LFIPairVariantEntry";
  let CppTypeName    = "PairVariantEntry";
  let Fields         = ["LFIInst", "IsPre", "Scale", "BaseInst"];
  let PrimaryKey     = ["LFIInst"];
  let PrimaryKeyName = "lookupPairVariantByOpcode";
}

def SIMDPostTable : GenericTable {
  let FilterClass    = "LFISIMDPostEntry";
  let CppTypeName    = "SIMDPostEntry";
  let Fields         = ["LFIInst", "NaturalOffset", "BaseInst"];
  let PrimaryKey     = ["LFIInst"];
  let PrimaryKeyName = "lookupSIMDPostByOpcode";
}

def MemInfoTable : GenericTable {
  let FilterClass    = "LFIInstruction";
  let FilterClassField = "IsLFIMem";
  let CppTypeName    = "MemInfoEntry";
  let Fields         = ["LFIInst", "BaseIdx", "OffsetIdx", "HasOffset",
                        "IsPrePost", "IsLiteral"];
  let PrimaryKey     = ["LFIInst"];
  let PrimaryKeyName = "lookupMemInfoByOpcode";
}

//===----------------------------------------------------------------------===//
// MemInfo helper subclasses (used to explicitly map CAS/LSE/exclusive ops)
//===----------------------------------------------------------------------===//

class MemInfo<Instruction inst, bits<4> baseIdx, bits<4> offsetIdx,
              bit hasOffset, bit isPrePost, bit isLiteral> : LFIInstruction {
  let LFIInst   = inst;
  let IsLFIMem  = 1;
  let BaseIdx   = baseIdx;
  let OffsetIdx = offsetIdx;
  let HasOffset = hasOffset;
  let IsPrePost = isPrePost;
  let IsLiteral = isLiteral;
}

class MemLiteral<Instruction inst, int baseIdx>
  : MemInfo<inst, baseIdx, 0, 0, 0, 1>;
class MemNoOffset<Instruction inst, int baseIdx>
  : MemInfo<inst, baseIdx, 0, 0, 0, 0>;
class MemIndexed<Instruction inst, int baseIdx, int offsetIdx>
  : MemInfo<inst, baseIdx, offsetIdx, 1, 0, 0>;
class MemPrePost<Instruction inst, int baseIdx, int offsetIdx>
  : MemInfo<inst, baseIdx, offsetIdx, 1, 1, 0>;

//===----------------------------------------------------------------------===//
// Unscaled-immediate scalar loads/stores (LDUR/STUR/PRFUM). Same operand
// layout as the indexed forms - BaseIdx=1, OffsetIdx=2.
//===----------------------------------------------------------------------===//

foreach inst = [LDURBBi, LDURBi, LDURDi, LDURHHi, LDURHi, LDURQi,
                LDURSBWi, LDURSBXi, LDURSHWi, LDURSHXi, LDURSWi, LDURSi,
                LDURWi, LDURXi,
                STURBBi, STURBi, STURDi, STURHHi, STURHi, STURQi,
                STURSi, STURWi, STURXi,
                PRFUMi] in
  def : MemIndexed<inst, 1, 2>;

//===----------------------------------------------------------------------===//
// Non-temporal pair indexed forms (LDNP/STNP). Same operand layout as `i`.
//===----------------------------------------------------------------------===//

foreach inst = [LDNPDi, LDNPQi, LDNPSi, LDNPWi, LDNPXi,
                STNPDi, STNPQi, STNPSi, STNPWi, STNPXi] in
  def : MemIndexed<inst, 2, 3>;

//===----------------------------------------------------------------------===//
// PC-relative literal loads: Rt, label. BaseIdx=0, no offset operand.
//===----------------------------------------------------------------------===//

foreach inst = [LDRWl, LDRXl, LDRSWl, LDRSl, LDRDl, LDRQl, PRFMl] in
  def : MemLiteral<inst, 0>;

//===----------------------------------------------------------------------===//
// Exclusive / acquire / RCPC loads and store-release: Rt, [Rn].
//===----------------------------------------------------------------------===//

foreach inst = [LDXRB, LDXRH, LDXRW, LDXRX,
                LDAXRB, LDAXRH, LDAXRW, LDAXRX,
                LDLARB, LDLARH, LDLARW, LDLARX,
                LDARB, LDARH, LDARW, LDARX,
                // RCPC
                LDAPRB, LDAPRH, LDAPRW, LDAPRX,
                // Store-release
                STLRB, STLRH, STLRW, STLRX,
                STLLRB, STLLRH, STLLRW, STLLRX] in
  def : MemNoOffset<inst, 1>;

//===----------------------------------------------------------------------===//
// Exclusive stores: Ws, Rt, [Rn]. Exclusive pair loads: Rt, Rt2, [Rn].
//===----------------------------------------------------------------------===//

foreach inst = [STXRB, STXRH, STXRW, STXRX,
                STLXRB, STLXRH, STLXRW, STLXRX,
                LDXPW, LDXPX, LDAXPW, LDAXPX] in
  def : MemNoOffset<inst, 2>;

// Exclusive store pairs: Ws, Rt, Rt2, [Rn].
foreach inst = [STXPW, STXPX, STLXPW, STLXPX] in
  def : MemNoOffset<inst, 3>;

//===----------------------------------------------------------------------===//
// CAS variants: Rs(out), Rs(in, tied), Rt, [Rn]. BaseIdx=3.
//===----------------------------------------------------------------------===//

defvar AArch64CASBases = ["CAS", "CASA", "CASL", "CASAL"];
defvar AArch64CASSuffixes = ["B", "H", "W", "X"];

foreach base = AArch64CASBases in
  foreach suff = AArch64CASSuffixes in
    def : MemNoOffset<!cast<Instruction>(base # suff), 3>;

// CASP variants: Ws_Ws2(out), Ws_Ws2(in, tied), Wt_Wt2, [Rn]. BaseIdx=3.
foreach inst = [CASPW, CASPX, CASPAW, CASPAX,
                CASPLW, CASPLX, CASPALW, CASPALX] in
  def : MemNoOffset<inst, 3>;

//===----------------------------------------------------------------------===//
// LSE atomics: Rs, Rt, [Rn].
//===----------------------------------------------------------------------===//

defvar AArch64LSEBases = [
  "LDADD", "LDCLR", "LDEOR", "LDSET",
  "LDSMAX", "LDSMIN", "LDUMAX", "LDUMIN", "SWP",
];
defvar AArch64LSESuffixes = [
  "B",   "H",   "W",   "X",
  "AB",  "AH",  "AW",  "AX",
  "LB",  "LH",  "LW",  "LX",
  "ALB", "ALH", "ALW", "ALX",
];

foreach base = AArch64LSEBases in
  foreach suff = AArch64LSESuffixes in
    def : MemNoOffset<!cast<Instruction>(base # suff), 2>;

//===----------------------------------------------------------------------===//
// GPR / FPR / SIMD Scalar Families explicit mapping
//===----------------------------------------------------------------------===//

multiclass LFIFamily<string prefix, bits<3> log2size> {
  def : LFIVariantEntry<!cast<Instruction>(prefix # "ui"),   LFI_AM_Ui,   log2size, !cast<Instruction>(prefix # "roW")>;
  def : LFIVariantEntry<!cast<Instruction>(prefix # "roW"),  LFI_AM_RoW,  log2size, !cast<Instruction>(prefix # "roW")>;
  def : LFIVariantEntry<!cast<Instruction>(prefix # "roX"),  LFI_AM_RoX,  log2size, !cast<Instruction>(prefix # "roW")>;
  def : LFIVariantEntry<!cast<Instruction>(prefix # "pre"),  LFI_AM_Pre,  log2size, !cast<Instruction>(prefix # "roW")>;
  def : LFIVariantEntry<!cast<Instruction>(prefix # "post"), LFI_AM_Post, log2size, !cast<Instruction>(prefix # "roW")>;
}

defm : LFIFamily<"LDRW", 2>;
defm : LFIFamily<"LDRX", 3>;
defm : LFIFamily<"LDRBB", 0>;
defm : LFIFamily<"LDRB", 0>;
defm : LFIFamily<"LDRHH", 1>;
defm : LFIFamily<"LDRH", 1>;
defm : LFIFamily<"LDRS", 2>;
defm : LFIFamily<"LDRD", 3>;
defm : LFIFamily<"LDRQ", 4>;
defm : LFIFamily<"LDRSBW", 0>;
defm : LFIFamily<"LDRSBX", 0>;
defm : LFIFamily<"LDRSHW", 1>;
defm : LFIFamily<"LDRSHX", 1>;
defm : LFIFamily<"LDRSW", 2>;

defm : LFIFamily<"STRW", 2>;
defm : LFIFamily<"STRX", 3>;
defm : LFIFamily<"STRBB", 0>;
defm : LFIFamily<"STRB", 0>;
defm : LFIFamily<"STRHH", 1>;
defm : LFIFamily<"STRH", 1>;
defm : LFIFamily<"STRS", 2>;
defm : LFIFamily<"STRD", 3>;
defm : LFIFamily<"STRQ", 4>;

// Prefetch families (no pre/post index forms)
multiclass LFIPrefetchFamily<string prefix> {
  def : LFIVariantEntry<!cast<Instruction>(prefix # "ui"),  LFI_AM_Ui,  3, !cast<Instruction>(prefix # "roW")>;
  def : LFIVariantEntry<!cast<Instruction>(prefix # "roW"), LFI_AM_RoW, 3, !cast<Instruction>(prefix # "roW")>;
  def : LFIVariantEntry<!cast<Instruction>(prefix # "roX"), LFI_AM_RoX, 3, !cast<Instruction>(prefix # "roW")>;
}

defm : LFIPrefetchFamily<"PRFM">;

//===----------------------------------------------------------------------===//
// Load/Store Pair Families explicit mapping
//===----------------------------------------------------------------------===//

multiclass LFIPairFamily<string base, bits<5> scale> {
  def : LFIPairVariantEntry<!cast<Instruction>(base # "pre"),  1, scale, !cast<Instruction>(base # "i")>;
  def : LFIPairVariantEntry<!cast<Instruction>(base # "post"), 0, scale, !cast<Instruction>(base # "i")>;
}

defm : LFIPairFamily<"LDPD", 8>;
defm : LFIPairFamily<"LDPQ", 16>;
defm : LFIPairFamily<"LDPS", 4>;
defm : LFIPairFamily<"LDPW", 4>;
defm : LFIPairFamily<"LDPX", 8>;

defm : LFIPairFamily<"STPD", 8>;
defm : LFIPairFamily<"STPQ", 16>;
defm : LFIPairFamily<"STPS", 4>;
defm : LFIPairFamily<"STPW", 4>;
defm : LFIPairFamily<"STPX", 8>;

//===----------------------------------------------------------------------===//
// SIMD families explicit mapping
//===----------------------------------------------------------------------===//

class SIMDPostFam<string b, int off> {
  string Base = b;
  int Offset = off;
}

// SIMD lane stores.
defvar AArch64SIMDLaneStoreFams = [
  SIMDPostFam<"ST1i8",  1>, SIMDPostFam<"ST1i16",  2>,
  SIMDPostFam<"ST1i32", 4>, SIMDPostFam<"ST1i64",  8>,
  SIMDPostFam<"ST2i8",  2>, SIMDPostFam<"ST2i16",  4>,
  SIMDPostFam<"ST2i32", 8>, SIMDPostFam<"ST2i64", 16>,
  SIMDPostFam<"ST3i8",  3>, SIMDPostFam<"ST3i16",  6>,
  SIMDPostFam<"ST3i32",12>, SIMDPostFam<"ST3i64", 24>,
  SIMDPostFam<"ST4i8",  4>, SIMDPostFam<"ST4i16",  8>,
  SIMDPostFam<"ST4i32",16>, SIMDPostFam<"ST4i64", 32>,
];

// SIMD lane loads.
defvar AArch64SIMDLaneLoadFams = [
  SIMDPostFam<"LD1i8",  1>, SIMDPostFam<"LD1i16",  2>,
  SIMDPostFam<"LD1i32", 4>, SIMDPostFam<"LD1i64",  8>,
  SIMDPostFam<"LD2i8",  2>, SIMDPostFam<"LD2i16",  4>,
  SIMDPostFam<"LD2i32", 8>, SIMDPostFam<"LD2i64", 16>,
  SIMDPostFam<"LD3i8",  3>, SIMDPostFam<"LD3i16",  6>,
  SIMDPostFam<"LD3i32",12>, SIMDPostFam<"LD3i64", 24>,
  SIMDPostFam<"LD4i8",  4>, SIMDPostFam<"LD4i16",  8>,
  SIMDPostFam<"LD4i32",16>, SIMDPostFam<"LD4i64", 32>,
];

// SIMD multi/replicate.
defvar AArch64SIMDMultiFams = [
  // Replicate loads.
  SIMDPostFam<"LD1Rv8b",  1>, SIMDPostFam<"LD1Rv16b",  1>,
  SIMDPostFam<"LD1Rv4h",  2>, SIMDPostFam<"LD1Rv8h",   2>,
  SIMDPostFam<"LD1Rv2s",  4>, SIMDPostFam<"LD1Rv4s",   4>,
  SIMDPostFam<"LD1Rv1d",  8>, SIMDPostFam<"LD1Rv2d",   8>,
  SIMDPostFam<"LD2Rv8b",  2>, SIMDPostFam<"LD2Rv16b",  2>,
  SIMDPostFam<"LD2Rv4h",  4>, SIMDPostFam<"LD2Rv8h",   4>,
  SIMDPostFam<"LD2Rv2s",  8>, SIMDPostFam<"LD2Rv4s",   8>,
  SIMDPostFam<"LD2Rv1d", 16>, SIMDPostFam<"LD2Rv2d",  16>,
  SIMDPostFam<"LD3Rv8b",  3>, SIMDPostFam<"LD3Rv16b",  3>,
  SIMDPostFam<"LD3Rv4h",  6>, SIMDPostFam<"LD3Rv8h",   6>,
  SIMDPostFam<"LD3Rv2s", 12>, SIMDPostFam<"LD3Rv4s",  12>,
  SIMDPostFam<"LD3Rv1d", 24>, SIMDPostFam<"LD3Rv2d",  24>,
  SIMDPostFam<"LD4Rv8b",  4>, SIMDPostFam<"LD4Rv16b",  4>,
  SIMDPostFam<"LD4Rv4h",  8>, SIMDPostFam<"LD4Rv8h",   8>,
  SIMDPostFam<"LD4Rv2s", 16>, SIMDPostFam<"LD4Rv4s",  16>,
  SIMDPostFam<"LD4Rv1d", 32>, SIMDPostFam<"LD4Rv2d",  32>,

  // LD1/ST1 One register.
  SIMDPostFam<"LD1Onev8b",  8>, SIMDPostFam<"LD1Onev16b", 16>,
  SIMDPostFam<"LD1Onev4h",  8>, SIMDPostFam<"LD1Onev8h",  16>,
  SIMDPostFam<"LD1Onev2s",  8>, SIMDPostFam<"LD1Onev4s",  16>,
  SIMDPostFam<"LD1Onev1d",  8>, SIMDPostFam<"LD1Onev2d",  16>,
  SIMDPostFam<"ST1Onev8b",  8>, SIMDPostFam<"ST1Onev16b", 16>,
  SIMDPostFam<"ST1Onev4h",  8>, SIMDPostFam<"ST1Onev8h",  16>,
  SIMDPostFam<"ST1Onev2s",  8>, SIMDPostFam<"ST1Onev4s",  16>,
  SIMDPostFam<"ST1Onev1d",  8>, SIMDPostFam<"ST1Onev2d",  16>,

  // LD1/ST1 Two registers.
  SIMDPostFam<"LD1Twov8b", 16>, SIMDPostFam<"LD1Twov16b", 32>,
  SIMDPostFam<"LD1Twov4h", 16>, SIMDPostFam<"LD1Twov8h",  32>,
  SIMDPostFam<"LD1Twov2s", 16>, SIMDPostFam<"LD1Twov4s",  32>,
  SIMDPostFam<"LD1Twov1d", 16>, SIMDPostFam<"LD1Twov2d",  32>,
  SIMDPostFam<"ST1Twov8b", 16>, SIMDPostFam<"ST1Twov16b", 32>,
  SIMDPostFam<"ST1Twov4h", 16>, SIMDPostFam<"ST1Twov8h",  32>,
  SIMDPostFam<"ST1Twov2s", 16>, SIMDPostFam<"ST1Twov4s",  32>,
  SIMDPostFam<"ST1Twov1d", 16>, SIMDPostFam<"ST1Twov2d",  32>,

  // LD1/ST1 Three registers.
  SIMDPostFam<"LD1Threev8b", 24>, SIMDPostFam<"LD1Threev16b", 48>,
  SIMDPostFam<"LD1Threev4h", 24>, SIMDPostFam<"LD1Threev8h",  48>,
  SIMDPostFam<"LD1Threev2s", 24>, SIMDPostFam<"LD1Threev4s",  48>,
  SIMDPostFam<"LD1Threev1d", 24>, SIMDPostFam<"LD1Threev2d",  48>,
  SIMDPostFam<"ST1Threev8b", 24>, SIMDPostFam<"ST1Threev16b", 48>,
  SIMDPostFam<"ST1Threev4h", 24>, SIMDPostFam<"ST1Threev8h",  48>,
  SIMDPostFam<"ST1Threev2s", 24>, SIMDPostFam<"ST1Threev4s",  48>,
  SIMDPostFam<"ST1Threev1d", 24>, SIMDPostFam<"ST1Threev2d",  48>,

  // LD1/ST1 Four registers.
  SIMDPostFam<"LD1Fourv8b", 32>, SIMDPostFam<"LD1Fourv16b", 64>,
  SIMDPostFam<"LD1Fourv4h", 32>, SIMDPostFam<"LD1Fourv8h",  64>,
  SIMDPostFam<"LD1Fourv2s", 32>, SIMDPostFam<"LD1Fourv4s",  64>,
  SIMDPostFam<"LD1Fourv1d", 32>, SIMDPostFam<"LD1Fourv2d",  64>,
  SIMDPostFam<"ST1Fourv8b", 32>, SIMDPostFam<"ST1Fourv16b", 64>,
  SIMDPostFam<"ST1Fourv4h", 32>, SIMDPostFam<"ST1Fourv8h",  64>,
  SIMDPostFam<"ST1Fourv2s", 32>, SIMDPostFam<"ST1Fourv4s",  64>,
  SIMDPostFam<"ST1Fourv1d", 32>, SIMDPostFam<"ST1Fourv2d",  64>,

  // LD2/ST2 Two registers.
  SIMDPostFam<"LD2Twov8b",  16>, SIMDPostFam<"LD2Twov16b", 32>,
  SIMDPostFam<"LD2Twov4h",  16>, SIMDPostFam<"LD2Twov8h",  32>,
  SIMDPostFam<"LD2Twov2s",  16>, SIMDPostFam<"LD2Twov4s",  32>,
  SIMDPostFam<"LD2Twov2d",  32>,
  SIMDPostFam<"ST2Twov8b",  16>, SIMDPostFam<"ST2Twov16b", 32>,
  SIMDPostFam<"ST2Twov4h",  16>, SIMDPostFam<"ST2Twov8h",  32>,
  SIMDPostFam<"ST2Twov2s",  16>, SIMDPostFam<"ST2Twov4s",  32>,
  SIMDPostFam<"ST2Twov2d",  32>,

  // LD3/ST3 Three registers.
  SIMDPostFam<"LD3Threev8b",  24>, SIMDPostFam<"LD3Threev16b", 48>,
  SIMDPostFam<"LD3Threev4h",  24>, SIMDPostFam<"LD3Threev8h",  48>,
  SIMDPostFam<"LD3Threev2s",  24>, SIMDPostFam<"LD3Threev4s",  48>,
  SIMDPostFam<"LD3Threev2d",  48>,
  SIMDPostFam<"ST3Threev8b",  24>, SIMDPostFam<"ST3Threev16b", 48>,
  SIMDPostFam<"ST3Threev4h",  24>, SIMDPostFam<"ST3Threev8h",  48>,
  SIMDPostFam<"ST3Threev2s",  24>, SIMDPostFam<"ST3Threev4s",  48>,
  SIMDPostFam<"ST3Threev2d",  48>,

  // LD4/ST4 Four registers.
  SIMDPostFam<"LD4Fourv8b",  32>, SIMDPostFam<"LD4Fourv16b", 64>,
  SIMDPostFam<"LD4Fourv4h",  32>, SIMDPostFam<"LD4Fourv8h",  64>,
  SIMDPostFam<"LD4Fourv2s",  32>, SIMDPostFam<"LD4Fourv4s",  64>,
  SIMDPostFam<"LD4Fourv2d",  64>,
  SIMDPostFam<"ST4Fourv8b",  32>, SIMDPostFam<"ST4Fourv16b", 64>,
  SIMDPostFam<"ST4Fourv4h",  32>, SIMDPostFam<"ST4Fourv8h",  64>,
  SIMDPostFam<"ST4Fourv2s",  32>, SIMDPostFam<"ST4Fourv4s",  64>,
  SIMDPostFam<"ST4Fourv2d",  64>,
];

foreach f = AArch64SIMDMultiFams in {
  defvar Base = !cast<Instruction>(f.Base);
  defvar Post = !cast<Instruction>(f.Base # "_POST");
  def : LFISIMDPostEntry<Post, f.Offset, Base>;
  def : MemNoOffset<Base, 1>;
}

foreach f = AArch64SIMDLaneStoreFams in {
  defvar Base = !cast<Instruction>(f.Base);
  defvar Post = !cast<Instruction>(f.Base # "_POST");
  def : LFISIMDPostEntry<Post, f.Offset, Base>;
  def : MemNoOffset<Base, 2>;
}

foreach f = AArch64SIMDLaneLoadFams in {
  defvar Base = !cast<Instruction>(f.Base);
  defvar Post = !cast<Instruction>(f.Base # "_POST");
  def : LFISIMDPostEntry<Post, f.Offset, Base>;
}
