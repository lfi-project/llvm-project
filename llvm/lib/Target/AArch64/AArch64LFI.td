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

// LFI addressing-mode enumeration (matches the rewriter's logical view).
// Values are exposed to C++ as plain integers.
class LFIAddrMode<int v> { int Value = v; }
def LFI_AM_Ui   : LFIAddrMode<0>;
def LFI_AM_RoW  : LFIAddrMode<1>;
def LFI_AM_RoX  : LFIAddrMode<2>;
def LFI_AM_Pre  : LFIAddrMode<3>;
def LFI_AM_Post : LFIAddrMode<4>;

//===----------------------------------------------------------------------===//
// Searchable Tables
//===----------------------------------------------------------------------===//

def LFIVariantTable : GenericTable {
  let FilterClass    = "LFIInstruction";
  let FilterClassField = "HasLFIVariant";
  let CppTypeName    = "LFIVariantEntry";
  let Fields         = ["LFIInst", "AddrMode", "Log2Size", "RoWInst"];
  let PrimaryKey     = ["LFIInst"];
  let PrimaryKeyName = "lookupLFIVariantByOpcode";
}

def PairVariantTable : GenericTable {
  let FilterClass    = "LFIInstruction";
  let FilterClassField = "HasPairVariant";
  let CppTypeName    = "PairVariantEntry";
  let Fields         = ["LFIInst", "IsPre", "Scale", "BaseInst"];
  let PrimaryKey     = ["LFIInst"];
  let PrimaryKeyName = "lookupPairVariantByOpcode";
}

def SIMDPostTable : GenericTable {
  let FilterClass    = "LFIInstruction";
  let FilterClassField = "HasSIMDPost";
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
                LDARB, LDARH, LDARW, LDARX,
                LDLARB, LDLARH, LDLARW, LDLARX,
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
