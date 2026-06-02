def LFIDummyInst : Instruction {
  let OutOperandList = (outs);
  let InOperandList = (ins);
  let isPseudo = 1;
  let isCodeGenOnly = 1;
  let hasNoSchedulingInfo = 1;
}

class LFIInstruction {
  Instruction LFIInst = LFIDummyInst;

  // Generic memory operand layout (MemInfo Table)
  bit         IsLFIMem      = 0;
  bits<4>     BaseIdx       = 0;
  bits<4>     OffsetIdx     = 0;
  bit         HasOffset     = 0;
  bit         IsPrePost     = 0;
  bit         IsLiteral     = 0;
}

class LFILoadStoreRO<bits<2> sz, bit V> : LFIInstruction {
  let LFIInst = !cast<Instruction>(NAME);
  let IsLFIMem = 1;
  let BaseIdx = 1;
  let OffsetIdx = 2;
  let HasOffset = 1;
}

class LFILaneBaseStore : LFIInstruction {
  let LFIInst = !cast<Instruction>(NAME);
  let IsLFIMem = 1;
  let BaseIdx = 2;
}

class LFILaneBaseLoadTied : LFIInstruction {
  let LFIInst = !cast<Instruction>(NAME);
  let IsLFIMem = 1;
  let BaseIdx = 3;
}

class LFILanePostStore<int scale> : LFIInstruction {
  let LFIInst = !cast<Instruction>(NAME);
  let IsLFIMem = 1;
  let BaseIdx = 3;
  let OffsetIdx = 4;
  let HasOffset = 1;
  let IsPrePost = 1;
}

class LFILanePostLoadTied<int scale> : LFIInstruction {
  let LFIInst = !cast<Instruction>(NAME);
  let IsLFIMem = 1;
  let BaseIdx = 4;
  let OffsetIdx = 5;
  let HasOffset = 1;
  let IsPrePost = 1;
}

// Explicit Type-Safe LFI Mapping Table Entry Definitions
class LFIAddrMode<int v> { int Value = v; }
def LFI_AM_Ui   : LFIAddrMode<0>;
def LFI_AM_RoW  : LFIAddrMode<1>;
def LFI_AM_RoX  : LFIAddrMode<2>;
def LFI_AM_Pre  : LFIAddrMode<3>;
def LFI_AM_Post : LFIAddrMode<4>;

class LFIVariantEntry<Instruction inst, LFIAddrMode mode, bits<3> log2size, Instruction row> {
  Instruction LFIInst = inst;
  bits<3> AddrMode = mode.Value;
  bits<3> Log2Size = log2size;
  Instruction RoWInst = row;
}

class LFIPairVariantEntry<Instruction inst, bit isPre, bits<5> scale, Instruction base> {
  Instruction LFIInst = inst;
  bit IsPre = isPre;
  bits<5> Scale = scale;
  Instruction BaseInst = base;
}

class LFISIMDPostEntry<Instruction inst, bits<7> naturalOffset, Instruction base> {
  Instruction LFIInst = inst;
  bits<7> NaturalOffset = naturalOffset;
  Instruction BaseInst = base;
}
