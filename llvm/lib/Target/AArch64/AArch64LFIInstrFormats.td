def LFIDummyInst : Instruction {
  let OutOperandList = (outs);
  let InOperandList = (ins);
  let isPseudo = 1;
  let isCodeGenOnly = 1;
  let hasNoSchedulingInfo = 1;
}

class LFIInstruction {
  Instruction LFIInst = !cast<Instruction>(NAME);

  // Generic memory operand layout (MemInfo Table)
  bit         IsLFIMem      = 0;
  bits<4>     BaseIdx       = 0;
  bits<4>     OffsetIdx     = 0;
  bit         HasOffset     = 0;
  bit         IsPrePost     = 0;
  bit         IsLiteral     = 0;

  // Scalar variants (LFIVariant Table)
  bit         HasLFIVariant = 0;
  bits<3>     AddrMode      = 0;
  bits<3>     Log2Size      = !cond(
    !ne(NAME, !subst("LDRQ", "", !subst("STRQ", "", NAME))) : 4,
    !ne(NAME, !subst("LDRX", "", !subst("STRX", "", NAME))) : 3,
    !ne(NAME, !subst("LDRD", "", !subst("STRD", "", NAME))) : 3,
    !ne(NAME, !subst("LDRSB", "", NAME)) : 0,
    !ne(NAME, !subst("LDRSH", "", NAME)) : 1,
    !ne(NAME, !subst("LDRW", "", !subst("STRW", "", NAME))) : 2,
    !ne(NAME, !subst("LDRS", "", !subst("STRS", "", NAME))) : 2,
    !ne(NAME, !subst("LDRH", "", !subst("STRH", "", NAME))) : 1,
    1 : 0
  );
  Instruction RoWInst       = LFIDummyInst;

  // Pair variants (PairVariant Table)
  bit         HasPairVariant = 0;
  bit         IsPre          = 0;
  bits<5>     Scale          = 0;
  Instruction BaseInst       = LFIDummyInst;

  // SIMD Post-index (SIMDPost Table)
  bit         HasSIMDPost    = 0;
  bits<7>     NaturalOffset  = 0;
}

// Centralized, non-brittle Scalar Variant format helper (Option A)
class LFIScalarVariant : LFIInstruction {
  let HasLFIVariant = 1;
  let AddrMode = !cond(
    !ne(NAME, !subst("roW", "", NAME))  : 1, // LFI_AM_RoW
    !ne(NAME, !subst("roX", "", NAME))  : 2, // LFI_AM_RoX
    !ne(NAME, !subst("pre", "", NAME))  : 3, // LFI_AM_Pre
    !ne(NAME, !subst("post", "", NAME)) : 4, // LFI_AM_Post
    1 : 0                                    // LFI_AM_Ui
  );
  let RoWInst = !cast<Instruction>(
    !subst("ui", "",
    !subst("pre", "",
    !subst("post", "",
    !subst("roX", "",
    !subst("roW", "", NAME))))) # "roW"
  );
}

class LFILoadStoreRO<bits<2> sz, bit V> : LFIScalarVariant {
  let IsLFIMem = 1;
  let BaseIdx = 1;
  let OffsetIdx = 2;
  let HasOffset = 1;
}

class LFILaneBaseStore : LFIInstruction {
  let IsLFIMem = 1;
  let BaseIdx = 2;
}

class LFILaneBaseLoadTied : LFIInstruction {
  let IsLFIMem = 1;
  let BaseIdx = 3;
}

class LFILanePostStore<int scale> : LFIInstruction {
  let IsLFIMem = 1;
  let BaseIdx = 3;
  let OffsetIdx = 4;
  let HasOffset = 1;
  let IsPrePost = 1;
  let HasSIMDPost = 1;
  let BaseInst = !cast<Instruction>(!subst("_POST", "", NAME));
  let NaturalOffset = !mul(
    scale,
    !cond(
      !ne(NAME, !subst("LD2", "", !subst("ST2", "", NAME))) : 2,
      !ne(NAME, !subst("LD3", "", !subst("ST3", "", NAME))) : 3,
      !ne(NAME, !subst("LD4", "", !subst("ST4", "", NAME))) : 4,
      1 : 1
    )
  );
}

class LFILanePostLoadTied<int scale> : LFIInstruction {
  let IsLFIMem = 1;
  let BaseIdx = 4;
  let OffsetIdx = 5;
  let HasOffset = 1;
  let IsPrePost = 1;
  let HasSIMDPost = 1;
  let BaseInst = !cast<Instruction>(!subst("_POST", "", NAME));
  let NaturalOffset = !mul(
    scale,
    !cond(
      !ne(NAME, !subst("LD2", "", !subst("ST2", "", NAME))) : 2,
      !ne(NAME, !subst("LD3", "", !subst("ST3", "", NAME))) : 3,
      !ne(NAME, !subst("LD4", "", !subst("ST4", "", NAME))) : 4,
      1 : 1
    )
  );
}
