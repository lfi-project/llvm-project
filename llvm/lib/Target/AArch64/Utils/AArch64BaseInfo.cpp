//===-- AArch64BaseInfo.cpp - AArch64 Base encoding information------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides basic encoding and assembly information for AArch64.
//
//===----------------------------------------------------------------------===//
#include "AArch64BaseInfo.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/Regex.h"

using namespace llvm;

namespace llvm {
  namespace AArch64AT {
#define GET_ATsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}


namespace llvm {
  namespace AArch64DBnXS {
#define GET_DBnXSsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64DB {
#define GET_DBsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64DC {
#define GET_DCsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64IC {
#define GET_ICsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64ISB {
#define GET_ISBsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64TSB {
#define GET_TSBsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64PRFM {
#define GET_PRFMsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64SVEPRFM {
#define GET_SVEPRFMsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64RPRFM {
#define GET_RPRFMsList_IMPL
#include "AArch64GenSystemOperands.inc"
  } // namespace AArch64RPRFM
} // namespace llvm

namespace llvm {
  namespace AArch64SVEPredPattern {
#define GET_SVEPREDPATsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
namespace AArch64SVEVecLenSpecifier {
#define GET_SVEVECLENSPECIFIERsList_IMPL
#include "AArch64GenSystemOperands.inc"
} // namespace AArch64SVEVecLenSpecifier
} // namespace llvm

namespace llvm {
  namespace AArch64ExactFPImm {
#define GET_ExactFPImmsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64PState {
#define GET_PStateImm0_15sList_IMPL
#include "AArch64GenSystemOperands.inc"
#define GET_PStateImm0_1sList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64PSBHint {
#define GET_PSBsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
namespace AArch64PHint {
#define GET_PHintsList_IMPL
#include "AArch64GenSystemOperands.inc"
} // namespace AArch64PHint
} // namespace llvm

namespace llvm {
  namespace AArch64BTIHint {
#define GET_BTIsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64SysReg {
#define GET_SysRegsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

uint32_t AArch64SysReg::parseGenericRegister(StringRef Name) {
  // Try to parse an S<op0>_<op1>_<Cn>_<Cm>_<op2> register name
  static const Regex GenericRegPattern("^S([0-3])_([0-7])_C([0-9]|1[0-5])_C([0-9]|1[0-5])_([0-7])$");

  std::string UpperName = Name.upper();
  SmallVector<StringRef, 5> Ops;
  if (!GenericRegPattern.match(UpperName, &Ops))
    return -1;

  uint32_t Op0 = 0, Op1 = 0, CRn = 0, CRm = 0, Op2 = 0;
  uint32_t Bits;
  Ops[1].getAsInteger(10, Op0);
  Ops[2].getAsInteger(10, Op1);
  Ops[3].getAsInteger(10, CRn);
  Ops[4].getAsInteger(10, CRm);
  Ops[5].getAsInteger(10, Op2);
  Bits = (Op0 << 14) | (Op1 << 11) | (CRn << 7) | (CRm << 3) | Op2;

  return Bits;
}

std::string AArch64SysReg::genericRegisterString(uint32_t Bits) {
  assert(Bits < 0x10000);
  uint32_t Op0 = (Bits >> 14) & 0x3;
  uint32_t Op1 = (Bits >> 11) & 0x7;
  uint32_t CRn = (Bits >> 7) & 0xf;
  uint32_t CRm = (Bits >> 3) & 0xf;
  uint32_t Op2 = Bits & 0x7;

  return "S" + utostr(Op0) + "_" + utostr(Op1) + "_C" + utostr(CRn) + "_C" +
         utostr(CRm) + "_" + utostr(Op2);
}

namespace llvm {
  namespace AArch64TLBI {
#define GET_TLBITable_IMPL
#include "AArch64GenSystemOperands.inc"
  }
}

namespace llvm {
  namespace AArch64SVCR {
#define GET_SVCRsList_IMPL
#include "AArch64GenSystemOperands.inc"
  }

static const LdStNInstrDesc LdStNInstInfo[] = {
  { AArch64::LD1i8,             "ld1",  ".b",     1, true,  0  },
  { AArch64::LD1i16,            "ld1",  ".h",     1, true,  0  },
  { AArch64::LD1i32,            "ld1",  ".s",     1, true,  0  },
  { AArch64::LD1i64,            "ld1",  ".d",     1, true,  0  },
  { AArch64::LD1i8_POST,        "ld1",  ".b",     2, true,  1  },
  { AArch64::LD1i16_POST,       "ld1",  ".h",     2, true,  2  },
  { AArch64::LD1i32_POST,       "ld1",  ".s",     2, true,  4  },
  { AArch64::LD1i64_POST,       "ld1",  ".d",     2, true,  8  },
  { AArch64::LD1Rv16b,          "ld1r", ".16b",   0, false, 0  },
  { AArch64::LD1Rv8h,           "ld1r", ".8h",    0, false, 0  },
  { AArch64::LD1Rv4s,           "ld1r", ".4s",    0, false, 0  },
  { AArch64::LD1Rv2d,           "ld1r", ".2d",    0, false, 0  },
  { AArch64::LD1Rv8b,           "ld1r", ".8b",    0, false, 0  },
  { AArch64::LD1Rv4h,           "ld1r", ".4h",    0, false, 0  },
  { AArch64::LD1Rv2s,           "ld1r", ".2s",    0, false, 0  },
  { AArch64::LD1Rv1d,           "ld1r", ".1d",    0, false, 0  },
  { AArch64::LD1Rv16b_POST,     "ld1r", ".16b",   1, false, 1  },
  { AArch64::LD1Rv8h_POST,      "ld1r", ".8h",    1, false, 2  },
  { AArch64::LD1Rv4s_POST,      "ld1r", ".4s",    1, false, 4  },
  { AArch64::LD1Rv2d_POST,      "ld1r", ".2d",    1, false, 8  },
  { AArch64::LD1Rv8b_POST,      "ld1r", ".8b",    1, false, 1  },
  { AArch64::LD1Rv4h_POST,      "ld1r", ".4h",    1, false, 2  },
  { AArch64::LD1Rv2s_POST,      "ld1r", ".2s",    1, false, 4  },
  { AArch64::LD1Rv1d_POST,      "ld1r", ".1d",    1, false, 8  },
  { AArch64::LD1Onev16b,        "ld1",  ".16b",   0, false, 0  },
  { AArch64::LD1Onev8h,         "ld1",  ".8h",    0, false, 0  },
  { AArch64::LD1Onev4s,         "ld1",  ".4s",    0, false, 0  },
  { AArch64::LD1Onev2d,         "ld1",  ".2d",    0, false, 0  },
  { AArch64::LD1Onev8b,         "ld1",  ".8b",    0, false, 0  },
  { AArch64::LD1Onev4h,         "ld1",  ".4h",    0, false, 0  },
  { AArch64::LD1Onev2s,         "ld1",  ".2s",    0, false, 0  },
  { AArch64::LD1Onev1d,         "ld1",  ".1d",    0, false, 0  },
  { AArch64::LD1Onev16b_POST,   "ld1",  ".16b",   1, false, 16 },
  { AArch64::LD1Onev8h_POST,    "ld1",  ".8h",    1, false, 16 },
  { AArch64::LD1Onev4s_POST,    "ld1",  ".4s",    1, false, 16 },
  { AArch64::LD1Onev2d_POST,    "ld1",  ".2d",    1, false, 16 },
  { AArch64::LD1Onev8b_POST,    "ld1",  ".8b",    1, false, 8  },
  { AArch64::LD1Onev4h_POST,    "ld1",  ".4h",    1, false, 8  },
  { AArch64::LD1Onev2s_POST,    "ld1",  ".2s",    1, false, 8  },
  { AArch64::LD1Onev1d_POST,    "ld1",  ".1d",    1, false, 8  },
  { AArch64::LD1Twov16b,        "ld1",  ".16b",   0, false, 0  },
  { AArch64::LD1Twov8h,         "ld1",  ".8h",    0, false, 0  },
  { AArch64::LD1Twov4s,         "ld1",  ".4s",    0, false, 0  },
  { AArch64::LD1Twov2d,         "ld1",  ".2d",    0, false, 0  },
  { AArch64::LD1Twov8b,         "ld1",  ".8b",    0, false, 0  },
  { AArch64::LD1Twov4h,         "ld1",  ".4h",    0, false, 0  },
  { AArch64::LD1Twov2s,         "ld1",  ".2s",    0, false, 0  },
  { AArch64::LD1Twov1d,         "ld1",  ".1d",    0, false, 0  },
  { AArch64::LD1Twov16b_POST,   "ld1",  ".16b",   1, false, 32 },
  { AArch64::LD1Twov8h_POST,    "ld1",  ".8h",    1, false, 32 },
  { AArch64::LD1Twov4s_POST,    "ld1",  ".4s",    1, false, 32 },
  { AArch64::LD1Twov2d_POST,    "ld1",  ".2d",    1, false, 32 },
  { AArch64::LD1Twov8b_POST,    "ld1",  ".8b",    1, false, 16 },
  { AArch64::LD1Twov4h_POST,    "ld1",  ".4h",    1, false, 16 },
  { AArch64::LD1Twov2s_POST,    "ld1",  ".2s",    1, false, 16 },
  { AArch64::LD1Twov1d_POST,    "ld1",  ".1d",    1, false, 16 },
  { AArch64::LD1Threev16b,      "ld1",  ".16b",   0, false, 0  },
  { AArch64::LD1Threev8h,       "ld1",  ".8h",    0, false, 0  },
  { AArch64::LD1Threev4s,       "ld1",  ".4s",    0, false, 0  },
  { AArch64::LD1Threev2d,       "ld1",  ".2d",    0, false, 0  },
  { AArch64::LD1Threev8b,       "ld1",  ".8b",    0, false, 0  },
  { AArch64::LD1Threev4h,       "ld1",  ".4h",    0, false, 0  },
  { AArch64::LD1Threev2s,       "ld1",  ".2s",    0, false, 0  },
  { AArch64::LD1Threev1d,       "ld1",  ".1d",    0, false, 0  },
  { AArch64::LD1Threev16b_POST, "ld1",  ".16b",   1, false, 48 },
  { AArch64::LD1Threev8h_POST,  "ld1",  ".8h",    1, false, 48 },
  { AArch64::LD1Threev4s_POST,  "ld1",  ".4s",    1, false, 48 },
  { AArch64::LD1Threev2d_POST,  "ld1",  ".2d",    1, false, 48 },
  { AArch64::LD1Threev8b_POST,  "ld1",  ".8b",    1, false, 24 },
  { AArch64::LD1Threev4h_POST,  "ld1",  ".4h",    1, false, 24 },
  { AArch64::LD1Threev2s_POST,  "ld1",  ".2s",    1, false, 24 },
  { AArch64::LD1Threev1d_POST,  "ld1",  ".1d",    1, false, 24 },
  { AArch64::LD1Fourv16b,       "ld1",  ".16b",   0, false, 0  },
  { AArch64::LD1Fourv8h,        "ld1",  ".8h",    0, false, 0  },
  { AArch64::LD1Fourv4s,        "ld1",  ".4s",    0, false, 0  },
  { AArch64::LD1Fourv2d,        "ld1",  ".2d",    0, false, 0  },
  { AArch64::LD1Fourv8b,        "ld1",  ".8b",    0, false, 0  },
  { AArch64::LD1Fourv4h,        "ld1",  ".4h",    0, false, 0  },
  { AArch64::LD1Fourv2s,        "ld1",  ".2s",    0, false, 0  },
  { AArch64::LD1Fourv1d,        "ld1",  ".1d",    0, false, 0  },
  { AArch64::LD1Fourv16b_POST,  "ld1",  ".16b",   1, false, 64 },
  { AArch64::LD1Fourv8h_POST,   "ld1",  ".8h",    1, false, 64 },
  { AArch64::LD1Fourv4s_POST,   "ld1",  ".4s",    1, false, 64 },
  { AArch64::LD1Fourv2d_POST,   "ld1",  ".2d",    1, false, 64 },
  { AArch64::LD1Fourv8b_POST,   "ld1",  ".8b",    1, false, 32 },
  { AArch64::LD1Fourv4h_POST,   "ld1",  ".4h",    1, false, 32 },
  { AArch64::LD1Fourv2s_POST,   "ld1",  ".2s",    1, false, 32 },
  { AArch64::LD1Fourv1d_POST,   "ld1",  ".1d",    1, false, 32 },
  { AArch64::LD2i8,             "ld2",  ".b",     1, true,  0  },
  { AArch64::LD2i16,            "ld2",  ".h",     1, true,  0  },
  { AArch64::LD2i32,            "ld2",  ".s",     1, true,  0  },
  { AArch64::LD2i64,            "ld2",  ".d",     1, true,  0  },
  { AArch64::LD2i8_POST,        "ld2",  ".b",     2, true,  2  },
  { AArch64::LD2i16_POST,       "ld2",  ".h",     2, true,  4  },
  { AArch64::LD2i32_POST,       "ld2",  ".s",     2, true,  8  },
  { AArch64::LD2i64_POST,       "ld2",  ".d",     2, true,  16  },
  { AArch64::LD2Rv16b,          "ld2r", ".16b",   0, false, 0  },
  { AArch64::LD2Rv8h,           "ld2r", ".8h",    0, false, 0  },
  { AArch64::LD2Rv4s,           "ld2r", ".4s",    0, false, 0  },
  { AArch64::LD2Rv2d,           "ld2r", ".2d",    0, false, 0  },
  { AArch64::LD2Rv8b,           "ld2r", ".8b",    0, false, 0  },
  { AArch64::LD2Rv4h,           "ld2r", ".4h",    0, false, 0  },
  { AArch64::LD2Rv2s,           "ld2r", ".2s",    0, false, 0  },
  { AArch64::LD2Rv1d,           "ld2r", ".1d",    0, false, 0  },
  { AArch64::LD2Rv16b_POST,     "ld2r", ".16b",   1, false, 2  },
  { AArch64::LD2Rv8h_POST,      "ld2r", ".8h",    1, false, 4  },
  { AArch64::LD2Rv4s_POST,      "ld2r", ".4s",    1, false, 8  },
  { AArch64::LD2Rv2d_POST,      "ld2r", ".2d",    1, false, 16 },
  { AArch64::LD2Rv8b_POST,      "ld2r", ".8b",    1, false, 2  },
  { AArch64::LD2Rv4h_POST,      "ld2r", ".4h",    1, false, 4  },
  { AArch64::LD2Rv2s_POST,      "ld2r", ".2s",    1, false, 8  },
  { AArch64::LD2Rv1d_POST,      "ld2r", ".1d",    1, false, 16 },
  { AArch64::LD2Twov16b,        "ld2",  ".16b",   0, false, 0  },
  { AArch64::LD2Twov8h,         "ld2",  ".8h",    0, false, 0  },
  { AArch64::LD2Twov4s,         "ld2",  ".4s",    0, false, 0  },
  { AArch64::LD2Twov2d,         "ld2",  ".2d",    0, false, 0  },
  { AArch64::LD2Twov8b,         "ld2",  ".8b",    0, false, 0  },
  { AArch64::LD2Twov4h,         "ld2",  ".4h",    0, false, 0  },
  { AArch64::LD2Twov2s,         "ld2",  ".2s",    0, false, 0  },
  { AArch64::LD2Twov16b_POST,   "ld2",  ".16b",   1, false, 32 },
  { AArch64::LD2Twov8h_POST,    "ld2",  ".8h",    1, false, 32 },
  { AArch64::LD2Twov4s_POST,    "ld2",  ".4s",    1, false, 32 },
  { AArch64::LD2Twov2d_POST,    "ld2",  ".2d",    1, false, 32 },
  { AArch64::LD2Twov8b_POST,    "ld2",  ".8b",    1, false, 16 },
  { AArch64::LD2Twov4h_POST,    "ld2",  ".4h",    1, false, 16 },
  { AArch64::LD2Twov2s_POST,    "ld2",  ".2s",    1, false, 16 },
  { AArch64::LD3i8,             "ld3",  ".b",     1, true,  0  },
  { AArch64::LD3i16,            "ld3",  ".h",     1, true,  0  },
  { AArch64::LD3i32,            "ld3",  ".s",     1, true,  0  },
  { AArch64::LD3i64,            "ld3",  ".d",     1, true,  0  },
  { AArch64::LD3i8_POST,        "ld3",  ".b",     2, true,  3  },
  { AArch64::LD3i16_POST,       "ld3",  ".h",     2, true,  6  },
  { AArch64::LD3i32_POST,       "ld3",  ".s",     2, true,  12 },
  { AArch64::LD3i64_POST,       "ld3",  ".d",     2, true,  24 },
  { AArch64::LD3Rv16b,          "ld3r", ".16b",   0, false, 0  },
  { AArch64::LD3Rv8h,           "ld3r", ".8h",    0, false, 0  },
  { AArch64::LD3Rv4s,           "ld3r", ".4s",    0, false, 0  },
  { AArch64::LD3Rv2d,           "ld3r", ".2d",    0, false, 0  },
  { AArch64::LD3Rv8b,           "ld3r", ".8b",    0, false, 0  },
  { AArch64::LD3Rv4h,           "ld3r", ".4h",    0, false, 0  },
  { AArch64::LD3Rv2s,           "ld3r", ".2s",    0, false, 0  },
  { AArch64::LD3Rv1d,           "ld3r", ".1d",    0, false, 0  },
  { AArch64::LD3Rv16b_POST,     "ld3r", ".16b",   1, false, 3  },
  { AArch64::LD3Rv8h_POST,      "ld3r", ".8h",    1, false, 6  },
  { AArch64::LD3Rv4s_POST,      "ld3r", ".4s",    1, false, 12 },
  { AArch64::LD3Rv2d_POST,      "ld3r", ".2d",    1, false, 24 },
  { AArch64::LD3Rv8b_POST,      "ld3r", ".8b",    1, false, 3  },
  { AArch64::LD3Rv4h_POST,      "ld3r", ".4h",    1, false, 6  },
  { AArch64::LD3Rv2s_POST,      "ld3r", ".2s",    1, false, 12 },
  { AArch64::LD3Rv1d_POST,      "ld3r", ".1d",    1, false, 24 },
  { AArch64::LD3Threev16b,      "ld3",  ".16b",   0, false, 0  },
  { AArch64::LD3Threev8h,       "ld3",  ".8h",    0, false, 0  },
  { AArch64::LD3Threev4s,       "ld3",  ".4s",    0, false, 0  },
  { AArch64::LD3Threev2d,       "ld3",  ".2d",    0, false, 0  },
  { AArch64::LD3Threev8b,       "ld3",  ".8b",    0, false, 0  },
  { AArch64::LD3Threev4h,       "ld3",  ".4h",    0, false, 0  },
  { AArch64::LD3Threev2s,       "ld3",  ".2s",    0, false, 0  },
  { AArch64::LD3Threev16b_POST, "ld3",  ".16b",   1, false, 48 },
  { AArch64::LD3Threev8h_POST,  "ld3",  ".8h",    1, false, 48 },
  { AArch64::LD3Threev4s_POST,  "ld3",  ".4s",    1, false, 48 },
  { AArch64::LD3Threev2d_POST,  "ld3",  ".2d",    1, false, 48 },
  { AArch64::LD3Threev8b_POST,  "ld3",  ".8b",    1, false, 24 },
  { AArch64::LD3Threev4h_POST,  "ld3",  ".4h",    1, false, 24 },
  { AArch64::LD3Threev2s_POST,  "ld3",  ".2s",    1, false, 24 },
  { AArch64::LD4i8,             "ld4",  ".b",     1, true,  0  },
  { AArch64::LD4i16,            "ld4",  ".h",     1, true,  0  },
  { AArch64::LD4i32,            "ld4",  ".s",     1, true,  0  },
  { AArch64::LD4i64,            "ld4",  ".d",     1, true,  0  },
  { AArch64::LD4i8_POST,        "ld4",  ".b",     2, true,  4  },
  { AArch64::LD4i16_POST,       "ld4",  ".h",     2, true,  8  },
  { AArch64::LD4i32_POST,       "ld4",  ".s",     2, true,  16 },
  { AArch64::LD4i64_POST,       "ld4",  ".d",     2, true,  32 },
  { AArch64::LD4Rv16b,          "ld4r", ".16b",   0, false, 0  },
  { AArch64::LD4Rv8h,           "ld4r", ".8h",    0, false, 0  },
  { AArch64::LD4Rv4s,           "ld4r", ".4s",    0, false, 0  },
  { AArch64::LD4Rv2d,           "ld4r", ".2d",    0, false, 0  },
  { AArch64::LD4Rv8b,           "ld4r", ".8b",    0, false, 0  },
  { AArch64::LD4Rv4h,           "ld4r", ".4h",    0, false, 0  },
  { AArch64::LD4Rv2s,           "ld4r", ".2s",    0, false, 0  },
  { AArch64::LD4Rv1d,           "ld4r", ".1d",    0, false, 0  },
  { AArch64::LD4Rv16b_POST,     "ld4r", ".16b",   1, false, 4  },
  { AArch64::LD4Rv8h_POST,      "ld4r", ".8h",    1, false, 8  },
  { AArch64::LD4Rv4s_POST,      "ld4r", ".4s",    1, false, 16 },
  { AArch64::LD4Rv2d_POST,      "ld4r", ".2d",    1, false, 32 },
  { AArch64::LD4Rv8b_POST,      "ld4r", ".8b",    1, false, 4  },
  { AArch64::LD4Rv4h_POST,      "ld4r", ".4h",    1, false, 8  },
  { AArch64::LD4Rv2s_POST,      "ld4r", ".2s",    1, false, 16 },
  { AArch64::LD4Rv1d_POST,      "ld4r", ".1d",    1, false, 32 },
  { AArch64::LD4Fourv16b,       "ld4",  ".16b",   0, false, 0  },
  { AArch64::LD4Fourv8h,        "ld4",  ".8h",    0, false, 0  },
  { AArch64::LD4Fourv4s,        "ld4",  ".4s",    0, false, 0  },
  { AArch64::LD4Fourv2d,        "ld4",  ".2d",    0, false, 0  },
  { AArch64::LD4Fourv8b,        "ld4",  ".8b",    0, false, 0  },
  { AArch64::LD4Fourv4h,        "ld4",  ".4h",    0, false, 0  },
  { AArch64::LD4Fourv2s,        "ld4",  ".2s",    0, false, 0  },
  { AArch64::LD4Fourv16b_POST,  "ld4",  ".16b",   1, false, 64 },
  { AArch64::LD4Fourv8h_POST,   "ld4",  ".8h",    1, false, 64 },
  { AArch64::LD4Fourv4s_POST,   "ld4",  ".4s",    1, false, 64 },
  { AArch64::LD4Fourv2d_POST,   "ld4",  ".2d",    1, false, 64 },
  { AArch64::LD4Fourv8b_POST,   "ld4",  ".8b",    1, false, 32 },
  { AArch64::LD4Fourv4h_POST,   "ld4",  ".4h",    1, false, 32 },
  { AArch64::LD4Fourv2s_POST,   "ld4",  ".2s",    1, false, 32 },
  { AArch64::ST1i8,             "st1",  ".b",     0, true,  0  },
  { AArch64::ST1i16,            "st1",  ".h",     0, true,  0  },
  { AArch64::ST1i32,            "st1",  ".s",     0, true,  0  },
  { AArch64::ST1i64,            "st1",  ".d",     0, true,  0  },
  { AArch64::ST1i8_POST,        "st1",  ".b",     1, true,  1  },
  { AArch64::ST1i16_POST,       "st1",  ".h",     1, true,  2  },
  { AArch64::ST1i32_POST,       "st1",  ".s",     1, true,  4  },
  { AArch64::ST1i64_POST,       "st1",  ".d",     1, true,  8  },
  { AArch64::ST1Onev16b,        "st1",  ".16b",   0, false, 0  },
  { AArch64::ST1Onev8h,         "st1",  ".8h",    0, false, 0  },
  { AArch64::ST1Onev4s,         "st1",  ".4s",    0, false, 0  },
  { AArch64::ST1Onev2d,         "st1",  ".2d",    0, false, 0  },
  { AArch64::ST1Onev8b,         "st1",  ".8b",    0, false, 0  },
  { AArch64::ST1Onev4h,         "st1",  ".4h",    0, false, 0  },
  { AArch64::ST1Onev2s,         "st1",  ".2s",    0, false, 0  },
  { AArch64::ST1Onev1d,         "st1",  ".1d",    0, false, 0  },
  { AArch64::ST1Onev16b_POST,   "st1",  ".16b",   1, false, 16 },
  { AArch64::ST1Onev8h_POST,    "st1",  ".8h",    1, false, 16 },
  { AArch64::ST1Onev4s_POST,    "st1",  ".4s",    1, false, 16 },
  { AArch64::ST1Onev2d_POST,    "st1",  ".2d",    1, false, 16 },
  { AArch64::ST1Onev8b_POST,    "st1",  ".8b",    1, false, 8  },
  { AArch64::ST1Onev4h_POST,    "st1",  ".4h",    1, false, 8  },
  { AArch64::ST1Onev2s_POST,    "st1",  ".2s",    1, false, 8  },
  { AArch64::ST1Onev1d_POST,    "st1",  ".1d",    1, false, 8  },
  { AArch64::ST1Twov16b,        "st1",  ".16b",   0, false, 0  },
  { AArch64::ST1Twov8h,         "st1",  ".8h",    0, false, 0  },
  { AArch64::ST1Twov4s,         "st1",  ".4s",    0, false, 0  },
  { AArch64::ST1Twov2d,         "st1",  ".2d",    0, false, 0  },
  { AArch64::ST1Twov8b,         "st1",  ".8b",    0, false, 0  },
  { AArch64::ST1Twov4h,         "st1",  ".4h",    0, false, 0  },
  { AArch64::ST1Twov2s,         "st1",  ".2s",    0, false, 0  },
  { AArch64::ST1Twov1d,         "st1",  ".1d",    0, false, 0  },
  { AArch64::ST1Twov16b_POST,   "st1",  ".16b",   1, false, 32 },
  { AArch64::ST1Twov8h_POST,    "st1",  ".8h",    1, false, 32 },
  { AArch64::ST1Twov4s_POST,    "st1",  ".4s",    1, false, 32 },
  { AArch64::ST1Twov2d_POST,    "st1",  ".2d",    1, false, 32 },
  { AArch64::ST1Twov8b_POST,    "st1",  ".8b",    1, false, 16 },
  { AArch64::ST1Twov4h_POST,    "st1",  ".4h",    1, false, 16 },
  { AArch64::ST1Twov2s_POST,    "st1",  ".2s",    1, false, 16 },
  { AArch64::ST1Twov1d_POST,    "st1",  ".1d",    1, false, 16 },
  { AArch64::ST1Threev16b,      "st1",  ".16b",   0, false, 0  },
  { AArch64::ST1Threev8h,       "st1",  ".8h",    0, false, 0  },
  { AArch64::ST1Threev4s,       "st1",  ".4s",    0, false, 0  },
  { AArch64::ST1Threev2d,       "st1",  ".2d",    0, false, 0  },
  { AArch64::ST1Threev8b,       "st1",  ".8b",    0, false, 0  },
  { AArch64::ST1Threev4h,       "st1",  ".4h",    0, false, 0  },
  { AArch64::ST1Threev2s,       "st1",  ".2s",    0, false, 0  },
  { AArch64::ST1Threev1d,       "st1",  ".1d",    0, false, 0  },
  { AArch64::ST1Threev16b_POST, "st1",  ".16b",   1, false, 48 },
  { AArch64::ST1Threev8h_POST,  "st1",  ".8h",    1, false, 48 },
  { AArch64::ST1Threev4s_POST,  "st1",  ".4s",    1, false, 48 },
  { AArch64::ST1Threev2d_POST,  "st1",  ".2d",    1, false, 48 },
  { AArch64::ST1Threev8b_POST,  "st1",  ".8b",    1, false, 24 },
  { AArch64::ST1Threev4h_POST,  "st1",  ".4h",    1, false, 24 },
  { AArch64::ST1Threev2s_POST,  "st1",  ".2s",    1, false, 24 },
  { AArch64::ST1Threev1d_POST,  "st1",  ".1d",    1, false, 24 },
  { AArch64::ST1Fourv16b,       "st1",  ".16b",   0, false, 0  },
  { AArch64::ST1Fourv8h,        "st1",  ".8h",    0, false, 0  },
  { AArch64::ST1Fourv4s,        "st1",  ".4s",    0, false, 0  },
  { AArch64::ST1Fourv2d,        "st1",  ".2d",    0, false, 0  },
  { AArch64::ST1Fourv8b,        "st1",  ".8b",    0, false, 0  },
  { AArch64::ST1Fourv4h,        "st1",  ".4h",    0, false, 0  },
  { AArch64::ST1Fourv2s,        "st1",  ".2s",    0, false, 0  },
  { AArch64::ST1Fourv1d,        "st1",  ".1d",    0, false, 0  },
  { AArch64::ST1Fourv16b_POST,  "st1",  ".16b",   1, false, 64 },
  { AArch64::ST1Fourv8h_POST,   "st1",  ".8h",    1, false, 64 },
  { AArch64::ST1Fourv4s_POST,   "st1",  ".4s",    1, false, 64 },
  { AArch64::ST1Fourv2d_POST,   "st1",  ".2d",    1, false, 64 },
  { AArch64::ST1Fourv8b_POST,   "st1",  ".8b",    1, false, 32 },
  { AArch64::ST1Fourv4h_POST,   "st1",  ".4h",    1, false, 32 },
  { AArch64::ST1Fourv2s_POST,   "st1",  ".2s",    1, false, 32 },
  { AArch64::ST1Fourv1d_POST,   "st1",  ".1d",    1, false, 32 },
  { AArch64::ST2i8,             "st2",  ".b",     0, true,  0  },
  { AArch64::ST2i16,            "st2",  ".h",     0, true,  0  },
  { AArch64::ST2i32,            "st2",  ".s",     0, true,  0  },
  { AArch64::ST2i64,            "st2",  ".d",     0, true,  0  },
  { AArch64::ST2i8_POST,        "st2",  ".b",     1, true,  2  },
  { AArch64::ST2i16_POST,       "st2",  ".h",     1, true,  4  },
  { AArch64::ST2i32_POST,       "st2",  ".s",     1, true,  8  },
  { AArch64::ST2i64_POST,       "st2",  ".d",     1, true,  16 },
  { AArch64::ST2Twov16b,        "st2",  ".16b",   0, false, 0  },
  { AArch64::ST2Twov8h,         "st2",  ".8h",    0, false, 0  },
  { AArch64::ST2Twov4s,         "st2",  ".4s",    0, false, 0  },
  { AArch64::ST2Twov2d,         "st2",  ".2d",    0, false, 0  },
  { AArch64::ST2Twov8b,         "st2",  ".8b",    0, false, 0  },
  { AArch64::ST2Twov4h,         "st2",  ".4h",    0, false, 0  },
  { AArch64::ST2Twov2s,         "st2",  ".2s",    0, false, 0  },
  { AArch64::ST2Twov16b_POST,   "st2",  ".16b",   1, false, 32 },
  { AArch64::ST2Twov8h_POST,    "st2",  ".8h",    1, false, 32 },
  { AArch64::ST2Twov4s_POST,    "st2",  ".4s",    1, false, 32 },
  { AArch64::ST2Twov2d_POST,    "st2",  ".2d",    1, false, 32 },
  { AArch64::ST2Twov8b_POST,    "st2",  ".8b",    1, false, 16 },
  { AArch64::ST2Twov4h_POST,    "st2",  ".4h",    1, false, 16 },
  { AArch64::ST2Twov2s_POST,    "st2",  ".2s",    1, false, 16 },
  { AArch64::ST3i8,             "st3",  ".b",     0, true,  0  },
  { AArch64::ST3i16,            "st3",  ".h",     0, true,  0  },
  { AArch64::ST3i32,            "st3",  ".s",     0, true,  0  },
  { AArch64::ST3i64,            "st3",  ".d",     0, true,  0  },
  { AArch64::ST3i8_POST,        "st3",  ".b",     1, true,  3  },
  { AArch64::ST3i16_POST,       "st3",  ".h",     1, true,  6  },
  { AArch64::ST3i32_POST,       "st3",  ".s",     1, true,  12 },
  { AArch64::ST3i64_POST,       "st3",  ".d",     1, true,  24 },
  { AArch64::ST3Threev16b,      "st3",  ".16b",   0, false, 0  },
  { AArch64::ST3Threev8h,       "st3",  ".8h",    0, false, 0  },
  { AArch64::ST3Threev4s,       "st3",  ".4s",    0, false, 0  },
  { AArch64::ST3Threev2d,       "st3",  ".2d",    0, false, 0  },
  { AArch64::ST3Threev8b,       "st3",  ".8b",    0, false, 0  },
  { AArch64::ST3Threev4h,       "st3",  ".4h",    0, false, 0  },
  { AArch64::ST3Threev2s,       "st3",  ".2s",    0, false, 0  },
  { AArch64::ST3Threev16b_POST, "st3",  ".16b",   1, false, 48 },
  { AArch64::ST3Threev8h_POST,  "st3",  ".8h",    1, false, 48 },
  { AArch64::ST3Threev4s_POST,  "st3",  ".4s",    1, false, 48 },
  { AArch64::ST3Threev2d_POST,  "st3",  ".2d",    1, false, 48 },
  { AArch64::ST3Threev8b_POST,  "st3",  ".8b",    1, false, 24 },
  { AArch64::ST3Threev4h_POST,  "st3",  ".4h",    1, false, 24 },
  { AArch64::ST3Threev2s_POST,  "st3",  ".2s",    1, false, 24 },
  { AArch64::ST4i8,             "st4",  ".b",     0, true,  0  },
  { AArch64::ST4i16,            "st4",  ".h",     0, true,  0  },
  { AArch64::ST4i32,            "st4",  ".s",     0, true,  0  },
  { AArch64::ST4i64,            "st4",  ".d",     0, true,  0  },
  { AArch64::ST4i8_POST,        "st4",  ".b",     1, true,  4  },
  { AArch64::ST4i16_POST,       "st4",  ".h",     1, true,  8  },
  { AArch64::ST4i32_POST,       "st4",  ".s",     1, true,  16 },
  { AArch64::ST4i64_POST,       "st4",  ".d",     1, true,  32 },
  { AArch64::ST4Fourv16b,       "st4",  ".16b",   0, false, 0  },
  { AArch64::ST4Fourv8h,        "st4",  ".8h",    0, false, 0  },
  { AArch64::ST4Fourv4s,        "st4",  ".4s",    0, false, 0  },
  { AArch64::ST4Fourv2d,        "st4",  ".2d",    0, false, 0  },
  { AArch64::ST4Fourv8b,        "st4",  ".8b",    0, false, 0  },
  { AArch64::ST4Fourv4h,        "st4",  ".4h",    0, false, 0  },
  { AArch64::ST4Fourv2s,        "st4",  ".2s",    0, false, 0  },
  { AArch64::ST4Fourv16b_POST,  "st4",  ".16b",   1, false, 64 },
  { AArch64::ST4Fourv8h_POST,   "st4",  ".8h",    1, false, 64 },
  { AArch64::ST4Fourv4s_POST,   "st4",  ".4s",    1, false, 64 },
  { AArch64::ST4Fourv2d_POST,   "st4",  ".2d",    1, false, 64 },
  { AArch64::ST4Fourv8b_POST,   "st4",  ".8b",    1, false, 32 },
  { AArch64::ST4Fourv4h_POST,   "st4",  ".4h",    1, false, 32 },
  { AArch64::ST4Fourv2s_POST,   "st4",  ".2s",    1, false, 32 },
};

const LdStNInstrDesc *getLdStNInstrDesc(unsigned Opcode) {
  for (const auto &Info : LdStNInstInfo)
    if (Info.Opcode == Opcode)
      return &Info;

  return nullptr;
}

}
