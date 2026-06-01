//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file implements the MCLFIRewriter class, a base class that
/// encapsulates the rewriting logic for MCInsts.
///
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCLFIRewriter.h"
#include "llvm/ADT/Twine.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstPrinter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/TargetRegistry.h"

using namespace llvm;

// Render an instruction as assembly text for diagnostics.
static std::string formatInst(MCContext &Ctx, const MCInstrInfo &II,
                              const MCRegisterInfo &RI, const MCInst &Inst) {
  std::string S;
  raw_string_ostream OS(S);

  const MCAsmInfo &MAI = Ctx.getAsmInfo();
  const MCSubtargetInfo *STI = Ctx.getSubtargetInfo();
  const Triple &TT = Ctx.getTargetTriple();

  std::string Err;
  const Target *T = TargetRegistry::lookupTarget(TT, Err);

  std::unique_ptr<MCInstPrinter> IP;
  if (T)
    IP.reset(
        T->createMCInstPrinter(TT, MAI.getAssemblerDialect(), MAI, II, RI));

  if (IP && STI)
    IP->printInst(&Inst, /*Address=*/0, /*Annot=*/"", *STI, OS);
  else
    OS << II.getName(Inst.getOpcode());

  OS.flush();
  return StringRef(S).trim().str();
}

void MCLFIRewriter::error(const MCInst &Inst, const Twine &Msg) {
  Ctx.reportError(Inst.getLoc(),
                  Msg + " in '" + formatInst(Ctx, *InstInfo, *RegInfo, Inst) +
                      "'");
}

void MCLFIRewriter::warning(const MCInst &Inst, const Twine &Msg) {
  Ctx.reportWarning(Inst.getLoc(),
                    Msg + " in '" + formatInst(Ctx, *InstInfo, *RegInfo, Inst) +
                        "'");
}

bool MCLFIRewriter::isCall(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isCall();
}

bool MCLFIRewriter::isBranch(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isBranch();
}

bool MCLFIRewriter::isIndirectBranch(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isIndirectBranch();
}

bool MCLFIRewriter::isReturn(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isReturn();
}

bool MCLFIRewriter::mayLoad(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).mayLoad();
}

bool MCLFIRewriter::mayStore(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).mayStore();
}

bool MCLFIRewriter::mayModifyRegister(const MCInst &Inst,
                                      MCRegister Reg) const {
  return InstInfo->get(Inst.getOpcode()).hasDefOfPhysReg(Inst, Reg, *RegInfo);
}

bool MCLFIRewriter::explicitlyModifiesRegister(const MCInst &Inst,
                                               MCRegister Reg) const {
  return InstInfo->get(Inst.getOpcode())
      .hasExplicitDefOfPhysReg(Inst, Reg, *RegInfo);
}
