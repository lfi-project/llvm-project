//===- MCLFIExpander.cpp ----------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the Native Client authors, modified for LFI.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MCLFIExpander class. This is a base
// class that encapsulates the expansion logic for MCInsts, and holds
// state such as available scratch registers.
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {

void MCLFIExpander::Error(const MCInst &Inst, const char msg[]) {
  Ctx.reportError(Inst.getLoc(), msg);
}

bool MCLFIExpander::addScratchReg(MCRegister Reg) {
  if (!isValidScratchRegister(Reg))
    return true;
  ScratchRegs.push_back(Reg);
  return false;
}

void MCLFIExpander::invalidateScratchRegs(const MCInst &Inst) {
  // TODO: There are arch-specific special cases where this fails, e.g.
  // xchg/cmpxchg
  // TODO(zyedidia): investigate whether there are cases for AArch64.
  const MCInstrDesc &Desc = InstInfo->get(Inst.getOpcode());
  for (auto I = ScratchRegs.begin(), E = ScratchRegs.end(); I != E; ++I) {
    if (Desc.hasDefOfPhysReg(Inst, *I, *RegInfo))
      I = ScratchRegs.erase(I);
  }
}

void MCLFIExpander::clearScratchRegs() {
  ScratchRegs.clear();
}

MCRegister MCLFIExpander::getScratchReg(int index) {
  assert(index >= 0 && static_cast<unsigned>(index) < numScratchRegs());
  return ScratchRegs[numScratchRegs()  - index - 1];
}

unsigned MCLFIExpander::numScratchRegs() const { return ScratchRegs.size(); }

bool MCLFIExpander::isPseudo(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isPseudo();
}

bool MCLFIExpander::mayAffectControlFlow(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).mayAffectControlFlow(Inst, *RegInfo);
}

bool MCLFIExpander::isCall(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isCall();
}

bool MCLFIExpander::isBranch(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isBranch();
}

bool MCLFIExpander::isIndirectBranch(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isIndirectBranch();
}

bool MCLFIExpander::isReturn(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isReturn();
}

bool MCLFIExpander::isVariadic(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isVariadic();
}

bool MCLFIExpander::mayLoad(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).mayLoad();
}

bool MCLFIExpander::mayStore(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).mayStore();
}

bool MCLFIExpander::mayModifyRegister(const MCInst &Inst, MCRegister Reg) const {
  return InstInfo->get(Inst.getOpcode()).hasDefOfPhysReg(Inst, Reg, *RegInfo);
}

bool MCLFIExpander::explicitlyModifiesRegister(const MCInst &Inst,
                                                MCRegister Reg) const {
  const MCInstrDesc &Desc = InstInfo->get(Inst.getOpcode());
  for (int i = 0; i < Desc.NumDefs; ++i) {
    if (Desc.operands()[i].OperandType == MCOI::OPERAND_REGISTER &&
        RegInfo->isSubRegisterEq(Reg, Inst.getOperand(i).getReg()))
      return true;
  }
  return false;
}
}
