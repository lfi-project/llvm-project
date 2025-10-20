//===- MCLFIExpander.cpp ----------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the Native Client and LFI authors.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MCLFIExpander class. This is a base
// class that encapsulates the expansion logic for MCInsts.
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {

void MCLFIExpander::disable() { Enabled = false; }

void MCLFIExpander::enable() { Enabled = true; }

bool MCLFIExpander::isEnabled() { return Enabled; }

bool MCLFIExpander::isPseudo(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).isPseudo();
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

bool MCLFIExpander::mayLoad(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).mayLoad();
}

bool MCLFIExpander::mayStore(const MCInst &Inst) const {
  return InstInfo->get(Inst.getOpcode()).mayStore();
}

bool MCLFIExpander::mayModifyRegister(const MCInst &Inst,
                                      MCRegister Reg) const {
  return InstInfo->get(Inst.getOpcode()).hasDefOfPhysReg(Inst, Reg, *RegInfo);
}
} // namespace llvm
