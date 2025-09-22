//===- RISCVMCLFIExpander.cpp - RISCV LFI Expander --------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the RISCVMCLFIExpander class, the RISCV specific
// subclass of MCLFIExpander.
//
// This file was written by the Native Client authors.
//
//===----------------------------------------------------------------------===//
#include "RISCVExpander.h"
#include "RISCVBaseInfo.h"

#include "RISCVMCTargetDesc.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/Alignment.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "saigo"

static const int BundleSize = 32;

bool RISCV::RISCVMCLFIExpander::isValidScratchRegister(MCRegister Reg) const {
  return false;
}

void RISCV::RISCVMCLFIExpander::expandDirectCall(const MCInst &Inst,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
}

void RISCV::RISCVMCLFIExpander::emitIndirectJumpReg(MCRegister Reg, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {

}

void RISCV::RISCVMCLFIExpander::emitIndirectCallReg(MCRegister Reg, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
}

void RISCV::RISCVMCLFIExpander::expandIndirectBranch(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
}

void RISCV::RISCVMCLFIExpander::expandReturn(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
}

// Expands any operations that load to or store from memory, but do
// not explicitly modify the stack or base pointer.
void RISCV::RISCVMCLFIExpander::expandLoadStore(const MCInst &Inst,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI,
                                             bool EmitPrefixes) {
}

void RISCV::RISCVMCLFIExpander::expandStringOperation(
    const MCInst &Inst,
    MCStreamer &Out,
    const MCSubtargetInfo &STI,
    bool EmitPrefixes) {
}

void RISCV::RISCVMCLFIExpander::expandExplicitStackManipulation(
    MCRegister StackReg, const MCInst &Inst, MCStreamer &Out,
    const MCSubtargetInfo &STI, bool EmitPrefixes) {
}

void RISCV::RISCVMCLFIExpander::expandStackRegPush(const MCInst &Inst,
                                                MCStreamer &Out,
                                                const MCSubtargetInfo &STI) {
}

void RISCV::RISCVMCLFIExpander::doExpandInst(const MCInst &Inst,
                                          MCStreamer &Out,
                                          const MCSubtargetInfo &STI,
                                          bool EmitPrefixes) {
  printf("TODO: doExpandInst\n");
  Out.emitInstruction(Inst, STI);
}

bool RISCV::RISCVMCLFIExpander::expandInst(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  if (Guard)
    return false;
  Guard = true;

  doExpandInst(Inst, Out, STI, true);
  invalidateScratchRegs(Inst);

  Guard = false;
  return true;
}