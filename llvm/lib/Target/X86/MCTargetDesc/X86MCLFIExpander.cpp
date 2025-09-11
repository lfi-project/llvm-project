//===- X86MCLFIExpander.cpp - X86 LFI Expander --------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the X86MCLFIExpander class, the X86 specific
// subclass of MCLFIExpander.
//
// This file was written by the Native Client authors.
//
//===----------------------------------------------------------------------===//
#include "X86MCLFIExpander.h"
#include "X86BaseInfo.h"

#include "X86MCTargetDesc.h"
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

bool X86::X86MCLFIExpander::isValidScratchRegister(MCRegister Reg) const {
  return false;
}

void X86::X86MCLFIExpander::expandDirectCall(const MCInst &Inst,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
}

void X86::X86MCLFIExpander::emitIndirectJumpReg(MCRegister Reg, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {

}

void X86::X86MCLFIExpander::emitIndirectCallReg(MCRegister Reg, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
}

void X86::X86MCLFIExpander::expandIndirectBranch(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
}

void X86::X86MCLFIExpander::expandReturn(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
}

// Expands any operations that load to or store from memory, but do
// not explicitly modify the stack or base pointer.
void X86::X86MCLFIExpander::expandLoadStore(const MCInst &Inst,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI,
                                             bool EmitPrefixes) {
}

void X86::X86MCLFIExpander::expandStringOperation(
    const MCInst &Inst,
    MCStreamer &Out,
    const MCSubtargetInfo &STI,
    bool EmitPrefixes) {
}

void X86::X86MCLFIExpander::expandExplicitStackManipulation(
    MCRegister StackReg, const MCInst &Inst, MCStreamer &Out,
    const MCSubtargetInfo &STI, bool EmitPrefixes) {
}

void X86::X86MCLFIExpander::expandStackRegPush(const MCInst &Inst,
                                                MCStreamer &Out,
                                                const MCSubtargetInfo &STI) {
}

void X86::X86MCLFIExpander::doExpandInst(const MCInst &Inst,
                                          MCStreamer &Out,
                                          const MCSubtargetInfo &STI,
                                          bool EmitPrefixes) {
  printf("TODO: doExpandInst\n");
  Out.emitInstruction(Inst, STI);
}

bool X86::X86MCLFIExpander::expandInst(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  if (Guard)
    return false;
  Guard = true;

  doExpandInst(Inst, Out, STI, true);
  invalidateScratchRegs(Inst);

  Guard = false;
  return true;
}
