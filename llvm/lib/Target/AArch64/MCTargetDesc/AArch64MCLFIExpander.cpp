//===- AArch64MCLFIExpander.cpp ---------------------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the AArch64MCLFIExpander class, the AArch64 specific
// subclass of MCLFIExpander.
//
//===----------------------------------------------------------------------===//
#include "AArch64MCLFIExpander.h"
#include "AArch64AddressingModes.h"
#include "Utils/AArch64BaseInfo.h"

#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "lfi"

bool AArch64::AArch64MCLFIExpander::isValidScratchRegister(MCRegister Reg) const {
  return false;
}

void AArch64::AArch64MCLFIExpander::expandIndirectBranch(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI,
                                                  bool isCall) {
}

void AArch64::AArch64MCLFIExpander::expandCall(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
}

void AArch64::AArch64MCLFIExpander::expandReturn(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
}

void AArch64::AArch64MCLFIExpander::expandControlFlow(const MCInst &Inst,
                                               MCStreamer &Out,
                                               const MCSubtargetInfo &STI) {

}

bool AArch64::AArch64MCLFIExpander::mayModifyStack(const MCInst &Inst) {
  return false;
}

void AArch64::AArch64MCLFIExpander::expandStackManipulation(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
}

void AArch64::AArch64MCLFIExpander::expandPrefetch(const MCInst &Inst, MCStreamer &Out,
                                            const MCSubtargetInfo &STI) {
}

void AArch64::AArch64MCLFIExpander::expandLoadStore(const MCInst &Inst,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI) {
}

void AArch64::AArch64MCLFIExpander::doExpandInst(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
}

bool AArch64::AArch64MCLFIExpander::expandInst(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  return false;
}
