//===- AArch64MCHLFIRewriter.cpp --------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the AArch64MCHLFIRewriter class for HLFI sandboxing.
//
// HLFI only rewrites system calls and TLS accesses at the MC level.
// Memory sandboxing and CFI are handled at the LLVM IR level.
//
//===----------------------------------------------------------------------===//

#include "AArch64MCHLFIRewriter.h"
#include "AArch64InstrInfo.h"
#include "MCTargetDesc/AArch64MCTargetDesc.h"
#include "Utils/AArch64BaseInfo.h"

#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "aarch64-hlfi-rewriter"

namespace {

// HLFI reserved registers.
const MCRegister HLFIBaseReg = AArch64::X27;    // Sandbox base
const MCRegister HLFIContextReg = AArch64::X25; // HLFI context pointer
const MCRegister HLFIScratchReg = AArch64::X26; // Scratch register for syscalls

// HLFI Context offsets (byte offsets from x25).
// These must match the HLFI pass and runtime.
const unsigned HLFIContextOffsetReserved = 0;
const unsigned HLFIContextOffsetUnsafeStack = 8;
const unsigned HLFIContextOffsetCFITable = 16;
const unsigned HLFIContextOffsetThreadPointer = 24;

// Scaled offset for LDR/STR instructions (divide by 8 for 64-bit loads).
const unsigned HLFITPOffsetScaled = HLFIContextOffsetThreadPointer / 8; // 3

// Syscall handler is at [x27-8], like LFI.
const int HLFISyscallOffset = -8;

// Instruction classification.
static bool isSyscall(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::SVC;
}

static bool isTLSRead(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::MRS &&
         Inst.getOperand(1).getImm() == AArch64SysReg::TPIDR_EL0;
}

static bool isTLSWrite(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::MSR &&
         Inst.getOperand(0).getImm() == AArch64SysReg::TPIDR_EL0;
}

} // anonymous namespace

void AArch64::AArch64MCHLFIRewriter::emitInst(const MCInst &Inst,
                                               MCStreamer &Out,
                                               const MCSubtargetInfo &STI) {
  Guard = true;
  Out.emitInstruction(Inst, STI);
  Guard = false;
}

void AArch64::AArch64MCHLFIRewriter::rewriteSyscall(const MCInst &Inst,
                                                     MCStreamer &Out,
                                                     const MCSubtargetInfo &STI) {
  // svc #0 is rewritten to:
  //   mov x26, x30              ; save LR to scratch
  //   ldur x30, [x27, #-8]      ; load syscall handler from [x27-8]
  //   blr x30                   ; call the handler
  //   mov x30, x26              ; restore LR from scratch

  // Save LR to scratch register.
  MCInst MovSave;
  MovSave.setOpcode(AArch64::ORRXrs);
  MovSave.addOperand(MCOperand::createReg(HLFIScratchReg)); // Rd
  MovSave.addOperand(MCOperand::createReg(AArch64::XZR));   // Rn
  MovSave.addOperand(MCOperand::createReg(AArch64::LR));    // Rm
  MovSave.addOperand(MCOperand::createImm(0));              // shift
  emitInst(MovSave, Out, STI);

  // Load syscall handler address from [x27-8].
  MCInst Load;
  Load.setOpcode(AArch64::LDURXi);
  Load.addOperand(MCOperand::createReg(AArch64::LR));
  Load.addOperand(MCOperand::createReg(HLFIBaseReg));
  Load.addOperand(MCOperand::createImm(HLFISyscallOffset));
  emitInst(Load, Out, STI);

  // Call the runtime via BLR.
  MCInst Call;
  Call.setOpcode(AArch64::BLR);
  Call.addOperand(MCOperand::createReg(AArch64::LR));
  emitInst(Call, Out, STI);

  // Restore LR from scratch register.
  MCInst MovRestore;
  MovRestore.setOpcode(AArch64::ORRXrs);
  MovRestore.addOperand(MCOperand::createReg(AArch64::LR));    // Rd
  MovRestore.addOperand(MCOperand::createReg(AArch64::XZR));   // Rn
  MovRestore.addOperand(MCOperand::createReg(HLFIScratchReg)); // Rm
  MovRestore.addOperand(MCOperand::createImm(0));              // shift
  emitInst(MovRestore, Out, STI);
}

void AArch64::AArch64MCHLFIRewriter::rewriteTLSRead(const MCInst &Inst,
                                                     MCStreamer &Out,
                                                     const MCSubtargetInfo &STI) {
  // mrs xN, tpidr_el0  =>  ldr xN, [x25, #32]
  MCRegister DestReg = Inst.getOperand(0).getReg();

  MCInst Load;
  Load.setOpcode(AArch64::LDRXui);
  Load.addOperand(MCOperand::createReg(DestReg));
  Load.addOperand(MCOperand::createReg(HLFIContextReg));
  Load.addOperand(MCOperand::createImm(HLFITPOffsetScaled));
  emitInst(Load, Out, STI);
}

void AArch64::AArch64MCHLFIRewriter::rewriteTLSWrite(const MCInst &Inst,
                                                      MCStreamer &Out,
                                                      const MCSubtargetInfo &STI) {
  // msr tpidr_el0, xN  =>  str xN, [x25, #32]
  MCRegister SrcReg = Inst.getOperand(1).getReg();

  MCInst Store;
  Store.setOpcode(AArch64::STRXui);
  Store.addOperand(MCOperand::createReg(SrcReg));
  Store.addOperand(MCOperand::createReg(HLFIContextReg));
  Store.addOperand(MCOperand::createImm(HLFITPOffsetScaled));
  emitInst(Store, Out, STI);
}

bool AArch64::AArch64MCHLFIRewriter::rewriteInst(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  // Prevent recursion when we emit instructions ourselves.
  if (Guard)
    return false;

  // Check if HLFI rewriting is enabled for this subtarget.
  if (!STI.hasFeature(AArch64::FeatureHLFI))
    return false;

  // System call rewriting.
  if (isSyscall(Inst)) {
    rewriteSyscall(Inst, Out, STI);
    return true;
  }

  // TLS access rewriting.
  if (isTLSRead(Inst)) {
    rewriteTLSRead(Inst, Out, STI);
    return true;
  }

  if (isTLSWrite(Inst)) {
    rewriteTLSWrite(Inst, Out, STI);
    return true;
  }

  // All other instructions pass through unchanged.
  return false;
}
