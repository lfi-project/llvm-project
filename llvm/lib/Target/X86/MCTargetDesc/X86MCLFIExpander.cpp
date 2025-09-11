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

#define DEBUG_TYPE "lfi"

static const int BundleSize = 32;

static const MCRegister LFIBaseReg = X86::R14;

static MCRegister getReg64(MCRegister Reg) {
  switch (Reg) {
  default:
    return getX86SubSuperRegister(Reg, 64, false);
  case X86::IP:
  case X86::EIP:
  case X86::RIP:
    return X86::RIP;
  }
}

static MCRegister getReg32(MCRegister Reg) {
  switch (Reg) {
  default:
    return getX86SubSuperRegister(Reg, 32, false);
  case X86::IP:
  case X86::EIP:
    return X86::EIP;
  case X86::RIP:
    llvm_unreachable("Trying to demote %rip");
  }
}

bool X86::X86MCLFIExpander::isValidScratchRegister(MCRegister Reg) const {
  return false;
}

void X86::X86MCLFIExpander::expandDirectCall(const MCInst &Inst,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
  Out.emitInstruction(Inst, STI);
  Out.emitCodeAlignment(Align(BundleSize), &STI);
}

void X86::X86MCLFIExpander::emitSandboxBranchReg(MCRegister Reg, MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {

  MCInst AndInst;
  AndInst.setOpcode(X86::AND32ri8);

  MCOperand Target32 = MCOperand::createReg(getReg32(Reg));
  AndInst.addOperand(Target32);
  AndInst.addOperand(Target32);
  AndInst.addOperand(MCOperand::createImm(-BundleSize));
  Out.emitInstruction(AndInst, STI);

  MCInst Add;
  Add.setOpcode(X86::ADD64rr);

  MCOperand Target64 = MCOperand::createReg(getReg64(Reg));
  Add.addOperand(Target64);
  Add.addOperand(Target64);
  Add.addOperand(MCOperand::createReg(LFIBaseReg));
  Out.emitInstruction(Add, STI);
}

void X86::X86MCLFIExpander::emitIndirectJumpReg(MCRegister Reg, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  Out.emitBundleLock(false);
  emitSandboxBranchReg(Reg, Out, STI);

  MCInst Jmp;
  Jmp.setOpcode(X86::JMP64r);
  MCOperand Target = MCOperand::createReg(getReg64(Reg));
  Jmp.addOperand(Target);

  Out.emitInstruction(Jmp, STI);
  Out.emitBundleUnlock();
}

void X86::X86MCLFIExpander::emitIndirectCallReg(MCRegister Reg, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  MCOperand Target = MCOperand::createReg(getReg64(Reg));

  Out.emitBundleLock(false);
  emitSandboxBranchReg(Reg, Out, STI);
  MCInst Call;
  Call.setOpcode(X86::CALL64r);
  Call.addOperand(Target);
  Out.emitInstruction(Call, STI);
  Out.emitBundleUnlock();
  Out.emitCodeAlignment(Align(BundleSize), &STI);
}

void X86::X86MCLFIExpander::expandIndirectBranch(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  MCRegister Target;
  if (mayLoad(Inst)) {
    // indirect jmp/call through memory
    MCInst Mov;
    Mov.setOpcode(X86::MOV64rm);

    Target = X86::R11;

    Mov.addOperand(MCOperand::createReg(getReg64(Target)));
    Mov.addOperand(Inst.getOperand(0));
    Mov.addOperand(Inst.getOperand(1));
    Mov.addOperand(Inst.getOperand(2));
    Mov.addOperand(Inst.getOperand(3));
    Mov.addOperand(Inst.getOperand(4));
    doExpandInst(Mov, Out, STI, false);
  } else {
    Target = Inst.getOperand(0).getReg();
  }

  if (isCall(Inst))
    emitIndirectCallReg(Target, Out, STI);
  else
    emitIndirectJumpReg(Target, Out, STI);
}

static bool isValidReturnRegister(const MCRegister& Reg) {
  return Reg == X86::EAX ||
         Reg == X86::RAX ||
         Reg == X86::AL ||
         Reg == X86::AX ||
         Reg == X86::XMM0;
}

void X86::X86MCLFIExpander::expandReturn(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
  MCRegister ScratchReg = X86::R11;

  MCInst Pop;
  Pop.setOpcode(X86::POP64r);
  Pop.addOperand(MCOperand::createReg(ScratchReg));
  Out.emitInstruction(Pop, STI);

  if (Inst.getNumOperands() > 0) {
    if (Inst.getOpcode() == X86::RETI32 || Inst.getOpcode() == X86::RETI64) {
      MCOperand StackPointer = MCOperand::createReg(X86::RSP);

      MCInst Add;
      Add.setOpcode(X86::ADD64ri32);
      Add.addOperand(StackPointer);
      Add.addOperand(StackPointer);
      Add.addOperand(Inst.getOperand(0));

      doExpandInst(Add, Out, STI, false);
    } else if (Inst.getOpcode() == X86::RET32 ||
               Inst.getOpcode() == X86::RET64) {
      LLVM_DEBUG(Inst.dump());
      assert(Inst.getOperand(0).isReg());
      assert(isValidReturnRegister(Inst.getOperand(0).getReg()));
    } else {
      LLVM_DEBUG(Inst.dump());
      Error(Inst, "Unexpected return instruction.");
    }
  }

  emitIndirectJumpReg(ScratchReg, Out, STI);
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

// Returns true if Inst is an X86 prefix
static bool isPrefix(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case X86::LOCK_PREFIX:
  case X86::REP_PREFIX:
  case X86::REPNE_PREFIX:
  case X86::REX64_PREFIX:
    return true;
  default:
    return false;
  }
}

static bool isDirectCall(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
    case X86::CALLpcrel32:
    case X86::CALL64pcrel32:
      return true;
    default:
      return false;
  }
}

// Emit prefixes + instruction if EmitPrefixes argument is true.
// Otherwise, emit the bare instruction.
void X86::X86MCLFIExpander::emitInstruction(const MCInst &Inst,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI,
                                             bool EmitPrefixes) {
  if (EmitPrefixes) {
    for (const MCInst &Prefix : Prefixes)
      Out.emitInstruction(Prefix, STI);
    Prefixes.clear();
  }
  Out.emitInstruction(Inst, STI);
}

void X86::X86MCLFIExpander::doExpandInst(const MCInst &Inst,
                                          MCStreamer &Out,
                                          const MCSubtargetInfo &STI,
                                          bool EmitPrefixes) {
  if (isPrefix(Inst)) {
    return Prefixes.push_back(Inst);
  }
  if (isDirectCall(Inst)) {
    expandDirectCall(Inst, Out, STI);
  } else if (isIndirectBranch(Inst) || isCall(Inst)) {
    expandIndirectBranch(Inst, Out, STI);
  } else if (isReturn(Inst)) {
    expandReturn(Inst, Out, STI);
  } else {
    emitInstruction(Inst, Out, STI, EmitPrefixes);
  }
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
