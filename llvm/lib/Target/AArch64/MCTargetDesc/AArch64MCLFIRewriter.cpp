//===- AArch64MCLFIRewriter.cpp ---------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the AArch64MCLFIRewriter class, the AArch64 specific
// subclass of MCLFIRewriter.
//
//===----------------------------------------------------------------------===//

#include "AArch64MCLFIRewriter.h"
#include "AArch64AddressingModes.h"
#include "MCTargetDesc/AArch64MCTargetDesc.h"
#include "Utils/AArch64BaseInfo.h"

#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

// LFI reserved registers.
static constexpr MCRegister LFIBaseReg = AArch64::X27;
static constexpr MCRegister LFIAddrReg = AArch64::X28;
static constexpr MCRegister LFIScratchReg = AArch64::X26;
static constexpr MCRegister LFICtxReg = AArch64::X25;

// Offset into the context register block (pointed to by LFICtxReg) where the
// thread pointer is stored. This is a scaled offset (multiplied by 8 for
// 64-bit loads), so a value of 4 means an actual byte offset of 32.
static constexpr unsigned LFITPOffset = 4;

static bool isSyscall(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::SVC;
}

static bool isPrivilegedTP(int64_t Reg) {
  return Reg == AArch64SysReg::TPIDR_EL1 || Reg == AArch64SysReg::TPIDR_EL2 ||
         Reg == AArch64SysReg::TPIDR_EL3;
}

static bool isTPRead(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::MRS &&
         Inst.getOperand(1).getImm() == AArch64SysReg::TPIDR_EL0;
}

static bool isTPWrite(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::MSR &&
         Inst.getOperand(0).getImm() == AArch64SysReg::TPIDR_EL0;
}

static bool isPrivilegedTPAccess(const MCInst &Inst) {
  if (Inst.getOpcode() == AArch64::MRS)
    return isPrivilegedTP(Inst.getOperand(1).getImm());
  if (Inst.getOpcode() == AArch64::MSR)
    return isPrivilegedTP(Inst.getOperand(0).getImm());
  return false;
}

static bool isDCZVA(const MCInst &Inst) {
  // DC ZVA is encoded as SYSxt with op1=3, Cn=7, Cm=4, op2=1
  if (Inst.getOpcode() != AArch64::SYSxt)
    return false;
  return Inst.getOperand(0).getImm() == 3 && // op1
         Inst.getOperand(1).getImm() == 7 && // Cn
         Inst.getOperand(2).getImm() == 4 && // Cm
         Inst.getOperand(3).getImm() == 1;   // op2
}

bool AArch64MCLFIRewriter::mayModifyReserved(const MCInst &Inst) const {
  return mayModifyRegister(Inst, LFIAddrReg) ||
         mayModifyRegister(Inst, LFIBaseReg) ||
         mayModifyRegister(Inst, LFICtxReg);
}

void AArch64MCLFIRewriter::emitInst(const MCInst &Inst, MCStreamer &Out,
                                    const MCSubtargetInfo &STI) {
  Out.emitInstruction(Inst, STI);
}

void AArch64MCLFIRewriter::emitAddMask(MCRegister Dest, MCRegister Src,
                                       MCStreamer &Out,
                                       const MCSubtargetInfo &STI) {
  // add Dest, LFIBaseReg, W(Src), uxtw
  MCInst Inst;
  Inst.setOpcode(AArch64::ADDXrx);
  Inst.addOperand(MCOperand::createReg(Dest));
  Inst.addOperand(MCOperand::createReg(LFIBaseReg));
  Inst.addOperand(MCOperand::createReg(getWRegFromXReg(Src)));
  Inst.addOperand(
      MCOperand::createImm(AArch64_AM::getArithExtendImm(AArch64_AM::UXTW, 0)));
  emitInst(Inst, Out, STI);
}

void AArch64MCLFIRewriter::emitBranch(unsigned Opcode, MCRegister Target,
                                      MCStreamer &Out,
                                      const MCSubtargetInfo &STI) {
  MCInst Branch;
  Branch.setOpcode(Opcode);
  Branch.addOperand(MCOperand::createReg(Target));
  emitInst(Branch, Out, STI);
}

void AArch64MCLFIRewriter::emitMov(MCRegister Dest, MCRegister Src,
                                   MCStreamer &Out,
                                   const MCSubtargetInfo &STI) {
  // orr Dest, xzr, Src
  MCInst Inst;
  Inst.setOpcode(AArch64::ORRXrs);
  Inst.addOperand(MCOperand::createReg(Dest));
  Inst.addOperand(MCOperand::createReg(AArch64::XZR));
  Inst.addOperand(MCOperand::createReg(Src));
  Inst.addOperand(MCOperand::createImm(0));
  emitInst(Inst, Out, STI);
}

void AArch64MCLFIRewriter::emitSyscall(MCStreamer &Out,
                                       const MCSubtargetInfo &STI) {
  // Save LR to scratch.
  emitMov(LFIScratchReg, AArch64::LR, Out, STI);

  // Load syscall handler address from negative offset from sandbox base.
  MCInst Load;
  Load.setOpcode(AArch64::LDURXi);
  Load.addOperand(MCOperand::createReg(AArch64::LR));
  Load.addOperand(MCOperand::createReg(LFIBaseReg));
  Load.addOperand(MCOperand::createImm(-8));
  emitInst(Load, Out, STI);

  // Call the runtime.
  emitBranch(AArch64::BLR, AArch64::LR, Out, STI);

  // Restore LR with guard.
  emitAddMask(AArch64::LR, LFIScratchReg, Out, STI);
}

// svc #0
// ->
// mov x26, x30
// ldur x30, [x27, #-8]
// blr x30
// add x30, x27, w26, uxtw
void AArch64MCLFIRewriter::rewriteSyscall(const MCInst &, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
  emitSyscall(Out, STI);
}

// mrs xN, tpidr_el0
// ->
// ldr xN, [x25, #32]
void AArch64MCLFIRewriter::rewriteTPRead(const MCInst &Inst, MCStreamer &Out,
                                         const MCSubtargetInfo &STI) {
  MCRegister DestReg = Inst.getOperand(0).getReg();

  MCInst Load;
  Load.setOpcode(AArch64::LDRXui);
  Load.addOperand(MCOperand::createReg(DestReg));
  Load.addOperand(MCOperand::createReg(LFICtxReg));
  Load.addOperand(MCOperand::createImm(LFITPOffset));
  emitInst(Load, Out, STI);
}

// msr tpidr_el0, xN
// ->
// str xN, [x25, #32]
void AArch64MCLFIRewriter::rewriteTPWrite(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
  MCRegister SrcReg = Inst.getOperand(1).getReg();

  MCInst Store;
  Store.setOpcode(AArch64::STRXui);
  Store.addOperand(MCOperand::createReg(SrcReg));
  Store.addOperand(MCOperand::createReg(LFICtxReg));
  Store.addOperand(MCOperand::createImm(LFITPOffset));
  emitInst(Store, Out, STI);
}

// dc zva, xN
// ->
// add x28, x27, wN, uxtw
// dc zva, x28
void AArch64MCLFIRewriter::rewriteDCZVA(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  MCRegister AddrReg = Inst.getOperand(4).getReg();

  emitAddMask(LFIAddrReg, AddrReg, Out, STI);

  MCInst NewInst;
  NewInst.setOpcode(AArch64::SYSxt);
  NewInst.addOperand(Inst.getOperand(0)); // op1
  NewInst.addOperand(Inst.getOperand(1)); // Cn
  NewInst.addOperand(Inst.getOperand(2)); // Cm
  NewInst.addOperand(Inst.getOperand(3)); // op2
  NewInst.addOperand(MCOperand::createReg(LFIAddrReg));
  emitInst(NewInst, Out, STI);
}

void AArch64MCLFIRewriter::doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                                         const MCSubtargetInfo &STI) {
  // Reserved register modification is an error.
  if (mayModifyReserved(Inst)) {
    error(Inst, "illegal modification of reserved LFI register");
    return;
  }

  // System instructions.
  if (isSyscall(Inst))
    return rewriteSyscall(Inst, Out, STI);

  if (isTPRead(Inst))
    return rewriteTPRead(Inst, Out, STI);

  if (isTPWrite(Inst))
    return rewriteTPWrite(Inst, Out, STI);

  if (isPrivilegedTPAccess(Inst)) {
    error(Inst, "illegal access to privileged thread pointer register");
    return;
  }

  if (isDCZVA(Inst))
    return rewriteDCZVA(Inst, Out, STI);

  emitInst(Inst, Out, STI);
}

bool AArch64MCLFIRewriter::rewriteInst(const MCInst &Inst, MCStreamer &Out,
                                       const MCSubtargetInfo &STI) {
  if (!Enabled || Guard)
    return false;
  Guard = true;

  doRewriteInst(Inst, Out, STI);

  Guard = false;
  return true;
}
