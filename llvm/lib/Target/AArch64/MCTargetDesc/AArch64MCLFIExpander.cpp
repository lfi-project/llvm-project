//===- AArch64MCLFIExpander.cpp ---------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the LFI authors.
//
//===----------------------------------------------------------------------===//
//
// This file implements the AArch64MCLFIExpander class, the AArch64 specific
// subclass of MCLFIExpander.
//
//===----------------------------------------------------------------------===//
#include "AArch64MCLFIExpander.h"
#include "Utils/AArch64BaseInfo.h"

#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

#define DEBUG_TYPE "lfi"

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

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rs, int64_t Imm, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rs));
  Inst.addOperand(MCOperand::createImm(Imm));
  Out.emitInstruction(Inst, STI);
}

static void emit(unsigned int Op, MCRegister Rd, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Out.emitInstruction(Inst, STI);
}

void AArch64::AArch64MCLFIExpander::emitFuncCall(StringRef FuncName, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Call;
  Call.setOpcode(AArch64::BL);
  MCSymbol *Sym = Out.getContext().getOrCreateSymbol(FuncName);
  const MCExpr *Expr = MCSymbolRefExpr::create(Sym, Out.getContext());
  Call.addOperand(MCOperand::createExpr(Expr));
  Out.emitInstruction(Call, STI);
}

void AArch64::AArch64MCLFIExpander::doExpandInst(const MCInst &Inst,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  const int64_t TPTLSOffset = 0;
  if (isTLSRead(Inst)) {
    return emit(AArch64::LDRXui,
        Inst.getOperand(0).getReg(),
        AArch64::X28,
        TPTLSOffset,
        Out, STI);
  } else if (isTLSWrite(Inst)) {
    return emit(AArch64::STRXui,
        Inst.getOperand(1).getReg(),
        AArch64::X28,
        TPTLSOffset,
        Out, STI);
  } else if (isSyscall(Inst)) {
    const int64_t TmpTLSOffset = 1;
    const int64_t SyscallTLSOffset = 2;
    emit(AArch64::STRXui, AArch64::LR, AArch64::X28, TmpTLSOffset, Out, STI);
    // emitFuncCall("lfi_weak_syscall_entry", Out, STI);
    emit(AArch64::LDRXui, AArch64::LR, AArch64::X28, SyscallTLSOffset, Out, STI);
    emit(AArch64::BLR, AArch64::LR, Out, STI);
    emit(AArch64::LDRXui, AArch64::LR, AArch64::X28, TmpTLSOffset, Out, STI);
    return;
  }

  Out.emitInstruction(Inst, STI);
}

bool AArch64::AArch64MCLFIExpander::expandInst(const MCInst &Inst,
                                               MCStreamer &Out,
                                               const MCSubtargetInfo &STI) {
  if (Guard)
    return false;
  Guard = true;

  doExpandInst(Inst, Out, STI);

  Guard = false;
  return true;
}
