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

static MCRegister LFIAddrReg = AArch64::X28;
static MCRegister LFIBaseReg = AArch64::X27;
static MCRegister LFIScratchReg = AArch64::X26;

bool AArch64::AArch64MCLFIExpander::isValidScratchRegister(MCRegister Reg) const {
  return Reg != AArch64::SP;
}

MCRegister AArch64::AArch64MCLFIExpander::getScratch() {
  if (numScratchRegs() == 0) {
    return LFIScratchReg;
  }
  return getScratchReg(0);
}

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rs,
    int64_t Imm, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rs));
  Inst.addOperand(MCOperand::createImm(Imm));
  Out.emitInstruction(Inst, STI);
}

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rt1,
    MCRegister Rt2, int64_t Imm, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rt1));
  Inst.addOperand(MCOperand::createReg(Rt2));
  Inst.addOperand(MCOperand::createImm(Imm));
  Out.emitInstruction(Inst, STI);
}

static void emit(unsigned int Op, MCRegister Reg, MCStreamer &Out,
    const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Reg));
  Out.emitInstruction(Inst, STI);
}

static void emitMov(MCRegister Dest, MCRegister Src, MCStreamer &Out, const MCSubtargetInfo &STI) {
  emit(AArch64::ORRXrs, Dest, AArch64::XZR, Src, 0, Out, STI);
}

static void emitAddMask(MCRegister Dest, MCRegister Src, MCStreamer &Out,
                        const MCSubtargetInfo &STI) {
  emit(AArch64::ADDXrx,
      Dest,
      LFIBaseReg,
      getWRegFromXReg(Src),
      AArch64_AM::getArithExtendImm(AArch64_AM::UXTW, 0),
      Out, STI);
}

static void emitBranch(unsigned int Opcode, MCRegister Target, MCStreamer &Out,
                       const MCSubtargetInfo &STI) {
  MCInst Branch;
  Branch.setOpcode(Opcode);
  Branch.addOperand(MCOperand::createReg(Target));
  Out.emitInstruction(Branch, STI);
}

void AArch64::AArch64MCLFIExpander::expandIndirectBranch(const MCInst &Inst,
                                                         MCStreamer &Out,
                                                         const MCSubtargetInfo &STI,
                                                         bool IsCall) {
  (void) IsCall;

  assert(Inst.getOperand(0).isReg());
  MCRegister BranchReg = Inst.getOperand(0).getReg();

  emitAddMask(LFIAddrReg, BranchReg, Out, STI);
  emitBranch(Inst.getOpcode(), LFIAddrReg, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandCall(const MCInst &Inst, MCStreamer &Out,
                                               const MCSubtargetInfo &STI) {
  if (Inst.getOperand(0).isReg())
    expandIndirectBranch(Inst, Out, STI, true);
  else
    Out.emitInstruction(Inst, STI);
}

void AArch64::AArch64MCLFIExpander::expandReturn(const MCInst &Inst, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  assert(Inst.getOperand(0).isReg());
  if (Inst.getOperand(0).getReg() != AArch64::LR)
    expandIndirectBranch(Inst, Out, STI, false);
  else
    Out.emitInstruction(Inst, STI);
}

bool AArch64::AArch64MCLFIExpander::mayModifyStack(const MCInst &Inst) {
  return mayModifyRegister(Inst, AArch64::SP);
}

bool AArch64::AArch64MCLFIExpander::mayModifyReserved(const MCInst &Inst) {
  return mayModifyRegister(Inst, LFIAddrReg) || mayModifyRegister(Inst, LFIBaseReg);
}

bool AArch64::AArch64MCLFIExpander::mayModifyLR(const MCInst &Inst) {
  return mayModifyRegister(Inst, AArch64::LR);
}

// Rewrites for modifications of "special" registers: x28, x27, lr.
void AArch64::AArch64MCLFIExpander::expandSpecialModification(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
}

void AArch64::AArch64MCLFIExpander::expandStackModification(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  if (mayLoad(Inst) || mayStore(Inst)) {
    if (mayModifyReserved(Inst) || mayModifyLR(Inst))
      return expandSpecialModification(Inst, Out, STI);
    return Out.emitInstruction(Inst, STI);
  }

  MCInst ModInst;
  MCRegister Scratch = getScratch();
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(0).getReg() == AArch64::SP);
  ModInst.setOpcode(Inst.getOpcode());
  ModInst.addOperand(MCOperand::createReg(Scratch));
  for (unsigned I = 1, E = Inst.getNumOperands(); I != E; ++I) {
    ModInst.addOperand(Inst.getOperand(I));
  }
  Out.emitInstruction(ModInst, STI);
  emitAddMask(AArch64::SP, Scratch, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandLoadStore(const MCInst &Inst,
                                                    MCStreamer &Out,
                                                    const MCSubtargetInfo &STI) {
}

void AArch64::AArch64MCLFIExpander::emitLFICall(LFICallType CallType, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCRegister Scratch = getScratch();
  emitMov(Scratch, AArch64::LR, Out, STI);
  unsigned Offset;
  switch (CallType) {
    case LFISyscall: Offset = 0; break;
    case LFITLSRead: Offset = 1; break;
    case LFITLSWrite: Offset = 2; break;
  }
  emit(AArch64::LDRXui, AArch64::LR, LFIBaseReg, Offset, Out, STI);
  emit(AArch64::BLR, AArch64::LR, Out, STI);
  emitAddMask(AArch64::LR, Scratch, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandSyscall(const MCInst &Inst, MCStreamer &Out,
                   const MCSubtargetInfo &STI) {
  emitLFICall(LFISyscall, Out, STI);
}

static void emitSwap(MCRegister Reg1, MCRegister Reg2, MCStreamer &Out, const MCSubtargetInfo &STI) {
  emit(AArch64::EORXrs, Reg1, Reg1, Reg2, 0, Out, STI);
  emit(AArch64::EORXrs, Reg2, Reg1, Reg2, 0, Out, STI);
  emit(AArch64::EORXrs, Reg1, Reg1, Reg2, 0, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandTLSRead(const MCInst &Inst, MCStreamer &Out,
                   const MCSubtargetInfo &STI) {
  MCRegister Reg = Inst.getOperand(0).getReg();
  if (Reg == AArch64::X0) {
    emitLFICall(LFITLSRead, Out, STI);
  } else {
    emitMov(Reg, AArch64::X0, Out, STI);
    emitLFICall(LFITLSRead, Out, STI);
    emitSwap(AArch64::X0, Reg, Out, STI);
  }
}

void AArch64::AArch64MCLFIExpander::expandTLSWrite(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI) {
  MCRegister Reg = Inst.getOperand(1).getReg();
  if (Reg == AArch64::X0) {
    emitLFICall(LFITLSWrite, Out, STI);
  } else {
    emitSwap(Reg, AArch64::X0, Out, STI);
    emitLFICall(LFITLSWrite, Out, STI);
    emitSwap(AArch64::X0, Reg, Out, STI);
  }
}

static bool isSyscall(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::SVC;
}

static bool isTLSRead(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::MRS &&
    Inst.getOperand(1).getReg() == AArch64SysReg::TPIDR_EL0;
}

static bool isTLSWrite(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::MSR &&
    Inst.getOperand(0).getReg() == AArch64SysReg::TPIDR_EL0;
}

void AArch64::AArch64MCLFIExpander::doExpandInst(const MCInst &Inst, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  if (isSyscall(Inst))
    return expandSyscall(Inst, Out, STI);

  if (isTLSRead(Inst))
    return expandTLSRead(Inst, Out, STI);

  if (isTLSWrite(Inst))
    return expandTLSWrite(Inst, Out, STI);

  if (isReturn(Inst))
    return expandReturn(Inst, Out, STI);

  if (isIndirectBranch(Inst))
    return expandIndirectBranch(Inst, Out, STI, false);

  if (isCall(Inst))
    return expandCall(Inst, Out, STI);

  if (isBranch(Inst))
    return Out.emitInstruction(Inst, STI);

  if (mayModifyStack(Inst))
    return expandStackModification(Inst, Out, STI);

  if (mayModifyReserved(Inst) || mayModifyLR(Inst))
    return expandSpecialModification(Inst, Out, STI);

  if (mayLoad(Inst) || mayStore(Inst))
    return expandLoadStore(Inst, Out, STI);

  return Out.emitInstruction(Inst, STI);
}

bool AArch64::AArch64MCLFIExpander::expandInst(const MCInst &Inst, MCStreamer &Out,
                                               const MCSubtargetInfo &STI) {
  if (Guard)
    return false;
  Guard = true;

  doExpandInst(Inst, Out, STI);

  Guard = false;
  return true;
}
