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

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rt1,
    MCRegister Rt2, int64_t Imm1, int64_t Imm2,
    MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rt1));
  Inst.addOperand(MCOperand::createReg(Rt2));
  Inst.addOperand(MCOperand::createImm(Imm1));
  Inst.addOperand(MCOperand::createImm(Imm2));
  Out.emitInstruction(Inst, STI);
}

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rt,
    int64_t Imm1, int64_t Imm2,
    MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rt));
  Inst.addOperand(MCOperand::createImm(Imm1));
  Inst.addOperand(MCOperand::createImm(Imm2));
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

// Emit 'add Dest, LFIBaseReg, W(Src), uxtw'
static void emitAddMask(MCRegister Dest, MCRegister Src, MCStreamer &Out,
                        const MCSubtargetInfo &STI) {
  emit(AArch64::ADDXrx,
      Dest,
      LFIBaseReg,
      getWRegFromXReg(Src),
      AArch64_AM::getArithExtendImm(AArch64_AM::UXTW, 0),
      Out, STI);
}

// Emit 'Op(ld/st) Dest, [LFIBaseReg, W(Target), uxtw]'
static void emitMemMask(unsigned Op, MCRegister Dest, MCRegister Target, MCStreamer &Out, const MCSubtargetInfo &STI) {
  emit(Op, Dest, LFIBaseReg, getWRegFromXReg(Target), 0, 0, Out, STI);
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

void AArch64::AArch64MCLFIExpander::expandLRModification(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCRegister Scratch = getScratch();
  MCInst New;
  New.setLoc(Inst.getLoc());
  New.setOpcode(Inst.getOpcode());
  for (unsigned I = 0; I < Inst.getNumOperands(); ++I) {
    const MCOperand &Op = Inst.getOperand(I);
    if (Op.isReg() && Op.getReg() == AArch64::LR) {
      New.addOperand(MCOperand::createReg(Scratch));
    } else {
      New.addOperand(Op);
    }
  }
  if (mayLoad(New) || mayStore(New))
    expandLoadStore(New, Out, STI);
  else
    Out.emitInstruction(New, STI);
  emitAddMask(AArch64::LR, Scratch, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandStackModification(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  if (mayLoad(Inst) || mayStore(Inst)) {
    if (mayModifyLR(Inst))
      return expandLRModification(Inst, Out, STI);
    return Out.emitInstruction(Inst, STI);
  }

  MCInst ModInst;
  MCRegister Scratch = getScratch();
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(0).getReg() == AArch64::SP);
  ModInst.setOpcode(Inst.getOpcode());
  ModInst.setLoc(Inst.getLoc());
  ModInst.addOperand(MCOperand::createReg(Scratch));
  for (unsigned I = 1, E = Inst.getNumOperands(); I != E; ++I) {
    ModInst.addOperand(Inst.getOperand(I));
  }
  Out.emitInstruction(ModInst, STI);
  emitAddMask(AArch64::SP, Scratch, Out, STI);
}

static bool canConvertToRoW(unsigned Op);
static unsigned convertRoXToRoW(unsigned Op, unsigned &Shift);
static unsigned convertRoWToRoW(unsigned Op, unsigned &Shift);
static unsigned convertUiToRoW(unsigned Op);
static unsigned convertPreToRoW(unsigned Op);
static unsigned convertPostToRoW(unsigned Op);

static void emitSafeLoadStore(const MCInst &Inst, unsigned N, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst LoadStore;
  LoadStore.setOpcode(Inst.getOpcode());
  for (unsigned I = 0; I < N; ++I)
    LoadStore.addOperand(Inst.getOperand(I));
  LoadStore.addOperand(MCOperand::createReg(LFIAddrReg));
  for (unsigned I = N + 1; I < Inst.getNumOperands(); ++I)
    LoadStore.addOperand(Inst.getOperand(I));
  Out.emitInstruction(LoadStore, STI);
}

void AArch64::AArch64MCLFIExpander::expandLoadStoreBasic(const MCInst &Inst, MemInstInfo &MII,
    MCStreamer &Out, const MCSubtargetInfo &STI) {
  emitAddMask(LFIAddrReg, Inst.getOperand(MII.BaseRegIdx).getReg(), Out, STI);

  if (MII.IsPrePost) {
    assert(OffsetIdx != -1 && "Pre/Post must have valid OffsetIdx");
    return Out.getContext().reportWarning(
        Inst.getLoc(), "TODO: pre/post index");
  }

  return emitSafeLoadStore(Inst, MII.BaseRegIdx, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandLoadStoreRoW(const MCInst &Inst, MemInstInfo &MII,
    MCStreamer &Out, const MCSubtargetInfo &STI) {
  unsigned MemOp;
  unsigned Op = Inst.getOpcode();
  if ((MemOp = convertUiToRoW(Op)) != AArch64::INSTRUCTION_LIST_END) {
    auto OffsetMCO = Inst.getOperand(2);
    if (OffsetMCO.isImm() && OffsetMCO.getImm() == 0)
      return emitMemMask(MemOp, Inst.getOperand(0).getReg(), Inst.getOperand(1).getReg(), Out, STI);
    return expandLoadStoreBasic(Inst, MII, Out, STI);
  }

  if ((MemOp = convertPreToRoW(Op)) != AArch64::INSTRUCTION_LIST_END) {
    MCRegister Reg = Inst.getOperand(2).getReg();
    int64_t Imm = Inst.getOperand(3).getImm();
    if (Imm >= 0)
      emit(AArch64::ADDXri, Reg, Reg, Imm, 0, Out, STI);
    else
      emit(AArch64::SUBXri, Reg, Reg, -Imm, 0, Out, STI);
    return emitMemMask(MemOp, Inst.getOperand(1).getReg(), Reg, Out, STI);
  }

  if ((MemOp = convertPostToRoW(Op)) != AArch64::INSTRUCTION_LIST_END) {
    MCRegister Reg = Inst.getOperand(2).getReg();
    emitMemMask(MemOp, Inst.getOperand(1).getReg(), Reg, Out, STI);
    int64_t Imm = Inst.getOperand(3).getImm();
    if (Imm >= 0)
      emit(AArch64::ADDXri, Reg, Reg, Imm, 0, Out, STI);
    else
      emit(AArch64::SUBXri, Reg, Reg, -Imm, 0, Out, STI);
    return;
  }

  unsigned Shift;
  if ((MemOp = convertRoXToRoW(Op, Shift)) != AArch64::INSTRUCTION_LIST_END) {
    MCRegister Reg1 = Inst.getOperand(1).getReg();
    MCRegister Reg2 = Inst.getOperand(2).getReg();
    int64_t Extend = Inst.getOperand(3).getImm();
    int64_t IsShift = Inst.getOperand(4).getImm();
    MCRegister Scratch = getScratch();
    if (!IsShift)
      Shift = 0;
    if (Extend)
      emit(AArch64::ADDXrx, Scratch, Reg1, Reg2, AArch64_AM::getArithExtendImm(AArch64_AM::SXTX, Shift), Out, STI);
    else
      emit(AArch64::ADDXrs, Scratch, Reg1, Reg2, AArch64_AM::getShifterImm(AArch64_AM::LSL, Shift), Out, STI);
    return emitMemMask(MemOp, Inst.getOperand(0).getReg(), Scratch, Out, STI);
  }

  if ((MemOp = convertRoWToRoW(Op, Shift)) != AArch64::INSTRUCTION_LIST_END) {
    MCRegister Reg1 = Inst.getOperand(1).getReg();
    MCRegister Reg2 = Inst.getOperand(2).getReg();
    int64_t S = Inst.getOperand(3).getImm();
    int64_t IsShift = Inst.getOperand(4).getImm();
    MCRegister Scratch = getScratch();
    if (!IsShift)
      Shift = 0;
    if (S)
      emit(AArch64::ADDXrx, Scratch, Reg1, Reg2, AArch64_AM::getArithExtendImm(AArch64_AM::SXTW, Shift), Out, STI);
    else
      emit(AArch64::ADDXrx, Scratch, Reg1, Reg2, AArch64_AM::getArithExtendImm(AArch64_AM::UXTW, Shift), Out, STI);
    return emitMemMask(MemOp, Inst.getOperand(0).getReg(), Scratch, Out, STI);
  }
}

void AArch64::AArch64MCLFIExpander::expandLoadStore(const MCInst &Inst,
                                                    MCStreamer &Out,
                                                    const MCSubtargetInfo &STI) {
  auto MII = getLoadInfo(Inst);
  if (!MII.has_value()) {
    MII = getStoreInfo(Inst);
    if (!MII.has_value())
      return Out.getContext().reportError(
          Inst.getLoc(), "this load/store is not supported by LFI");
  }

  // Stack accesses without a register offset don't need rewriting.
  if (Inst.getOperand(MII->BaseRegIdx).getReg() == AArch64::SP) {
    if (MII->BaseRegIdx == (int) Inst.getNumOperands() - 1 ||
        !Inst.getOperand(MII->BaseRegIdx + 1).isReg())
      return Out.emitInstruction(Inst, STI);
  }

  // Try to convert to RoW if we can, otherwise use fallback.
  if (canConvertToRoW(Inst.getOpcode()))
    expandLoadStoreRoW(Inst, MII.value(), Out, STI);
  else
    expandLoadStoreBasic(Inst, MII.value(), Out, STI);
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

  // Bail out with an error. In the future, we could consider automatically
  // rewriting uses of reserved LFI registers.
  if (mayModifyReserved(Inst))
    return Out.getContext().reportError(
        Inst.getLoc(), "illegal modification of reserved LFI register");

  if (mayModifyStack(Inst))
    return expandStackModification(Inst, Out, STI);

  if (mayModifyLR(Inst))
    return expandLRModification(Inst, Out, STI);

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

static unsigned convertRoXToRoW(unsigned Op, unsigned &Shift) {
  Shift = 0;
  switch (Op) {
  case AArch64::LDRBBroX:
    return AArch64::LDRBBroW;
  case AArch64::LDRBroX:
    return AArch64::LDRBroW;
  case AArch64::LDRDroX:
    Shift = 3;
    return AArch64::LDRDroW;
  case AArch64::LDRHHroX:
    Shift = 1;
    return AArch64::LDRHHroW;
  case AArch64::LDRHroX:
    Shift = 1;
    return AArch64::LDRHroW;
  case AArch64::LDRQroX:
    Shift = 4;
    return AArch64::LDRQroW;
  case AArch64::LDRSBWroX:
    Shift = 1;
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXroX:
    Shift = 1;
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWroX:
    Shift = 1;
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXroX:
    Shift = 1;
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWroX:
    Shift = 2;
    return AArch64::LDRSWroW;
  case AArch64::LDRSroX:
    Shift = 2;
    return AArch64::LDRSroW;
  case AArch64::LDRWroX:
    Shift = 2;
    return AArch64::LDRWroW;
  case AArch64::LDRXroX:
    Shift = 3;
    return AArch64::LDRXroW;
  case AArch64::STRBBroX:
    return AArch64::STRBBroW;
  case AArch64::STRBroX:
    return AArch64::STRBroW;
  case AArch64::STRDroX:
    Shift = 3;
    return AArch64::STRDroW;
  case AArch64::STRHHroX:
    Shift = 1;
    return AArch64::STRHHroW;
  case AArch64::STRHroX:
    Shift = 1;
    return AArch64::STRHroW;
  case AArch64::STRQroX:
    Shift = 4;
    return AArch64::STRQroW;
  case AArch64::STRSroX:
    Shift = 2;
    return AArch64::STRSroW;
  case AArch64::STRWroX:
    Shift = 2;
    return AArch64::STRWroW;
  case AArch64::STRXroX:
    Shift = 3;
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertRoWToRoW(unsigned Op, unsigned &Shift) {
  Shift = 0;
  switch (Op) {
  case AArch64::LDRBBroW:
    return AArch64::LDRBBroW;
  case AArch64::LDRBroW:
    return AArch64::LDRBroW;
  case AArch64::LDRDroW:
    Shift = 3;
    return AArch64::LDRDroW;
  case AArch64::LDRHHroW:
    Shift = 1;
    return AArch64::LDRHHroW;
  case AArch64::LDRHroW:
    Shift = 1;
    return AArch64::LDRHroW;
  case AArch64::LDRQroW:
    Shift = 4;
    return AArch64::LDRQroW;
  case AArch64::LDRSBWroW:
    Shift = 1;
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXroW:
    Shift = 1;
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWroW:
    Shift = 1;
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXroW:
    Shift = 1;
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWroW:
    Shift = 2;
    return AArch64::LDRSWroW;
  case AArch64::LDRSroW:
    Shift = 2;
    return AArch64::LDRSroW;
  case AArch64::LDRWroW:
    Shift = 2;
    return AArch64::LDRWroW;
  case AArch64::LDRXroW:
    Shift = 3;
    return AArch64::LDRXroW;
  case AArch64::STRBBroW:
    return AArch64::STRBBroW;
  case AArch64::STRBroW:
    return AArch64::STRBroW;
  case AArch64::STRDroW:
    Shift = 3;
    return AArch64::STRDroW;
  case AArch64::STRHHroW:
    Shift = 1;
    return AArch64::STRHHroW;
  case AArch64::STRHroW:
    Shift = 1;
    return AArch64::STRHroW;
  case AArch64::STRQroW:
    Shift = 4;
    return AArch64::STRQroW;
  case AArch64::STRSroW:
    Shift = 2;
    return AArch64::STRSroW;
  case AArch64::STRWroW:
    Shift = 2;
    return AArch64::STRWroW;
  case AArch64::STRXroW:
    Shift = 3;
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertUiToRoW(unsigned Op) {
  switch (Op) {
  case AArch64::LDRBBui:
    return AArch64::LDRBBroW;
  case AArch64::LDRBui:
    return AArch64::LDRBroW;
  case AArch64::LDRDui:
    return AArch64::LDRDroW;
  case AArch64::LDRHHui:
    return AArch64::LDRHHroW;
  case AArch64::LDRHui:
    return AArch64::LDRHroW;
  case AArch64::LDRQui:
    return AArch64::LDRQroW;
  case AArch64::LDRSBWui:
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXui:
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWui:
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXui:
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWui:
    return AArch64::LDRSWroW;
  case AArch64::LDRSui:
    return AArch64::LDRSroW;
  case AArch64::LDRWui:
    return AArch64::LDRWroW;
  case AArch64::LDRXui:
    return AArch64::LDRXroW;
  case AArch64::STRBBui:
    return AArch64::STRBBroW;
  case AArch64::STRBui:
    return AArch64::STRBroW;
  case AArch64::STRDui:
    return AArch64::STRDroW;
  case AArch64::STRHHui:
    return AArch64::STRHHroW;
  case AArch64::STRHui:
    return AArch64::STRHroW;
  case AArch64::STRQui:
    return AArch64::STRQroW;
  case AArch64::STRSui:
    return AArch64::STRSroW;
  case AArch64::STRWui:
    return AArch64::STRWroW;
  case AArch64::STRXui:
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertPreToRoW(unsigned Op) {
  switch (Op) {
  case AArch64::LDRBBpre:
    return AArch64::LDRBBroW;
  case AArch64::LDRBpre:
    return AArch64::LDRBroW;
  case AArch64::LDRDpre:
    return AArch64::LDRDroW;
  case AArch64::LDRHHpre:
    return AArch64::LDRHHroW;
  case AArch64::LDRHpre:
    return AArch64::LDRHroW;
  case AArch64::LDRQpre:
    return AArch64::LDRQroW;
  case AArch64::LDRSBWpre:
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXpre:
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWpre:
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXpre:
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWpre:
    return AArch64::LDRSWroW;
  case AArch64::LDRSpre:
    return AArch64::LDRSroW;
  case AArch64::LDRWpre:
    return AArch64::LDRWroW;
  case AArch64::LDRXpre:
    return AArch64::LDRXroW;
  case AArch64::STRBBpre:
    return AArch64::STRBBroW;
  case AArch64::STRBpre:
    return AArch64::STRBroW;
  case AArch64::STRDpre:
    return AArch64::STRDroW;
  case AArch64::STRHHpre:
    return AArch64::STRHHroW;
  case AArch64::STRHpre:
    return AArch64::STRHroW;
  case AArch64::STRQpre:
    return AArch64::STRQroW;
  case AArch64::STRSpre:
    return AArch64::STRSroW;
  case AArch64::STRWpre:
    return AArch64::STRWroW;
  case AArch64::STRXpre:
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertPostToRoW(unsigned Op) {
  switch (Op) {
  case AArch64::LDRBBpost:
    return AArch64::LDRBBroW;
  case AArch64::LDRBpost:
    return AArch64::LDRBroW;
  case AArch64::LDRDpost:
    return AArch64::LDRDroW;
  case AArch64::LDRHHpost:
    return AArch64::LDRHHroW;
  case AArch64::LDRHpost:
    return AArch64::LDRHroW;
  case AArch64::LDRQpost:
    return AArch64::LDRQroW;
  case AArch64::LDRSBWpost:
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXpost:
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWpost:
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXpost:
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWpost:
    return AArch64::LDRSWroW;
  case AArch64::LDRSpost:
    return AArch64::LDRSroW;
  case AArch64::LDRWpost:
    return AArch64::LDRWroW;
  case AArch64::LDRXpost:
    return AArch64::LDRXroW;
  case AArch64::STRBBpost:
    return AArch64::STRBBroW;
  case AArch64::STRBpost:
    return AArch64::STRBroW;
  case AArch64::STRDpost:
    return AArch64::STRDroW;
  case AArch64::STRHHpost:
    return AArch64::STRHHroW;
  case AArch64::STRHpost:
    return AArch64::STRHroW;
  case AArch64::STRQpost:
    return AArch64::STRQroW;
  case AArch64::STRSpost:
    return AArch64::STRSroW;
  case AArch64::STRWpost:
    return AArch64::STRWroW;
  case AArch64::STRXpost:
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static bool canConvertToRoW(unsigned Op) {
  unsigned Shift;
  return convertUiToRoW(Op) != AArch64::INSTRUCTION_LIST_END ||
    convertPreToRoW(Op) != AArch64::INSTRUCTION_LIST_END ||
    convertPostToRoW(Op) != AArch64::INSTRUCTION_LIST_END ||
    convertRoXToRoW(Op, Shift) != AArch64::INSTRUCTION_LIST_END ||
    convertRoWToRoW(Op, Shift) != AArch64::INSTRUCTION_LIST_END;
}
