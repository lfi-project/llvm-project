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

bool AArch64::AArch64MCLFIExpander::isValidScratchRegister(MCRegister Reg) const {
  return Reg != AArch64::SP;
}

static void emitAddMask(MCRegister Dest, MCRegister Src, MCStreamer &Out,
                        const MCSubtargetInfo &STI) {
  MCInst Add;
  Add.setOpcode(AArch64::ADDXrx);
  Add.addOperand(MCOperand::createReg(Dest));
  Add.addOperand(MCOperand::createReg(LFIBaseReg));
  Add.addOperand(MCOperand::createReg(getWRegFromXReg(Src)));
  Add.addOperand(MCOperand::createImm(AArch64_AM::getArithExtendImm(AArch64_AM::UXTW, 0)));
  Out.emitInstruction(Add, STI);;
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
  if (isReturn(Inst))
    return expandReturn(Inst, Out, STI);

  if (isIndirectBranch(Inst))
    return expandIndirectBranch(Inst, Out, STI, false);

  if (isCall(Inst))
    return expandCall(Inst, Out, STI);

  if (isBranch(Inst))
    return Out.emitInstruction(Inst, STI);

  if (mayAffectControlFlow(Inst))
    return expandControlFlow(Inst, Out, STI);

  if (mayModifyStack(Inst))
    return expandStackManipulation(Inst, Out, STI);

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
