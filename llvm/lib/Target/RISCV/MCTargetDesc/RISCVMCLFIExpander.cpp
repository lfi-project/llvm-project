//===- RISCVMCLFIExpander.cpp - RISC-V LFI Expander ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the RISCVMCLFIExpander class, the RISC-V specific
// subclass of MCLFIExpander.
//
// This file was written by the Native Client authors, modified for LFI.
//
//===----------------------------------------------------------------------===//
#include "RISCVMCLFIExpander.h"
#include "RISCVMCTargetDesc.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Alignment.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
using namespace llvm;
#define DEBUG_TYPE "lfi"

static cl::opt<bool> RISCVLFIErrorReserved(
    "riscv-lfi-error-reserved", cl::Hidden, cl::init(false),
    cl::desc("Produce errors for uses of LFI reserved registers"));

static const int BundleSize = 8;
static const MCRegister LFIBaseReg = RISCV::X27;
static const MCRegister LFIAddrReg = RISCV::X9;
//static const MCRegister LFIScratchReg = RISCV::X27;

static bool isAbsoluteReg(MCRegister Reg) {
  return (Reg == LFIBaseReg || Reg == RISCV::X2); // X2 is stack pointer
}

static bool isDirectCall(const MCInst &Inst) {
  // have a case JALR x1, add op at the end to disticnt parent class
  // Psuedocall is a seperate case and true 
  // what should the sandboxing be-- research question 
  // if i see x26 replace with x9
  switch (Inst.getOpcode()) {
  case RISCV::PseudoCALL:
  case RISCV::JAL:
    return true;
  default:
    return false;
  }
}

static bool isIndirectBranch(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case RISCV::JALR:
    return true;
  default:
    return false;
  }
}

static bool isCall(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case RISCV::PseudoCALL:
  case RISCV::JALR:
    return Inst.getOperand(0).getReg() == RISCV::X1;
  default:
    return false;
  }
}

static bool isReturn(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case RISCV::PseudoRET:
  case RISCV::JALR:
    if (Inst.getOpcode() == RISCV::JALR) {
      return Inst.getOperand(0).getReg() == RISCV::X0 &&
             Inst.getOperand(1).getReg() == RISCV::X1 &&
             Inst.getOperand(2).getImm() == 0;
    }
    return true;
  default:
    return false;
  }
}

static bool isSyscall(const MCInst &Inst) {
  return Inst.getOpcode() == RISCV::ECALL;
}

static bool isLoadStore(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case RISCV::LB: case RISCV::LBU: case RISCV::LH: case RISCV::LHU:
  case RISCV::LW: case RISCV::LWU: case RISCV::LD:
  case RISCV::SB: case RISCV::SH: case RISCV::SW: case RISCV::SD:
  case RISCV::FLH: case RISCV::FLW: case RISCV::FLD:
  case RISCV::FSH: case RISCV::FSW: case RISCV::FSD:
    return true;
  default:
    return false;
  }
}

static bool explicitlyModifiesRegister(const MCInst &Inst, MCRegister Reg) {
  for (unsigned i = 0; i < Inst.getNumOperands(); ++i) {
    const MCOperand &Op = Inst.getOperand(i);
    if (Op.isReg() && Op.getReg() == Reg) {
      if (i == 0) return true;
    }
  }
  return false;
}

bool RISCV::RISCVMCLFIExpander::isValidScratchRegister(MCRegister Reg) const {
  return Reg != LFIBaseReg && Reg != RISCV::X2;
}

void RISCV::RISCVMCLFIExpander::expandDirectCall(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  Out.emitInstruction(Inst, STI);
  Out.emitCodeAlignment(Align(BundleSize), &STI);
}

void RISCV::RISCVMCLFIExpander::emitSandboxBranchReg(MCRegister Reg,
                                                      MCStreamer &Out,
                                                      const MCSubtargetInfo &STI) {
  
  MCInst AndInst;
  AndInst.setOpcode(RISCV::ANDI);
  AndInst.addOperand(MCOperand::createReg(Reg));
  AndInst.addOperand(MCOperand::createReg(Reg));
  AndInst.addOperand(MCOperand::createImm(-BundleSize));
  Out.emitInstruction(AndInst, STI);

  // add rd, rs, base_reg
  MCInst Add;
  Add.setOpcode(RISCV::ADD_UW);
  Add.addOperand(MCOperand::createReg(Reg));
  Add.addOperand(MCOperand::createReg(Reg));
  Add.addOperand(MCOperand::createReg(LFIBaseReg));
  Out.emitInstruction(Add, STI);
}

void RISCV::RISCVMCLFIExpander::emitIndirectJumpReg(MCRegister Reg,
                                                     MCStreamer &Out,
                                                     const MCSubtargetInfo &STI) {
  
  // jalr x0, reg, 0 (indirect jump)
  MCInst Jmp;
  Jmp.setOpcode(RISCV::JALR);
  Jmp.addOperand(MCOperand::createReg(RISCV::X0));
  Jmp.addOperand(MCOperand::createReg(Reg));
  Jmp.addOperand(MCOperand::createImm(0));
  Out.emitInstruction(Jmp, STI);
}

void RISCV::RISCVMCLFIExpander::emitIndirectCallReg(MCRegister Reg,
                                                     MCStreamer &Out,
                                                     const MCSubtargetInfo &STI) {

  MCInst Call;
  Call.setOpcode(RISCV::JALR);
  Call.addOperand(MCOperand::createReg(RISCV::X1));
  Call.addOperand(MCOperand::createReg(Reg));
  Call.addOperand(MCOperand::createImm(0));
  Out.emitInstruction(Call, STI);
  Out.emitCodeAlignment(Align(BundleSize), &STI);
}

void RISCV::RISCVMCLFIExpander::expandIndirectBranch(const MCInst &Inst,
                                                      MCStreamer &Out,
                                                      const MCSubtargetInfo &STI) {
  MCRegister Target;
  
  
  if (Inst.getOpcode() == RISCV::JALR) {
    Target = Inst.getOperand(1).getReg(); 
  } else {

    Target = Inst.getOperand(0).getReg();
  }
  
  if (isCall(Inst))
    emitIndirectCallReg(Target, Out, STI);
  else
    emitIndirectJumpReg(Target, Out, STI);
}

void RISCV::RISCVMCLFIExpander::expandReturn(const MCInst &Inst, 
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
  MCInst Jmp;
  Jmp.setOpcode(RISCV::JALR);
  Jmp.addOperand(MCOperand::createReg(RISCV::X0));   
  Jmp.addOperand(MCOperand::createReg(RISCV::X1));   
  Jmp.addOperand(MCOperand::createImm(0));           
  Out.emitInstruction(Jmp, STI);
}

void RISCV::RISCVMCLFIExpander::expandLoadStore(const MCInst &Inst, 
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI,
                                                 bool EmitPrefixes) {
  MCInst SandboxedInst(Inst);
  
  MCOperand &Base = SandboxedInst.getOperand(1);
  
  if (Base.isReg() && isAbsoluteReg(Base.getReg())) {
    emitInstruction(SandboxedInst, Out, STI, EmitPrefixes);
    return;
  }
  
  Out.emitBundleLock(false);

  MCInst ClearBits;
 
  MCInst AddUW;
  AddUW.setOpcode(RISCV::ADD_UW);
  AddUW.addOperand(MCOperand::createReg(LFIAddrReg));        
  AddUW.addOperand(MCOperand::createReg(Base.getReg()));    
  AddUW.addOperand(MCOperand::createReg(LFIBaseReg));        
  Out.emitInstruction(AddUW, STI);

  
  Base.setReg(LFIAddrReg);
  
  emitInstruction(SandboxedInst, Out, STI, EmitPrefixes);
  Out.emitBundleUnlock();
}

void RISCV::RISCVMCLFIExpander::emitPrefixes(MCStreamer &Out, const MCSubtargetInfo &STI) {
  Prefixes.clear();
}

void RISCV::RISCVMCLFIExpander::expandStringOperation(const MCInst &Inst,
                                                       MCStreamer &Out,
                                                       const MCSubtargetInfo &STI,
                                                       bool EmitPrefixes) {
  emitInstruction(Inst, Out, STI, EmitPrefixes);
}

void RISCV::RISCVMCLFIExpander::expandExplicitStackManipulation(MCRegister StackReg,
                                                                 const MCInst &Inst,
                                                                 MCStreamer &Out,
                                                                 const MCSubtargetInfo &STI,
                                                                 bool EmitPrefixes) {
  expandLoadStore(Inst, Out, STI, EmitPrefixes);
}

void RISCV::RISCVMCLFIExpander::expandStackRegPush(const MCInst &Inst,
                                                    MCStreamer &Out,
                                                    const MCSubtargetInfo &STI) {
  emitInstruction(Inst, Out, STI, false);
}



void RISCV::RISCVMCLFIExpander::emitSandboxMemOp(MCInst &Inst, int MemIdx,
                                                  MCRegister ScratchReg,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  if (MemIdx >= (int)Inst.getNumOperands()) return;
  MCOperand &Base = Inst.getOperand(MemIdx);
  if (!Base.isReg()) return;
  if (isAbsoluteReg(Base.getReg())) return;

  
  MCInst AddUW;
  AddUW.setOpcode(RISCV::ADD_UW);
  AddUW.addOperand(MCOperand::createReg(LFIAddrReg));
  AddUW.addOperand(MCOperand::createReg(Base.getReg()));
  AddUW.addOperand(MCOperand::createReg(LFIBaseReg));
  Out.emitInstruction(AddUW, STI);

  Base.setReg(LFIAddrReg);
}

bool RISCV::RISCVMCLFIExpander::emitSandboxMemOps(MCInst &Inst,
                                                   MCRegister ScratchReg,
                                                   MCStreamer &Out,
                                                   const MCSubtargetInfo &STI,
                                                   bool EmitInstructions) {
  // For RISC-V load/store instructions, the base register is typically operand 1
  if (!isLoadStore(Inst)) return false;
  
  if (!EmitInstructions) {
    MCOperand &Base = Inst.getOperand(1);
    return Base.isReg() && !isAbsoluteReg(Base.getReg());
  }
  
  // Actually emit the sandboxing instructions
  bool emittedInstructions = false;
  MCOperand &Base = Inst.getOperand(1);
  if (Base.isReg() && !isAbsoluteReg(Base.getReg())) {
    emitSandboxMemOp(Inst, 1, ScratchReg, Out, STI);
    emittedInstructions = true;
  }
  
  return emittedInstructions;
}

void RISCV::RISCVMCLFIExpander::emitInstruction(const MCInst &Inst, 
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI,
                                                 bool EmitPrefixes) {
  Out.emitInstruction(Inst, STI);
}

static MCInst replaceReg(const MCInst &Inst, MCRegister Dest, MCRegister Src) {
  MCInst New;
  New.setOpcode(Inst.getOpcode());
  New.setLoc(Inst.getLoc());
  for (unsigned I = 0; I < Inst.getNumOperands(); ++I) {
    const MCOperand &Op = Inst.getOperand(I);
    if (Op.isReg() && Op.getReg() == Src) {
      New.addOperand(MCOperand::createReg(Dest));
    } else {
      New.addOperand(Op);
    }
  }
  return New;
}

void RISCV::RISCVMCLFIExpander::doExpandInst(const MCInst &Inst, MCStreamer &Out,
                                              const MCSubtargetInfo &STI,
                                              bool EmitPrefixes) {
  // Check for modifications to reserved registers
  if (explicitlyModifiesRegister(Inst, LFIBaseReg)) {
    if (RISCVLFIErrorReserved)
      return Out.getContext().reportError(
          Inst.getLoc(), "illegal modification of reserved LFI register");
    Out.getContext().reportWarning(
        Inst.getLoc(), "deleting modification of reserved LFI register");
    MCInst New = replaceReg(Inst, LFIAddrReg, LFIBaseReg);
    return doExpandInst(New, Out, STI, EmitPrefixes);
  } else if (isSyscall(Inst)) {
    emitInstruction(Inst, Out, STI, EmitPrefixes);
  } else if (isDirectCall(Inst)) {
    expandDirectCall(Inst, Out, STI);
  } else if (isIndirectBranch(Inst) || isCall(Inst)) {
    expandIndirectBranch(Inst, Out, STI);
  } else if (isReturn(Inst)) {
    expandReturn(Inst, Out, STI);
  } else if (isLoadStore(Inst)) {
    expandLoadStore(Inst, Out, STI, EmitPrefixes);
  } else if (explicitlyModifiesRegister(Inst, RISCV::X2)) {
    expandLoadStore(Inst, Out, STI, EmitPrefixes);
  } else {
    emitInstruction(Inst, Out, STI, EmitPrefixes);
  }
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