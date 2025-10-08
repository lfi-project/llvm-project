//===- RISCVMCLFIExpander.h - RISCV LFI Expander -------------------*- C++-*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the Native Client authors.
//
//
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

static const MCRegister LFIBaseReg   = RISCV::X27; // s11
static const MCRegister LFIAddrReg   = RISCV::X9;  // s1
static const MCRegister LFICtrlReg   = RISCV::X25; // s9
static const MCRegister LFITmpReg    = RISCV::X26; // s10
static const MCRegister LFIReturnReg = RISCV::X1;  // ra
static const MCRegister LFIStackReg  = RISCV::X2;  // sp
static const MCRegister LFIThreadReg = RISCV::X4;  // tp

// andi mask for 8-byte bundle alignment (imm12 = -8)
static const int64_t BundleMaskImm = -BundleSize;


static inline MCOperand R(MCRegister Reg) { return MCOperand::createReg(Reg); }
static inline MCOperand I64(int64_t Imm)  { return MCOperand::createImm(Imm); }

static bool isAbsoluteReg(MCRegister Reg) {
  // Allowed as base without sandboxing for address formation:
  //   - LFI base (s11)
  //   - stack pointer (sp)
  return (Reg == LFIBaseReg || Reg == LFIStackReg);
}

static bool isLoad(const MCInst &I) {
  switch (I.getOpcode()) {
  case RISCV::LB: case RISCV::LBU: case RISCV::LH: case RISCV::LHU:
  case RISCV::LW: case RISCV::LWU: case RISCV::LD:
  case RISCV::FLH: case RISCV::FLW: case RISCV::FLD:
  case RISCV::C_LBU:
  case RISCV::C_LD:
  case RISCV::C_LDSP:
  case RISCV::C_LH:
  case RISCV::C_LHU:
  case RISCV::C_LH_INX:
  case RISCV::C_LW:
  case RISCV::C_LWSP:
  case RISCV::C_LWSP_INX:
  case RISCV::C_LW_INX:
    return true;
  default:
    return false;
  }
}

static bool isStore(const MCInst &I) {
  switch (I.getOpcode()) {
  case RISCV::SB: case RISCV::SH: case RISCV::SW: case RISCV::SD:
  case RISCV::FSH: case RISCV::FSW: case RISCV::FSD:
  case RISCV:: C_SB: case RISCV:: C_SD: case RISCV:: C_SDSP:
  case RISCV:: C_SH: case RISCV:: C_SH_INX: case RISCV:: C_SW:
  case RISCV:: C_SWSP: case RISCV:: C_SWSP_INX: case RISCV:: C_SW_INX:
    return true;
  default:
    return false;
  }
}

static bool isLoadStore(const MCInst &I) { return isLoad(I) || isStore(I); }

static bool explicitlyModifiesRegister(const MCInst &Inst, MCRegister Reg) {
  if (Inst.getNumOperands() == 0) return false;
  const MCOperand &Op0 = Inst.getOperand(0);
  return Op0.isReg() && Op0.getReg() == Reg;
}

static bool isJAL(const MCInst &I)  { return I.getOpcode() == RISCV::JAL; }
static bool isJALR(const MCInst &I) { return I.getOpcode() == RISCV::JALR; }
static bool isSyscall(const MCInst &I) { return I.getOpcode() == RISCV::ECALL; }

static bool isCall(const MCInst &I) {
  // Writes ra (x1)
  if (isJAL(I))
    return I.getNumOperands() >= 1 && I.getOperand(0).isReg() &&
           I.getOperand(0).getReg() == LFIReturnReg;
  if (isJALR(I))
    return I.getNumOperands() >= 1 && I.getOperand(0).isReg() &&
           I.getOperand(0).getReg() == LFIReturnReg;
  return false;
}

static bool isRVIndirectBranch(const MCInst &I) {
  // jalr not writing ra is an indirect jump
  return isJALR(I);
}

static bool isReturn(const MCInst &I) {
  if (I.getOpcode() == RISCV::PseudoRET) return true;
  if (!isJALR(I)) return false;
  // ret == jalr x0, ra, 0
  return I.getNumOperands() >= 3 &&
         I.getOperand(0).isReg() && I.getOperand(0).getReg() == RISCV::X0 &&
         I.getOperand(1).isReg() && I.getOperand(1).getReg() == LFIReturnReg &&
         I.getOperand(2).isImm() && I.getOperand(2).getImm() == 0;
}

static bool isADDI(const MCInst &I) { return I.getOpcode() == RISCV::ADDI; }
static bool isADD (const MCInst &I) { return I.getOpcode() == RISCV::ADD;  }
static bool isSUB (const MCInst &I) { return I.getOpcode() == RISCV::SUB;  }

static bool isMV(const MCInst &I) {
  return isADDI(I) && I.getNumOperands() == 3 &&
         I.getOperand(2).isImm() && I.getOperand(2).getImm() == 0;
}

static bool isLdRaFromSp(const MCInst &I) {
  // ld ra, 0(sp) → special RA mask sequence
  if (I.getOpcode() != RISCV::LD || I.getNumOperands() < 3) return false;
  return I.getOperand(0).isReg() && I.getOperand(0).getReg() == LFIReturnReg &&
         I.getOperand(1).isReg() && I.getOperand(1).getReg() == LFIStackReg &&
         I.getOperand(2).isImm() && I.getOperand(2).getImm() == 0;
}

static bool isMvRaX(const MCInst &I) {
  if (!isMV(I)) return false;
  return I.getOperand(0).isReg() && I.getOperand(0).getReg() == LFIReturnReg;
}

static bool isStackAddi(const MCInst &I) {
  // addi sp, sp, imm
  return isADDI(I) &&
         I.getOperand(0).isReg() && I.getOperand(0).getReg() == LFIStackReg &&
         I.getOperand(1).isReg() && I.getOperand(1).getReg() == LFIStackReg;
}

static bool isAddSpSpX(const MCInst &I) {
  // add sp, sp, xN
  return isADD(I) &&
         I.getOperand(0).isReg() && I.getOperand(0).getReg() == LFIStackReg &&
         I.getOperand(1).isReg() && I.getOperand(1).getReg() == LFIStackReg;
}

static bool isSubSpSpX(const MCInst &I) {
  // sub sp, sp, xN
  return isSUB(I) &&
         I.getOperand(0).isReg() && I.getOperand(0).getReg() == LFIStackReg &&
         I.getOperand(1).isReg() && I.getOperand(1).getReg() == LFIStackReg;
}

static bool isMvSpX(const MCInst &I) {
  if (!isMV(I)) return false;
  return I.getOperand(0).isReg() && I.getOperand(0).getReg() == LFIStackReg;
}

// TLS patterns:
static bool isMvTpX(const MCInst &I) {
  if (!isMV(I)) return false;
  return I.getOperand(0).isReg() && I.getOperand(0).getReg() == LFIThreadReg;
}
static bool isMvXTp(const MCInst &I) {
  if (!isMV(I)) return false;
  return I.getOperand(1).isReg() && I.getOperand(1).getReg() == LFIThreadReg;
}
static bool isAddA0A0Tp(const MCInst &I) {
  return isADD(I) &&
         I.getOperand(0).isReg() && I.getOperand(0).getReg() == RISCV::X10 && // a0
         I.getOperand(1).isReg() && I.getOperand(1).getReg() == RISCV::X10 &&
         I.getOperand(2).isReg() && I.getOperand(2).getReg() == LFIThreadReg;
}


static inline void emit(MCStreamer &Out, const MCSubtargetInfo &STI,
                        unsigned Opc, ArrayRef<MCOperand> Ops) {
  MCInst T; T.setOpcode(Opc);
  for (const auto &Op : Ops) T.addOperand(Op);
  Out.emitInstruction(T, STI);
}


bool RISCV::RISCVMCLFIExpander::isValidScratchRegister(MCRegister /*Reg*/) const {
  return false;
}

void RISCV::RISCVMCLFIExpander::expandDirectCall(const MCInst &Inst,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  // jal rd,label
  bool WritesRA =
      Inst.getNumOperands() >= 1 && Inst.getOperand(0).isReg() &&
      Inst.getOperand(0).getReg() == LFIReturnReg;
  if (WritesRA) {
    Out.emitBundleLock(true);
    Out.emitInstruction(Inst, STI);
    Out.emitBundleUnlock();
  } else {
    Out.emitInstruction(Inst, STI);
  }
}

void RISCV::RISCVMCLFIExpander::emitSandboxBranchReg(MCRegister Reg,
                                                     MCStreamer &Out,
                                                     const MCSubtargetInfo &STI) {
  // add.uw s1, Reg, s11 ; andi s9, s1, ~7
  emit(Out, STI, RISCV::ADD_UW, { R(LFIAddrReg), R(Reg), R(LFIBaseReg) });
  emit(Out, STI, RISCV::ANDI,   { R(LFICtrlReg), R(LFIAddrReg), I64(BundleMaskImm) });
}

void RISCV::RISCVMCLFIExpander::emitIndirectJumpReg(MCRegister Reg,
                                                    MCStreamer &Out,
                                                    const MCSubtargetInfo &STI) {
  Out.emitBundleLock(false);
  emitSandboxBranchReg(Reg, Out, STI);
  emit(Out, STI, RISCV::JALR, { R(RISCV::X0), R(LFICtrlReg), I64(0) });
  Out.emitBundleUnlock();
}

void RISCV::RISCVMCLFIExpander::emitIndirectCallReg(MCRegister Reg,
                                                    MCStreamer &Out,
                                                    const MCSubtargetInfo &STI) {
  Out.emitBundleLock(true); 
  emitSandboxBranchReg(Reg, Out, STI);
  emit(Out, STI, RISCV::JALR, { R(LFIReturnReg), R(LFICtrlReg), I64(0) });
  Out.emitBundleUnlock();
  Out.emitCodeAlignment(Align(BundleSize), &STI);
}

static bool isValidReturnRegister(const MCRegister &Reg) {
  (void)Reg;
  return true;
}

void RISCV::RISCVMCLFIExpander::expandReturn(const MCInst &Inst,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI) {
  
  (void)Inst; (void)STI;
  Out.emitInstruction(Inst, STI);
}

// Expands memory ops: add.uw s1, base, s11 ; use I(s1)
void RISCV::RISCVMCLFIExpander::expandLoadStore(const MCInst &Inst,
                                                MCStreamer &Out,
                                                const MCSubtargetInfo &STI,
                                                bool EmitPrefixes) {
  (void)EmitPrefixes;
  MCInst S(Inst);


  MCOperand &Base = S.getOperand(1);
  if (Base.isReg() && !isAbsoluteReg(Base.getReg())) {
    emit(Out, STI, RISCV::ADD_UW, { R(LFIAddrReg), R(Base.getReg()), R(LFIBaseReg) });
    Base.setReg(LFIAddrReg);
    Out.emitInstruction(S, STI);
    
  } else {
    Out.emitInstruction(S, STI);
  }
}


void RISCV::RISCVMCLFIExpander::expandStringOperation(const MCInst &Inst,
                                                      MCStreamer &Out,
                                                      const MCSubtargetInfo &STI,
                                                      bool EmitPrefixes) {
  (void)EmitPrefixes;
  Out.emitInstruction(Inst, STI);
}

// Stack manipulation (sp) via s10 then add.uw sp, s10, s11
static void emitStackFixupFromTmp(MCStreamer &Out, const MCSubtargetInfo &STI) {
  emit(Out, STI, RISCV::ADD_UW, { R(LFIStackReg), R(LFITmpReg), R(LFIBaseReg) });
}

void RISCV::RISCVMCLFIExpander::expandExplicitStackManipulation(
    MCRegister StackReg, const MCInst &Inst, MCStreamer &Out,
    const MCSubtargetInfo &STI, bool EmitPrefixes) {
  (void)EmitPrefixes;
  assert(StackReg == LFIStackReg && "only sp is supported");

  if (isStackAddi(Inst)) {
    // addi s10, sp, imm ; add.uw sp, s10, s11
    emit(Out, STI, RISCV::ADDI,  { R(LFITmpReg),  R(LFIStackReg), Inst.getOperand(2) });
    emitStackFixupFromTmp(Out, STI);
    return;
  }

  if (isAddSpSpX(Inst)) {
    // add s10, sp, xN ; add.uw sp, s10, s11
    emit(Out, STI, RISCV::ADD,   { R(LFITmpReg),  R(LFIStackReg), Inst.getOperand(2) });
    emitStackFixupFromTmp(Out, STI);
    return;
  }

  if (isSubSpSpX(Inst)) {
    // sub s10, sp, xN ; add.uw sp, s10, s11
    emit(Out, STI, RISCV::SUB,   { R(LFITmpReg),  R(LFIStackReg), Inst.getOperand(2) });
    emitStackFixupFromTmp(Out, STI);
    return;
  }

  if (isMvSpX(Inst)) {
    // mv s10, xN ; add.uw sp, s10, s11
    MCRegister XN = Inst.getOperand(1).getReg();
    emit(Out, STI, RISCV::ADDI,   { R(LFITmpReg),   R(XN), I64 (0) });
    emitStackFixupFromTmp(Out, STI);
    return;
  }

  // Fallback
  Out.emitInstruction(Inst, STI);
}

//  ld ra,(sp)  -> ld s10,(sp) ; add.uw s1,s10,s11 ; andi ra,s1,~7
//  mv ra,xN    -> mv s10,xN  ; add.uw s1,s10,s11 ; andi ra,s1,~7
void RISCV::RISCVMCLFIExpander::expandRetAddrMods(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  if (isLdRaFromSp(Inst)) {
    emit(Out, STI, RISCV::LD,    { R(LFITmpReg), R(LFIStackReg), I64(0) });
    emit(Out, STI, RISCV::ADD_UW,{ R(LFIAddrReg), R(LFITmpReg),  R(LFIBaseReg) });
    emit(Out, STI, RISCV::ANDI,  { R(LFIReturnReg), R(LFIAddrReg), I64(BundleMaskImm) });
    return;
  }

  if (isMvRaX(Inst)) {
    MCRegister XN = Inst.getOperand(1).getReg();
    emit(Out, STI, RISCV::ADDI,   { R(LFITmpReg),   R(XN), I64(0) });
    emit(Out, STI, RISCV::ADD_UW,   { R(LFIAddrReg),  R(LFITmpReg), R(LFIBaseReg) });
    emit(Out, STI, RISCV::ANDI,     { R(LFIReturnReg), R(LFIAddrReg), I64(BundleMaskImm) });
    return;
  }

  Out.emitInstruction(Inst, STI);
}

// Syscalls:
//  ecall ->
//    mv   s10, ra
//    ld   ra, (s11)
//    jalr ra
//    add.uw s1, s10, s11
//    andi ra, s1, ~7
void RISCV::RISCVMCLFIExpander::expandSyscall(const MCInst &Inst,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
  (void)Inst;
  emit(Out, STI, RISCV::ADDI,     { R(LFITmpReg),   R(LFIReturnReg), I64(0) });
  emit(Out, STI, RISCV::LD,       { R(LFIReturnReg), R(LFIBaseReg), I64(0) });
  emit(Out, STI, RISCV::JALR,     { R(LFIReturnReg), R(LFIReturnReg), I64(0) });
  emit(Out, STI, RISCV::ADD_UW,   { R(LFIAddrReg),  R(LFITmpReg),   R(LFIBaseReg) });
  emit(Out, STI, RISCV::ANDI,     { R(LFIReturnReg), R(LFIAddrReg), I64(BundleMaskImm) });
}


void RISCV::RISCVMCLFIExpander::expandTLSShim(const MCInst &Inst,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
  if (isMvTpX(Inst)) {
    MCRegister XN = Inst.getOperand(1).getReg();
    emit(Out, STI, RISCV::ADDI,     { R(LFITmpReg), R(LFIReturnReg), I64(0) });     // mv s10,ra             // mv s10,ra
    emit(Out, STI, RISCV::XOR,      { R(RISCV::X10), R(RISCV::X10), R(XN) });        // xor a0,a0,xN
    emit(Out, STI, RISCV::XOR,      { R(XN),         R(RISCV::X10), R(XN) });        // xor xN,a0,xN
    emit(Out, STI, RISCV::XOR,      { R(RISCV::X10), R(RISCV::X10), R(XN) });        // xor a0,a0,xN
    emit(Out, STI, RISCV::LD,       { R(LFIReturnReg), R(LFIBaseReg), I64(16) });    // ld ra,16(s11)
    emit(Out, STI, RISCV::JALR,     { R(LFIReturnReg), R(LFIReturnReg), I64(0) });   // jalr ra
    emit(Out, STI, RISCV::XOR,      { R(RISCV::X10), R(RISCV::X10), R(XN) });
    emit(Out, STI, RISCV::XOR,      { R(XN),         R(RISCV::X10), R(XN) });
    emit(Out, STI, RISCV::XOR,      { R(RISCV::X10), R(RISCV::X10), R(XN) });
    emit(Out, STI, RISCV::ADD_UW,   { R(LFIAddrReg), R(LFITmpReg), R(LFIBaseReg) });
    emit(Out, STI, RISCV::ANDI,     { R(LFIReturnReg), R(LFIAddrReg), I64(BundleMaskImm) });
    return;
  }

  if (isMvXTp(Inst)) {
    MCRegister XN = Inst.getOperand(0).getReg();
    emit(Out, STI, RISCV::ADDI,     { R(XN),        R(RISCV::X10),  I64(0) });      // mv xN,a0
    emit(Out, STI, RISCV::ADDI,     { R(LFITmpReg), R(LFIReturnReg), I64(0) });     // mv s10,ra
    emit(Out, STI, RISCV::LD,       { R(LFIReturnReg), R(LFIBaseReg), I64(8) });    // ld ra,8(s11)
    emit(Out, STI, RISCV::JALR,     { R(LFIReturnReg), R(LFIReturnReg), I64(0) });  // jalr ra
    emit(Out, STI, RISCV::XOR,      { R(RISCV::X10), R(RISCV::X10), R(XN) });
    emit(Out, STI, RISCV::XOR,      { R(XN),         R(RISCV::X10), R(XN) });
    emit(Out, STI, RISCV::XOR,      { R(RISCV::X10), R(RISCV::X10), R(XN) });
    emit(Out, STI, RISCV::ADD_UW,   { R(LFIAddrReg), R(LFITmpReg), R(LFIBaseReg) });
    emit(Out, STI, RISCV::ANDI,     { R(LFIReturnReg), R(LFIAddrReg), I64(BundleMaskImm) });
    return;
  }

  if (isAddA0A0Tp(Inst)) {
    emit(Out, STI, RISCV::SD,       { R(RISCV::X11), R(LFIStackReg), I64(-8) });    // sd a1,-8(sp)
    emit(Out, STI, RISCV::ADDI,     { R(RISCV::X11), R(RISCV::X10),  I64(0) });     // mv a1,a0
    emit(Out, STI, RISCV::ADDI,     { R(LFITmpReg),  R(LFIReturnReg), I64(0) });    // mv s10,ra
    emit(Out, STI, RISCV::LD,       { R(LFIReturnReg), R(LFIBaseReg), I64(8) });    // ld ra,8(s11)
    emit(Out, STI, RISCV::JALR,     { R(LFIReturnReg), R(LFIReturnReg), I64(0) });  // jalr ra
    emit(Out, STI, RISCV::ADD_UW,   { R(LFIAddrReg), R(LFITmpReg), R(LFIBaseReg) });
    emit(Out, STI, RISCV::ANDI,     { R(LFIReturnReg), R(LFIAddrReg), I64(BundleMaskImm) });
    emit(Out, STI, RISCV::ADD,      { R(RISCV::X10), R(RISCV::X11), R(RISCV::X10) });// add a0,a1,a0
    emit(Out, STI, RISCV::LD,       { R(RISCV::X11), R(LFIStackReg), I64(-8) });    // ld a1,-8(sp)
    return;
  }

  Out.emitInstruction(Inst, STI);
}

void RISCV::RISCVMCLFIExpander::emitInstruction(const MCInst &Inst,
                                                MCStreamer &Out,
                                                const MCSubtargetInfo &STI,
                                                bool EmitPrefixes) {
  (void)EmitPrefixes;
  if (!expandInst(Inst, Out, STI)) {
   
    doExpandInst(Inst, Out, STI, false);
  }
}

static MCInst replaceReg(const MCInst &Inst, MCRegister Dest, MCRegister Src) {
  MCInst New;
  New.setOpcode(Inst.getOpcode());
  New.setLoc(Inst.getLoc());
  for (unsigned I = 0; I < Inst.getNumOperands(); ++I) {
    const MCOperand &Op = Inst.getOperand(I);
    if (Op.isReg() && Op.getReg() == Src)
      New.addOperand(MCOperand::createReg(Dest));
    else
      New.addOperand(Op);
  }
  return New;
}

void RISCV::RISCVMCLFIExpander::doExpandInst(const MCInst &Inst, MCStreamer &Out,
                                             const MCSubtargetInfo &STI,
                                             bool EmitPrefixes) {
  (void)EmitPrefixes;

  // Reserved base s11 protection
  if (explicitlyModifiesRegister(Inst, LFIBaseReg)) {
    if (RISCVLFIErrorReserved)
      return Out.getContext().reportError(
          Inst.getLoc(), "illegal modification of reserved LFI register (s11)");
    Out.getContext().reportWarning(
        Inst.getLoc(), "deleting modification of reserved LFI register (s11)");
    return;
  }

  // Syscall
  if (isSyscall(Inst)) return expandSyscall(Inst, Out, STI);

  if (isMvTpX(Inst) || isMvXTp(Inst) || isAddA0A0Tp(Inst))
    return expandTLSShim(Inst, Out, STI);

  if (isLdRaFromSp(Inst) || isMvRaX(Inst))
    return expandRetAddrMods(Inst, Out, STI);

  // Direct calls (jal / pseudo call)
  if (isJAL(Inst)) return expandDirectCall(Inst, Out, STI);

  // Indirect branches/calls (jalr)
  if (isRVIndirectBranch(Inst) || isCall(Inst))
    return expandIndirectBranch(Inst, Out, STI);

  // Return
  if (isReturn(Inst)) return expandReturn(Inst, Out, STI);

  // Stack manipulation (sp)
  if (isStackAddi(Inst) || isAddSpSpX(Inst) || isSubSpSpX(Inst) || isMvSpX(Inst))
    return expandExplicitStackManipulation(LFIStackReg, Inst, Out, STI, false);

  // Loads / stores
  if (isLoadStore(Inst))
    return expandLoadStore(Inst, Out, STI, false);

  // Default passthrough
  Out.emitInstruction(Inst, STI);
}

bool RISCV::RISCVMCLFIExpander::expandInst(const MCInst &Inst,
                                           MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {

    if (Guard)
      return false;
    Guard = true;
    doExpandInst(Inst, Out, STI, true);

    Guard = false;
    return true;                                      
}



void RISCV::RISCVMCLFIExpander::expandIndirectBranch(const MCInst &Inst,
                                           MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {


  // if (Inst.getOpcode() == RISCV::JALR &&
  //     Inst.getNumOperands() >= 3 &&
  //     Inst.getOperand(0).isReg() &&
  //     Inst.getOperand(1).isReg() &&
  //     Inst.getOperand(2).isImm()) {

    printf("expandindirect\n");

    const unsigned Rd  = Inst.getOperand(0).getReg();
    const unsigned Rs1 = Inst.getOperand(1).getReg();
    const int64_t  Imm = Inst.getOperand(2).getImm();
   
    // add.uw s1, xM, s11
    {
      MCInst M; M.setOpcode(RISCV::ADD_UW);
      M.addOperand(R(LFIAddrReg));
      M.addOperand(R(Rs1));
      M.addOperand(R(LFIBaseReg));
      Out.emitInstruction(M, STI);
    }
    // andi s9, s1, -8
    {
      MCInst M; M.setOpcode(RISCV::ANDI);
      M.addOperand(R(LFICtrlReg));
      M.addOperand(R(LFIAddrReg));
      M.addOperand(I64(-8));
      Out.emitInstruction(M, STI);
    }
    // jalr rd, s9, imm   (prints as jalr rd, imm(s9))
    {
       if (Rd == RISCV::X1){
        Out.emitBundleLock(true);
      }
      MCInst M; M.setOpcode(RISCV::JALR);
      M.addOperand(R(Rd));
      M.addOperand(R(LFICtrlReg));
      M.addOperand(I64(Imm));
      Out.emitInstruction(M, STI);
    }

    if (Rd == RISCV::X1){
      Out.emitBundleUnlock();
    }


  //}

}