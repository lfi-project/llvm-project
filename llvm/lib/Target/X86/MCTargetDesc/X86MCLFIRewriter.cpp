//===- X86MCLFIRewriter.cpp -------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the X86MCLFIRewriter class, which rewrites X86-64
// instructions for LFI (Lightweight Fault Isolation) sandboxing.
//
//===----------------------------------------------------------------------===//

#include "X86MCLFIRewriter.h"
#include "X86BaseInfo.h"
#include "X86MCTargetDesc.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCLFI.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "x86-lfi-rewriter"

static const int BundleSize = 32;

static const MCRegister LFIBaseReg = X86::R14;
static const MCRegister LFIScratchReg = X86::R11;
static const MCRegister LFIBaseSeg = X86::GS;

// Byte offset into the virtual register file (pointed to by R15) where the
// thread pointer is stored.
static const int TPOffset = 32;

// Shadow call stack pointer offset in the virtual register file (R15).
static const int SCSOffset = 16;

// Temporary save slot offset in the virtual register file (R15).
static const int SCSTempOffset = 24;


static cl::opt<bool>
    X86LFIJumpsOnly("x86-lfi-jumps-only",
                     cl::desc("Only rewrite control flow and syscalls, skip "
                              "memory sandboxing (equivalent to "
                              "+no-lfi-loads,+no-lfi-stores)"),
                     cl::init(false));

static cl::opt<bool>
    X86LFIHwShstk("x86-lfi-hw-shstk",
                   cl::desc("Hardware shadow call stack is available; skip "
                            "software SCS rewrites"),
                   cl::init(false));

static cl::opt<bool>
    X86LFIHwEndbr("x86-lfi-hw-endbr",
                   cl::desc("Hardware endbr CFI is available; skip software "
                            "endbr comparison checks at indirect branches"),
                   cl::init(false));

// The expected encoding of endbr64 (f3 0f 1e fa) as a 32-bit LE value.
static const int32_t ENDBR64Encoding = static_cast<int32_t>(0xfa1e0ff3);

// Forward declarations for helper functions.
static MCRegister getReg64(MCRegister Reg);
static MCRegister getReg32(MCRegister Reg);
static bool isAbsoluteReg(MCRegister Reg);
static bool isPrefix(const MCInst &Inst);
static bool isDirectCall(const MCInst &Inst);
static bool isStringOperation(const MCInst &Inst);
static bool isSyscall(const MCInst &Inst);
static bool isTLSRead(const MCInst &Inst);
static MCRegister xchgStackReg(const MCInst &Inst);
static unsigned demoteOpcode(unsigned Opcode);
static void demoteInst(MCInst &Inst, const MCInstrInfo &InstInfo);
static void emitStackFixup(MCRegister StackReg, MCStreamer &Out,
                           const MCSubtargetInfo &STI);
static void clearHighBits(const MCOperand &Reg, MCStreamer &Out,
                          const MCSubtargetInfo &STI);
static void fixupStringOpReg(const MCOperand &Op, MCStreamer &Out,
                             const MCSubtargetInfo &STI);
static bool willEmitSandboxInsts(const MCInst &Inst, int Idx);

//===----------------------------------------------------------------------===//
// Feature checking helpers
//===----------------------------------------------------------------------===//

static bool hasFeature(const FeatureBitset &Feature,
                       const MCSubtargetInfo &STI) {
  return (STI.getFeatureBits() & Feature) == Feature;
}

bool X86::X86MCLFIRewriter::hasSegue(const MCSubtargetInfo &STI) const {
  return !hasFeature(FeatureBitset({X86::FeatureNoLFISegue}), STI);
}

bool X86::X86MCLFIRewriter::hasNoLFILoads(const MCSubtargetInfo &STI) const {
  return X86LFIJumpsOnly ||
         hasFeature(FeatureBitset({X86::FeatureNoLFILoads}), STI);
}

bool X86::X86MCLFIRewriter::hasNoLFIStores(const MCSubtargetInfo &STI) const {
  return X86LFIJumpsOnly ||
         hasFeature(FeatureBitset({X86::FeatureNoLFIStores}), STI);
}

//===----------------------------------------------------------------------===//
// Register conversion helpers
//===----------------------------------------------------------------------===//

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

static bool isAbsoluteReg(MCRegister Reg) {
  Reg = getReg64(Reg);
  return (Reg == LFIBaseReg || Reg == X86::RSP || Reg == X86::RIP);
}

//===----------------------------------------------------------------------===//
// Instruction classification helpers
//===----------------------------------------------------------------------===//

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

static bool isStringOperation(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case X86::CMPSB:
  case X86::CMPSW:
  case X86::CMPSL:
  case X86::CMPSQ:
  case X86::MOVSB:
  case X86::MOVSW:
  case X86::MOVSL:
  case X86::MOVSQ:
  case X86::STOSB:
  case X86::STOSW:
  case X86::STOSL:
  case X86::STOSQ:
    return true;
  default:
    return false;
  }
}

static bool isSyscall(const MCInst &Inst) {
  return Inst.getOpcode() == X86::SYSCALL;
}

static bool isTLSRead(const MCInst &Inst) {
  return Inst.getOpcode() == X86::MOV64rm &&
         Inst.getOperand(1).getReg() == X86::NoRegister &&
         Inst.getOperand(2).isImm() && Inst.getOperand(2).getImm() == 1 &&
         Inst.getOperand(3).getReg() == X86::NoRegister &&
         Inst.getOperand(4).isImm() && Inst.getOperand(4).getImm() == 0 &&
         Inst.getOperand(5).getReg() == X86::FS;
}

static MCRegister xchgStackReg(const MCInst &Inst) {
  MCRegister Reg1 = X86::NoRegister, Reg2 = X86::NoRegister;
  switch (Inst.getOpcode()) {
  case X86::XCHG64ar:
  case X86::XCHG64rm:
    Reg1 = Inst.getOperand(0).getReg();
    break;
  case X86::XCHG64rr:
    Reg1 = Inst.getOperand(0).getReg();
    Reg2 = Inst.getOperand(2).getReg();
    break;
  default:
    return X86::NoRegister;
  }
  if (Reg1 == X86::RSP)
    return Reg1;
  if (Reg2 == X86::RSP)
    return Reg2;
  return X86::NoRegister;
}

static bool isHighReg(MCRegister Reg) {
  return Reg == X86::AH || Reg == X86::BH || Reg == X86::CH || Reg == X86::DH;
}

//===----------------------------------------------------------------------===//
// Instruction emission helpers
//===----------------------------------------------------------------------===//

/// Emit: movq %SrcReg, Offset(%r15)
static void emitMovToR15Slot(MCRegister SrcReg, int Offset, MCStreamer &Out,
                              const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(X86::MOV64mr);
  Mov.addOperand(MCOperand::createReg(X86::R15));
  Mov.addOperand(MCOperand::createImm(1));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Mov.addOperand(MCOperand::createImm(Offset));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Mov.addOperand(MCOperand::createReg(SrcReg));
  Out.emitInstruction(Mov, STI);
}

/// Emit: movq Offset(%r15), %DstReg
static void emitMovFromR15Slot(MCRegister DstReg, int Offset, MCStreamer &Out,
                                const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(X86::MOV64rm);
  Mov.addOperand(MCOperand::createReg(DstReg));
  Mov.addOperand(MCOperand::createReg(X86::R15));
  Mov.addOperand(MCOperand::createImm(1));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Mov.addOperand(MCOperand::createImm(Offset));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Mov, STI);
}

static void maybeEmitBundleLock(bool AlignToEnd, MCStreamer &Out,
                                 const MCSubtargetInfo &STI) {
  if (FlagX86LFIBundling)
    Out.emitBundleLock(AlignToEnd, STI);
}

static void maybeEmitBundleUnlock(MCStreamer &Out,
                                   const MCSubtargetInfo &STI) {
  if (FlagX86LFIBundling)
    Out.emitBundleUnlock(STI);
}

void X86::X86MCLFIRewriter::emitInstruction(const MCInst &Inst, MCStreamer &Out,
                                            const MCSubtargetInfo &STI,
                                            bool EmitPrefixes) {
  if (EmitPrefixes) {
    for (const MCInst &Prefix : Prefixes)
      Out.emitInstruction(Prefix, STI);
    Prefixes.clear();
  }
  Out.emitInstruction(Inst, STI);
}

//===----------------------------------------------------------------------===//
// Control flow rewriting
//===----------------------------------------------------------------------===//

MCSymbol *
X86::X86MCLFIRewriter::getOrEmitTrapSymbol(MCStreamer &Out,
                                             const MCSubtargetInfo &STI) {
  if (LFITrapSymbol)
    return LFITrapSymbol;

  LFITrapSymbol = Out.getContext().getOrCreateSymbol("_lfi_trap");

  Out.pushSection();
  MCSection *TrapSec = Out.getContext().getELFSection(
      ".text_lfi_trap", ELF::SHT_PROGBITS,
      ELF::SHF_ALLOC | ELF::SHF_EXECINSTR | ELF::SHF_GROUP,
      0, "_lfi_trap", /*IsComdat=*/true);
  Out.switchSection(TrapSec);
  Out.emitSymbolAttribute(LFITrapSymbol, MCSA_Weak);
  Out.emitLabel(LFITrapSymbol);
  // ud2
  Out.emitBytes(StringRef("\x0f\x0b", 2));
  Out.popSection();

  return LFITrapSymbol;
}

void X86::X86MCLFIRewriter::emitCFICheck(MCRegister Reg, MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {
  MCSymbol *TrapSym = getOrEmitTrapSymbol(Out, STI);

  // cmpl $0xfa1e0ff3, (%r14, %rX)
  MCInst Cmp;
  Cmp.setOpcode(X86::CMP32mi);
  Cmp.addOperand(MCOperand::createReg(LFIBaseReg));        // Base = %r14
  Cmp.addOperand(MCOperand::createImm(1));                  // Scale = 1
  Cmp.addOperand(MCOperand::createReg(getReg64(Reg)));     // Index = %rX
  Cmp.addOperand(MCOperand::createImm(0));                  // Disp = 0
  Cmp.addOperand(MCOperand::createReg(X86::NoRegister));   // Segment
  Cmp.addOperand(MCOperand::createImm(ENDBR64Encoding));   // Immediate
  Out.emitInstruction(Cmp, STI);

  // jne _lfi_trap
  MCInst Jne;
  Jne.setOpcode(X86::JCC_1);
  Jne.addOperand(MCOperand::createExpr(
      MCSymbolRefExpr::create(TrapSym, Out.getContext())));
  Jne.addOperand(MCOperand::createImm(X86::COND_NE));
  Out.emitInstruction(Jne, STI);
}

void X86::X86MCLFIRewriter::emitSandboxBranchReg(MCRegister Reg,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI,
                                                  bool CheckCFI) {
  // andl $-32, %eX
  MCInst AndInst;
  AndInst.setOpcode(X86::AND32ri8);
  MCOperand Target32 = MCOperand::createReg(getReg32(Reg));
  AndInst.addOperand(Target32);
  AndInst.addOperand(Target32);
  AndInst.addOperand(MCOperand::createImm(-BundleSize));
  Out.emitInstruction(AndInst, STI);

  // Forward-edge CFI: check for endbr64 at the aligned target.
  if (!FlagX86LFIBundling && !X86LFIHwEndbr && CheckCFI)
    emitCFICheck(Reg, Out, STI);

  // addq %r14, %rX
  MCInst Add;
  Add.setOpcode(X86::ADD64rr);
  MCOperand Target64 = MCOperand::createReg(getReg64(Reg));
  Add.addOperand(Target64);
  Add.addOperand(Target64);
  Add.addOperand(MCOperand::createReg(LFIBaseReg));
  Out.emitInstruction(Add, STI);
}

void X86::X86MCLFIRewriter::emitIndirectJumpReg(MCRegister Reg, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI,
                                                 bool CheckCFI) {
  maybeEmitBundleLock(false, Out, STI);
  emitSandboxBranchReg(Reg, Out, STI, CheckCFI);

  MCInst Jmp;
  Jmp.setOpcode(X86::JMP64r);
  Jmp.addOperand(MCOperand::createReg(getReg64(Reg)));
  Out.emitInstruction(Jmp, STI);

  maybeEmitBundleUnlock(Out, STI);
}

void X86::X86MCLFIRewriter::emitIndirectCallReg(MCRegister Reg, MCStreamer &Out,
                                                 const MCSubtargetInfo &STI,
                                                 bool CheckCFI) {
  maybeEmitBundleLock(true, Out, STI);
  emitSandboxBranchReg(Reg, Out, STI, CheckCFI);

  MCInst Call;
  Call.setOpcode(X86::CALL64r);
  Call.addOperand(MCOperand::createReg(getReg64(Reg)));
  Out.emitInstruction(Call, STI);

  maybeEmitBundleUnlock(Out, STI);
}

MCSymbol *
X86::X86MCLFIRewriter::emitShadowCallPrologue(MCStreamer &Out,
                                                const MCSubtargetInfo &STI) {
  MCSymbol *RetLabel = Out.getContext().createTempSymbol();

  // movq %rsp, SCSTempOffset(%r15) - save rsp to temp slot
  emitMovToR15Slot(X86::RSP, SCSTempOffset, Out, STI);

  // movq SCSOffset(%r15), %rsp - load shadow call stack pointer
  emitMovFromR15Slot(X86::RSP, SCSOffset, Out, STI);

  // leaq RetLabel(%rip), %r11 - compute return address
  MCInst Lea;
  Lea.setOpcode(X86::LEA64r);
  Lea.addOperand(MCOperand::createReg(LFIScratchReg));
  Lea.addOperand(MCOperand::createReg(X86::RIP));
  Lea.addOperand(MCOperand::createImm(1));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Lea.addOperand(MCOperand::createExpr(
      MCSymbolRefExpr::create(RetLabel, Out.getContext())));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Lea, STI);

  // pushq %r11 - push return address onto shadow call stack
  MCInst Push;
  Push.setOpcode(X86::PUSH64r);
  Push.addOperand(MCOperand::createReg(LFIScratchReg));
  Out.emitInstruction(Push, STI);

  // movq %rsp, SCSOffset(%r15) - save updated shadow call stack pointer
  emitMovToR15Slot(X86::RSP, SCSOffset, Out, STI);

  // movq SCSTempOffset(%r15), %rsp - restore real rsp
  emitMovFromR15Slot(X86::RSP, SCSTempOffset, Out, STI);

  return RetLabel;
}

void X86::X86MCLFIRewriter::emitShadowCallEpilogue(MCSymbol *RetLabel,
                                                     MCStreamer &Out,
                                                     const MCSubtargetInfo &STI,
                                                     bool ReturnsTwice) {
  // For returns_twice calls (e.g. setjmp), align RetLabel to 32 bytes and
  // insert ENDBR64 so that longjmp can reach it via an indirect jump that
  // passes the forward-edge CFI check.
  if (ReturnsTwice)
    Out.emitCodeAlignment(llvm::Align(32), &STI);

  // RetLabel:
  Out.emitLabel(RetLabel);

  if (ReturnsTwice) {
    MCInst Endbr;
    Endbr.setOpcode(X86::ENDBR64);
    Out.emitInstruction(Endbr, STI);
  }

  // movq %rsp, SCSOffset(%r15) - save shadow stack ptr (updated by callee's ret)
  emitMovToR15Slot(X86::RSP, SCSOffset, Out, STI);

  // movq SCSTempOffset(%r15), %rsp - restore real rsp (saved by callee's ret)
  emitMovFromR15Slot(X86::RSP, SCSTempOffset, Out, STI);

  // popq %r11 - pop the return address that callq pushed on the real stack
  MCInst Pop;
  Pop.setOpcode(X86::POP64r);
  Pop.addOperand(MCOperand::createReg(LFIScratchReg));
  Out.emitInstruction(Pop, STI);
}

void X86::X86MCLFIRewriter::expandDirectCall(const MCInst &Inst,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
  MCSymbol *RetLabel = nullptr;
  bool ReturnsTwice = Inst.getFlags() & X86::IP_LFI_RETURNS_TWICE;
  if (!FlagX86LFIBundling && !X86LFIHwShstk)
    RetLabel = emitShadowCallPrologue(Out, STI);

  maybeEmitBundleLock(true, Out, STI);
  Out.emitInstruction(Inst, STI);
  maybeEmitBundleUnlock(Out, STI);

  if (!FlagX86LFIBundling && !X86LFIHwShstk)
    emitShadowCallEpilogue(RetLabel, Out, STI, ReturnsTwice);
}

void X86::X86MCLFIRewriter::expandIndirectBranch(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  // Emit shadow call stack prologue before loading the target, since the
  // prologue uses and frees r11 before the target load may need it.
  MCSymbol *RetLabel = nullptr;
  if (!FlagX86LFIBundling && !X86LFIHwShstk && isCall(Inst))
    RetLabel = emitShadowCallPrologue(Out, STI);

  MCRegister Target;
  if (mayLoad(Inst)) {
    // Indirect jmp/call through memory - load address first.
    MCInst Mov;
    Mov.setOpcode(X86::MOV64rm);
    Target = LFIScratchReg;

    Mov.addOperand(MCOperand::createReg(getReg64(Target)));
    Mov.addOperand(Inst.getOperand(0));
    Mov.addOperand(Inst.getOperand(1));
    Mov.addOperand(Inst.getOperand(2));
    Mov.addOperand(Inst.getOperand(3));
    Mov.addOperand(Inst.getOperand(4));
    doRewriteInst(Mov, Out, STI, false);
  } else {
    Target = Inst.getOperand(0).getReg();
  }

  if (isCall(Inst))
    emitIndirectCallReg(Target, Out, STI);
  else
    emitIndirectJumpReg(Target, Out, STI);

  if (!FlagX86LFIBundling && !X86LFIHwShstk && isCall(Inst))
    emitShadowCallEpilogue(RetLabel, Out, STI);
}

void X86::X86MCLFIRewriter::expandReturn(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
  if (!FlagX86LFIBundling) {
    if (X86LFIHwShstk) {
      // Hardware shadow stack handles backward-edge CFI; emit ret as-is.
      Out.emitInstruction(Inst, STI);
      return;
    }

    // Handle ret with immediate - adjust rsp before saving to temp.
    if (Inst.getNumOperands() > 0) {
      if (Inst.getOpcode() == X86::RETI32 || Inst.getOpcode() == X86::RETI64) {
        MCInst Add;
        Add.setOpcode(X86::ADD64ri32);
        MCOperand StackPointer = MCOperand::createReg(X86::RSP);
        Add.addOperand(StackPointer);
        Add.addOperand(StackPointer);
        Add.addOperand(Inst.getOperand(0));
        doRewriteInst(Add, Out, STI, false);
      }
    }

    // movq %rsp, SCSTempOffset(%r15) - save rsp to temp slot
    emitMovToR15Slot(X86::RSP, SCSTempOffset, Out, STI);

    // movq SCSOffset(%r15), %rsp - load shadow call stack pointer
    emitMovFromR15Slot(X86::RSP, SCSOffset, Out, STI);

    // ret - return via shadow call stack
    MCInst Ret;
    Ret.setOpcode(X86::RET64);
    Out.emitInstruction(Ret, STI);
    return;
  }

  // pop %r11
  MCInst Pop;
  Pop.setOpcode(X86::POP64r);
  Pop.addOperand(MCOperand::createReg(LFIScratchReg));
  Out.emitInstruction(Pop, STI);

  // Handle ret with immediate (pop additional bytes from stack).
  if (Inst.getNumOperands() > 0) {
    if (Inst.getOpcode() == X86::RETI32 || Inst.getOpcode() == X86::RETI64) {
      MCInst Add;
      Add.setOpcode(X86::ADD64ri32);
      MCOperand StackPointer = MCOperand::createReg(X86::RSP);
      Add.addOperand(StackPointer);
      Add.addOperand(StackPointer);
      Add.addOperand(Inst.getOperand(0));
      doRewriteInst(Add, Out, STI, false);
    }
  }

  emitIndirectJumpReg(LFIScratchReg, Out, STI, /*CheckCFI=*/false);
}

//===----------------------------------------------------------------------===//
// Syscall and TLS rewriting
//===----------------------------------------------------------------------===//

void X86::X86MCLFIRewriter::emitLFICall(LFICallType CallType, MCStreamer &Out,
                                         const MCSubtargetInfo &STI) {
  maybeEmitBundleLock(false, Out, STI);

  MCSymbol *Symbol = Out.getContext().createTempSymbol();

  // leaq .Ltmp(%rip), %r11
  MCInst Lea;
  Lea.setOpcode(X86::LEA64r);
  Lea.addOperand(MCOperand::createReg(LFIScratchReg));
  Lea.addOperand(MCOperand::createReg(X86::RIP));
  Lea.addOperand(MCOperand::createImm(1));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Lea.addOperand(
      MCOperand::createExpr(MCSymbolRefExpr::create(Symbol, Out.getContext())));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Lea, STI);

  unsigned Offset;
  switch (CallType) {
  case LFISyscall:
    Offset = 0;
    break;
  case LFITLSRead:
    Offset = 8;
    break;
  case LFITLSWrite:
    Offset = 16;
    break;
  }

  // jmpq *Offset(%r14)
  MCInst Jmp;
  Jmp.setOpcode(X86::JMP64m);
  Jmp.addOperand(MCOperand::createReg(LFIBaseReg));
  Jmp.addOperand(MCOperand::createImm(1));
  Jmp.addOperand(MCOperand::createReg(X86::NoRegister));
  Jmp.addOperand(MCOperand::createImm(Offset));
  Jmp.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Jmp, STI);

  Out.emitLabel(Symbol);
  maybeEmitBundleUnlock(Out, STI);
}

void X86::X86MCLFIRewriter::expandSyscall(const MCInst &Inst, MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {
  emitLFICall(LFISyscall, Out, STI);
}

void X86::X86MCLFIRewriter::expandTLSRead(const MCInst &Inst, MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {
  // Rewrite: movq %fs:0, %rX  ->  movq TP_OFFSET(%r15), %rX
  // R15 points to a virtual register file with the thread pointer at TPOffset.
  MCRegister DestReg = Inst.getOperand(0).getReg();

  MCInst Mov;
  Mov.setOpcode(X86::MOV64rm);
  Mov.addOperand(MCOperand::createReg(DestReg));
  Mov.addOperand(MCOperand::createReg(X86::R15));    // Base
  Mov.addOperand(MCOperand::createImm(1));           // Scale
  Mov.addOperand(MCOperand::createReg(X86::NoRegister)); // Index
  Mov.addOperand(MCOperand::createImm(TPOffset));    // Displacement
  Mov.addOperand(MCOperand::createReg(X86::NoRegister)); // Segment
  Out.emitInstruction(Mov, STI);
}

//===----------------------------------------------------------------------===//
// String operation rewriting
//===----------------------------------------------------------------------===//

static void clearHighBits(const MCOperand &Reg, MCStreamer &Out,
                          const MCSubtargetInfo &STI) {
  // movl %eX, %eX
  MCInst Mov;
  Mov.setOpcode(X86::MOV32rr);
  MCOperand Op = MCOperand::createReg(getReg32(Reg.getReg()));
  Mov.addOperand(Op);
  Mov.addOperand(Op);
  Out.emitInstruction(Mov, STI);
}

static void fixupStringOpReg(const MCOperand &Op, MCStreamer &Out,
                             const MCSubtargetInfo &STI) {
  clearHighBits(Op, Out, STI);

  // leaq (%r14, %rX), %rX
  MCInst Lea;
  Lea.setOpcode(X86::LEA64r);
  Lea.addOperand(MCOperand::createReg(getReg64(Op.getReg())));
  Lea.addOperand(MCOperand::createReg(LFIBaseReg));
  Lea.addOperand(MCOperand::createImm(1));
  Lea.addOperand(MCOperand::createReg(getReg64(Op.getReg())));
  Lea.addOperand(MCOperand::createImm(0));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Lea, STI);
}

void X86::X86MCLFIRewriter::expandStringOperation(const MCInst &Inst,
                                                   MCStreamer &Out,
                                                   const MCSubtargetInfo &STI,
                                                   bool EmitPrefixes) {
  bool JumpsOnly = hasNoLFILoads(STI) && hasNoLFIStores(STI);
  bool StoresOnly = hasNoLFILoads(STI) && !hasNoLFIStores(STI);

  maybeEmitBundleLock(false, Out, STI);

  switch (Inst.getOpcode()) {
  case X86::CMPSB:
  case X86::CMPSW:
  case X86::CMPSL:
  case X86::CMPSQ:
  case X86::MOVSB:
  case X86::MOVSW:
  case X86::MOVSL:
  case X86::MOVSQ:
    // Source operand (RSI).
    if (!JumpsOnly && !StoresOnly)
      fixupStringOpReg(Inst.getOperand(1), Out, STI);
    // Destination operand (RDI).
    if (!JumpsOnly)
      fixupStringOpReg(Inst.getOperand(0), Out, STI);
    break;
  case X86::STOSB:
  case X86::STOSW:
  case X86::STOSL:
  case X86::STOSQ:
    // Destination operand (RDI).
    if (!JumpsOnly)
      fixupStringOpReg(Inst.getOperand(0), Out, STI);
    break;
  }

  emitInstruction(Inst, Out, STI, EmitPrefixes);
  maybeEmitBundleUnlock(Out, STI);
}

//===----------------------------------------------------------------------===//
// Stack modification rewriting
//===----------------------------------------------------------------------===//

static void emitStackFixup(MCRegister StackReg, MCStreamer &Out,
                           const MCSubtargetInfo &STI) {
  // leaq (%rsp, %r14), %rsp
  MCInst Lea;
  Lea.setOpcode(X86::LEA64r);
  Lea.addOperand(MCOperand::createReg(StackReg));
  Lea.addOperand(MCOperand::createReg(StackReg));
  Lea.addOperand(MCOperand::createImm(1));
  Lea.addOperand(MCOperand::createReg(LFIBaseReg));
  Lea.addOperand(MCOperand::createImm(0));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Lea, STI);
}

void X86::X86MCLFIRewriter::expandStackModification(MCRegister StackReg,
                                                     const MCInst &Inst,
                                                     MCStreamer &Out,
                                                     const MCSubtargetInfo &STI,
                                                     bool EmitPrefixes) {
  bool JumpsOnly = hasNoLFILoads(STI) && hasNoLFIStores(STI);
  if (JumpsOnly)
    return emitInstruction(Inst, Out, STI, EmitPrefixes);

  if (Inst.getOpcode() == X86::POP64r) {
    // Transform pop %rsp into:
    // pop %r11
    // .bundle_lock
    // movl %r11d, %esp
    // leaq (%rsp,%r14), %rsp
    // .bundle_unlock
    MCInst PopR11;
    PopR11.setOpcode(X86::POP64r);
    PopR11.addOperand(MCOperand::createReg(LFIScratchReg));
    Out.emitInstruction(PopR11, STI);

    maybeEmitBundleLock(false, Out, STI);

    MCInst MovR11ToESP;
    MovR11ToESP.setOpcode(X86::MOV32rr);
    MovR11ToESP.addOperand(MCOperand::createReg(getReg32(StackReg)));
    MovR11ToESP.addOperand(MCOperand::createReg(X86::R11D));
    Out.emitInstruction(MovR11ToESP, STI);

    emitStackFixup(StackReg, Out, STI);

    maybeEmitBundleUnlock(Out, STI);
    return;
  }

  // For other stack modifications, demote to 32-bit and add fixup.
  MCInst SandboxedInst(Inst);
  demoteInst(SandboxedInst, *InstInfo);

  bool MemSandboxed =
      emitSandboxMemOps(SandboxedInst, X86::R11D, Out, STI, true);

  maybeEmitBundleLock(false, Out, STI);

  emitInstruction(SandboxedInst, Out, STI, EmitPrefixes);
  if (MemSandboxed)
    maybeEmitBundleUnlock(Out, STI);
  emitStackFixup(StackReg, Out, STI);

  maybeEmitBundleUnlock(Out, STI);
}

//===----------------------------------------------------------------------===//
// Memory sandboxing
//===----------------------------------------------------------------------===//

void X86::X86MCLFIRewriter::prepareSandboxMemOp(MCInst &Inst, int MemIdx,
                                                 MCRegister ScratchReg,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  MCOperand &Base = Inst.getOperand(MemIdx);
  MCOperand &Scale = Inst.getOperand(MemIdx + 1);
  MCOperand &Index = Inst.getOperand(MemIdx + 2);
  MCOperand &Segment = Inst.getOperand(MemIdx + 4);

  // Handle %fs segment for TLS accesses.
  // Load thread pointer from TP_OFFSET(%r15) into scratch, then use normal addressing.
  if (Segment.getReg() == X86::FS) {
    // Load thread pointer: movq TP_OFFSET(%r15), %r11
    MCInst LoadTP;
    LoadTP.setOpcode(X86::MOV64rm);
    LoadTP.addOperand(MCOperand::createReg(LFIScratchReg));
    LoadTP.addOperand(MCOperand::createReg(X86::R15));       // Base
    LoadTP.addOperand(MCOperand::createImm(1));              // Scale
    LoadTP.addOperand(MCOperand::createReg(X86::NoRegister)); // Index
    LoadTP.addOperand(MCOperand::createImm(TPOffset));       // Displacement
    LoadTP.addOperand(MCOperand::createReg(X86::NoRegister)); // Segment
    Out.emitInstruction(LoadTP, STI);

    // Now transform the addressing mode to use %r11 as base.
    // %fs:imm -> imm(%r11)
    // %fs:imm(reg) -> imm(%r11, reg)
    // %fs:(reg) -> (%r11, reg)
    if (Base.getReg() == 0 && Index.getReg() == 0) {
      // %fs:imm -> imm(%r11)
      Base.setReg(LFIScratchReg);
    } else if (Base.getReg() == 0 && Index.getReg() != 0) {
      // %fs:imm(,reg,scale) -> imm(%r11, reg, scale)
      Base.setReg(LFIScratchReg);
    } else if (Base.getReg() != 0 && Index.getReg() == 0) {
      // %fs:imm(reg) -> imm(%r11, reg) with scale 1
      Index.setReg(Base.getReg());
      Scale.setImm(1);
      Base.setReg(LFIScratchReg);
    } else {
      // Both base and index used - need to compute effective address first.
      // LEA to compute base + index*scale + offset into scratch, then use that.
      error(Inst, "complex %fs addressing mode not supported");
      return;
    }
    Segment.setReg(0);
  }
}

void X86::X86MCLFIRewriter::emitSandboxMemOp(MCInst &Inst, int MemIdx,
                                              MCRegister ScratchReg,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
  bool JumpsOnly = hasNoLFILoads(STI) && hasNoLFIStores(STI);
  bool StoresOnly = hasNoLFILoads(STI) && !hasNoLFIStores(STI);

  if (JumpsOnly)
    return;
  if (StoresOnly && !mayStore(Inst))
    return;

  MCOperand &Base = Inst.getOperand(MemIdx);
  MCOperand &Scale = Inst.getOperand(MemIdx + 1);
  MCOperand &Index = Inst.getOperand(MemIdx + 2);
  MCOperand &Offset = Inst.getOperand(MemIdx + 3);
  MCOperand &Segment = Inst.getOperand(MemIdx + 4);

  // LEA and similar instructions have memory operands but don't actually
  // access memory. We still need to demote the registers but don't apply
  // the GS segment.
  bool NoMemAccess = !mayLoad(Inst) && !mayStore(Inst);

  // Case 1: Absolute register (RSP, RIP, R14) with no index.
  if (isAbsoluteReg(Base.getReg()) && Index.getReg() == 0) {
    Base.setReg(getReg64(Base.getReg()));
    return;
  }

  // Case 2: No base, absolute index with scale 1.
  if (Base.getReg() == 0 && isAbsoluteReg(Index.getReg()) &&
      Scale.isImm() && Scale.getImm() == 1) {
    Base.setReg(getReg64(Index.getReg()));
    Index.setReg(0);
    return;
  }

  // Case 3: No index and no base - use R14 as base.
  if (Index.getReg() == 0 && Base.getReg() == 0) {
    if (NoMemAccess)
      return;
    Base.setReg(LFIBaseReg);
    return;
  }

  // Case 4: Use GS segment if available.
  if (hasSegue(STI) && Segment.getReg() == 0) {
    if (!NoMemAccess)
      Segment.setReg(LFIBaseSeg);
    Base.setReg(getReg32(Base.getReg()));
    Index.setReg(getReg32(Index.getReg()));
    return;
  }

  // LEA doesn't need the non-segue sandboxing paths below.
  if (NoMemAccess)
    return;

  // Case 5: Need to use scratch register for sandboxing.
  MCRegister ScratchReg32 = 0;
  if (ScratchReg != 0) {
    ScratchReg32 = getReg32(ScratchReg);
  } else {
    error(Inst, "Not enough scratch registers for sandboxed memory operation.");
    return;
  }

  // Case 5a: Absolute base with non-absolute index and zero offset.
  if (isAbsoluteReg(Base.getReg()) && !isAbsoluteReg(Index.getReg()) &&
      Offset.isImm() && Offset.getImm() == 0) {
    MCInst MovIdxToScratch;
    MovIdxToScratch.setOpcode(X86::MOV32rr);
    MovIdxToScratch.addOperand(MCOperand::createReg(ScratchReg32));
    MovIdxToScratch.addOperand(MCOperand::createReg(getReg32(Index.getReg())));
    Out.emitInstruction(MovIdxToScratch, STI);

    Base.setReg(getReg64(Base.getReg()));
    Index.setReg(getReg64(ScratchReg32));
    return;
  }

  // Case 5b: No index with base and zero offset.
  if (Index.getReg() == 0 && Base.getReg() != 0 && Offset.isImm() &&
      Offset.getImm() == 0) {
    MCInst MovBaseToScratch;
    MovBaseToScratch.setOpcode(X86::MOV32rr);
    MovBaseToScratch.addOperand(MCOperand::createReg(ScratchReg32));
    MovBaseToScratch.addOperand(MCOperand::createReg(getReg32(Base.getReg())));
    Out.emitInstruction(MovBaseToScratch, STI);

    Index.setReg(getReg64(ScratchReg32));
    Base.setReg(LFIBaseReg);
    return;
  }

  // General case: Use LEA to compute effective address.
  MCRegister ScratchReg64 = getReg64(ScratchReg32);
  MCRegister BaseReg64 = getReg64(Base.getReg());
  MCRegister IndexReg64 = getReg64(Index.getReg());

  MCInst Lea;
  Lea.setOpcode(X86::LEA64_32r);
  Lea.addOperand(MCOperand::createReg(ScratchReg32));
  Lea.addOperand(MCOperand::createReg(BaseReg64));
  Lea.addOperand(Scale);
  Lea.addOperand(MCOperand::createReg(IndexReg64));
  Lea.addOperand(Offset);
  Lea.addOperand(Segment);

  // Special case: no base and scale is 1.
  if (Base.getReg() == 0 && Scale.isImm() && Scale.getImm() == 1) {
    Lea.getOperand(1).setReg(IndexReg64);
    Lea.getOperand(3).setReg(0);
  }

  Out.emitInstruction(Lea, STI);

  Base.setReg(LFIBaseReg);
  Scale.setImm(1);
  Index.setReg(ScratchReg64);
  if (Offset.isImm()) {
    Offset.setImm(0);
  } else {
    Inst.erase(Inst.begin() + MemIdx + 3);
    Inst.insert(Inst.begin() + MemIdx + 3, MCOperand::createImm(0));
  }
}

static bool willEmitSandboxInsts(const MCInst &Inst, int Idx) {
  const MCOperand &Base = Inst.getOperand(Idx);
  const MCOperand &Scale = Inst.getOperand(Idx + 1);
  const MCOperand &Index = Inst.getOperand(Idx + 2);

  if (isAbsoluteReg(Base.getReg()) && Index.getReg() == 0) {
    return false;
  } else if (Base.getReg() == 0 && isAbsoluteReg(Index.getReg()) &&
             Scale.isImm() && Scale.getImm() == 1) {
    return false;
  }

  return true;
}

bool X86::X86MCLFIRewriter::emitSandboxMemOps(MCInst &Inst,
                                               MCRegister ScratchReg,
                                               MCStreamer &Out,
                                               const MCSubtargetInfo &STI,
                                               bool EmitInstructions) {
  const ArrayRef<MCOperandInfo> OpInfo =
      InstInfo->get(Inst.getOpcode()).operands();

  bool AnyInstsEmitted = false;

  for (int i = 0, e = Inst.getNumOperands(); i < e; ++i) {
    if (OpInfo[i].OperandType == MCOI::OPERAND_MEMORY) {
      prepareSandboxMemOp(Inst, i, ScratchReg, Out, STI);

      if (!AnyInstsEmitted && willEmitSandboxInsts(Inst, i)) {
        if (!EmitInstructions)
          return true;

        if (!hasSegue(STI)) {
          maybeEmitBundleLock(false, Out, STI);
          AnyInstsEmitted = true;
        }
      }
      emitSandboxMemOp(Inst, i, ScratchReg, Out, STI);
      i += 4;
    }
  }

  return AnyInstsEmitted;
}

//===----------------------------------------------------------------------===//
// Load/store expansion
//===----------------------------------------------------------------------===//

static unsigned normalizeOpcode(unsigned Op) {
  switch (Op) {
  case X86::ADD_FrST0:
    return X86::ADD_FST0r;
  case X86::DIVR_FrST0:
    return X86::DIVR_FST0r;
  case X86::DIV_FrST0:
    return X86::DIV_FST0r;
  case X86::MUL_FrST0:
    return X86::MUL_FST0r;
  case X86::SUBR_FrST0:
    return X86::SUBR_FST0r;
  case X86::SUB_FrST0:
    return X86::SUB_FST0r;
  }
  return Op;
}

void X86::X86MCLFIRewriter::expandLoadStore(const MCInst &Inst, MCStreamer &Out,
                                             const MCSubtargetInfo &STI,
                                             bool EmitPrefixes) {
  unsigned Op = Inst.getOpcode();

  // Optimize: for mov into a register, use dest as scratch.
  bool ElideScratchReg = false;
  switch (Op) {
  case X86::MOV64rm:
  case X86::MOV32rm:
  case X86::MOV16rm:
  case X86::MOV8rm:
    ElideScratchReg = true;
    break;
  }

  MCInst SandboxedInst(Inst);

  if (normalizeOpcode(Op) != Op)
    SandboxedInst.setOpcode(normalizeOpcode(Op));

  MCRegister ScratchReg;
  if (ElideScratchReg)
    ScratchReg = Inst.getOperand(0).getReg();
  else
    ScratchReg = X86::R11D;

  // Check if sandboxing is needed.
  bool InstNeedsSandboxing =
      emitSandboxMemOps(SandboxedInst, ScratchReg, Out, STI, false);

  // Handle high byte registers (AH, BH, CH, DH) - need rotation.
  MCRegister RotateRegister = X86::NoRegister;
  if (InstNeedsSandboxing &&
      (SandboxedInst.getOpcode() == X86::MOV8rm_NOREX ||
       SandboxedInst.getOpcode() == X86::MOV8rm) &&
      isHighReg(SandboxedInst.getOperand(0).getReg())) {
    RotateRegister = SandboxedInst.getOperand(0).getReg();
    SandboxedInst.setOpcode(X86::MOV8rm);
    SandboxedInst.getOperand(0).setReg(
        getX86SubSuperRegister(RotateRegister, 8, false));
  } else if (InstNeedsSandboxing &&
             (SandboxedInst.getOpcode() == X86::MOV8mr_NOREX ||
              SandboxedInst.getOpcode() == X86::MOV8mr) &&
             isHighReg(SandboxedInst.getOperand(5).getReg())) {
    RotateRegister = SandboxedInst.getOperand(5).getReg();
    SandboxedInst.setOpcode(X86::MOV8mr);
    SandboxedInst.getOperand(5).setReg(
        getX86SubSuperRegister(RotateRegister, 8, false));
  }

  if (RotateRegister != X86::NoRegister) {
    MCInst RotateHtoL;
    RotateHtoL.setOpcode(X86::ROR64ri);
    RotateHtoL.addOperand(MCOperand::createReg(getReg64(RotateRegister)));
    RotateHtoL.addOperand(MCOperand::createReg(getReg64(RotateRegister)));
    RotateHtoL.addOperand(MCOperand::createImm(8));
    Out.emitInstruction(RotateHtoL, STI);
  }

  bool BundleLock = emitSandboxMemOps(SandboxedInst, ScratchReg, Out, STI, true);
  emitInstruction(SandboxedInst, Out, STI, EmitPrefixes);
  if (BundleLock)
    maybeEmitBundleUnlock(Out, STI);

  if (RotateRegister != X86::NoRegister) {
    MCInst RotateLtoH;
    RotateLtoH.setOpcode(X86::ROL64ri);
    RotateLtoH.addOperand(MCOperand::createReg(getReg64(RotateRegister)));
    RotateLtoH.addOperand(MCOperand::createReg(getReg64(RotateRegister)));
    RotateLtoH.addOperand(MCOperand::createImm(8));
    Out.emitInstruction(RotateLtoH, STI);
  }
}

//===----------------------------------------------------------------------===//
// Opcode demotion (64-bit to 32-bit)
//===----------------------------------------------------------------------===//

static unsigned demoteOpcode(unsigned Opcode) {
  switch (Opcode) {
  case X86::ADC64rr:
    return X86::ADC32rr;
  case X86::ADC64ri8:
    return X86::ADC32ri8;
  case X86::ADC64ri32:
    return X86::ADC32ri;
  case X86::ADC64rm:
    return X86::ADC32rm;
  case X86::ADCX64rr:
    return X86::ADCX32rr;
  case X86::ADCX64rm:
    return X86::ADCX32rm;
  case X86::ADD64rr:
    return X86::ADD32rr;
  case X86::ADD64ri8:
    return X86::ADD32ri8;
  case X86::ADD64ri32:
    return X86::ADD32ri;
  case X86::ADD64rm:
    return X86::ADD32rm;
  case X86::ADOX64rr:
    return X86::ADOX32rr;
  case X86::ADOX64rm:
    return X86::ADOX32rm;
  case X86::ANDN64rr:
    return X86::ANDN32rr;
  case X86::ANDN64rm:
    return X86::ANDN32rm;
  case X86::AND64rr:
    return X86::AND32rr;
  case X86::AND64ri8:
    return X86::AND32ri8;
  case X86::AND64ri32:
    return X86::AND32ri;
  case X86::AND64rm:
    return X86::AND32rm;
  case X86::BEXTRI64ri:
    return X86::BEXTRI32ri;
  case X86::BEXTRI64mi:
    return X86::BEXTRI32mi;
  case X86::BEXTR64rr:
    return X86::BEXTR32rr;
  case X86::BEXTR64rm:
    return X86::BEXTR32rm;
  case X86::BLCFILL64rr:
    return X86::BLCFILL32rr;
  case X86::BLCFILL64rm:
    return X86::BLCFILL32rm;
  case X86::BLCI64rr:
    return X86::BLCI32rr;
  case X86::BLCI64rm:
    return X86::BLCI32rm;
  case X86::BLCIC64rr:
    return X86::BLCIC32rr;
  case X86::BLCIC64rm:
    return X86::BLCIC32rm;
  case X86::BLCMSK64rr:
    return X86::BLCMSK32rr;
  case X86::BLCMSK64rm:
    return X86::BLCMSK32rm;
  case X86::BLCS64rr:
    return X86::BLCS32rr;
  case X86::BLCS64rm:
    return X86::BLCS32rm;
  case X86::BLSFILL64rr:
    return X86::BLSFILL32rr;
  case X86::BLSFILL64rm:
    return X86::BLSFILL32rm;
  case X86::BLSIC64rr:
    return X86::BLSIC32rr;
  case X86::BLSIC64rm:
    return X86::BLSIC32rm;
  case X86::BLSI64rr:
    return X86::BLSI32rr;
  case X86::BLSI64rm:
    return X86::BLSI32rm;
  case X86::BLSMSK64rr:
    return X86::BLSMSK32rr;
  case X86::BLSMSK64rm:
    return X86::BLSMSK32rm;
  case X86::BLSR64rr:
    return X86::BLSR32rr;
  case X86::BLSR64rm:
    return X86::BLSR32rm;
  case X86::BSF64rr:
    return X86::BSF32rr;
  case X86::BSF64rm:
    return X86::BSF32rm;
  case X86::BSR64rr:
    return X86::BSR32rr;
  case X86::BSR64rm:
    return X86::BSR32rm;
  case X86::BSWAP64r:
    return X86::BSWAP32r;
  case X86::BTC64rr:
    return X86::BTC32rr;
  case X86::BTC64ri8:
    return X86::BTC32ri8;
  case X86::BT64rr:
    return X86::BT32rr;
  case X86::BT64ri8:
    return X86::BT32ri8;
  case X86::BTR64rr:
    return X86::BTR32rr;
  case X86::BTR64ri8:
    return X86::BTR32ri8;
  case X86::BTS64rr:
    return X86::BTS32rr;
  case X86::BTS64ri8:
    return X86::BTS32ri8;
  case X86::BZHI64rr:
    return X86::BZHI32rr;
  case X86::BZHI64rm:
    return X86::BZHI32rm;
  case X86::CALL64r:
    return X86::CALL32r;
  case X86::XOR64rr:
    return X86::XOR32rr;
  case X86::CMOV64rr:
    return X86::CMOV32rr;
  case X86::CMOV64rm:
    return X86::CMOV32rm;
  case X86::CMP64rr:
    return X86::CMP32rr;
  case X86::CMP64ri8:
    return X86::CMP32ri8;
  case X86::CMP64ri32:
    return X86::CMP32ri;
  case X86::CMP64rm:
    return X86::CMP32rm;
  case X86::CMPXCHG64rr:
    return X86::CMPXCHG32rr;
  case X86::CRC32r64r8:
    return X86::CRC32r32r8;
  case X86::CRC32r64m8:
    return X86::CRC32r64m8;
  case X86::CRC32r64r64:
    return X86::CRC32r32r32;
  case X86::CRC32r64m64:
    return X86::CRC32r32m32;
  case X86::CVTSD2SI64rr_Int:
    return X86::CVTSD2SIrr_Int;
  case X86::CVTSD2SI64rm_Int:
    return X86::CVTSD2SIrm_Int;
  case X86::CVTSS2SI64rr_Int:
    return X86::CVTSS2SIrr_Int;
  case X86::CVTSS2SI64rm_Int:
    return X86::CVTSS2SIrm_Int;
  case X86::CVTTSD2SI64rr:
    return X86::CVTTSD2SIrr;
  case X86::CVTTSD2SI64rm:
    return X86::CVTTSD2SIrm;
  case X86::CVTTSS2SI64rr:
    return X86::CVTTSS2SIrr;
  case X86::CVTTSS2SI64rm:
    return X86::CVTTSS2SIrm;
  case X86::DEC64r:
    return X86::DEC32r;
  case X86::DIV64r:
    return X86::DIV32r;
  case X86::IDIV64r:
    return X86::IDIV32r;
  case X86::IMUL64r:
    return X86::IMUL32r;
  case X86::IMUL64rr:
    return X86::IMUL32rr;
  case X86::IMUL64rri8:
    return X86::IMUL32rri8;
  case X86::IMUL64rri32:
    return X86::IMUL32rri;
  case X86::IMUL64rm:
    return X86::IMUL32rm;
  case X86::IMUL64rmi8:
    return X86::IMUL32rmi8;
  case X86::IMUL64rmi32:
    return X86::IMUL32rmi;
  case X86::INC64r:
    return X86::INC32r;
  case X86::INVEPT64:
    return X86::INVEPT32;
  case X86::INVPCID64:
    return X86::INVPCID32;
  case X86::INVVPID64:
    return X86::INVVPID32;
  case X86::JMP64r:
    return X86::JMP32r;
  case X86::KMOVQrk:
    return X86::KMOVQrk;
  case X86::LAR64rr:
    return X86::LAR32rr;
  case X86::LAR64rm:
    return X86::LAR32rm;
  case X86::LEA64r:
    return X86::LEA32r;
  case X86::LFS64rm:
    return X86::LFS32rm;
  case X86::LGS64rm:
    return X86::LGS32rm;
  case X86::LSL64rr:
    return X86::LSL32rr;
  case X86::LSL64rm:
    return X86::LSL32rm;
  case X86::LSS64rm:
    return X86::LSS32rm;
  case X86::LZCNT64rr:
    return X86::LZCNT32rr;
  case X86::LZCNT64rm:
    return X86::LZCNT32rm;
  case X86::MOV64ri:
    return X86::MOV32ri;
  case X86::MOVBE64rm:
    return X86::MOVBE32rm;
  case X86::MOV64rr:
    return X86::MOV32rr;
  case X86::MMX_MOVD64from64rr:
    return X86::MMX_MOVD64grr;
  case X86::MOVPQIto64rr:
    return X86::MOVPDI2DIrr;
  case X86::MOV64rs:
    return X86::MOV32rs;
  case X86::MOV64rd:
    return X86::MOV32rd;
  case X86::MOV64rc:
    return X86::MOV32rc;
  case X86::MOV64ri32:
    return X86::MOV32ri;
  case X86::MOV64rm:
    return X86::MOV32rm;
  case X86::MOVSX64rr8:
    return X86::MOVSX32rr8;
  case X86::MOVSX64rm8:
    return X86::MOVSX32rm8;
  case X86::MOVSX64rr32:
    return X86::MOV32rr;
  case X86::MOVSX64rm32:
    return X86::MOV32rm;
  case X86::MOVSX64rr16:
    return X86::MOVSX32rr16;
  case X86::MOVSX64rm16:
    return X86::MOVSX32rm16;
  case X86::MOVZX64rr8:
    return X86::MOVZX32rr8;
  case X86::MOVZX64rm8:
    return X86::MOVZX32rm8;
  case X86::MOVZX64rr16:
    return X86::MOVZX32rr16;
  case X86::MOVZX64rm16:
    return X86::MOVZX32rm16;
  case X86::MUL64r:
    return X86::MUL32r;
  case X86::MULX64rr:
    return X86::MULX32rr;
  case X86::MULX64rm:
    return X86::MULX32rm;
  case X86::NEG64r:
    return X86::NEG32r;
  case X86::NOT64r:
    return X86::NOT32r;
  case X86::OR64rr:
    return X86::OR32rr;
  case X86::OR64ri8:
    return X86::OR32ri8;
  case X86::OR64ri32:
    return X86::OR32ri;
  case X86::OR64rm:
    return X86::OR32rm;
  case X86::PDEP64rr:
    return X86::PDEP32rr;
  case X86::PDEP64rm:
    return X86::PDEP32rm;
  case X86::PEXT64rr:
    return X86::PEXT32rr;
  case X86::PEXT64rm:
    return X86::PEXT32rm;
  case X86::PEXTRQrri:
    return X86::PEXTRQrri;
  case X86::POPCNT64rr:
    return X86::POPCNT32rr;
  case X86::POPCNT64rm:
    return X86::POPCNT32rm;
  case X86::POP64r:
    return X86::POP32r;
  case X86::POP64rmr:
    return X86::POP32rmr;
  case X86::PUSH64r:
    return X86::PUSH32r;
  case X86::PUSH64rmr:
    return X86::PUSH32rmr;
  case X86::RCL64r1:
    return X86::RCL32r1;
  case X86::RCL64rCL:
    return X86::RCL32rCL;
  case X86::RCL64ri:
    return X86::RCL32ri;
  case X86::RCR64r1:
    return X86::RCR32r1;
  case X86::RCR64rCL:
    return X86::RCR32rCL;
  case X86::RCR64ri:
    return X86::RCR32ri;
  case X86::RDFSBASE64:
    return X86::RDFSBASE;
  case X86::RDGSBASE64:
    return X86::RDGSBASE;
  case X86::RDRAND64r:
    return X86::RDRAND32r;
  case X86::RDSEED64r:
    return X86::RDSEED32r;
  case X86::ROL64r1:
    return X86::ROL32r1;
  case X86::ROL64rCL:
    return X86::ROL32rCL;
  case X86::ROL64ri:
    return X86::ROL32ri;
  case X86::ROR64r1:
    return X86::ROR32r1;
  case X86::ROR64rCL:
    return X86::ROR32rCL;
  case X86::ROR64ri:
    return X86::ROR32ri;
  case X86::RORX64ri:
    return X86::RORX32ri;
  case X86::RORX64mi:
    return X86::RORX64mi;
  case X86::SAR64r1:
    return X86::SAR32r1;
  case X86::SAR64rCL:
    return X86::SAR32rCL;
  case X86::SAR64ri:
    return X86::SAR32ri;
  case X86::SARX64rr:
    return X86::SARX32rr;
  case X86::SARX64rm:
    return X86::SARX32rm;
  case X86::SBB64rr:
    return X86::SBB32rr;
  case X86::SBB64ri8:
    return X86::SBB32ri8;
  case X86::SBB64ri32:
    return X86::SBB32ri;
  case X86::SBB64rm:
    return X86::SBB32rm;
  case X86::SHLD64rrCL:
    return X86::SHLD32rrCL;
  case X86::SHLD64rri8:
    return X86::SHLD32rri8;
  case X86::SHL64r1:
    return X86::SHL32r1;
  case X86::SHL64rCL:
    return X86::SHL32rCL;
  case X86::SHL64ri:
    return X86::SHL32ri;
  case X86::SHLX64rr:
    return X86::SHLX32rr;
  case X86::SHLX64rm:
    return X86::SHLX32rm;
  case X86::SHRD64rrCL:
    return X86::SHRD32rrCL;
  case X86::SHRD64rri8:
    return X86::SHRD32rri8;
  case X86::SHR64r1:
    return X86::SHR32r1;
  case X86::SHR64rCL:
    return X86::SHR32rCL;
  case X86::SHR64ri:
    return X86::SHR32ri;
  case X86::SHRX64rr:
    return X86::SHRX32rr;
  case X86::SHRX64rm:
    return X86::SHRX32rm;
  case X86::SLDT64r:
    return X86::SLDT32r;
  case X86::SMSW64r:
    return X86::SMSW32r;
  case X86::STR64r:
    return X86::STR32r;
  case X86::SUB64rr:
    return X86::SUB32rr;
  case X86::SUB64ri8:
    return X86::SUB32ri8;
  case X86::SUB64ri32:
    return X86::SUB32ri;
  case X86::SUB64rm:
    return X86::SUB32rm;
  case X86::T1MSKC64rr:
    return X86::T1MSKC32rr;
  case X86::T1MSKC64rm:
    return X86::T1MSKC32rm;
  case X86::TEST64rr:
    return X86::TEST32rr;
  case X86::TEST64ri32:
    return X86::TEST32ri;
  case X86::TEST64mr:
    return X86::TEST32mr;
  case X86::TZCNT64rr:
    return X86::TZCNT32rr;
  case X86::TZCNT64rm:
    return X86::TZCNT32rm;
  case X86::TZMSK64rr:
    return X86::TZMSK32rr;
  case X86::TZMSK64rm:
    return X86::TZMSK32rm;
  case X86::VCVTSD2SI64rr_Int:
    return X86::VCVTSD2SIrr_Int;
  case X86::VCVTSD2SI64Zrr_Int:
    return X86::VCVTSD2SIZrr_Int;
  case X86::VCVTSD2SI64Zrm_Int:
    return X86::VCVTSD2SIZrm_Int;
  case X86::VCVTSD2SI64rm_Int:
    return X86::VCVTSD2SIrm_Int;
  case X86::VCVTSD2USI64Zrr_Int:
    return X86::VCVTSD2USIZrr_Int;
  case X86::VCVTSD2USI64Zrm_Int:
    return X86::VCVTSD2USIZrm_Int;
  case X86::VCVTSS2SI64rr_Int:
    return X86::VCVTSS2SIrr_Int;
  case X86::VCVTSS2SI64Zrr_Int:
    return X86::VCVTSS2SIZrr_Int;
  case X86::VCVTSS2SI64Zrm_Int:
    return X86::VCVTSS2SIZrm_Int;
  case X86::VCVTSS2SI64rm_Int:
    return X86::VCVTSS2SIrm_Int;
  case X86::VCVTSS2USI64Zrr_Int:
    return X86::VCVTSS2USIZrr_Int;
  case X86::VCVTSS2USI64Zrm_Int:
    return X86::VCVTSS2USIZrm_Int;
  case X86::VCVTTSD2SI64rr:
    return X86::VCVTTSD2SIrr;
  case X86::VCVTTSD2SI64Zrr:
    return X86::VCVTTSD2SIZrr;
  case X86::VCVTTSD2SI64Zrm:
    return X86::VCVTTSD2SIZrm;
  case X86::VCVTTSD2SI64rm:
    return X86::VCVTTSD2SIrm;
  case X86::VCVTTSD2USI64Zrr:
    return X86::VCVTTSD2USIZrr;
  case X86::VCVTTSD2USI64Zrm:
    return X86::VCVTTSD2USIZrm;
  case X86::VCVTTSS2SI64rr:
    return X86::VCVTTSS2SIrr;
  case X86::VCVTTSS2SI64Zrr:
    return X86::VCVTTSS2SIZrr;
  case X86::VCVTTSS2SI64Zrm:
    return X86::VCVTTSS2SIZrm;
  case X86::VCVTTSS2SI64rm:
    return X86::VCVTTSS2SIrm;
  case X86::VCVTTSS2USI64Zrr:
    return X86::VCVTTSS2USIZrr;
  case X86::VCVTTSS2USI64Zrm:
    return X86::VCVTTSS2USIZrm;
  case X86::VMOVPQIto64rr:
    return X86::VMOVPDI2DIrr;
  case X86::VMOVPQIto64Zrr:
    return X86::VMOVPDI2DIZrr;
  case X86::VMREAD64rr:
    return X86::VMREAD32rr;
  case X86::VMWRITE64rr:
    return X86::VMWRITE32rr;
  case X86::VMWRITE64rm:
    return X86::VMWRITE32rm;
  case X86::VPEXTRQrri:
    return X86::VPEXTRQrri;
  case X86::WRFSBASE64:
    return X86::WRFSBASE;
  case X86::WRGSBASE64:
    return X86::WRGSBASE;
  case X86::XADD64rr:
    return X86::XADD32rr;
  case X86::XCHG64ar:
    return X86::XCHG32ar;
  case X86::XCHG64rr:
    return X86::XCHG32rr;
  case X86::XCHG64rm:
    return X86::XCHG32rm;
  case X86::XOR64ri8:
    return X86::XOR32ri8;
  case X86::XOR64ri32:
    return X86::XOR32ri;
  case X86::XOR64rm:
    return X86::XOR32rm;
  default:
    return Opcode;
  }
}

static void demoteInst(MCInst &Inst, const MCInstrInfo &InstInfo) {
  unsigned NewOpc = demoteOpcode(Inst.getOpcode());
  Inst.setOpcode(NewOpc);

  // Demote all 64-bit general purpose registers to 32-bit.
  const ArrayRef<MCOperandInfo> OpInfo = InstInfo.get(Inst.getOpcode()).operands();
  for (unsigned i = 0, e = Inst.getNumOperands(); i < e; ++i) {
    if (OpInfo[i].OperandType == MCOI::OPERAND_REGISTER) {
      assert(Inst.getOperand(i).isReg());
      MCRegister Reg = Inst.getOperand(i).getReg();
      if (getReg64(Reg) == Reg) {
        Inst.getOperand(i).setReg(getReg32(Reg));
      }
    }
  }
}

//===----------------------------------------------------------------------===//
// Main dispatch logic
//===----------------------------------------------------------------------===//

void X86::X86MCLFIRewriter::doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                                           const MCSubtargetInfo &STI,
                                           bool EmitPrefixes) {
  // Handle prefixes.
  if (isPrefix(Inst)) {
    Prefixes.push_back(Inst);
    return;
  }

  // Check for modification of reserved register R14.
  if (mayModifyRegister(Inst, LFIBaseReg)) {
    error(Inst, "illegal modification of reserved LFI register %r14");
    return;
  }

  // Dispatch based on instruction type.
  if (isSyscall(Inst)) {
    expandSyscall(Inst, Out, STI);
  } else if (isTLSRead(Inst)) {
    expandTLSRead(Inst, Out, STI);
  } else if (isDirectCall(Inst)) {
    expandDirectCall(Inst, Out, STI);
  } else if (isIndirectBranch(Inst) || isCall(Inst)) {
    expandIndirectBranch(Inst, Out, STI);
  } else if (isReturn(Inst)) {
    expandReturn(Inst, Out, STI);
  } else if (isStringOperation(Inst)) {
    expandStringOperation(Inst, Out, STI, EmitPrefixes);
  } else if (explicitlyModifiesRegister(Inst, X86::RSP)) {
    expandStackModification(X86::RSP, Inst, Out, STI, EmitPrefixes);
  } else if (xchgStackReg(Inst) != X86::NoRegister) {
    expandStackModification(X86::RSP, Inst, Out, STI, EmitPrefixes);
  } else {
    // Check for invalid use of GS segment.
    for (unsigned i = 0, e = Inst.getNumOperands(); i < e; ++i) {
      if (Inst.getOperand(i).isReg() && Inst.getOperand(i).getReg() == X86::GS) {
        error(Inst, "invalid use of %gs segment register");
        return;
      }
    }
    expandLoadStore(Inst, Out, STI, EmitPrefixes);
  }
}

bool X86::X86MCLFIRewriter::rewriteInst(const MCInst &Inst, MCStreamer &Out,
                                         const MCSubtargetInfo &STI) {
  if (Guard)
    return false;
  Guard = true;

  // Eagerly emit the trap symbol on first use when software CFI is enabled,
  // so the section switch doesn't happen inside a bundle lock.
  if (!FlagX86LFIBundling && !X86LFIHwEndbr && !LFITrapSymbol)
    getOrEmitTrapSymbol(Out, STI);

  doRewriteInst(Inst, Out, STI, true);

  Guard = false;
  return true;
}
