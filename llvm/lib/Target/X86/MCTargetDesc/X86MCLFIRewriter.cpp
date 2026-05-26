//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file implements the X86MCLFIRewriter class, which rewrites X86-64
/// instructions for LFI (Lightweight Fault Isolation) sandboxing.
///
//===----------------------------------------------------------------------===//

#include "X86MCLFIRewriter.h"
#include "X86BaseInfo.h"
#include "X86MCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

// LFI reserved registers and sandbox parameters.
static constexpr MCRegister LFIBaseReg = X86::R14;
static constexpr MCRegister LFIScratchReg = X86::R11;
static constexpr MCRegister LFITPReg = X86::R15;
static constexpr MCRegister LFIBaseSeg = X86::GS;

// Indirect branch targets must be aligned to a multiple of this size.
static constexpr unsigned BundleSize = 32;

// Byte offset into the context register file (pointed to by R15) where the
// thread pointer is stored.
static constexpr int TPOffset = 16;

//===----------------------------------------------------------------------===//
// Feature checking
//===----------------------------------------------------------------------===//

bool X86::X86MCLFIRewriter::hasSegue(const MCSubtargetInfo &STI) const {
  return !STI.hasFeature(X86::FeatureNoLFISegue);
}

bool X86::X86MCLFIRewriter::hasNoLFILoads(const MCSubtargetInfo &STI) const {
  return STI.hasFeature(X86::FeatureNoLFILoads);
}

bool X86::X86MCLFIRewriter::hasNoLFIStores(const MCSubtargetInfo &STI) const {
  return STI.hasFeature(X86::FeatureNoLFIStores);
}

//===----------------------------------------------------------------------===//
// Register helpers
//===----------------------------------------------------------------------===//

static MCRegister getReg64(MCRegister Reg) {
  switch (Reg) {
  case X86::IP:
  case X86::EIP:
  case X86::RIP:
    return X86::RIP;
  default:
    return getX86SubSuperRegister(Reg, 64, false);
  }
}

static MCRegister getReg32(MCRegister Reg) {
  switch (Reg) {
  case X86::IP:
  case X86::EIP:
    return X86::EIP;
  case X86::RIP:
    llvm_unreachable("Trying to demote %rip");
  default:
    return getX86SubSuperRegister(Reg, 32, false);
  }
}

// Registers that are always known to point inside the sandbox and thus do not
// need any further sandboxing when used as a memory base.
static bool isAbsoluteReg(MCRegister Reg) {
  Reg = getReg64(Reg);
  return Reg == LFIBaseReg || Reg == X86::RSP || Reg == X86::RIP;
}

static bool isHighReg(MCRegister Reg) {
  return Reg == X86::AH || Reg == X86::BH || Reg == X86::CH || Reg == X86::DH;
}

//===----------------------------------------------------------------------===//
// Instruction classification
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

static bool isSyscall(const MCInst &Inst) {
  return Inst.getOpcode() == X86::SYSCALL;
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

// xchg variants that swap a register with the stack pointer require the same
// sandboxing handling as an explicit modification of %rsp.
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

//===----------------------------------------------------------------------===//
// Prefix accumulation
//===----------------------------------------------------------------------===//

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
// Control-flow rewrites
//===----------------------------------------------------------------------===//

// Emit:
//   andl $-BundleSize, %eX
//   addq %r14, %rX
// This zeroes the top 32 bits and bottom log2(BundleSize) bits of the target,
// then fills in the top 32 bits with the sandbox base.
void X86::X86MCLFIRewriter::emitSandboxBranchReg(MCRegister Reg,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  MCRegister Reg32 = getReg32(Reg);
  MCRegister Reg64 = getReg64(Reg);

  MCInst And;
  And.setOpcode(X86::AND32ri8);
  And.addOperand(MCOperand::createReg(Reg32));
  And.addOperand(MCOperand::createReg(Reg32));
  And.addOperand(MCOperand::createImm(-static_cast<int>(BundleSize)));
  Out.emitInstruction(And, STI);

  MCInst Add;
  Add.setOpcode(X86::ADD64rr);
  Add.addOperand(MCOperand::createReg(Reg64));
  Add.addOperand(MCOperand::createReg(Reg64));
  Add.addOperand(MCOperand::createReg(LFIBaseReg));
  Out.emitInstruction(Add, STI);
}

void X86::X86MCLFIRewriter::rewriteIndirectJumpReg(MCRegister Reg,
                                                   MCStreamer &Out,
                                                   const MCSubtargetInfo &STI) {
  Out.emitBundleLock(/*AlignToEnd=*/false, STI);
  emitSandboxBranchReg(Reg, Out, STI);

  MCInst Jmp;
  Jmp.setOpcode(X86::JMP64r);
  Jmp.addOperand(MCOperand::createReg(getReg64(Reg)));
  Out.emitInstruction(Jmp, STI);

  Out.emitBundleUnlock(STI);
}

void X86::X86MCLFIRewriter::rewriteIndirectCallReg(MCRegister Reg,
                                                   MCStreamer &Out,
                                                   const MCSubtargetInfo &STI) {
  Out.emitBundleLock(/*AlignToEnd=*/true, STI);
  emitSandboxBranchReg(Reg, Out, STI);

  MCInst Call;
  Call.setOpcode(X86::CALL64r);
  Call.addOperand(MCOperand::createReg(getReg64(Reg)));
  Out.emitInstruction(Call, STI);

  Out.emitBundleUnlock(STI);
}

void X86::X86MCLFIRewriter::rewriteIndirectBranch(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  MCRegister Target;
  if (mayLoad(Inst)) {
    // Indirect jmp/call through memory: load the target address into the
    // scratch register first (with normal load sandboxing) and then dispatch
    // via the register-based rewrite.
    Target = LFIScratchReg;

    MCInst Mov;
    Mov.setOpcode(X86::MOV64rm);
    Mov.addOperand(MCOperand::createReg(Target));
    Mov.addOperand(Inst.getOperand(0));
    Mov.addOperand(Inst.getOperand(1));
    Mov.addOperand(Inst.getOperand(2));
    Mov.addOperand(Inst.getOperand(3));
    Mov.addOperand(Inst.getOperand(4));
    doRewriteInst(Mov, Out, STI, /*EmitPrefixes=*/false);
  } else {
    Target = Inst.getOperand(0).getReg();
  }

  if (isCall(Inst))
    rewriteIndirectCallReg(Target, Out, STI);
  else
    rewriteIndirectJumpReg(Target, Out, STI);
}

void X86::X86MCLFIRewriter::rewriteDirectCall(const MCInst &Inst,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
  // Direct calls must be placed at the end of a bundle so the return address
  // is bundle-aligned.
  Out.emitBundleLock(/*AlignToEnd=*/true, STI);
  Out.emitInstruction(Inst, STI);
  Out.emitBundleUnlock(STI);
}

void X86::X86MCLFIRewriter::rewriteReturn(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
  // popq %r11
  MCInst Pop;
  Pop.setOpcode(X86::POP64r);
  Pop.addOperand(MCOperand::createReg(LFIScratchReg));
  Out.emitInstruction(Pop, STI);

  // ret with immediate: pop additional bytes from the stack with addq, which
  // re-enters the rewriter so the %rsp modification is sandboxed.
  if (Inst.getOpcode() == X86::RETI32 || Inst.getOpcode() == X86::RETI64) {
    MCInst Add;
    Add.setOpcode(X86::ADD64ri32);
    Add.addOperand(MCOperand::createReg(X86::RSP));
    Add.addOperand(MCOperand::createReg(X86::RSP));
    Add.addOperand(Inst.getOperand(0));
    doRewriteInst(Add, Out, STI, /*EmitPrefixes=*/false);
  }

  rewriteIndirectJumpReg(LFIScratchReg, Out, STI);
}

//===----------------------------------------------------------------------===//
// Syscall and TLS rewrites
//===----------------------------------------------------------------------===//

// syscall
// ->
// .bundle_lock
// leaq .Ltmp(%rip), %r11
// jmpq *(%r14)
// .Ltmp:
// .bundle_unlock
//
// The leaq and jmpq must execute atomically as a single bundle so the
// runtime always sees %r11 holding the post-syscall return address.
void X86::X86MCLFIRewriter::rewriteSyscall(const MCInst &Inst, MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {
  Out.emitBundleLock(/*AlignToEnd=*/false, STI);

  MCSymbol *Symbol = Out.getContext().createTempSymbol();

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

  MCInst Jmp;
  Jmp.setOpcode(X86::JMP64m);
  Jmp.addOperand(MCOperand::createReg(LFIBaseReg));
  Jmp.addOperand(MCOperand::createImm(1));
  Jmp.addOperand(MCOperand::createReg(X86::NoRegister));
  Jmp.addOperand(MCOperand::createImm(0));
  Jmp.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Jmp, STI);

  Out.emitLabel(Symbol);
  Out.emitBundleUnlock(STI);
}

//===----------------------------------------------------------------------===//
// %fs-segment (TLS) rewriting
//===----------------------------------------------------------------------===//

// Find the index of the first memory operand with %fs segment override, or -1
// if Inst has no such operand.
static int findFSMemOperand(const MCInst &Inst, const MCInstrInfo &InstInfo) {
  const MCInstrDesc &Desc = InstInfo.get(Inst.getOpcode());
  for (unsigned I = 0, E = Desc.getNumOperands(); I < E; ++I) {
    if (Desc.operands()[I].OperandType == MCOI::OPERAND_MEMORY) {
      if (I + 4 < Inst.getNumOperands() && Inst.getOperand(I + 4).isReg() &&
          Inst.getOperand(I + 4).getReg() == X86::FS)
        return I;
      I += 4;
    }
  }
  return -1;
}

// Emit: movq TPOffset(%r15), %Reg
static void emitTPLoad(MCRegister Reg, MCStreamer &Out,
                       const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(X86::MOV64rm);
  Mov.addOperand(MCOperand::createReg(Reg));
  Mov.addOperand(MCOperand::createReg(LFITPReg));
  Mov.addOperand(MCOperand::createImm(1));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Mov.addOperand(MCOperand::createImm(TPOffset));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Mov, STI);
}

bool X86::X86MCLFIRewriter::isFSAccess(const MCInst &Inst) {
  return (mayLoad(Inst) || mayStore(Inst)) &&
         findFSMemOperand(Inst, *InstInfo) >= 0;
}

// Rewrite %fs-segment memory accesses to use the virtual thread pointer stored
// at TPOffset(%r15). Example rewrites:
//
// movq %fs:0, %rax
// ->
// movq 16(%r15), %rax
//
// movq %fs:(%rdi), %rax
// ->
// movq 16(%r15), %rax
// movq (%rax, %rdi), %rax
//
// movq 8(%rdi, %rsi, 2), %rax
// ->
// movq 16(%r15), %rax
// leaq (%rax, %rdi), %rax
// movq 8(%rax, %rsi, 2), %rax
void X86::X86MCLFIRewriter::rewriteFSAccess(const MCInst &Inst, MCStreamer &Out,
                                            const MCSubtargetInfo &STI) {
  int MemIdx = findFSMemOperand(Inst, *InstInfo);
  assert(MemIdx >= 0);

  MCRegister BaseReg = Inst.getOperand(MemIdx).getReg();
  MCRegister IndexReg = Inst.getOperand(MemIdx + 2).getReg();
  bool HasBase = BaseReg != X86::NoRegister;
  bool HasIndex = IndexReg != X86::NoRegister;
  bool HasDisp = !Inst.getOperand(MemIdx + 3).isImm() ||
                 Inst.getOperand(MemIdx + 3).getImm() != 0;

  // %fs:0 -> TPOffset(%r15)
  if (!HasBase && !HasIndex && !HasDisp) {
    MCInst Modified(Inst);
    Modified.getOperand(MemIdx).setReg(LFITPReg);
    Modified.getOperand(MemIdx + 3).setImm(TPOffset);
    Modified.getOperand(MemIdx + 4).setReg(X86::NoRegister);
    return Out.emitInstruction(Modified, STI);
  }

  // Use the dest register as TP temporary when it is available and not used in
  // the addressing mode, otherwise use %r11.
  MCRegister TPDest = LFIScratchReg;
  if (MemIdx > 0 && Inst.getOperand(0).isReg()) {
    const MCInstrDesc &Desc = InstInfo->get(Inst.getOpcode());
    MCRegister DestReg = Inst.getOperand(0).getReg();
    if (Desc.getOperandConstraint(0, MCOI::TIED_TO) == -1 &&
        X86MCRegisterClasses[X86::GR64RegClassID].contains(DestReg) &&
        (!HasBase || DestReg != BaseReg) && (!HasIndex || DestReg != IndexReg))
      TPDest = DestReg;
  }

  emitTPLoad(TPDest, Out, STI);

  // Both slots occupied: fold base into TPDest via lea.
  if (HasBase && HasIndex) {
    MCInst Lea;
    Lea.setOpcode(X86::LEA64r);
    Lea.addOperand(MCOperand::createReg(TPDest));
    Lea.addOperand(MCOperand::createReg(TPDest));
    Lea.addOperand(MCOperand::createImm(1));
    Lea.addOperand(MCOperand::createReg(BaseReg));
    Lea.addOperand(MCOperand::createImm(0));
    Lea.addOperand(MCOperand::createReg(X86::NoRegister));
    Out.emitInstruction(Lea, STI);
  }

  MCInst Modified(Inst);
  Modified.getOperand(MemIdx).setReg(TPDest);
  if (HasBase && !HasIndex)
    Modified.getOperand(MemIdx + 2).setReg(BaseReg);
  Modified.getOperand(MemIdx + 4).setReg(X86::NoRegister);
  // Re-enter the rewriter so the resulting non-FS access is memory-sandboxed
  // by the standard load/store path.
  doRewriteInst(Modified, Out, STI, /*EmitPrefixes=*/false);
}

//===----------------------------------------------------------------------===//
// Memory sandboxing
//===----------------------------------------------------------------------===//

// Hook for memory-operand pre-processing. The %fs segment is handled by the
// dedicated rewriteFSAccess path before the load/store sandbox pipeline is
// entered, so this is currently a no-op.
void X86::X86MCLFIRewriter::prepareSandboxMemOp(MCInst &Inst, int MemIdx,
                                                MCRegister ScratchReg,
                                                MCStreamer &Out,
                                                const MCSubtargetInfo &STI) {
  (void)Inst;
  (void)MemIdx;
  (void)ScratchReg;
  (void)Out;
  (void)STI;
}

// Returns true if emitSandboxMemOp would emit any auxiliary instructions for
// the memory operand at \p Idx (i.e., something more than rewriting the
// addressing mode in place).
static bool willEmitSandboxInsts(const MCInst &Inst, int Idx) {
  const MCOperand &Base = Inst.getOperand(Idx);
  const MCOperand &Scale = Inst.getOperand(Idx + 1);
  const MCOperand &Index = Inst.getOperand(Idx + 2);

  if (isAbsoluteReg(Base.getReg()) && Index.getReg() == X86::NoRegister)
    return false;
  if (Base.getReg() == X86::NoRegister && isAbsoluteReg(Index.getReg()) &&
      Scale.isImm() && Scale.getImm() == 1)
    return false;
  return true;
}

void X86::X86MCLFIRewriter::emitSandboxMemOp(MCInst &Inst, int MemIdx,
                                             MCRegister ScratchReg,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI) {
  bool SkipLoads = hasNoLFILoads(STI);
  bool SkipStores = hasNoLFIStores(STI);

  // Pure load/store gating: skip if both kinds are skipped or the relevant
  // direction is disabled.
  if (SkipLoads && SkipStores)
    return;
  if (SkipLoads && !mayStore(Inst))
    return;
  if (SkipStores && !mayLoad(Inst))
    return;

  MCOperand &Base = Inst.getOperand(MemIdx);
  MCOperand &Scale = Inst.getOperand(MemIdx + 1);
  MCOperand &Index = Inst.getOperand(MemIdx + 2);
  MCOperand &Offset = Inst.getOperand(MemIdx + 3);
  MCOperand &Segment = Inst.getOperand(MemIdx + 4);

  // LEA-style instructions have a memory operand but do not actually access
  // memory. The addressing mode still needs to be valid x86-64, but does not
  // need a sandbox prefix.
  bool NoMemAccess = !mayLoad(Inst) && !mayStore(Inst);

  // Case 1: base is an absolute register and there is no index.
  if (isAbsoluteReg(Base.getReg()) && Index.getReg() == X86::NoRegister) {
    Base.setReg(getReg64(Base.getReg()));
    return;
  }

  // Case 2: only an index, with scale 1, and the index is absolute.
  if (Base.getReg() == X86::NoRegister && isAbsoluteReg(Index.getReg()) &&
      Scale.isImm() && Scale.getImm() == 1) {
    Base.setReg(getReg64(Index.getReg()));
    Index.setReg(X86::NoRegister);
    return;
  }

  // Case 3: pure absolute address (no base, no index). Use %r14 as the base.
  if (Base.getReg() == X86::NoRegister && Index.getReg() == X86::NoRegister) {
    if (NoMemAccess)
      return;
    Base.setReg(LFIBaseReg);
    return;
  }

  // Case 4: prefer the %gs segment if available. This is the cheap path
  // because it avoids any scratch-register fixup; just demote the registers
  // to 32 bits and add the segment override.
  if (hasSegue(STI) && Segment.getReg() == X86::NoRegister) {
    if (!NoMemAccess)
      Segment.setReg(LFIBaseSeg);
    if (Base.getReg() != X86::NoRegister)
      Base.setReg(getReg32(Base.getReg()));
    if (Index.getReg() != X86::NoRegister)
      Index.setReg(getReg32(Index.getReg()));
    return;
  }

  // LEA does not need the non-segue sandboxing paths below.
  if (NoMemAccess)
    return;

  if (ScratchReg == X86::NoRegister) {
    error(Inst, "not enough scratch registers for sandboxed memory operation");
    return;
  }
  MCRegister ScratchReg32 = getReg32(ScratchReg);
  MCRegister ScratchReg64 = getReg64(ScratchReg);

  // Case 5a: absolute base + non-absolute index, zero offset. Fold the index
  // into the scratch register and use the original (absolute) base.
  if (isAbsoluteReg(Base.getReg()) && !isAbsoluteReg(Index.getReg()) &&
      Offset.isImm() && Offset.getImm() == 0) {
    MCInst MovIdxToScratch;
    MovIdxToScratch.setOpcode(X86::MOV32rr);
    MovIdxToScratch.addOperand(MCOperand::createReg(ScratchReg32));
    MovIdxToScratch.addOperand(MCOperand::createReg(getReg32(Index.getReg())));
    Out.emitInstruction(MovIdxToScratch, STI);

    Base.setReg(getReg64(Base.getReg()));
    Index.setReg(ScratchReg64);
    return;
  }

  // Case 5b: non-absolute base, no index, zero offset. Use the base as the
  // sandboxed offset and %r14 as the base.
  if (Index.getReg() == X86::NoRegister && Base.getReg() != X86::NoRegister &&
      Offset.isImm() && Offset.getImm() == 0) {
    MCInst MovBaseToScratch;
    MovBaseToScratch.setOpcode(X86::MOV32rr);
    MovBaseToScratch.addOperand(MCOperand::createReg(ScratchReg32));
    MovBaseToScratch.addOperand(MCOperand::createReg(getReg32(Base.getReg())));
    Out.emitInstruction(MovBaseToScratch, STI);

    Index.setReg(ScratchReg64);
    Base.setReg(LFIBaseReg);
    return;
  }

  // General case: compute the effective address with a 32-bit lea into the
  // scratch register, then use it as the index with %r14 as the base.
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

  // Special case: no base, scale 1. Move the index into the base slot so the
  // addressing mode is canonical.
  if (Base.getReg() == X86::NoRegister && Scale.isImm() && Scale.getImm() == 1) {
    Lea.getOperand(1).setReg(IndexReg64);
    Lea.getOperand(3).setReg(X86::NoRegister);
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

bool X86::X86MCLFIRewriter::emitSandboxMemOps(MCInst &Inst,
                                              MCRegister ScratchReg,
                                              MCStreamer &Out,
                                              const MCSubtargetInfo &STI,
                                              bool EmitInstructions) {
  const MCInstrDesc &Desc = InstInfo->get(Inst.getOpcode());
  const ArrayRef<MCOperandInfo> OpInfo = Desc.operands();

  bool BundleLockOpened = false;

  for (int I = 0, E = Inst.getNumOperands(); I < E; ++I) {
    if (OpInfo[I].OperandType != MCOI::OPERAND_MEMORY)
      continue;

    prepareSandboxMemOp(Inst, I, ScratchReg, Out, STI);

    if (!BundleLockOpened && willEmitSandboxInsts(Inst, I)) {
      if (!EmitInstructions)
        return true;

      // Without segment-based sandboxing the address computation and the
      // memory access must live in the same bundle, since otherwise the
      // verifier could not prove the access uses the freshly-computed
      // sandboxed address.
      if (!hasSegue(STI)) {
        Out.emitBundleLock(/*AlignToEnd=*/false, STI);
        BundleLockOpened = true;
      }
    }
    emitSandboxMemOp(Inst, I, ScratchReg, Out, STI);
    I += 4;
  }

  return BundleLockOpened;
}

//===----------------------------------------------------------------------===//
// Opcode demotion (64-bit -> 32-bit)
//===----------------------------------------------------------------------===//

// Some 64-bit opcodes have a different "normalized" form when used with a
// 32-bit operand register; map them to a canonical form before demotion.
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
  default:
    return Op;
  }
}

static unsigned demoteOpcode(unsigned Opcode) {
  switch (Opcode) {
  case X86::ADC64rr:    return X86::ADC32rr;
  case X86::ADC64ri8:   return X86::ADC32ri8;
  case X86::ADC64ri32:  return X86::ADC32ri;
  case X86::ADC64rm:    return X86::ADC32rm;
  case X86::ADCX64rr:   return X86::ADCX32rr;
  case X86::ADCX64rm:   return X86::ADCX32rm;
  case X86::ADD64rr:    return X86::ADD32rr;
  case X86::ADD64ri8:   return X86::ADD32ri8;
  case X86::ADD64ri32:  return X86::ADD32ri;
  case X86::ADD64rm:    return X86::ADD32rm;
  case X86::ADOX64rr:   return X86::ADOX32rr;
  case X86::ADOX64rm:   return X86::ADOX32rm;
  case X86::ANDN64rr:   return X86::ANDN32rr;
  case X86::ANDN64rm:   return X86::ANDN32rm;
  case X86::AND64rr:    return X86::AND32rr;
  case X86::AND64ri8:   return X86::AND32ri8;
  case X86::AND64ri32:  return X86::AND32ri;
  case X86::AND64rm:    return X86::AND32rm;
  case X86::BEXTRI64ri: return X86::BEXTRI32ri;
  case X86::BEXTRI64mi: return X86::BEXTRI32mi;
  case X86::BEXTR64rr:  return X86::BEXTR32rr;
  case X86::BEXTR64rm:  return X86::BEXTR32rm;
  case X86::BLCFILL64rr:return X86::BLCFILL32rr;
  case X86::BLCFILL64rm:return X86::BLCFILL32rm;
  case X86::BLCI64rr:   return X86::BLCI32rr;
  case X86::BLCI64rm:   return X86::BLCI32rm;
  case X86::BLCIC64rr:  return X86::BLCIC32rr;
  case X86::BLCIC64rm:  return X86::BLCIC32rm;
  case X86::BLCMSK64rr: return X86::BLCMSK32rr;
  case X86::BLCMSK64rm: return X86::BLCMSK32rm;
  case X86::BLCS64rr:   return X86::BLCS32rr;
  case X86::BLCS64rm:   return X86::BLCS32rm;
  case X86::BLSFILL64rr:return X86::BLSFILL32rr;
  case X86::BLSFILL64rm:return X86::BLSFILL32rm;
  case X86::BLSIC64rr:  return X86::BLSIC32rr;
  case X86::BLSIC64rm:  return X86::BLSIC32rm;
  case X86::BLSI64rr:   return X86::BLSI32rr;
  case X86::BLSI64rm:   return X86::BLSI32rm;
  case X86::BLSMSK64rr: return X86::BLSMSK32rr;
  case X86::BLSMSK64rm: return X86::BLSMSK32rm;
  case X86::BLSR64rr:   return X86::BLSR32rr;
  case X86::BLSR64rm:   return X86::BLSR32rm;
  case X86::BSF64rr:    return X86::BSF32rr;
  case X86::BSF64rm:    return X86::BSF32rm;
  case X86::BSR64rr:    return X86::BSR32rr;
  case X86::BSR64rm:    return X86::BSR32rm;
  case X86::BSWAP64r:   return X86::BSWAP32r;
  case X86::BTC64rr:    return X86::BTC32rr;
  case X86::BTC64ri8:   return X86::BTC32ri8;
  case X86::BT64rr:     return X86::BT32rr;
  case X86::BT64ri8:    return X86::BT32ri8;
  case X86::BTR64rr:    return X86::BTR32rr;
  case X86::BTR64ri8:   return X86::BTR32ri8;
  case X86::BTS64rr:    return X86::BTS32rr;
  case X86::BTS64ri8:   return X86::BTS32ri8;
  case X86::BZHI64rr:   return X86::BZHI32rr;
  case X86::BZHI64rm:   return X86::BZHI32rm;
  case X86::CALL64r:    return X86::CALL32r;
  case X86::CMOV64rr:   return X86::CMOV32rr;
  case X86::CMOV64rm:   return X86::CMOV32rm;
  case X86::CMP64rr:    return X86::CMP32rr;
  case X86::CMP64ri8:   return X86::CMP32ri8;
  case X86::CMP64ri32:  return X86::CMP32ri;
  case X86::CMP64rm:    return X86::CMP32rm;
  case X86::CMPXCHG64rr:return X86::CMPXCHG32rr;
  case X86::CRC32r64r8: return X86::CRC32r32r8;
  case X86::CRC32r64r64:return X86::CRC32r32r32;
  case X86::CRC32r64m64:return X86::CRC32r32m32;
  case X86::CVTSD2SI64rr_Int: return X86::CVTSD2SIrr_Int;
  case X86::CVTSD2SI64rm_Int: return X86::CVTSD2SIrm_Int;
  case X86::CVTSS2SI64rr_Int: return X86::CVTSS2SIrr_Int;
  case X86::CVTSS2SI64rm_Int: return X86::CVTSS2SIrm_Int;
  case X86::CVTTSD2SI64rr:    return X86::CVTTSD2SIrr;
  case X86::CVTTSD2SI64rm:    return X86::CVTTSD2SIrm;
  case X86::CVTTSS2SI64rr:    return X86::CVTTSS2SIrr;
  case X86::CVTTSS2SI64rm:    return X86::CVTTSS2SIrm;
  case X86::DEC64r:    return X86::DEC32r;
  case X86::DIV64r:    return X86::DIV32r;
  case X86::IDIV64r:   return X86::IDIV32r;
  case X86::IMUL64r:   return X86::IMUL32r;
  case X86::IMUL64rr:  return X86::IMUL32rr;
  case X86::IMUL64rri8:return X86::IMUL32rri8;
  case X86::IMUL64rri32: return X86::IMUL32rri;
  case X86::IMUL64rm:  return X86::IMUL32rm;
  case X86::IMUL64rmi8:return X86::IMUL32rmi8;
  case X86::IMUL64rmi32: return X86::IMUL32rmi;
  case X86::INC64r:    return X86::INC32r;
  case X86::INVEPT64:  return X86::INVEPT32;
  case X86::INVPCID64: return X86::INVPCID32;
  case X86::INVVPID64: return X86::INVVPID32;
  case X86::JMP64r:    return X86::JMP32r;
  case X86::LAR64rr:   return X86::LAR32rr;
  case X86::LAR64rm:   return X86::LAR32rm;
  case X86::LEA64r:    return X86::LEA32r;
  case X86::LFS64rm:   return X86::LFS32rm;
  case X86::LGS64rm:   return X86::LGS32rm;
  case X86::LSL64rr:   return X86::LSL32rr;
  case X86::LSL64rm:   return X86::LSL32rm;
  case X86::LSS64rm:   return X86::LSS32rm;
  case X86::LZCNT64rr: return X86::LZCNT32rr;
  case X86::LZCNT64rm: return X86::LZCNT32rm;
  case X86::MOV64ri:   return X86::MOV32ri;
  case X86::MOVBE64rm: return X86::MOVBE32rm;
  case X86::MOV64rr:   return X86::MOV32rr;
  case X86::MMX_MOVD64from64rr: return X86::MMX_MOVD64grr;
  case X86::MOVPQIto64rr: return X86::MOVPDI2DIrr;
  case X86::MOV64rs:   return X86::MOV32rs;
  case X86::MOV64rd:   return X86::MOV32rd;
  case X86::MOV64rc:   return X86::MOV32rc;
  case X86::MOV64ri32: return X86::MOV32ri;
  case X86::MOV64rm:   return X86::MOV32rm;
  case X86::MOVSX64rr8:  return X86::MOVSX32rr8;
  case X86::MOVSX64rm8:  return X86::MOVSX32rm8;
  case X86::MOVSX64rr32: return X86::MOV32rr;
  case X86::MOVSX64rm32: return X86::MOV32rm;
  case X86::MOVSX64rr16: return X86::MOVSX32rr16;
  case X86::MOVSX64rm16: return X86::MOVSX32rm16;
  case X86::MOVZX64rr8:  return X86::MOVZX32rr8;
  case X86::MOVZX64rm8:  return X86::MOVZX32rm8;
  case X86::MOVZX64rr16: return X86::MOVZX32rr16;
  case X86::MOVZX64rm16: return X86::MOVZX32rm16;
  case X86::MUL64r:    return X86::MUL32r;
  case X86::MULX64rr:  return X86::MULX32rr;
  case X86::MULX64rm:  return X86::MULX32rm;
  case X86::NEG64r:    return X86::NEG32r;
  case X86::NOT64r:    return X86::NOT32r;
  case X86::OR64rr:    return X86::OR32rr;
  case X86::OR64ri8:   return X86::OR32ri8;
  case X86::OR64ri32:  return X86::OR32ri;
  case X86::OR64rm:    return X86::OR32rm;
  case X86::PDEP64rr:  return X86::PDEP32rr;
  case X86::PDEP64rm:  return X86::PDEP32rm;
  case X86::PEXT64rr:  return X86::PEXT32rr;
  case X86::PEXT64rm:  return X86::PEXT32rm;
  case X86::POPCNT64rr:return X86::POPCNT32rr;
  case X86::POPCNT64rm:return X86::POPCNT32rm;
  case X86::POP64r:    return X86::POP32r;
  case X86::POP64rmr:  return X86::POP32rmr;
  case X86::PUSH64r:   return X86::PUSH32r;
  case X86::PUSH64rmr: return X86::PUSH32rmr;
  case X86::RCL64r1:   return X86::RCL32r1;
  case X86::RCL64rCL:  return X86::RCL32rCL;
  case X86::RCL64ri:   return X86::RCL32ri;
  case X86::RCR64r1:   return X86::RCR32r1;
  case X86::RCR64rCL:  return X86::RCR32rCL;
  case X86::RCR64ri:   return X86::RCR32ri;
  case X86::RDFSBASE64:return X86::RDFSBASE;
  case X86::RDGSBASE64:return X86::RDGSBASE;
  case X86::RDRAND64r: return X86::RDRAND32r;
  case X86::RDSEED64r: return X86::RDSEED32r;
  case X86::ROL64r1:   return X86::ROL32r1;
  case X86::ROL64rCL:  return X86::ROL32rCL;
  case X86::ROL64ri:   return X86::ROL32ri;
  case X86::ROR64r1:   return X86::ROR32r1;
  case X86::ROR64rCL:  return X86::ROR32rCL;
  case X86::ROR64ri:   return X86::ROR32ri;
  case X86::RORX64ri:  return X86::RORX32ri;
  case X86::SAR64r1:   return X86::SAR32r1;
  case X86::SAR64rCL:  return X86::SAR32rCL;
  case X86::SAR64ri:   return X86::SAR32ri;
  case X86::SARX64rr:  return X86::SARX32rr;
  case X86::SARX64rm:  return X86::SARX32rm;
  case X86::SBB64rr:   return X86::SBB32rr;
  case X86::SBB64ri8:  return X86::SBB32ri8;
  case X86::SBB64ri32: return X86::SBB32ri;
  case X86::SBB64rm:   return X86::SBB32rm;
  case X86::SHLD64rrCL:return X86::SHLD32rrCL;
  case X86::SHLD64rri8:return X86::SHLD32rri8;
  case X86::SHL64r1:   return X86::SHL32r1;
  case X86::SHL64rCL:  return X86::SHL32rCL;
  case X86::SHL64ri:   return X86::SHL32ri;
  case X86::SHLX64rr:  return X86::SHLX32rr;
  case X86::SHLX64rm:  return X86::SHLX32rm;
  case X86::SHRD64rrCL:return X86::SHRD32rrCL;
  case X86::SHRD64rri8:return X86::SHRD32rri8;
  case X86::SHR64r1:   return X86::SHR32r1;
  case X86::SHR64rCL:  return X86::SHR32rCL;
  case X86::SHR64ri:   return X86::SHR32ri;
  case X86::SHRX64rr:  return X86::SHRX32rr;
  case X86::SHRX64rm:  return X86::SHRX32rm;
  case X86::SLDT64r:   return X86::SLDT32r;
  case X86::SMSW64r:   return X86::SMSW32r;
  case X86::STR64r:    return X86::STR32r;
  case X86::SUB64rr:   return X86::SUB32rr;
  case X86::SUB64ri8:  return X86::SUB32ri8;
  case X86::SUB64ri32: return X86::SUB32ri;
  case X86::SUB64rm:   return X86::SUB32rm;
  case X86::T1MSKC64rr:return X86::T1MSKC32rr;
  case X86::T1MSKC64rm:return X86::T1MSKC32rm;
  case X86::TEST64rr:  return X86::TEST32rr;
  case X86::TEST64ri32:return X86::TEST32ri;
  case X86::TEST64mr:  return X86::TEST32mr;
  case X86::TZCNT64rr: return X86::TZCNT32rr;
  case X86::TZCNT64rm: return X86::TZCNT32rm;
  case X86::TZMSK64rr: return X86::TZMSK32rr;
  case X86::TZMSK64rm: return X86::TZMSK32rm;
  case X86::VCVTSD2SI64rr_Int:  return X86::VCVTSD2SIrr_Int;
  case X86::VCVTSD2SI64Zrr_Int: return X86::VCVTSD2SIZrr_Int;
  case X86::VCVTSD2SI64Zrm_Int: return X86::VCVTSD2SIZrm_Int;
  case X86::VCVTSD2SI64rm_Int:  return X86::VCVTSD2SIrm_Int;
  case X86::VCVTSD2USI64Zrr_Int:return X86::VCVTSD2USIZrr_Int;
  case X86::VCVTSD2USI64Zrm_Int:return X86::VCVTSD2USIZrm_Int;
  case X86::VCVTSS2SI64rr_Int:  return X86::VCVTSS2SIrr_Int;
  case X86::VCVTSS2SI64Zrr_Int: return X86::VCVTSS2SIZrr_Int;
  case X86::VCVTSS2SI64Zrm_Int: return X86::VCVTSS2SIZrm_Int;
  case X86::VCVTSS2SI64rm_Int:  return X86::VCVTSS2SIrm_Int;
  case X86::VCVTSS2USI64Zrr_Int:return X86::VCVTSS2USIZrr_Int;
  case X86::VCVTSS2USI64Zrm_Int:return X86::VCVTSS2USIZrm_Int;
  case X86::VCVTTSD2SI64rr:   return X86::VCVTTSD2SIrr;
  case X86::VCVTTSD2SI64Zrr:  return X86::VCVTTSD2SIZrr;
  case X86::VCVTTSD2SI64Zrm:  return X86::VCVTTSD2SIZrm;
  case X86::VCVTTSD2SI64rm:   return X86::VCVTTSD2SIrm;
  case X86::VCVTTSD2USI64Zrr: return X86::VCVTTSD2USIZrr;
  case X86::VCVTTSD2USI64Zrm: return X86::VCVTTSD2USIZrm;
  case X86::VCVTTSS2SI64rr:   return X86::VCVTTSS2SIrr;
  case X86::VCVTTSS2SI64Zrr:  return X86::VCVTTSS2SIZrr;
  case X86::VCVTTSS2SI64Zrm:  return X86::VCVTTSS2SIZrm;
  case X86::VCVTTSS2SI64rm:   return X86::VCVTTSS2SIrm;
  case X86::VCVTTSS2USI64Zrr: return X86::VCVTTSS2USIZrr;
  case X86::VCVTTSS2USI64Zrm: return X86::VCVTTSS2USIZrm;
  case X86::VMOVPQIto64rr:    return X86::VMOVPDI2DIrr;
  case X86::VMOVPQIto64Zrr:   return X86::VMOVPDI2DIZrr;
  case X86::VMREAD64rr:       return X86::VMREAD32rr;
  case X86::VMWRITE64rr:      return X86::VMWRITE32rr;
  case X86::VMWRITE64rm:      return X86::VMWRITE32rm;
  case X86::WRFSBASE64:       return X86::WRFSBASE;
  case X86::WRGSBASE64:       return X86::WRGSBASE;
  case X86::XADD64rr:         return X86::XADD32rr;
  case X86::XCHG64ar:         return X86::XCHG32ar;
  case X86::XCHG64rr:         return X86::XCHG32rr;
  case X86::XCHG64rm:         return X86::XCHG32rm;
  case X86::XOR64rr:          return X86::XOR32rr;
  case X86::XOR64ri8:         return X86::XOR32ri8;
  case X86::XOR64ri32:        return X86::XOR32ri;
  case X86::XOR64rm:          return X86::XOR32rm;
  default:
    return Opcode;
  }
}

static void demoteInst(MCInst &Inst, const MCInstrInfo &InstInfo) {
  Inst.setOpcode(demoteOpcode(Inst.getOpcode()));

  // Demote any 64-bit GPR operands to their 32-bit counterparts.
  const ArrayRef<MCOperandInfo> OpInfo = InstInfo.get(Inst.getOpcode()).operands();
  for (unsigned I = 0, E = Inst.getNumOperands(); I < E; ++I) {
    if (OpInfo[I].OperandType != MCOI::OPERAND_REGISTER)
      continue;
    MCRegister Reg = Inst.getOperand(I).getReg();
    if (Reg == X86::NoRegister)
      continue;
    if (getReg64(Reg) == Reg)
      Inst.getOperand(I).setReg(getReg32(Reg));
  }
}

//===----------------------------------------------------------------------===//
// Load/store rewriting
//===----------------------------------------------------------------------===//

void X86::X86MCLFIRewriter::rewriteLoadStore(const MCInst &Inst, MCStreamer &Out,
                                             const MCSubtargetInfo &STI,
                                             bool EmitPrefixes) {
  unsigned Op = Inst.getOpcode();

  // For a register-destination mov-from-memory, the destination register is
  // dead before the load, so it can serve as the scratch register.
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

  MCRegister ScratchReg = ElideScratchReg ? Inst.getOperand(0).getReg()
                                          : MCRegister(X86::R11D);

  // Probe whether sandboxing will need to emit any aux instructions.
  bool NeedsSandbox =
      emitSandboxMemOps(SandboxedInst, ScratchReg, Out, STI,
                        /*EmitInstructions=*/false);

  // Special-case: AH/BH/CH/DH cannot be combined with a REX prefix in mov8,
  // so when sandboxing forces us to use a memory operand that requires REX
  // (e.g. via R11), rotate the high byte to the low position around the load.
  MCRegister RotateRegister = X86::NoRegister;
  if (NeedsSandbox &&
      (SandboxedInst.getOpcode() == X86::MOV8rm_NOREX ||
       SandboxedInst.getOpcode() == X86::MOV8rm) &&
      isHighReg(SandboxedInst.getOperand(0).getReg())) {
    RotateRegister = SandboxedInst.getOperand(0).getReg();
    SandboxedInst.setOpcode(X86::MOV8rm);
    SandboxedInst.getOperand(0).setReg(
        getX86SubSuperRegister(RotateRegister, 8, false));
  } else if (NeedsSandbox &&
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

  bool BundleLockOpened =
      emitSandboxMemOps(SandboxedInst, ScratchReg, Out, STI,
                        /*EmitInstructions=*/true);
  emitInstruction(SandboxedInst, Out, STI, EmitPrefixes);
  if (BundleLockOpened)
    Out.emitBundleUnlock(STI);

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
// String operation rewriting
//===----------------------------------------------------------------------===//

// Truncate the 64-bit register to its 32 low bits via "movl %eX, %eX".
static void clearHighBits(const MCOperand &Reg, MCStreamer &Out,
                          const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(X86::MOV32rr);
  MCOperand Op = MCOperand::createReg(getReg32(Reg.getReg()));
  Mov.addOperand(Op);
  Mov.addOperand(Op);
  Out.emitInstruction(Mov, STI);
}

// Force the implicit %rsi/%rdi register used by string ops to a sandbox-valid
// pointer (movl %eX,%eX; leaq (%r14,%rX),%rX).
static void fixupStringOpReg(const MCOperand &Op, MCStreamer &Out,
                             const MCSubtargetInfo &STI) {
  clearHighBits(Op, Out, STI);

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

void X86::X86MCLFIRewriter::rewriteStringOperation(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI,
    bool EmitPrefixes) {
  bool SkipLoads = hasNoLFILoads(STI);
  bool SkipStores = hasNoLFIStores(STI);

  Out.emitBundleLock(/*AlignToEnd=*/false, STI);

  switch (Inst.getOpcode()) {
  case X86::CMPSB:
  case X86::CMPSW:
  case X86::CMPSL:
  case X86::CMPSQ:
    // Both operands are loads.
    if (!SkipLoads) {
      fixupStringOpReg(Inst.getOperand(0), Out, STI);
      fixupStringOpReg(Inst.getOperand(1), Out, STI);
    }
    break;
  case X86::MOVSB:
  case X86::MOVSW:
  case X86::MOVSL:
  case X86::MOVSQ:
    if (!SkipStores)
      fixupStringOpReg(Inst.getOperand(0), Out, STI);
    if (!SkipLoads)
      fixupStringOpReg(Inst.getOperand(1), Out, STI);
    break;
  case X86::STOSB:
  case X86::STOSW:
  case X86::STOSL:
  case X86::STOSQ:
    if (!SkipStores)
      fixupStringOpReg(Inst.getOperand(0), Out, STI);
    break;
  }

  emitInstruction(Inst, Out, STI, EmitPrefixes);
  Out.emitBundleUnlock(STI);
}

//===----------------------------------------------------------------------===//
// Stack-pointer modification rewriting
//===----------------------------------------------------------------------===//

// Restore %rsp to a sandbox-valid pointer with leaq (%rsp,%r14),%rsp. This
// avoids touching flags, unlike addq %r14, %rsp.
static void emitStackFixup(MCRegister StackReg, MCStreamer &Out,
                           const MCSubtargetInfo &STI) {
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

void X86::X86MCLFIRewriter::rewriteStackModification(MCRegister StackReg,
                                                     const MCInst &Inst,
                                                     MCStreamer &Out,
                                                     const MCSubtargetInfo &STI,
                                                     bool EmitPrefixes) {
  // In jumps-only mode the stack pointer does not need to be sandbox-valid.
  bool JumpsOnly = hasNoLFILoads(STI) && hasNoLFIStores(STI);
  if (JumpsOnly) {
    emitInstruction(Inst, Out, STI, EmitPrefixes);
    return;
  }

  if (Inst.getOpcode() == X86::POP64r) {
    // Transform "pop %rsp" into:
    //   pop %r11
    //   .bundle_lock
    //   movl %r11d, %esp
    //   leaq (%rsp,%r14), %rsp
    //   .bundle_unlock
    MCInst PopR11;
    PopR11.setOpcode(X86::POP64r);
    PopR11.addOperand(MCOperand::createReg(LFIScratchReg));
    Out.emitInstruction(PopR11, STI);

    Out.emitBundleLock(/*AlignToEnd=*/false, STI);

    MCInst MovR11ToESP;
    MovR11ToESP.setOpcode(X86::MOV32rr);
    MovR11ToESP.addOperand(MCOperand::createReg(getReg32(StackReg)));
    MovR11ToESP.addOperand(MCOperand::createReg(X86::R11D));
    Out.emitInstruction(MovR11ToESP, STI);

    emitStackFixup(StackReg, Out, STI);

    Out.emitBundleUnlock(STI);
    return;
  }

  // For other %rsp modifications, demote the instruction to a 32-bit form so
  // the high 32 bits of %rsp are zeroed, then restore the sandbox base with a
  // lea fixup.
  MCInst SandboxedInst(Inst);
  demoteInst(SandboxedInst, *InstInfo);

  bool BundleLockOpened =
      emitSandboxMemOps(SandboxedInst, X86::R11D, Out, STI,
                        /*EmitInstructions=*/true);

  Out.emitBundleLock(/*AlignToEnd=*/false, STI);

  emitInstruction(SandboxedInst, Out, STI, EmitPrefixes);
  if (BundleLockOpened)
    Out.emitBundleUnlock(STI);
  emitStackFixup(StackReg, Out, STI);

  Out.emitBundleUnlock(STI);
}

//===----------------------------------------------------------------------===//
// Top-level dispatch
//===----------------------------------------------------------------------===//

void X86::X86MCLFIRewriter::doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI,
                                          bool EmitPrefixes) {
  // Stash prefixes for emission with the next non-prefix instruction.
  if (isPrefix(Inst)) {
    Prefixes.push_back(Inst);
    return;
  }

  if (mayModifyRegister(Inst, LFIBaseReg) ||
      mayModifyRegister(Inst, LFITPReg))
    return error(Inst, "illegal modification of reserved LFI register");

  if (isSyscall(Inst))
    return rewriteSyscall(Inst, Out, STI);

  if (isDirectCall(Inst))
    return rewriteDirectCall(Inst, Out, STI);

  if (isReturn(Inst))
    return rewriteReturn(Inst, Out, STI);

  if (isIndirectBranch(Inst) || isCall(Inst))
    return rewriteIndirectBranch(Inst, Out, STI);

  if (isStringOperation(Inst))
    return rewriteStringOperation(Inst, Out, STI, EmitPrefixes);

  if (explicitlyModifiesRegister(Inst, X86::RSP))
    return rewriteStackModification(X86::RSP, Inst, Out, STI, EmitPrefixes);

  if (xchgStackReg(Inst) != X86::NoRegister)
    return rewriteStackModification(X86::RSP, Inst, Out, STI, EmitPrefixes);

  if (isFSAccess(Inst))
    return rewriteFSAccess(Inst, Out, STI);

  // Reject sandbox-incompatible uses of the %gs segment, which is reserved
  // for memory sandboxing.
  for (unsigned I = 0, E = Inst.getNumOperands(); I < E; ++I) {
    if (Inst.getOperand(I).isReg() &&
        Inst.getOperand(I).getReg() == X86::GS)
      return error(Inst, "invalid use of %gs segment register");
  }

  rewriteLoadStore(Inst, Out, STI, EmitPrefixes);
}

bool X86::X86MCLFIRewriter::rewriteInst(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  // The guard prevents rewrite-recursion when we emit instructions from inside
  // the rewriter (such instructions should not be rewritten again).
  if (!Enabled || Guard)
    return false;
  Guard = true;

  doRewriteInst(Inst, Out, STI, /*EmitPrefixes=*/true);

  Guard = false;
  return true;
}
