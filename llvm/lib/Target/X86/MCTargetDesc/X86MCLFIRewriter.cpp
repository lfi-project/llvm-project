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
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
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
// In large-sandbox mode this register holds the sandbox size mask (2^k - 1).
// Large-sandbox mode implies GS-context mode, which frees r15 by moving the
// context register file into the GS segment base.
static constexpr MCRegister LFIMaskReg = X86::R15;

// Indirect branch targets must be aligned to a multiple of this size.
static constexpr unsigned BundleSize = 32;

// Byte offset into the context register file (pointed to by R15) where the
// thread pointer is stored.
static constexpr int TPOffset = 16;

// Small-sandbox mode: size of the guard regions surrounding the sandbox. Only
// a displacement within (-LFISmallGuardSize, LFISmallGuardSize -
// LFISmallMaxAccess) may be left on a trusted base; anything else is folded
// into the effective address before the mask, where it wraps within the
// sandbox. LFISmallMaxAccess is headroom for the size of the access itself
// (the largest, an xsave area with AMX state, is about 11KiB).
static constexpr int64_t LFISmallGuardSize = 128 * 1024;
static constexpr int64_t LFISmallMaxAccess = 16 * 1024;

//===----------------------------------------------------------------------===//
// Feature checking
//===----------------------------------------------------------------------===//

bool X86::X86MCLFIRewriter::hasSegue(const MCSubtargetInfo &STI) const {
  return !STI.hasFeature(X86::FeatureNoLFISegue);
}

bool X86::X86MCLFIRewriter::hasGSContext(const MCSubtargetInfo &STI) const {
  return STI.hasFeature(X86::FeatureLFIGSContext);
}

bool X86::X86MCLFIRewriter::hasLargeSandbox(const MCSubtargetInfo &STI) const {
  return STI.hasFeature(X86::FeatureLFILargeSandbox);
}

bool X86::X86MCLFIRewriter::hasSmallSandbox(const MCSubtargetInfo &STI) const {
  return STI.hasFeature(X86::FeatureLFISmallSandbox);
}

bool X86::X86MCLFIRewriter::hasNoLFILoads(const MCSubtargetInfo &STI) const {
  return STI.hasFeature(X86::FeatureNoLFILoads);
}

bool X86::X86MCLFIRewriter::hasNoLFIStores(const MCSubtargetInfo &STI) const {
  return STI.hasFeature(X86::FeatureNoLFIStores);
}

bool X86::X86MCLFIRewriter::hasUseRet(const MCSubtargetInfo &STI) const {
  return STI.hasFeature(X86::FeatureLFIUseRet);
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

// Registers that are always known to point inside the sandbox.
static bool isAbsoluteReg(MCRegister Reg) {
  Reg = getReg64(Reg);
  return Reg == LFIBaseReg || Reg == X86::RSP || Reg == X86::RIP;
}

static bool isHighReg(MCRegister Reg) {
  return Reg == X86::AH || Reg == X86::BH || Reg == X86::CH || Reg == X86::DH;
}

// Returns true if the displacement operand \p Offset is small enough to leave
// on a trusted base in small-sandbox mode (see LFISmallGuardSize). A symbolic
// displacement that cannot be evaluated is treated as large.
static bool isSmallDisp(const MCOperand &Offset) {
  int64_t Value;
  if (Offset.isImm())
    Value = Offset.getImm();
  else if (!Offset.isExpr() || !Offset.getExpr()->evaluateAsAbsolute(Value))
    return false;
  return Value > -LFISmallGuardSize &&
         Value < LFISmallGuardSize - LFISmallMaxAccess;
}

// For a pop with a memory destination, the amount by which %rsp is incremented
// before the destination address is computed; 0 for any other instruction.
static int64_t popStackAdjust(unsigned Opcode) {
  switch (Opcode) {
  case X86::POP16rmm:
    return 2;
  case X86::POP32rmm:
    return 4;
  case X86::POP64rmm:
    return 8;
  default:
    return 0;
  }
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

static bool hasNoTrackPrefix(const MCInst &Inst, const MCInstrInfo &InstInfo) {
  return (InstInfo.get(Inst.getOpcode()).TSFlags & X86II::NOTRACK) ||
         (Inst.getFlags() & X86::IP_HAS_NOTRACK);
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
  case X86::REP_MOVSB_32:
  case X86::REP_MOVSW_32:
  case X86::REP_MOVSD_32:
  case X86::REP_MOVSQ_32:
  case X86::REP_MOVSB_64:
  case X86::REP_MOVSW_64:
  case X86::REP_MOVSD_64:
  case X86::REP_MOVSQ_64:
  case X86::REP_STOSB_32:
  case X86::REP_STOSW_32:
  case X86::REP_STOSD_32:
  case X86::REP_STOSQ_32:
  case X86::REP_STOSB_64:
  case X86::REP_STOSW_64:
  case X86::REP_STOSD_64:
  case X86::REP_STOSQ_64:
    return true;
  default:
    return false;
  }
}

// xchg variants that swap a register with the stack pointer, which need the
// same handling as an explicit modification of %rsp.
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
//   andq %r15, %rX               (small-sandbox mode only)
//   addq %r14, %rX
// This zeroes the top 32 bits and bottom log2(BundleSize) bits of the target,
// then fills in the top 32 bits with the sandbox base. Large-sandbox mode uses
// the same fixed-4GiB sequence, since executable code is confined to the low
// 4GiB of the sandbox. Small-sandbox mode cannot assume the sandbox spans 4GiB,
// so the (already bundle-aligned) target is also masked with the sandbox size.
void X86::X86MCLFIRewriter::emitSandboxBranchReg(MCRegister Reg,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  MCRegister Reg32 = RegInfo->getSubReg(Reg, X86::sub_32bit);
  MCRegister Reg64 = getReg64(Reg);

  Out.emitInstruction(MCInstBuilder(X86::AND32ri8)
                          .addReg(Reg32)
                          .addReg(Reg32)
                          .addImm(-static_cast<int>(BundleSize)),
                      STI);

  if (hasSmallSandbox(STI)) {
    Out.emitInstruction(MCInstBuilder(X86::AND64rr)
                            .addReg(Reg64)
                            .addReg(Reg64)
                            .addReg(LFIMaskReg),
                        STI);
  }

  Out.emitInstruction(MCInstBuilder(X86::ADD64rr)
                          .addReg(Reg64)
                          .addReg(Reg64)
                          .addReg(LFIBaseReg),
                      STI);
}

void X86::X86MCLFIRewriter::rewriteIndirectJumpReg(MCRegister Reg,
                                                   MCStreamer &Out,
                                                   const MCSubtargetInfo &STI,
                                                   bool NoTrack) {
  Out.emitBundleLock(/*AlignToEnd=*/false, STI);
  emitSandboxBranchReg(Reg, Out, STI);

  MCInst Jmp = MCInstBuilder(X86::JMP64r).addReg(getReg64(Reg));
  if (NoTrack)
    Jmp.setFlags(Jmp.getFlags() | X86::IP_HAS_NOTRACK);
  Out.emitInstruction(Jmp, STI);

  Out.emitBundleUnlock(STI);
}

void X86::X86MCLFIRewriter::rewriteIndirectCallReg(MCRegister Reg,
                                                   MCStreamer &Out,
                                                   const MCSubtargetInfo &STI,
                                                   bool NoTrack) {
  Out.emitBundleLock(/*AlignToEnd=*/true, STI);
  emitSandboxBranchReg(Reg, Out, STI);

  MCInst Call = MCInstBuilder(X86::CALL64r).addReg(getReg64(Reg));
  if (NoTrack)
    Call.setFlags(Call.getFlags() | X86::IP_HAS_NOTRACK);
  Out.emitInstruction(Call, STI);

  Out.emitBundleUnlock(STI);
}

void X86::X86MCLFIRewriter::rewriteIndirectBranch(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  MCRegister Target;
  if (mayLoad(Inst)) {
    // Indirect jmp/call through memory: load the target into the scratch
    // register first, then dispatch via the register-based rewrite.
    Target = LFIScratchReg;

    doRewriteInst(MCInstBuilder(X86::MOV64rm)
                      .addReg(Target)
                      .addOperand(Inst.getOperand(0))
                      .addOperand(Inst.getOperand(1))
                      .addOperand(Inst.getOperand(2))
                      .addOperand(Inst.getOperand(3))
                      .addOperand(Inst.getOperand(4)),
                  Out, STI, /*EmitPrefixes=*/false);
  } else {
    Target = Inst.getOperand(0).getReg();
  }

  bool NoTrack = hasNoTrackPrefix(Inst, *InstInfo);
  if (isCall(Inst))
    rewriteIndirectCallReg(Target, Out, STI, NoTrack);
  else
    rewriteIndirectJumpReg(Target, Out, STI, NoTrack);
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
  Out.emitInstruction(MCInstBuilder(X86::POP64r).addReg(LFIScratchReg), STI);

  // ret with immediate: pop the additional bytes with addq, re-entering the
  // rewriter so the %rsp modification is sandboxed.
  if (Inst.getOpcode() == X86::RETI32 || Inst.getOpcode() == X86::RETI64) {
    doRewriteInst(MCInstBuilder(X86::ADD64ri32)
                      .addReg(X86::RSP)
                      .addReg(X86::RSP)
                      .addOperand(Inst.getOperand(0)),
                  Out, STI, /*EmitPrefixes=*/false);
  }

  rewriteIndirectJumpReg(LFIScratchReg, Out, STI);
}

//===----------------------------------------------------------------------===//
// Syscall and TLS rewrites
//===----------------------------------------------------------------------===//

// Return true if the instruction reads from Reg.
static bool readsRegister(const MCInst &Inst, const MCInstrDesc &Desc,
                          MCRegister Reg, const MCRegisterInfo &RI) {
  for (unsigned I = Desc.getNumDefs(), E = Inst.getNumOperands(); I < E; ++I) {
    const MCOperand &Op = Inst.getOperand(I);
    if (Op.isReg() && Op.getReg() && RI.regsOverlap(Op.getReg(), Reg))
      return true;
  }
  for (MCPhysReg Use : Desc.implicit_uses())
    if (RI.regsOverlap(Use, Reg))
      return true;
  return false;
}

// Return true if Reg is absent or a 64-bit general-purpose register.
static bool isGR64OrNone(MCRegister Reg) {
  return Reg == X86::NoRegister ||
         getX86MCRegisterClass(X86::GR64RegClassID).contains(Reg);
}

// syscall
// ->
// .bundle_lock
// leaq .Ltmp(%rip), %r11
// jmpq *-8(%r14)
// .Ltmp:
// .bundle_unlock
//
// The runtime call entrypoint is stored at a fixed negative offset from the
// sandbox base. The leaq and jmpq must be a single bundle so the runtime always
// sees %r11 holding the post-syscall return address.
void X86::X86MCLFIRewriter::rewriteSyscall(const MCInst &Inst, MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {
  Out.emitBundleLock(/*AlignToEnd=*/false, STI);

  MCSymbol *Symbol = Out.getContext().createTempSymbol();

  Out.emitInstruction(
      MCInstBuilder(X86::LEA64r)
          .addReg(LFIScratchReg)
          .addReg(X86::RIP)
          .addImm(1)
          .addReg(X86::NoRegister)
          .addExpr(MCSymbolRefExpr::create(Symbol, Out.getContext()))
          .addReg(X86::NoRegister),
      STI);

  Out.emitInstruction(MCInstBuilder(X86::JMP64m)
                          .addReg(LFIBaseReg)
                          .addImm(1)
                          .addReg(X86::NoRegister)
                          .addImm(-8)
                          .addReg(X86::NoRegister),
                      STI);

  Out.emitLabel(Symbol);
  Out.emitBundleUnlock(STI);
}

//===----------------------------------------------------------------------===//
// %fs-segment (TLS) rewriting
//===----------------------------------------------------------------------===//

// Find the index of the memory operand if it has an %fs segment override.
// Returns -1 if there is no memory operand or no %fs override.
static int findFSMemOperand(const MCInst &Inst, const MCInstrInfo &InstInfo) {
  int MemIdx = X86II::getMemoryOperandIdx(InstInfo.get(Inst.getOpcode()));
  if (MemIdx < 0)
    return -1;
  const MCOperand &Seg = Inst.getOperand(MemIdx + X86::AddrSegmentReg);
  if (Seg.isReg() && Seg.getReg() == X86::FS)
    return MemIdx;
  return -1;
}

// Emit the thread-pointer load from the context register file:
//   movq TPOffset(%r15), %Reg      (default, context register is r15)
//   movq %gs:TPOffset, %Reg        (GS-context mode)
static void emitTPLoad(MCRegister Reg, bool GSContext, MCStreamer &Out,
                       const MCSubtargetInfo &STI) {
  Out.emitInstruction(MCInstBuilder(X86::MOV64rm)
                          .addReg(Reg)
                          .addReg(GSContext ? X86::NoRegister : LFITPReg)
                          .addImm(1)
                          .addReg(X86::NoRegister)
                          .addImm(TPOffset)
                          .addReg(GSContext ? LFIBaseSeg : X86::NoRegister),
                      STI);
}

bool X86::X86MCLFIRewriter::isFSAccess(const MCInst &Inst) {
  return (mayLoad(Inst) || mayStore(Inst)) &&
         findFSMemOperand(Inst, *InstInfo) >= 0;
}

// Rewrite %fs-segment memory accesses to use the virtual thread pointer stored
// at TPOffset in the context register file. In the default configuration the
// context register is r15 and the thread pointer lives at TPOffset(%r15); in
// GS-context mode each TPOffset(%r15) below is instead %gs:TPOffset. Example
// rewrites (default configuration):
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

  MCRegister BaseReg = Inst.getOperand(MemIdx + X86::AddrBaseReg).getReg();
  MCRegister IndexReg = Inst.getOperand(MemIdx + X86::AddrIndexReg).getReg();
  bool HasBase = BaseReg != X86::NoRegister;
  bool HasIndex = IndexReg != X86::NoRegister;
  bool HasDisp = !Inst.getOperand(MemIdx + X86::AddrDisp).isImm() ||
                 Inst.getOperand(MemIdx + X86::AddrDisp).getImm() != 0;

  bool GSContext = hasGSContext(STI);

  // %fs:0 -> TPOffset(%r15)   (or %gs:TPOffset in GS-context mode)
  if (!HasBase && !HasIndex && !HasDisp) {
    MCInst Modified(Inst);
    Modified.getOperand(MemIdx + X86::AddrBaseReg)
        .setReg(GSContext ? X86::NoRegister : LFITPReg);
    Modified.getOperand(MemIdx + X86::AddrDisp).setImm(TPOffset);
    Modified.getOperand(MemIdx + X86::AddrSegmentReg)
        .setReg(GSContext ? LFIBaseSeg : X86::NoRegister);
    return Out.emitInstruction(Modified, STI);
  }

  if (!isGR64OrNone(BaseReg) || !isGR64OrNone(IndexReg) ||
      BaseReg == X86::RSP || BaseReg == X86::RIP)
    return error(Inst, "unsupported addressing mode for %fs access");

  const MCInstrDesc &Desc = InstInfo->get(Inst.getOpcode());

  // Reuse operand 0 as the TP temporary when the instruction writes it without
  // also reading it, otherwise use %r11.
  MCRegister TPDest = LFIScratchReg;
  if (MemIdx > 0 && Inst.getOperand(0).isReg()) {
    MCRegister DestReg = Inst.getOperand(0).getReg();
    if (Desc.getNumDefs() > 0 &&
        getX86MCRegisterClass(X86::GR64RegClassID).contains(DestReg) &&
        !readsRegister(Inst, Desc, DestReg, *RegInfo))
      TPDest = DestReg;
  }

  if (TPDest == LFIScratchReg &&
      readsRegister(Inst, Desc, LFIScratchReg, *RegInfo))
    return error(Inst, "%fs access reads reserved register %r11");

  emitTPLoad(TPDest, GSContext, Out, STI);

  // Both slots occupied: the compute base via lea. For example:
  //
  // movq %fs:8(%rdi,%rsi,2), %rax
  // ->
  // movq 16(%r15), %rax
  // leaq (%rax,%rdi), %rax
  // movq 8(%rax,%rsi,2), %rax
  if (HasBase && HasIndex) {
    Out.emitInstruction(MCInstBuilder(X86::LEA64r)
                            .addReg(TPDest)
                            .addReg(TPDest)
                            .addImm(1)
                            .addReg(BaseReg)
                            .addImm(0)
                            .addReg(X86::NoRegister),
                        STI);
  }

  MCInst Modified(Inst);
  Modified.getOperand(MemIdx + X86::AddrBaseReg).setReg(TPDest);
  if (HasBase && !HasIndex)
    Modified.getOperand(MemIdx + X86::AddrIndexReg).setReg(BaseReg);
  Modified.getOperand(MemIdx + X86::AddrSegmentReg).setReg(X86::NoRegister);
  // Re-enter the rewriter so the non-FS access is memory-sandboxed.
  doRewriteInst(Modified, Out, STI, /*EmitPrefixes=*/false);
}

//===----------------------------------------------------------------------===//
// Memory sandboxing
//===----------------------------------------------------------------------===//

// Returns true if emitSandboxMemOp would emit any auxiliary instructions for
// the memory operand at \p Idx, rather than just rewriting the addressing mode
// in place. Must agree with the early-outs taken by emitSandboxMemOp.
static bool willEmitSandboxInsts(const MCInst &Inst, int Idx,
                                 bool SmallSandbox) {
  const MCOperand &Base = Inst.getOperand(Idx);
  const MCOperand &Scale = Inst.getOperand(Idx + 1);
  const MCOperand &Index = Inst.getOperand(Idx + 2);
  const MCOperand &Offset = Inst.getOperand(Idx + 3);

  // In small-sandbox mode a large displacement cannot be left on a trusted
  // base (except %rip), so the operand takes the masked path instead.
  bool DispOK = !SmallSandbox || isSmallDisp(Offset);

  if (isAbsoluteReg(Base.getReg()) && Index.getReg() == X86::NoRegister)
    return !DispOK && getReg64(Base.getReg()) != X86::RIP;
  if (Base.getReg() == X86::NoRegister && isAbsoluteReg(Index.getReg()) &&
      Scale.isImm() && Scale.getImm() == 1)
    return !DispOK;
  return true;
}

void X86::X86MCLFIRewriter::emitSandboxMemOp(MCInst &Inst, int MemIdx,
                                             MCRegister ScratchReg,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI) {
  bool SkipLoads = hasNoLFILoads(STI);
  bool SkipStores = hasNoLFIStores(STI);

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

  // LEA-style instructions have a memory operand but do not access memory:
  // the addressing mode must stay valid, but needs no sandbox prefix.
  bool NoMemAccess = !mayLoad(Inst) && !mayStore(Inst);

  // Small-sandbox mode: a displacement left on a trusted base must keep the
  // access within the guard region (see LFISmallGuardSize); anything larger
  // falls through to the masked path below. %rip-relative operands are exempt,
  // since the linker resolves them within the image.
  bool DispOK = !hasSmallSandbox(STI) || isSmallDisp(Offset);

  // Case 1: base is an absolute register and there is no index.
  if (isAbsoluteReg(Base.getReg()) && Index.getReg() == X86::NoRegister &&
      (DispOK || getReg64(Base.getReg()) == X86::RIP)) {
    Base.setReg(getReg64(Base.getReg()));
    return;
  }

  // Case 2: only an index, with scale 1, and the index is absolute.
  if (Base.getReg() == X86::NoRegister && isAbsoluteReg(Index.getReg()) &&
      Scale.isImm() && Scale.getImm() == 1 && DispOK) {
    Base.setReg(getReg64(Index.getReg()));
    Index.setReg(X86::NoRegister);
    return;
  }

  // Case 3: pure absolute address (no base, no index). Use %r14 as the base,
  // unless the displacement is too large for the small sandbox.
  if (Base.getReg() == X86::NoRegister && Index.getReg() == X86::NoRegister) {
    if (NoMemAccess)
      return;
    if (DispOK) {
      Base.setReg(LFIBaseReg);
      return;
    }
  }

  // Case 4: prefer the %gs segment if available. This is the cheap path: no
  // scratch-register fixup, just 32-bit registers and a segment override.
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

  // Large-sandbox mode: compute the full effective address into the scratch
  // register, mask it to the sandbox size with %r15, and access via
  // (%r14, scratch).
  if (hasLargeSandbox(STI)) {
    // The mask is emitted before the memory access, so the cheaper
    // flag-clobbering andq is only safe when the access overwrites EFLAGS
    // without also reading them (e.g. add, but not adc/sbb), or when a late
    // pass has proved EFLAGS is dead across it. Otherwise pext preserves them.
    const MCInstrDesc &Desc = InstInfo->get(Inst.getOpcode());
    bool WritesFlags = Desc.hasImplicitDefOfPhysReg(X86::EFLAGS);
    bool ReadsFlags = Desc.hasImplicitUseOfPhysReg(X86::EFLAGS);
    bool FlagsDead = (Inst.getFlags() & X86::IP_LFI_FLAGS_DEAD) != 0;
    bool PreserveFlags = ReadsFlags || (!WritesFlags && !FlagsDead);

    // A bare base register can be masked directly; otherwise compute the
    // effective address with a 64-bit lea first.
    MCRegister AddrReg;
    bool OnlyBase = Base.getReg() != X86::NoRegister &&
                    Index.getReg() == X86::NoRegister && Offset.isImm() &&
                    Offset.getImm() == 0;
    if (OnlyBase) {
      AddrReg = getReg64(Base.getReg());
    } else {
      MCInst Lea = MCInstBuilder(X86::LEA64r)
                       .addReg(ScratchReg64)
                       .addReg(getReg64(Base.getReg()))
                       .addOperand(Scale)
                       .addReg(getReg64(Index.getReg()))
                       .addOperand(Offset)
                       .addReg(X86::NoRegister);
      // Canonicalize "(,%index,1)" into "(%index)".
      if (Base.getReg() == X86::NoRegister && Scale.isImm() &&
          Scale.getImm() == 1) {
        Lea.getOperand(1).setReg(getReg64(Index.getReg()));
        Lea.getOperand(3).setReg(X86::NoRegister);
      }
      // A pop with a memory destination computes its address after %rsp has
      // been incremented, but the lea runs before the pop, so compensate.
      int64_t PopAdj = popStackAdjust(Inst.getOpcode());
      if (PopAdj != 0 && getReg64(Base.getReg()) == X86::RSP) {
        MCOperand &LeaOffset = Lea.getOperand(4);
        if (LeaOffset.isImm())
          LeaOffset.setImm(LeaOffset.getImm() + PopAdj);
        else
          LeaOffset = MCOperand::createExpr(MCBinaryExpr::createAdd(
              LeaOffset.getExpr(),
              MCConstantExpr::create(PopAdj, Out.getContext()),
              Out.getContext()));
      }
      Out.emitInstruction(Lea, STI);
      AddrReg = ScratchReg64;
    }

    if (PreserveFlags) {
      // pext %r15, %AddrReg, %scratch
      Out.emitInstruction(MCInstBuilder(X86::PEXT64rr)
                              .addReg(ScratchReg64)
                              .addReg(AddrReg)
                              .addReg(LFIMaskReg),
                          STI);
    } else {
      if (AddrReg != ScratchReg64) {
        Out.emitInstruction(
            MCInstBuilder(X86::MOV64rr).addReg(ScratchReg64).addReg(AddrReg),
            STI);
      }
      // andq %r15, %scratch
      Out.emitInstruction(MCInstBuilder(X86::AND64rr)
                              .addReg(ScratchReg64)
                              .addReg(ScratchReg64)
                              .addReg(LFIMaskReg),
                          STI);
    }

    Base.setReg(LFIBaseReg);
    Scale.setImm(1);
    Index.setReg(ScratchReg64);
    if (Offset.isImm()) {
      Offset.setImm(0);
    } else {
      Inst.erase(Inst.begin() + MemIdx + 3);
      Inst.insert(Inst.begin() + MemIdx + 3, MCOperand::createImm(0));
    }
    return;
  }

  // Case 5a: absolute base + non-absolute index, zero offset. Fold the index
  // into the scratch register and use the original (absolute) base.
  if (isAbsoluteReg(Base.getReg()) && !isAbsoluteReg(Index.getReg()) &&
      Offset.isImm() && Offset.getImm() == 0) {
    Out.emitInstruction(MCInstBuilder(X86::MOV32rr)
                            .addReg(ScratchReg32)
                            .addReg(getReg32(Index.getReg())),
                        STI);

    Base.setReg(getReg64(Base.getReg()));
    Index.setReg(ScratchReg64);
    return;
  }

  // Case 5b: non-absolute base, no index, zero offset. Use the base as the
  // sandboxed offset and %r14 as the base.
  if (Index.getReg() == X86::NoRegister && Base.getReg() != X86::NoRegister &&
      Offset.isImm() && Offset.getImm() == 0) {
    Out.emitInstruction(MCInstBuilder(X86::MOV32rr)
                            .addReg(ScratchReg32)
                            .addReg(getReg32(Base.getReg())),
                        STI);

    Index.setReg(ScratchReg64);
    Base.setReg(LFIBaseReg);
    return;
  }

  // General case: compute the effective address with a 32-bit lea into the
  // scratch register, then use it as the index with %r14 as the base.
  MCRegister BaseReg64 = getReg64(Base.getReg());
  MCRegister IndexReg64 = getReg64(Index.getReg());

  MCInst Lea = MCInstBuilder(X86::LEA64_32r)
                   .addReg(ScratchReg32)
                   .addReg(BaseReg64)
                   .addOperand(Scale)
                   .addReg(IndexReg64)
                   .addOperand(Offset)
                   .addOperand(Segment);

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
  bool SmallSandbox = hasSmallSandbox(STI);

  for (int I = 0, E = Inst.getNumOperands(); I < E; ++I) {
    if (OpInfo[I].OperandType != MCOI::OPERAND_MEMORY)
      continue;

    if (!BundleLockOpened && willEmitSandboxInsts(Inst, I, SmallSandbox)) {
      if (!EmitInstructions)
        return true;

      // Without segment-based sandboxing the address computation and the
      // memory access must live in the same bundle, so that the verifier can
      // prove the access uses the freshly-computed sandboxed address.
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

  // AH/BH/CH/DH cannot be combined with the REX prefix that a sandboxed
  // memory operand requires, so rotate the high byte to the low position
  // around the access.
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
    Out.emitInstruction(MCInstBuilder(X86::ROR64ri)
                            .addReg(getReg64(RotateRegister))
                            .addReg(getReg64(RotateRegister))
                            .addImm(8),
                        STI);
  }

  bool BundleLockOpened =
      emitSandboxMemOps(SandboxedInst, ScratchReg, Out, STI,
                        /*EmitInstructions=*/true);
  emitInstruction(SandboxedInst, Out, STI, EmitPrefixes);
  if (BundleLockOpened)
    Out.emitBundleUnlock(STI);

  if (RotateRegister != X86::NoRegister) {
    Out.emitInstruction(MCInstBuilder(X86::ROL64ri)
                            .addReg(getReg64(RotateRegister))
                            .addReg(getReg64(RotateRegister))
                            .addImm(8),
                        STI);
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
// pointer. The fixed-4GiB scheme truncates it to 32 bits; large-sandbox mode
// masks it to the sandbox size with %r15, using pext so the flags survive (a
// string instruction does not necessarily set them).
static void fixupStringOpReg(const MCOperand &Op, bool Large, MCStreamer &Out,
                             const MCSubtargetInfo &STI) {
  if (Large) {
    MCRegister Reg = getReg64(Op.getReg());
    Out.emitInstruction(
        MCInstBuilder(X86::PEXT64rr).addReg(Reg).addReg(Reg).addReg(LFIMaskReg),
        STI);
  } else {
    clearHighBits(Op, Out, STI);
  }

  Out.emitInstruction(MCInstBuilder(X86::LEA64r)
                          .addReg(getReg64(Op.getReg()))
                          .addReg(LFIBaseReg)
                          .addImm(1)
                          .addReg(getReg64(Op.getReg()))
                          .addImm(0)
                          .addReg(X86::NoRegister),
                      STI);
}

void X86::X86MCLFIRewriter::rewriteStringOperation(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI,
    bool EmitPrefixes) {
  bool SkipLoads = hasNoLFILoads(STI);
  bool SkipStores = hasNoLFIStores(STI);
  bool Large = hasLargeSandbox(STI);

  Out.emitBundleLock(/*AlignToEnd=*/false, STI);

  switch (Inst.getOpcode()) {
  case X86::CMPSB:
  case X86::CMPSW:
  case X86::CMPSL:
  case X86::CMPSQ:
    // Both operands are loads.
    if (!SkipLoads) {
      fixupStringOpReg(Inst.getOperand(0), Large, Out, STI);
      fixupStringOpReg(Inst.getOperand(1), Large, Out, STI);
    }
    break;
  case X86::MOVSB:
  case X86::MOVSW:
  case X86::MOVSL:
  case X86::MOVSQ:
    if (!SkipStores)
      fixupStringOpReg(Inst.getOperand(0), Large, Out, STI);
    if (!SkipLoads)
      fixupStringOpReg(Inst.getOperand(1), Large, Out, STI);
    break;
  case X86::REP_MOVSB_32:
  case X86::REP_MOVSW_32:
  case X86::REP_MOVSD_32:
  case X86::REP_MOVSQ_32:
  case X86::REP_MOVSB_64:
  case X86::REP_MOVSW_64:
  case X86::REP_MOVSD_64:
  case X86::REP_MOVSQ_64:
    if (!SkipStores)
      fixupStringOpReg(MCOperand::createReg(X86::RDI), Large, Out, STI);
    if (!SkipLoads)
      fixupStringOpReg(MCOperand::createReg(X86::RSI), Large, Out, STI);
    break;
  case X86::STOSB:
  case X86::STOSW:
  case X86::STOSL:
  case X86::STOSQ:
    if (!SkipStores)
      fixupStringOpReg(Inst.getOperand(0), Large, Out, STI);
    break;
  case X86::REP_STOSB_32:
  case X86::REP_STOSW_32:
  case X86::REP_STOSD_32:
  case X86::REP_STOSQ_32:
  case X86::REP_STOSB_64:
  case X86::REP_STOSW_64:
  case X86::REP_STOSD_64:
  case X86::REP_STOSQ_64:
    if (!SkipStores)
      fixupStringOpReg(MCOperand::createReg(X86::RDI), Large, Out, STI);
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
  Out.emitInstruction(MCInstBuilder(X86::LEA64r)
                          .addReg(StackReg)
                          .addReg(StackReg)
                          .addImm(1)
                          .addReg(LFIBaseReg)
                          .addImm(0)
                          .addReg(X86::NoRegister),
                      STI);
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
    // Transform "pop %rsp" into a pop into the scratch register followed by a
    // masked rebuild of %rsp:
    //   pop %r11
    //   .bundle_lock
    //   movl %r11d, %esp           (fixed-4GiB)   OR   pext %r15, %r11, %r11
    //   leaq (%rsp,%r14), %rsp     (fixed-4GiB)   OR   leaq (%r14,%r11), %rsp
    //   .bundle_unlock
    // In large-sandbox mode pext preserves the flags, which "pop" does not set.
    Out.emitInstruction(MCInstBuilder(X86::POP64r).addReg(LFIScratchReg), STI);

    Out.emitBundleLock(/*AlignToEnd=*/false, STI);

    if (hasLargeSandbox(STI)) {
      Out.emitInstruction(MCInstBuilder(X86::PEXT64rr)
                              .addReg(LFIScratchReg)
                              .addReg(LFIScratchReg)
                              .addReg(LFIMaskReg),
                          STI);

      Out.emitInstruction(MCInstBuilder(X86::LEA64r)
                              .addReg(StackReg)
                              .addReg(LFIBaseReg)
                              .addImm(1)
                              .addReg(LFIScratchReg)
                              .addImm(0)
                              .addReg(X86::NoRegister),
                          STI);
    } else {
      Out.emitInstruction(MCInstBuilder(X86::MOV32rr)
                              .addReg(getReg32(StackReg))
                              .addReg(X86::R11D),
                          STI);

      emitStackFixup(StackReg, Out, STI);
    }

    Out.emitBundleUnlock(STI);
    return;
  }

  // For other %rsp modifications: the fixed-4GiB scheme demotes the
  // instruction to a 32-bit form to zero the high 32 bits of %rsp;
  // large-sandbox mode masks %rsp afterwards instead. Either way the sandbox
  // base is restored with a lea fixup.
  bool Large = hasLargeSandbox(STI);

  MCInst SandboxedInst(Inst);
  if (!Large)
    demoteInst(SandboxedInst, *InstInfo);

  // emitSandboxMemOps may open a bundle lock itself, and the bundler does not
  // allow nesting. The %rsp modification and its fixup belong in that same
  // bundle, so only open one when it did not, and close exactly one.
  bool BundleLockOpened =
      emitSandboxMemOps(SandboxedInst, X86::R11D, Out, STI,
                        /*EmitInstructions=*/true);
  if (!BundleLockOpened)
    Out.emitBundleLock(/*AlignToEnd=*/false, STI);

  emitInstruction(SandboxedInst, Out, STI, EmitPrefixes);

  if (Large) {
    // pext %r15, %rsp, %rsp masks %rsp without clobbering the flags, which
    // the modifying instruction may have set.
    Out.emitInstruction(MCInstBuilder(X86::PEXT64rr)
                            .addReg(StackReg)
                            .addReg(StackReg)
                            .addReg(LFIMaskReg),
                        STI);
  }

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

  // r14 is always reserved. r15 is reserved as the context register, or as the
  // sandbox mask register in large-sandbox mode; it is general-purpose only in
  // GS-context mode without large-sandbox.
  bool R15Reserved = !hasGSContext(STI) || hasLargeSandbox(STI);
  if (mayModifyRegister(Inst, LFIBaseReg) ||
      (R15Reserved && mayModifyRegister(Inst, LFITPReg)))
    return error(Inst, "illegal modification of reserved LFI register");

  if (isSyscall(Inst))
    return rewriteSyscall(Inst, Out, STI);

  if (isDirectCall(Inst))
    return rewriteDirectCall(Inst, Out, STI);

  if (isReturn(Inst)) {
    // With lfi-use-ret, returns are trusted and left unrewritten.
    if (hasUseRet(STI))
      return emitInstruction(Inst, Out, STI, EmitPrefixes);
    return rewriteReturn(Inst, Out, STI);
  }

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

  // The %gs segment is reserved by the LFI ABI.
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

  // GS-context mode repurposes the %gs segment base as the context register,
  // so %gs can no longer hold the sandbox base for Segue. Large-sandbox mode
  // requires GS-context mode, since it repurposes r15 as the mask register.
  if (!ConfigChecked) {
    ConfigChecked = true;
    if (hasGSContext(STI) && hasSegue(STI))
      error(Inst, "lfi-gs-context requires Segue to be disabled "
                  "(add +no-lfi-segue)");
    if (hasLargeSandbox(STI) && !hasGSContext(STI))
      error(Inst, "lfi-large-sandbox requires GS-context mode "
                  "(add +lfi-gs-context)");
  }

  // While a `.lfi_flags_dead` window is active, tag each instruction with
  // IP_LFI_FLAGS_DEAD so the large-sandbox memory rewrite may mask with andq
  // instead of pext. The window ends at the first instruction that redefines
  // the flags. Prefixes neither carry the annotation nor end the window.
  if (FlagsDeadActive && !isPrefix(Inst)) {
    MCInst Annotated(Inst);
    Annotated.setFlags(Annotated.getFlags() | X86::IP_LFI_FLAGS_DEAD);
    doRewriteInst(Annotated, Out, STI, /*EmitPrefixes=*/true);
    if (InstInfo->get(Inst.getOpcode()).hasImplicitDefOfPhysReg(X86::EFLAGS))
      FlagsDeadActive = false;
  } else {
    doRewriteInst(Inst, Out, STI, /*EmitPrefixes=*/true);
  }

  Guard = false;
  return true;
}
