//===- X86MCLFIRewriter.h ---------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the X86MCLFIRewriter class, the X86 specific
// subclass of MCLFIRewriter.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_X86_MCTARGETDESC_X86MCLFIREWRITER_H
#define LLVM_LIB_TARGET_X86_MCTARGETDESC_X86MCLFIREWRITER_H

#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIRewriter.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {
class MCContext;
class MCStreamer;
class MCSubtargetInfo;

namespace X86 {

class X86MCLFIRewriter : public MCLFIRewriter {
public:
  X86MCLFIRewriter(MCContext &Ctx, std::unique_ptr<MCRegisterInfo> &&RI,
                   std::unique_ptr<MCInstrInfo> &&II)
      : MCLFIRewriter(Ctx, std::move(RI), std::move(II)) {}

  bool rewriteInst(const MCInst &Inst, MCStreamer &Out,
                   const MCSubtargetInfo &STI) override;

private:
  /// Recursion guard to prevent infinite loops when emitting instructions.
  bool Guard = false;

  /// Set once the subtarget feature configuration has been validated.
  bool ConfigChecked = false;

  /// Accumulated prefix instructions (LOCK, REP, etc.) to emit alongside the
  /// next non-prefix instruction.
  SmallVector<MCInst, 2> Prefixes;

  /// Subtarget feature checks.
  bool hasSegue(const MCSubtargetInfo &STI) const;
  bool hasNoLFILoads(const MCSubtargetInfo &STI) const;
  bool hasNoLFIStores(const MCSubtargetInfo &STI) const;

  /// Masks addresses with the r15 mask register instead of relying on a fixed
  /// 4GiB truncation, supporting any power-of-two sandbox size.
  bool hasLargeSandbox(const MCSubtargetInfo &STI) const;

  /// The large-sandbox scheme applied to a sandbox that may be smaller than
  /// 4GiB. Implies the large-sandbox scheme.
  bool hasSmallSandbox(const MCSubtargetInfo &STI) const;

  /// Returns true if the context register is the %gs segment base (instead of
  /// r15).
  bool hasGSContext(const MCSubtargetInfo &STI) const;

  /// Returns true if return instructions are left unrewritten.
  bool hasUseRet(const MCSubtargetInfo &STI) const;

  void doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI, bool EmitPrefixes);

  void emitInstruction(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Emit the mask sequence (andl $-32, %eX; addq %r14, %rX) that turns an
  /// arbitrary register value into a bundle-aligned sandbox address.
  void emitSandboxBranchReg(MCRegister Reg, MCStreamer &Out,
                            const MCSubtargetInfo &STI);

  void rewriteIndirectJumpReg(MCRegister Reg, MCStreamer &Out,
                              const MCSubtargetInfo &STI);
  void rewriteIndirectCallReg(MCRegister Reg, MCStreamer &Out,
                              const MCSubtargetInfo &STI);

  void rewriteIndirectBranch(const MCInst &Inst, MCStreamer &Out,
                             const MCSubtargetInfo &STI);
  void rewriteDirectCall(const MCInst &Inst, MCStreamer &Out,
                         const MCSubtargetInfo &STI);
  void rewriteReturn(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI);

  /// Expand load/store instructions with memory sandboxing.
  void rewriteLoadStore(const MCInst &Inst, MCStreamer &Out,
                        const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Expand string operations (rep movs, rep stos, etc.).
  void rewriteStringOperation(const MCInst &Inst, MCStreamer &Out,
                              const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Expand instructions that explicitly modify the stack pointer.
  void rewriteStackModification(MCRegister StackReg, const MCInst &Inst,
                                MCStreamer &Out, const MCSubtargetInfo &STI,
                                bool EmitPrefixes);

  /// Expand syscall instruction.
  void rewriteSyscall(const MCInst &Inst, MCStreamer &Out,
                      const MCSubtargetInfo &STI);

  /// Returns true if Inst has a memory operand using the %fs segment override.
  bool isFSAccess(const MCInst &Inst);

  /// Rewrite a %fs-segmented memory access into a thread-pointer-relative
  /// access via the context register.
  void rewriteFSAccess(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI);

  /// Apply sandboxing to all memory operands of \p Inst. Returns true if a
  /// bundle lock is still open and needs to be closed by the caller.
  bool emitSandboxMemOps(MCInst &Inst, MCRegister ScratchReg, MCStreamer &Out,
                         const MCSubtargetInfo &STI, bool EmitInstructions);

  /// Apply sandboxing to a single memory operand at index \p MemIdx.
  void emitSandboxMemOp(MCInst &Inst, int MemIdx, MCRegister ScratchReg,
                        MCStreamer &Out, const MCSubtargetInfo &STI);
};

} // namespace X86
} // namespace llvm
#endif // LLVM_LIB_TARGET_X86_MCTARGETDESC_X86MCLFIREWRITER_H
