//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file declares the X86MCLFIRewriter class, the X86 specific
/// subclass of MCLFIRewriter.
///
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

  /// Set once the subtarget feature configuration has been validated, so the
  /// check (and any diagnostic) is only performed a single time.
  bool ConfigChecked = false;

  /// Accumulated prefix instructions (LOCK, REP, etc.) to emit alongside the
  /// next non-prefix instruction.
  SmallVector<MCInst, 2> Prefixes;

  /// Subtarget feature checks.
  bool hasSegue(const MCSubtargetInfo &STI) const;
  bool hasNoLFILoads(const MCSubtargetInfo &STI) const;
  bool hasNoLFIStores(const MCSubtargetInfo &STI) const;

  /// Returns true if the context register is the %gs segment base (instead of
  /// r15). Requires Segue to be disabled.
  bool hasGSContext(const MCSubtargetInfo &STI) const;

  /// Main dispatch function for instruction rewriting.
  void doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Emit an instruction, optionally flushing the accumulated prefix queue
  /// first.
  void emitInstruction(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Emit the mask sequence (andl $-32, %eX; addq %r14, %rX) that turns
  /// an arbitrary register value into a valid sandbox address aligned to a
  /// bundle boundary.
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
  /// access via the context register: r15 by default, or the %gs segment base
  /// in GS-context mode.
  void rewriteFSAccess(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI);

  /// Apply sandboxing to all memory operands of \p Inst, including handling
  /// of the %fs segment used for TLS. Returns true if a bundle lock is still
  /// open and needs to be closed by the caller (only happens when running
  /// without segment-based sandboxing).
  bool emitSandboxMemOps(MCInst &Inst, MCRegister ScratchReg, MCStreamer &Out,
                         const MCSubtargetInfo &STI, bool EmitInstructions);

  /// Apply sandboxing to a single memory operand at index \p MemIdx.
  void emitSandboxMemOp(MCInst &Inst, int MemIdx, MCRegister ScratchReg,
                        MCStreamer &Out, const MCSubtargetInfo &STI);

  /// Pre-process a memory operand at index \p MemIdx, e.g., rewriting a
  /// %fs-segment access to load the thread pointer first.
  void prepareSandboxMemOp(MCInst &Inst, int MemIdx, MCRegister ScratchReg,
                           MCStreamer &Out, const MCSubtargetInfo &STI);
};

} // namespace X86
} // namespace llvm
#endif // LLVM_LIB_TARGET_X86_MCTARGETDESC_X86MCLFIREWRITER_H
