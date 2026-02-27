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
#ifndef LLVM_MC_X86MCLFIREWRITER_H
#define LLVM_MC_X86MCLFIREWRITER_H

#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIRewriter.h"
#include "llvm/MC/MCRegister.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/ADT/SmallVector.h"

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

  /// Accumulated prefix instructions to emit with the next instruction.
  SmallVector<MCInst, 4> Prefixes;

  /// Check if STI has the no-lfi-segue feature (disables GS segment use).
  bool hasSegue(const MCSubtargetInfo &STI) const;

  /// Check if STI is in stores-only mode (no load sandboxing).
  bool hasNoLFILoads(const MCSubtargetInfo &STI) const;

  /// Check if STI is in jumps-only mode (no load/store sandboxing).
  bool hasNoLFIStores(const MCSubtargetInfo &STI) const;

  /// Check if STI has enabled SafeStack.
  bool hasSafeStack(const MCSubtargetInfo &STI) const;

  /// Main dispatch function for instruction rewriting.
  void doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Emit an instruction, optionally with accumulated prefixes.
  void emitInstruction(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Emit sandboxing code for a branch target register.
  void emitSandboxBranchReg(MCRegister Reg, MCStreamer &Out,
                            const MCSubtargetInfo &STI);

  /// Emit an indirect jump through a register.
  void emitIndirectJumpReg(MCRegister Reg, MCStreamer &Out,
                           const MCSubtargetInfo &STI);

  /// Emit an indirect call through a register.
  void emitIndirectCallReg(MCRegister Reg, MCStreamer &Out,
                           const MCSubtargetInfo &STI);

  /// Expand a direct call instruction (align to end of bundle).
  void expandDirectCall(const MCInst &Inst, MCStreamer &Out,
                        const MCSubtargetInfo &STI);

  /// Expand an indirect branch or call instruction.
  void expandIndirectBranch(const MCInst &Inst, MCStreamer &Out,
                            const MCSubtargetInfo &STI);

  /// Expand a return instruction.
  void expandReturn(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI);

  void expandSafeStackReturn(const MCInst &Inst, MCStreamer &Out,
                             const MCSubtargetInfo &STI);

  /// Expand load/store instructions with memory sandboxing.
  void expandLoadStore(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Expand string operations (rep movs, rep stos, etc.).
  void expandStringOperation(const MCInst &Inst, MCStreamer &Out,
                             const MCSubtargetInfo &STI, bool EmitPrefixes);

  /// Expand instructions that explicitly modify the stack pointer.
  void expandStackModification(MCRegister StackReg, const MCInst &Inst,
                               MCStreamer &Out, const MCSubtargetInfo &STI,
                               bool EmitPrefixes);

  void expandSafeStackModification(MCRegister StackReg, const MCInst &Inst,
                               MCStreamer &Out, const MCSubtargetInfo &STI,
                               bool EmitPrefixes);

  /// Expand syscall instruction.
  void expandSyscall(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI);

  /// Expand TLS read (mov %fs:0, %rX).
  void expandTLSRead(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI);

  /// Emit an LFI runtime call (syscall, TLS read, etc.).
  enum LFICallType { LFISyscall, LFITLSRead, LFITLSWrite };
  void emitLFICall(LFICallType CallType, MCStreamer &Out,
                   const MCSubtargetInfo &STI);

  /// Emit sandboxing for memory operands, returning true if bundle lock needed.
  bool emitSandboxMemOps(MCInst &Inst, MCRegister ScratchReg, MCStreamer &Out,
                         const MCSubtargetInfo &STI, bool EmitInstructions);

  /// Emit sandboxing for a single memory operand at MemIdx.
  void emitSandboxMemOp(MCInst &Inst, int MemIdx, MCRegister ScratchReg,
                        MCStreamer &Out, const MCSubtargetInfo &STI);

  /// Prepare a memory operand for sandboxing (handle %fs segment).
  void prepareSandboxMemOp(MCInst &Inst, int MemIdx, MCRegister ScratchReg,
                           MCStreamer &Out, const MCSubtargetInfo &STI);
};

} // namespace X86
} // namespace llvm
#endif
