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

#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIRewriter.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {
class MCContext;
class MCInst;
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

  /// Cached symbol for the lazily-emitted ``_lfi_trap`` weak/COMDAT symbol.
  MCSymbol *LFITrapSymbol = nullptr;

  void doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI);

  void rewriteSyscall(const MCInst &Inst, MCStreamer &Out,
                      const MCSubtargetInfo &STI);

  bool isFSAccess(const MCInst &Inst);
  void rewriteFSAccess(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI);

  MCSymbol *getOrEmitTrapSymbol(MCStreamer &Out, const MCSubtargetInfo &STI);
  void emitCFICheck(MCRegister Reg, MCStreamer &Out,
                    const MCSubtargetInfo &STI);
  void emitSandboxBranchReg(MCRegister Reg, MCStreamer &Out,
                            const MCSubtargetInfo &STI);
  void expandIndirectBranch(const MCInst &Inst, MCStreamer &Out,
                            const MCSubtargetInfo &STI);
  void expandReturnsTwiceDirectCall(const MCInst &Inst, MCStreamer &Out,
                                    const MCSubtargetInfo &STI);

  /// Rewrite an explicit %rsp-modifying instruction by demoting it to its
  /// 32-bit form (writing to %r13d, which clears the upper 32 bits) and
  /// re-folding the sandbox base into %r13.
  void rewriteRSPModify(const MCInst &Inst, MCStreamer &Out,
                        const MCSubtargetInfo &STI);

  /// Workaround for the high-byte + REX encoding restriction. When an
  /// instruction has both a high-byte register operand (AH/BH/CH/DH) and an
  /// %rsp memory operand, a naive substitution to %r13 would require a REX
  /// prefix that cannot coexist with the high-byte register. This routes
  /// the address through %rsp (a legacy register that doesn't need REX).
  void rewriteHighByteRSPMem(const MCInst &Inst, MCStreamer &Out,
                             const MCSubtargetInfo &STI);

  /// Expand RDSSPQ %rX into a direct read of %rsp, which holds the CFS
  /// (shadow stack) pointer in the dual-stack scheme.
  void expandRDSSP(const MCInst &Inst, MCStreamer &Out,
                   const MCSubtargetInfo &STI);

  /// Expand INCSSPQ %rX into an LFI runtime call. The count register is
  /// passed through the runtime-call ctxreg slot; the runtime stub adds
  /// count*8 to %rsp after a bounds check against the CFS top.
  void expandINCSSP(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI);

  void expandPush(const MCInst &Inst, MCStreamer &Out,
                  const MCSubtargetInfo &STI);
  void expandPop(const MCInst &Inst, MCStreamer &Out,
                 const MCSubtargetInfo &STI);
  void expandLeave(const MCInst &Inst, MCStreamer &Out,
                   const MCSubtargetInfo &STI);
};

} // namespace X86
} // namespace llvm
#endif // LLVM_LIB_TARGET_X86_MCTARGETDESC_X86MCLFIREWRITER_H
