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

  /// Push the upcoming call's return address onto the shadow call stack and
  /// return the symbol that emitShadowCallEpilogue must bind after the call.
  MCSymbol *emitShadowCallPrologue(MCStreamer &Out,
                                   const MCSubtargetInfo &STI);

  /// Bind the post-call label and undo the SCS prologue's stack swap. If
  /// \p ReturnsTwice is true, aligns the label to 32 bytes and inserts an
  /// endbr64, so longjmp's indirect jump back to it passes the forward-edge
  /// CFI check.
  void emitShadowCallEpilogue(MCSymbol *RetLabel, MCStreamer &Out,
                              const MCSubtargetInfo &STI,
                              bool ReturnsTwice = false);

  void expandDirectCall(const MCInst &Inst, MCStreamer &Out,
                        const MCSubtargetInfo &STI);
  void expandIndirectBranch(const MCInst &Inst, MCStreamer &Out,
                            const MCSubtargetInfo &STI);
  void expandReturn(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI);
  void expandRDSSP(const MCInst &Inst, MCStreamer &Out,
                   const MCSubtargetInfo &STI);
  void expandINCSSP(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI);
  void expandReturnsTwiceDirectCall(const MCInst &Inst, MCStreamer &Out,
                                    const MCSubtargetInfo &STI);
};

} // namespace X86
} // namespace llvm
#endif // LLVM_LIB_TARGET_X86_MCTARGETDESC_X86MCLFIREWRITER_H
