//===- AArch64MCHLFIRewriter.h ----------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the AArch64MCHLFIRewriter class, the AArch64 specific
// subclass of MCLFIRewriter for HLFI (High-Level LFI).
//
// HLFI uses a simpler rewriting approach than LFI, only handling:
// - System calls (svc #0)
// - TLS accesses (mrs/msr tpidr_el0)
//
// Memory sandboxing and CFI are handled at the LLVM IR level.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_MC_AARCH64MCHLFIREWRITER_H
#define LLVM_MC_AARCH64MCHLFIREWRITER_H

#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIRewriter.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {
class MCContext;
class MCInst;
class MCStreamer;
class MCSubtargetInfo;

namespace AArch64 {

/// AArch64MCHLFIRewriter - Rewrites AArch64 instructions for HLFI sandboxing.
///
/// This class implements minimal instruction rewriting for HLFI:
/// - System calls are redirected through the runtime
/// - TLS accesses use the HLFI context structure
///
/// Reserved registers:
/// - X27: Sandbox base address (fixed, same for all threads)
/// - X25: HLFI context pointer (per-thread)
///
/// HLFI Context Layout (pointed to by X25):
///   [x25+0]  - Reserved
///   [x25+8]  - Unsafe stack pointer
///   [x25+16] - CFI table pointer
///   [x25+24] - Thread pointer (TLS)
///
/// Syscall handler is at [x27-8] (negative offset from sandbox base).
class AArch64MCHLFIRewriter : public MCLFIRewriter {
public:
  AArch64MCHLFIRewriter(MCContext &Ctx, std::unique_ptr<MCRegisterInfo> &&RI,
                        std::unique_ptr<MCInstrInfo> &&II)
      : MCLFIRewriter(Ctx, std::move(RI), std::move(II)) {}

  bool rewriteInst(const MCInst &Inst, MCStreamer &Out,
                   const MCSubtargetInfo &STI) override;

  void onLabel(const MCSymbol *Symbol) override {}

private:
  /// Recursion guard to prevent infinite loops when emitting instructions.
  bool Guard = false;

  // Instruction emission.
  void emitInst(const MCInst &Inst, MCStreamer &Out,
                const MCSubtargetInfo &STI);

  // System instructions.
  void rewriteSyscall(const MCInst &Inst, MCStreamer &Out,
                      const MCSubtargetInfo &STI);
  void rewriteTLSRead(const MCInst &Inst, MCStreamer &Out,
                      const MCSubtargetInfo &STI);
  void rewriteTLSWrite(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI);
};

} // namespace AArch64
} // namespace llvm
#endif
