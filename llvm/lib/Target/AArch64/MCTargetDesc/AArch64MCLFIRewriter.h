//===- AArch64MCLFIRewriter.h -----------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the Native Client and LFI authors.
//
//===----------------------------------------------------------------------===//
//
// This file declares the AArch64MCLFIRewriter class, the AArch64 specific
// subclass of MCLFIRewriter.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_MC_AARCH64MCLFIREWRITER_H
#define LLVM_MC_AARCH64MCLFIREWRITER_H

#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIRewriter.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {
class MCContext;
class MCInst;
class MCStreamer;
class MCSubtargetInfo;

namespace AArch64 {
class AArch64MCLFIRewriter : public MCLFIRewriter {
public:
  AArch64MCLFIRewriter(MCContext &Ctx, std::unique_ptr<MCRegisterInfo> &&RI,
                       std::unique_ptr<MCInstrInfo> &&II)
      : MCLFIRewriter(Ctx, std::move(RI), std::move(II)) {}

  bool rewriteInst(const MCInst &Inst, MCStreamer &Out,
                   const MCSubtargetInfo &STI) override;

private:
  bool Guard = false; // recursion guard
};
} // namespace AArch64
} // namespace llvm
#endif
