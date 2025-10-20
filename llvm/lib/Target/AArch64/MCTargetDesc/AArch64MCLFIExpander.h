//===- AArch64MCLFIExpander.h -----------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the Native Client and LFI authors.
//
//===----------------------------------------------------------------------===//
//
// This file declares the AArch64MCLFIExpander class, the AArch64 specific
// subclass of MCLFIExpander.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_MC_AARCH64MCLFIEXPANDER_H
#define LLVM_MC_AARCH64MCLFIEXPANDER_H

#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {
class MCContext;
class MCInst;
class MCStreamer;
class MCSubtargetInfo;

namespace AArch64 {
class AArch64MCLFIExpander : public MCLFIExpander {
public:
  AArch64MCLFIExpander(MCContext &Ctx, std::unique_ptr<MCRegisterInfo> &&RI,
                       std::unique_ptr<MCInstrInfo> &&II)
      : MCLFIExpander(Ctx, std::move(RI), std::move(II)) {}

  bool expandInst(const MCInst &Inst, MCStreamer &Out,
                  const MCSubtargetInfo &STI) override;

private:
  bool Guard = false; // recursion guard
};
} // namespace AArch64
} // namespace llvm
#endif
