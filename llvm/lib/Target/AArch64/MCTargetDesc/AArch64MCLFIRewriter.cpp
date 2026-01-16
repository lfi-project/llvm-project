//===- AArch64MCLFIRewriter.cpp ---------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the LFI authors.
//
//===----------------------------------------------------------------------===//
//
// This file implements the AArch64MCLFIRewriter class, the AArch64 specific
// subclass of MCLFIRewriter.
//
//===----------------------------------------------------------------------===//
#include "AArch64MCLFIRewriter.h"

#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIRewriter.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

#define DEBUG_TYPE "lfi"

bool AArch64::AArch64MCLFIRewriter::rewriteInst(const MCInst &Inst,
                                                MCStreamer &Out,
                                                const MCSubtargetInfo &STI) {
  if (Guard)
    return false;
  Guard = true;

  Out.emitInstruction(Inst, STI);

  Guard = false;
  return true;
}
