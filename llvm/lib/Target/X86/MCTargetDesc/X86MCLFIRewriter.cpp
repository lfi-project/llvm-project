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
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

namespace llvm {
namespace X86 {

bool X86MCLFIRewriter::rewriteInst(const MCInst &Inst, MCStreamer &Out,
                                   const MCSubtargetInfo &STI) {
  if (Guard)
    return false;
  Guard = true;

  // TODO: Implement instruction rewriting for LFI sandboxing.
  // For now, emit the instruction unchanged.
  Out.emitInstruction(Inst, STI);

  Guard = false;
  return true;
}

} // namespace X86
} // namespace llvm
