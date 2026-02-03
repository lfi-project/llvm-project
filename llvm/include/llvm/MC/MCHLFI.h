//===- MCHLFI.h - HLFI-specific MC support ----------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares HLFI-specific MC support functions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC_MCHLFI_H
#define LLVM_MC_MCHLFI_H

namespace llvm {
class MCContext;
class MCStreamer;
class Triple;

/// Initialize the MC streamer for HLFI targets.
/// This sets up the HLFI MC rewriter for syscall and TLS rewriting.
void initializeHLFIMCStreamer(MCStreamer &Streamer, MCContext &Ctx,
                               const Triple &TheTriple);

} // namespace llvm

#endif // LLVM_MC_MCHLFI_H
