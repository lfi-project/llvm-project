//===- HLFI.h - High-Level Lightweight Fault Isolation ----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the HLFI pass which implements High-Level Lightweight
// Fault Isolation through LLVM IR instrumentation.
//
// HLFI provides:
// - Forward-edge CFI via an indirect branch table
// - Memory sandboxing via heap masking
// - Backward-edge CFI via safe-stack
//
// Reserved Registers:
// - x27: Sandbox base address (fixed, same for all threads)
// - x25: HLFI context pointer (per-thread context structure)
//
// HLFI Context Layout (pointed to by x25):
//   [x25+0]  - Reserved
//   [x25+8]  - Unsafe stack pointer
//   [x25+16] - CFI table pointer
//
// Runtime Variables:
// - @__hlfi_sandbox_base: Global holding sandbox base (matches x27)
// - @__hlfi_context: TLS pointer to context structure (matches x25)
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_TRANSFORMS_INSTRUMENTATION_HLFI_H
#define LLVM_TRANSFORMS_INSTRUMENTATION_HLFI_H

#include "llvm/IR/PassManager.h"
#include "llvm/Support/Compiler.h"

namespace llvm {
class Module;

struct HLFIOptions {
  bool EnableForwardCFI = true;
  bool EnableHeapMasking = true;
  bool EnableSafeStack = true;
};

/// HLFI (High-Level Lightweight Fault Isolation) pass.
///
/// This pass implements sandboxing through LLVM IR instrumentation:
///
/// 1. Forward-edge CFI: Indirect calls/branches go through an index-based
///    table. Function pointers become indices, validated at call sites.
///    The CFI table is accessed via [x25+16].
///
/// 2. Heap masking: Memory accesses are masked to stay within a 4GB sandbox
///    region. Address = sandbox_base + (ptr & 0xFFFFFFFF).
///    The sandbox base is in x27.
///
/// 3. Safe-stack: Separates stack into safe (return addresses) and unsafe
///    (buffers). Unsafe stack pointer is accessed via [x25+8].
///
class HLFIPass : public PassInfoMixin<HLFIPass> {
  HLFIOptions Options;

public:
  HLFIPass() = default;
  explicit HLFIPass(HLFIOptions Opts) : Options(Opts) {}

  LLVM_ABI PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);

  static bool isRequired() { return true; }
};

} // namespace llvm

#endif // LLVM_TRANSFORMS_INSTRUMENTATION_HLFI_H
