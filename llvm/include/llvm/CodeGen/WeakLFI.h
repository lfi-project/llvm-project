//===--------------------- llvm/CodeGen/WeakLFI.h ---------------*- C++-*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CODEGEN_WEAKLFI_H
#define LLVM_CODEGEN_WEAKLFI_H

#include "llvm/IR/PassManager.h"

namespace llvm {

class TargetMachine;

class WeakLFIPass : public PassInfoMixin<WeakLFIPass> {
  const TargetMachine *TM;

public:
  explicit WeakLFIPass(const TargetMachine &TM) : TM(&TM) {}
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &FAM);
};

} // namespace llvm

#endif // LLVM_CODEGEN_WEAKLFI_H
