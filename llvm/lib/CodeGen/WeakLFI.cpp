//===- WeakLFI.cpp - Weak LFI Instrumentation ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/CodeGen/WeakLFI.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Module.h"
#include "llvm/Pass.h"
#include "llvm/InitializePasses.h"
#include "llvm/Transforms/Utils/Local.h"
#include <cassert>

using namespace llvm;

#define DEBUG_TYPE "weak-lfi"

namespace {

class WeakLFI {
  Function &F;

public:
  WeakLFI(Function &F) : F(F) {}

  bool run();
};

class WeakLFILegacyPass : public FunctionPass {
  const TargetMachine *TM = nullptr;

public:
  static char ID;

  WeakLFILegacyPass() : FunctionPass(ID) {
    initializeWeakLFILegacyPassPass(*PassRegistry::getPassRegistry());
  }

  bool runOnFunction(Function &F) override {
    return WeakLFI(F).run();
  }
};

} // end anonymous namespace

static Value *readRegister(IRBuilder<> &IRB, StringRef Name) {
  Module *M = IRB.GetInsertBlock()->getParent()->getParent();
  MDNode *MD =
      MDNode::get(M->getContext(), {MDString::get(M->getContext(), Name)});
  Value *Args[] = {MetadataAsValue::get(M->getContext(), MD)};
  return IRB.CreateIntrinsic(Intrinsic::read_register,
                             IRB.getIntPtrTy(M->getDataLayout()), Args);
}

static Value *guardPtr(IRBuilder<> &B, Value *Base, Value *Ptr) {
  Value *Addr64 = B.CreatePtrToInt(Ptr, B.getInt64Ty());
  Value *Addr32 = B.CreateTrunc(Addr64, B.getInt32Ty());
  Value *Addr32Ext = B.CreateZExt(Addr32, B.getInt64Ty());
  Value *AddrMasked = B.CreateAdd(Base, Addr32Ext);
  Value *PtrMasked = B.CreateIntToPtr(AddrMasked, B.getPtrTy());
  return PtrMasked;
}

static Value *makeResultSafe(Instruction *I, Value *Base, ValueToValueMapTy &SafeValues) {
  if (!I->getType()->isPointerTy())
    return I;
  IRBuilder<> B(I);
  Value *SafeValue = guardPtr(B, Base, I);
  SafeValues[I] = SafeValue;
  return SafeValue;
}

bool WeakLFI::run() {
  SmallVector<Instruction*, 64> ToInstrument;

  BasicBlock &Entry = F.getEntryBlock();
  IRBuilder<> BF(&Entry, Entry.begin());
  Value *Base = readRegister(BF, "x27");

  ValueToValueMapTy SafeValues;

  // Collect loads and stores.
  for (Instruction &I : instructions(F))
    if (isa<LoadInst>(&I) || isa<StoreInst>(&I))
      ToInstrument.push_back(&I);

  for (Instruction *I : ToInstrument) {
    if (LoadInst *LI = dyn_cast<LoadInst>(I)) {
      IRBuilder<> B(LI);
      Value *Ptr = LI->getPointerOperand();
      LI->setOperand(0, guardPtr(B, Base, Ptr));
    } else if (StoreInst *SI = dyn_cast<StoreInst>(I)) {
      IRBuilder<> B(SI);
      Value *Ptr = SI->getPointerOperand();
      SI->setOperand(1, guardPtr(B, Base, Ptr));
    }
  }

  return false;
}

char WeakLFILegacyPass::ID = 0;

INITIALIZE_PASS_BEGIN(WeakLFILegacyPass, DEBUG_TYPE,
    "Weak LFI instrumentation", false, false)
INITIALIZE_PASS_END(WeakLFILegacyPass, DEBUG_TYPE,
    "Weak LFI instrumentation", false, false)

PreservedAnalyses WeakLFIPass::run(Function &F,
                                    FunctionAnalysisManager &AM) {
  bool Changed = WeakLFI(F).run();
  if (!Changed)
    return PreservedAnalyses::all();
  PreservedAnalyses PA;
  PA.preserve<DominatorTreeAnalysis>();
  return PA;
}

FunctionPass *llvm::createWeakLFIPass() { return new WeakLFILegacyPass(); }
