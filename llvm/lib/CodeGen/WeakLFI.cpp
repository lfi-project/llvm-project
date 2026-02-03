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

static Value *condPtr(IRBuilder<> &B, Value *Lower, Value *Upper, Value *Ptr) {
  Value *Addr64 = B.CreatePtrToInt(Ptr, B.getInt64Ty());
  Value *LTUpper = B.CreateICmpULT(Addr64, Upper);
  Value *GTLower = B.CreateICmpUGT(Addr64, Lower);
  Value *InRange = B.CreateAnd(LTUpper, GTLower);
  return InRange;
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

  const size_t GuardSize = 128 * 1024;
  const size_t BoxSize = 4ULL * 1024 * 1024 * 1024;

  // Base - GuardSize
  Value *GuardSizeConst = BF.getInt64(GuardSize);
  Value *GuardLower = BF.CreateSub(Base, GuardSizeConst);
  // Base + 4GiB + GuardSize
  Value *BoxGuardSizeConst = BF.getInt64(BoxSize + GuardSize);
  Value *GuardUpper = BF.CreateAdd(Base, BoxGuardSizeConst);

  Value *Lower = Base;
  Value *BoxSizeConst = BF.getInt64(BoxSize);
  Value *Upper = BF.CreateAdd(Base, BoxSizeConst);

  ValueToValueMapTy SafeValues;

  // Collect loads and stores.
  for (Instruction &I : instructions(F))
    if (isa<LoadInst>(&I) || isa<StoreInst>(&I))
      ToInstrument.push_back(&I);

  for (Instruction *I : ToInstrument) {
    if (LoadInst *LI = dyn_cast<LoadInst>(I)) {
      IRBuilder<> B(LI);

      Value *Ptr = LI->getPointerOperand();
      Value *WC = B.CreateIntrinsic(Intrinsic::experimental_widenable_condition, {}, nullptr, "widenable_cond");
      Value *Cond = B.CreateAnd(condPtr(B, GuardUpper, GuardLower, Ptr), WC);

      BasicBlock *OldBlock = LI->getParent();
      BasicBlock *GuardBlock = OldBlock->splitBasicBlock(LI, "");
      BasicBlock *RestBlock = GuardBlock->splitBasicBlock(LI, "");


      Instruction *OldTerm = OldBlock->getTerminator();
      OldTerm->eraseFromParent();
      B.SetInsertPoint(OldBlock);
      B.CreateCondBr(Cond, RestBlock, GuardBlock);

      // BasicBlock *GuardBlock = BasicBlock::Create(LI->getContext(), "", OldBlock->getParent(), OldBlock);
      // IRBuilder<> GB(GuardBlock);
      // GB.CreateBr(RestBlock);

      B.SetInsertPoint(LI->getParent(), ++BasicBlock::iterator(LI));
      Function *FnAssume =
          Intrinsic::getOrInsertDeclaration(LI->getModule(), Intrinsic::assume);
      B.CreateCall(FnAssume, condPtr(B, Lower, Upper, Ptr));
    } else if (StoreInst *SI = dyn_cast<StoreInst>(I)) {
      IRBuilder<> B(SI);



      B.SetInsertPoint(LI->getParent(), ++BasicBlock::iterator(LI));
      Function *FnAssume =
          Intrinsic::getOrInsertDeclaration(LI->getModule(), Intrinsic::assume);
      B.CreateCall(FnAssume, condPtr(B, Lower, Upper, LI->getPointerOperand()));
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
