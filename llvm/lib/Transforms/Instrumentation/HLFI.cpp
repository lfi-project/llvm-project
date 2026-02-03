//===-- HLFI.cpp - High-Level Lightweight Fault Isolation -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This pass implements High-Level Lightweight Fault Isolation (HLFI) through
// LLVM IR instrumentation, providing:
//
// 1. Forward-edge CFI via indirect branch table
// 2. Memory sandboxing via heap masking
// 3. Backward-edge CFI via safe-stack
//
// HLFI uses two reserved registers:
// - x27: Sandbox base address (fixed, same for all threads)
// - x25: HLFI context pointer (per-thread context structure)
//
// HLFI Context Layout (pointed to by x25):
//   [x25+0]  - Reserved
//   [x25+8]  - Unsafe stack pointer
//   [x25+16] - CFI table pointer
//
//===----------------------------------------------------------------------===//

#include "llvm/Transforms/Instrumentation/HLFI.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DiagnosticInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "hlfi"

STATISTIC(NumFunctionsIndexed, "Number of functions added to CFI table");
STATISTIC(NumBlockAddressesIndexed, "Number of block addresses added to CFI table");
STATISTIC(NumIndirectCalls, "Number of indirect calls transformed");
STATISTIC(NumIndirectBranches, "Number of indirect branches transformed");
STATISTIC(NumLoadsMasked, "Number of loads instrumented with heap masking");
STATISTIC(NumStoresMasked, "Number of stores instrumented with heap masking");
STATISTIC(NumAtomicsMasked, "Number of atomic operations instrumented");
STATISTIC(NumUnsafeAllocas, "Number of allocas moved to unsafe stack");

namespace {

// CFI table size (power of 2). Mask = size - 1.
constexpr uint64_t HLFICFITableSize = 4096;
constexpr uint64_t HLFICFITableMask = HLFICFITableSize - 1;

// Sandbox mask for 4GB region (32-bit offset)
constexpr uint64_t HLFISandboxMask = 0xFFFFFFFFULL;

// HLFI Context offsets (relative to x25)
constexpr int64_t HLFIContextOffsetReserved = 0;
constexpr int64_t HLFIContextOffsetUnsafeStack = 8;
constexpr int64_t HLFIContextOffsetCFITable = 16;

/// Check if a pointer is to an HLFI runtime variable (should not be masked)
static bool isHLFIRuntimePointer(Value *Ptr) {
  Value *Base = Ptr->stripPointerCasts();
  if (auto *GV = dyn_cast<GlobalVariable>(Base)) {
    return GV->getName().starts_with("__hlfi_");
  }
  return false;
}

/// Check if an alloca is "unsafe" (should go on unsafe stack)
/// Unsafe allocas: arrays, variable-size, or address-taken
static bool isUnsafeAlloca(const AllocaInst *AI, const DataLayout &DL) {
  // Variable-size allocas are unsafe
  if (!AI->isStaticAlloca())
    return true;

  // Array allocas are unsafe
  if (AI->isArrayAllocation())
    return true;

  Type *Ty = AI->getAllocatedType();

  // Array types are unsafe
  if (Ty->isArrayTy())
    return true;

  // Check if the alloca is address-taken (used in a way that could escape)
  for (const Use &U : AI->uses()) {
    const User *Usr = U.getUser();

    // Stores of the pointer are unsafe (pointer escapes)
    if (const auto *SI = dyn_cast<StoreInst>(Usr)) {
      if (SI->getValueOperand() == AI)
        return true;
    }

    // Calls that pass the pointer (except lifetime intrinsics)
    if (const auto *CB = dyn_cast<CallBase>(Usr)) {
      if (const auto *II = dyn_cast<IntrinsicInst>(CB)) {
        if (II->isLifetimeStartOrEnd())
          continue;
      }
      for (const auto &Arg : CB->args()) {
        if (Arg == AI)
          return true;
      }
    }

    // GEPs into aggregate types are conservatively unsafe
    if (isa<GetElementPtrInst>(Usr)) {
      if (Ty->isAggregateType())
        return true;
    }
  }

  return false;
}

class HLFIImpl {
  Module &M;
  LLVMContext &Ctx;
  const DataLayout &DL;
  HLFIOptions Options;

  // Runtime globals
  GlobalVariable *SandboxBase = nullptr;  // Global (fixed, x27)
  GlobalVariable *HLFIContext = nullptr;  // TLS (per-thread, x25)

  // Forward CFI state
  DenseMap<Function *, GlobalVariable *> FunctionIndexMap;
  DenseMap<BlockAddress *, GlobalVariable *> BlockAddressIndexMap;

  // Types
  Type *Int32Ty;
  Type *Int64Ty;
  PointerType *PtrTy;

public:
  HLFIImpl(Module &M, HLFIOptions Opts)
      : M(M), Ctx(M.getContext()), DL(M.getDataLayout()), Options(Opts) {
    Int32Ty = Type::getInt32Ty(Ctx);
    Int64Ty = Type::getInt64Ty(Ctx);
    PtrTy = PointerType::get(Ctx, 0);
  }

  bool run();

private:
  // Setup
  void createRuntimeGlobals();

  // Context access helpers
  Value *loadFromContext(IRBuilder<> &Builder, int64_t Offset, Type *Ty,
                         const Twine &Name);
  void storeToContext(IRBuilder<> &Builder, int64_t Offset, Value *Val);

  // Forward CFI
  void collectCFITargets(SmallVectorImpl<Function *> &Functions,
                         SmallVectorImpl<BlockAddress *> &BlockAddresses);
  void createCFITableEntries(ArrayRef<Function *> Functions,
                             ArrayRef<BlockAddress *> BlockAddresses);
  GlobalVariable *createCFITableEntry(Constant *Target, const Twine &Name);
  GlobalVariable *createCFIIndexSlot(const Twine &Name);
  void transformFunctionAddressUses();
  void transformBlockAddressUses();
  void transformIndirectCalls();
  void transformIndirectBranches();
  void transformCallSite(CallBase *CB);
  void transformIndirectBr(IndirectBrInst *IBI);

  // Heap masking
  Value *maskPointer(IRBuilder<> &Builder, Value *Ptr);
  void collectMemoryAccesses(Function &F,
                             SmallVectorImpl<Instruction *> &Accesses);
  void maskMemoryAccess(Instruction *I);

  // Safe stack
  void transformFunctionSafeStack(Function &F);
};

void HLFIImpl::createRuntimeGlobals() {
  // Sandbox base: global (not TLS - x27 is fixed for all threads)
  SandboxBase = M.getGlobalVariable("__hlfi_sandbox_base");
  if (!SandboxBase) {
    SandboxBase = new GlobalVariable(
        M, Int64Ty, false, GlobalValue::ExternalLinkage, nullptr,
        "__hlfi_sandbox_base");
  }

  // HLFI context pointer: TLS (per-thread, maps to x25)
  // The runtime ensures x25 always holds this value
  HLFIContext = M.getGlobalVariable("__hlfi_context");
  if (!HLFIContext) {
    HLFIContext = new GlobalVariable(
        M, PtrTy, false, GlobalValue::ExternalLinkage, nullptr,
        "__hlfi_context", nullptr, GlobalValue::GeneralDynamicTLSModel);
  }
}

Value *HLFIImpl::loadFromContext(IRBuilder<> &Builder, int64_t Offset,
                                  Type *Ty, const Twine &Name) {
  // Load context pointer (x25)
  Value *Ctx = Builder.CreateLoad(PtrTy, HLFIContext, "hlfi.ctx");
  // GEP to offset
  Value *FieldPtr = Builder.CreateConstInBoundsGEP1_64(
      Type::getInt8Ty(this->Ctx), Ctx, Offset, Name + ".ptr");
  // Load the field
  return Builder.CreateLoad(Ty, FieldPtr, Name);
}

void HLFIImpl::storeToContext(IRBuilder<> &Builder, int64_t Offset,
                               Value *Val) {
  // Load context pointer (x25)
  Value *Ctx = Builder.CreateLoad(PtrTy, HLFIContext, "hlfi.ctx");
  // GEP to offset
  Value *FieldPtr = Builder.CreateConstInBoundsGEP1_64(
      Type::getInt8Ty(this->Ctx), Ctx, Offset, "hlfi.ctx.field.ptr");
  // Store the value
  Builder.CreateStore(Val, FieldPtr);
}

//===----------------------------------------------------------------------===//
// Forward CFI Implementation
//===----------------------------------------------------------------------===//

void HLFIImpl::collectCFITargets(
    SmallVectorImpl<Function *> &Functions,
    SmallVectorImpl<BlockAddress *> &BlockAddresses) {

  for (Function &F : M) {
    if (F.isIntrinsic())
      continue;

    if (F.hasAddressTaken()) {
      if (F.isDeclaration()) {
        M.getContext().diagnose(DiagnosticInfoUnsupported(
            F, "HLFI: cannot take address of external function",
            F.getSubprogram()));
        continue;
      }
      Functions.push_back(&F);
    }
  }

  for (Function &F : M) {
    for (BasicBlock &BB : F) {
      if (BB.hasAddressTaken()) {
        BlockAddress *BA = BlockAddress::get(&BB);
        BlockAddresses.push_back(BA);
      }
    }
  }
}

GlobalVariable *HLFIImpl::createCFITableEntry(Constant *Target,
                                               const Twine &Name) {
  // Private linkage - the post-linker uses position-based matching,
  // so symbol names don't need to survive linking.
  auto *GV = new GlobalVariable(M, PtrTy, true, GlobalValue::PrivateLinkage,
                                 Target, Name);
  GV->setSection(".hlfi_cfi_table");
  GV->setAlignment(Align(8));
  return GV;
}

GlobalVariable *HLFIImpl::createCFIIndexSlot(const Twine &Name) {
  // Private linkage with 32-bit zero initializer.
  // The post-linker patches these with actual indices based on position.
  // Index slot N corresponds to table entry N.
  auto *GV = new GlobalVariable(M, Int32Ty, false, GlobalValue::PrivateLinkage,
                                 ConstantInt::get(Int32Ty, 0), Name);
  GV->setSection(".hlfi_cfi_indices");
  GV->setAlignment(Align(4));
  return GV;
}

void HLFIImpl::createCFITableEntries(ArrayRef<Function *> Functions,
                                      ArrayRef<BlockAddress *> BlockAddresses) {
  for (Function *F : Functions) {
    StringRef FName = F->getName();
    createCFITableEntry(F, ".hlfi_cfi_table." + FName);
    GlobalVariable *IndexSlot = createCFIIndexSlot(".hlfi_cfi_index." + FName);
    FunctionIndexMap[F] = IndexSlot;
    ++NumFunctionsIndexed;
    LLVM_DEBUG(dbgs() << "HLFI: indexed function " << FName << "\n");
  }

  for (BlockAddress *BA : BlockAddresses) {
    Function *F = BA->getFunction();
    BasicBlock *BB = BA->getBasicBlock();
    std::string Name = (F->getName() + "." + BB->getName()).str();
    createCFITableEntry(BA, ".hlfi_cfi_table." + Name);
    GlobalVariable *IndexSlot = createCFIIndexSlot(".hlfi_cfi_index." + Name);
    BlockAddressIndexMap[BA] = IndexSlot;
    ++NumBlockAddressesIndexed;
    LLVM_DEBUG(dbgs() << "HLFI: indexed blockaddress " << Name << "\n");
  }
}

void HLFIImpl::transformFunctionAddressUses() {
  for (auto &[F, IndexSlot] : FunctionIndexMap) {
    SmallVector<Use *, 16> UsesToReplace;

    for (Use &U : F->uses()) {
      User *Usr = U.getUser();
      if (auto *CB = dyn_cast<CallBase>(Usr)) {
        if (CB->isCallee(&U))
          continue;
      }
      UsesToReplace.push_back(&U);
    }

    for (Use *U : UsesToReplace) {
      auto *I = dyn_cast<Instruction>(U->getUser());
      if (!I)
        continue;

      IRBuilder<> Builder(I);
      Value *Idx = Builder.CreateLoad(Int32Ty, IndexSlot, "hlfi.idx");
      Value *IdxExt = Builder.CreateZExt(Idx, Int64Ty, "hlfi.idx.ext");
      Value *IdxAsPtr = Builder.CreateIntToPtr(IdxExt, F->getType());
      U->set(IdxAsPtr);
    }
  }
}

void HLFIImpl::transformBlockAddressUses() {
  for (auto &[BA, IndexSlot] : BlockAddressIndexMap) {
    SmallVector<Use *, 16> UsesToReplace;

    for (Use &U : BA->uses()) {
      UsesToReplace.push_back(&U);
    }

    for (Use *U : UsesToReplace) {
      auto *I = dyn_cast<Instruction>(U->getUser());
      if (!I)
        continue;

      IRBuilder<> Builder(I);
      Value *Idx = Builder.CreateLoad(Int32Ty, IndexSlot, "hlfi.ba.idx");
      Value *IdxExt = Builder.CreateZExt(Idx, Int64Ty, "hlfi.ba.idx.ext");
      Value *IdxAsPtr = Builder.CreateIntToPtr(IdxExt, BA->getType());
      U->set(IdxAsPtr);
    }
  }
}

void HLFIImpl::transformCallSite(CallBase *CB) {
  if (!CB->isIndirectCall())
    return;

  IRBuilder<> Builder(CB);
  Value *Callee = CB->getCalledOperand();

  // Convert callee to index
  Value *Idx = Builder.CreatePtrToInt(Callee, Int64Ty, "hlfi.call.idx");
  Value *MaskedIdx = Builder.CreateAnd(
      Idx, ConstantInt::get(Int64Ty, HLFICFITableMask), "hlfi.call.masked");

  // Load CFI table from context [x25+16]
  Value *TableBase = loadFromContext(Builder, HLFIContextOffsetCFITable,
                                      PtrTy, "hlfi.cfi.table");

  // Index into table and load target
  Value *Slot = Builder.CreateInBoundsGEP(PtrTy, TableBase, MaskedIdx,
                                           "hlfi.cfi.slot");
  Value *Target = Builder.CreateLoad(PtrTy, Slot, "hlfi.cfi.target");

  CB->setCalledOperand(Target);
  ++NumIndirectCalls;
}

void HLFIImpl::transformIndirectBr(IndirectBrInst *IBI) {
  IRBuilder<> Builder(IBI);
  Value *Addr = IBI->getAddress();

  // Convert address to index
  Value *Idx = Builder.CreatePtrToInt(Addr, Int64Ty, "hlfi.ibr.idx");
  Value *MaskedIdx = Builder.CreateAnd(
      Idx, ConstantInt::get(Int64Ty, HLFICFITableMask), "hlfi.ibr.masked");

  // Load CFI table from context [x25+16]
  Value *TableBase = loadFromContext(Builder, HLFIContextOffsetCFITable,
                                      PtrTy, "hlfi.cfi.table");

  // Index into table and load target
  Value *Slot = Builder.CreateInBoundsGEP(PtrTy, TableBase, MaskedIdx,
                                           "hlfi.cfi.slot");
  Value *Target = Builder.CreateLoad(PtrTy, Slot, "hlfi.ibr.target");

  IBI->setAddress(Target);
  ++NumIndirectBranches;
}

void HLFIImpl::transformIndirectCalls() {
  SmallVector<CallBase *, 32> IndirectCalls;
  for (Function &F : M) {
    for (BasicBlock &BB : F) {
      for (Instruction &I : BB) {
        if (auto *CB = dyn_cast<CallBase>(&I)) {
          if (CB->isIndirectCall())
            IndirectCalls.push_back(CB);
        }
      }
    }
  }

  for (CallBase *CB : IndirectCalls)
    transformCallSite(CB);
}

void HLFIImpl::transformIndirectBranches() {
  SmallVector<IndirectBrInst *, 16> IndirectBranches;
  for (Function &F : M) {
    for (BasicBlock &BB : F) {
      if (auto *IBI = dyn_cast<IndirectBrInst>(BB.getTerminator()))
        IndirectBranches.push_back(IBI);
    }
  }

  for (IndirectBrInst *IBI : IndirectBranches)
    transformIndirectBr(IBI);
}

//===----------------------------------------------------------------------===//
// Heap Masking Implementation
//===----------------------------------------------------------------------===//

Value *HLFIImpl::maskPointer(IRBuilder<> &Builder, Value *Ptr) {
  Value *PtrInt = Builder.CreatePtrToInt(Ptr, Int64Ty, "hlfi.ptr.int");
  Value *Offset = Builder.CreateAnd(
      PtrInt, ConstantInt::get(Int64Ty, HLFISandboxMask), "hlfi.offset");
  Value *Base = Builder.CreateLoad(Int64Ty, SandboxBase, "hlfi.base");
  Value *MaskedInt = Builder.CreateAdd(Base, Offset, "hlfi.masked.int");
  Value *MaskedPtr =
      Builder.CreateIntToPtr(MaskedInt, Ptr->getType(), "hlfi.masked.ptr");
  return MaskedPtr;
}

void HLFIImpl::collectMemoryAccesses(Function &F,
                                      SmallVectorImpl<Instruction *> &Accesses) {
  for (BasicBlock &BB : F) {
    for (Instruction &I : BB) {
      if (isa<LoadInst>(&I) || isa<StoreInst>(&I) ||
          isa<AtomicRMWInst>(&I) || isa<AtomicCmpXchgInst>(&I)) {
        Accesses.push_back(&I);
      }
    }
  }
}

void HLFIImpl::maskMemoryAccess(Instruction *I) {
  Value *Ptr = nullptr;
  unsigned PtrOpIdx = 0;

  if (auto *LI = dyn_cast<LoadInst>(I)) {
    Ptr = LI->getPointerOperand();
    PtrOpIdx = LI->getPointerOperandIndex();
  } else if (auto *SI = dyn_cast<StoreInst>(I)) {
    Ptr = SI->getPointerOperand();
    PtrOpIdx = SI->getPointerOperandIndex();
  } else if (auto *AI = dyn_cast<AtomicRMWInst>(I)) {
    Ptr = AI->getPointerOperand();
    PtrOpIdx = AtomicRMWInst::getPointerOperandIndex();
  } else if (auto *AI = dyn_cast<AtomicCmpXchgInst>(I)) {
    Ptr = AI->getPointerOperand();
    PtrOpIdx = AtomicCmpXchgInst::getPointerOperandIndex();
  } else {
    return;
  }

  // Skip HLFI runtime variable accesses
  if (isHLFIRuntimePointer(Ptr))
    return;

  IRBuilder<> Builder(I);
  Value *MaskedPtr = maskPointer(Builder, Ptr);
  I->setOperand(PtrOpIdx, MaskedPtr);

  if (isa<LoadInst>(I))
    ++NumLoadsMasked;
  else if (isa<StoreInst>(I))
    ++NumStoresMasked;
  else
    ++NumAtomicsMasked;
}

//===----------------------------------------------------------------------===//
// Safe Stack Implementation
//===----------------------------------------------------------------------===//

void HLFIImpl::transformFunctionSafeStack(Function &F) {
  if (F.isDeclaration())
    return;

  // Collect unsafe allocas
  SmallVector<AllocaInst *, 16> UnsafeAllocas;
  for (BasicBlock &BB : F) {
    for (Instruction &I : BB) {
      if (auto *AI = dyn_cast<AllocaInst>(&I)) {
        if (isUnsafeAlloca(AI, DL)) {
          UnsafeAllocas.push_back(AI);
        }
      }
    }
  }

  if (UnsafeAllocas.empty())
    return;

  // Insert at the beginning of the entry block, after existing allocas
  BasicBlock &Entry = F.getEntryBlock();
  Instruction *InsertPt = &*Entry.getFirstInsertionPt();

  // Find the last alloca to insert after it
  for (Instruction &I : Entry) {
    if (isa<AllocaInst>(&I))
      InsertPt = I.getNextNode();
    else
      break;
  }

  IRBuilder<> Builder(InsertPt);

  // Transform each unsafe alloca
  for (AllocaInst *AI : UnsafeAllocas) {
    Type *Ty = AI->getAllocatedType();
    uint64_t Size = DL.getTypeAllocSize(Ty);
    Align Alignment = AI->getAlign();

    // Load current unsafe stack pointer from context [x25+8]
    Value *UnsafeSP = loadFromContext(Builder, HLFIContextOffsetUnsafeStack,
                                       PtrTy, "hlfi.unsafe.sp");

    // Compute new stack pointer (grows down)
    Value *UnsafeSPInt = Builder.CreatePtrToInt(UnsafeSP, Int64Ty);
    Value *NewSPInt = Builder.CreateSub(
        UnsafeSPInt, ConstantInt::get(Int64Ty, Size), "hlfi.unsafe.sub");

    // Align
    uint64_t AlignMask = ~(Alignment.value() - 1);
    NewSPInt = Builder.CreateAnd(NewSPInt, ConstantInt::get(Int64Ty, AlignMask),
                                  "hlfi.unsafe.aligned");

    Value *NewSP = Builder.CreateIntToPtr(NewSPInt, PtrTy, "hlfi.unsafe.ptr");

    // Store new unsafe stack pointer to context [x25+8]
    storeToContext(Builder, HLFIContextOffsetUnsafeStack, NewSP);

    // Replace all uses of the alloca with the unsafe stack allocation
    AI->replaceAllUsesWith(NewSP);
    AI->eraseFromParent();

    ++NumUnsafeAllocas;
    LLVM_DEBUG(dbgs() << "HLFI: moved alloca to unsafe stack in "
                      << F.getName() << "\n");
  }
}

//===----------------------------------------------------------------------===//
// Main Pass Entry Point
//===----------------------------------------------------------------------===//

bool HLFIImpl::run() {
  createRuntimeGlobals();

  bool Changed = false;

  // Phase 1: Safe stack transformation
  // Must happen first so heap masking can instrument unsafe stack accesses
  if (Options.EnableSafeStack) {
    for (Function &F : M) {
      transformFunctionSafeStack(F);
    }
    Changed = true;
  }

  // Phase 2: Forward CFI
  if (Options.EnableForwardCFI) {
    SmallVector<Function *, 64> Functions;
    SmallVector<BlockAddress *, 16> BlockAddresses;
    collectCFITargets(Functions, BlockAddresses);

    if (!Functions.empty() || !BlockAddresses.empty()) {
      createCFITableEntries(Functions, BlockAddresses);
      transformFunctionAddressUses();
      transformBlockAddressUses();
      transformIndirectCalls();
      transformIndirectBranches();
      Changed = true;
    }
  }

  // Phase 3: Heap masking
  // Happens last - masks all memory accesses including unsafe stack
  if (Options.EnableHeapMasking) {
    for (Function &F : M) {
      if (F.isDeclaration())
        continue;

      SmallVector<Instruction *, 64> Accesses;
      collectMemoryAccesses(F, Accesses);

      for (Instruction *I : Accesses) {
        maskMemoryAccess(I);
      }
    }
    Changed = true;
  }

  return Changed;
}

} // anonymous namespace

PreservedAnalyses HLFIPass::run(Module &M, ModuleAnalysisManager &AM) {
  HLFIImpl Impl(M, Options);
  if (!Impl.run())
    return PreservedAnalyses::all();

  return PreservedAnalyses::none();
}
