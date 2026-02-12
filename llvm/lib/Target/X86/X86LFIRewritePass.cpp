//=== X86LFIRewritePass.cpp - Modify instructions for LFI --------*- C++ -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
#define DEBUG_TYPE "lfi-rewrite-pass"

#include "X86.h"
#include "X86InstrInfo.h"
#include "X86Subtarget.h"
#include "X86TargetMachine.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/IR/Module.h"
#include "llvm/MC/MCLFI.h"
#include "llvm/Support/Alignment.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

static cl::opt<bool> UnalignedDirectBranches(
    "x86-lfi-unaligned-direct-branches",
    cl::desc("Only align indirect branch targets for LFI (not all basic blocks)"),
    cl::init(false));

namespace {
class X86LFIRewritePass : public MachineFunctionPass {
public:
  static char ID;
  X86LFIRewritePass() : MachineFunctionPass(ID) {}

  virtual bool runOnMachineFunction(MachineFunction &Fn) override;

  virtual StringRef getPassName() const override { return "LFI Rewrites"; }

private:
  const TargetInstrInfo *TII;
  const X86Subtarget *Subtarget;
};

char X86LFIRewritePass::ID = 0;
} // namespace

/// Checks if the function may be reachable via an indirect call/jump.
static bool needsPrologueENDBR(MachineFunction &MF) {
  Function &F = MF.getFunction();
  if (F.doesNoCfCheck())
    return false;
  if (MF.getTarget().getCodeModel() == CodeModel::Large)
    return true;
  return F.hasAddressTaken() || !F.hasLocalLinkage();
}

/// Inserts an ENDBR64 before I if one is not already present.
static bool addENDBR(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     const TargetInstrInfo *TII) {
  if (I != MBB.end() && I->getOpcode() == X86::ENDBR64)
    return false;
  BuildMI(MBB, I, MBB.findDebugLoc(I), TII->get(X86::ENDBR64));
  return true;
}

/// Aligns MBB and optionally inserts ENDBR64 at position I when CFI is active.
static bool alignAndMaybeENDBR(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I,
                               const TargetInstrInfo *TII) {
  MBB.setAlignment(llvm::Align(32));
  if (!FlagX86LFIBundling)
    return addENDBR(MBB, I, TII);
  return false;
}

bool X86LFIRewritePass::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  TII = MF.getSubtarget().getInstrInfo();
  Subtarget = &MF.getSubtarget<X86Subtarget>();
  assert(Subtarget->isLFI() && "Unexpected target in LFIRewritePass!");

  // In bundling mode, align all basic blocks to bundle boundaries unless
  // UnalignedDirectBranches is set.
  if (FlagX86LFIBundling) {
    MF.setAlignment(llvm::Align(32));
    if (!UnalignedDirectBranches) {
      for (MachineBasicBlock &MBB : MF)
        MBB.setAlignment(llvm::Align(32));
      return true;
    }
  }

  // Without bundling, insert ENDBR64 at the function prologue if reachable
  // indirectly. Also align the function itself so the symbol and endbr64 are
  // at the same 32-byte-aligned address (no NOP padding between them).
  if (!FlagX86LFIBundling && needsPrologueENDBR(MF)) {
    MF.setAlignment(llvm::Align(32));
    auto MBB = MF.begin();
    Modified |= alignAndMaybeENDBR(*MBB, MBB->begin(), TII);
  }

  // Collect jump table targets since LLVM doesn't consider them address-taken.
  SmallPtrSet<MachineBasicBlock *, 8> JumpTableTargets;
  if (auto *JTI = MF.getJumpTableInfo())
    for (auto &JTE : JTI->getJumpTables())
      for (auto *MBB : JTE.MBBs)
        JumpTableTargets.insert(MBB);

  const X86TargetMachine *TM =
      static_cast<const X86TargetMachine *>(&MF.getTarget());

  for (MachineBasicBlock &MBB : MF) {
    if (MBB.hasAddressTaken() || MBB.isMachineBlockAddressTaken() ||
        MBB.isIRBlockAddressTaken() || JumpTableTargets.count(&MBB)) {
      Modified |= alignAndMaybeENDBR(MBB, MBB.begin(), TII);
    }

    // Exception handlers may be reached via indirect jumps.
    if (TM->Options.ExceptionModel == ExceptionHandling::SjLj) {
      for (auto I = MBB.begin(); I != MBB.end(); ++I) {
        if (MBB.isEHPad()) {
          if (I->isDebugInstr())
            continue;
          Modified |= alignAndMaybeENDBR(MBB, I, TII);
          break;
        } else if (I->isEHLabel()) {
          MCSymbol *Sym = I->getOperand(0).getMCSymbol();
          if (!MF.hasCallSiteLandingPad(Sym))
            continue;
          Modified |= alignAndMaybeENDBR(MBB, std::next(I), TII);
          break;
        }
      }
    } else if (MBB.isEHPad()) {
      for (auto I = MBB.begin(); I != MBB.end(); ++I) {
        if (!I->isEHLabel())
          continue;
        Modified |= alignAndMaybeENDBR(MBB, std::next(I), TII);
        break;
      }
    }
  }
  return Modified;
}

/// createX86LFIRewritePass - returns an instance of the pass.
namespace llvm {
FunctionPass *createX86LFIRewritePass() { return new X86LFIRewritePass(); }
} // namespace llvm
