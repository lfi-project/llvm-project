//=== X86LFIRewritePAss.cpp - Modify instructions for LFI ---------*- C++ -*-=//
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
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/Support/Alignment.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

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

bool X86LFIRewritePass::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  TII = MF.getSubtarget().getInstrInfo();
  Subtarget = &MF.getSubtarget<X86Subtarget>();

  MF.setAlignment(llvm::Align(32));

  // // LLVM does not consider basic blocks which are the targets of jump tables
  // // to be address-taken (the address can't escape anywhere else), but they are
  // // used for indirect branches, so need to be properly aligned.
  // SmallPtrSet<MachineBasicBlock *, 8> JumpTableTargets;
  // if (auto *JTI = MF.getJumpTableInfo())
  //   for (auto &JTE : JTI->getJumpTables())
  //     for (auto *MBB : JTE.MBBs)
  //       JumpTableTargets.insert(MBB);
  //
  // const X86TargetMachine *TM =
  //     static_cast<const X86TargetMachine *>(&MF.getTarget());
  //
  // for (MachineBasicBlock &MBB : MF) {
  //   // TODO: consider only setting this if MBB.hasAddressTaken() is true.
  //   if (MBB.hasAddressTaken() || JumpTableTargets.count(&MBB)) {
  //     MBB.setAlignment(llvm::Align(32));
  //     Modified = true;
  //   }
  //   // Exception handle may indirectly jump to catch pad, So we should add
  //   // ENDBR before catch pad instructions. For SjLj exception model, it will
  //   // create a new BB(new landingpad) indirectly jump to the old landingpad.
  //   if (TM->Options.ExceptionModel == ExceptionHandling::SjLj) {
  //     for (MachineBasicBlock::iterator I = MBB.begin(); I != MBB.end(); ++I) {
  //       // New Landingpad BB without EHLabel.
  //       if (MBB.isEHPad()) {
  //         if (I->isDebugInstr())
  //           continue;
  //         MBB.setAlignment(llvm::Align(32));
  //         Modified = true;
  //         break;
  //       } else if (I->isEHLabel()) {
  //         // Old Landingpad BB (is not Landingpad now) with
  //         // the old "callee" EHLabel.
  //         MCSymbol *Sym = I->getOperand(0).getMCSymbol();
  //         if (!MF.hasCallSiteLandingPad(Sym))
  //           continue;
  //         MBB.setAlignment(llvm::Align(32));
  //         Modified = true;
  //         break;
  //       }
  //     }
  //   } else if (MBB.isEHPad()){
  //     for (MachineBasicBlock::iterator I = MBB.begin(); I != MBB.end(); ++I) {
  //       if (!I->isEHLabel())
  //         continue;
  //       MBB.setAlignment(llvm::Align(32));
  //       Modified = true;
  //       break;
  //     }
  //   }
  // }
  return Modified;
}

/// createX86LFIRewritePassPass - returns an instance of the pass.
namespace llvm {
FunctionPass *createX86LFIRewritePass() { return new X86LFIRewritePass(); }
} // namespace llvm

