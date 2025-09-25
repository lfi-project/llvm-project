//=== RISCVLFIRewritePAss.cpp - Modify instructions for LFI ---------*- C++ -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
#define DEBUG_TYPE "lfi-rewrite-pass"

#include "RISCV.h"
#include "RISCVInstrInfo.h"
#include "RISCVSubtarget.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/Support/Alignment.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
  class RISCVLFIRewritePass : public MachineFunctionPass {
  public:
    static char ID;
    RISCVLFIRewritePass() : MachineFunctionPass(ID) {}

    virtual bool runOnMachineFunction(MachineFunction &Fn) override;

    virtual StringRef getPassName() const override {
      return "LFI Rewrites";
    }

  private:
    const TargetInstrInfo *TII;
    const RISCVSubtarget *Subtarget;
  };

  char RISCVLFIRewritePass::ID = 0;
} // namespace

bool RISCVLFIRewritePass::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  TII = MF.getSubtarget().getInstrInfo();
  Subtarget = &MF.getSubtarget<RISCVSubtarget>();
  assert(Subtarget->isLFI() && "Unexpected target in LFIRewritePass!");

  MF.setAlignment(llvm::Align(8));

  for (MachineBasicBlock &MBB : MF) {
    // TODO: consider only setting this if MBB.hasAddressTaken() is true.
    MBB.setAlignment(llvm::Align(8));
  }
  return Modified;
}

/// createRISCVLFIRewritePassPass - returns an instance of the pass.
namespace llvm {
  FunctionPass* createRISCVLFIRewritePass() {
    return new RISCVLFIRewritePass();
  }
} // namespace llvm