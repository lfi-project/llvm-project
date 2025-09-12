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
#include "llvm/CodeGen/MachineFunctionPass.h"
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
  assert(Subtarget->isLFI() && "Unexpected target in LFIRewritePass!");

  MF.setAlignment(llvm::Align(32));

  for (MachineBasicBlock &MBB : MF) {
    // TODO: consider only setting this if MBB.hasAddressTaken() is true.
    MBB.setAlignment(llvm::Align(32));
  }
  return Modified;
}

/// createX86LFIRewritePassPass - returns an instance of the pass.
namespace llvm {
FunctionPass *createX86LFIRewritePass() { return new X86LFIRewritePass(); }
} // namespace llvm
