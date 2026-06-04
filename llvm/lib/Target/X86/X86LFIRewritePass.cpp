//=== X86LFIRewritePass.cpp - Modify instructions for LFI --------*- C++ -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This pass inserts ENDBR64 instructions at all valid indirect-branch targets
// for the X86-64 LFI target, and aligns those targets to 32-byte boundaries so
// that the LFI rewriter's forward-edge CFI check can succeed.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "lfi-rewrite-pass"

#include "X86.h"
#include "X86InstrInfo.h"
#include "X86Subtarget.h"
#include "X86TargetMachine.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Alignment.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
class X86LFIRewritePass : public MachineFunctionPass {
public:
  static char ID;
  X86LFIRewritePass() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return "LFI Rewrites"; }

private:
  const TargetInstrInfo *TII = nullptr;
  const X86Subtarget *Subtarget = nullptr;
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

/// Aligns MBB to 32 bytes and inserts ENDBR64 at position I.
static bool alignAndENDBR(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator I,
                          const TargetInstrInfo *TII) {
  MBB.setAlignment(llvm::Align(32));
  return addENDBR(MBB, I, TII);
}

// In large-sandbox mode the LFI rewriter masks addresses with andq when it can
// (it clobbers EFLAGS) and falls back to the slower flag-preserving pext
// otherwise. Mark each memory access that neither reads nor writes EFLAGS and
// across which EFLAGS is dead with a dead implicit EFLAGS def. X86MCInstLower
// turns that marker into the IP_LFI_FLAGS_DEAD MCInst flag, letting the
// rewriter use the cheaper andq for that access.
static bool markDeadFlagsMemOps(MachineBasicBlock &MBB,
                                const TargetRegisterInfo *TRI) {
  SmallVector<MachineInstr *, 16> ToMark;
  for (MachineBasicBlock::iterator I = MBB.begin(), E = MBB.end(); I != E; ++I) {
    MachineInstr &MI = *I;
    if (!MI.mayLoad() && !MI.mayStore())
      continue;
    const MCInstrDesc &Desc = MI.getDesc();
    if (Desc.hasImplicitDefOfPhysReg(X86::EFLAGS) ||
        Desc.hasImplicitUseOfPhysReg(X86::EFLAGS))
      continue;
    // Compute liveness on the unmodified block, then mark, so the queries are
    // not perturbed by the markers we add.
    if (MBB.computeRegisterLiveness(TRI, X86::EFLAGS, I) ==
        MachineBasicBlock::LQR_Dead)
      ToMark.push_back(&MI);
  }
  for (MachineInstr *MI : ToMark)
    MI->addRegisterDead(X86::EFLAGS, TRI, /*AddIfNotFound=*/true);
  return !ToMark.empty();
}

bool X86LFIRewritePass::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  TII = MF.getSubtarget().getInstrInfo();
  Subtarget = &MF.getSubtarget<X86Subtarget>();
  assert(Subtarget->isLFI() && "Unexpected target in LFIRewritePass!");

  // Large-sandbox mode masks memory addresses with andq (clobbers EFLAGS) when
  // the flags are dead across the access, and pext otherwise. Annotate the
  // flag-dead accesses so X86MCInstLower can flag them for the rewriter.
  if (Subtarget->isLFILargeSandbox()) {
    const TargetRegisterInfo *TRI = Subtarget->getRegisterInfo();
    for (MachineBasicBlock &MBB : MF)
      Modified |= markDeadFlagsMemOps(MBB, TRI);
  }

  // Insert ENDBR64 at the function prologue if reachable indirectly. Also
  // align the function itself so the symbol and ENDBR64 are at the same
  // 32-byte-aligned address (no NOP padding between them).
  if (needsPrologueENDBR(MF)) {
    MF.setAlignment(llvm::Align(32));
    auto MBB = MF.begin();
    Modified |= alignAndENDBR(*MBB, MBB->begin(), TII);
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
      Modified |= alignAndENDBR(MBB, MBB.begin(), TII);
    }

    // Exception handlers may be reached via indirect jumps.
    if (TM->Options.ExceptionModel == ExceptionHandling::SjLj) {
      for (auto I = MBB.begin(); I != MBB.end(); ++I) {
        if (MBB.isEHPad()) {
          if (I->isDebugInstr())
            continue;
          Modified |= alignAndENDBR(MBB, I, TII);
          break;
        } else if (I->isEHLabel()) {
          MCSymbol *Sym = I->getOperand(0).getMCSymbol();
          if (!MF.hasCallSiteLandingPad(Sym))
            continue;
          Modified |= alignAndENDBR(MBB, std::next(I), TII);
          break;
        }
      }
    } else if (MBB.isEHPad()) {
      for (auto I = MBB.begin(); I != MBB.end(); ++I) {
        if (!I->isEHLabel())
          continue;
        Modified |= alignAndENDBR(MBB, std::next(I), TII);
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
