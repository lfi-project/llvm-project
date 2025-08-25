#include "AArch64.h"
#include "AArch64InstrInfo.h"
#include "MCTargetDesc/AArch64AddressingModes.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/Pass.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "aarch64-lfi-guard-elimination-pass"

using namespace llvm;

namespace {
class AArch64LFIGuardElimination : public MachineFunctionPass {
  const AArch64InstrInfo *TII;

public:
  static char ID;
  AArch64LFIGuardElimination() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "AArch64 LFI Guard Elimination Pass";
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    AU.addRequired<MachineLoopInfoWrapperPass>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

private:
  void emitNoExpand(MachineBasicBlock &MBB, MachineInstr &MI);
  void emitExpand(MachineBasicBlock &MBB, MachineInstr &MI);
};

char AArch64LFIGuardElimination::ID = 0;

} // end anonymous namespace

void AArch64LFIGuardElimination::emitNoExpand(MachineBasicBlock &MBB, MachineInstr &MI) {
  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(TargetOpcode::INLINEASM))
    .addExternalSymbol(".no_expand");
}

void AArch64LFIGuardElimination::emitExpand(MachineBasicBlock &MBB, MachineInstr &MI) {
  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(TargetOpcode::INLINEASM))
    .addExternalSymbol(".expand");
}

bool AArch64LFIGuardElimination::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  LLVM_DEBUG(
      dbgs() << "Running AArch64 LFI Guard Elimination Pass on function: "
             << MF.getName() << "\n");

  TII = static_cast<const AArch64InstrInfo *>(MF.getSubtarget().getInstrInfo());

  for (auto &MBB : MF) {
    for (auto &MI : llvm::make_early_inc_range(MBB)) {
      // TODO: Expand non RoW load/stores.
    }
    for (auto &MI : llvm::make_early_inc_range(MBB)) {
      // TODO: Eliminate redundant guard instructions.
      //
      // Approach:
      // 1. Find current guard instruction.
      // 2. For each instruction:
      // * if it modifies guarded reg: remove currently active guard instruction.
      // * if it's a guard:
      // ** if it's the same as the active guard: eliminate it
      // ** if it's different, reset active guard
    }
  }

  return Changed;
}

// Register the pass
INITIALIZE_PASS(AArch64LFIGuardElimination,
                "aarch64-lfi-guard-elimination-pass",
                "AArch64 LFI Guard Elimination Pass", false, false)

FunctionPass *llvm::createAArch64LFIGuardEliminationPass() {
  return new AArch64LFIGuardElimination();
}
