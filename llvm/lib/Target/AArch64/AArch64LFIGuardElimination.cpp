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
};

char AArch64LFIGuardElimination::ID = 0;

} // end anonymous namespace

bool AArch64LFIGuardElimination::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  LLVM_DEBUG(
      dbgs() << "Running AArch64 LFI Guard Elimination Pass on function: "
             << MF.getName() << "\n");

  return Changed;
}

// Register the pass
INITIALIZE_PASS(AArch64LFIGuardElimination,
                "aarch64-lfi-guard-elimination-pass",
                "AArch64 LFI Guard Elimination Pass", false, false)

FunctionPass *llvm::createAArch64LFIGuardEliminationPass() {
  return new AArch64LFIGuardElimination();
}
