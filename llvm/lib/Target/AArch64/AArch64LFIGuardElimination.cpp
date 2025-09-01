#include "AArch64.h"
#include "AArch64InstrInfo.h"
#include "MCTargetDesc/AArch64AddressingModes.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/Pass.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "aarch64-lfi-guard-elimination-pass"

using namespace llvm;

static Register LFIHoistReg = AArch64::X24;

namespace {
class AArch64LFIGuardElimination : public MachineFunctionPass {
  const AArch64InstrInfo *TII;
  MachineFunction *MF;

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
  void emitBBStart(MachineBasicBlock &MBB, MachineInstr &MI);
  void emitBBEnd(MachineBasicBlock &MBB, MachineInstr &MI);
  void emitGuard(MachineBasicBlock &MBB, Register Guard, Register Reg);
  void emitGuardEnd(MachineBasicBlock &MBB, Register Reg);

  void expandLoadStoreBasic(MachineBasicBlock &MBB, MachineInstr &MI);

  MachineInstr *createAddMask(MachineInstr &MI, Register Dest, Register Src);
};

char AArch64LFIGuardElimination::ID = 0;

} // end anonymous namespace

void AArch64LFIGuardElimination::emitNoExpand(MachineBasicBlock &MBB, MachineInstr &MI) {
  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(TargetOpcode::INLINEASM))
    .addExternalSymbol(".no_expand")
    .addImm(0);
}

void AArch64LFIGuardElimination::emitExpand(MachineBasicBlock &MBB, MachineInstr &MI) {
  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(TargetOpcode::INLINEASM))
    .addExternalSymbol(".expand")
    .addImm(0);
}

void AArch64LFIGuardElimination::emitBBStart(MachineBasicBlock &MBB, MachineInstr &MI) {
  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(TargetOpcode::INLINEASM))
    .addExternalSymbol(".bb_start")
    .addImm(0);
}

void AArch64LFIGuardElimination::emitBBEnd(MachineBasicBlock &MBB, MachineInstr &MI) {
  BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(TargetOpcode::INLINEASM))
    .addExternalSymbol(".bb_end")
    .addImm(0);
}

void AArch64LFIGuardElimination::emitGuard(MachineBasicBlock &MBB, Register Guard, Register Reg) {
  auto MIB = BuildMI(*MF, DebugLoc(), TII->get(TargetOpcode::INLINEASM))
    .addExternalSymbol(".guard $0 $1")
    .addImm(0)
    .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
    .addReg(Guard)
    .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
    .addReg(Reg);
  MBB.insert(MBB.begin(), MIB);
}

void AArch64LFIGuardElimination::emitGuardEnd(MachineBasicBlock &MBB, Register Reg) {
  auto MIB = BuildMI(*MF, DebugLoc(), TII->get(TargetOpcode::INLINEASM))
    .addExternalSymbol(".guard_end $0")
    .addImm(0)
    .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
    .addReg(Reg);
  MBB.insert(MBB.end(), MIB);
}

// MachineInstr *AArch64LFIGuardElimination::createAddMask(MachineInstr &MI, Register Dest, Register Src) {
//   return BuildMI(*MF, MI.getDebugLoc(), TII->get(AArch64::ADDXrx), Dest)
//     .addReg(LFIBaseReg)
//     .addReg(getWRegFromXReg(Src))
//     .addImm(AArch64_AM::getArithExtendImm(AArch64_AM::UXTW, 0));
// }

void AArch64LFIGuardElimination::expandLoadStoreBasic(MachineBasicBlock &MBB, MachineInstr &MI) {
  // emitNoExpand(MBB, MI);
  // emitAddMask
  // if pre/post emit load/store demoted, plus add/sub
  // else emit safe load store
  // emitExpand(MBB, end...);
}

bool AArch64LFIGuardElimination::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  LLVM_DEBUG(
      dbgs() << "Running AArch64 LFI Guard Elimination Pass on function: "
             << MF.getName() << "\n");

  TII = static_cast<const AArch64InstrInfo *>(MF.getSubtarget().getInstrInfo());
  this->MF = &MF;
  MachineLoopInfo &MLI = getAnalysis<MachineLoopInfoWrapperPass>().getLI();
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

  for (auto *ML : MLI) {
    SmallDenseMap<Register, int, 8> Uses;
    SmallSet<Register, 8> Defs;

    bool HasCall = false;

    for (auto *MBB : ML->blocks()) {
      for (auto &MI : *MBB) {
        bool IsMemAccess = MI.mayLoad() || MI.mayStore();
        if (MI.isCall()) {
          HasCall = true;
        }

        for (auto &MO : MI.operands()) {
          if (!MO.isReg())
            continue;
          Register R = MO.getReg();
          if (!R)
            continue;

          if (MO.isUse() && IsMemAccess)
            Uses[R]++;
          if (MO.isDef())
            Defs.insert(R);
        }
      }
    }

    if (HasCall)
      continue;

    std::pair<Register, int> BestReg(AArch64::XZR, 0);
    for (auto &KV : Uses) {
      Register R = KV.first;
      int NumUses = KV.second;
      if (!Defs.contains(R))
        if (NumUses > BestReg.second)
          BestReg = std::pair(R, NumUses);
    }

    if (BestReg.second > 0) {
      auto *Preheader = ML->getLoopPreheader();
      if (!Preheader)
        continue;
      dbgs() << MF.getName() << ": hoisting use of register " << TRI->getRegAsmName(BestReg.first) << ", uses: " << BestReg.second << "\n";
      auto MIB = BuildMI(MF, DebugLoc(), TII->get(TargetOpcode::INLINEASM))
        .addExternalSymbol(".guard_load $0 $1")
        .addImm(0)
        .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
        .addReg(LFIHoistReg)
        .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
        .addReg(BestReg.first);
      Preheader->insert(Preheader->end(), MIB);

      for (auto *MBB : ML->blocks()) {
        if (MBB->empty())
          continue;
        auto Guard = BuildMI(MF, DebugLoc(), TII->get(TargetOpcode::INLINEASM))
          .addExternalSymbol(".guard $0 $1")
          .addImm(0)
          .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
          .addReg(LFIHoistReg)
          .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
          .addReg(BestReg.first);
        MBB->insert(MBB->begin(), Guard);

        auto NoGuard = BuildMI(MF, DebugLoc(), TII->get(TargetOpcode::INLINEASM))
          .addExternalSymbol(".no_guard $0 $1")
          .addImm(0)
          .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
          .addReg(LFIHoistReg)
          .addImm(InlineAsm::Flag(InlineAsm::Kind::RegUse, 1))
          .addReg(BestReg.first);
        MBB->insert(MBB->end(), NoGuard);
      }
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
