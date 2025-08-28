//===- AArch64MCLFIExpander.cpp ---------------------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the AArch64MCLFIExpander class, the AArch64 specific
// subclass of MCLFIExpander.
//
//===----------------------------------------------------------------------===//
#include "AArch64MCLFIExpander.h"
#include "AArch64AddressingModes.h"
#include "Utils/AArch64BaseInfo.h"

#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "lfi"

static cl::opt<bool> AArch64LFIErrorReserved(
    "aarch64-lfi-error-reserved", cl::Hidden, cl::init(false),
    cl::desc("Produce errors for uses of LFI reserved registers"));

static MCRegister LFIAddrReg = AArch64::X28;
static MCRegister LFIBaseReg = AArch64::X27;
static MCRegister LFIScratchReg = AArch64::X26;
static MCRegister LFITLSReg = AArch64::X25;

static bool hasFeature(const FeatureBitset Feature,
                       const MCSubtargetInfo &STI) {
  return (STI.getFeatureBits() & Feature) == Feature;
}

bool AArch64::AArch64MCLFIExpander::isValidScratchRegister(
    MCRegister Reg) const {
  return Reg != AArch64::SP;
}

MCRegister AArch64::AArch64MCLFIExpander::getScratch() {
  if (numScratchRegs() == 0) {
    return LFIScratchReg;
  }
  return getScratchReg(0);
}

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rs, int64_t Imm,
                 MCLFIExpander &Exp, MCStreamer &Out,
                 const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rs));
  Inst.addOperand(MCOperand::createImm(Imm));
  Exp.emitInst(Inst, Out, STI);
}

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rt1, MCRegister Rt2,
                 int64_t Imm, MCLFIExpander &Exp, MCStreamer &Out,
                 const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rt1));
  Inst.addOperand(MCOperand::createReg(Rt2));
  Inst.addOperand(MCOperand::createImm(Imm));
  Exp.emitInst(Inst, Out, STI);
}

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rt1, MCRegister Rt2,
                 int64_t Imm1, int64_t Imm2, MCLFIExpander &Exp,
                 MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rt1));
  Inst.addOperand(MCOperand::createReg(Rt2));
  Inst.addOperand(MCOperand::createImm(Imm1));
  Inst.addOperand(MCOperand::createImm(Imm2));
  Exp.emitInst(Inst, Out, STI);
}

static void emit(unsigned int Op, MCRegister Rd, MCRegister Rt, int64_t Imm1,
                 int64_t Imm2, MCLFIExpander &Exp, MCStreamer &Out,
                 const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Rd));
  Inst.addOperand(MCOperand::createReg(Rt));
  Inst.addOperand(MCOperand::createImm(Imm1));
  Inst.addOperand(MCOperand::createImm(Imm2));
  Exp.emitInst(Inst, Out, STI);
}

static void emit(unsigned int Op, MCRegister Reg, MCLFIExpander &Exp,
                 MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Inst;
  Inst.setOpcode(Op);
  Inst.addOperand(MCOperand::createReg(Reg));
  Exp.emitInst(Inst, Out, STI);
}

static void emitMov(MCRegister Dest, MCRegister Src, MCLFIExpander &Exp,
                    MCStreamer &Out, const MCSubtargetInfo &STI) {
  emit(AArch64::ORRXrs, Dest, AArch64::XZR, Src, 0, Exp, Out, STI);
}

// Emit 'add Dest, LFIBaseReg, W(Src), uxtw'
void AArch64::AArch64MCLFIExpander::emitAddMask(MCRegister Dest, MCRegister Src,
                        MCStreamer &Out, const MCSubtargetInfo &STI) {
  if (Dest == LFIAddrReg && ActiveBB && ActiveGuard && ActiveGuardReg == Src)
    return;
  emit(AArch64::ADDXrx, Dest, LFIBaseReg, getWRegFromXReg(Src),
       AArch64_AM::getArithExtendImm(AArch64_AM::UXTW, 0), *this, Out, STI);
  if (Dest == LFIAddrReg && ActiveBB) {
    ActiveGuard = true;
    ActiveGuardReg = Src;
  }
}

void AArch64::AArch64MCLFIExpander::emitInst(const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  if (ActiveGuard && (mayModifyRegister(Inst, ActiveGuardReg) ||
      mayModifyRegister(Inst, getWRegFromXReg(ActiveGuardReg)) ||
      mayAffectControlFlow(Inst)))
    ActiveGuard = false;

  Out.emitInstruction(Inst, STI);
}

// Emit 'Op(ld/st) Dest, [LFIBaseReg, W(Target), uxtw]'
static void emitMemMask(unsigned Op, MCRegister Dest, MCRegister Target,
                        MCLFIExpander &Exp, MCStreamer &Out,
                        const MCSubtargetInfo &STI) {
  emit(Op, Dest, LFIBaseReg, getWRegFromXReg(Target), 0, 0, Exp, Out, STI);
}

static void emitBranch(unsigned int Opcode, MCRegister Target,
                       MCLFIExpander &Exp, MCStreamer &Out,
                       const MCSubtargetInfo &STI) {
  MCInst Branch;
  Branch.setOpcode(Opcode);
  Branch.addOperand(MCOperand::createReg(Target));
  Exp.emitInst(Branch, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandIndirectBranch(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI,
    bool IsCall) {
  (void)IsCall;

  assert(Inst.getOperand(0).isReg());
  MCRegister BranchReg = Inst.getOperand(0).getReg();

  emitAddMask(LFIAddrReg, BranchReg, Out, STI);
  emitBranch(Inst.getOpcode(), LFIAddrReg, *this, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandCall(const MCInst &Inst,
                                               MCStreamer &Out,
                                               const MCSubtargetInfo &STI) {
  if (Inst.getOperand(0).isReg())
    expandIndirectBranch(Inst, Out, STI, true);
  else
    emitInst(Inst, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandReturn(const MCInst &Inst,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  assert(Inst.getOperand(0).isReg());
  if (Inst.getOperand(0).getReg() != AArch64::LR)
    expandIndirectBranch(Inst, Out, STI, false);
  else
    emitInst(Inst, Out, STI);
}

bool AArch64::AArch64MCLFIExpander::mayModifyStack(const MCInst &Inst) {
  return mayModifyRegister(Inst, AArch64::SP);
}

bool AArch64::AArch64MCLFIExpander::mayModifyReserved(const MCInst &Inst) {
  return mayModifyRegister(Inst, LFIAddrReg) ||
         mayModifyRegister(Inst, LFIBaseReg);
}

bool AArch64::AArch64MCLFIExpander::mayModifyLR(const MCInst &Inst) {
  return mayModifyRegister(Inst, AArch64::LR);
}

static MCInst replaceReg(const MCInst &Inst, MCRegister Dest, MCRegister Src) {
  MCInst New;
  New.setOpcode(Inst.getOpcode());
  New.setLoc(Inst.getLoc());
  for (unsigned I = 0; I < Inst.getNumOperands(); ++I) {
    const MCOperand &Op = Inst.getOperand(I);
    if (Op.isReg() && Op.getReg() == Src) {
      New.addOperand(MCOperand::createReg(Dest));
    } else {
      New.addOperand(Op);
    }
  }
  return New;
}

void AArch64::AArch64MCLFIExpander::expandLRModification(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCRegister Scratch = getScratch();
  MCInst New = replaceReg(Inst, Scratch, AArch64::LR);
  if (mayLoad(New) || mayStore(New))
    expandLoadStore(New, Out, STI);
  else
    emitInst(New, Out, STI);
  emitAddMask(AArch64::LR, Scratch, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandStackModification(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  if (mayLoad(Inst) || mayStore(Inst)) {
    if (mayModifyLR(Inst))
      return expandLRModification(Inst, Out, STI);
    return emitInst(Inst, Out, STI);
  }

  MCInst ModInst;
  MCRegister Scratch = getScratch();
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(0).getReg() == AArch64::SP);
  ModInst.setOpcode(Inst.getOpcode());
  ModInst.setLoc(Inst.getLoc());
  ModInst.addOperand(MCOperand::createReg(Scratch));
  for (unsigned I = 1, E = Inst.getNumOperands(); I != E; ++I) {
    ModInst.addOperand(Inst.getOperand(I));
  }
  emitInst(ModInst, Out, STI);
  emitAddMask(AArch64::SP, Scratch, Out, STI);
}

static bool canConvertToRoW(unsigned Op);
static unsigned convertRoXToRoW(unsigned Op, unsigned &Shift);
static unsigned convertRoWToRoW(unsigned Op, unsigned &Shift);
static unsigned convertUiToRoW(unsigned Op);
static unsigned convertPreToRoW(unsigned Op);
static unsigned convertPostToRoW(unsigned Op);

static unsigned convertPrePostToBase(unsigned Op, bool &IsPre,
                                     bool &IsBaseNoOffset);
static unsigned getPrePostScale(unsigned Op);

static void emitSafeLoadStoreDemoted(const MCInst &Inst, unsigned N,
                                     MCLFIExpander &Exp, MCStreamer &Out,
                                     const MCSubtargetInfo &STI) {
  MCInst LoadStore;
  bool IsPre, IsBaseNoOffset;
  auto NewOpCode =
      convertPrePostToBase(Inst.getOpcode(), IsPre, IsBaseNoOffset);
  LoadStore.setOpcode(NewOpCode);
  for (unsigned I = 1; I < N; I++)
    LoadStore.addOperand(Inst.getOperand(I));
  LoadStore.addOperand(MCOperand::createReg(LFIAddrReg));
  if (IsPre)
    LoadStore.addOperand(Inst.getOperand(N + 1));
  else if (!IsBaseNoOffset)
    LoadStore.addOperand(MCOperand::createImm(0));
  Exp.emitInst(LoadStore, Out, STI);
}

static void emitSafeLoadStore(const MCInst &Inst, unsigned N,
                              MCLFIExpander &Exp, MCStreamer &Out,
                              const MCSubtargetInfo &STI) {
  MCInst LoadStore;
  LoadStore.setOpcode(Inst.getOpcode());
  for (unsigned I = 0; I < N; ++I)
    LoadStore.addOperand(Inst.getOperand(I));
  LoadStore.addOperand(MCOperand::createReg(LFIAddrReg));
  for (unsigned I = N + 1; I < Inst.getNumOperands(); ++I)
    LoadStore.addOperand(Inst.getOperand(I));
  Exp.emitInst(LoadStore, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandLoadStoreBasic(
    const MCInst &Inst, MemInstInfo &MII, MCStreamer &Out,
    const MCSubtargetInfo &STI) {
  MCRegister Base = Inst.getOperand(MII.BaseRegIdx).getReg();
  bool SkipGuard = false;
  if (GuardMap.count(Base)) {
    if (GuardUses[Base] != 0)
      SkipGuard = true;
    GuardUses[Base]++;
  }

  if (!SkipGuard)
    emitAddMask(LFIAddrReg, Inst.getOperand(MII.BaseRegIdx).getReg(),
                Out, STI);

  if (MII.IsPrePost) {
    assert(MII.OffsetIdx != -1 && "Pre/Post must have valid OffsetIdx");

    emitSafeLoadStoreDemoted(Inst, MII.BaseRegIdx, *this, Out, STI);
    MCRegister Base = Inst.getOperand(MII.BaseRegIdx).getReg();
    MCOperand OffsetMO = Inst.getOperand(MII.OffsetIdx);
    if (OffsetMO.isReg()) {
      // The immediate offset of post-indexed addressing NEON Instrs has a fixed
      // value, and it is encoded as a post-index addressing with XZR register
      // operand. e.g., LD3Threev3d_POST can only have #48 as its operand and
      // its offset MachineOperand holds XZR, which is a *Register* kind, not
      // Imm.
      MCRegister OffReg = OffsetMO.getReg();
      if (OffReg == AArch64::XZR) {
        const LdStNInstrDesc *Info = getLdStNInstrDesc(Inst.getOpcode());
        assert(Info && Info->NaturalOffset >= 0);
        return emit(AArch64::ADDXri, Base, Base, Info->NaturalOffset, 0, *this,
                    Out, STI);
      }
      assert(OffReg != AArch64::WZR);
      return emit(AArch64::ADDXrs, Base, Base, OffsetMO.getReg(), 0, *this, Out,
                  STI);
    } else {
      auto Offset = Inst.getOperand(MII.OffsetIdx).getImm() *
                    getPrePostScale(Inst.getOpcode());
      if (Offset >= 0)
        return emit(AArch64::ADDXri, Base, Base, Offset, 0, *this, Out, STI);
      return emit(AArch64::SUBXri, Base, Base, -Offset, 0, *this, Out, STI);
    }
  }

  return emitSafeLoadStore(Inst, MII.BaseRegIdx, *this, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandLoadStoreRoW(
    const MCInst &Inst, MemInstInfo &MII, MCStreamer &Out,
    const MCSubtargetInfo &STI) {
  unsigned MemOp;
  unsigned Op = Inst.getOpcode();
  if ((MemOp = convertUiToRoW(Op)) != AArch64::INSTRUCTION_LIST_END) {
    auto OffsetMCO = Inst.getOperand(2);
    if (OffsetMCO.isImm() && OffsetMCO.getImm() == 0)
      return emitMemMask(MemOp, Inst.getOperand(0).getReg(),
                         Inst.getOperand(1).getReg(), *this, Out, STI);
    return expandLoadStoreBasic(Inst, MII, Out, STI);
  }

  if ((MemOp = convertPreToRoW(Op)) != AArch64::INSTRUCTION_LIST_END) {
    MCRegister Reg = Inst.getOperand(2).getReg();
    int64_t Imm = Inst.getOperand(3).getImm();
    if (Imm >= 0)
      emit(AArch64::ADDXri, Reg, Reg, Imm, 0, *this, Out, STI);
    else
      emit(AArch64::SUBXri, Reg, Reg, -Imm, 0, *this, Out, STI);
    return emitMemMask(MemOp, Inst.getOperand(1).getReg(), Reg, *this, Out,
                       STI);
  }

  if ((MemOp = convertPostToRoW(Op)) != AArch64::INSTRUCTION_LIST_END) {
    MCRegister Reg = Inst.getOperand(2).getReg();
    emitMemMask(MemOp, Inst.getOperand(1).getReg(), Reg, *this, Out, STI);
    int64_t Imm = Inst.getOperand(3).getImm();
    if (Imm >= 0)
      emit(AArch64::ADDXri, Reg, Reg, Imm, 0, *this, Out, STI);
    else
      emit(AArch64::SUBXri, Reg, Reg, -Imm, 0, *this, Out, STI);
    return;
  }

  unsigned Shift;
  if ((MemOp = convertRoXToRoW(Op, Shift)) != AArch64::INSTRUCTION_LIST_END) {
    MCRegister Reg1 = Inst.getOperand(1).getReg();
    MCRegister Reg2 = Inst.getOperand(2).getReg();
    int64_t Extend = Inst.getOperand(3).getImm();
    int64_t IsShift = Inst.getOperand(4).getImm();
    MCRegister Scratch = getScratch();
    if (!IsShift)
      Shift = 0;
    if (Extend)
      emit(AArch64::ADDXrx, Scratch, Reg1, Reg2,
           AArch64_AM::getArithExtendImm(AArch64_AM::SXTX, Shift), *this, Out,
           STI);
    else
      emit(AArch64::ADDXrs, Scratch, Reg1, Reg2,
           AArch64_AM::getShifterImm(AArch64_AM::LSL, Shift), *this, Out, STI);
    return emitMemMask(MemOp, Inst.getOperand(0).getReg(), Scratch, *this, Out,
                       STI);
  }

  if ((MemOp = convertRoWToRoW(Op, Shift)) != AArch64::INSTRUCTION_LIST_END) {
    MCRegister Reg1 = Inst.getOperand(1).getReg();
    MCRegister Reg2 = Inst.getOperand(2).getReg();
    int64_t S = Inst.getOperand(3).getImm();
    int64_t IsShift = Inst.getOperand(4).getImm();
    MCRegister Scratch = getScratch();
    if (!IsShift)
      Shift = 0;
    if (S)
      emit(AArch64::ADDXrx, Scratch, Reg1, Reg2,
           AArch64_AM::getArithExtendImm(AArch64_AM::SXTW, Shift), *this, Out,
           STI);
    else
      emit(AArch64::ADDXrx, Scratch, Reg1, Reg2,
           AArch64_AM::getArithExtendImm(AArch64_AM::UXTW, Shift), *this, Out,
           STI);
    return emitMemMask(MemOp, Inst.getOperand(0).getReg(), Scratch, *this, Out,
                       STI);
  }
}

static std::optional<MemInstInfo> getAtomicLoadStoreInfo(const MCInst &Inst);

static std::optional<MemInstInfo> getMemInstInfo(const MCInst &Inst) {
  auto MII = getLoadInfo(Inst);
  if (MII.has_value())
    return MII;
  MII = getStoreInfo(Inst);
  if (MII.has_value())
    return MII;
  MII = getAtomicLoadStoreInfo(Inst);
  if (MII.has_value())
    return MII;
  return std::nullopt;
}

void AArch64::AArch64MCLFIExpander::expandLoadStore(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  if (hasFeature(FeatureBitset({AArch64::FeatureLFIJumps}), STI))
    return emitInst(Inst, Out, STI);
  if (hasFeature(FeatureBitset({AArch64::FeatureLFIStores}), STI) &&
      !mayStore(Inst))
    return emitInst(Inst, Out, STI);

  auto MII = getMemInstInfo(Inst);
  if (!MII.has_value())
    return emitInst(Inst, Out, STI);

  // Stack accesses without a register offset don't need rewriting.
  if (Inst.getOperand(MII->BaseRegIdx).getReg() == AArch64::SP) {
    if (MII->BaseRegIdx == (int)Inst.getNumOperands() - 1 ||
        !Inst.getOperand(MII->BaseRegIdx + 1).isReg())
      return emitInst(Inst, Out, STI);
  }

  // Try to convert to RoW if we can, otherwise use fallback.
  if (canConvertToRoW(Inst.getOpcode()))
    expandLoadStoreRoW(Inst, MII.value(), Out, STI);
  else
    expandLoadStoreBasic(Inst, MII.value(), Out, STI);
}

void AArch64::AArch64MCLFIExpander::emitLFICall(LFICallType CallType,
                                                MCStreamer &Out,
                                                const MCSubtargetInfo &STI) {
  MCRegister Scratch = getScratch();
  emitMov(Scratch, AArch64::LR, *this, Out, STI);
  unsigned Offset;
  switch (CallType) {
  case LFISyscall:
    Offset = 0;
    break;
  case LFITLSRead:
    Offset = 1;
    break;
  case LFITLSWrite:
    Offset = 2;
    break;
  }
  emit(AArch64::LDRXui, AArch64::LR, LFIBaseReg, Offset, *this, Out, STI);
  emit(AArch64::BLR, AArch64::LR, *this, Out, STI);
  emitAddMask(AArch64::LR, Scratch, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandSyscall(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  emitLFICall(LFISyscall, Out, STI);
}

static void emitSwap(MCRegister Reg1, MCRegister Reg2, MCLFIExpander &Exp,
                     MCStreamer &Out, const MCSubtargetInfo &STI) {
  emit(AArch64::EORXrs, Reg1, Reg1, Reg2, 0, Exp, Out, STI);
  emit(AArch64::EORXrs, Reg2, Reg1, Reg2, 0, Exp, Out, STI);
  emit(AArch64::EORXrs, Reg1, Reg1, Reg2, 0, Exp, Out, STI);
}

void AArch64::AArch64MCLFIExpander::expandTLSRead(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  MCRegister Reg = Inst.getOperand(0).getReg();

  if (hasFeature(FeatureBitset({AArch64::FeatureLFITLSReg}), STI))
    return emit(AArch64::LDRXui, Reg, LFITLSReg, 0, *this, Out, STI);

  if (Reg == AArch64::X0) {
    emitLFICall(LFITLSRead, Out, STI);
  } else {
    emitMov(Reg, AArch64::X0, *this, Out, STI);
    emitLFICall(LFITLSRead, Out, STI);
    emitSwap(AArch64::X0, Reg, *this, Out, STI);
  }
}

void AArch64::AArch64MCLFIExpander::expandTLSWrite(const MCInst &Inst,
                                                   MCStreamer &Out,
                                                   const MCSubtargetInfo &STI) {
  MCRegister Reg = Inst.getOperand(1).getReg();
  if (Reg == AArch64::X0) {
    emitLFICall(LFITLSWrite, Out, STI);
  } else {
    emitSwap(Reg, AArch64::X0, *this, Out, STI);
    emitLFICall(LFITLSWrite, Out, STI);
    emitSwap(AArch64::X0, Reg, *this, Out, STI);
  }
}

static bool isSyscall(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::SVC;
}

static bool isTLSRead(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::MRS &&
         Inst.getOperand(1).getImm() == AArch64SysReg::TPIDR_EL0;
}

static bool isTLSWrite(const MCInst &Inst) {
  return Inst.getOpcode() == AArch64::MSR &&
         Inst.getOperand(0).getImm() == AArch64SysReg::TPIDR_EL0;
}

void AArch64::AArch64MCLFIExpander::doExpandInst(const MCInst &Inst,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  if (GuardMap.size() > 0)
    for (auto &KV : GuardMap)
      if (mayModifyRegister(Inst, KV.first))
        return Out.getContext().reportError(
            Inst.getLoc(), "illegal modification guarded register");

  if (isSyscall(Inst))
    return expandSyscall(Inst, Out, STI);

  if (isTLSRead(Inst))
    return expandTLSRead(Inst, Out, STI);

  if (isTLSWrite(Inst))
    return expandTLSWrite(Inst, Out, STI);

  if (isReturn(Inst))
    return expandReturn(Inst, Out, STI);

  if (isIndirectBranch(Inst))
    return expandIndirectBranch(Inst, Out, STI, false);

  if (isCall(Inst))
    return expandCall(Inst, Out, STI);

  if (isBranch(Inst))
    return emitInst(Inst, Out, STI);

  // Bail out with an error. In the future, we could consider automatically
  // rewriting uses of reserved LFI registers.
  if (mayModifyReserved(Inst)) {
    if (AArch64LFIErrorReserved)
      return Out.getContext().reportError(
          Inst.getLoc(), "illegal modification of reserved LFI register");
    Out.getContext().reportWarning(
        Inst.getLoc(), "deleting modification of reserved LFI register");
    MCInst New = replaceReg(Inst, AArch64::XZR, LFIBaseReg);
    if (mayModifyReserved(New)) {
      MCRegister Scratch = getScratch();
      New = replaceReg(New, Scratch, LFIAddrReg);
    }
    assert(!mayModifyReserved(New));
    return doExpandInst(New, Out, STI);
  }

  if (mayModifyStack(Inst))
    return expandStackModification(Inst, Out, STI);

  if (mayModifyLR(Inst))
    return expandLRModification(Inst, Out, STI);

  if (mayLoad(Inst) || mayStore(Inst))
    return expandLoadStore(Inst, Out, STI);

  return emitInst(Inst, Out, STI);
}

void AArch64::AArch64MCLFIExpander::startBB(MCStreamer &Out,
                                            const MCSubtargetInfo &STI) {
  ActiveBB = true;
  ActiveGuard = false;
}

void AArch64::AArch64MCLFIExpander::endBB(MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
  ActiveBB = false;
  ActiveGuard = false;
}

bool AArch64::AArch64MCLFIExpander::expandInst(const MCInst &Inst,
                                               MCStreamer &Out,
                                               const MCSubtargetInfo &STI) {
  if (Guard)
    return false;
  Guard = true;

  doExpandInst(Inst, Out, STI);

  Guard = false;
  return true;
}

static unsigned convertRoXToRoW(unsigned Op, unsigned &Shift) {
  Shift = 0;
  switch (Op) {
  case AArch64::LDRBBroX:
    return AArch64::LDRBBroW;
  case AArch64::LDRBroX:
    return AArch64::LDRBroW;
  case AArch64::LDRDroX:
    Shift = 3;
    return AArch64::LDRDroW;
  case AArch64::LDRHHroX:
    Shift = 1;
    return AArch64::LDRHHroW;
  case AArch64::LDRHroX:
    Shift = 1;
    return AArch64::LDRHroW;
  case AArch64::LDRQroX:
    Shift = 4;
    return AArch64::LDRQroW;
  case AArch64::LDRSBWroX:
    Shift = 1;
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXroX:
    Shift = 1;
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWroX:
    Shift = 1;
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXroX:
    Shift = 1;
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWroX:
    Shift = 2;
    return AArch64::LDRSWroW;
  case AArch64::LDRSroX:
    Shift = 2;
    return AArch64::LDRSroW;
  case AArch64::LDRWroX:
    Shift = 2;
    return AArch64::LDRWroW;
  case AArch64::LDRXroX:
    Shift = 3;
    return AArch64::LDRXroW;
  case AArch64::STRBBroX:
    return AArch64::STRBBroW;
  case AArch64::STRBroX:
    return AArch64::STRBroW;
  case AArch64::STRDroX:
    Shift = 3;
    return AArch64::STRDroW;
  case AArch64::STRHHroX:
    Shift = 1;
    return AArch64::STRHHroW;
  case AArch64::STRHroX:
    Shift = 1;
    return AArch64::STRHroW;
  case AArch64::STRQroX:
    Shift = 4;
    return AArch64::STRQroW;
  case AArch64::STRSroX:
    Shift = 2;
    return AArch64::STRSroW;
  case AArch64::STRWroX:
    Shift = 2;
    return AArch64::STRWroW;
  case AArch64::STRXroX:
    Shift = 3;
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertRoWToRoW(unsigned Op, unsigned &Shift) {
  Shift = 0;
  switch (Op) {
  case AArch64::LDRBBroW:
    return AArch64::LDRBBroW;
  case AArch64::LDRBroW:
    return AArch64::LDRBroW;
  case AArch64::LDRDroW:
    Shift = 3;
    return AArch64::LDRDroW;
  case AArch64::LDRHHroW:
    Shift = 1;
    return AArch64::LDRHHroW;
  case AArch64::LDRHroW:
    Shift = 1;
    return AArch64::LDRHroW;
  case AArch64::LDRQroW:
    Shift = 4;
    return AArch64::LDRQroW;
  case AArch64::LDRSBWroW:
    Shift = 1;
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXroW:
    Shift = 1;
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWroW:
    Shift = 1;
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXroW:
    Shift = 1;
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWroW:
    Shift = 2;
    return AArch64::LDRSWroW;
  case AArch64::LDRSroW:
    Shift = 2;
    return AArch64::LDRSroW;
  case AArch64::LDRWroW:
    Shift = 2;
    return AArch64::LDRWroW;
  case AArch64::LDRXroW:
    Shift = 3;
    return AArch64::LDRXroW;
  case AArch64::STRBBroW:
    return AArch64::STRBBroW;
  case AArch64::STRBroW:
    return AArch64::STRBroW;
  case AArch64::STRDroW:
    Shift = 3;
    return AArch64::STRDroW;
  case AArch64::STRHHroW:
    Shift = 1;
    return AArch64::STRHHroW;
  case AArch64::STRHroW:
    Shift = 1;
    return AArch64::STRHroW;
  case AArch64::STRQroW:
    Shift = 4;
    return AArch64::STRQroW;
  case AArch64::STRSroW:
    Shift = 2;
    return AArch64::STRSroW;
  case AArch64::STRWroW:
    Shift = 2;
    return AArch64::STRWroW;
  case AArch64::STRXroW:
    Shift = 3;
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertUiToRoW(unsigned Op) {
  switch (Op) {
  case AArch64::LDRBBui:
    return AArch64::LDRBBroW;
  case AArch64::LDRBui:
    return AArch64::LDRBroW;
  case AArch64::LDRDui:
    return AArch64::LDRDroW;
  case AArch64::LDRHHui:
    return AArch64::LDRHHroW;
  case AArch64::LDRHui:
    return AArch64::LDRHroW;
  case AArch64::LDRQui:
    return AArch64::LDRQroW;
  case AArch64::LDRSBWui:
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXui:
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWui:
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXui:
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWui:
    return AArch64::LDRSWroW;
  case AArch64::LDRSui:
    return AArch64::LDRSroW;
  case AArch64::LDRWui:
    return AArch64::LDRWroW;
  case AArch64::LDRXui:
    return AArch64::LDRXroW;
  case AArch64::STRBBui:
    return AArch64::STRBBroW;
  case AArch64::STRBui:
    return AArch64::STRBroW;
  case AArch64::STRDui:
    return AArch64::STRDroW;
  case AArch64::STRHHui:
    return AArch64::STRHHroW;
  case AArch64::STRHui:
    return AArch64::STRHroW;
  case AArch64::STRQui:
    return AArch64::STRQroW;
  case AArch64::STRSui:
    return AArch64::STRSroW;
  case AArch64::STRWui:
    return AArch64::STRWroW;
  case AArch64::STRXui:
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertPreToRoW(unsigned Op) {
  switch (Op) {
  case AArch64::LDRBBpre:
    return AArch64::LDRBBroW;
  case AArch64::LDRBpre:
    return AArch64::LDRBroW;
  case AArch64::LDRDpre:
    return AArch64::LDRDroW;
  case AArch64::LDRHHpre:
    return AArch64::LDRHHroW;
  case AArch64::LDRHpre:
    return AArch64::LDRHroW;
  case AArch64::LDRQpre:
    return AArch64::LDRQroW;
  case AArch64::LDRSBWpre:
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXpre:
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWpre:
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXpre:
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWpre:
    return AArch64::LDRSWroW;
  case AArch64::LDRSpre:
    return AArch64::LDRSroW;
  case AArch64::LDRWpre:
    return AArch64::LDRWroW;
  case AArch64::LDRXpre:
    return AArch64::LDRXroW;
  case AArch64::STRBBpre:
    return AArch64::STRBBroW;
  case AArch64::STRBpre:
    return AArch64::STRBroW;
  case AArch64::STRDpre:
    return AArch64::STRDroW;
  case AArch64::STRHHpre:
    return AArch64::STRHHroW;
  case AArch64::STRHpre:
    return AArch64::STRHroW;
  case AArch64::STRQpre:
    return AArch64::STRQroW;
  case AArch64::STRSpre:
    return AArch64::STRSroW;
  case AArch64::STRWpre:
    return AArch64::STRWroW;
  case AArch64::STRXpre:
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertPostToRoW(unsigned Op) {
  switch (Op) {
  case AArch64::LDRBBpost:
    return AArch64::LDRBBroW;
  case AArch64::LDRBpost:
    return AArch64::LDRBroW;
  case AArch64::LDRDpost:
    return AArch64::LDRDroW;
  case AArch64::LDRHHpost:
    return AArch64::LDRHHroW;
  case AArch64::LDRHpost:
    return AArch64::LDRHroW;
  case AArch64::LDRQpost:
    return AArch64::LDRQroW;
  case AArch64::LDRSBWpost:
    return AArch64::LDRSBWroW;
  case AArch64::LDRSBXpost:
    return AArch64::LDRSBXroW;
  case AArch64::LDRSHWpost:
    return AArch64::LDRSHWroW;
  case AArch64::LDRSHXpost:
    return AArch64::LDRSHXroW;
  case AArch64::LDRSWpost:
    return AArch64::LDRSWroW;
  case AArch64::LDRSpost:
    return AArch64::LDRSroW;
  case AArch64::LDRWpost:
    return AArch64::LDRWroW;
  case AArch64::LDRXpost:
    return AArch64::LDRXroW;
  case AArch64::STRBBpost:
    return AArch64::STRBBroW;
  case AArch64::STRBpost:
    return AArch64::STRBroW;
  case AArch64::STRDpost:
    return AArch64::STRDroW;
  case AArch64::STRHHpost:
    return AArch64::STRHHroW;
  case AArch64::STRHpost:
    return AArch64::STRHroW;
  case AArch64::STRQpost:
    return AArch64::STRQroW;
  case AArch64::STRSpost:
    return AArch64::STRSroW;
  case AArch64::STRWpost:
    return AArch64::STRWroW;
  case AArch64::STRXpost:
    return AArch64::STRXroW;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static bool canConvertToRoW(unsigned Op) {
  unsigned Shift;
  return convertUiToRoW(Op) != AArch64::INSTRUCTION_LIST_END ||
         convertPreToRoW(Op) != AArch64::INSTRUCTION_LIST_END ||
         convertPostToRoW(Op) != AArch64::INSTRUCTION_LIST_END ||
         convertRoXToRoW(Op, Shift) != AArch64::INSTRUCTION_LIST_END ||
         convertRoWToRoW(Op, Shift) != AArch64::INSTRUCTION_LIST_END;
}

static unsigned convertPrePostToBase(unsigned Op, bool &IsPre,
                                     bool &IsBaseNoOffset) {
  IsPre = false;
  IsBaseNoOffset = false;
  switch (Op) {
  case AArch64::LDPDpost:
    return AArch64::LDPDi;
  case AArch64::LDPDpre:
    IsPre = true;
    return AArch64::LDPDi;
  case AArch64::LDPQpost:
    return AArch64::LDPQi;
  case AArch64::LDPQpre:
    IsPre = true;
    return AArch64::LDPQi;
  case AArch64::LDPSWpost:
    return AArch64::LDPSWi;
  case AArch64::LDPSWpre:
    IsPre = true;
    return AArch64::LDPSWi;
  case AArch64::LDPSpost:
    return AArch64::LDPSi;
  case AArch64::LDPSpre:
    IsPre = true;
    return AArch64::LDPSi;
  case AArch64::LDPWpost:
    return AArch64::LDPWi;
  case AArch64::LDPWpre:
    IsPre = true;
    return AArch64::LDPWi;
  case AArch64::LDPXpost:
    return AArch64::LDPXi;
  case AArch64::LDPXpre:
    IsPre = true;
    return AArch64::LDPXi;
  case AArch64::STPDpost:
    return AArch64::STPDi;
  case AArch64::STPDpre:
    IsPre = true;
    return AArch64::STPDi;
  case AArch64::STPQpost:
    return AArch64::STPQi;
  case AArch64::STPQpre:
    IsPre = true;
    return AArch64::STPQi;
  case AArch64::STPSpost:
    return AArch64::STPSi;
  case AArch64::STPSpre:
    IsPre = true;
    return AArch64::STPSi;
  case AArch64::STPWpost:
    return AArch64::STPWi;
  case AArch64::STPWpre:
    IsPre = true;
    return AArch64::STPWi;
  case AArch64::STPXpost:
    return AArch64::STPXi;
  case AArch64::STPXpre:
    IsPre = true;
    return AArch64::STPXi;
  // case AArch64::STLRWpre:
  //   IsPre = true;
  //   IsBaseNoOffset = true;
  //   return AArch64::STPWi;
  // case AArch64::STLRXpre:
  //   IsPre = true;
  //   IsBaseNoOffset = true;
  //   return AArch64::STLRX;
  case AArch64::LD1i64_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1i64;
  case AArch64::LD2i64_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2i64;
  case AArch64::LD1i8_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1i8;
  case AArch64::LD1i16_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1i16;
  case AArch64::LD1i32_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1i32;
  case AArch64::LD2i8_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2i8;
  case AArch64::LD2i16_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2i16;
  case AArch64::LD2i32_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2i32;
  case AArch64::LD3i8_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3i8;
  case AArch64::LD3i16_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3i16;
  case AArch64::LD3i32_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3i32;
  case AArch64::LD3i64_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3i64;
  case AArch64::LD4i8_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4i8;
  case AArch64::LD4i16_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4i16;
  case AArch64::LD4i32_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4i32;
  case AArch64::LD4i64_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4i64;
  case AArch64::LD1Onev1d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Onev1d;
  case AArch64::LD1Onev2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Onev2s;
  case AArch64::LD1Onev4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Onev4h;
  case AArch64::LD1Onev8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Onev8b;
  case AArch64::LD1Onev2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Onev2d;
  case AArch64::LD1Onev4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Onev4s;
  case AArch64::LD1Onev8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Onev8h;
  case AArch64::LD1Onev16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Onev16b;
  case AArch64::LD1Rv1d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Rv1d;
  case AArch64::LD1Rv2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Rv2s;
  case AArch64::LD1Rv4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Rv4h;
  case AArch64::LD1Rv8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Rv8b;
  case AArch64::LD1Rv2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Rv2d;
  case AArch64::LD1Rv4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Rv4s;
  case AArch64::LD1Rv8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Rv8h;
  case AArch64::LD1Rv16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Rv16b;
  case AArch64::LD1Twov1d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Twov1d;
  case AArch64::LD1Twov2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Twov2s;
  case AArch64::LD1Twov4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Twov4h;
  case AArch64::LD1Twov8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Twov8b;
  case AArch64::LD1Twov2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Twov2d;
  case AArch64::LD1Twov4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Twov4s;
  case AArch64::LD1Twov8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Twov8h;
  case AArch64::LD1Twov16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Twov16b;
  case AArch64::LD1Threev1d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Threev1d;
  case AArch64::LD1Threev2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Threev2s;
  case AArch64::LD1Threev4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Threev4h;
  case AArch64::LD1Threev8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Threev8b;
  case AArch64::LD1Threev2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Threev2d;
  case AArch64::LD1Threev4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Threev4s;
  case AArch64::LD1Threev8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Threev8h;
  case AArch64::LD1Threev16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Threev16b;
  case AArch64::LD1Fourv1d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Fourv1d;
  case AArch64::LD1Fourv2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Fourv2s;
  case AArch64::LD1Fourv4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Fourv4h;
  case AArch64::LD1Fourv8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Fourv8b;
  case AArch64::LD1Fourv2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Fourv2d;
  case AArch64::LD1Fourv4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Fourv4s;
  case AArch64::LD1Fourv8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Fourv8h;
  case AArch64::LD1Fourv16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD1Fourv16b;
  case AArch64::LD2Twov2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Twov2s;
  case AArch64::LD2Twov4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Twov4s;
  case AArch64::LD2Twov8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Twov8b;
  case AArch64::LD2Twov2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Twov2d;
  case AArch64::LD2Twov4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Twov4h;
  case AArch64::LD2Twov8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Twov8h;
  case AArch64::LD2Twov16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Twov16b;
  case AArch64::LD2Rv1d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Rv1d;
  case AArch64::LD2Rv2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Rv2s;
  case AArch64::LD2Rv4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Rv4s;
  case AArch64::LD2Rv8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Rv8b;
  case AArch64::LD2Rv2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Rv2d;
  case AArch64::LD2Rv4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Rv4h;
  case AArch64::LD2Rv8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Rv8h;
  case AArch64::LD2Rv16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD2Rv16b;
  case AArch64::LD3Threev2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Threev2s;
  case AArch64::LD3Threev4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Threev4h;
  case AArch64::LD3Threev8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Threev8b;
  case AArch64::LD3Threev2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Threev2d;
  case AArch64::LD3Threev4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Threev4s;
  case AArch64::LD3Threev8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Threev8h;
  case AArch64::LD3Threev16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Threev16b;
  case AArch64::LD3Rv1d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Rv1d;
  case AArch64::LD3Rv2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Rv2s;
  case AArch64::LD3Rv4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Rv4h;
  case AArch64::LD3Rv8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Rv8b;
  case AArch64::LD3Rv2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Rv2d;
  case AArch64::LD3Rv4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Rv4s;
  case AArch64::LD3Rv8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Rv8h;
  case AArch64::LD3Rv16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD3Rv16b;
  case AArch64::LD4Fourv2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Fourv2s;
  case AArch64::LD4Fourv4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Fourv4h;
  case AArch64::LD4Fourv8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Fourv8b;
  case AArch64::LD4Fourv2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Fourv2d;
  case AArch64::LD4Fourv4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Fourv4s;
  case AArch64::LD4Fourv8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Fourv8h;
  case AArch64::LD4Fourv16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Fourv16b;
  case AArch64::LD4Rv1d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Rv1d;
  case AArch64::LD4Rv2s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Rv2s;
  case AArch64::LD4Rv4h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Rv4h;
  case AArch64::LD4Rv8b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Rv8b;
  case AArch64::LD4Rv2d_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Rv2d;
  case AArch64::LD4Rv4s_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Rv4s;
  case AArch64::LD4Rv8h_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Rv8h;
  case AArch64::LD4Rv16b_POST:
    IsBaseNoOffset = true;
    return AArch64::LD4Rv16b;
  case AArch64::ST1i64_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1i64;
  case AArch64::ST2i64_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2i64;
  case AArch64::ST1i8_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1i8;
  case AArch64::ST1i16_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1i16;
  case AArch64::ST1i32_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1i32;
  case AArch64::ST2i8_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2i8;
  case AArch64::ST2i16_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2i16;
  case AArch64::ST2i32_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2i32;
  case AArch64::ST3i8_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3i8;
  case AArch64::ST3i16_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3i16;
  case AArch64::ST3i32_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3i32;
  case AArch64::ST3i64_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3i64;
  case AArch64::ST4i8_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4i8;
  case AArch64::ST4i16_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4i16;
  case AArch64::ST4i32_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4i32;
  case AArch64::ST4i64_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4i64;
  case AArch64::ST1Onev1d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Onev1d;
  case AArch64::ST1Onev2s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Onev2s;
  case AArch64::ST1Onev4h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Onev4h;
  case AArch64::ST1Onev8b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Onev8b;
  case AArch64::ST1Onev2d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Onev2d;
  case AArch64::ST1Onev4s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Onev4s;
  case AArch64::ST1Onev8h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Onev8h;
  case AArch64::ST1Onev16b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Onev16b;
  case AArch64::ST1Twov1d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Twov1d;
  case AArch64::ST1Twov2s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Twov2s;
  case AArch64::ST1Twov4h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Twov4h;
  case AArch64::ST1Twov8b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Twov8b;
  case AArch64::ST1Twov2d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Twov2d;
  case AArch64::ST1Twov4s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Twov4s;
  case AArch64::ST1Twov8h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Twov8h;
  case AArch64::ST1Twov16b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Twov16b;
  case AArch64::ST1Threev1d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Threev1d;
  case AArch64::ST1Threev2s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Threev2s;
  case AArch64::ST1Threev4h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Threev4h;
  case AArch64::ST1Threev8b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Threev8b;
  case AArch64::ST1Threev2d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Threev2d;
  case AArch64::ST1Threev4s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Threev4s;
  case AArch64::ST1Threev8h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Threev8h;
  case AArch64::ST1Threev16b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Threev16b;
  case AArch64::ST1Fourv1d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Fourv1d;
  case AArch64::ST1Fourv2s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Fourv2s;
  case AArch64::ST1Fourv4h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Fourv4h;
  case AArch64::ST1Fourv8b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Fourv8b;
  case AArch64::ST1Fourv2d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Fourv2d;
  case AArch64::ST1Fourv4s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Fourv4s;
  case AArch64::ST1Fourv8h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Fourv8h;
  case AArch64::ST1Fourv16b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST1Fourv16b;
  case AArch64::ST2Twov2s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2Twov2s;
  case AArch64::ST2Twov4s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2Twov4s;
  case AArch64::ST2Twov8b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2Twov8b;
  case AArch64::ST2Twov2d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2Twov2d;
  case AArch64::ST2Twov4h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2Twov4h;
  case AArch64::ST2Twov8h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2Twov8h;
  case AArch64::ST2Twov16b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST2Twov16b;
  case AArch64::ST3Threev2s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3Threev2s;
  case AArch64::ST3Threev4h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3Threev4h;
  case AArch64::ST3Threev8b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3Threev8b;
  case AArch64::ST3Threev2d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3Threev2d;
  case AArch64::ST3Threev4s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3Threev4s;
  case AArch64::ST3Threev8h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3Threev8h;
  case AArch64::ST3Threev16b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST3Threev16b;
  case AArch64::ST4Fourv2s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4Fourv2s;
  case AArch64::ST4Fourv4h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4Fourv4h;
  case AArch64::ST4Fourv8b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4Fourv8b;
  case AArch64::ST4Fourv2d_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4Fourv2d;
  case AArch64::ST4Fourv4s_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4Fourv4s;
  case AArch64::ST4Fourv8h_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4Fourv8h;
  case AArch64::ST4Fourv16b_POST:
    IsBaseNoOffset = true;
    return AArch64::ST4Fourv16b;
  }
  return AArch64::INSTRUCTION_LIST_END;
}

static unsigned getPrePostScale(unsigned Op) {
  switch (Op) {
  case AArch64::LDPDpost:
  case AArch64::LDPDpre:
    return 8;
  case AArch64::LDPQpost:
  case AArch64::LDPQpre:
    return 16;
  case AArch64::LDPSWpost:
  case AArch64::LDPSWpre:
    return 4;
  case AArch64::LDPSpost:
  case AArch64::LDPSpre:
    return 4;
  case AArch64::LDPWpost:
  case AArch64::LDPWpre:
    return 4;
  case AArch64::LDPXpost:
  case AArch64::LDPXpre:
    return 8;
  case AArch64::STPDpost:
  case AArch64::STPDpre:
    return 8;
  case AArch64::STPQpost:
  case AArch64::STPQpre:
    return 16;
  case AArch64::STPSpost:
  case AArch64::STPSpre:
    return 4;
  case AArch64::STPWpost:
  case AArch64::STPWpre:
    return 4;
  case AArch64::STPXpost:
  case AArch64::STPXpre:
    return 8;
  }
  return 0;
}

static std::optional<MemInstInfo> getAtomicLoadStoreInfo(const MCInst &Inst) {
  int DestRegIdx;
  int BaseRegIdx;
  const int OffsetIdx = -1;
  const bool IsPrePost = false;
  bool IsPair = false;
  // (ST|LD)OPRegister: ld*, st*
  // CompareAndSwap(Pair)?(Unprvileged)?
  // Swap(LSUI)?
  // StoreRelease
  // LoadAcquire
  switch (Inst.getOpcode()) {
  // Compare-and-swap CAS(A)?(L)?(B|H|W|X)
  // MIR: Rs = op Rs, Rt, [Xn]
  case AArch64::CASB:
  case AArch64::CASH:
  case AArch64::CASW:
  case AArch64::CASX:
  case AArch64::CASAB:
  case AArch64::CASAH:
  case AArch64::CASAW:
  case AArch64::CASAX:
  case AArch64::CASALB:
  case AArch64::CASALH:
  case AArch64::CASALW:
  case AArch64::CASALX:
  case AArch64::CASLB:
  case AArch64::CASLH:
  case AArch64::CASLW:
  case AArch64::CASLX:
    DestRegIdx = 2;
    BaseRegIdx = 3;
    break;
  // rs1_rs2 = op rs1_rs2, rt1_rt2, [Xn]
  case AArch64::CASPW:
  case AArch64::CASPX:
  case AArch64::CASPAW:
  case AArch64::CASPAX:
  case AArch64::CASPALW:
  case AArch64::CASPALX:
  case AArch64::CASPLW:
  case AArch64::CASPLX:
    DestRegIdx = 2;
    BaseRegIdx = 3;
    IsPair = true;
    break;
  // swap SWP(A)?(L)?(B|H|W|X)
  // MIR: op Rs, Rt, [Xn]
  case AArch64::SWPB:
  case AArch64::SWPH:
  case AArch64::SWPW:
  case AArch64::SWPX:
  case AArch64::SWPAB:
  case AArch64::SWPAH:
  case AArch64::SWPAW:
  case AArch64::SWPAX:
  case AArch64::SWPALB:
  case AArch64::SWPALH:
  case AArch64::SWPALW:
  case AArch64::SWPALX:
  case AArch64::SWPLB:
  case AArch64::SWPLH:
  case AArch64::SWPLW:
  case AArch64::SWPLX:
    DestRegIdx = 1;
    BaseRegIdx = 2;
    break;
  // LD(ADD|CLR|EOR|SET)(A)?(L)?(B|H|W|X)
  // op Rs, Rt, [Xn]
  // if Rt == xzr, LD* aliases to ST*
  case AArch64::LDADDB:
  case AArch64::LDADDH:
  case AArch64::LDADDX:
  case AArch64::LDADDW:
  case AArch64::LDADDAB:
  case AArch64::LDADDAH:
  case AArch64::LDADDAW:
  case AArch64::LDADDAX:
  case AArch64::LDADDALB:
  case AArch64::LDADDALH:
  case AArch64::LDADDALW:
  case AArch64::LDADDALX:
  case AArch64::LDADDLB:
  case AArch64::LDADDLH:
  case AArch64::LDADDLX:
  case AArch64::LDADDLW:
    DestRegIdx = 1;
    BaseRegIdx = 2;
    break;
  case AArch64::LDCLRB:
  case AArch64::LDCLRH:
  case AArch64::LDCLRX:
  case AArch64::LDCLRW:
  case AArch64::LDCLRAB:
  case AArch64::LDCLRAH:
  case AArch64::LDCLRAW:
  case AArch64::LDCLRAX:
  case AArch64::LDCLRALB:
  case AArch64::LDCLRALH:
  case AArch64::LDCLRALW:
  case AArch64::LDCLRALX:
  case AArch64::LDCLRLB:
  case AArch64::LDCLRLH:
  case AArch64::LDCLRLX:
  case AArch64::LDCLRLW:
    DestRegIdx = 1;
    BaseRegIdx = 2;
    break;
  case AArch64::LDEORB:
  case AArch64::LDEORH:
  case AArch64::LDEORX:
  case AArch64::LDEORW:
  case AArch64::LDEORAB:
  case AArch64::LDEORAH:
  case AArch64::LDEORAW:
  case AArch64::LDEORAX:
  case AArch64::LDEORALB:
  case AArch64::LDEORALH:
  case AArch64::LDEORALW:
  case AArch64::LDEORALX:
  case AArch64::LDEORLB:
  case AArch64::LDEORLH:
  case AArch64::LDEORLX:
  case AArch64::LDEORLW:
    DestRegIdx = 1;
    BaseRegIdx = 2;
    break;
  case AArch64::LDSETB:
  case AArch64::LDSETH:
  case AArch64::LDSETX:
  case AArch64::LDSETW:
  case AArch64::LDSETAB:
  case AArch64::LDSETAH:
  case AArch64::LDSETAW:
  case AArch64::LDSETAX:
  case AArch64::LDSETALB:
  case AArch64::LDSETALH:
  case AArch64::LDSETALW:
  case AArch64::LDSETALX:
  case AArch64::LDSETLB:
  case AArch64::LDSETLH:
  case AArch64::LDSETLX:
  case AArch64::LDSETLW:
    DestRegIdx = 1;
    BaseRegIdx = 2;
    break;
  default:
    return std::nullopt;
  }
  return MemInstInfo{DestRegIdx, BaseRegIdx, OffsetIdx, IsPrePost, IsPair};
}
