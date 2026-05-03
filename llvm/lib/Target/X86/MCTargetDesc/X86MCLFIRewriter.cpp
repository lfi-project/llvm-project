//===- X86MCLFIRewriter.cpp -------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the X86MCLFIRewriter class, which rewrites X86-64
// instructions for LFI (Lightweight Fault Isolation) sandboxing.
//
// In this scheme the address space is split across two stack pointers:
//
//   * %rsp holds the *control flow stack* (CFS). It is populated only by
//     `call` (which pushes a return address) and consumed by `ret`. Because
//     every entry on it was pushed by an in-sandbox `call`, every value on
//     the CFS is automatically a valid in-sandbox return target, so `ret`
//     needs no extra check.
//
//   * %r13 holds the *data stack pointer* (DSP). All compiled code believes
//     it is using %rsp for the data stack; this rewriter retargets every
//     explicit %rsp use (push/pop, frame setup, mem operands, etc.) to %r13.
//
// %r13 must always hold an in-sandbox address. After any modification to
// %r13 we re-fold the sandbox base via `leaq (%r13, %r14), %r13`. Direct
// modifications use the 32-bit form (writes to %r13d) so the upper 32 bits
// are zeroed before re-folding.
//
//===----------------------------------------------------------------------===//

#include "X86MCLFIRewriter.h"
#include "X86BaseInfo.h"
#include "X86MCTargetDesc.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

// LFI reserved registers.
static constexpr MCRegister LFIBaseReg = X86::R14;
static constexpr MCRegister LFIScratchReg = X86::R11;
static constexpr MCRegister LFIDataStackReg = X86::R13;
static constexpr MCRegister LFITPReg = X86::R15;

// Byte offset into the context register file (pointed to by R15) where the
// thread pointer is stored.
static constexpr int TPOffset = 16;

// Byte offset into the context register file used as a scratch slot to save
// %rsp during the high-byte + %rsp encoding workaround. See
// rewriteHighByteRSPMem for details.
static constexpr int RSPSaveSlot = 24;

// Byte offset into the context register file used to pass the argument
// register for an LFI runtime call (currently only INCSSPQ). The runtime
// reads the value at this offset.
static constexpr int RtSlotOffset = 40;

// Forward-edge CFI alignment for valid indirect branch targets.
static constexpr int IndBranchAlignment = 32;

// Encoding of endbr64 (f3 0f 1e fa) as a 32-bit little-endian value.
static constexpr int32_t ENDBR64Encoding = static_cast<int32_t>(0xfa1e0ff3);

static cl::opt<bool>
    X86LFIHwEndbr("x86-lfi-hw-endbr",
                  cl::desc("Hardware endbr CFI is available; skip software "
                           "endbr comparison checks at indirect branches"),
                  cl::init(true));

static MCRegister getReg64(MCRegister Reg) {
  switch (Reg) {
  default:
    return getX86SubSuperRegister(Reg, 64, false);
  case X86::IP:
  case X86::EIP:
  case X86::RIP:
    return X86::RIP;
  }
}

static MCRegister getReg32(MCRegister Reg) {
  switch (Reg) {
  default:
    return getX86SubSuperRegister(Reg, 32, false);
  case X86::IP:
  case X86::EIP:
    return X86::EIP;
  case X86::RIP:
    llvm_unreachable("Trying to demote %rip");
  }
}

static bool isSyscall(const MCInst &Inst) {
  return Inst.getOpcode() == X86::SYSCALL;
}

static bool isDirectCall(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case X86::CALLpcrel32:
  case X86::CALL64pcrel32:
    return true;
  default:
    return false;
  }
}

static bool isPush(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case X86::PUSH64r:
  case X86::PUSH64rmm:
  case X86::PUSH64rmr:
  case X86::PUSH64i32:
  case X86::PUSH64i8:
  case X86::PUSH16r:
  case X86::PUSH16rmm:
  case X86::PUSH16i:
  case X86::PUSH16i8:
  case X86::PUSHF64:
  case X86::PUSHF16:
    return true;
  default:
    return false;
  }
}

static bool isPop(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case X86::POP64r:
  case X86::POP64rmm:
  case X86::POP16r:
  case X86::POP16rmm:
  case X86::POPF64:
  case X86::POPF16:
    return true;
  default:
    return false;
  }
}

static bool isLeave(const MCInst &Inst) {
  switch (Inst.getOpcode()) {
  case X86::LEAVE:
  case X86::LEAVE64:
    return true;
  default:
    return false;
  }
}

static bool isEnter(const MCInst &Inst) {
  return Inst.getOpcode() == X86::ENTER;
}

static bool isRDSSP(const MCInst &Inst) {
  return Inst.getOpcode() == X86::RDSSPQ;
}

static bool isINCSSP(const MCInst &Inst) {
  return Inst.getOpcode() == X86::INCSSPQ;
}

// Find the index of the first memory operand with %fs segment override.
// Returns -1 if not found.
static int findFSMemOperand(const MCInst &Inst, const MCInstrInfo &InstInfo) {
  const MCInstrDesc &Desc = InstInfo.get(Inst.getOpcode());
  for (unsigned I = 0, E = Desc.getNumOperands(); I < E; ++I) {
    if (Desc.operands()[I].OperandType == MCOI::OPERAND_MEMORY) {
      if (I + 4 < Inst.getNumOperands() && Inst.getOperand(I + 4).isReg() &&
          Inst.getOperand(I + 4).getReg() == X86::FS)
        return I;
      I += 4;
    }
  }
  return -1;
}

// Returns true if any explicit operand of Inst (register operand or memory
// operand base/index) refers to %rsp.
static bool referencesRSP(const MCInst &Inst) {
  for (unsigned I = 0, E = Inst.getNumOperands(); I < E; ++I) {
    const MCOperand &Op = Inst.getOperand(I);
    if (!Op.isReg())
      continue;
    MCRegister R = Op.getReg();
    if (R != X86::NoRegister && getReg64(R) == X86::RSP)
      return true;
  }
  return false;
}

// Returns true if Inst contains a high-byte register operand
// (%ah/%bh/%ch/%dh). These cannot be encoded with a REX prefix, so any
// instruction that combines them with an extended register (R8–R15) is
// invalid. After the rewriter substitutes %rsp with %r13, an existing
// high-byte operand can suddenly trigger this conflict.
static bool hasHighByteReg(const MCInst &Inst) {
  for (unsigned I = 0, E = Inst.getNumOperands(); I < E; ++I) {
    const MCOperand &Op = Inst.getOperand(I);
    if (!Op.isReg())
      continue;
    MCRegister R = Op.getReg();
    if (R == X86::AH || R == X86::BH || R == X86::CH || R == X86::DH)
      return true;
  }
  return false;
}

// Find the index of the first memory operand whose base or index register
// is %rsp (at any width). Returns -1 if none.
static int findRSPMemOperand(const MCInst &Inst,
                             const MCInstrInfo &InstInfo) {
  const MCInstrDesc &Desc = InstInfo.get(Inst.getOpcode());
  for (unsigned I = 0, E = Desc.getNumOperands(); I < E; ++I) {
    if (Desc.operands()[I].OperandType == MCOI::OPERAND_MEMORY) {
      if (I + 4 < Inst.getNumOperands()) {
        MCRegister Base = Inst.getOperand(I).getReg();
        MCRegister Index = Inst.getOperand(I + 2).getReg();
        if ((Base != X86::NoRegister && getReg64(Base) == X86::RSP) ||
            (Index != X86::NoRegister && getReg64(Index) == X86::RSP))
          return I;
      }
      I += 4;
    }
  }
  return -1;
}

// Returns true if the explicit destination operand of Inst (operand 0) is
// %rsp at any width. This catches `MOD ..., %rsp` instructions emitted by
// the compiler for stack frame setup.
static bool explicitlyDefsRSP(const MCInst &Inst, const MCInstrInfo &InstInfo) {
  const MCInstrDesc &Desc = InstInfo.get(Inst.getOpcode());
  if (Desc.getNumDefs() == 0)
    return false;
  const MCOperand &Op = Inst.getOperand(0);
  if (!Op.isReg() || Op.getReg() == X86::NoRegister)
    return false;
  return getReg64(Op.getReg()) == X86::RSP;
}

// Replace every %rsp register operand of Inst with the equivalent-width %r13
// register. Mem operand base/index registers are stored as register operands
// too, so this rewrites them as well.
static void replaceRSPWithR13(MCInst &Inst) {
  for (unsigned I = 0, E = Inst.getNumOperands(); I < E; ++I) {
    MCOperand &Op = Inst.getOperand(I);
    if (!Op.isReg())
      continue;
    MCRegister R = Op.getReg();
    switch (R) {
    case X86::RSP:
      Op.setReg(X86::R13);
      break;
    case X86::ESP:
      Op.setReg(X86::R13D);
      break;
    case X86::SP:
      Op.setReg(X86::R13W);
      break;
    case X86::SPL:
      Op.setReg(X86::R13B);
      break;
    default:
      break;
    }
  }
}

// Maps a 64-bit X86 opcode to its 32-bit counterpart, or 0 if no mapping
// exists. Used to demote an instruction that explicitly modifies %rsp so
// that it writes to %r13d instead, zeroing the upper 32 bits.
//
// Only the opcodes that the X86 backend can plausibly emit with %rsp as an
// explicit destination are handled. Add new entries as new patterns are
// observed.
static unsigned demoteOpcode(unsigned Opcode) {
  switch (Opcode) {
  case X86::ADD64rr:
    return X86::ADD32rr;
  case X86::ADD64ri8:
    return X86::ADD32ri8;
  case X86::ADD64ri32:
    return X86::ADD32ri;
  case X86::ADD64rm:
    return X86::ADD32rm;
  case X86::SUB64rr:
    return X86::SUB32rr;
  case X86::SUB64ri8:
    return X86::SUB32ri8;
  case X86::SUB64ri32:
    return X86::SUB32ri;
  case X86::SUB64rm:
    return X86::SUB32rm;
  case X86::AND64rr:
    return X86::AND32rr;
  case X86::AND64ri8:
    return X86::AND32ri8;
  case X86::AND64ri32:
    return X86::AND32ri;
  case X86::AND64rm:
    return X86::AND32rm;
  case X86::OR64rr:
    return X86::OR32rr;
  case X86::OR64ri8:
    return X86::OR32ri8;
  case X86::OR64ri32:
    return X86::OR32ri;
  case X86::OR64rm:
    return X86::OR32rm;
  case X86::XOR64rr:
    return X86::XOR32rr;
  case X86::XOR64ri8:
    return X86::XOR32ri8;
  case X86::XOR64ri32:
    return X86::XOR32ri;
  case X86::XOR64rm:
    return X86::XOR32rm;
  case X86::LEA64r:
    return X86::LEA32r;
  case X86::MOV64rr:
    return X86::MOV32rr;
  case X86::MOV64ri:
    return X86::MOV32ri;
  case X86::MOV64ri32:
    return X86::MOV32ri;
  case X86::MOV64rm:
    return X86::MOV32rm;
  default:
    return 0;
  }
}

// Demote Inst's opcode and all 64-bit register operands to their 32-bit
// counterparts in place. Returns false if the opcode can't be demoted.
static bool demoteInst(MCInst &Inst, const MCInstrInfo &InstInfo) {
  unsigned NewOpc = demoteOpcode(Inst.getOpcode());
  if (NewOpc == 0)
    return false;
  Inst.setOpcode(NewOpc);

  ArrayRef<MCOperandInfo> OpInfo = InstInfo.get(NewOpc).operands();
  for (unsigned I = 0, E = Inst.getNumOperands(); I < E; ++I) {
    if (I >= OpInfo.size())
      break;
    if (OpInfo[I].OperandType != MCOI::OPERAND_REGISTER)
      continue;
    MCOperand &Op = Inst.getOperand(I);
    if (!Op.isReg())
      continue;
    MCRegister R = Op.getReg();
    if (R == X86::NoRegister)
      continue;
    if (getReg64(R) == R)
      Op.setReg(getReg32(R));
  }
  return true;
}

// Emit `leaq (%r13, %r14), %r13` to re-fold the sandbox base into %r13
// after %r13d has been modified. Uses lea instead of add to avoid
// clobbering flags.
static void emitR13Reguard(MCStreamer &Out, const MCSubtargetInfo &STI) {
  MCInst Lea;
  Lea.setOpcode(X86::LEA64r);
  Lea.addOperand(MCOperand::createReg(LFIDataStackReg));
  Lea.addOperand(MCOperand::createReg(LFIDataStackReg));
  Lea.addOperand(MCOperand::createImm(1));
  Lea.addOperand(MCOperand::createReg(LFIBaseReg));
  Lea.addOperand(MCOperand::createImm(0));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Lea, STI);
}

// Emit `leal Disp(%r13d), %r13d` to adjust %r13 by a small immediate
// without clobbering flags. Caller must follow with emitR13Reguard.
static void emitR13Adjust(int32_t Disp, MCStreamer &Out,
                          const MCSubtargetInfo &STI) {
  MCInst Lea;
  Lea.setOpcode(X86::LEA32r);
  Lea.addOperand(MCOperand::createReg(X86::R13D));
  Lea.addOperand(MCOperand::createReg(X86::R13D));
  Lea.addOperand(MCOperand::createImm(1));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Lea.addOperand(MCOperand::createImm(Disp));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Lea, STI);
}

// Emit an LFI runtime call to the entry at RtOffset(%r14):
//
//   leaq .Ltmp(%rip), %r11
//   jmpq *RtOffset(%r14)
//   .Ltmp:
//
// The runtime preserves %r14/%r15 and the data-stack registers, and returns
// via the saved address in %r11.
static void emitLFICall(unsigned RtOffset, MCStreamer &Out,
                        const MCSubtargetInfo &STI) {
  MCSymbol *Symbol = Out.getContext().createTempSymbol();

  // leaq .Ltmp(%rip), %r11
  MCInst Lea;
  Lea.setOpcode(X86::LEA64r);
  Lea.addOperand(MCOperand::createReg(LFIScratchReg));
  Lea.addOperand(MCOperand::createReg(X86::RIP));
  Lea.addOperand(MCOperand::createImm(1));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Lea.addOperand(
      MCOperand::createExpr(MCSymbolRefExpr::create(Symbol, Out.getContext())));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Lea, STI);

  // jmpq *RtOffset(%r14)
  MCInst Jmp;
  Jmp.setOpcode(X86::JMP64m);
  Jmp.addOperand(MCOperand::createReg(LFIBaseReg));
  Jmp.addOperand(MCOperand::createImm(1));
  Jmp.addOperand(MCOperand::createReg(X86::NoRegister));
  Jmp.addOperand(MCOperand::createImm(RtOffset));
  Jmp.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Jmp, STI);

  Out.emitLabel(Symbol);
}

// Runtime entry-point offsets within the %r14 jump table.
static constexpr unsigned RtSyscallOffset = 0;
static constexpr unsigned RtIncSSPOffset = 32;

void X86::X86MCLFIRewriter::rewriteSyscall(const MCInst &Inst, MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {
  emitLFICall(RtSyscallOffset, Out, STI);
}

// Emit: movq TPOffset(%r15), %Reg
static void emitTPLoad(MCRegister Reg, MCStreamer &Out,
                       const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(X86::MOV64rm);
  Mov.addOperand(MCOperand::createReg(Reg));
  Mov.addOperand(MCOperand::createReg(LFITPReg));
  Mov.addOperand(MCOperand::createImm(1));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Mov.addOperand(MCOperand::createImm(TPOffset));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Mov, STI);
}

bool X86::X86MCLFIRewriter::isFSAccess(const MCInst &Inst) {
  return (mayLoad(Inst) || mayStore(Inst)) &&
         findFSMemOperand(Inst, *InstInfo) >= 0;
}

// Rewrite %fs-segment memory accesses to use the virtual thread pointer stored
// at TPOffset(%r15). The actual memory access is currently unsandboxed because
// load/store sandboxing is not yet supported. Example rewrites:
//
// movq %fs:0, %rax
// ->
// movq 16(%r15), %rax
//
// movq %fs:(%rdi), %rax
// ->
// movq 16(%r15), %rax
// movq (%rax, %rdi), %rax
//
// movq 8(%rdi, %rsi, 2), %rax
// ->
// movq 16(%r15), %rax
// leaq (%rax, %rdi), %rax
// movq 8(%rax, %rsi, 2), %rax
void X86::X86MCLFIRewriter::rewriteFSAccess(const MCInst &Inst, MCStreamer &Out,
                                            const MCSubtargetInfo &STI) {
  int MemIdx = findFSMemOperand(Inst, *InstInfo);
  assert(MemIdx >= 0);

  MCRegister BaseReg = Inst.getOperand(MemIdx).getReg();
  MCRegister IndexReg = Inst.getOperand(MemIdx + 2).getReg();
  bool HasBase = BaseReg != X86::NoRegister;
  bool HasIndex = IndexReg != X86::NoRegister;
  bool HasDisp = !Inst.getOperand(MemIdx + 3).isImm() ||
                 Inst.getOperand(MemIdx + 3).getImm() != 0;

  // %fs:0 -> TPOffset(%r15)
  if (!HasBase && !HasIndex && !HasDisp) {
    MCInst Modified(Inst);
    Modified.getOperand(MemIdx).setReg(LFITPReg);
    Modified.getOperand(MemIdx + 3).setImm(TPOffset);
    Modified.getOperand(MemIdx + 4).setReg(X86::NoRegister);
    return Out.emitInstruction(Modified, STI);
  }

  // Use the dest register as TP temporary when it is available and not used in
  // the addressing mode, otherwise use %r11.
  MCRegister TPDest = LFIScratchReg;
  if (MemIdx > 0 && Inst.getOperand(0).isReg()) {
    const MCInstrDesc &Desc = InstInfo->get(Inst.getOpcode());
    MCRegister DestReg = Inst.getOperand(0).getReg();
    if (Desc.getOperandConstraint(0, MCOI::TIED_TO) == -1 &&
        X86MCRegisterClasses[X86::GR64RegClassID].contains(DestReg) &&
        (!HasBase || DestReg != BaseReg) && (!HasIndex || DestReg != IndexReg))
      TPDest = DestReg;
  }

  emitTPLoad(TPDest, Out, STI);

  // Both slots occupied: fold base into TPDest via lea.
  if (HasBase && HasIndex) {
    MCInst Lea;
    Lea.setOpcode(X86::LEA64r);
    Lea.addOperand(MCOperand::createReg(TPDest));
    Lea.addOperand(MCOperand::createReg(TPDest));
    Lea.addOperand(MCOperand::createImm(1));
    Lea.addOperand(MCOperand::createReg(BaseReg));
    Lea.addOperand(MCOperand::createImm(0));
    Lea.addOperand(MCOperand::createReg(X86::NoRegister));
    Out.emitInstruction(Lea, STI);
  }

  MCInst Modified(Inst);
  Modified.getOperand(MemIdx).setReg(TPDest);
  if (HasBase && !HasIndex)
    Modified.getOperand(MemIdx + 2).setReg(BaseReg);
  Modified.getOperand(MemIdx + 4).setReg(X86::NoRegister);
  Out.emitInstruction(Modified, STI);
}

// Lazily emit the weak ``_lfi_trap`` symbol into a COMDAT
// ``.text_lfi_trap`` section so that all translation units share one copy.
// The symbol contains a single ``ud2`` (0f 0b).
MCSymbol *
X86::X86MCLFIRewriter::getOrEmitTrapSymbol(MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {
  if (LFITrapSymbol)
    return LFITrapSymbol;

  LFITrapSymbol = Out.getContext().getOrCreateSymbol("_lfi_trap");

  Out.pushSection();
  MCSection *TrapSec = Out.getContext().getELFSection(
      ".text_lfi_trap", ELF::SHT_PROGBITS,
      ELF::SHF_ALLOC | ELF::SHF_EXECINSTR | ELF::SHF_GROUP, 0, "_lfi_trap",
      /*IsComdat=*/true);
  Out.switchSection(TrapSec);
  Out.emitSymbolAttribute(LFITrapSymbol, MCSA_Weak);
  Out.emitLabel(LFITrapSymbol);
  // ud2
  Out.emitBytes(StringRef("\x0f\x0b", 2));
  Out.popSection();

  return LFITrapSymbol;
}

// Emit the forward-edge CFI check at an aligned indirect-branch target:
//
//   .p2align 1
//   cs cmpl $0xfa1e0ff3, (%r14, %rX)
//   jne _lfi_trap
//
// The .p2align 1 directive and the cs prefix together ensure that the
// embedded immediate (the endbr64 encoding) never lands at a 32-byte-aligned
// address, so it cannot be reached as a forged landing pad.
void X86::X86MCLFIRewriter::emitCFICheck(MCRegister Reg, MCStreamer &Out,
                                         const MCSubtargetInfo &STI) {
  MCSymbol *TrapSym = getOrEmitTrapSymbol(Out, STI);

  Out.emitCodeAlignment(llvm::Align(2), &STI);

  // cs prefix
  MCInst CSPrefix;
  CSPrefix.setOpcode(X86::CS_PREFIX);
  Out.emitInstruction(CSPrefix, STI);

  // cmpl $0xfa1e0ff3, (%r14, %rX)
  MCInst Cmp;
  Cmp.setOpcode(X86::CMP32mi);
  Cmp.addOperand(MCOperand::createReg(LFIBaseReg));      // Base = %r14
  Cmp.addOperand(MCOperand::createImm(1));               // Scale
  Cmp.addOperand(MCOperand::createReg(getReg64(Reg)));   // Index = %rX
  Cmp.addOperand(MCOperand::createImm(0));               // Displacement
  Cmp.addOperand(MCOperand::createReg(X86::NoRegister)); // Segment
  Cmp.addOperand(MCOperand::createImm(ENDBR64Encoding));
  Out.emitInstruction(Cmp, STI);

  // jne _lfi_trap
  MCInst Jne;
  Jne.setOpcode(X86::JCC_1);
  Jne.addOperand(
      MCOperand::createExpr(MCSymbolRefExpr::create(TrapSym, Out.getContext())));
  Jne.addOperand(MCOperand::createImm(X86::COND_NE));
  Out.emitInstruction(Jne, STI);
}

// Emit the alignment, optional forward-edge CFI check, and base-relocation
// for an indirect-branch target held in Reg:
//
//   andl $-32, %eX
//   .p2align 1                  ; emitted by emitCFICheck
//   cs cmpl $0xfa1e0ff3, (%r14, %rX)
//   jne _lfi_trap
//   addq %r14, %rX
void X86::X86MCLFIRewriter::emitSandboxBranchReg(MCRegister Reg,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  // andl $-32, %eX
  MCInst AndInst;
  AndInst.setOpcode(X86::AND32ri8);
  MCOperand Target32 = MCOperand::createReg(getReg32(Reg));
  AndInst.addOperand(Target32);
  AndInst.addOperand(Target32);
  AndInst.addOperand(MCOperand::createImm(-IndBranchAlignment));
  Out.emitInstruction(AndInst, STI);

  if (!X86LFIHwEndbr)
    emitCFICheck(Reg, Out, STI);

  // addq %r14, %rX
  MCInst Add;
  Add.setOpcode(X86::ADD64rr);
  MCOperand Target64 = MCOperand::createReg(getReg64(Reg));
  Add.addOperand(Target64);
  Add.addOperand(Target64);
  Add.addOperand(MCOperand::createReg(LFIBaseReg));
  Out.emitInstruction(Add, STI);
}

// CALL64pcrel32 is encoded as a 5-byte E8 + rel32 sequence.
static constexpr unsigned DirectCallSize = 5;

// Pad before a direct call to a `returns_twice` function so that the byte
// after the call lands on a 32-byte boundary, then emit ENDBR64 there. This
// is the address that setjmp saves into the jmp_buf and that longjmp later
// reaches via an indirect jmp; the alignment + endbr64 lets that indirect
// jmp pass the forward-edge CFI check.
void X86::X86MCLFIRewriter::expandReturnsTwiceDirectCall(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  Out.emitCodeAlignment(llvm::Align(IndBranchAlignment), &STI);
  Out.emitFill(IndBranchAlignment - DirectCallSize, /*FillValue=*/0x90);
  Out.emitInstruction(Inst, STI);

  MCInst Endbr;
  Endbr.setOpcode(X86::ENDBR64);
  Out.emitInstruction(Endbr, STI);
}

// Expand an indirect call or jump (register or memory operand variant) into
// the sandboxed sequence above followed by the original branch instruction.
void X86::X86MCLFIRewriter::expandIndirectBranch(const MCInst &Inst,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  MCRegister Target;
  if (mayLoad(Inst)) {
    // Indirect jmp/call through memory — load address into the scratch
    // register first.
    Target = LFIScratchReg;
    MCInst Mov;
    Mov.setOpcode(X86::MOV64rm);
    Mov.addOperand(MCOperand::createReg(getReg64(Target)));
    Mov.addOperand(Inst.getOperand(0));
    Mov.addOperand(Inst.getOperand(1));
    Mov.addOperand(Inst.getOperand(2));
    Mov.addOperand(Inst.getOperand(3));
    Mov.addOperand(Inst.getOperand(4));
    doRewriteInst(Mov, Out, STI);
  } else {
    Target = Inst.getOperand(0).getReg();
  }

  emitSandboxBranchReg(Target, Out, STI);

  MCInst Branch;
  Branch.setOpcode(isCall(Inst) ? X86::CALL64r : X86::JMP64r);
  Branch.addOperand(MCOperand::createReg(getReg64(Target)));
  Out.emitInstruction(Branch, STI);
}

// Returns the "natural" size in bytes of a push/pop targeting the given
// opcode. This is the amount %rsp would have moved by under the original
// semantics, and therefore the amount %r13 must move by in our rewrite.
static int pushPopSize(unsigned Opcode) {
  switch (Opcode) {
  case X86::PUSH16r:
  case X86::PUSH16rmm:
  case X86::PUSH16i:
  case X86::PUSH16i8:
  case X86::POP16r:
  case X86::POP16rmm:
  case X86::PUSHF16:
  case X86::POPF16:
    return 2;
  default:
    return 8;
  }
}

// Returns the appropriate-width MOV opcode for storing a register to memory
// using a slot of the given size in bytes.
static unsigned getMovRegToMemOpcode(int Size) {
  switch (Size) {
  case 8:
    return X86::MOV64mr;
  case 2:
    return X86::MOV16mr;
  default:
    llvm_unreachable("unsupported push slot size");
  }
}

// Returns the appropriate-width MOV opcode for loading a register from memory
// using a slot of the given size in bytes.
static unsigned getMovMemToRegOpcode(int Size) {
  switch (Size) {
  case 8:
    return X86::MOV64rm;
  case 2:
    return X86::MOV16rm;
  default:
    llvm_unreachable("unsupported pop slot size");
  }
}

// Returns the appropriate-width MOV opcode for storing an immediate to memory
// using a slot of the given size in bytes.
static unsigned getMovImmToMemOpcode(int Size) {
  switch (Size) {
  case 8:
    return X86::MOV64mi32;
  case 2:
    return X86::MOV16mi;
  default:
    llvm_unreachable("unsupported push imm slot size");
  }
}

// Returns the same-width sub-register of LFIScratchReg used for transferring
// a value through the scratch register during push/pop expansion.
static MCRegister getScratchSubReg(int Size) {
  switch (Size) {
  case 8:
    return X86::R11;
  case 2:
    return X86::R11W;
  default:
    llvm_unreachable("unsupported scratch sub-register size");
  }
}

// Emit a store of `SrcReg` to the top of the data stack:
//   movX %SrcReg, (%r13)
static void emitStoreToDataStack(MCRegister SrcReg, int Size, MCStreamer &Out,
                                 const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(getMovRegToMemOpcode(Size));
  Mov.addOperand(MCOperand::createReg(LFIDataStackReg));   // base
  Mov.addOperand(MCOperand::createImm(1));                 // scale
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));   // index
  Mov.addOperand(MCOperand::createImm(0));                 // disp
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));   // seg
  Mov.addOperand(MCOperand::createReg(SrcReg));            // src
  Out.emitInstruction(Mov, STI);
}

// Emit a load from the top of the data stack into `DstReg`:
//   movX (%r13), %DstReg
static void emitLoadFromDataStack(MCRegister DstReg, int Size, MCStreamer &Out,
                                  const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(getMovMemToRegOpcode(Size));
  Mov.addOperand(MCOperand::createReg(DstReg));            // dst
  Mov.addOperand(MCOperand::createReg(LFIDataStackReg));   // base
  Mov.addOperand(MCOperand::createImm(1));                 // scale
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));   // index
  Mov.addOperand(MCOperand::createImm(0));                 // disp
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));   // seg
  Out.emitInstruction(Mov, STI);
}

// Expand a push instruction onto the data stack:
//
//   pushq %rX            ->  leal -8(%r13d), %r13d
//                            leaq (%r13, %r14), %r13
//                            movq %rX, (%r13)
//
//   pushq $imm           ->  leal -8(%r13d), %r13d
//                            leaq (%r13, %r14), %r13
//                            movq $imm, (%r13)
//
//   pushq mem            ->  movq mem, %r11   (rewritten recursively)
//                            (then expand as pushq %r11)
//
// The "natural" call/ret stack remains %rsp; this redirects only the data
// stack push.
void X86::X86MCLFIRewriter::expandPush(const MCInst &Inst, MCStreamer &Out,
                                       const MCSubtargetInfo &STI) {
  unsigned Opc = Inst.getOpcode();
  int Size = pushPopSize(Opc);

  // PUSHF/PUSHF16: read flags into %r11 via pushf/pop, then push %r11. We
  // can't avoid touching %rsp here, but we use it only as a transient slot
  // for the flags and immediately undo the modification.
  if (Opc == X86::PUSHF64 || Opc == X86::PUSHF16) {
    return error(Inst, "pushf/pushfq is not yet supported under LFI");
  }

  // The original PUSH semantics are read-source then decrement-then-store, so
  // the value to be pushed must be evaluated using the OLD %r13 (= old %rsp).
  // We materialize the value into a register or commit to an immediate before
  // touching %r13, then adjust %r13 and store. Otherwise sequences like
  // `push N(%rsp)` would observe the wrong base after the adjustment.
  switch (Opc) {
  case X86::PUSH64r:
  case X86::PUSH16r: {
    // Operand 0 is the source register. If it is %rsp/%sp itself, snapshot
    // the OLD %r13 into the scratch register before the adjustment so that
    // the stored value matches `push %rsp` semantics.
    MCRegister SrcReg = Inst.getOperand(0).getReg();
    if (SrcReg == X86::RSP || SrcReg == X86::SP) {
      MCRegister Scratch = getScratchSubReg(Size);
      MCInst Snap;
      Snap.setOpcode(Size == 8 ? X86::MOV64rr : X86::MOV16rr);
      Snap.addOperand(MCOperand::createReg(Scratch));
      Snap.addOperand(MCOperand::createReg(Size == 8 ? X86::R13 : X86::R13W));
      Out.emitInstruction(Snap, STI);
      SrcReg = Scratch;
    }
    emitR13Adjust(-Size, Out, STI);
    emitR13Reguard(Out, STI);
    emitStoreToDataStack(SrcReg, Size, Out, STI);
    return;
  }
  case X86::PUSH64i32:
  case X86::PUSH64i8:
  case X86::PUSH16i:
  case X86::PUSH16i8: {
    // Imm form: the value is independent of %r13.
    emitR13Adjust(-Size, Out, STI);
    emitR13Reguard(Out, STI);
    MCInst Mov;
    Mov.setOpcode(getMovImmToMemOpcode(Size));
    Mov.addOperand(MCOperand::createReg(LFIDataStackReg));   // base
    Mov.addOperand(MCOperand::createImm(1));                 // scale
    Mov.addOperand(MCOperand::createReg(X86::NoRegister));   // index
    Mov.addOperand(MCOperand::createImm(0));                 // disp
    Mov.addOperand(MCOperand::createReg(X86::NoRegister));   // seg
    Mov.addOperand(Inst.getOperand(0));                      // imm
    Out.emitInstruction(Mov, STI);
    return;
  }
  case X86::PUSH64rmm:
  case X86::PUSH64rmr:
  case X86::PUSH16rmm: {
    // Memory operand form: load through the scratch register using the OLD
    // %r13 (the inner load goes through doRewriteInst so %rsp→%r13 + %fs
    // handling apply). Only after the load do we decrement %r13.
    MCRegister ScratchReg = getScratchSubReg(Size);
    MCInst Load;
    Load.setOpcode(getMovMemToRegOpcode(Size));
    Load.addOperand(MCOperand::createReg(ScratchReg));
    Load.addOperand(Inst.getOperand(0)); // base
    Load.addOperand(Inst.getOperand(1)); // scale
    Load.addOperand(Inst.getOperand(2)); // index
    Load.addOperand(Inst.getOperand(3)); // disp
    Load.addOperand(Inst.getOperand(4)); // seg
    doRewriteInst(Load, Out, STI);
    emitR13Adjust(-Size, Out, STI);
    emitR13Reguard(Out, STI);
    emitStoreToDataStack(ScratchReg, Size, Out, STI);
    return;
  }
  default:
    return error(Inst, "unsupported push variant under LFI");
  }
}

// Expand a pop instruction off the data stack:
//
//   popq %rX             ->  movq (%r13), %rX
//                            leal 8(%r13d), %r13d
//                            leaq (%r13, %r14), %r13
//
//   popq mem             ->  movq (%r13), %r11
//                            leal 8(%r13d), %r13d
//                            leaq (%r13, %r14), %r13
//                            movq %r11, mem    (rewritten recursively)
//
// If the destination register is %rsp itself (`pop %rsp`), we route through
// %r13d + reguard so that the resulting %rsp value is sandbox-relative;
// however since %rsp is the CFS in our scheme, this is almost certainly
// wrong code and we report an error.
void X86::X86MCLFIRewriter::expandPop(const MCInst &Inst, MCStreamer &Out,
                                      const MCSubtargetInfo &STI) {
  unsigned Opc = Inst.getOpcode();
  int Size = pushPopSize(Opc);

  if (Opc == X86::POPF64 || Opc == X86::POPF16) {
    return error(Inst, "popf/popfq is not yet supported under LFI");
  }

  switch (Opc) {
  case X86::POP64r:
  case X86::POP16r: {
    MCRegister DstReg = Inst.getOperand(0).getReg();
    if (DstReg == X86::RSP || DstReg == X86::SP) {
      // Popping into the data-stack pointer itself.
      MCRegister Sub = (DstReg == X86::SP) ? X86::R13W : X86::R13;
      emitLoadFromDataStack(Sub, Size, Out, STI);
      // The loaded value is treated as a 32-bit data-stack offset; we drop
      // the upper bits via a 32-bit mov-to-self before re-folding.
      MCInst Mov;
      Mov.setOpcode(X86::MOV32rr);
      Mov.addOperand(MCOperand::createReg(X86::R13D));
      Mov.addOperand(MCOperand::createReg(X86::R13D));
      Out.emitInstruction(Mov, STI);
      emitR13Adjust(Size, Out, STI);
      emitR13Reguard(Out, STI);
      return;
    }
    emitLoadFromDataStack(DstReg, Size, Out, STI);
    emitR13Adjust(Size, Out, STI);
    emitR13Reguard(Out, STI);
    return;
  }
  case X86::POP64rmm:
  case X86::POP16rmm: {
    MCRegister ScratchReg = getScratchSubReg(Size);
    emitLoadFromDataStack(ScratchReg, Size, Out, STI);
    emitR13Adjust(Size, Out, STI);
    emitR13Reguard(Out, STI);

    MCInst Store;
    Store.setOpcode(getMovRegToMemOpcode(Size));
    Store.addOperand(Inst.getOperand(0)); // base
    Store.addOperand(Inst.getOperand(1)); // scale
    Store.addOperand(Inst.getOperand(2)); // index
    Store.addOperand(Inst.getOperand(3)); // disp
    Store.addOperand(Inst.getOperand(4)); // seg
    Store.addOperand(MCOperand::createReg(ScratchReg));
    doRewriteInst(Store, Out, STI);
    return;
  }
  default:
    return error(Inst, "unsupported pop variant under LFI");
  }
}

// Expand `leave` (`leave64`) into its mov+pop equivalents on the data stack:
//
//   leave   ->  mov %rbp, %r13d   (clear upper, then re-fold)
//               leaq (%r13, %r14), %r13
//               popq %rbp         (re-expanded as the data-stack pop)
void X86::X86MCLFIRewriter::expandLeave(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  // mov %ebp, %r13d
  MCInst Mov;
  Mov.setOpcode(X86::MOV32rr);
  Mov.addOperand(MCOperand::createReg(X86::R13D));
  Mov.addOperand(MCOperand::createReg(X86::EBP));
  Out.emitInstruction(Mov, STI);
  emitR13Reguard(Out, STI);

  // popq %rbp -- re-enter the rewriter so it gets the data-stack pop.
  MCInst Pop;
  Pop.setOpcode(X86::POP64r);
  Pop.addOperand(MCOperand::createReg(X86::RBP));
  doRewriteInst(Pop, Out, STI);
}

// Rewrite an instruction that explicitly modifies %rsp by:
//   1. Replacing every %rsp-family operand with the corresponding %r13.
//   2. Demoting the opcode and 64-bit register operands to 32-bit, so the
//      destination write zeros the upper 32 bits of %r13 and leaves the
//      sandbox base out of the calculation.
//   3. Emitting a `leaq (%r13, %r14), %r13` to re-fold the sandbox base.
//
// For instructions with no 32-bit demotion (e.g., the 8-bit/16-bit mov
// patterns the compiler should never use on %rsp), we report an error
// rather than guess.
void X86::X86MCLFIRewriter::rewriteRSPModify(const MCInst &Inst,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI) {
  MCInst Modified(Inst);
  replaceRSPWithR13(Modified);
  if (!demoteInst(Modified, *InstInfo))
    return error(Inst, "cannot demote %rsp-modifying instruction under LFI");
  Out.emitInstruction(Modified, STI);
  emitR13Reguard(Out, STI);
}

// Workaround for the high-byte + REX encoding restriction. The naive rewrite
// of e.g. `or %ch, 1(%rsp)` to `or %ch, 1(%r13)` is not encodable: %r13
// requires a REX prefix, but high-byte registers (%ah/%bh/%ch/%dh) cannot
// coexist with REX. Instead, route the address through %rsp itself:
//
//   <op> %Xh, mem(%rsp,...)
//   ->
//   movq %rsp, RSPSaveSlot(%r15)         ; stash the CFS pointer
//   leaq mem(%r13,...), %rsp             ; compute the target address into %rsp
//   <op> %Xh, (%rsp)                     ; %rsp + %Xh: both legacy, no REX
//   movq RSPSaveSlot(%r15), %rsp         ; restore the CFS pointer
//
// %rsp briefly stops pointing at the CFS during this sequence, which would
// trip a signal handler that needed the CFS — we don't support sandbox
// signals in this scheme regardless.
void X86::X86MCLFIRewriter::rewriteHighByteRSPMem(const MCInst &Inst,
                                                  MCStreamer &Out,
                                                  const MCSubtargetInfo &STI) {
  int MemIdx = findRSPMemOperand(Inst, *InstInfo);
  if (MemIdx < 0) {
    // No %rsp memory operand. The high-byte conflict can't have been our
    // fault; fall back to a plain operand substitution.
    MCInst Modified(Inst);
    replaceRSPWithR13(Modified);
    return Out.emitInstruction(Modified, STI);
  }

  // movq %rsp, RSPSaveSlot(%r15)
  MCInst SaveRsp;
  SaveRsp.setOpcode(X86::MOV64mr);
  SaveRsp.addOperand(MCOperand::createReg(LFITPReg));
  SaveRsp.addOperand(MCOperand::createImm(1));
  SaveRsp.addOperand(MCOperand::createReg(X86::NoRegister));
  SaveRsp.addOperand(MCOperand::createImm(RSPSaveSlot));
  SaveRsp.addOperand(MCOperand::createReg(X86::NoRegister));
  SaveRsp.addOperand(MCOperand::createReg(X86::RSP));
  Out.emitInstruction(SaveRsp, STI);

  // leaq <mem with rsp→r13>, %rsp
  MCInst Lea;
  Lea.setOpcode(X86::LEA64r);
  Lea.addOperand(MCOperand::createReg(X86::RSP));
  for (int I = 0; I < 5; ++I) {
    MCOperand Op = Inst.getOperand(MemIdx + I);
    if (Op.isReg()) {
      switch (Op.getReg()) {
      case X86::RSP:
        Op.setReg(X86::R13);
        break;
      case X86::ESP:
        Op.setReg(X86::R13D);
        break;
      case X86::SP:
        Op.setReg(X86::R13W);
        break;
      case X86::SPL:
        Op.setReg(X86::R13B);
        break;
      default:
        break;
      }
    }
    Lea.addOperand(Op);
  }
  Out.emitInstruction(Lea, STI);

  // Modified instruction with the memory operand collapsed to (%rsp).
  MCInst Modified(Inst);
  Modified.getOperand(MemIdx + 0).setReg(X86::RSP);
  Modified.getOperand(MemIdx + 1).setImm(1);
  Modified.getOperand(MemIdx + 2).setReg(X86::NoRegister);
  Modified.getOperand(MemIdx + 3).setImm(0);
  Modified.getOperand(MemIdx + 4).setReg(X86::NoRegister);
  Out.emitInstruction(Modified, STI);

  // movq RSPSaveSlot(%r15), %rsp
  MCInst RestoreRsp;
  RestoreRsp.setOpcode(X86::MOV64rm);
  RestoreRsp.addOperand(MCOperand::createReg(X86::RSP));
  RestoreRsp.addOperand(MCOperand::createReg(LFITPReg));
  RestoreRsp.addOperand(MCOperand::createImm(1));
  RestoreRsp.addOperand(MCOperand::createReg(X86::NoRegister));
  RestoreRsp.addOperand(MCOperand::createImm(RSPSaveSlot));
  RestoreRsp.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(RestoreRsp, STI);
}

// In the dual-stack scheme %rsp is the shadow stack pointer, so RDSSPQ is
// just a direct read of %rsp. RDSSPQ has tied src=dst, so the input MCInst
// stores the destination twice; we only consume operand 0.
//
//   rdsspq %rX  →  movq %rsp, %rX
void X86::X86MCLFIRewriter::expandRDSSP(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(X86::MOV64rr);
  Mov.addOperand(MCOperand::createReg(Inst.getOperand(0).getReg()));
  Mov.addOperand(MCOperand::createReg(X86::RSP));
  Out.emitInstruction(Mov, STI);
}

// INCSSPQ adjusts the shadow stack pointer; in our scheme that means %rsp
// itself, which the sandbox is not allowed to modify directly. Pass the
// count to the runtime, which validates it against the CFS upper bound and
// updates %rsp on our behalf.
//
//   incsspq %rX  →  movq %rX, RtSlotOffset(%r15)
//                   leaq .Ltmp(%rip), %r11
//                   jmpq *RtIncSSPOffset(%r14)
//                   .Ltmp:
//
// The slot store happens before emitLFICall's leaq so that an %r11 source
// operand is consumed before %r11 is clobbered by the call sequence.
void X86::X86MCLFIRewriter::expandINCSSP(const MCInst &Inst, MCStreamer &Out,
                                         const MCSubtargetInfo &STI) {
  MCInst Store;
  Store.setOpcode(X86::MOV64mr);
  Store.addOperand(MCOperand::createReg(LFITPReg));
  Store.addOperand(MCOperand::createImm(1));
  Store.addOperand(MCOperand::createReg(X86::NoRegister));
  Store.addOperand(MCOperand::createImm(RtSlotOffset));
  Store.addOperand(MCOperand::createReg(X86::NoRegister));
  Store.addOperand(MCOperand::createReg(Inst.getOperand(0).getReg()));
  Out.emitInstruction(Store, STI);

  emitLFICall(RtIncSSPOffset, Out, STI);
}

void X86::X86MCLFIRewriter::doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
  if (mayModifyRegister(Inst, LFIBaseReg) ||
      mayModifyRegister(Inst, LFITPReg) ||
      mayModifyRegister(Inst, LFIDataStackReg))
    return error(Inst, "illegal modification of reserved LFI register");

  if (isSyscall(Inst))
    return rewriteSyscall(Inst, Out, STI);

  // Indirect branches are dispatched before isFSAccess so that an indirect
  // call/jmp through an %fs-segment memory operand (e.g. callq *%fs:0) goes
  // through expandIndirectBranch first; the inner load it emits will then
  // hit rewriteFSAccess on its own.
  if (isIndirectBranch(Inst) || (isCall(Inst) && !isDirectCall(Inst)))
    return expandIndirectBranch(Inst, Out, STI);

  if (isDirectCall(Inst) && (Inst.getFlags() & X86::IP_LFI_RETURNS_TWICE))
    return expandReturnsTwiceDirectCall(Inst, Out, STI);

  // call/ret/direct-call all leave %rsp untouched here (it is the CFS).
  if (isCall(Inst) || isReturn(Inst)) {
    if (isFSAccess(Inst))
      return rewriteFSAccess(Inst, Out, STI);
    return Out.emitInstruction(Inst, STI);
  }

  // Push/pop expand to data-stack operations on %r13.
  if (isPush(Inst))
    return expandPush(Inst, Out, STI);
  if (isPop(Inst))
    return expandPop(Inst, Out, STI);
  if (isLeave(Inst))
    return expandLeave(Inst, Out, STI);
  if (isEnter(Inst))
    return error(Inst, "'enter' is not supported under LFI");

  // CET shadow-stack instructions read/write %rsp (the CFS pointer in our
  // scheme). RDSSPQ becomes a direct register read; INCSSPQ goes through a
  // runtime call so the new %rsp can be bounds-checked.
  if (isRDSSP(Inst))
    return expandRDSSP(Inst, Out, STI);
  if (isINCSSP(Inst))
    return expandINCSSP(Inst, Out, STI);

  if (isFSAccess(Inst))
    return rewriteFSAccess(Inst, Out, STI);

  // Explicit %rsp-modifying instructions (e.g. `subq $N, %rsp`,
  // `mov %r13, %rsp`, `lea N(%rsp,...), %rsp`) need both the operand
  // rename and a re-guard.
  if (explicitlyDefsRSP(Inst, *InstInfo))
    return rewriteRSPModify(Inst, Out, STI);

  // %rsp appears only as a source (memory operand base/index, or as a
  // read-only register operand). Just substitute %r13 in place — except
  // when a high-byte register is also present, in which case %r13's REX
  // requirement would clash with the high-byte encoding restriction and
  // we have to route the address through %rsp itself.
  if (referencesRSP(Inst)) {
    if (hasHighByteReg(Inst))
      return rewriteHighByteRSPMem(Inst, Out, STI);
    MCInst Modified(Inst);
    replaceRSPWithR13(Modified);
    return Out.emitInstruction(Modified, STI);
  }

  // Pass through all other instructions unchanged.
  Out.emitInstruction(Inst, STI);
}

bool X86::X86MCLFIRewriter::rewriteInst(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  // The guard prevents rewrite-recursion when we emit instructions from inside
  // the rewriter (such instructions should not be rewritten).
  if (!Enabled || Guard)
    return false;
  Guard = true;

  // Eagerly emit the trap symbol on first use, so the section switch doesn't
  // land in the middle of a sandbox sequence.
  if (!X86LFIHwEndbr && !LFITrapSymbol)
    getOrEmitTrapSymbol(Out, STI);

  doRewriteInst(Inst, Out, STI);

  Guard = false;
  return true;
}
