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
static constexpr MCRegister LFITPReg = X86::R15;

// Byte offsets into the context register file (pointed to by R15).
//
//   16 : thread pointer
//   24 : temporary save slot used by the SCS prologue
//   32 : shadow call stack pointer
//   40 : argument/return slot for LFI runtime calls
static constexpr int TPOffset = 16;
static constexpr int SCSTempOffset = 24;
static constexpr int SCSOffset = 32;
static constexpr int RtSlotOffset = 40;

// Runtime entry-point offsets within the %r14 jump table.
static constexpr unsigned RtSyscallOffset = 0;
static constexpr unsigned RtSCSUnwindOffset = 48;

// Forward-edge CFI alignment for valid indirect branch targets.
static constexpr int IndBranchAlignment = 32;

// Encoding of endbr64 (f3 0f 1e fa) as a 32-bit little-endian value.
static constexpr int32_t ENDBR64Encoding = static_cast<int32_t>(0xfa1e0ff3);

static cl::opt<bool>
    X86LFIHwEndbr("x86-lfi-hw-endbr",
                  cl::desc("Hardware endbr CFI is available; skip software "
                           "endbr comparison checks at indirect branches"),
                  cl::init(false));

static cl::opt<bool>
    X86LFIHwShstk("x86-lfi-hw-shstk",
                  cl::desc("Hardware shadow call stack is available; skip "
                           "software SCS rewrites"),
                  cl::init(false));

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

// Emit an LFI runtime call to the entry at RtOffset(%r14):
//
//   leaq .Ltmp(%rip), %r11
//   jmpq *RtOffset(%r14)
//   .Ltmp:
//
// The runtime preserves %r15 and %r14 and restores control via %r11; only
// %r11 is guaranteed clobbered. Arguments and return values flow through
// RtSlotOffset(%r15) (caller is responsible for setting/reading it).
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

void X86::X86MCLFIRewriter::rewriteSyscall(const MCInst &Inst, MCStreamer &Out,
                                           const MCSubtargetInfo &STI) {
  emitLFICall(RtSyscallOffset, Out, STI);
}

// Emit: movq Offset(%r15), %DstReg
static void emitMovFromR15Slot(MCRegister DstReg, int Offset, MCStreamer &Out,
                               const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(X86::MOV64rm);
  Mov.addOperand(MCOperand::createReg(DstReg));
  Mov.addOperand(MCOperand::createReg(LFITPReg));
  Mov.addOperand(MCOperand::createImm(1));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Mov.addOperand(MCOperand::createImm(Offset));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Mov, STI);
}

// Emit: movq %SrcReg, Offset(%r15)
static void emitMovToR15Slot(MCRegister SrcReg, int Offset, MCStreamer &Out,
                             const MCSubtargetInfo &STI) {
  MCInst Mov;
  Mov.setOpcode(X86::MOV64mr);
  Mov.addOperand(MCOperand::createReg(LFITPReg));
  Mov.addOperand(MCOperand::createImm(1));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Mov.addOperand(MCOperand::createImm(Offset));
  Mov.addOperand(MCOperand::createReg(X86::NoRegister));
  Mov.addOperand(MCOperand::createReg(SrcReg));
  Out.emitInstruction(Mov, STI);
}

// Emit: movq TPOffset(%r15), %Reg
static void emitTPLoad(MCRegister Reg, MCStreamer &Out,
                       const MCSubtargetInfo &STI) {
  emitMovFromR15Slot(Reg, TPOffset, Out, STI);
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
//
// Only used in hardware-shstk mode. In software-shstk mode the equivalent
// alignment + endbr64 is emitted at the SCS RetLabel by emitShadowCallEpilogue.
void X86::X86MCLFIRewriter::expandReturnsTwiceDirectCall(
    const MCInst &Inst, MCStreamer &Out, const MCSubtargetInfo &STI) {
  Out.emitCodeAlignment(llvm::Align(IndBranchAlignment), &STI);
  Out.emitFill(IndBranchAlignment - DirectCallSize, /*FillValue=*/0x90);
  Out.emitInstruction(Inst, STI);

  MCInst Endbr;
  Endbr.setOpcode(X86::ENDBR64);
  Out.emitInstruction(Endbr, STI);
}

// Push the return address of the upcoming call onto the shadow call stack.
//
//   movq %rsp, SCSTempOffset(%r15)   ; save real rsp to temp slot
//   movq SCSOffset(%r15), %rsp       ; load SCS pointer
//   leaq RetLabel(%rip), %r11        ; compute return address
//   pushq %r11                       ; push return address onto SCS
//   movq %rsp, SCSOffset(%r15)       ; save updated SCS pointer
//   movq SCSTempOffset(%r15), %rsp   ; restore real rsp
//
// Returns the RetLabel that is bound after the call by emitShadowCallEpilogue.
MCSymbol *
X86::X86MCLFIRewriter::emitShadowCallPrologue(MCStreamer &Out,
                                              const MCSubtargetInfo &STI) {
  MCSymbol *RetLabel = Out.getContext().createTempSymbol();

  emitMovToR15Slot(X86::RSP, SCSTempOffset, Out, STI);
  emitMovFromR15Slot(X86::RSP, SCSOffset, Out, STI);

  // leaq RetLabel(%rip), %r11
  MCInst Lea;
  Lea.setOpcode(X86::LEA64r);
  Lea.addOperand(MCOperand::createReg(LFIScratchReg));
  Lea.addOperand(MCOperand::createReg(X86::RIP));
  Lea.addOperand(MCOperand::createImm(1));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Lea.addOperand(MCOperand::createExpr(
      MCSymbolRefExpr::create(RetLabel, Out.getContext())));
  Lea.addOperand(MCOperand::createReg(X86::NoRegister));
  Out.emitInstruction(Lea, STI);

  // pushq %r11
  MCInst Push;
  Push.setOpcode(X86::PUSH64r);
  Push.addOperand(MCOperand::createReg(LFIScratchReg));
  Out.emitInstruction(Push, STI);

  emitMovToR15Slot(X86::RSP, SCSOffset, Out, STI);
  emitMovFromR15Slot(X86::RSP, SCSTempOffset, Out, STI);

  return RetLabel;
}

// Bind the RetLabel after a call and undo the SCS prologue's bookkeeping:
//
//   RetLabel:
//   [ endbr64 ]                            ; iff ReturnsTwice
//   movq %rsp, SCSOffset(%r15)             ; save SCS ptr (callee's ret popped it)
//   movq %r11, %rsp                        ; restore real rsp from r11
//   popq %r11                              ; discard real return addr from real stack
//
// The callee's ret (in expandReturn) saves the real rsp into %r11 before
// switching to the SCS, so %r11 holds the real rsp when control reaches
// RetLabel. For ReturnsTwice calls, expandDirectCall has already padded
// before the callq so the label here lands on a 32-byte boundary; the
// endbr64 at that boundary is the CFI landing pad both for setjmp's normal
// return and for longjmp's indirect jump back into the call site.
void X86::X86MCLFIRewriter::emitShadowCallEpilogue(MCSymbol *RetLabel,
                                                   MCStreamer &Out,
                                                   const MCSubtargetInfo &STI,
                                                   bool ReturnsTwice) {
  Out.emitLabel(RetLabel);

  if (ReturnsTwice) {
    MCInst Endbr;
    Endbr.setOpcode(X86::ENDBR64);
    Out.emitInstruction(Endbr, STI);
  }

  emitMovToR15Slot(X86::RSP, SCSOffset, Out, STI);

  // movq %r11, %rsp
  MCInst RestoreRsp;
  RestoreRsp.setOpcode(X86::MOV64rr);
  RestoreRsp.addOperand(MCOperand::createReg(X86::RSP));
  RestoreRsp.addOperand(MCOperand::createReg(LFIScratchReg));
  Out.emitInstruction(RestoreRsp, STI);

  // popq %r11
  MCInst Pop;
  Pop.setOpcode(X86::POP64r);
  Pop.addOperand(MCOperand::createReg(LFIScratchReg));
  Out.emitInstruction(Pop, STI);
}

// Expand a direct call. In software-shstk mode, wrap with the SCS
// prologue/epilogue. In hardware-shstk mode, the call passes through; only
// returns_twice callees need the post-call alignment + endbr64.
void X86::X86MCLFIRewriter::expandDirectCall(const MCInst &Inst,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo &STI) {
  bool ReturnsTwice = Inst.getFlags() & X86::IP_LFI_RETURNS_TWICE;

  if (X86LFIHwShstk) {
    if (ReturnsTwice)
      return expandReturnsTwiceDirectCall(Inst, Out, STI);
    Out.emitInstruction(Inst, STI);
    return;
  }

  MCSymbol *RetLabel = emitShadowCallPrologue(Out, STI);
  if (ReturnsTwice) {
    // Pad so the byte after the callq lands on a 32-byte boundary; the
    // epilogue then binds RetLabel and emits endbr64 at exactly that
    // boundary. The address callq pushes onto the real stack therefore
    // matches RetLabel (the address the SCS prologue pushed) and is itself
    // a valid CFI landing pad, so a libc setjmp that reads the real-stack
    // return address and a longjmp that indirect-jumps to it both pass the
    // forward-edge CFI check.
    Out.emitCodeAlignment(llvm::Align(IndBranchAlignment), &STI);
    Out.emitFill(IndBranchAlignment - DirectCallSize, /*FillValue=*/0x90);
  }
  Out.emitInstruction(Inst, STI);
  emitShadowCallEpilogue(RetLabel, Out, STI, ReturnsTwice);
}

// Expand an indirect call or jump (register or memory operand variant) into
// the sandboxed sequence above followed by the original branch instruction.
// In software-shstk mode an indirect *call* is wrapped in the SCS
// prologue/epilogue. The prologue runs before the target load because both
// the prologue and the load clobber %r11.
void X86::X86MCLFIRewriter::expandIndirectBranch(const MCInst &Inst,
                                                 MCStreamer &Out,
                                                 const MCSubtargetInfo &STI) {
  bool IsCall = isCall(Inst);

  MCSymbol *RetLabel = nullptr;
  if (!X86LFIHwShstk && IsCall)
    RetLabel = emitShadowCallPrologue(Out, STI);

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
  Branch.setOpcode(IsCall ? X86::CALL64r : X86::JMP64r);
  Branch.addOperand(MCOperand::createReg(getReg64(Target)));
  Out.emitInstruction(Branch, STI);

  if (!X86LFIHwShstk && IsCall)
    emitShadowCallEpilogue(RetLabel, Out, STI);
}

// Expand a return through the shadow call stack:
//
//   [ addq $imm, %rsp ]              ; iff RETI{32,64}
//   movq %rsp, %r11                  ; save real rsp; the caller's epilogue
//                                    ;  reads it back from %r11
//   movq SCSOffset(%r15), %rsp       ; switch rsp to the SCS top
//   ret                               ; pops SCS top into %rip
//
// In hardware-shstk mode the original ret is emitted unchanged.
void X86::X86MCLFIRewriter::expandReturn(const MCInst &Inst, MCStreamer &Out,
                                         const MCSubtargetInfo &STI) {
  if (X86LFIHwShstk) {
    Out.emitInstruction(Inst, STI);
    return;
  }

  // For ret $imm, fold the immediate into the real rsp before stashing it.
  unsigned Opcode = Inst.getOpcode();
  if ((Opcode == X86::RETI32 || Opcode == X86::RETI64) &&
      Inst.getNumOperands() > 0) {
    MCInst Add;
    Add.setOpcode(X86::ADD64ri32);
    MCOperand StackPointer = MCOperand::createReg(X86::RSP);
    Add.addOperand(StackPointer);
    Add.addOperand(StackPointer);
    Add.addOperand(Inst.getOperand(0));
    Out.emitInstruction(Add, STI);
  }

  // movq %rsp, %r11
  MCInst SaveRsp;
  SaveRsp.setOpcode(X86::MOV64rr);
  SaveRsp.addOperand(MCOperand::createReg(LFIScratchReg));
  SaveRsp.addOperand(MCOperand::createReg(X86::RSP));
  Out.emitInstruction(SaveRsp, STI);

  // movq SCSOffset(%r15), %rsp
  emitMovFromR15Slot(X86::RSP, SCSOffset, Out, STI);

  // retq
  MCInst Ret;
  Ret.setOpcode(X86::RET64);
  Out.emitInstruction(Ret, STI);
}

// Expand rdsspq into a direct load of the SCS pointer slot.
//
//   rdsspq %rX  →  movq SCSOffset(%r15), %rX
//
// rdsspq's destination operand is tied src=dst, so the MCInst has the same
// register in both slots; we only consume operand 0. In hardware-shstk mode
// the instruction passes through unchanged.
void X86::X86MCLFIRewriter::expandRDSSP(const MCInst &Inst, MCStreamer &Out,
                                        const MCSubtargetInfo &STI) {
  if (X86LFIHwShstk) {
    Out.emitInstruction(Inst, STI);
    return;
  }

  emitMovFromR15Slot(Inst.getOperand(0).getReg(), SCSOffset, Out, STI);
}

// Expand incsspq %rX into an LFI runtime call to SCS unwind. The unwind
// count is passed through the runtime-call slot at RtSlotOffset(%r15):
//
//   incsspq %rX  →  movq %rX, RtSlotOffset(%r15)
//                   leaq .Ltmp(%rip), %r11
//                   jmpq *RtSCSUnwindOffset(%r14)
//                   .Ltmp:
//
// The slot store happens before emitLFICall's leaq so that an %r11 source
// operand is read before %r11 is clobbered. In hardware-shstk mode the
// instruction passes through unchanged.
void X86::X86MCLFIRewriter::expandINCSSP(const MCInst &Inst, MCStreamer &Out,
                                         const MCSubtargetInfo &STI) {
  if (X86LFIHwShstk) {
    Out.emitInstruction(Inst, STI);
    return;
  }

  emitMovToR15Slot(Inst.getOperand(0).getReg(), RtSlotOffset, Out, STI);
  emitLFICall(RtSCSUnwindOffset, Out, STI);
}

void X86::X86MCLFIRewriter::doRewriteInst(const MCInst &Inst, MCStreamer &Out,
                                          const MCSubtargetInfo &STI) {
  if (mayModifyRegister(Inst, LFIBaseReg) || mayModifyRegister(Inst, LFITPReg))
    return error(Inst, "illegal modification of reserved LFI register");

  if (isSyscall(Inst))
    return rewriteSyscall(Inst, Out, STI);

  // Indirect branches are dispatched before isFSAccess so that an indirect
  // call/jmp through an %fs-segment memory operand (e.g. callq *%fs:0) goes
  // through expandIndirectBranch first; the inner load it emits will then
  // hit rewriteFSAccess on its own.
  if (isIndirectBranch(Inst) || (isCall(Inst) && !isDirectCall(Inst)))
    return expandIndirectBranch(Inst, Out, STI);

  if (isDirectCall(Inst))
    return expandDirectCall(Inst, Out, STI);

  if (isReturn(Inst))
    return expandReturn(Inst, Out, STI);

  if (isRDSSP(Inst))
    return expandRDSSP(Inst, Out, STI);

  if (isINCSSP(Inst))
    return expandINCSSP(Inst, Out, STI);

  if (isFSAccess(Inst))
    return rewriteFSAccess(Inst, Out, STI);

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
