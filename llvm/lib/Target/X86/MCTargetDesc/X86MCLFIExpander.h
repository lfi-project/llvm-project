//===- X86MCLFIExpander.h - X86 LFI Expander -------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the Native Client authors.
//
//===----------------------------------------------------------------------===//
//
// This file declares the X86MCLFIExpander class, the X86 specific
// subclass of MCLFIExpander.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_MC_X86MCNACLEXPANDER_H
#define LLVM_MC_X86MCNACLEXPANDER_H

#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {
class MCContext;
class MCStreamer;
class MCSubtargetInfo;

namespace X86 {
class X86MCLFIExpander : public MCLFIExpander {
public:
  X86MCLFIExpander(MCContext &Ctx, std::unique_ptr<MCRegisterInfo> &&RI,
                    std::unique_ptr<MCInstrInfo> &&II)
      : MCLFIExpander(Ctx, std::move(RI), std::move(II)) {}

  bool expandInst(const MCInst &Inst, MCStreamer &Out,
                  const MCSubtargetInfo &STI) override;
protected:
  bool isValidScratchRegister(MCRegister Reg) const override;

private:
  bool Guard = false; // recursion guard
  SmallVector<MCInst, 4> Prefixes;

  void emitPrefixes(MCStreamer &Out, const MCSubtargetInfo &STI);

  void expandDirectCall(const MCInst &Inst, MCStreamer &Out,
                        const MCSubtargetInfo &STI);

  void expandIndirectBranch(const MCInst &Inst, MCStreamer &Out,
                            const MCSubtargetInfo &STI);

  void expandReturn(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI);

  void expandSyscall(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI);

  void expandTLSRead(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI);

  void expandLoadStore(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI, bool EmitPrefixes);

  void expandStringOperation(const MCInst &Inst, MCStreamer &Out,
           const MCSubtargetInfo &STI, bool EmitPrefixes);

  void doExpandInst(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI, bool EmitPrefixes);

  void expandExplicitStackManipulation(MCRegister StackReg, const MCInst &Inst,
                                       MCStreamer &Out,
                                       const MCSubtargetInfo &STI,
                                       bool EmitPrefixes);

  void emitSandboxMemOp(MCInst &Inst, int MemIdx, MCRegister ScratchReg,
                        MCStreamer &Out, const MCSubtargetInfo &STI);

  bool emitSandboxMemOps(MCInst &Inst, MCRegister ScratchReg, MCStreamer &Out,
                         const MCSubtargetInfo &STI, bool EmitInstructions);

  void emitInstruction(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI, bool EmitPrefixes);

  void emitSandboxBranchReg(MCRegister Reg, MCStreamer &Out,
                            const MCSubtargetInfo &STI);
  void emitIndirectJumpReg(MCRegister Reg, MCStreamer &Out,
                           const MCSubtargetInfo &STI);
  void emitIndirectCallReg(MCRegister Reg, MCStreamer &Out,
                           const MCSubtargetInfo &STI);

  enum LFICallType {
    LFISyscall,
    LFITLSRead,
    LFITLSWrite,
  };

  void emitLFICall(LFICallType CallType, MCStreamer &Out,
                   const MCSubtargetInfo &STI);
};
}
}
#endif
