//===- AArch64MCLFIExpander.h -----------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the Native Client authors, modified for LFI.
//
//===----------------------------------------------------------------------===//
//
// This file declares the AArch64MCLFIExpander class, the AArch64 specific
// subclass of MCLFIExpander.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_MC_AARCH64MCLFIEXPANDER_H
#define LLVM_MC_AARCH64MCLFIEXPANDER_H

#include "Utils/AArch64BaseInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {
class MCContext;
class MCInst;
class MCStreamer;
class MCSubtargetInfo;

namespace AArch64 {
class AArch64MCLFIExpander : public MCLFIExpander {
public:
  AArch64MCLFIExpander(MCContext &Ctx, std::unique_ptr<MCRegisterInfo> &&RI,
                    std::unique_ptr<MCInstrInfo> &&II)
      : MCLFIExpander(Ctx, std::move(RI), std::move(II)) {}

  bool expandInst(const MCInst &Inst, MCStreamer &Out,
                  const MCSubtargetInfo &STI) override;

protected:
  bool isValidScratchRegister(MCRegister Reg) const override;

private:
  bool Guard = false; // recursion guard

  MCRegister getScratch();

  void expandIndirectBranch(const MCInst &Inst, MCStreamer &Out,
                            const MCSubtargetInfo &STI, bool isCall);

  void expandCall(const MCInst &Inst, MCStreamer &Out,
                  const MCSubtargetInfo &STI);

  void expandReturn(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI);

  bool mayModifyStack(const MCInst &Inst);

  bool mayModifyReserved(const MCInst &Inst);

  bool mayModifyLR(const MCInst &Inst);

  void expandControlFlow(const MCInst &Inst, MCStreamer &Out,
                         const MCSubtargetInfo &STI);

  void expandStackModification(const MCInst &Inst, MCStreamer &Out,
                               const MCSubtargetInfo &STI);

  void expandLRModification(const MCInst &Inst, MCStreamer &Out,
                                 const MCSubtargetInfo &STI);

  void expandPrefetch(const MCInst &Inst, MCStreamer &Out,
                      const MCSubtargetInfo &STI);

  void expandLoadStore(const MCInst &Inst, MCStreamer &Out,
                       const MCSubtargetInfo &STI);

  void expandLoadStoreBasic(const MCInst &Inst, MemInstInfo &InstInfo,
      MCStreamer &Out, const MCSubtargetInfo &STI);

  void expandLoadStoreRoW(const MCInst &Inst, MemInstInfo &InstInfo,
      MCStreamer &Out, const MCSubtargetInfo &STI);

  void expandSyscall(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI);

  void expandTLSRead(const MCInst &Inst, MCStreamer &Out,
                     const MCSubtargetInfo &STI);

  void expandTLSWrite(const MCInst &Inst, MCStreamer &Out,
                      const MCSubtargetInfo &STI);

  void doExpandInst(const MCInst &Inst, MCStreamer &Out,
                    const MCSubtargetInfo &STI);

  enum LFICallType {
    LFISyscall,
    LFITLSRead,
    LFITLSWrite,
  };

  void emitLFICall(LFICallType CallType, MCStreamer &Out,
                   const MCSubtargetInfo &STI);
};
} // namespace AArch64
} // namespace llvm
#endif
