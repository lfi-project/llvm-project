//===- llvm/MC/MCLFIExpander.h ----------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// This file was written by the Native Client and LFI authors.
//
//===----------------------------------------------------------------------===//
//
// This file declares the MCLFIExpander class. This is an abstract
// class that encapsulates the expansion logic for MCInsts.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC_MCLFIEXPANDER_H
#define LLVM_MC_MCLFIEXPANDER_H

#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {
class MCInst;
class MCSubtargetInfo;
class MCStreamer;

class MCLFIExpander {
private:
  MCContext &Ctx;
  bool Enabled = true;

protected:
  std::unique_ptr<MCInstrInfo> InstInfo;
  std::unique_ptr<MCRegisterInfo> RegInfo;

public:
  MCLFIExpander(MCContext &Ctx, std::unique_ptr<MCRegisterInfo> &&RI,
                std::unique_ptr<MCInstrInfo> &&II)
      : Ctx(Ctx), InstInfo(std::move(II)), RegInfo(std::move(RI)) {}

  void disable();
  void enable();
  bool isEnabled();

  bool isPseudo(const MCInst &Inst) const;

  bool isCall(const MCInst &Inst) const;
  bool isBranch(const MCInst &Inst) const;
  bool isIndirectBranch(const MCInst &Inst) const;
  bool isReturn(const MCInst &Inst) const;

  bool mayLoad(const MCInst &Inst) const;
  bool mayStore(const MCInst &Inst) const;

  bool mayModifyRegister(const MCInst &Inst, MCRegister Reg) const;

  virtual ~MCLFIExpander() = default;
  virtual bool expandInst(const MCInst &Inst, MCStreamer &Out,
                          const MCSubtargetInfo &STI) = 0;
};

} // namespace llvm
#endif
