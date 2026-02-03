//===- lib/MC/MCHLFI.cpp - HLFI-specific MC implementation ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements HLFI-specific MC support.
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCHLFI.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCLFIRewriter.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Alignment.h"
#include "llvm/TargetParser/Triple.h"

static const char NoteNamespace[] = "HLFI";

namespace llvm {

void initializeHLFIMCStreamer(MCStreamer &Streamer, MCContext &Ctx,
                               const Triple &TheTriple) {
  assert(TheTriple.isHLFI());
  const char *NoteName;
  const char *NoteArch;
  switch (TheTriple.getArch()) {
  case Triple::aarch64:
    NoteName = ".note.HLFI.ABI.aarch64";
    NoteArch = "aarch64";
    break;
  default:
    report_fatal_error("Unsupported architecture for HLFI");
  }

  std::string Error;
  const Target *TheTarget = TargetRegistry::lookupTarget(TheTriple, Error);
  assert(TheTarget != nullptr);

  // Create the HLFI MC rewriter via the target registry.
  // The factory function will check if HLFI features are enabled and create
  // the appropriate rewriter.
  TheTarget->createMCLFIRewriter(
      Streamer,
      std::unique_ptr<MCRegisterInfo>(TheTarget->createMCRegInfo(TheTriple)),
      std::unique_ptr<MCInstrInfo>(TheTarget->createMCInstrInfo()));

  // Emit an ELF Note section which identifies HLFI object files.
  MCSectionELF *Note = Ctx.getELFSection(NoteName, ELF::SHT_NOTE,
                                         ELF::SHF_ALLOC | ELF::SHF_GROUP, 0,
                                         NoteName, /*IsComdat=*/true);

  Streamer.pushSection();
  Streamer.switchSection(Note);
  Streamer.emitIntValue(strlen(NoteNamespace) + 1, 4);
  Streamer.emitIntValue(strlen(NoteArch) + 1, 4);
  Streamer.emitIntValue(ELF::NT_VERSION, 4);
  Streamer.emitBytes(NoteNamespace);
  Streamer.emitIntValue(0, 1); // NUL terminator
  Streamer.emitValueToAlignment(Align(4));
  Streamer.emitBytes(NoteArch);
  Streamer.emitIntValue(0, 1); // NUL terminator
  Streamer.emitValueToAlignment(Align(4));
  Streamer.popSection();
}

} // namespace llvm
