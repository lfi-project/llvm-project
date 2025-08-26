//===- LFIAsmParser.cpp - LFI Assembly Parser -----------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file was written by the Native Client authors, modified for LFI.
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCAsmParserExtension.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCRegister.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

class LFIAsmParser : public MCAsmParserExtension {
  MCLFIExpander *Expander;
  template <bool (LFIAsmParser::*HandlerMethod)(StringRef, SMLoc)>
  void addDirectiveHandler(StringRef Directive) {
    MCAsmParser::ExtensionDirectiveHandler Handler =
        std::make_pair(this, HandleDirective<LFIAsmParser, HandlerMethod>);

    getParser().addDirectiveHandler(Directive, Handler);
  }

public:
  LFIAsmParser(MCLFIExpander *Exp) : Expander(Exp) {}
  void Initialize(MCAsmParser &Parser) override {
    // Call the base implementation.
    MCAsmParserExtension::Initialize(Parser);
    addDirectiveHandler<&LFIAsmParser::ParseScratch>(".scratch");
    addDirectiveHandler<&LFIAsmParser::ParseUnscratch>(".scratch_clear");
    addDirectiveHandler<&LFIAsmParser::ParseExpandDisable>(".no_expand");
    addDirectiveHandler<&LFIAsmParser::ParseExpandEnable>(".expand");
    addDirectiveHandler<&LFIAsmParser::ParseGuard>(".guard");
    addDirectiveHandler<&LFIAsmParser::ParseGuardEnd>(".guard_end");
    addDirectiveHandler<&LFIAsmParser::ParseBBStart>(".bb_start");
    addDirectiveHandler<&LFIAsmParser::ParseBBEnd>(".bb_end");
  }

  /// ::= {.scratch} reg
  bool ParseScratch(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    MCRegister RegNo;
    const char *kInvalidOptionError =
        "expected register name after '.scratch' directive";

    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      if (getParser().getTargetParser().parseRegister(RegNo, Loc, Loc))
        return Error(Loc, kInvalidOptionError);

      else if (getLexer().isNot(AsmToken::EndOfStatement))
        return Error(Loc, kInvalidOptionError);
    } else {
      return Error(Loc, kInvalidOptionError);
    }
    Lex();

    if (Expander->addScratchReg(RegNo))
      return Error(Loc, "Register can't be used as a scratch register");
    return false;
  }

  /// ::= {.scratch_clear}
  bool ParseUnscratch(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    if (getLexer().isNot(AsmToken::EndOfStatement))
      return TokError("unexpected token in '.scratch_clear' directive");
    Lex();

    Expander->clearScratchRegs();

    return false;
  }

  /// ::= {.no_expand}
  bool ParseExpandDisable(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    if (getLexer().isNot(AsmToken::EndOfStatement))
      return TokError("unexpected token in '.no_expand' directive");
    Lex();

    Expander->disable();

    return false;
  }

  /// ::= {.expand}
  bool ParseExpandEnable(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    if (getLexer().isNot(AsmToken::EndOfStatement))
      return TokError("unexpected token in '.expand' directive");
    Lex();

    Expander->enable();

    return false;
  }

  /// ::= {.bb_start}
  bool ParseBBStart(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    if (getLexer().isNot(AsmToken::EndOfStatement))
      return TokError("unexpected token in '.bb_start' directive");
    Lex();

    Expander->startBB(getStreamer(), getParser().getTargetParser().getSTI());

    return false;
  }

  /// ::= {.bb_end}
  bool ParseBBEnd(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    if (getLexer().isNot(AsmToken::EndOfStatement))
      return TokError("unexpected token in '.bb_end' directive");
    Lex();

    Expander->endBB(getStreamer(), getParser().getTargetParser().getSTI());

    return false;
  }

  /// ::= {.guard} reg reg
  bool ParseGuard(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    MCRegister GuardRegNo, RegNo;
    const char *kInvalidOptionError =
        "expected register name after '.guard' directive";

    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      if (getParser().getTargetParser().parseRegister(GuardRegNo, Loc, Loc))
        return Error(Loc, kInvalidOptionError);
      if (getParser().getTargetParser().parseRegister(RegNo, Loc, Loc))
        return Error(Loc, kInvalidOptionError);

      else if (getLexer().isNot(AsmToken::EndOfStatement))
        return Error(Loc, kInvalidOptionError);
    } else {
      return Error(Loc, kInvalidOptionError);
    }
    Lex();

    if (Expander->guard(GuardRegNo, RegNo))
      return Error(Loc, "Invalid registers for .guard");
    return false;
  }

  /// ::= {.guard_end} reg
  bool ParseGuardEnd(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    MCRegister RegNo;
    const char *kInvalidOptionError =
        "expected register name after '.guard_end' directive";

    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      if (getParser().getTargetParser().parseRegister(RegNo, Loc, Loc))
        return Error(Loc, kInvalidOptionError);

      else if (getLexer().isNot(AsmToken::EndOfStatement))
        return Error(Loc, kInvalidOptionError);
    } else {
      return Error(Loc, kInvalidOptionError);
    }
    Lex();

    if (Expander->guardEnd(RegNo))
      return Error(Loc, "Invalid register for .guard_end");
    return false;
  }
};

namespace llvm {
MCAsmParserExtension *createLFIAsmParser(MCLFIExpander *Exp) {
  return new LFIAsmParser(Exp);
}
} // namespace llvm
