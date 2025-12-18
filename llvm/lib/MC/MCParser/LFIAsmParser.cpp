//===- LFIAsmParser.cpp - LFI Assembly Parser -----------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file was written by the LFI and Native Client authors.
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCLFIExpander.h"
#include "llvm/MC/MCParser/MCAsmParserExtension.h"
#include "llvm/MC/MCStreamer.h"

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
    addDirectiveHandler<&LFIAsmParser::parseExpandDisable>(".lfi_no_expand");
    addDirectiveHandler<&LFIAsmParser::parseExpandEnable>(".lfi_expand");
  }

  /// ::= {.lfi_no_expand}
  bool parseExpandDisable(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    if (getLexer().isNot(AsmToken::EndOfStatement))
      return TokError("unexpected token");
    Lex();

    Expander->disable();

    return false;
  }

  /// ::= {.lfi_expand}
  bool parseExpandEnable(StringRef Directive, SMLoc Loc) {
    getParser().checkForValidSection();
    if (getLexer().isNot(AsmToken::EndOfStatement))
      return TokError("unexpected token");
    Lex();

    Expander->enable();

    return false;
  }
};

namespace llvm {
MCAsmParserExtension *createLFIAsmParser(MCLFIExpander *Exp) {
  return new LFIAsmParser(Exp);
}
} // namespace llvm
