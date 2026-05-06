//===-- LFILinux.cpp - LFI ToolChain Implementations ------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "LFILinux.h"
#include "clang/Basic/Sanitizers.h"
#include "clang/Driver/Driver.h"

using namespace clang::driver;
using namespace clang::driver::toolchains;
using namespace llvm::opt;

ToolChain::CXXStdlibType LFILinux::GetDefaultCXXStdlibType() const {
  return ToolChain::CST_Libstdcxx;
}

void LFILinux::AddCXXStdlibLibArgs(const ArgList &Args,
                                   ArgStringList &CmdArgs) const {
  ToolChain::AddCXXStdlibLibArgs(Args, CmdArgs);
  CmdArgs.push_back("-lc++abi");
}

clang::SanitizerMask LFILinux::getDefaultSanitizers() const {
  // SafeStack is the LFI x86-64 backward-edge CFI mechanism: the safe stack
  // holds return addresses (and provably-safe locals), and the LFI rewriter
  // bounds %rsp to the safe-stack region. Enable it by default so users do
  // not have to remember -fsanitize=safe-stack on every compile.
  clang::SanitizerMask Res;
  if (getTriple().getArch() == llvm::Triple::x86_64)
    Res |= clang::SanitizerKind::SafeStack;
  return Res;
}
