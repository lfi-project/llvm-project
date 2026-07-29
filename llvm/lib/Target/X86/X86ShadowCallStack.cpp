//===- X86ShadowCallStack.cpp - Caller side of the v2 SCS form -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// The shadow call stack has two callee forms, selected per function in
// X86FrameLowering:
//
//   v1 (stack-sourced): the prologue reads the return address from (%rsp).
//       Callable by anything, including uninstrumented code. The default.
//   v2 (register-sourced): the prologue stores %r11, which the caller has
//       loaded with the return address. The enforced value never transits
//       writable memory, closing the call-edge race.
//
// v2 is only sound where the compiler controls every call site, so it is
// restricted to internal, non-address-taken functions (see isShadowCallStackV2
// in X86.h) -- every caller is then a direct call this compiler emits. This
// pass supplies the caller half: before each direct call to a v2 callee it
// materializes the return address (the label immediately after the call, which
// is exactly what `call` pushes) into %r11:
//
//     leaq  .Lscsret(%rip), %r11
//     call  callee
//   .Lscsret:
//
// The callee's v2 prologue then stores %r11 without ever reading the stack.
//
//===----------------------------------------------------------------------===//

#include "X86.h"
#include "X86InstrInfo.h"
#include "X86Subtarget.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSymbol.h"

using namespace llvm;

#define PASS_KEY "x86-shadow-call-stack"
#define DEBUG_TYPE PASS_KEY

STATISTIC(NumV2CallSites,
          "Number of direct call sites given the race-free v2 return-address "
          "handoff");

bool llvm::isShadowCallStackV2(const Function &F) {
  if (!F.hasFnAttribute(Attribute::ShadowCallStack))
    return false;

  // Must be internal to this module and non-interposable, so every caller is
  // visible here (and gets the lea below); and not address-taken, so it is
  // never entered indirectly or by the OS/runtime -- this rules out function
  // pointers, vtables, signal handlers, global constructors/destructors, and
  // ifunc resolvers, all of which take the function's address.
  if (!F.hasLocalLinkage() || F.hasAddressTaken())
    return false;

  // Restrict to conventions where %r11 has no inbound role, so it is free to
  // carry the return address into the callee. Anything else falls back to v1.
  switch (F.getCallingConv()) {
  case CallingConv::C:
  case CallingConv::Fast:
  case CallingConv::Cold:
    return true;
  default:
    return false;
  }
}

namespace {

class X86ShadowCallStack : public MachineFunctionPass {
public:
  X86ShadowCallStack() : MachineFunctionPass(ID) {}
  StringRef getPassName() const override {
    return "X86 Shadow Call Stack (v2 call sites)";
  }
  bool runOnMachineFunction(MachineFunction &MF) override;

  static char ID;
};

} // end anonymous namespace

char X86ShadowCallStack::ID = 0;

bool X86ShadowCallStack::runOnMachineFunction(MachineFunction &MF) {
  const X86Subtarget &STI = MF.getSubtarget<X86Subtarget>();
  if (!STI.is64Bit())
    return false;

  const X86InstrInfo *TII = STI.getInstrInfo();
  bool Changed = false;

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      // Only direct calls have a known callee; indirect calls target
      // address-taken (hence v1) functions by construction.
      if (MI.getOpcode() != X86::CALL64pcrel32)
        continue;
      const MachineOperand &Callee = MI.getOperand(0);
      if (!Callee.isGlobal())
        continue;
      const auto *F = dyn_cast<Function>(Callee.getGlobal());
      if (!F || !isShadowCallStackV2(*F))
        continue;

      // Load the return address -- the address of the label placed right after
      // the call -- into %r11, the value the v2 prologue stores to the SCS.
      MCSymbol *RetSym = MF.getContext().createTempSymbol("scsret", true);
      BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(X86::LEA64r), X86::R11)
          .addReg(X86::RIP)
          .addImm(1)
          .addReg(0)
          .addSym(RetSym)
          .addReg(0);
      MI.setPostInstrSymbol(MF, RetSym);
      ++NumV2CallSites;
      Changed = true;
    }
  }

  return Changed;
}

INITIALIZE_PASS(X86ShadowCallStack, PASS_KEY, "X86 shadow call stack", false,
                false)

FunctionPass *llvm::createX86ShadowCallStackPass() {
  return new X86ShadowCallStack();
}
