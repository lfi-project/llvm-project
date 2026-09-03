// RUN: not llvm-mc -triple x86_64_lfi -mattr=+lfi-large-sandbox %s 2>&1 | FileCheck %s

// In large-sandbox mode r15 is reserved as the sandbox mask register (the
// context register file moves to the %gs segment base).
movq %rax, %r15
// CHECK: error: illegal modification of reserved LFI register

// r13 is a general-purpose register in every LFI configuration.
movq %rax, %r13
// CHECK-NOT: error
