// RUN: llvm-mc -triple x86_64_lfi %s | FileCheck %s

// Direct calls and jumps are not rewritten: their targets are fixed at link
// time and do not need a forward-edge CFI check.

callq foo
// CHECK:     callq foo
// CHECK-NOT: cmpl

call bar
// CHECK:     callq bar
// CHECK-NOT: cmpl

jmp foo
// CHECK:     jmp foo
// CHECK-NOT: cmpl

je foo
// CHECK:     je foo
// CHECK-NOT: cmpl

jne bar
// CHECK:     jne bar
// CHECK-NOT: cmpl
