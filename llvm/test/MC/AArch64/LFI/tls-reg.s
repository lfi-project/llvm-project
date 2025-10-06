// RUN: llvm-mc -filetype asm -triple aarch64_lfi -mattr +lfi-tls-reg %s | FileCheck %s

msr tpidr_el0, x0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #16]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw

mrs x0, tpidr_el0
// CHECK:      ldr x0, [x25]

mrs x1, tpidr_el0
// CHECK:      ldr x1, [x25]
