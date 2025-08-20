// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

mov x30, x0
// CHECK:      mov x26, x0
// CHECK-NEXT: add x30, x27, w26, uxtw

ldr x30, [sp]
// CHECK:      ldr x26, [sp]
// CHECK-NEXT: add x30, x27, w26, uxtw

ldp x29, x30, [sp]
// CHECK:      ldp x29, x26, [sp]
// CHECK-NEXT: add x30, x27, w26, uxtw

.scratch x10
ldr x30, [sp]
// CHECK:      ldr x10, [sp]
// CHECK-NEXT: add x30, x27, w10, uxtw
.scratch_clear
