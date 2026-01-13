// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

.bb_start
mov x30, x0
.bb_end
// CHECK:      mov x30, x0
// CHECK-NEXT: add x30, x27, w30, uxtw

.bb_start
ldr x30, [sp]
.bb_end
// CHECK:      ldr x30, [sp]
// CHECK-NEXT: add x30, x27, w30, uxtw

.bb_start
ldp x29, x30, [sp]
.bb_end
// CHECK:      ldp x29, x30, [sp]
// CHECK-NEXT: add x30, x27, w30, uxtw
