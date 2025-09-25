// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

.bb_start
ldr x0, [x1, #8]
ldr x0, [x1, #8]
ldr x0, [x1, #8]
ldr x0, [x1, #8]
.bb_end
// CHECK:      add x28, x27, w1, uxtw
// CHECK-NEXT: ldr x0, [x28, #8]
// CHECK-NEXT: ldr x0, [x28, #8]
// CHECK-NEXT: ldr x0, [x28, #8]
// CHECK-NEXT: ldr x0, [x28, #8]
