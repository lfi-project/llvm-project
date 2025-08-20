// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

ldr x0, [sp]
// CHECK: ldr x0, [sp]

ldr x0, [sp, #8]
// CHECK: ldr x0, [sp, #8]

ldp x0, x1, [sp, #8]
// CHECK: ldp x0, x1, [sp, #8]

str x0, [sp]
// CHECK: str x0, [sp]

str x0, [sp, #8]
// CHECK: str x0, [sp, #8]

stp x0, x1, [sp, #8]
// CHECK: stp x0, x1, [sp, #8]

ldur x0, [x1]
// CHECK:      add x28, x27, w1, uxtw
// CHECK-NEXT: ldur x0, [x28]

stur x0, [x1]
// CHECK:      add x28, x27, w1, uxtw
// CHECK-NEXT: stur x0, [x28]

ldp x0, x1, [x2]
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: ldp x0, x1, [x28]

stp x0, x1, [x2]
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: stp x0, x1, [x28]

ldr x0, [x1]
// CHECK: ldr x0, [x27, w1, uxtw]

ldr x0, [x1, #8]
// CHECK:      add x28, x27, w1, uxtw
// CHECK-NEXT: ldr x0, [x28, #8]

ldr x0, [x1, #8]!
// CHECK:      add x1, x1, #8
// CHECK-NEXT: ldr x0, [x27, w1, uxtw]

str x0, [x1, #8]!
// CHECK:      add x1, x1, #8
// CHECK-NEXT: str x0, [x27, w1, uxtw]

ldr x0, [x1, #-8]!
// CHECK:      sub x1, x1, #8
// CHECK-NEXT: ldr x0, [x27, w1, uxtw]

str x0, [x1, #-8]!
// CHECK:      sub x1, x1, #8
// CHECK-NEXT: str x0, [x27, w1, uxtw]

ldr x0, [x1], #8
// CHECK:      ldr x0, [x27, w1, uxtw]
// CHECK-NEXT: add x1, x1, #8

str x0, [x1], #8
// CHECK:      str x0, [x27, w1, uxtw]
// CHECK-NEXT: add x1, x1, #8

ldr x0, [x1], #-8
// CHECK:      ldr x0, [x27, w1, uxtw]
// CHECK-NEXT: sub x1, x1, #8

str x0, [x1], #-8
// CHECK:      str x0, [x27, w1, uxtw]
// CHECK-NEXT: sub x1, x1, #8

ldr x0, [x1, x2]
// CHECK:      add x26, x1, x2
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]

ldr x0, [x1, x2, lsl #3]
// CHECK:      add x26, x1, x2, lsl #3
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]

ldr x0, [x1, x2, sxtx #0]
// CHECK:      add x26, x1, x2, sxtx
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]

ldr x0, [x1, x2, sxtx #3]
// CHECK:      add x26, x1, x2, sxtx #3
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]

ldr x0, [x1, w2, uxtw]
// CHECK:      add x26, x1, w2, uxtw
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]

ldr x0, [x1, w2, uxtw #3]
// CHECK:      add x26, x1, w2, uxtw #3
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]

ldr x0, [x1, w2, sxtw]
// CHECK:      add x26, x1, w2, sxtw
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]

ldr x0, [x1, w2, sxtw #3]
// CHECK:      add x26, x1, w2, sxtw #3
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]
