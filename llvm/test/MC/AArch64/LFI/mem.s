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

ldp x0, x1, [sp], #8
// CHECK: ldp x0, x1, [sp], #8

ldp x0, x1, [x2], #8
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: ldp x0, x1, [x28]
// CHECK-NEXT: add x2, x2, #8

ldp x0, x1, [x2, #8]!
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: ldp x0, x1, [x28, #8]
// CHECK-NEXT: add x2, x2, #8

ldp x0, x1, [x2], #-8
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: ldp x0, x1, [x28]
// CHECK-NEXT: sub x2, x2, #8

ldp x0, x1, [x2, #-8]!
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: ldp x0, x1, [x28, #-8]
// CHECK-NEXT: sub x2, x2, #8

stp x0, x1, [x2, #-8]!
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: stp x0, x1, [x28, #-8]
// CHECK-NEXT: sub x2, x2, #8

ld3 { v0.4s, v1.4s, v2.4s }, [x0], #48
// CHECK:      add x28, x27, w0, uxtw
// CHECK-NEXT: ld3 { v0.4s, v1.4s, v2.4s }, [x28]
// CHECK-NEXT: add x0, x0, #48

st2 { v1.8b, v2.8b }, [x14], #16
// CHECK:      add x28, x27, w14, uxtw
// CHECK-NEXT: st2 { v1.8b, v2.8b }, [x28]
// CHECK-NEXT: add x14, x14, #16

st2 { v1.8b, v2.8b }, [x14]
// CHECK:      add x28, x27, w14, uxtw
// CHECK-NEXT: st2 { v1.8b, v2.8b }, [x28]

ld1 { v0.s }[1], [x8]
// CHECK:      add x28, x27, w8, uxtw
// CHECK-NEXT: ld1 { v0.s }[1], [x28]

ld1r { v3.2d }, [x9]
// CHECK:      add x28, x27, w9, uxtw
// CHECK-NEXT: ld1r { v3.2d }, [x28]

ld1 { v0.s }[1], [x8], x10
// CHECK:      add x28, x27, w8, uxtw
// CHECK-NEXT: ld1 { v0.s }[1], [x28]
// CHECK-NEXT: add x8, x8, x10

ldaxr x0, [x2]
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: ldaxr x0, [x28]

stlxr w15, w17, [x1]
// CHECK:      add x28, x27, w1, uxtw
// CHECK-NEXT: stlxr w15, w17, [x28]

ldr w4, [sp, w3, uxtw #2]
// CHECK:      add x26, sp, w3, uxtw #2
// CHECK-NEXT: ldr w4, [x27, w26, uxtw]

stxrb w11, w10, [x8]
// CHECK:      add x28, x27, w8, uxtw
// CHECK-NEXT: stxrb w11, w10, [x28]

ldr x0, [x0, :got_lo12:x]
// CHECK:      add x28, x27, w0, uxtw
// CHECK-NEXT: ldr x0, [x28, :got_lo12:x]
