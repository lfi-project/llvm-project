// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

.arch_extension lse

ldadd x0, x1, [x2]
// CHECK:      add x28, x27, w2, uxtw
// CHECK-NEXT: ldadd x0, x1, [x28]

swpal w0, w0, [x1]
// CHECK:      add x28, x27, w1, uxtw
// CHECK-NEXT: swpal w0, w0, [x28]

caspal w0, w1, w2, w3, [x4]
// CHECK:      add x28, x27, w4, uxtw
// CHECK-NEXT: caspal w0, w1, w2, w3, [x28]
