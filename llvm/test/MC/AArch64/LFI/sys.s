// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

// Syscall: svc #0 -> runtime call via negative offset from sandbox base
svc #0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldur x30, [x27, #-8]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw

// DC ZVA: dc zva, xN -> add x28, x27, wN, uxtw; dc zva, x28
dc zva, x0
// CHECK:      add x28, x27, w0, uxtw
// CHECK-NEXT: dc zva, x28

dc zva, x5
// CHECK:      add x28, x27, w5, uxtw
// CHECK-NEXT: dc zva, x28
