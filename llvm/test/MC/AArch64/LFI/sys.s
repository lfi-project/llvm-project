// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

// Syscall: svc #0 -> runtime call via sandbox base
svc #0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldr x30, [x27]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw
