// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+no-lfi-loads,+no-lfi-stores %s | FileCheck %s

// Jumps-only mode: no memory sandboxing at all.

stosq
// CHECK:      stosq
// CHECK-NOT:  leaq

movsq
// CHECK:      movsq
// CHECK-NOT:  leaq

cmpsq
// CHECK:      cmpsq
// CHECK-NOT:  leaq

rep stosq
// CHECK:      rep stosq
// CHECK-NOT:  leaq

rep movsq
// CHECK:      rep movsq
// CHECK-NOT:  leaq

repne cmpsb
// CHECK:      repne cmpsb
// CHECK-NOT:  leaq
