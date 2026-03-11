// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+no-lfi-stores %s | FileCheck %s

// Loads-only mode: stores are not sandboxed, loads are sandboxed.

// stosq - RDI is a store, should not be sandboxed
stosq
// CHECK:      stosq
// CHECK-NOT:  leaq

// movsq - RSI is a load (sandbox), RDI is a store (skip)
movsq
// CHECK:      movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movsq

// cmpsq - both RSI and RDI are loads (sandbox both)
cmpsq
// CHECK:      movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: cmpsq

// rep variants
rep stosq
// CHECK:      rep stosq
// CHECK-NOT:  leaq

rep movsq
// CHECK:      movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: rep movsq

rep cmpsq
// CHECK:      movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep cmpsq
