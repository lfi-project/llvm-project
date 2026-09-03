// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+no-lfi-loads %s | FileCheck %s

// Stores-only mode: loads are not sandboxed, stores are sandboxed.

// stosq - RDI is a store, should be sandboxed
stosq
// CHECK:      movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosq

// movsq - RSI is a load (skip), RDI is a store (sandbox)
movsq
// CHECK:      movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsq

// cmpsq - both RSI and RDI are loads (skip both)
cmpsq
// CHECK:      cmpsq
// CHECK-NOT:  leaq

// rep variants
rep stosq
// CHECK:      movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep stosq

rep movsq
// CHECK:      movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep movsq

rep cmpsq
// CHECK:      rep cmpsq
// CHECK-NOT:  leaq
