// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+no-lfi-stores %s | FileCheck %s

// Loads-only mode: stores are not sandboxed, loads are sandboxed.

// Load - should be sandboxed
movl (%rax), %ecx
// CHECK: movl %gs:(%eax), %ecx

// Store - should NOT be sandboxed
movl %ecx, (%rax)
// CHECK: movl %ecx, (%rax)
// CHECK-NOT: %gs
