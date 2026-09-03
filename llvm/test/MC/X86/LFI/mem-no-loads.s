// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+no-lfi-loads %s | FileCheck %s

// Stores-only mode: loads are not sandboxed, stores are sandboxed.

// Load - should NOT be sandboxed
movl (%rax), %ecx
// CHECK: movl (%rax), %ecx
// CHECK-NOT: %gs

// Store - should be sandboxed
movl %ecx, (%rax)
// CHECK: movl %ecx, %gs:(%eax)
