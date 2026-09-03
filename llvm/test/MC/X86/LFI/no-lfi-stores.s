// RUN: llvm-mc -triple x86_64_lfi -mattr=+no-lfi-stores %s | FileCheck %s

// Loads-only mode: stores pass through, loads are sandboxed.

movq %rcx, (%rax)
// CHECK: movq %rcx, (%rax)

movq %rcx, 8(%rax,%rdi,4)
// CHECK: movq %rcx, 8(%rax,%rdi,4)

movq (%rax), %rdi
// CHECK: movq %gs:(%eax), %rdi

movq 8(%rax,%rcx,4), %rdi
// CHECK: movq %gs:8(%eax,%ecx,4), %rdi

// An instruction that both loads and stores is still sandboxed.

addq $1, (%rax)
// CHECK: addq $1, %gs:(%eax)

// The stack pointer must still stay inside the sandbox.

addq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// String operations only guard the pointers they load through.

stosq
// CHECK-NOT:  .bundle_lock
// CHECK:      stosq

movsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movsq
// CHECK-NEXT: .bundle_unlock

cmpsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: cmpsq
// CHECK-NEXT: .bundle_unlock
