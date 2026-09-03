// RUN: llvm-mc -triple x86_64_lfi -mattr=+no-lfi-loads %s | FileCheck %s

// Stores-only mode: loads pass through, stores are sandboxed.

movq (%rax), %rdi
// CHECK: movq (%rax), %rdi

movq 8(%rax,%rcx,4), %rdi
// CHECK: movq 8(%rax,%rcx,4), %rdi

movq %rcx, (%rax)
// CHECK: movq %rcx, %gs:(%eax)

movq %rcx, 8(%rax,%rdi,4)
// CHECK: movq %rcx, %gs:8(%eax,%edi,4)

// An instruction that both loads and stores is still sandboxed.

addq $1, (%rax)
// CHECK: addq $1, %gs:(%eax)

// The stack pointer must still stay inside the sandbox.

addq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// String operations only guard the pointer they store through.

stosq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosq
// CHECK-NEXT: .bundle_unlock

movsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsq
// CHECK-NEXT: .bundle_unlock

cmpsq
// CHECK-NOT:  .bundle_lock
// CHECK:      cmpsq

lodsq
// CHECK-NOT:  .bundle_lock
// CHECK:      lodsq
