// RUN: llvm-mc -triple x86_64_lfi -mattr=+no-lfi-loads,+no-lfi-stores %s | FileCheck %s

// Jumps-only mode: only control flow is sandboxed.

movq (%rax), %rdi
// CHECK: movq (%rax), %rdi

movq %rcx, (%rax)
// CHECK: movq %rcx, (%rax)

addq $1, (%rax)
// CHECK: addq $1, (%rax)

movq %fs:(%rdi), %rax
// CHECK:      movq 16(%r15), %rax
// CHECK-NEXT: movq (%rax,%rdi), %rax

// The stack pointer does not have to stay inside the sandbox either.

addq $8, %rsp
// CHECK-NOT:  .bundle_lock
// CHECK:      addq $8, %rsp

movq %rdi, %rsp
// CHECK-NOT:  .bundle_lock
// CHECK:      movq %rdi, %rsp

stosq
// CHECK-NOT:  .bundle_lock
// CHECK:      stosq

movsq
// CHECK-NOT:  .bundle_lock
// CHECK:      movsq

// Control flow is still sandboxed.

jmpq *%rax
// CHECK:      .bundle_lock
// CHECK-NEXT: andl $-32, %eax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax
// CHECK-NEXT: .bundle_unlock

ret
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock
