// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+no-lfi-segue,+lfi-gs-context %s | FileCheck %s

// GS-context mode: the context register file is addressed through the %gs
// segment base instead of r15, and r15 becomes a general-purpose register.
// This mode requires Segue to be disabled, so the memory accesses produced by
// the thread-pointer rewrite use the non-Segue (r14 + bundle) sandboxing
// sequence.

// TLS read: movq %fs:0, %rX  ->  movq %gs:16, %rX
movq %fs:0, %rax
// CHECK: movq %gs:16, %rax

// Any %fs:0 load/store op is rewritten in place to a %gs:16 access.
addq %fs:0, %rax
// CHECK: addq %gs:16, %rax

// TLS with register base: load the thread pointer from %gs:16, then sandbox
// the resulting access via the non-Segue path.
movq %fs:(%rcx), %rax
// CHECK:      movq %gs:16, %rax
// CHECK-NEXT: leal (%rax,%rcx), %eax
// CHECK-NEXT: movq (%r14,%rax), %rax

// TLS store: the destination is the memory operand, so %r11 holds the TP.
movq %rax, %fs:(%rdi)
// CHECK:      movq %gs:16, %r11
// CHECK-NEXT: leal (%r11,%rdi), %r11d
// CHECK-NEXT: movq %rax, (%r14,%r11)

// TLS with base, index, scale, and displacement.
movq %fs:8(%rdi,%rsi,2), %rax
// CHECK:      movq %gs:16, %rax
// CHECK-NEXT: leaq (%rax,%rdi), %rax
// CHECK-NEXT: leal 8(%rax,%rsi,2), %eax
// CHECK-NEXT: movq (%r14,%rax), %rax

// r15 is an ordinary general-purpose register in GS-context mode: it is
// sandboxed like any other base register when used in a memory access...
movq (%r15), %rax
// CHECK: movl %r15d, %eax
// CHECK-NEXT: movq (%r14,%rax), %rax

// ...and may be modified freely (modifying r15 is rejected in the default
// r15-context configuration, but allowed here).
movq %rax, %r15
// CHECK: movq %rax, %r15
