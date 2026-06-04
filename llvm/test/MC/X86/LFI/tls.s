// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// TLS read: movq %fs:0, %rX  ->  movq 16(%r15), %rX
movq %fs:0, %rax
// CHECK: movq 16(%r15), %rax

movq %fs:0, %rdi
// CHECK: movq 16(%r15), %rdi

movq %fs:0, %rcx
// CHECK: movq 16(%r15), %rcx

// TLS with immediate offset
// Load thread pointer into the destination register and access with %gs:
// sandboxing for the resulting load.
movq %fs:8, %rax
// CHECK:      movq 16(%r15), %rax
// CHECK-NEXT: movq %gs:8(%eax), %rax

movq %fs:16, %rdi
// CHECK:      movq 16(%r15), %rdi
// CHECK-NEXT: movq %gs:16(%edi), %rdi

// TLS with register base
// Load thread pointer into the destination register, the original base
// register becomes the index, and the access is sandboxed via %gs:.
movq %fs:(%rcx), %rax
// CHECK:      movq 16(%r15), %rax
// CHECK-NEXT: movq %gs:(%eax,%ecx), %rax

// TLS with register base and offset
movq %fs:16(%rcx), %rax
// CHECK:      movq 16(%r15), %rax
// CHECK-NEXT: movq %gs:16(%eax,%ecx), %rax

// TLS store: the destination is the memory operand, so we fall back to %r11
// as the TP scratch, then sandbox the resulting store.
movq %rax, %fs:(%rcx)
// CHECK:      movq 16(%r15), %r11
// CHECK-NEXT: movq %rax, %gs:(%r11d,%ecx)

// TLS add: pure %fs:0 form, rewritten in-place to access the runtime VRF
// directly (no further sandboxing needed for %r15-relative access).
addq %fs:0, %rax
// CHECK: addq 16(%r15), %rax
