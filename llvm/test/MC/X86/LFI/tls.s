// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// TLS read: movq %fs:0, %rX  ->  movq 32(%r15), %rX
movq %fs:0, %rax
// CHECK: movq 32(%r15), %rax

movq %fs:0, %rdi
// CHECK: movq 32(%r15), %rdi

movq %fs:0, %rcx
// CHECK: movq 32(%r15), %rcx

// TLS with immediate offset
// Load thread pointer into %r11, then access with offset via GS segment.
movq %fs:8, %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %gs:8(%r11d), %rax

movq %fs:16, %rdi
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %gs:16(%r11d), %rdi

// TLS with register base
// Load thread pointer into %r11, base becomes index.
movq %fs:(%rcx), %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %gs:(%r11d,%ecx), %rax

// TLS with register base and offset
movq %fs:16(%rcx), %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %gs:16(%r11d,%ecx), %rax

// TLS store
movq %rax, %fs:(%rcx)
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %rax, %gs:(%r11d,%ecx)

// TLS add (e.g. addq %fs:0, %rax)
addq %fs:0, %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: addq %gs:(%r11d), %rax
