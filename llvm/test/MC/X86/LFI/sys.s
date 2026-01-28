// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// Syscall is converted to runtime call
syscall
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq .Ltmp0(%rip), %r11
// CHECK-NEXT: jmpq *(%r14)
// CHECK-NEXT: .Ltmp0:
// CHECK-NEXT: .bundle_unlock

// TLS read: movq %fs:0, %rX  ->  movq 32(%r15), %rX
movq %fs:0, %rax
// CHECK: movq 32(%r15), %rax

movq %fs:0, %rdi
// CHECK: movq 32(%r15), %rdi

// TLS with immediate offset: movq %fs:8, %rX
// Load thread pointer into %r11, then access 8(%r11)
movq %fs:8, %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %gs:8(%r11d), %rax

// TLS with register: movq %fs:(%rcx), %rX
// Load thread pointer into %r11, then access (%r11, %rcx)
movq %fs:(%rcx), %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %gs:(%r11d,%ecx), %rax

// TLS with register and offset: movq %fs:16(%rcx), %rX
movq %fs:16(%rcx), %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %gs:16(%r11d,%ecx), %rax
