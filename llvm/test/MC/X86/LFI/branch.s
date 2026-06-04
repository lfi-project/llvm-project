// RUN: llvm-mc -triple x86_64_lfi %s | FileCheck %s

// Indirect call through register
callq *%rax
// CHECK:      andl $-32, %eax
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%rax)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: callq *%rax

callq *%rbx
// CHECK:      andl $-32, %ebx
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%rbx)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rbx
// CHECK-NEXT: callq *%rbx

callq *%rdi
// CHECK:      andl $-32, %edi
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%rdi)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rdi
// CHECK-NEXT: callq *%rdi

// Indirect call through memory - target loaded into r11 first (and sandboxed)
callq *(%rax)
// CHECK:      movq %gs:(%eax), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11

callq *8(%rbx)
// CHECK:      movq %gs:8(%ebx), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11

callq *(%rax,%rcx,8)
// CHECK:      movq %gs:(%eax,%ecx,8), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11

// Indirect jump through register
jmpq *%rax
// CHECK:      andl $-32, %eax
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%rax)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax

jmpq *%rbx
// CHECK:      andl $-32, %ebx
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%rbx)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rbx
// CHECK-NEXT: jmpq *%rbx

// Indirect jump through memory
jmpq *(%rax)
// CHECK:      movq %gs:(%eax), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11

jmpq *16(%rbx)
// CHECK:      movq %gs:16(%ebx), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11

jmpq *(%rax,%rcx,8)
// CHECK:      movq %gs:(%eax,%ecx,8), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
