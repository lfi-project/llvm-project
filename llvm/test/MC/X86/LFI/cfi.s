// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// Indirect call through register - CFI check before sandbox
callq *%rax
// CHECK:      andl $-32, %eax
// CHECK-NEXT: cmpl $-98693133, (%r14,%rax)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: callq *%rax

// Indirect call through another register
callq *%rbx
// CHECK:      andl $-32, %ebx
// CHECK-NEXT: cmpl $-98693133, (%r14,%rbx)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rbx
// CHECK-NEXT: callq *%rbx

// Indirect call through memory - target loaded into r11 first
callq *(%rax)
// CHECK:      movq %gs:(%eax), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11

// Indirect jump through register - CFI check applies
jmpq *%rax
// CHECK:      andl $-32, %eax
// CHECK-NEXT: cmpl $-98693133, (%r14,%rax)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax

// Return - no CFI check (return addresses don't have endbr64)
ret
// CHECK:      movq %rsp, %r11
// CHECK-NEXT: movq 16(%r15), %rsp
// CHECK-NEXT: retq

// Direct call - no CFI check needed (target is statically known)
callq foo
// CHECK:      callq foo
// CHECK-NOT:  cmpl
