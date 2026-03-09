// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// Direct call with shadow call stack
callq foo
// CHECK:      movq %rsp, 24(%r15)
// CHECK-NEXT: movq 16(%r15), %rsp
// CHECK-NEXT: leaq .Ltmp0(%rip), %r11
// CHECK-NEXT: pushq %r11
// CHECK-NEXT: movq %rsp, 16(%r15)
// CHECK-NEXT: movq 24(%r15), %rsp
// CHECK-NEXT: callq foo
// CHECK-NEXT: .Ltmp0:
// CHECK-NEXT: movq %rsp, 16(%r15)
// CHECK-NEXT: movq %r11, %rsp
// CHECK-NEXT: popq %r11

// Indirect call through register
callq *%rax
// CHECK:      movq %rsp, 24(%r15)
// CHECK-NEXT: movq 16(%r15), %rsp
// CHECK-NEXT: leaq .Ltmp1(%rip), %r11
// CHECK-NEXT: pushq %r11
// CHECK-NEXT: movq %rsp, 16(%r15)
// CHECK-NEXT: movq 24(%r15), %rsp
// CHECK-NEXT: andl $-32, %eax
// CHECK-NEXT: cmpl $-98693133, (%r14,%rax)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: callq *%rax
// CHECK-NEXT: .Ltmp1:
// CHECK-NEXT: movq %rsp, 16(%r15)
// CHECK-NEXT: movq %r11, %rsp
// CHECK-NEXT: popq %r11

// Indirect call through memory
callq *(%rax)
// CHECK:      movq %rsp, 24(%r15)
// CHECK-NEXT: movq 16(%r15), %rsp
// CHECK-NEXT: leaq .Ltmp2(%rip), %r11
// CHECK-NEXT: pushq %r11
// CHECK-NEXT: movq %rsp, 16(%r15)
// CHECK-NEXT: movq 24(%r15), %rsp
// CHECK-NEXT: movq %gs:(%eax), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11
// CHECK-NEXT: .Ltmp2:
// CHECK-NEXT: movq %rsp, 16(%r15)
// CHECK-NEXT: movq %r11, %rsp
// CHECK-NEXT: popq %r11

// Basic return
ret
// CHECK:      movq %rsp, %r11
// CHECK-NEXT: movq 16(%r15), %rsp
// CHECK-NEXT: retq

// Return with immediate
retq $8
// CHECK:      addl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: movq %rsp, %r11
// CHECK-NEXT: movq 16(%r15), %rsp
// CHECK-NEXT: retq

retq $16
// CHECK:      addl $16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: movq %rsp, %r11
// CHECK-NEXT: movq 16(%r15), %rsp
// CHECK-NEXT: retq

// Indirect jump should not have shadow call stack (but has CFI)
jmpq *%rax
// CHECK:      andl $-32, %eax
// CHECK-NEXT: cmpl $-98693133, (%r14,%rax)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax
