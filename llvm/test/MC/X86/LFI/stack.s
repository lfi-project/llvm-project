// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// Move to RSP
movq %rdi, %rsp
// CHECK: movl %edi, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

movq %rax, %rsp
// CHECK: movl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// Add to RSP
addq %rax, %rsp
// CHECK: addl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

addq %rcx, %rsp
// CHECK: addl %ecx, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// Add immediate to RSP
addq $8, %rsp
// CHECK: addl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

addq $16, %rsp
// CHECK: addl $16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

addq $128, %rsp
// CHECK: addl $128, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// Sub from RSP
subq $8, %rsp
// CHECK: subl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

subq $16, %rsp
// CHECK: subl $16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

subq %rax, %rsp
// CHECK: subl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// And with RSP
andq $-16, %rsp
// CHECK: andl $-16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// Or with RSP
orq $8, %rsp
// CHECK: orl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// LEA into RSP
leaq 8(%rax), %rsp
// CHECK: leal 8(%eax), %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

leaq (%rax,%rcx), %rsp
// CHECK: leal (%eax,%ecx), %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// Pop into RSP (special case)
popq %rsp
// CHECK:      popq %r11
// CHECK-NEXT: movl %r11d, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// Regular push/pop should NOT trigger stack modification handling
// (they implicitly modify RSP but don't have RSP as an explicit destination)
pushq %rax
// CHECK: pushq %rax

popq %rax
// CHECK: popq %rax

pushq %rbx
// CHECK: pushq %rbx

popq %rbx
// CHECK: popq %rbx
