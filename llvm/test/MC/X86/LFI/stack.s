// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// Move to RSP
movq %rdi, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

movq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// Add to RSP
addq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

addq %rcx, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl %ecx, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// Add immediate to RSP
addq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

addq $16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl $16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

addq $128, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl $128, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// Sub from RSP
subq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: subl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

subq $16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: subl $16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

subq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: subl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// And with RSP
andq $-16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: andl $-16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// Or with RSP
orq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: orl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// LEA into RSP
leaq 8(%rax), %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: leal 8(%eax), %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

leaq (%rax,%rcx), %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: leal (%eax,%ecx), %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// Pop into RSP (special case)
popq %rsp
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: movl %r11d, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

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
