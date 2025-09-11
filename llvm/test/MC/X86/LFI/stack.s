// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

movq %rdi, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

addq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

addq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock
