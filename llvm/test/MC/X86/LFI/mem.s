// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

movq (%rax), %rdi
// CHECK: movq %gs:(%eax), %rdi

movq %rcx, (%rax)
// CHECK: movq %rcx, %gs:(%eax)

movq %rcx, (%rax, %rdi)
// CHECK: movq %rcx, %gs:(%eax,%edi)
