// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// Basic load through register - uses GS segment
movq (%rax), %rdi
// CHECK: movq %gs:(%eax), %rdi

// Basic store through register
movq %rcx, (%rax)
// CHECK: movq %rcx, %gs:(%eax)

// Memory access with base and index
movq %rcx, (%rax, %rdi)
// CHECK: movq %rcx, %gs:(%eax,%edi)

// Memory access with scale
movq (%rax, %rdi, 4), %rcx
// CHECK: movq %gs:(%eax,%edi,4), %rcx

movq (%rax, %rdi, 8), %rcx
// CHECK: movq %gs:(%eax,%edi,8), %rcx

// Memory access with offset
movq 8(%rax), %rdi
// CHECK: movq %gs:8(%eax), %rdi

movq 16(%rax), %rdi
// CHECK: movq %gs:16(%eax), %rdi

movq -8(%rax), %rdi
// CHECK: movq %gs:-8(%eax), %rdi

// Memory access with offset and index
movq 8(%rax, %rcx), %rdi
// CHECK: movq %gs:8(%eax,%ecx), %rdi

movq 16(%rax, %rcx, 4), %rdi
// CHECK: movq %gs:16(%eax,%ecx,4), %rdi

// RSP access is safe (no sandboxing needed)
movq (%rsp), %rax
// CHECK: movq (%rsp), %rax

movq 8(%rsp), %rax
// CHECK: movq 8(%rsp), %rax

movq %rax, (%rsp)
// CHECK: movq %rax, (%rsp)

movq %rax, -8(%rsp)
// CHECK: movq %rax, -8(%rsp)

// RIP-relative access is safe
movq foo(%rip), %rax
// CHECK: movq foo(%rip), %rax

movq %rax, foo(%rip)
// CHECK: movq %rax, foo(%rip)

// R14 (sandbox base) access is safe
movq (%r14), %rax
// CHECK: movq (%r14), %rax

movq 8(%r14), %rax
// CHECK: movq 8(%r14), %rax

// Different data sizes
movl (%rax), %edi
// CHECK: movl %gs:(%eax), %edi

movw (%rax), %di
// CHECK: movw %gs:(%eax), %di

movb (%rax), %dil
// CHECK: movb %gs:(%eax), %dil

// Different instructions
addq (%rax), %rdi
// CHECK: addq %gs:(%eax), %rdi

subq (%rax), %rdi
// CHECK: subq %gs:(%eax), %rdi

andq (%rax), %rdi
// CHECK: andq %gs:(%eax), %rdi

orq (%rax), %rdi
// CHECK: orq %gs:(%eax), %rdi

xorq (%rax), %rdi
// CHECK: xorq %gs:(%eax), %rdi

cmpq (%rax), %rdi
// CHECK: cmpq %gs:(%eax), %rdi

testq (%rax), %rdi
// CHECK: testq %rdi, %gs:(%eax)

// LEA does not need sandboxing (no memory access)
leaq (%rax), %rdi
// CHECK: leaq (%rax), %rdi

leaq 8(%rax, %rcx, 4), %rdi
// CHECK: leaq 8(%rax,%rcx,4), %rdi

// Push/pop with memory operand
pushq (%rax)
// CHECK: pushq %gs:(%eax)

popq (%rax)
// CHECK: popq %gs:(%eax)

// Atomic operations
lock incq (%rax)
// CHECK: lock
// CHECK: incq %gs:(%eax)

lock addq $1, (%rax)
// CHECK: lock
// CHECK: addq $1, %gs:(%eax)
