// RUN: not llvm-mc -triple x86_64_lfi %s 2>&1 | FileCheck %s

// %gs is reserved: it is what makes a sandboxed access relative to the
// sandbox base.

movq %gs:(%rax), %rbx
// CHECK: error: invalid use of reserved segment register %gs

movq %gs:8, %rbx
// CHECK: error: invalid use of reserved segment register %gs

movw %ax, %gs
// CHECK: error: illegal modification of reserved LFI register

// The moffs form encodes an absolute address in the instruction instead of an
// addressing mode, so there is nothing to make relative to the sandbox base.

movabsq 0x123456789abcdef, %rax
// CHECK: error: unsupported memory access

movabsb %al, 0x123456789abcdef
// CHECK: error: unsupported memory access

// The same goes for instructions that reach memory through a fixed register.

xlatb
// CHECK: error: unsupported memory access

maskmovdqu %xmm1, %xmm0
// CHECK: error: unsupported memory access

// Only a few instructions have a 32-bit form that can be used to write the
// stack pointer.

xchgq %rax, %rsp
// CHECK: error: unsupported modification of the stack pointer

shlq $1, %rsp
// CHECK: error: unsupported modification of the stack pointer

movw %ax, %sp
// CHECK: error: unsupported modification of the stack pointer

// A prefix must be followed by the instruction it applies to.

lock
// CHECK: error: unsupported instruction prefix
