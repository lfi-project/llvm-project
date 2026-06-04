// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+no-lfi-segue %s | FileCheck %s

// In no-segue mode, memory sandboxing uses explicit instructions instead of GS segment.
// The general pattern is: clear high bits with mov32, then use R14 as base.

// Load with simple addressing - dest register used as scratch
movq (%rax), %rdi
// CHECK: movl %eax, %edi
// CHECK-NEXT: movq (%r14,%rdi), %rdi

// Store - R11 used as scratch
movq %rsi, (%rax)
// CHECK: movl %eax, %r11d
// CHECK-NEXT: movq %rsi, (%r14,%r11)

// Non-mov instruction - R11 used as scratch
addq (%rax), %rdi
// CHECK: movl %eax, %r11d
// CHECK-NEXT: addq (%r14,%r11), %rdi

// Load with offset - LEA used to compute address
movq 8(%rax), %rdi
// CHECK: leal 8(%rax), %edi
// CHECK-NEXT: movq (%r14,%rdi), %rdi

// Absolute register (RSP) - no sandboxing needed
movq (%rsp), %rdi
// CHECK: movq (%rsp), %rdi

// Absolute register (R14) - no sandboxing needed
movq (%r14), %rdi
// CHECK: movq (%r14), %rdi

// Index with absolute base - just clear index high bits
movq (%r14,%rax), %rdi
// CHECK: movl %eax, %edi
// CHECK-NEXT: movq (%r14,%rdi), %rdi

// Complex addressing with offset - LEA used
movq 16(%rax,%rcx,4), %rdi
// CHECK: leal 16(%rax,%rcx,4), %edi
// CHECK-NEXT: movq (%r14,%rdi), %rdi

// Store with offset
movq %rsi, 8(%rax)
// CHECK: leal 8(%rax), %r11d
// CHECK-NEXT: movq %rsi, (%r14,%r11)
