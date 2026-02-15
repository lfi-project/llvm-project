// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+no-lfi-loads %s | FileCheck %s

// In stores-only mode, loads are not sandboxed but stores are.

// Load through register - not sandboxed
movq (%rax), %rdi
// CHECK: movq (%rax), %rdi

// Store through register - sandboxed
movq %rcx, (%rax)
// CHECK: movq %rcx, %gs:(%eax)

// Load with base and index - not sandboxed
movq (%rax, %rdi), %rcx
// CHECK: movq (%rax,%rdi), %rcx

// Store with base and index - sandboxed
movq %rcx, (%rax, %rdi)
// CHECK: movq %rcx, %gs:(%eax,%edi)

// Load with offset - not sandboxed
movq 8(%rax), %rdi
// CHECK: movq 8(%rax), %rdi

// Store with offset - sandboxed
movq %rdi, 8(%rax)
// CHECK: movq %rdi, %gs:8(%eax)

// ALU load - not sandboxed
addq (%rax), %rdi
// CHECK: addq (%rax), %rdi

// ALU store - sandboxed
addq %rdi, (%rax)
// CHECK: addq %rdi, %gs:(%eax)

// Compare (load-only) - not sandboxed
cmpq (%rax), %rdi
// CHECK: cmpq (%rax), %rdi

// LEA - never sandboxed
leaq (%rax), %rdi
// CHECK: leaq (%rax), %rdi

// Push/pop with memory (both may store) - sandboxed
pushq (%rax)
// CHECK: pushq %gs:(%eax)

popq (%rax)
// CHECK: popq %gs:(%eax)

// Lock prefix (atomic store) - sandboxed
lock incq (%rax)
// CHECK: lock
// CHECK: incq %gs:(%eax)

// RSP access is safe (no sandboxing needed)
movq (%rsp), %rax
// CHECK: movq (%rsp), %rax

movq %rax, (%rsp)
// CHECK: movq %rax, (%rsp)

// RIP-relative access is safe
movq foo(%rip), %rax
// CHECK: movq foo(%rip), %rax

movq %rax, foo(%rip)
// CHECK: movq %rax, foo(%rip)

// TLS read (fs:0) - thread pointer rewrite still happens
movq %fs:0, %rax
// CHECK: movq 32(%r15), %rax

// TLS with offset (load) - TP rewrite but no GS sandbox
movq %fs:8, %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq 8(%r11), %rax

// TLS with register base (load) - TP rewrite but no GS sandbox
movq %fs:(%rcx), %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq (%r11,%rcx), %rax

// TLS with register base and offset (load) - TP rewrite but no GS sandbox
movq %fs:16(%rcx), %rax
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq 16(%r11,%rcx), %rax

// TLS store - TP rewrite with GS sandbox
movq %rax, %fs:(%rcx)
// CHECK:      movq 32(%r15), %r11
// CHECK-NEXT: movq %rax, %gs:(%r11d,%ecx)

// String operations - only destination (RDI) is sandboxed
movsb
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsb (%rsi), %es:(%rdi)
// CHECK-NEXT: .bundle_unlock

stosb
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosb %al, %es:(%rdi)
// CHECK-NEXT: .bundle_unlock

rep movsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep movsq (%rsi), %es:(%rdi)
// CHECK-NEXT: .bundle_unlock
