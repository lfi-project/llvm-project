// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+lfi-large-sandbox %s | FileCheck %s

// Large-sandbox mode masks addresses with the r13 mask register instead of the
// 32-bit truncation / %gs segment used by the default 4GiB scheme (it implies
// +no-lfi-segue). A plain mov load/store must not clobber the flags, so the
// mask is performed with pext; a read-modify-write, which already sets the
// flags, uses the cheaper andq.

// Load through a register: the destination doubles as the scratch register.
movq (%rax), %rdi
// CHECK:      .bundle_lock
// CHECK-NEXT: pextq %r13, %rax, %rdi
// CHECK-NEXT: movq (%r14,%rdi), %rdi
// CHECK-NEXT: .bundle_unlock

// Store through a register (mov: preserve flags via pext).
movq %rcx, (%rax)
// CHECK:      .bundle_lock
// CHECK-NEXT: pextq %r13, %rax, %r11
// CHECK-NEXT: movq %rcx, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// Read-modify-write store already clobbers the flags: use mov + andq.
addq %rcx, (%rax)
// CHECK:      .bundle_lock
// CHECK-NEXT: movq %rax, %r11
// CHECK-NEXT: andq %r13, %r11
// CHECK-NEXT: addq %rcx, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// Full addressing mode: compute the effective address with a 64-bit lea first.
movq (%rax,%rdi,4), %rcx
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq (%rax,%rdi,4), %rcx
// CHECK-NEXT: pextq %r13, %rcx, %rcx
// CHECK-NEXT: movq (%r14,%rcx), %rcx
// CHECK-NEXT: .bundle_unlock

movq 8(%rax), %rdi
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 8(%rax), %rdi
// CHECK-NEXT: pextq %r13, %rdi, %rdi
// CHECK-NEXT: movq (%r14,%rdi), %rdi
// CHECK-NEXT: .bundle_unlock

// RSP-relative access is always valid and needs no masking.
movq (%rsp), %rax
// CHECK: movq (%rsp), %rax

// adc/sbb read the carry flag, so the mask must not clobber it: use pext even
// though they also write the flags (so the andq optimization does not apply).
adcq %rax, (%rbx)
// CHECK:      .bundle_lock
// CHECK-NEXT: pextq %r13, %rbx, %r11
// CHECK-NEXT: adcq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

sbbq %rcx, (%rdx)
// CHECK:      .bundle_lock
// CHECK-NEXT: pextq %r13, %rdx, %r11
// CHECK-NEXT: sbbq %rcx, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock
