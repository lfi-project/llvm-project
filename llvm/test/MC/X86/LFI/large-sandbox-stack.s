// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+lfi-large-sandbox %s | FileCheck %s

// Large-sandbox mode keeps the 64-bit stack-pointer modification and masks
// %rsp afterwards, with pext so the mask preserves any flags the modification
// set. The base is re-attached with leaq.

subq $16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: subq $16, %rsp
// CHECK-NEXT: pextq %r15, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

movq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movq %rax, %rsp
// CHECK-NEXT: pextq %r15, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

addq %rcx, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addq %rcx, %rsp
// CHECK-NEXT: pextq %r15, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// A stack modification whose source is itself a sandboxed memory operand: all
// of it must share a single bundle (no nested .bundle_lock).
addq (%rax), %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movq %rax, %r11
// CHECK-NEXT: andq %r15, %r11
// CHECK-NEXT: addq (%r14,%r11), %rsp
// CHECK-NEXT: pextq %r15, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// pop into %rsp: pop to the scratch register, mask, then rebuild %rsp.
popq %rsp
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: leaq (%r14,%r11), %rsp
// CHECK-NEXT: .bundle_unlock

// Regular push/pop are not explicit %rsp modifications and are left alone.
pushq %rax
// CHECK: pushq %rax
popq %rax
// CHECK: popq %rax
