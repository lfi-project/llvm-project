// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+lfi-large-sandbox %s | FileCheck %s

// Large-sandbox mode keeps the 64-bit stack-pointer modification (rather than
// demoting it to 32 bits) and masks %rsp to the sandbox size afterwards. pext
// is used so the mask does not clobber any flags set by the modification, and
// the base is re-attached with leaq.

subq $16, %rsp
// CHECK: subq $16, %rsp
// CHECK-NEXT: pextq %r13, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

movq %rax, %rsp
// CHECK: movq %rax, %rsp
// CHECK-NEXT: pextq %r13, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

addq %rcx, %rsp
// CHECK: addq %rcx, %rsp
// CHECK-NEXT: pextq %r13, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// A stack modification whose source is itself a sandboxed memory operand: the
// operand is masked first, then the modification and the %rsp fixup follow.
addq (%rax), %rsp
// CHECK: movq %rax, %r11
// CHECK-NEXT: andq %r13, %r11
// CHECK-NEXT: addq (%r14,%r11), %rsp
// CHECK-NEXT: pextq %r13, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp

// pop into %rsp: pop to the scratch register, mask, then rebuild %rsp.
popq %rsp
// CHECK:      popq %r11
// CHECK-NEXT: pextq %r13, %r11, %r11
// CHECK-NEXT: leaq (%r14,%r11), %rsp

// Regular push/pop are not explicit %rsp modifications and are left alone.
pushq %rax
// CHECK: pushq %rax
popq %rax
// CHECK: popq %rax
