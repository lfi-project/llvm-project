// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+lfi-safestack %s | FileCheck %s

/// Stack Accesses
movq (%rsp), %rax
// CHECK: movq (%rsp), %rax

movq 8(%rsp), %rax
// CHECK: movq 8(%rsp), %rax

movq %rax, (%rsp)
// CHECK: movq %rax, (%rsp)

movq %rax, -8(%rsp)
// CHECK: movq %rax, -8(%rsp)

/// Stack Pointer Modification
// Move to RSP
movq %rdi, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: and $0xfffffffffff00000, %rsp
// CHECK-NEXT: and $0xfffff, %rdi
// CHECK-NEXT: addq %rdi, %rsp
// CHECK-NEXT: .bundle_unlock

movq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: and $0xfffffffffff00000, %rsp
// CHECK-NEXT: and $0xfffff, %rax
// CHECK-NEXT: addq %rax, %rsp
// CHECK-NEXT: .bundle_unlock

// Add to RSP
addq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movq %rsp, %r11
// CHECK-NEXT: andq $0xfffffffffff00000, %r11
// CHECK-NEXT: addq %rax, %rsp
// CHECK-NEXT: andq $0xfffff, %rsp
// CHECK-NEXT: leaq (%rsp,%r11), %rsp
// CHECK-NEXT: .bundle_unlock

addq %rcx, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movq %rsp, %r11
// CHECK-NEXT: andq $0xfffffffffff00000, %r11
// CHECK-NEXT: addq %rcx, %rsp
// CHECK-NEXT: andq $0xfffff, %rsp
// CHECK-NEXT: leaq (%rsp,%r11), %rsp
// CHECK-NEXT: .bundle_unlock

// Add immediate to RSP
addq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addq $8, $rsp
// CHECK-NEXT: movq (%rsp), %r11
// CHECK-NEXT: .bundle_unlock

addq $16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addq $16, $rsp
// CHECK-NEXT: movq (%rsp), %r11
// CHECK-NEXT: .bundle_unlock

addq $128, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addq $128, $rsp
// CHECK-NEXT: movq (%rsp), %r11
// CHECK-NEXT: .bundle_unlock

// Sub from RSP
subq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: subq $8, $rsp
// CHECK-NEXT: movq (%rsp), %r11
// CHECK-NEXT: .bundle_unlock

subq $16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: subq $16, $rsp
// CHECK-NEXT: movq (%rsp), %r11
// CHECK-NEXT: .bundle_unlock

subq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movq %rsp, %r11
// CHECK-NEXT: andq $0xfffffffffff00000, %r11
// CHECK-NEXT: subq %rax, %rsp
// CHECK-NEXT: andq $0xfffff, %rsp
// CHECK-NEXT: leaq (%rsp,%r11), %rsp
// CHECK-NEXT: .bundle_unlock

// And with RSP
andq $-16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movq %rsp, %r11
// CHECK-NEXT: andq $0xfffffffffff00000, %r11
// CHECK-NEXT: andq $-16, %rsp
// CHECK-NEXT: andq $0xfffff, %rsp
// CHECK-NEXT: leaq (%rsp,%r11), %rsp
// CHECK-NEXT: .bundle_unlock

// Or with RSP
orq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movq %rsp, %r11
// CHECK-NEXT: andq $0xfffffffffff00000, %r11
// CHECK-NEXT: orl $8, %rsp
// CHECK-NEXT: andq $0xfffff, %rsp
// CHECK-NEXT: leaq (%rsp,%r11), %rsp
// CHECK-NEXT: .bundle_unlock

// LEA into RSP
leaq 8(%rax), %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: andq $0xfffffffffff00000, %rsp
// CHECK-NEXT: leaq 8(%rax), %r11
// CHECK-NEXT: andq $0xfffff, %r11
// CHECK-NEXT: addq %r11, %rsp
// CHECK-NEXT: .bundle_unlock

leaq (%rax,%rcx), %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: andq $0xfffffffffff00000, %rsp
// CHECK-NEXT: leaq (%rax,%rcx), %r11
// CHECK-NEXT: andq $0xfffff, %r11
// CHECK-NEXT: addq %r11, %rsp
// CHECK-NEXT: .bundle_unlock

// Pop into RSP (special case)
popq %rsp
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andq $0xfffffffffff00000, %rsp
// CHECK-NEXT: andq $0xfffff, %r11
// CHECK-NEXT: addq %r11, %rsp
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

xchgq %rax, %rsp

leave

ret

