// RUN: llvm-mc -triple x86_64_lfi -mattr=+lfi-large-sandbox %s | FileCheck %s

// CHECK: .bundle_align_mode 5

// Indirect jump: mask to the sandbox with r13, align to a bundle boundary by
// clearing the low bits, then add the base.
jmpq *%rax
// CHECK:      .bundle_lock
// CHECK-NEXT: andq %r13, %rax
// CHECK-NEXT: andq $-32, %rax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax
// CHECK-NEXT: .bundle_unlock

// Indirect call: same masking, but the call is placed at the end of the bundle
// so the return address is bundle-aligned.
callq *%rax
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: andq %r13, %rax
// CHECK-NEXT: andq $-32, %rax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: callq *%rax
// CHECK-NEXT: .bundle_unlock

// Return: pop the address into the scratch register and dispatch through it.
ret
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andq %r13, %r11
// CHECK-NEXT: andq $-32, %r11
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock
