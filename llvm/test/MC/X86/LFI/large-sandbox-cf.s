// RUN: llvm-mc -triple x86_64_lfi -mattr=+lfi-large-sandbox %s | FileCheck %s

// Control-flow masking in large-sandbox mode reuses the fixed-4GiB sequence
// (andl $-32; addq %r14), not the %r15 data mask, because executable code is
// confined to the low 4GiB of the sandbox.

// Indirect jump.
jmpq *%rax
// CHECK:      .bundle_lock
// CHECK-NEXT: andl $-32, %eax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax
// CHECK-NEXT: .bundle_unlock

// Indirect call: the call is placed at the end of the bundle so the return
// address is bundle-aligned.
callq *%rax
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %eax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: callq *%rax
// CHECK-NEXT: .bundle_unlock

// Return: pop the address into the scratch register and dispatch through it.
ret
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock
