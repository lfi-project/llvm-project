// RUN: llvm-mc -triple x86_64_lfi -mattr=+lfi-small-sandbox %s | FileCheck %s

// A small sandbox may be smaller than 4GiB, so the fixed-4GiB control-flow
// sequence (andl $-32; addq %r14) is not enough on its own: the truncated,
// bundle-aligned target is also masked with the sandbox size register.

// Indirect jump.
jmpq *%rax
// CHECK:      .bundle_lock
// CHECK-NEXT: andl $-32, %eax
// CHECK-NEXT: andq %r15, %rax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax
// CHECK-NEXT: .bundle_unlock

// Indirect jump through memory: the target is loaded with the usual memory
// sandboxing first.
jmpq *(%rdi)
// CHECK:      .bundle_lock
// CHECK-NEXT: pextq %r15, %rdi, %r11
// CHECK-NEXT: movq (%r14,%r11), %r11
// CHECK-NEXT: .bundle_unlock
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: andq %r15, %r11
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// Indirect call: the call is placed at the end of the bundle so the return
// address is bundle-aligned.
callq *%rcx
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %ecx
// CHECK-NEXT: andq %r15, %rcx
// CHECK-NEXT: addq %r14, %rcx
// CHECK-NEXT: callq *%rcx
// CHECK-NEXT: .bundle_unlock

// Direct calls only need bundle alignment.
callq foo
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: callq foo
// CHECK-NEXT: .bundle_unlock

// Return: pop the address into the scratch register and dispatch through it.
ret
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: andq %r15, %r11
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// Return with immediate: the stack adjustment is a sandboxed %rsp
// modification, as in the other modes.
retq $16
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: addq $16, %rsp
// CHECK-NEXT: pextq %r15, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: andq %r15, %r11
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock
