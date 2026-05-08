// RUN: llvm-mc -triple x86_64_lfi %s | FileCheck %s

// CHECK: .bundle_align_mode 5

// Indirect jump through a register.
jmpq *%rax
// CHECK:      .bundle_lock
// CHECK-NEXT: andl $-32, %eax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax
// CHECK-NEXT: .bundle_unlock

// Indirect jump through memory.
jmpq *(%rdi)
// CHECK:      movq %gs:(%edi), %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// Indirect jump through a complex memory addressing mode.
jmpq *8(%rdi, %rsi, 4)
// CHECK:      movq %gs:8(%edi,%esi,4), %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// Indirect call through a register: align_to_end so the return address is
// bundle-aligned.
callq *%rcx
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %ecx
// CHECK-NEXT: addq %r14, %rcx
// CHECK-NEXT: callq *%rcx
// CHECK-NEXT: .bundle_unlock

// Indirect call through memory.
callq *(%rdx)
// CHECK:      movq %gs:(%edx), %r11
// CHECK-NEXT: .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11
// CHECK-NEXT: .bundle_unlock

// Direct call: must be at the end of a bundle.
callq foo
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: callq foo
// CHECK-NEXT: .bundle_unlock

// Plain return.
ret
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// Return with immediate: pop additional stack bytes after popping the
// return address. The post-pop addq is itself a stack modification and so
// gets sandboxed (demoted to addl plus a lea fixup).
retq $16
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: addl $16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock
