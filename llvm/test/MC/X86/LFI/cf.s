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
// CHECK:      movq (%rdi), %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// Indirect jump through a complex memory addressing mode.
jmpq *8(%rdi, %rsi, 4)
// CHECK:      movq 8(%rdi,%rsi,4), %r11
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
// CHECK:      movq (%rdx), %r11
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
// return address.
retq $16
// CHECK:      popq %r11
// CHECK-NEXT: addq $16, %rsp
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock
