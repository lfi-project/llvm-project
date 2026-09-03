// RUN: llvm-mc -triple x86_64_lfi %s | FileCheck %s

// A write to the stack pointer is demoted to %esp, which clears the top 32
// bits, and the sandbox base is added back with a lea, which leaves the flags
// alone. Both halves stay in the same bundle.

movq %rdi, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

addq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

subq $16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: subl $16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

addq %rax, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: addl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

andq $-16, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: andl $-16, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

orq $8, %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: orl $8, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// The 32-bit lea keeps 64-bit address registers, so only the destination
// changes size.

leaq 8(%rax), %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: leal 8(%rax), %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// A load into the stack pointer is sandboxed like any other load. Only the
// low 32 bits are loaded, which are the offset within the sandbox.

movq (%rax), %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %gs:(%eax), %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// An instruction that already writes %esp clears the top 32 bits itself.

movl %eax, %esp
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %eax, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// popq %rsp loads through the stack pointer it overwrites, so it goes through
// the scratch register.

popq %rsp
// CHECK:      popq %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: movl %r11d, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// leave restores the stack pointer from the frame pointer, which is an
// ordinary register under LFI.

leave
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %ebp, %esp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock
// CHECK-NEXT: popq %rbp

// push and pop move the stack pointer by a fixed amount, so they keep it
// inside the sandbox and are left alone.

pushq %rax
// CHECK: pushq %rax

popq %rax
// CHECK: popq %rax

pushq %rsp
// CHECK: pushq %rsp
