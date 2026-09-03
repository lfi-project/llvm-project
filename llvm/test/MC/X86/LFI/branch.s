// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// Direct calls - align to end of bundle
callq foo
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: callq foo
// CHECK-NEXT: .bundle_unlock

call bar
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: callq bar
// CHECK-NEXT: .bundle_unlock

// Indirect call through register
callq *%rax
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %eax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: callq *%rax
// CHECK-NEXT: .bundle_unlock

callq *%rbx
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %ebx
// CHECK-NEXT: addq %r14, %rbx
// CHECK-NEXT: callq *%rbx
// CHECK-NEXT: .bundle_unlock

callq *%rdi
// CHECK:      .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %edi
// CHECK-NEXT: addq %r14, %rdi
// CHECK-NEXT: callq *%rdi
// CHECK-NEXT: .bundle_unlock

// Indirect call through memory
callq *(%rax)
// CHECK:      movq %gs:(%eax), %r11
// CHECK-NEXT: .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11
// CHECK-NEXT: .bundle_unlock

callq *8(%rbx)
// CHECK:      movq %gs:8(%ebx), %r11
// CHECK-NEXT: .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11
// CHECK-NEXT: .bundle_unlock

callq *(%rax,%rcx,8)
// CHECK:      movq %gs:(%eax,%ecx,8), %r11
// CHECK-NEXT: .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11
// CHECK-NEXT: .bundle_unlock

// Call through stack pointer (safe, no sandboxing needed for load)
callq *(%rsp)
// CHECK:      movq (%rsp), %r11
// CHECK-NEXT: .bundle_lock align_to_end
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11
// CHECK-NEXT: .bundle_unlock

// Indirect jump through register
jmpq *%rax
// CHECK:      .bundle_lock
// CHECK-NEXT: andl $-32, %eax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax
// CHECK-NEXT: .bundle_unlock

jmpq *%rbx
// CHECK:      .bundle_lock
// CHECK-NEXT: andl $-32, %ebx
// CHECK-NEXT: addq %r14, %rbx
// CHECK-NEXT: jmpq *%rbx
// CHECK-NEXT: .bundle_unlock

// Indirect jump through memory
jmpq *(%rax)
// CHECK:      movq %gs:(%eax), %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

jmpq *16(%rbx)
// CHECK:      movq %gs:16(%ebx), %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

jmpq *(%rax,%rcx,8)
// CHECK:      movq %gs:(%eax,%ecx,8), %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// Jump through stack pointer
jmpq *(%rsp)
// CHECK:      movq (%rsp), %r11
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// Direct jumps - no rewriting needed
jmp foo
// CHECK: jmp foo

je foo
// CHECK: je foo

jne bar
// CHECK: jne bar
