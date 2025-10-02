// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

syscall
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq .Ltmp0(%rip), %r11
// CHECK-NEXT: jmpq *(%r14)
// CHECK-NEXT: .Ltmp0:
// CHECK-NEXT: .bundle_unlock

movq %fs:0, %rax
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq .Ltmp1(%rip), %r11
// CHECK-NEXT: jmpq *8(%r14)
// CHECK-NEXT: .Ltmp1:
// CHECK-NEXT: .bundle_unlock

movq %fs:0, %rdi
// CHECK:      movq %rax, %rdi
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: leaq .Ltmp2(%rip), %r11
// CHECK-NEXT: jmpq *8(%r14)
// CHECK-NEXT: .Ltmp2:
// CHECK-NEXT: .bundle_unlock
// CHECK-NEXT: xchgq %rax, %rdi
