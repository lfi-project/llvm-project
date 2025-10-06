// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

rep stosq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep stosq %rax, %es:(%rdi)
// CHECK-NEXT: .bundle_unlock

rep movsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep movsq (%rsi), %es:(%rdi)
// CHECK-NEXT: .bundle_unlock
