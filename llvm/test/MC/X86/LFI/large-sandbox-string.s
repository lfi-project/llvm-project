// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+lfi-large-sandbox %s | FileCheck %s

// String-instruction pointers are masked with r13 (via pext, to preserve the
// flags) and rebased, instead of the 32-bit truncation the default scheme uses.

// movsq sandboxes both the source (%rsi) and the destination (%rdi).
rep movsq
// CHECK:      .bundle_lock
// CHECK-NEXT: pextq %r13, %rsi, %rsi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: pextq %r13, %rdi, %rdi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep movsq
// CHECK-NEXT: .bundle_unlock

// stosq sandboxes only the destination (%rdi).
rep stosq
// CHECK:      .bundle_lock
// CHECK-NEXT: pextq %r13, %rdi, %rdi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep stosq
// CHECK-NEXT: .bundle_unlock
