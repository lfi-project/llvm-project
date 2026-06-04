// RUN: llvm-mc -triple x86_64_lfi -x86-lfi-hw-endbr %s | FileCheck %s

// With --x86-lfi-hw-endbr, the software comparison check is omitted: the
// hardware enforces ENDBR64 landing pads. Alignment and base relocation are
// still emitted.

// Indirect call through register
callq *%rax
// CHECK:      andl $-32, %eax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: callq *%rax
// CHECK-NOT:  cmpl
// CHECK-NOT:  jne _lfi_trap

// Indirect call through memory (the load is still sandboxed)
callq *(%rax)
// CHECK:      movq %gs:(%eax), %r11
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: callq *%r11
// CHECK-NOT:  cmpl
// CHECK-NOT:  jne _lfi_trap

// Indirect jump through register
jmpq *%rax
// CHECK:      andl $-32, %eax
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax
// CHECK-NOT:  cmpl
// CHECK-NOT:  jne _lfi_trap
