// RUN: llvm-mc -triple x86_64_lfi -mattr=+lfi-large-sandbox %s | FileCheck %s

// Large-sandbox mode masks an indirect-branch target to the sandbox with the
// r13 mask register, then aligns it to a 32-byte ENDBR64 landing pad with a
// 64-bit and (andl would clear the high sandbox bits), runs the forward-edge
// CFI check, and adds the base. Unlike the bundling scheme there is no
// .bundle_lock: the ENDBR64 landing-pad requirement already prevents an
// indirect branch from entering the middle of the sequence.

// Indirect jump through a register.
jmpq *%rax
// CHECK:      andq %r13, %rax
// CHECK-NEXT: andq $-32, %rax
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%rax)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rax
// CHECK-NEXT: jmpq *%rax

// Indirect call through a register.
callq *%rcx
// CHECK:      andq %r13, %rcx
// CHECK-NEXT: andq $-32, %rcx
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%rcx)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %rcx
// CHECK-NEXT: callq *%rcx

// Indirect jump through memory: the load is sandboxed (large-sandbox masking)
// into the scratch register first, then dispatched through the register path.
// The load preserves the flags (pext) since it is hand-written assembly.
jmpq *(%rdx)
// CHECK:      pextq %r13, %rdx, %r11
// CHECK-NEXT: movq (%r14,%r11), %r11
// CHECK-NEXT: andq %r13, %r11
// CHECK-NEXT: andq $-32, %r11
// CHECK-NEXT: .p2align 1
// CHECK-NEXT: cs
// CHECK-NEXT: cmpl $-98693133, (%r14,%r11)
// CHECK-NEXT: jne _lfi_trap
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
