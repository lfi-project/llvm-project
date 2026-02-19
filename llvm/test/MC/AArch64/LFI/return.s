// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

// LR modifications are deferred until the next control flow instruction.
// The guard uses x30 -> x30 for PAC compatibility.

.arch_extension pauth

mov x30, x0
ret
// CHECK:      mov x30, x0
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret

ldr x30, [sp]
ret
// CHECK:      ldr x30, [sp]
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret

ldp x29, x30, [sp]
ret
// CHECK:      ldp x29, x30, [sp]
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret

// AUTIASP passes through unchanged (FEAT_FPAC assumed).
autiasp
nop
// CHECK:      autiasp
// CHECK-NEXT: nop

// PACIASP passes through unchanged (just signs LR).
paciasp
nop
// CHECK:      paciasp
// CHECK-NEXT: nop
