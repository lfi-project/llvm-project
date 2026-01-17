// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s
// RUN: llvm-mc -filetype asm -triple aarch64_lfi -mcpu=apple-m2 %s | FileCheck %s --check-prefix=FPAC

// LR modifications are deferred until the next control flow instruction.
// The guard uses x30 -> x30 for PAC compatibility.

mov x30, x0
ret
// CHECK:      mov x30, x0
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret
// FPAC:      mov x30, x0
// FPAC-NEXT: add x30, x27, w30, uxtw
// FPAC-NEXT: ret

ldr x30, [sp]
ret
// CHECK:      ldr x30, [sp]
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret
// FPAC:      ldr x30, [sp]
// FPAC-NEXT: add x30, x27, w30, uxtw
// FPAC-NEXT: ret

ldp x29, x30, [sp]
ret
// CHECK:      ldp x29, x30, [sp]
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret
// FPAC:      ldp x29, x30, [sp]
// FPAC-NEXT: add x30, x27, w30, uxtw
// FPAC-NEXT: ret

// AUTIASP emits validation load without FEAT_FPAC, but not with FEAT_FPAC
autiasp
nop
// CHECK:      hint #29
// CHECK-NEXT: ldr xzr, [x30]
// CHECK-NEXT: nop
// FPAC:      autiasp
// FPAC-NEXT: nop

// PACIASP passes through unchanged (just signs LR)
paciasp
nop
// CHECK:      hint #25
// CHECK-NEXT: nop
// FPAC:      paciasp
// FPAC-NEXT: nop

