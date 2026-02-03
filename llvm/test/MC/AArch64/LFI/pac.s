// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

// Tests for PAC instruction expansion in LFI.
// Authenticated branches/returns are expanded to their component operations.
// FEAT_FPAC is assumed, so no explicit validation loads are emitted.

.arch_extension pauth

// Authenticated returns.

// RETAA = AUTIASP + guard LR + RET
retaa
// CHECK:      autiasp
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret

// RETAB = AUTIBSP + guard LR + RET
retab
// CHECK:      autibsp
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret

// Authenticated branches.

// BRAA Xn, Xm = AUTIA Xn, Xm + guard + BR
braa x0, x1
// CHECK:      autia x0, x1
// CHECK-NEXT: add x28, x27, w0, uxtw
// CHECK-NEXT: br x28

// BRAAZ Xn = AUTIZA Xn + guard + BR
braaz x2
// CHECK:      autiza x2
// CHECK-NEXT: add x28, x27, w2, uxtw
// CHECK-NEXT: br x28

// BRAB Xn, Xm = AUTIB Xn, Xm + guard + BR
brab x3, x4
// CHECK:      autib x3, x4
// CHECK-NEXT: add x28, x27, w3, uxtw
// CHECK-NEXT: br x28

// BRABZ Xn = AUTIZB Xn + guard + BR
brabz x5
// CHECK:      autizb x5
// CHECK-NEXT: add x28, x27, w5, uxtw
// CHECK-NEXT: br x28

// Authenticated calls.

// BLRAA Xn, Xm = AUTIA Xn, Xm + guard + BLR
blraa x0, x1
// CHECK:      autia x0, x1
// CHECK-NEXT: add x28, x27, w0, uxtw
// CHECK-NEXT: blr x28

// BLRAAZ Xn = AUTIZA Xn + guard + BLR
blraaz x2
// CHECK:      autiza x2
// CHECK-NEXT: add x28, x27, w2, uxtw
// CHECK-NEXT: blr x28

// BLRAB Xn, Xm = AUTIB Xn, Xm + guard + BLR
blrab x3, x4
// CHECK:      autib x3, x4
// CHECK-NEXT: add x28, x27, w3, uxtw
// CHECK-NEXT: blr x28

// BLRABZ Xn = AUTIZB Xn + guard + BLR
blrabz x5
// CHECK:      autizb x5
// CHECK-NEXT: add x28, x27, w5, uxtw
// CHECK-NEXT: blr x28
