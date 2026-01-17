// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s
// RUN: llvm-mc -filetype asm -triple aarch64_lfi -mcpu=apple-m2 %s | FileCheck %s --check-prefix=FPAC

// Tests for PAC instruction expansion in LFI.
// Authenticated branches/returns are expanded to their component operations.

.arch_extension pauth

//===----------------------------------------------------------------------===//
// Authenticated returns (RETAA/RETAB)
//===----------------------------------------------------------------------===//

// RETAA = AUTIASP + [validation load] + guard LR + RET
retaa
// CHECK:      autiasp
// CHECK-NEXT: ldr xzr, [x30]
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret
// FPAC:      autiasp
// FPAC-NEXT: add x30, x27, w30, uxtw
// FPAC-NEXT: ret

// RETAB = AUTIBSP + [validation load] + guard LR + RET
retab
// CHECK:      autibsp
// CHECK-NEXT: ldr xzr, [x30]
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret
// FPAC:      autibsp
// FPAC-NEXT: add x30, x27, w30, uxtw
// FPAC-NEXT: ret

//===----------------------------------------------------------------------===//
// Authenticated branches (BRAA/BRAB/BRAAZ/BRABZ)
//===----------------------------------------------------------------------===//

// BRAA Xn, Xm = AUTIA Xn, Xm + [validation load] + guard + BR
braa x0, x1
// CHECK:      autia x0, x1
// CHECK-NEXT: ldr xzr, [x0]
// CHECK-NEXT: add x28, x27, w0, uxtw
// CHECK-NEXT: br x28
// FPAC:      autia x0, x1
// FPAC-NEXT: add x28, x27, w0, uxtw
// FPAC-NEXT: br x28

// BRAAZ Xn = AUTIZA Xn + [validation load] + guard + BR
braaz x2
// CHECK:      autiza x2
// CHECK-NEXT: ldr xzr, [x2]
// CHECK-NEXT: add x28, x27, w2, uxtw
// CHECK-NEXT: br x28
// FPAC:      autiza x2
// FPAC-NEXT: add x28, x27, w2, uxtw
// FPAC-NEXT: br x28

// BRAB Xn, Xm = AUTIB Xn, Xm + [validation load] + guard + BR
brab x3, x4
// CHECK:      autib x3, x4
// CHECK-NEXT: ldr xzr, [x3]
// CHECK-NEXT: add x28, x27, w3, uxtw
// CHECK-NEXT: br x28
// FPAC:      autib x3, x4
// FPAC-NEXT: add x28, x27, w3, uxtw
// FPAC-NEXT: br x28

// BRABZ Xn = AUTIZB Xn + [validation load] + guard + BR
brabz x5
// CHECK:      autizb x5
// CHECK-NEXT: ldr xzr, [x5]
// CHECK-NEXT: add x28, x27, w5, uxtw
// CHECK-NEXT: br x28
// FPAC:      autizb x5
// FPAC-NEXT: add x28, x27, w5, uxtw
// FPAC-NEXT: br x28

//===----------------------------------------------------------------------===//
// Authenticated calls (BLRAA/BLRAB/BLRAAZ/BLRABZ)
//===----------------------------------------------------------------------===//

// BLRAA Xn, Xm = AUTIA Xn, Xm + [validation load] + guard + BLR
blraa x0, x1
// CHECK:      autia x0, x1
// CHECK-NEXT: ldr xzr, [x0]
// CHECK-NEXT: add x28, x27, w0, uxtw
// CHECK-NEXT: blr x28
// FPAC:      autia x0, x1
// FPAC-NEXT: add x28, x27, w0, uxtw
// FPAC-NEXT: blr x28

// BLRAAZ Xn = AUTIZA Xn + [validation load] + guard + BLR
blraaz x2
// CHECK:      autiza x2
// CHECK-NEXT: ldr xzr, [x2]
// CHECK-NEXT: add x28, x27, w2, uxtw
// CHECK-NEXT: blr x28
// FPAC:      autiza x2
// FPAC-NEXT: add x28, x27, w2, uxtw
// FPAC-NEXT: blr x28

// BLRAB Xn, Xm = AUTIB Xn, Xm + [validation load] + guard + BLR
blrab x3, x4
// CHECK:      autib x3, x4
// CHECK-NEXT: ldr xzr, [x3]
// CHECK-NEXT: add x28, x27, w3, uxtw
// CHECK-NEXT: blr x28
// FPAC:      autib x3, x4
// FPAC-NEXT: add x28, x27, w3, uxtw
// FPAC-NEXT: blr x28

// BLRABZ Xn = AUTIZB Xn + [validation load] + guard + BLR
blrabz x5
// CHECK:      autizb x5
// CHECK-NEXT: ldr xzr, [x5]
// CHECK-NEXT: add x28, x27, w5, uxtw
// CHECK-NEXT: blr x28
// FPAC:      autizb x5
// FPAC-NEXT: add x28, x27, w5, uxtw
// FPAC-NEXT: blr x28

