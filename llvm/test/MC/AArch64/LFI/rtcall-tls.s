// RUN: llvm-mc -filetype asm -triple aarch64_lfi --aarch64-lfi-rtcall-tls %s | FileCheck %s

// Test TLS access using runtime calls instead of dedicated register (x25).
// Handler addresses are at:
//   [x27, #8]  = TLS read handler (offset 1, scaled)
//   [x27, #16] = TLS write handler (offset 2, scaled)

// TLS read to X0: direct call, result already in X0
mrs x0, tpidr_el0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #8]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw

// TLS read to non-X0 register: save X0, call, swap
mrs x1, tpidr_el0
// CHECK:      mov x1, x0
// CHECK-NEXT: mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #8]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw
// CHECK-NEXT: eor x0, x0, x1
// CHECK-NEXT: eor x1, x0, x1
// CHECK-NEXT: eor x0, x0, x1

mrs x5, tpidr_el0
// CHECK:      mov x5, x0
// CHECK-NEXT: mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #8]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw
// CHECK-NEXT: eor x0, x0, x5
// CHECK-NEXT: eor x5, x0, x5
// CHECK-NEXT: eor x0, x0, x5

// TLS write from X0: direct call
msr tpidr_el0, x0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #16]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw

// TLS write from non-X0 register: swap, call, swap back
msr tpidr_el0, x1
// CHECK:      eor x1, x1, x0
// CHECK-NEXT: eor x0, x1, x0
// CHECK-NEXT: eor x1, x1, x0
// CHECK-NEXT: mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #16]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw
// CHECK-NEXT: eor x0, x0, x1
// CHECK-NEXT: eor x1, x0, x1
// CHECK-NEXT: eor x0, x0, x1

msr tpidr_el0, x7
// CHECK:      eor x7, x7, x0
// CHECK-NEXT: eor x0, x7, x0
// CHECK-NEXT: eor x7, x7, x0
// CHECK-NEXT: mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #16]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw
// CHECK-NEXT: eor x0, x0, x7
// CHECK-NEXT: eor x7, x0, x7
// CHECK-NEXT: eor x0, x0, x7
