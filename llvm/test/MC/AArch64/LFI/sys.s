// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

svc #0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldr x30, [x27]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw

.scratch x10
svc #0
// CHECK:      mov x10, x30
// CHECK-NEXT: ldr x30, [x27]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w10, uxtw
.scratch_clear

msr tpidr_el0, x0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #16]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw

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

mrs x0, tpidr_el0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #8]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw

mrs x1, tpidr_el0
// CHECK:      mov x1, x0
// CHECK-NEXT: mov x26, x30
// CHECK-NEXT: ldr x30, [x27, #8]
// CHECK-NEXT: blr x30
// CHECK-NEXT: add x30, x27, w26, uxtw
// CHECK-NEXT: eor x0, x0, x1
// CHECK-NEXT: eor x1, x0, x1
// CHECK-NEXT: eor x0, x0, x1
