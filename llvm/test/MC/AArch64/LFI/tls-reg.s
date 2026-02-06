// RUN: llvm-mc -filetype asm -triple aarch64_lfi --aarch64-lfi-rtcall-tls=false %s | FileCheck %s

// TLS read: mrs xN, tpidr_el0 -> ldr xN, [x25, #32]
mrs x0, tpidr_el0
// CHECK: ldr x0, [x25, #32]

mrs x1, tpidr_el0
// CHECK: ldr x1, [x25, #32]

// TLS write: msr tpidr_el0, xN -> str xN, [x25, #32]
msr tpidr_el0, x0
// CHECK: str x0, [x25, #32]

msr tpidr_el0, x1
// CHECK: str x1, [x25, #32]
