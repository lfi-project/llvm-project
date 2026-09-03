// RUN: llvm-mc -triple aarch64_lfi --lfi-config=small-sandbox,sandbox-bits=24 --aarch64-lfi-guard-elim=false %s | FileCheck %s

// The LFI configuration comes from --lfi-config, not from subtarget features,
// so .arch and .cpu directives must not change the sandboxing scheme.

ldr x0, [x1]
// CHECK:      and x24, x1, #0xffffff
// CHECK-NEXT: ldr x0, [x27, x24]

.arch armv8-a+lse

// Still the small-sandbox form after .arch.
ldr x0, [x1]
// CHECK:      and x24, x1, #0xffffff
// CHECK-NEXT: ldr x0, [x27, x24]

// Extensions enabled by .arch still assemble, with sandboxed operands.
ldadd x0, x1, [x2]
// CHECK:      and x24, x2, #0xffffff
// CHECK-NEXT: add x28, x27, x24
// CHECK-NEXT: ldadd x0, x1, [x28]

.cpu generic+lse

ldr x0, [x1]
// CHECK:      and x24, x1, #0xffffff
// CHECK-NEXT: ldr x0, [x27, x24]

// An epilogue that reloads LR gets the masked control-flow guard.
ldp x29, x30, [sp], #16
ret
// CHECK:      and x24, x30, #0xffffff
// CHECK-NEXT: add x30, x27, x24
// CHECK-NEXT: ret
