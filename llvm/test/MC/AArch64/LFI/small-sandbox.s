// RUN: llvm-mc -triple aarch64_lfi --lfi-config=small-sandbox,sandbox-bits=30 --aarch64-lfi-guard-elim=false %s | FileCheck %s
// RUN: llvm-mc -triple aarch64_lfi --lfi-config=small-sandbox,sandbox-bits=30 %s | FileCheck %s --check-prefix=ELIM

// Small-sandbox mode applies the large-sandbox masking scheme to a sandbox
// that may be smaller than 4GiB, so control-flow guards use the sandbox size
// mask instead of the fixed-4GiB uxtw form. Memory rewrites are unchanged.

// Indirect branches and calls use the masked guard.
br x0
// CHECK:      and x24, x0, #0x3fffffff
// CHECK-NEXT: add x28, x27, x24, uxtx
// CHECK-NEXT: br x28

blr x1
// CHECK:      and x24, x1, #0x3fffffff
// CHECK-NEXT: add x28, x27, x24, uxtx
// CHECK-NEXT: blr x28

ret
// CHECK: ret

ret x2
// CHECK:      and x24, x2, #0x3fffffff
// CHECK-NEXT: add x28, x27, x24, uxtx
// CHECK-NEXT: ret x28

// A deferred LR guard also uses the masked form.
ldr x30, [sp]
ret
// CHECK:      ldr x30, [sp]
// CHECK-NEXT: and x24, x30, #0x3fffffff
// CHECK-NEXT: add x30, x27, x24, uxtx
// CHECK-NEXT: ret

// The LR restore after a syscall is a control-flow guard.
svc #0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldur x30, [x27, #-8]
// CHECK-NEXT: blr x30
// CHECK-NEXT: and x24, x26, #0x3fffffff
// CHECK-NEXT: add x30, x27, x24, uxtx

// Memory accesses are rewritten exactly as in large-sandbox mode.
ldr x0, [x1]
// CHECK:      and x24, x1, #0x3fffffff
// CHECK-NEXT: ldr x0, [x27, x24]

str x0, [x1, #8]
// CHECK:      and x24, x1, #0x3fffffff
// CHECK-NEXT: add x28, x27, x24, uxtx
// CHECK-NEXT: str x0, [x28, #8]

sub sp, sp, #16
// CHECK:      sub x26, sp, #16
// CHECK-NEXT: and x24, x26, #0x3fffffff
// CHECK-NEXT: add sp, x27, x24

// With guard elimination enabled, a preceding data guard of the same register
// elides a control-flow guard: the two forms are identical here.
elim:
ldr x0, [x2, #8]
br x2
// ELIM:      elim:
// ELIM-NEXT: and x24, x2, #0x3fffffff
// ELIM-NEXT: add x28, x27, x24, uxtx
// ELIM-NEXT: ldr x0, [x28, #8]
// ELIM-NEXT: br x28
