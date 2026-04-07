// RUN: llvm-mc -triple aarch64_lfi -mattr=+lfi-large-sandbox --aarch64-lfi-no-guard-elim --aarch64-lfi-sandbox-bits=33 %s | FileCheck %s

// Memory: zero offset (RoX fast path)
ldr x0, [x1]
// CHECK:      and x24, x1, #0x1ffffffff
// CHECK-NEXT: ldr x0, [x27, x24]

str x0, [x1]
// CHECK:      and x24, x1, #0x1ffffffff
// CHECK-NEXT: str x0, [x27, x24]

// Memory: immediate offset (basic path)
ldr x0, [x1, #8]
// CHECK:      and x24, x1, #0x1ffffffff
// CHECK-NEXT: add x28, x27, x24
// CHECK-NEXT: ldr x0, [x28, #8]

str x0, [x1, #8]
// CHECK:      and x24, x1, #0x1ffffffff
// CHECK-NEXT: add x28, x27, x24
// CHECK-NEXT: str x0, [x28, #8]

// Memory: pre-index
ldr x0, [x1, #8]!
// CHECK:      add x1, x1, #8
// CHECK-NEXT: and x24, x1, #0x1ffffffff
// CHECK-NEXT: ldr x0, [x27, x24]

str x0, [x1, #-8]!
// CHECK:      sub x1, x1, #8
// CHECK-NEXT: and x24, x1, #0x1ffffffff
// CHECK-NEXT: str x0, [x27, x24]

// Memory: post-index
ldr x0, [x1], #8
// CHECK:      and x24, x1, #0x1ffffffff
// CHECK-NEXT: ldr x0, [x27, x24]
// CHECK-NEXT: add x1, x1, #8

str x0, [x1], #-8
// CHECK:      and x24, x1, #0x1ffffffff
// CHECK-NEXT: str x0, [x27, x24]
// CHECK-NEXT: sub x1, x1, #8

// Memory: register offset
ldr x0, [x1, x2]
// CHECK:      add x26, x1, x2
// CHECK-NEXT: and x24, x26, #0x1ffffffff
// CHECK-NEXT: ldr x0, [x27, x24]

ldr x0, [x1, x2, lsl #3]
// CHECK:      add x26, x1, x2, lsl #3
// CHECK-NEXT: and x24, x26, #0x1ffffffff
// CHECK-NEXT: ldr x0, [x27, x24]

// Memory: LDSTx (no register-register addressing)
ldp x0, x1, [x2]
// CHECK:      and x24, x2, #0x1ffffffff
// CHECK-NEXT: add x28, x27, x24
// CHECK-NEXT: ldp x0, x1, [x28]

stp x0, x1, [x2, #16]
// CHECK:      and x24, x2, #0x1ffffffff
// CHECK-NEXT: add x28, x27, x24
// CHECK-NEXT: stp x0, x1, [x28, #16]

// Branches
br x0
// CHECK:      and x24, x0, #0x1ffffffff
// CHECK-NEXT: add x28, x27, x24
// CHECK-NEXT: br x28

blr x0
// CHECK:      and x24, x0, #0x1ffffffff
// CHECK-NEXT: add x28, x27, x24
// CHECK-NEXT: blr x28

ret
// CHECK: ret

ret x0
// CHECK:      and x24, x0, #0x1ffffffff
// CHECK-NEXT: add x28, x27, x24
// CHECK-NEXT: ret x28

// Stack modification
mov sp, x0
// CHECK:      add x26, x0, #0
// CHECK-NEXT: and x24, x26, #0x1ffffffff
// CHECK-NEXT: add sp, x27, x24

add sp, sp, #8
// CHECK:      add x26, sp, #8
// CHECK-NEXT: and x24, x26, #0x1ffffffff
// CHECK-NEXT: add sp, x27, x24

sub sp, sp, #8
// CHECK:      sub x26, sp, #8
// CHECK-NEXT: and x24, x26, #0x1ffffffff
// CHECK-NEXT: add sp, x27, x24

// Stack accesses don't need rewriting
ldr x0, [sp]
// CHECK: ldr x0, [sp]

ldr x0, [sp, #8]
// CHECK: ldr x0, [sp, #8]

// Syscall
svc #0
// CHECK:      mov x26, x30
// CHECK-NEXT: ldur x30, [x27, #-8]
// CHECK-NEXT: blr x30
// CHECK-NEXT: and x24, x26, #0x1ffffffff
// CHECK-NEXT: add x30, x27, x24

// TLS (unchanged from standard)
mrs x0, TPIDR_EL0
// CHECK: ldr x0, [x25, #32]

msr TPIDR_EL0, x0
// CHECK: str x0, [x25, #32]
