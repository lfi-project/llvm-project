// RUN: llvm-mc -triple aarch64_lfi --aarch64-lfi-guard-elim=false %s | FileCheck %s
// RUN: llvm-mc -triple aarch64_lfi --aarch64-lfi-warn-reserved-reg %s 2>&1 | FileCheck %s --check-prefix=WARN
// RUN: llvm-mc -triple aarch64_lfi %s 2>&1 | FileCheck %s --check-prefix=NOWARN

// Accesses to the reserved registers x25, x26, x27, and x28 are virtualized:
// the logical value of each is held in a context-area slot (byte offsets 24,
// 32, 40, 48 respectively), reachable from the context register x25. Donor
// registers are spilled to scratch slots at byte offsets 56, 64, 72.

// NOWARN-NOT: warning

// --- Compute: reserved register read, written, and read-modify-write ---

mov x0, x27
// CHECK:      ldr x26, [x25, #40]
// CHECK-NEXT: mov x0, x26
// WARN: warning: access to reserved LFI register X27 is virtualized

mov x27, x0
// CHECK:      mov x26, x0
// CHECK-NEXT: str x26, [x25, #40]

add x27, x27, #1
// CHECK:      ldr x26, [x25, #40]
// CHECK-NEXT: add x26, x26, #1
// CHECK-NEXT: str x26, [x25, #40]

// The 32-bit view of a reserved register is virtualized through w26; a w-write
// zero-extends into the full virtual slot.
mov w0, w27
// CHECK:      ldr x26, [x25, #40]
// CHECK-NEXT: mov w0, w26

// --- Compute: two reserved registers (x26 for the first, a donor for the
// second) ---

add x0, x27, x28
// CHECK:      str x1, [x25, #56]
// CHECK-NEXT: ldr x26, [x25, #40]
// CHECK-NEXT: ldr x1, [x25, #48]
// CHECK-NEXT: add x0, x26, x1
// CHECK-NEXT: ldr x1, [x25, #56]

// --- The scratch (x26) and context (x25) registers are virtualized too; the
// physical registers keep their LFI roles. ---

mov x0, x26
// CHECK:      ldr x26, [x25, #32]
// CHECK-NEXT: mov x0, x26
// WARN: warning: access to reserved LFI register X26 is virtualized

mov x25, x0
// CHECK:      mov x26, x0
// CHECK-NEXT: str x26, [x25, #24]
// WARN: warning: access to reserved LFI register X25 is virtualized

// --- Control flow: the branch target is loaded into x26 and guarded ---

br x27
// CHECK:      ldr x26, [x25, #40]
// CHECK-NEXT: add x28, x27, w26, uxtw
// CHECK-NEXT: br x28

blr x28
// CHECK:      ldr x26, [x25, #48]
// CHECK-NEXT: add x28, x27, w26, uxtw
// CHECK-NEXT: blr x28
// WARN: warning: access to reserved LFI register X28 is virtualized

ret x27
// CHECK:      ldr x26, [x25, #40]
// CHECK-NEXT: add x28, x27, w26, uxtw
// CHECK-NEXT: ret x28

// --- Memory: reserved register as base, stored data, and loaded data (donors,
// since the memory rewrite uses x26/x28 as scratch) ---

ldr x0, [x27]
// CHECK:      str x1, [x25, #56]
// CHECK-NEXT: ldr x1, [x25, #40]
// CHECK-NEXT: ldr x0, [x27, w1, uxtw]
// CHECK-NEXT: ldr x1, [x25, #56]

str x27, [x0]
// CHECK:      str x1, [x25, #56]
// CHECK-NEXT: ldr x1, [x25, #40]
// CHECK-NEXT: str x1, [x27, w0, uxtw]
// CHECK-NEXT: ldr x1, [x25, #56]

ldr x27, [x0]
// CHECK:      str x1, [x25, #56]
// CHECK-NEXT: ldr x1, [x27, w0, uxtw]
// CHECK-NEXT: str x1, [x25, #40]
// CHECK-NEXT: ldr x1, [x25, #56]

// Post-index: the base is both read and written, so the writeback updates the
// virtual slot.
ldr x0, [x27], #8
// CHECK:      str x1, [x25, #56]
// CHECK-NEXT: ldr x1, [x25, #40]
// CHECK-NEXT: ldr x0, [x27, w1, uxtw]
// CHECK-NEXT: add x1, x1, #8
// CHECK-NEXT: str x1, [x25, #40]
// CHECK-NEXT: ldr x1, [x25, #56]

// Register offset: the donor holds the virtual base while x26 remains free for
// the address computation.
ldr x0, [x27, x1]
// CHECK:      str x2, [x25, #56]
// CHECK-NEXT: ldr x2, [x25, #40]
// CHECK-NEXT: add x26, x2, x1
// CHECK-NEXT: ldr x0, [x27, w26, uxtw]
// CHECK-NEXT: ldr x2, [x25, #56]

// Unscaled (LDSTx) form uses the x28 guard sequence.
ldur x0, [x27, #8]
// CHECK:      str x1, [x25, #56]
// CHECK-NEXT: ldr x1, [x25, #40]
// CHECK-NEXT: add x28, x27, w1, uxtw
// CHECK-NEXT: ldur x0, [x28, #8]
// CHECK-NEXT: ldr x1, [x25, #56]

// Pair with two reserved data registers and a normal base (two donors).
stp x27, x28, [x0]
// CHECK:      str x1, [x25, #56]
// CHECK-NEXT: str x2, [x25, #64]
// CHECK-NEXT: ldr x1, [x25, #40]
// CHECK-NEXT: ldr x2, [x25, #48]
// CHECK-NEXT: add x28, x27, w0, uxtw
// CHECK-NEXT: stp x1, x2, [x28]
// CHECK-NEXT: ldr x1, [x25, #56]
// CHECK-NEXT: ldr x2, [x25, #64]

// Pair loading the link register from a reserved base (donor base + LR guard).
ldp x0, x30, [x27]
// CHECK:      str x1, [x25, #56]
// CHECK-NEXT: ldr x1, [x25, #40]
// CHECK-NEXT: add x28, x27, w1, uxtw
// CHECK-NEXT: ldp x0, x30, [x28]
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ldr x1, [x25, #56]

// --- Stack-pointer modification reading a reserved register (donor) ---

mov sp, x27
// CHECK:      str x0, [x25, #56]
// CHECK-NEXT: ldr x0, [x25, #40]
// CHECK-NEXT: add sp, x27, w0, uxtw
// CHECK-NEXT: ldr x0, [x25, #56]

// --- Thread-pointer access into/out of a reserved register (x26 substitute) ---

mrs x27, tpidr_el0
// CHECK:      ldr x26, [x25, #16]
// CHECK-NEXT: str x26, [x25, #40]

msr tpidr_el0, x27
// CHECK:      ldr x26, [x25, #40]
// CHECK-NEXT: str x26, [x25, #16]
