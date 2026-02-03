// RUN: llvm-mc -filetype asm -triple aarch64_lfi -mattr=+lfi-adrp-opt %s | FileCheck %s
// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s --check-prefix=NOOPT

// Tests for ADRP optimization (+lfi-adrp-opt).
// When enabled, ADRP followed by a matching load is optimized to use x28.

//===----------------------------------------------------------------------===//
// Basic ADRP optimization
//===----------------------------------------------------------------------===//

// ADRP followed by load that uses and overwrites the ADRP register.
adrp_basic:
    adrp x0, target
    ldr x0, [x0, #8]

// CHECK-LABEL: adrp_basic:
// CHECK-NEXT: adrp x28, target
// CHECK-NEXT: ldr x0, [x28, #8]

// NOOPT-LABEL: adrp_basic:
// NOOPT-NEXT: adrp x0, target
// NOOPT-NEXT: add x28, x27, w0, uxtw
// NOOPT-NEXT: ldr x0, [x28, #8]

//===----------------------------------------------------------------------===//
// Different load sizes
//===----------------------------------------------------------------------===//

adrp_ldrw:
    adrp x1, target
    ldr w1, [x1, #4]

// CHECK-LABEL: adrp_ldrw:
// CHECK-NEXT: adrp x28, target
// CHECK-NEXT: ldr w1, [x28, #4]

adrp_ldrb:
    adrp x2, target
    ldrb w2, [x2, #1]

// CHECK-LABEL: adrp_ldrb:
// CHECK-NEXT: adrp x28, target
// CHECK-NEXT: ldrb w2, [x28, #1]

adrp_ldrh:
    adrp x3, target
    ldrh w3, [x3, #2]

// CHECK-LABEL: adrp_ldrh:
// CHECK-NEXT: adrp x28, target
// CHECK-NEXT: ldrh w3, [x28, #2]

//===----------------------------------------------------------------------===//
// Non-matching cases (should NOT optimize)
//===----------------------------------------------------------------------===//

// Load uses different base register - no optimization.
adrp_diff_base:
    adrp x0, target
    ldr x1, [x2, #8]

// CHECK-LABEL: adrp_diff_base:
// CHECK-NEXT: adrp x0, target
// CHECK-NEXT: add x28, x27, w2, uxtw
// CHECK-NEXT: ldr x1, [x28, #8]

// Load doesn't overwrite ADRP register - no optimization.
adrp_no_overwrite:
    adrp x0, target
    ldr x1, [x0, #8]

// CHECK-LABEL: adrp_no_overwrite:
// CHECK-NEXT: adrp x0, target
// CHECK-NEXT: add x28, x27, w0, uxtw
// CHECK-NEXT: ldr x1, [x28, #8]

// ADRP followed by non-load instruction - no optimization.
adrp_non_load:
    adrp x0, target
    add x0, x0, #16

// CHECK-LABEL: adrp_non_load:
// CHECK-NEXT: adrp x0, target
// CHECK-NEXT: add x0, x0, #16

target:
