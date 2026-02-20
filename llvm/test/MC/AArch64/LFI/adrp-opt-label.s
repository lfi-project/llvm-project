// RUN: llvm-mc -filetype asm -triple aarch64_lfi -mattr=+lfi-adrp-opt %s | FileCheck %s

// Tests that ADRP is not dropped or incorrectly optimized across labels.

// ADRP followed by a label and then a matching load should not be optimized
// (the load may be reached via the label without executing the ADRP).
adrp_label_between:
    adrp x0, target
intervening_label:
    ldr x0, [x0, #8]

// CHECK-LABEL: adrp_label_between:
// CHECK-NEXT: adrp x0, target
// CHECK-LABEL: intervening_label:
// CHECK-NEXT: add x28, x27, w0, uxtw
// CHECK-NEXT: ldr x0, [x28, #8]

// ADRP as the last instruction before a label must not be dropped.
adrp_before_label:
    adrp x0, target
target:

// CHECK-LABEL: adrp_before_label:
// CHECK-NEXT: adrp x0, target
// CHECK-LABEL: target:

// ADRP as the last instruction in the file must not be dropped.
    adrp x1, target
// CHECK: adrp x1, target
