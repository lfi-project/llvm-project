// RUN: not llvm-mc -filetype asm -triple aarch64_lfi %s 2>&1 | FileCheck %s

// Tests for PC-relative literal loads that are not supported by LFI.
// These cannot be sandboxed because the address is computed relative to PC.

ldr x0, label
// CHECK: error: PC-relative literal loads are not supported in LFI

ldr w0, label
// CHECK: error: PC-relative literal loads are not supported in LFI

ldr s0, label
// CHECK: error: PC-relative literal loads are not supported in LFI

ldr d0, label
// CHECK: error: PC-relative literal loads are not supported in LFI

ldr q0, label
// CHECK: error: PC-relative literal loads are not supported in LFI

ldrsw x0, label
// CHECK: error: PC-relative literal loads are not supported in LFI

label:
.word 0x12345678

