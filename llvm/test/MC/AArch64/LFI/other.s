// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s | FileCheck %s

.no_expand
ldr x0, [x1]
// CHECK: ldr x0, [x1]
.expand
