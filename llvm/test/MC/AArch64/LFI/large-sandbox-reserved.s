// RUN: not llvm-mc -triple aarch64_lfi --lfi-config=large-sandbox,sandbox-bits=33 %s 2>&1 | FileCheck %s

// x24 is reserved in large sandbox mode as the offset register.
mov x24, x0
// CHECK: error: illegal modification of reserved LFI register
