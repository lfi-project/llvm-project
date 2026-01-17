// RUN: not llvm-mc -filetype asm -triple aarch64_lfi %s 2>&1 | FileCheck %s

mov x28, x0
// CHECK: error: illegal modification of reserved LFI register
// CHECK:        mov x28, x0

ldr x27, [x0]
// CHECK: error: illegal modification of reserved LFI register
// CHECK:        ldr x27, [x0]

ldp x27, x28, [x0]
// CHECK: error: illegal modification of reserved LFI register
// CHECK:        ldp x27, x28, [x0]
