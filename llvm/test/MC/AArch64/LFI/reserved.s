// RUN: not llvm-mc -filetype asm -triple aarch64_lfi %s 2> %t
// RUN: FileCheck --check-prefix=CHECK-ERROR %s <%t

mov x28, x0
// CHECK-ERROR: error: illegal modification of reserved LFI register
// CHECK-ERROR:        mov x28, x0
// CHECK-ERROR:        ^

ldr x27, [sp]
// CHECK-ERROR: error: illegal modification of reserved LFI register
// CHECK-ERROR:        ldr x27, [sp]
// CHECK-ERROR:        ^
