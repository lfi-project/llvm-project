// RUN: llvm-mc -filetype asm -triple aarch64_lfi %s 2> %t
// RUN: FileCheck --check-prefix=CHECK-ERROR %s <%t

mov x28, x0
// CHECK:       mov x26, x0
// CHECK-ERROR: warning: deleting modification of reserved LFI register
// CHECK-ERROR:        mov x28, x0
// CHECK-ERROR:        ^

ldr x27, [x0]
// CHECK:       ldr xzr, [x27, w0, uxtw]
// CHECK-ERROR: warning: deleting modification of reserved LFI register
// CHECK-ERROR:        ldr x27, [x0]
// CHECK-ERROR:        ^

ldp x27, x28, [x0]
// CHECK:       add x28, x27, w0, uxtw
// CHECK-NEXT:  ldp xzr, x26, [x28]
// CHECK-ERROR: warning: deleting modification of reserved LFI register
// CHECK-ERROR:        ldp x27, x28, [x0]
// CHECK-ERROR:        ^
