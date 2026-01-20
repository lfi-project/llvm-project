// RUN: llvm-mc -filetype asm -triple aarch64_lfi -mattr=+execute-only %s | FileCheck %s

// Tests for PAC validation in execute-only memory mode.
// With +execute-only, the validation load uses an alternative sequence that
// doesn't read from code addresses.

.arch_extension pauth

//===----------------------------------------------------------------------===//
// AUTIASP validation
//===----------------------------------------------------------------------===//

autiasp
// CHECK:      autiasp
// CHECK-NEXT: and x28, x30, #0xffffffff00000000
// CHECK-NEXT: ldur xzr, [x28, #-8]
// CHECK-NEXT: mov x28, x27

//===----------------------------------------------------------------------===//
// Authenticated returns
//===----------------------------------------------------------------------===//

retaa
// CHECK:      autiasp
// CHECK-NEXT: and x28, x30, #0xffffffff00000000
// CHECK-NEXT: ldur xzr, [x28, #-8]
// CHECK-NEXT: mov x28, x27
// CHECK-NEXT: add x30, x27, w30, uxtw
// CHECK-NEXT: ret

//===----------------------------------------------------------------------===//
// Authenticated branches
//===----------------------------------------------------------------------===//

braaz x0
// CHECK:      autiza x0
// CHECK-NEXT: and x28, x0, #0xffffffff00000000
// CHECK-NEXT: ldur xzr, [x28, #-8]
// CHECK-NEXT: mov x28, x27
// CHECK-NEXT: add x28, x27, w0, uxtw
// CHECK-NEXT: br x28

//===----------------------------------------------------------------------===//
// Authenticated calls
//===----------------------------------------------------------------------===//

blraaz x1
// CHECK:      autiza x1
// CHECK-NEXT: and x28, x1, #0xffffffff00000000
// CHECK-NEXT: ldur xzr, [x28, #-8]
// CHECK-NEXT: mov x28, x27
// CHECK-NEXT: add x28, x27, w1, uxtw
// CHECK-NEXT: blr x28
