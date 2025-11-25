// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

fmul %st, %st(0)
// CHECK: fmul %st(0), %st

fmul %st(0), %st
// CHECK: fmul %st(0), %st

fmul %st(1), %st
// CHECK: fmul %st(1), %st

fmul %st, %st(1)
// CHECK: fmul %st, %st(1)

fdiv %st, %st(2)
// CHECK: fdiv %st, %st(2)
