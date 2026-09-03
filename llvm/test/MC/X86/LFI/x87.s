// RUN: llvm-mc -filetype asm -triple x86_64_lfi -show-encoding %s | FileCheck %s

// x87 register-to-register arithmetic touches no memory, so the rewriter must
// leave it exactly as written. The encodings below are the ones a plain
// x86-64 assembler produces.
//
// The %st(i)-destination forms and the %st-destination forms are distinct
// instructions, 0xDC and 0xD8, so rewriting one into the other would change
// which stack slot is written. AT&T syntax makes that easy to miss: the
// assembler prints the 0xDC form of fsub as fsub and the 0xD8 form as fsubr.

fadd %st, %st(1)
// CHECK: fadd %st, %st(1)
// CHECK-SAME: encoding: [0xdc,0xc1]

fmul %st, %st(2)
// CHECK: fmul %st, %st(2)
// CHECK-SAME: encoding: [0xdc,0xca]

fdiv %st, %st(1)
// CHECK: fdiv %st, %st(1)
// CHECK-SAME: encoding: [0xdc,0xf1]

fsub %st, %st(1)
// CHECK: fsub %st, %st(1)
// CHECK-SAME: encoding: [0xdc,0xe1]

fdivr %st, %st(1)
// CHECK: fdivr %st, %st(1)
// CHECK-SAME: encoding: [0xdc,0xf9]

fsubr %st, %st(1)
// CHECK: fsubr %st, %st(1)
// CHECK-SAME: encoding: [0xdc,0xe9]

// The %st-destination forms are likewise unchanged.

fadd %st(1), %st
// CHECK: fadd %st(1), %st
// CHECK-SAME: encoding: [0xd8,0xc1]

fmul %st(2), %st
// CHECK: fmul %st(2), %st
// CHECK-SAME: encoding: [0xd8,0xca]

fdiv %st(1), %st
// CHECK: fdiv %st(1), %st
// CHECK-SAME: encoding: [0xd8,0xf1]

fsub %st(1), %st
// CHECK: fsub %st(1), %st
// CHECK-SAME: encoding: [0xd8,0xe1]

// x87 memory operations are sandboxed like any other memory access.

flds (%rax)
// CHECK: flds %gs:(%eax)

fldl (%rbx)
// CHECK: fldl %gs:(%ebx)

fstps (%rcx)
// CHECK: fstps %gs:(%ecx)

fstpl (%rdx)
// CHECK: fstpl %gs:(%edx)

fadds (%rax)
// CHECK: fadds %gs:(%eax)

faddl (%rax)
// CHECK: faddl %gs:(%eax)

fmuls (%rax)
// CHECK: fmuls %gs:(%eax)

fmull (%rax)
// CHECK: fmull %gs:(%eax)

fdivs (%rax)
// CHECK: fdivs %gs:(%eax)

fdivl (%rax)
// CHECK: fdivl %gs:(%eax)

fsubs (%rax)
// CHECK: fsubs %gs:(%eax)

fsubl (%rax)
// CHECK: fsubl %gs:(%eax)

// Accesses through %rsp are already in bounds and need no sandboxing.

flds (%rsp)
// CHECK: flds (%rsp)

fstps (%rsp)
// CHECK: fstps (%rsp)

fadds 8(%rsp)
// CHECK: fadds 8(%rsp)
