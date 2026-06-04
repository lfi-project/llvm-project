// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// Test x87 FPU instructions - these don't need memory sandboxing but
// the *_FrST0 forms get normalized to *_FST0r forms.

// fadd normalization: fadd %st, %st(N) -> fadd %st(N), %st
fadd %st, %st(1)
// CHECK: fadd %st(1), %st

fadd %st, %st(2)
// CHECK: fadd %st(2), %st

// fmul normalization
fmul %st, %st(1)
// CHECK: fmul %st(1), %st

fmul %st, %st(3)
// CHECK: fmul %st(3), %st

// fdiv normalization (fdiv %st, %st(N) uses fdivr opcode internally)
fdiv %st, %st(1)
// CHECK: fdivr %st(1), %st

fdiv %st, %st(2)
// CHECK: fdivr %st(2), %st

// fsub normalization (fsub %st, %st(N) uses fsubr opcode internally)
fsub %st, %st(1)
// CHECK: fsubr %st(1), %st

fsub %st, %st(2)
// CHECK: fsubr %st(2), %st

// fdivr normalization
fdivr %st, %st(1)
// CHECK: fdiv %st(1), %st

// fsubr normalization
fsubr %st, %st(1)
// CHECK: fsub %st(1), %st

// The reverse forms (already in *_FST0r form) should be unchanged
fadd %st(1), %st
// CHECK: fadd %st(1), %st

fmul %st(2), %st
// CHECK: fmul %st(2), %st

fdiv %st(1), %st
// CHECK: fdiv %st(1), %st

fsub %st(1), %st
// CHECK: fsub %st(1), %st

// x87 memory operations need sandboxing
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

// x87 memory operations with RSP are safe (no sandboxing)
flds (%rsp)
// CHECK: flds (%rsp)

fstps (%rsp)
// CHECK: fstps (%rsp)

fadds 8(%rsp)
// CHECK: fadds 8(%rsp)
