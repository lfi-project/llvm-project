// RUN: llvm-mc -filetype asm -triple x86_64_lfi -x86-lfi-hw-shstk %s | FileCheck --check-prefix=SHSTK %s
// RUN: llvm-mc -filetype asm -triple x86_64_lfi -x86-lfi-hw-endbr %s | FileCheck --check-prefix=ENDBR %s
// RUN: llvm-mc -filetype asm -triple x86_64_lfi -x86-lfi-hw-shstk -x86-lfi-hw-endbr %s | FileCheck --check-prefix=BOTH %s

// === Direct call ===
// hw-shstk: no SCS prologue/epilogue
// hw-endbr: SCS still present (direct calls don't have CFI checks anyway)
// both: no SCS
callq foo
// SHSTK:      callq foo
// SHSTK-NOT:  movq %rsp, 24(%r15)
//
// ENDBR:      movq %rsp, 24(%r15)
// ENDBR-NEXT: movq 16(%r15), %rsp
// ENDBR-NEXT: leaq .Ltmp0(%rip), %r11
// ENDBR-NEXT: pushq %r11
// ENDBR-NEXT: movq %rsp, 16(%r15)
// ENDBR-NEXT: movq 24(%r15), %rsp
// ENDBR-NEXT: callq foo
// ENDBR-NEXT: .Ltmp0:
// ENDBR-NEXT: movq %rsp, 16(%r15)
// ENDBR-NEXT: movq %r11, %rsp
// ENDBR-NEXT: popq %r11
//
// BOTH:      callq foo
// BOTH-NOT:  movq %rsp, 24(%r15)

// === Indirect call through register ===
// hw-shstk: no SCS, but software CFI check still present
// hw-endbr: SCS present, but no software CFI check
// both: no SCS and no software CFI check
callq *%rax
// SHSTK:      andl $-32, %eax
// SHSTK-NEXT: cmpl $-98693133, (%r14,%rax)
// SHSTK-NEXT: jne _lfi_trap
// SHSTK-NEXT: addq %r14, %rax
// SHSTK-NEXT: callq *%rax
//
// ENDBR:      movq %rsp, 24(%r15)
// ENDBR:      andl $-32, %eax
// ENDBR-NEXT: addq %r14, %rax
// ENDBR-NEXT: callq *%rax
// ENDBR-NOT:  cmpl
//
// BOTH:      andl $-32, %eax
// BOTH-NEXT: addq %r14, %rax
// BOTH-NEXT: callq *%rax

// === Indirect call through memory ===
// Target is loaded into r11 first, then sandboxed
callq *(%rax)
// SHSTK:      movq %gs:(%eax), %r11
// SHSTK-NEXT: andl $-32, %r11d
// SHSTK-NEXT: cmpl $-98693133, (%r14,%r11)
// SHSTK-NEXT: jne _lfi_trap
// SHSTK-NEXT: addq %r14, %r11
// SHSTK-NEXT: callq *%r11
//
// ENDBR:      movq %gs:(%eax), %r11
// ENDBR-NEXT: andl $-32, %r11d
// ENDBR-NEXT: addq %r14, %r11
// ENDBR-NEXT: callq *%r11
//
// BOTH:      movq %gs:(%eax), %r11
// BOTH-NEXT: andl $-32, %r11d
// BOTH-NEXT: addq %r14, %r11
// BOTH-NEXT: callq *%r11

// === Indirect jump through register ===
// Jumps never have SCS, but forward-edge CFI applies
jmpq *%rax
// SHSTK:      andl $-32, %eax
// SHSTK-NEXT: cmpl $-98693133, (%r14,%rax)
// SHSTK-NEXT: jne _lfi_trap
// SHSTK-NEXT: addq %r14, %rax
// SHSTK-NEXT: jmpq *%rax
//
// ENDBR:      andl $-32, %eax
// ENDBR-NEXT: addq %r14, %rax
// ENDBR-NEXT: jmpq *%rax
//
// BOTH:      andl $-32, %eax
// BOTH-NEXT: addq %r14, %rax
// BOTH-NEXT: jmpq *%rax

// === Return ===
// hw-shstk: ret emitted as-is (hardware handles backward-edge)
// hw-endbr: SCS return still present
// both: ret emitted as-is
ret
// SHSTK:      retq
// SHSTK-NOT:  movq %rsp, %r11
//
// ENDBR:      movq %rsp, %r11
// ENDBR-NEXT: movq 16(%r15), %rsp
// ENDBR-NEXT: retq
//
// BOTH: retq

// === Return with immediate ===
// hw-shstk: ret emitted as-is
// hw-endbr: SCS return with stack adjustment
retq $8
// SHSTK:      retq $8
// SHSTK-NOT:  movq %rsp, %r11
//
// ENDBR:      addl $8, %esp
// ENDBR-NEXT: leaq (%rsp,%r14), %rsp
// ENDBR-NEXT: movq %rsp, %r11
// ENDBR-NEXT: movq 16(%r15), %rsp
// ENDBR-NEXT: retq
//
// BOTH: retq $8

// === Direct call (no forward-edge CFI check needed) ===
// Verifies that direct calls never get cmpl regardless of flags
callq bar
// SHSTK-NOT: cmpl
// ENDBR-NOT: cmpl
// BOTH-NOT:  cmpl
