// RUN: llvm-mc -triple x86_64_lfi %s | FileCheck %s

// The address is computed from 32-bit registers, so that it wraps within the
// sandbox, and %gs supplies the sandbox base.

movq (%rax), %rdi
// CHECK: movq %gs:(%eax), %rdi

movq %rcx, (%rax)
// CHECK: movq %rcx, %gs:(%eax)

movq 8(%rax), %rdi
// CHECK: movq %gs:8(%eax), %rdi

movq -8(%rax), %rdi
// CHECK: movq %gs:-8(%eax), %rdi

movq (%rax,%rdi), %rcx
// CHECK: movq %gs:(%eax,%edi), %rcx

movq 16(%rax,%rdi,4), %rcx
// CHECK: movq %gs:16(%eax,%edi,4), %rcx

movq (,%rdi,8), %rcx
// CHECK: movq %gs:(,%edi,8), %rcx

// Any operand size works the same way.

movl (%rax), %edi
// CHECK: movl %gs:(%eax), %edi

movw (%rax), %di
// CHECK: movw %gs:(%eax), %di

movb (%rax), %dil
// CHECK: movb %gs:(%eax), %dil

movb (%rax), %ah
// CHECK: movb %gs:(%eax), %ah

// An address that is already 32-bit keeps its registers.

movl (%edi), %eax
// CHECK: movl %gs:(%edi), %eax

// So does an instruction that reads and writes the same location.

addq $1, (%rax)
// CHECK: addq $1, %gs:(%eax)

incq (%rax)
// CHECK: incq %gs:(%eax)

xchgq %rax, (%rcx)
// CHECK: xchgq %rax, %gs:(%ecx)

// A stand-alone prefix stays attached to the instruction it applies to.

lock incq (%rax)
// CHECK: lock {{.*}}incq %gs:(%eax)

lock
addq $1, (%rax)
// CHECK:      lock
// CHECK-NEXT: addq $1, %gs:(%eax)

// %rsp, %rip and %r14 always point into the sandbox, so an access based on
// one of them needs no rewriting.

movq (%rsp), %rax
// CHECK: movq (%rsp), %rax

movq 8(%rsp), %rax
// CHECK: movq 8(%rsp), %rax

movq %rax, -8(%rsp)
// CHECK: movq %rax, -8(%rsp)

movq foo(%rip), %rax
// CHECK: movq foo(%rip), %rax

movq %rax, foo(%rip)
// CHECK: movq %rax, foo(%rip)

movq (%r14), %rax
// CHECK: movq (%r14), %rax

movq 8(%r14), %rax
// CHECK: movq 8(%r14), %rax

// An index register can move the address back out of the sandbox, so the
// access is rewritten even when the base is safe.

movq (%rsp,%rax), %rcx
// CHECK: movq %gs:(%esp,%eax), %rcx

movq (%r14,%rax), %rcx
// CHECK: movq %gs:(%r14d,%eax), %rcx

// An absolute address has no register to truncate, so it is made relative to
// the sandbox base instead.

movq 4096, %rax
// CHECK: movq 4096(%r14), %rax

// lea computes an address but does not access memory, so it is left alone.

leaq (%rax), %rdi
// CHECK: leaq (%rax), %rdi

leaq 8(%rax,%rcx,4), %rdi
// CHECK: leaq 8(%rax,%rcx,4), %rdi

// Vector and x87 accesses are rewritten like any other.

movaps (%rdi), %xmm0
// CHECK: movaps %gs:(%edi), %xmm0

movups %xmm1, 16(%rsi,%rdx,4)
// CHECK: movups %xmm1, %gs:16(%esi,%edx,4)

flds (%rax)
// CHECK: flds %gs:(%eax)

fstpl 8(%rsp)
// CHECK: fstpl 8(%rsp)

// The implicit stack accesses of push and pop are safe, but an explicit
// memory operand is still rewritten.

pushq %rax
// CHECK: pushq %rax

pushq (%rax)
// CHECK: pushq %gs:(%eax)

popq (%rax)
// CHECK: popq %gs:(%eax)
