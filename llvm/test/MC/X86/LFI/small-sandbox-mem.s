// RUN: llvm-mc -filetype asm -triple x86_64_lfi -mattr=+lfi-small-sandbox %s | FileCheck %s

// A displacement left on a trusted base (%rsp, %r14, or an absolute address)
// must keep the access within the 128KiB guard region, so anything outside the
// window (-128KiB, 128KiB - 16KiB), or symbolic, is folded into the effective
// address before the mask. Everything else matches large-sandbox mode.

// Small displacements from %rsp are left alone, up to the window bounds.
movq %rax, 8(%rsp)
// CHECK: movq %rax, 8(%rsp)
movq %rax, -8(%rsp)
// CHECK: movq %rax, -8(%rsp)
movq %rax, 114687(%rsp)
// CHECK: movq %rax, 114687(%rsp)
movq %rax, -131071(%rsp)
// CHECK: movq %rax, -131071(%rsp)

// The first displacement past either bound takes the masked path: a 64-bit lea
// applies the displacement before the mask, and the access goes through
// (%r14, scratch) with no displacement.
movq %rax, 114688(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 114688(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock
movq %rax, -131072(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq -131072(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// A register-destination load uses its destination as the scratch register.
movq 262144(%rsp), %rax
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp), %rax
// CHECK-NEXT: pextq %r15, %rax, %rax
// CHECK-NEXT: movq (%r14,%rax), %rax
// CHECK-NEXT: .bundle_unlock

// A read-modify-write access already clobbers the flags, so andq is used.
addq $1, 262144(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp), %r11
// CHECK-NEXT: andq %r15, %r11
// CHECK-NEXT: addq $1, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// Vector and legacy-SSE accesses take the same path.
vmovdqu64 %zmm0, 262144(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: vmovdqu64 %zmm0, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock
movaps %xmm0, 262144(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movaps %xmm0, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// A symbolic displacement has an unknown value and is treated as large.
movq %rax, sym(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq sym(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// An assembler constant is folded to an immediate before the rewriter sees it.
.set OFF, 16
movq %rax, OFF(%rsp)
// CHECK: movq %rax, 16(%rsp)

// %rsp with an index was already masked in large-sandbox mode (the base is
// just another lea input); unchanged here.
movq %rax, 262144(%rsp,%rbx,8)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp,%rbx,8), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// push reads its operand with the pre-decrement %rsp, so the lea is exact.
pushq 262144(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: pushq (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// pop computes its destination address after incrementing %rsp by the operand
// size, but the lea runs before the pop, so the displacement is adjusted.
popq 262144(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262152(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: popq (%r14,%r11)
// CHECK-NEXT: .bundle_unlock
popw 262144(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262146(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: popw (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// An indirect branch through the stack loads its target with the same rewrite
// before the (small-sandbox) branch mask.
jmpq *262144(%rsp)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq (%r14,%r11), %r11
// CHECK-NEXT: .bundle_unlock
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: andl $-32, %r11d
// CHECK-NEXT: andq %r15, %r11
// CHECK-NEXT: addq %r14, %r11
// CHECK-NEXT: jmpq *%r11
// CHECK-NEXT: .bundle_unlock

// The memory operand of a stack-pointer modification shares the single bundle
// with the modification and its %rsp fixup.
addq 262144(%rsp), %rsp
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp), %r11
// CHECK-NEXT: andq %r15, %r11
// CHECK-NEXT: addq (%r14,%r11), %rsp
// CHECK-NEXT: pextq %r15, %rsp, %rsp
// CHECK-NEXT: leaq (%rsp,%r14), %rsp
// CHECK-NEXT: .bundle_unlock

// A high-byte register cannot be encoded alongside the REX prefix that %r11
// needs, so it is rotated into the low byte around the access.
movb %ah, 262144(%rsp)
// CHECK:      rorq $8, %rax
// CHECK-NEXT: .bundle_lock
// CHECK-NEXT: leaq 262144(%rsp), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movb %al, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock
// CHECK-NEXT: rolq $8, %rax

// The sandbox base is trusted like %rsp: small displacements stay, large ones
// are folded before the mask and wrap within the sandbox.
movq %rax, 8(%r14)
// CHECK: movq %rax, 8(%r14)
movq %rax, 262144(%r14)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%r14), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// An index-only form with scale 1 is canonicalized into the base slot first.
movq %rax, 262144(,%r14,1)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%r14), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// An absolute address is an offset from the sandbox base: small ones are
// rebased onto %r14, large or symbolic ones are materialized and masked.
movq %rax, 4096
// CHECK: movq %rax, 4096(%r14)
movq %rax, 262144
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144, %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock
movq %rax, sym
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq sym, %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock

// %rip-relative operands are left alone whatever the displacement; bounding
// them is the verifier's job.
movq %rax, sym(%rip)
// CHECK: movq %rax, sym(%rip)
movq %rax, 262144(%rip)
// CHECK: movq %rax, 262144(%rip)

// lea performs no memory access and is never rewritten.
leaq 262144(%rsp), %rax
// CHECK: leaq 262144(%rsp), %rax

// Untrusted bases are unaffected: the displacement was already folded into
// the lea in large-sandbox mode.
movq %rax, 262144(%rbx)
// CHECK:      .bundle_lock
// CHECK-NEXT: leaq 262144(%rbx), %r11
// CHECK-NEXT: pextq %r15, %r11, %r11
// CHECK-NEXT: movq %rax, (%r14,%r11)
// CHECK-NEXT: .bundle_unlock
