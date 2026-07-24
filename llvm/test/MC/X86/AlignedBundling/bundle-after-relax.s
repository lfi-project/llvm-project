# RUN: llvm-mc -filetype=obj -triple x86_64 %s \
# RUN:   | llvm-objdump -d - | FileCheck %s

## Branches inside a bundle-locked group keep their short encodings when their
## targets are in range; the group is padded only if it would otherwise cross
## a bundle boundary.
  .text
relax_in_bundle:
  .bundle_align_mode 4
  pushq   %rbp

  movl    %edi, %ebx
  callq   bar
  movl    %eax, %r14d
  imull   $17, %ebx, %ebp
  movl    %ebx, %edi
  callq   bar
  cmpl    %r14d, %ebp
  .bundle_lock
  jle     .L_ELSE
  addl    %ebp, %eax
  jmp     .L_RET
  .bundle_unlock
## The short encodings are used, so the 6-byte group fits within its bundle
## and no group padding is needed.
# CHECK:      18: 7e 04 jle
# CHECK-NEXT: 1a: 01 e8 addl
# CHECK-NEXT: 1c: eb 05 jmp
.L_ELSE:
  imull   %ebx, %eax
.L_RET:
  popq    %rbx
## The imull is padded to the next bundle as usual.
# CHECK-NEXT: 1e: {{[a-f0-9 ]+}} nop
# CHECK-NEXT: 20: 0f af c3 imull

## Test that an instruction near a bundle end gets properly padded to the next
## bundle after it is relaxed.
  .align 16
relax_at_bundle_end:
  .rept 14
  push %rax
  .endr
# CHECK: 3d: 50 pushq
# CHECK-NEXT: 3e: {{[a-f0-9 ]+}} nop
# CHECK-NEXT: 40: 0f 85
  jne 0x100

## A branch inside a bundle-locked group is still relaxed when its target is
## out of range and the relaxed size is ## what the bundle placement accounts
## for.
  .p2align 4
locked_far_branch:
  .bundle_lock
  jle     far_target
  addl    %ebp, %eax
  .bundle_unlock
# CHECK:      50: 0f 8e {{.*}} jle
# CHECK-NEXT: 56: 01 e8 addl
