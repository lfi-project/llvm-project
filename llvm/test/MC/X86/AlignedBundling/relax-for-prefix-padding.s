# RUN: llvm-mc -filetype=obj -triple=x86_64 %s --x86-pad-max-prefix-size=1 \
# RUN:   | llvm-objdump -d --no-show-raw-insn - | FileCheck %s

# This test checks that bundling relaxes branches proactively: prefix padding
# can shift a branch after its size is decided, so a displacement within a
# bundle size of the rel8 limit is relaxed even though rel8 would reach.

  .text
  .bundle_align_mode 5
  .p2align 5

  .rept 3
  callq x                     ## 5 bytes each
  .endr
# CHECK:       f: jmp
# CHECK-NEXT: 14: int3
  jmp near_target             ## rel8 would reach, but is relaxed to rel32
  .rept 8
  int3                        ## 1 byte each, 7 of them are prefix-padded
  .endr
  ## trailing NOPs are only consumed by prefix-padding the int3 instructions
# CHECK:      1e: int3
# CHECK-NEXT: 20: int3

  ## Three full bundles of spacer to push near_target close to rel8 max range
  .rept 3
  .bundle_lock
  .rept 32
  int3
  .endr
  .bundle_unlock
  .endr

  .rept 15
  int3
  .endr
# CHECK:           <near_target>:
# CHECK-NEXT:      8f: inc
near_target:
  inc %eax

  .bundle_lock
  .rept 32
  int3
  .endr
  .bundle_unlock
