# RUN: llvm-mc -filetype=obj -triple x86_64-pc-linux-gnu -mcpu=pentiumpro %s -o - --x86-prefix-pad-for-lfi=1 --x86-pad-max-prefix-size=5 \
# RUN:   | llvm-objdump -d --no-show-raw-insn - | FileCheck %s

  .text
ensure_two_bundles:
  .bundle_align_mode 5
# This callq instruction is 5 bytes long
  callq   bar
  callq   bar
  callq   bar
  callq   bar
# CHECK:        0:  call
# CHECK-NEXT:   5:  call
# CHECK-NEXT:   c:  call
# CHECK-NEXT:   16:  call
  .bundle_lock align_to_end
  callq   bar
  callq   bar
  callq   bar
  .bundle_unlock
# Current implementation cannot consume following nops using instructions in the previous bundle.
# CHECK:        20: nop
# CHECK-NEXT:   2a: nop
# CHECK-NEXT:   31: callq

  .p2align 5
a_bundle_with_align_to_end:
  callq   bar
# CHECK:        40: call
# CHECK-NEXT:   4a: nop
  .bundle_lock align_to_end
# CHECK:        5b: call
  callq   bar
  .bundle_unlock

  .p2align 5
no_mid_nops:
  callq   bar
  callq   bar
  callq   bar
  callq   bar
  .bundle_lock
  callq   bar
  callq   bar
  callq   bar
  .bundle_unlock
# CHECK:        76: call
# CHECK-NEXT:   80: call
# CHECK-NEXT:   85: call

  .p2align 5
ignore_pad_out_of_p2align:
  int3
  int3
  # no optimization for this 14-byte nop.
  .p2align 4
  int3
  .bundle_lock
  int3
  .bundle_unlock
  int3
# CHECK:        a0: int3
# CHECK-NEXT:   a1: int3
# CHECK-NEXT:   a2: nop
# CHECK:        b0: int3
# CHECK-NEXT:   b1: int3
# CHECK-NEXT:   b2: int3
# CHECK-NEXT:   b3: nop

  .p2align 5
# There is no need to optimize the last bundle because following nops are not meant to be executed.
last_bundle:
  callq   bar
  .bundle_lock
  callq   bar
  .bundle_unlock
# CHECK:        c0: call
# CHECK-NEXT:   c5: call

# TODO: relative-pc fixup boundary overflow test
