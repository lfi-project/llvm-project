# RUN: llvm-mc -filetype=obj -triple=x86_64 %s --x86-pad-max-prefix-size=5 -o %t.o
# RUN: llvm-objdump -d --no-show-raw-insn -s %t.o | FileCheck %s

## Prefix padding runs after layout has encoded the fragments holding a label
## difference, so padding must not move a label.

## Recorded offsets of .Lback and .Lfwd; they must match the code below.
# CHECK:      Contents of section .mydata:
# CHECK-NEXT: 0000 143b

  .text
  .bundle_align_mode 5

## padInstsBackward: padding the calls ahead of .Lback would move it to 0x1b.
.Lstart:
  callq bar
  callq bar
  callq bar
  callq bar
.Lback:
  .bundle_lock align_to_end
  callq bar
  .bundle_unlock

# CHECK:      0: call
# CHECK-NEXT: 5: call
# CHECK-NEXT: a: call
# CHECK-NEXT: f: call
# CHECK-NEXT: 14: nop
# CHECK-NEXT: 16: call

## padInstsForward: .Lpin blocks backward padding, so the gap is only reachable
## forwards. Growing the group's first call is safe, growing its second would
## move .Lfwd to 0x36.
  .p2align 5
  callq bar
  callq bar
.Lpin:
  .bundle_lock align_to_end
  callq bar
.Lfwd:
  callq bar
  .bundle_unlock

# CHECK-NEXT: 20: call
# CHECK-NEXT: 25: call
# CHECK-NEXT: 2a: nop
# CHECK-NEXT: 31: call
# CHECK-NEXT: 3b: call

  .section .mydata,"a",@progbits
  .uleb128 .Lback-.Lstart
  .uleb128 .Lfwd-.Lstart
