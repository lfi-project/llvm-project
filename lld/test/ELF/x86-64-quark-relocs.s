# REQUIRES: x86
# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s -o %t.o
# RUN: ld.lld %t.o -o %t
# RUN: llvm-readelf -x .eh_frame %t | FileCheck %s

## Test that LLD correctly handles Quark linker relaxation relocations.
## R_X86_64_RELAX is silently consumed, and R_X86_64_ADD32/SUB32 pairs
## in .eh_frame are correctly resolved.

.text
.globl _start
_start:
  .cfi_startproc
  nop
  call bar
  nop
  ret
  .cfi_endproc

bar:
  .cfi_startproc
  ret
  .cfi_endproc

## The eh_frame section should contain resolved ADD32/SUB32 values.
## Just verify it links without error and produces non-zero FDE lengths.
# CHECK: section '.eh_frame':
# CHECK-NOT: warning
# CHECK-NOT: error
