# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax -g %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 -g %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=NORELAX %s

## Test that DWARF .debug_line address deltas use ADD/SUB relocation
## pairs when Quark is enabled.

.text
.file 1 "test.c"
.globl _start
_start:
  .loc 1 1 0
  nop
  .loc 1 2 0
  call extern_func
  .loc 1 3 0
  nop
  .loc 1 4 0
  ret

## With quark-relax: debug_line has ADD16/SUB16 pairs for address deltas.
# RELAX:      .rela.debug_line {
# RELAX:        R_X86_64_64 .text 0x0
# RELAX-NEXT:   {{.*}} R_X86_64_ADD16 .L0  0x0
# RELAX-NEXT:   {{.*}} R_X86_64_SUB16 .L0  0x0
# RELAX-NEXT: }

## Without quark-relax: only the base address relocation.
# NORELAX:      .rela.debug_line {
# NORELAX-NEXT:   {{.*}} R_X86_64_64 .text 0x0
# NORELAX-NEXT: }
