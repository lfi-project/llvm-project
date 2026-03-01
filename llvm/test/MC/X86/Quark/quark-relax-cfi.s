# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=NORELAX %s

## Test that DWARF CFA (.eh_frame) relocations use ADD/SUB pairs when
## Quark relaxation is enabled. Without Quark, standard PC-relative
## relocations are used.

.text
.globl _start
_start:
  .cfi_startproc
  nop
  call extern_func
  nop
  ret
  .cfi_endproc

## With quark-relax: eh_frame uses R_X86_64_ADD32/R_X86_64_SUB32 pairs.
# RELAX:      .rela.text {
# RELAX-NEXT:   0x2 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0x2 R_X86_64_RELAX - 0x0
# RELAX-NEXT: }
# RELAX:      .rela.eh_frame {
# RELAX-NEXT:   0x20 R_X86_64_ADD32 .L0  0x0
# RELAX-NEXT:   0x20 R_X86_64_SUB32 .L0  0x0
# RELAX-NEXT:   0x24 R_X86_64_ADD32 .L0  0x0
# RELAX-NEXT:   0x24 R_X86_64_SUB32 .L0  0x0
# RELAX-NEXT: }

## Without quark-relax: standard R_X86_64_PC32 in eh_frame.
# NORELAX:      .rela.eh_frame {
# NORELAX-NEXT:   0x20 R_X86_64_PC32 .text 0x0
# NORELAX-NEXT: }
