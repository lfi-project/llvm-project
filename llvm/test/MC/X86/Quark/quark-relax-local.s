# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=NORELAX %s

## Test that Quark forces relocation emission for intra-section (local)
## branches and calls that would normally be resolved by the assembler.
## Without Quark, no relocations are emitted for local references.

.text
.globl _start
_start:
  nop
  jmp bar
  je bar
  call bar
bar:
  ret

# RELAX:      .rela.text {
# RELAX-NEXT:   0x2 R_X86_64_PLT32 .text 0xD
# RELAX-NEXT:   0x2 R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0x8 R_X86_64_PLT32 .text 0xD
# RELAX-NEXT:   0x8 R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0xD R_X86_64_PLT32 .text 0xD
# RELAX-NEXT:   0xD R_X86_64_RELAX - 0x0
# RELAX-NEXT: }

## Without quark-relax, intra-section references are resolved at assembly time
## and no relocations are emitted.
# NORELAX:      Relocations [
# NORELAX-NEXT: ]
