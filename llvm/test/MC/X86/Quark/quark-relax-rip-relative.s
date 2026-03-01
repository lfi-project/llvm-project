# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=NORELAX %s

## Test that Quark emits R_X86_64_PC32 + R_X86_64_RELAX pairs for
## RIP-relative memory accesses, both to local and external symbols.

.text
.globl _start

## RIP-relative to a local symbol.
_start:
  movq bar(%rip), %rax
  leaq bar(%rip), %rcx
  addq bar(%rip), %rdx
  cmpq bar(%rip), %rsi
bar:
  ret

# RELAX:      .rela.text {
# RELAX-NEXT:   0x3 R_X86_64_PC32 .text 0x18
# RELAX-NEXT:   0x3 R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0xA R_X86_64_PC32 .text 0x18
# RELAX-NEXT:   0xA R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0x11 R_X86_64_PC32 .text 0x18
# RELAX-NEXT:   0x11 R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0x18 R_X86_64_PC32 .text 0x18
# RELAX-NEXT:   0x18 R_X86_64_RELAX - 0x0
# RELAX-NEXT: }

## Without quark-relax, local RIP-relative references are resolved.
# NORELAX:      Relocations [
# NORELAX-NEXT: ]
