# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=NORELAX %s

## Test that RIP-relative accesses to external symbols get RELAX pairs
## with Quark, and plain relocations without.

.text
.globl _start
_start:
  movq extern_data(%rip), %rax
  leaq extern_data(%rip), %rcx
  ret

# RELAX:      .rela.text {
# RELAX-NEXT:   0x3 R_X86_64_PC32 extern_data 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0x3 R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0xA R_X86_64_PC32 extern_data 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0xA R_X86_64_RELAX - 0x0
# RELAX-NEXT: }

# NORELAX:      .rela.text {
# NORELAX-NEXT:   0x3 R_X86_64_PC32 extern_data 0xFFFFFFFFFFFFFFFC
# NORELAX-NOT:    R_X86_64_RELAX
# NORELAX:        0xA R_X86_64_PC32 extern_data 0xFFFFFFFFFFFFFFFC
# NORELAX-NOT:    R_X86_64_RELAX
# NORELAX:      }
