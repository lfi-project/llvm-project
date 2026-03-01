# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=NORELAX %s

## Test that calls to global symbols within the same file still get
## R_X86_64_RELAX pairs under Quark. Without Quark, the PLT32 relocation
## is emitted (because the callee is global) but without RELAX.

.text
.globl _start, bar
_start:
  call bar
bar:
  ret

# RELAX:      .rela.text {
# RELAX-NEXT:   0x1 R_X86_64_PLT32 bar 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0x1 R_X86_64_RELAX - 0x0
# RELAX-NEXT: }

# NORELAX:      .rela.text {
# NORELAX-NEXT:   0x1 R_X86_64_PLT32 bar 0xFFFFFFFFFFFFFFFC
# NORELAX-NOT:    R_X86_64_RELAX
# NORELAX:      }
