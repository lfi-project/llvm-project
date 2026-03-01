# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=NORELAX %s

## Test that Quark emits R_X86_64_PLT32 + R_X86_64_RELAX pairs for branches
## and calls to external symbols, and that without Quark, no RELAX relocations
## are emitted.

.text
.globl _start
_start:
  jmp extern_func
  je extern_func
  jne extern_func
  call extern_func
  ret

# RELAX:      .rela.text {
# RELAX-NEXT:   0x1 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0x1 R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0x7 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0x7 R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0xD R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0xD R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0x12 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0x12 R_X86_64_RELAX - 0x0
# RELAX-NEXT: }

# NORELAX:      .rela.text {
# NORELAX-NEXT:   0x1 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# NORELAX-NOT:    R_X86_64_RELAX
# NORELAX:        0x7 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# NORELAX-NOT:    R_X86_64_RELAX
# NORELAX:        0xD R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# NORELAX-NOT:    R_X86_64_RELAX
# NORELAX:        0x12 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# NORELAX-NOT:    R_X86_64_RELAX
# NORELAX:      }
