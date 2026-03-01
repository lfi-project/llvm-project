# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 %s \
# RUN:     | llvm-readobj -r - | FileCheck -check-prefix=NORELAX %s

## Test that R_X86_64_ALIGN relocations are emitted for alignment directives
## that follow linker-relaxable instructions. Alignment directives before the
## first relaxable instruction should NOT get ALIGN relocations.

.text
.globl _start

## This alignment is before any relaxable instruction, so no R_X86_64_ALIGN.
  .align 16

_start:
  call extern_func

## This alignment follows a relaxable call, so it gets R_X86_64_ALIGN.
  .align 16
  nop

  call extern_func

## This alignment also follows relaxable code.
  .align 32
  ret

# RELAX:      .rela.text {
# RELAX-NEXT:   0x1 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   0x1 R_X86_64_RELAX - 0x0
# RELAX-NEXT:   0x5 R_X86_64_ALIGN - 0xF
# RELAX:        R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# RELAX-NEXT:   {{.*}} R_X86_64_RELAX - 0x0
# RELAX-NEXT:   {{.*}} R_X86_64_ALIGN - 0x1F
# RELAX-NEXT: }

## Without quark-relax, no ALIGN relocations are emitted.
# NORELAX:      .rela.text {
# NORELAX-NOT:  R_X86_64_ALIGN
# NORELAX-NOT:  R_X86_64_RELAX
# NORELAX:      }
