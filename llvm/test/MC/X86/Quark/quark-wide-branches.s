# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-objdump -d - | FileCheck -check-prefix=RELAX %s
# RUN: llvm-mc -filetype=obj -triple=x86_64 %s \
# RUN:     | llvm-objdump -d - | FileCheck -check-prefix=NORELAX %s

## Test that Quark forces all branches to use 32-bit displacement,
## even when the target is nearby and would fit in 8 bits.

.text
.globl _start
_start:
  jmp .Lnext
.Lnext:
  je .Lnext
  jne .Lnext
  ret

## With quark-relax: all branches use wide encoding (e9, 0f 84, 0f 85).
# RELAX:      <_start>:
# RELAX-NEXT:  e9 00 00 00 00       jmp
# RELAX:       0f 84 00 00 00 00    je
# RELAX:       0f 85 00 00 00 00    jne
# RELAX:       c3                   retq

## Without quark-relax: branches use short encoding (eb, 74, 75).
# NORELAX:      <_start>:
# NORELAX-NEXT:  eb 00                jmp
# NORELAX:       74 fe                je
# NORELAX:       75 fc                jne
# NORELAX:       c3                   retq
