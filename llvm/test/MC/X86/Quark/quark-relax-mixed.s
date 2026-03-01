# RUN: llvm-mc -filetype=obj -triple=x86_64 -mattr=+quark-relax %s \
# RUN:     | llvm-readobj -r - | FileCheck %s

## Test a realistic mix of branch, call, RIP-relative, and alignment
## scenarios all in one file.

.text
.globl _start
_start:
  # Direct call to external symbol.
  call extern_func
  # Conditional branch to local label.
  testq %rax, %rax
  je .Llocal
  # RIP-relative load from external symbol.
  movq extern_data(%rip), %rax
  # RIP-relative load from local label.
  movq .Llocal(%rip), %rcx
  # Unconditional jump to local label.
  jmp .Llocal
  .align 8
.Llocal:
  # Call to same-file global.
  call bar
  ret

.globl bar
bar:
  nop
  ret

# CHECK:      .rela.text {
## call extern_func
# CHECK-NEXT:   0x1 R_X86_64_PLT32 extern_func 0xFFFFFFFFFFFFFFFC
# CHECK-NEXT:   0x1 R_X86_64_RELAX - 0x0
## je .Llocal (local conditional branch)
# CHECK-NEXT:   0xA R_X86_64_PLT32 .text 0x24
# CHECK-NEXT:   0xA R_X86_64_RELAX - 0x0
## movq extern_data(%rip), %rax (RIP-relative to external)
# CHECK-NEXT:   0x11 R_X86_64_PC32 extern_data 0xFFFFFFFFFFFFFFFC
# CHECK-NEXT:   0x11 R_X86_64_RELAX - 0x0
## movq .Llocal(%rip), %rcx (RIP-relative to local)
# CHECK-NEXT:   0x18 R_X86_64_PC32 .text 0x24
# CHECK-NEXT:   0x18 R_X86_64_RELAX - 0x0
## jmp .Llocal
# CHECK-NEXT:   0x1D R_X86_64_PLT32 .text 0x24
# CHECK-NEXT:   0x1D R_X86_64_RELAX - 0x0
## .align 8 after relaxable code
# CHECK-NEXT:   0x21 R_X86_64_ALIGN - 0x7
## call bar (global within same file)
# CHECK-NEXT:   0x29 R_X86_64_PLT32 bar 0xFFFFFFFFFFFFFFFC
# CHECK-NEXT:   0x29 R_X86_64_RELAX - 0x0
# CHECK-NEXT: }
