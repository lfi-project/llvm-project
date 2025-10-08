# RUN: llvm-mc -filetype asm -triple riscv64_lfi %s | FileCheck %s

addi sp, sp, 8

# CHECK:      addi    s10, sp, 8
# CHECK-NEXT: add.uw  sp, s10, s11

add sp, sp, t0

# CHECK:      add    s10, sp, t0
# CHECK-NEXT: add.uw  sp, s10, s11

sub sp, sp, t0

# CHECK:      sub    s10, sp, t0
# CHECK-NEXT: add.uw  sp, s10, s11

mv sp, t0

# CHECK:      mv s10, t0
# CHECK-NEXT: add.uw  sp, s10, s11


