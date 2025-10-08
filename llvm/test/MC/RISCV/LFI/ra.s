# RUN: llvm-mc -filetype asm -triple riscv64_lfi %s | FileCheck %s

ld ra, (sp)

# CHECK: ld      s10, 0(sp)
# CHECK-NEXT: add.uw  s1, s10, s11
# CHECK-NEXT: andi    ra, s1, -8

mv ra, t0

# CHECK: mv s10, t0
# CHECK-NEXT: add.uw  s1, s10, s11
# CHECK-NEXT: andi    ra, s1, -8
