# RUN: llvm-mc -filetype asm -triple riscv64_lfi %s | FileCheck %s

ecall

# CHECK:      mv      s10, ra
# CHECK-NEXT: ld      ra, 0(s11)
# CHECK-NEXT: jalr    ra
# CHECK-NEXT: add.uw  s1, s10, s11
# CHECK-NEXT: andi    ra, s1, -8