# RUN: llvm-mc -filetype asm -triple riscv64_lfi %s | FileCheck %s

mv tp, t0

# CHECK:      mv      s10, ra
# CHECK-NEXT: xor     a0, a0, t0
# CHECK-NEXT: xor     t0, a0, t0
# CHECK-NEXT: xor     a0, a0, t0
# CHECK-NEXT: ld      ra, 16(s11)
# CHECK-NEXT: jalr    ra
# CHECK-NEXT: xor     a0, a0, t0
# CHECK-NEXT: xor     t0, a0, t0
# CHECK-NEXT: xor     a0, a0, t0
# CHECK-NEXT: add.uw  s1, s10, s11
# CHECK-NEXT: andi    ra, s1, -8

mv t0, tp

# CHECK:      mv      t0, a0
# CHECK-NEXT: mv      s10, ra
# CHECK-NEXT: ld      ra, 8(s11)
# CHECK-NEXT: jalr    ra
# CHECK-NEXT: xor     a0, a0, t0
# CHECK-NEXT: xor     t0, a0, t0
# CHECK-NEXT: xor     a0, a0, t0
# CHECK-NEXT: add.uw  s1, s10, s11
# CHECK-NEXT: andi    ra, s1, -8

add a0, a0, tp

# CHECK:      sd      a1, -8(sp)
# CHECK-NEXT: mv      a1, a0
# CHECK-NEXT: mv      s10, ra
# CHECK-NEXT: ld      ra, 8(s11)
# CHECK-NEXT: jalr    ra
# CHECK-NEXT: add.uw  s1, s10, s11
# CHECK-NEXT: andi    ra, s1, -8
# CHECK-NEXT: add     a0, a1, a0
# CHECK-NEXT: ld      a1, -8(sp)