# RUN: llvm-mc -filetype asm -mattr=+c -triple riscv64_lfi %s | FileCheck %s

ld t1, 8(t0)

# CHECK:      add.uw  s1, t0, s11
# CHECK-NEXT: ld      t1, 8(s1)


lb t1, 8(t0)

# CHECK:      add.uw  s1, t0, s11
# CHECK-NEXT: lb      t1, 8(s1)


lhu t1, 8(t0)

# CHECK:      add.uw  s1, t0, s11
# CHECK-NEXT: lhu      t1, 8(s1)

sd t1, 8(t0)

# CHECK:      add.uw  s1, t0, s11
# CHECK-NEXT: sd      t1, 8(s1)


sb t1, 8(t0)

# CHECK:      add.uw  s1, t0, s11
# CHECK-NEXT: sb      t1, 8(s1)


sh t1, 8(t0)

# CHECK:      add.uw  s1, t0, s11
# CHECK-NEXT: sh      t1, 8(s1)

c.ld a0, (a1)

# CHECK:      add.uw  s1, a1, s11
# CHECK-NEXT: ld      a0, 0(s1)

sh t1, 8(sp)

# CHECK:    sh t1, 8(sp) 


c.sd a0, (a1)

# CHECK:      add.uw  s1, a1, s11
# CHECK-NEXT: sd      a0, 0(s1)

