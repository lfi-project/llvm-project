# RUN: llvm-mc -filetype asm -triple riscv64_lfi %s | FileCheck %s

# ---------------------------------------------------------------------------
# jalr xN, xM, <offset>  ==>  add.uw s1, xM, s11 ; andi s9, s1, -8 ; jalr xN, s9, <offset>
# ---------------------------------------------------------------------------

# Pick some normal registers for rd=xN and rs1=xM.
# a0 = x10  ,  t2 = x7
jalr a0, t2, 12
# CHECK:      add.uw s1, t2, s11
# CHECK-NEXT: andi   s9, s1, -8
# CHECK-NEXT: jalr   a0, 12(s9)

# Another jalr example, different regs and offset
# s0 = x8 , s3 = x19
jalr s0, s3, -4
# CHECK:      add.uw s1, s3, s11
# CHECK-NEXT: andi   s9, s1, -8
# CHECK-NEXT: jalr   s0, -4(s9)
 
# ---------------------------------------------------------------------------
# jalr ra, xM, <offset>  ==> add.uw/andi + bundle-locked jalr ra, s9, <offset>
# ---------------------------------------------------------------------------

# ra = x1
jalr ra, s3, 0
# CHECK:      add.uw s1, s3, s11
# CHECK-NEXT: andi   s9, s1, -8
# CHECK-NEXT: .bundle_lock align_to_end
# CHECK-NEXT: jalr   s9
# CHECK-NEXT: .bundle_unlock

# ---------------------------------------------------------------------------
# jr xM  (pseudoinst for jalr x0, xM, 0)
# ---------------------------------------------------------------------------

jr s3
# CHECK:      add.uw s1, s3, s11
# CHECK-NEXT: andi   s9, s1, -8
# CHECK-NEXT: jr   s9

# ---------------------------------------------------------------------------
# jal xN, <label>
#   - rd != ra: stays as plain jal
#   - rd == ra: wrapped in bundle lock to align-to-end
# ---------------------------------------------------------------------------

# rd != ra
jal t1, L_nonra
# CHECK:      jal    t1, L_nonra

# rd == ra
jal ra, L_ret
# CHECK:      .bundle_lock align_to_end
# CHECK-NEXT: jal    L_ret
# CHECK-NEXT: .bundle_unlock



# Targets
L_nonra:
  nop
L_ret:
  nop
