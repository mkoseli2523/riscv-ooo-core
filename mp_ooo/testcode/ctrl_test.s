.section .text
.globl _start
_start:

    # initialize
    li x1, 0
    li x2, 5
    li x3, 0
    li x4, 30
    li x5, 50
    li x6, 60
    li x7, 62
    li x8, 21
    li x9, 28
    li x10, 33
    li x11, 8
    li x12, 4
    li x13, 18
    li x14, 3
    li x15, 1

# -------------------------
# bne test
loop_bne:
    addi x1, x1, 1
    addi x3, x3, 1
    bne x1, x2, loop_bne

# -------------------------
# beq test
    li x16, 10
    li x17, 10
loop_beq:
    addi x1, x1, 1
    addi x3, x3, 1
    beq x16, x17, after_beq
    # j after_beq
    addi x1, x1, 1
    addi x3, x3, 1
    j loop_beq
after_beq:

# -------------------------
# blt test
    li x18, 1
    li x19, 20
loop_blt:
    addi x18, x18, 1
    blt x18, x19, loop_blt

# -------------------------
# bge test
    li x20, 20
    li x21, 5
loop_bge:
    addi x20, x20, -1
    bge x20, x21, loop_bge

# -------------------------
# bltu test (unsigned)
    li x22, 1
    li x23, 20
loop_bltu:
    addi x22, x22, 1
    bltu x22, x23, loop_bltu

# -------------------------
# bgeu test (unsigned)
    li x24, 20
    li x25, 5
loop_bgeu:
    addi x24, x24, -1
    bgeu x24, x25, loop_bgeu

# -------------------------
# jal test (jump forward)
    jal x26, after_jal
dead_jal:
    li x27, 0xDEAD  # Should never execute
after_jal:

# -------------------------
# jalr test (jump indirect)
    la x28, after_jalr
    jalr x29, 0(x28)
dead_jalr:
    li x30, 0xBEEF  # Should never execute
after_jalr:

# -------------------------

# -------------------------
# bne not taken (edge case: equality)
    li x31, 42
    li x2, 42
loop_bne_eq:
    bne x31, x2, bne_fail  # Should NOT branch
    j bne_pass
bne_fail:
    li x1, 0xBAD
bne_pass:

# -------------------------
# beq not taken (edge case: inequality)
    li x31, 99
    li x2, 100
    beq x31, x2, beq_fail  # Should NOT branch
    j beq_pass
beq_fail:
    li x1, 0xBAD
beq_pass:

# -------------------------
# blt equal (not less)
    li x1, 50
    li x2, 50
    blt x1, x2, blt_fail  # Not taken
    j blt_pass
blt_fail:
    li x1, 0xBAD
blt_pass:

# -------------------------
# bge equal (should be taken)
    li x1, 99
    li x2, 99
    bge x1, x2, bge_pass
    j bge_fail
bge_fail:
    li x1, 0xBAD
bge_pass:

# -------------------------
# bltu wrapping edge case
    li x1, -1          # 0xFFFFFFFF (unsigned large)
    li x2, 0
    bltu x1, x2, bltu_fail  # Should NOT branch
    j bltu_pass
bltu_fail:
    li x1, 0xBAD
bltu_pass:

# -------------------------
# bgeu wrapping edge case
    li x1, 0
    li x2, -1          # 0xFFFFFFFF (unsigned large)
    bgeu x1, x2, bgeu_fail  # Should NOT branch
    j bgeu_pass
bgeu_fail:
    li x1, 0xBAD
bgeu_pass:

# -------------------------
# jal backward (infinite loop if not careful)
    jal x1, backward_label
    nop
    nop
    nop
    nop
backward_label:
    # x1 contains return addr
    nop
    nop

# -------------------------
# jalr with offset
    la x5, target_jalr_offset
    addi x5, x5, -4     # Point before label
    jalr x6, 4(x5)      # Should jump to target_jalr_offset
dead_jalr_offset:
    li x7, 0xBAD
target_jalr_offset:

# -------------------------
# jalr to unaligned address (should raise exception if enforced)
    la x8, after_jalr_unaligned
    addi x8, x8, 1      # Unaligned address
    jalr x9, 0(x8)      # On real hardware, may trap
dead_jalr_unaligned:
    li x10, 0xBADD
after_jalr_unaligned:

end:
    nop
    nop
    nop
    nop
    nop

halt:
    slti x0, x0, -256
