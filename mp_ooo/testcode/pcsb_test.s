ooo_test.s:
.align 4
.section .text
.globl _start
    # This program will provide a simple test for
    # demonstrating OOO-ness

    # This test is NOT exhaustive
_start:

# initialize
lui x1, 0xFFFF0
li x2, 20

nop
nop
nop
nop
nop

sw x2, 0(x1)
lw x3, 0(x1)

add x4, x3, x2

halt:
    slti x0, x0, -256
