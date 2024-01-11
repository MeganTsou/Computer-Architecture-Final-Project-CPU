.data
    n: .word 10
    
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
    # You should store the output into x10
    addi sp, sp, -8
    sw a0, 4(sp) # store n
    sw x1, 0(sp) # store return address
    addi x7, x0, 1 # x7 = 1
    bne a0, x7, L1 # if (n != 1) L1
    addi x10, x0, 2 # T(1) = 2
    addi sp,sp,8 # pop stack of n = 1
    beq ra, x0, result
    jalr x0, 0(x1)
    
L1:
    srli a0, a0, 1 # n = n/2 (floor)
    jal x1, FUNCTION
    lw x1, 0(sp) # load return address
    lw a0, 4(sp) # load n
    addi sp, sp, 8 # pop
    addi x7, x0, 5
    mul x10, x10, x7 # 5T(n/2)
    addi x7, x0, 6
    mul a0, a0, x7 # 6n
    addi a0, a0, 4
    add x10, x10, a0 # T(n) = 5T(n/2) + 6n + 4
    beq ra, x0, result
    jalr x0, 0(x1)
    
result:
    
  # Ends the program with status code 0
    addi a0, x0, 10
    ecall

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall