// Control Flow Lab template file
.data
.global lowercase_string
lowercase_string:
    // Specify a string ending with '\0'
    .asciz "ece 362 is awesome!"

.global login_name
login_name:
    .asciz "sriva115"

.global puid
puid:
    .asciz "0034504038"

// Align to 4bytes boundary as we have a string above
.balign 4

.text
.global asm_strlen
asm_strlen:
    /* uint32_t asm_strlen(char *str)
     * Take in a pointer to a string and return
     * its length up to the null terminating
     * character `\0`
    **/   

    /* Enter your code after this comment */
    li t0, 0
    beqz x10, exit   

    loop:
        lb t1, 0(x10)
        beqz t1, exit
        addi x10, x10, 1
        addi t0, t0, 1
        j loop
    exit:
        mv x10, t0
    /* Enter your code above this comment */
    ret

.global asm_fib
asm_fib:
    /* void asm_fib(int *arr, uint32_t length)
     * Accept a pointer to an array of integer and
     * the length of the array, then generate fibonacci
     * series starting at `0` and store the result
     * back to the array `arr` at corresponding indices.
     **/

    /* Enter your code after this comment */
    beqz a0, fib_exit       # if array pointer is null
    beqz a1, fib_exit       # if length is 0
    li t0, 0                # prev = 0
    li t1, 1                # curr = 1
    li t2, 0                # i = 0
    li t5, 4                # word size = 4

fib_loop:
    bge t2, a1, fib_exit    # if i >= n, exit

    mul t4, t2, t5          # offset = i * 4
    add a3, a0, t4          # address = base + offset

    beqz t2, fib_0          # if i == 0
    li t6, 1
    beq t2, t6, fib_1       # if i == 1

    add t3, zero, t1        # t3 = t1 (curr)
    add t1, t0, t1          # curr = prev + curr
    mv t0, t3               # prev = old curr
    sw t1, 0(a3)            # store curr
    addi t2, t2, 1          # i++
    j fib_loop

fib_0:
    sw t0, 0(a3)
    addi t2, t2, 1
    j fib_loop

fib_1:
    sw t1, 0(a3)
    addi t2, t2, 1
    j fib_loop

fib_exit:
    ret

.global asm_toUppercase
asm_toUppercase:
    /* void asm_toUppercase(char *str)
     * loop through an entire string and only convert
     * the lowercase letter to uppercase, leaving the
     * rest of the string intact
     **/

    /* Enter your code after this comment */
    beqz x10, case_exit // edge case
    li t1, 97
    li t2, 122
    li t3, 32
    case_loop:
        lb t0, 0(x10)
        beqz t0, case_exit
        blt t0, t1, skip
        bgt t0, t2, skip
        sub t4, t0, t3
        sb t4, 0(x10)
        addi x10, x10, 1
        j case_loop
    skip:
        addi x10, x10, 1
        j case_loop
    /* Enter your code above this comment */
    case_exit:
        ret
