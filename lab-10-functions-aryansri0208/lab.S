// Functions Lab template file
.data

.global login_name
login_name:
    .asciz "sriva115"

.global puid
puid:
    .asciz "00345044038"

.balign 4

.equ TEST_SORT,         0x1
.equ TEST_STRCONCAT,    0x2
.equ TEST_FIB,          0x4
.equ TEST_BSEARCH,      0x8
.equ TEST_ALL,          (TEST_SORT | TEST_STRCONCAT | TEST_FIB | TEST_BSEARCH)

// Replace the TEST_ALL with the tests you want to run
// OR expression also support like the TEST_ALL above
// i.e. replace the TEST_ALL below with TEST_SORT | TEST_STRCONCAT
//      will test on the sort and strconcat
.global test
test:
    .word TEST_ALL


// Align to 4bytes boundary as we have a string above
.balign 4

.text
.global asm_sort_int
asm_sort_int:
    /* void asm_sort_int(int32_t* arr, uint32_t n)
     * relies on `qsort` in C standard library to 
     * sort in ascending order
     **/

    /* Enter your code after this comment */
    addi sp, sp, -16
    sw ra, 0(sp)
    sw a0, 4(sp)
    sw a1, 8(sp)

    li a2, 4
    la a3, asm_cmp
    jal ra, qsort

    lw ra, 0(sp)
    addi sp, sp, 16
    /* Enter your code above this comment */
    ret

.global asm_cmp
asm_cmp:
    // int asm_cmp(const void * a, const void * b)
    // Compare function to int32_t
    // used for qsort
    /* Enter your code after this comment */
    lw t0, 0(a0)
    lw t1, 0(a1)
    sub t2, t0, t1
    blt t2, zero, less
    li a0, 1
    ret
less:
    li a0, -1
    /* Enter your code above this comment */
    ret

.global asm_strconcat
asm_strconcat:
    /* char* asm_strconcat(char * str1, char * str2)
     * First assign memory space with `malloc`, 
     * concatenating `str1` and `str2`, 
     * and return the resulted string.
     **/
    /* Enter your code after this comment */
    addi sp, sp, -32
    sw ra, 0(sp)
    sw s0, 4(sp)     
    sw s1, 8(sp)     
    sw s2, 12(sp)    
    sw s3, 16(sp)    
    sw s4, 20(sp)    

    mv s0, a0        
    mv s1, a1        

    mv a0, s0
    jal ra, strlen
    mv s2, a0        

    mv a0, s1
    jal ra, strlen
    mv s3, a0        

    add a0, s2, s3
    addi a0, a0, 1
    jal ra, malloc
    mv s4, a0      

    mv a0, s4        
    mv a1, s0        
    mv a2, s2        
    jal ra, memcpy

    add a0, s4, s2   
    mv a1, s1        
    mv a2, s3        
    jal ra, memcpy

    add a0, s4, s2
    add a0, a0, s3
    sb zero, 0(a0)

    mv a0, s4        

    lw ra, 0(sp)
    lw s0, 4(sp)
    lw s1, 8(sp)
    lw s2, 12(sp)
    lw s3, 16(sp)
    lw s4, 20(sp)
    addi sp, sp, 32
    /* Enter your code above this comment */
    ret

.global asm_fib
asm_fib:
    /* uint32_t asm_fib(uint32_t n)
     * accept an index term `n` and return 
     * the Fn fibonacci term (F0 = 0, F1 = 1)
     **/

    /* Enter your code after this comment */
    addi sp, sp, -20
    sw ra, 0(sp)
    sw a0, 4(sp)
    sw s1, 8(sp)

    li t0, 2
    blt a0, t0, base_case

    addi a0, a0, -1
    jal ra, asm_fib
    mv s1, a0             

    lw a0, 4(sp)
    addi a0, a0, -2
    jal ra, asm_fib
    add a0, a0, s1        
    lw ra, 0(sp)
    lw s1, 8(sp)
    addi sp, sp, 20
    ret

base_case:
    lw ra, 0(sp)
    lw a0, 4(sp)
    addi sp, sp, 20
    /* Enter your code above this comment */
    ret

.global asm_bsearch
asm_bsearch:
    /* int32_t asm_bsearch(int *arr, int key, 
                    int32_t start,
                    int32_t end)
     * Return the index of the element in the array
     * if the element does not exist, return -1 instead
     * arr: integer array to be searched
     * key: the element we want to search on
     * start: start index of subarray, inclusive
     * end: end index of subarray, inclusiveloop through an entire string and only convert
     **/

    /* Enter your code after this comment */
    addi sp, sp, -32
    sw ra, 0(sp)
    sw a0, 4(sp)
    sw a1, 8(sp)
    sw a2, 12(sp)
    sw a3, 16(sp)

    bgt a2, a3, not_found

    add t0, a2, a3
    srai t0, t0, 1
    slli t1, t0, 2
    add t1, a0, t1
    lw t2, 0(t1)

    blt t2, a1, recurse_right
    bgt t2, a1, recurse_left

    mv a0, t0
    lw ra, 0(sp)
    addi sp, sp, 32
    ret

recurse_left:
    lw a0, 4(sp)
    lw a1, 8(sp)
    lw a2, 12(sp)
    addi a3, t0, -1
    jal ra, asm_bsearch
    lw ra, 0(sp)
    addi sp, sp, 32
    ret

recurse_right:
    lw a0, 4(sp)
    lw a1, 8(sp)
    addi a2, t0, 1
    lw a3, 16(sp)
    jal ra, asm_bsearch
    lw ra, 0(sp)
    addi sp, sp, 32
    ret

not_found:
    li a0, -1
    lw ra, 0(sp)
    addi sp, sp, 32
    /* Enter your code above this comment */
    ret
