# 0 "/users/ddegimli/Cores-VeeR-EH2-PD-UPC/testbench/asm/pmp_test.s"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "/users/ddegimli/Cores-VeeR-EH2-PD-UPC/testbench/asm/pmp_test.s"







# 1 "snapshots/default/defines.h" 1
# 9 "/users/ddegimli/Cores-VeeR-EH2-PD-UPC/testbench/asm/pmp_test.s" 2
# 49 "/users/ddegimli/Cores-VeeR-EH2-PD-UPC/testbench/asm/pmp_test.s"
.macro get_tid reg=a0
    csrr \reg, mhartid
    andi \reg, \reg, 0xf
.endm


.macro fork targ, reg=a0
    get_tid \reg
    bnez \reg, \targ
.endm


.macro pmp_sync
    .rept 8
    nop
    .endr
.endm



.macro clear_all_pmp
    csrw 0x3A0, zero
    csrw 0x3A1, zero
    csrw 0x3A2, zero
    csrw 0x3A3, zero
    csrw 0x3B0, zero
    csrw 0x3B1, zero
    csrw 0x3B2, zero
    csrw 0x3B3, zero
    csrw 0x3B4, zero
    csrw 0x3B5, zero
    csrw 0x3B6, zero
    csrw 0x3B7, zero
    csrw 0x3B8, zero
    csrw 0x3B9, zero
    csrw 0x3BA, zero
    csrw 0x3BB, zero
    csrw 0x3BC, zero
    csrw 0x3BD, zero
    csrw 0x3BE, zero
    csrw 0x3BF, zero
    pmp_sync
.endm


.macro arm_trap expect_cause
    li t0, 1
    la t1, expect_trap_flag
    sw t0, 0(t1)
    li t0, \expect_cause
    la t1, expect_cause_val
    sw t0, 0(t1)
.endm


.macro disarm_trap
    la t1, expect_trap_flag
    sw zero, 0(t1)
.endm



.macro check_trapped
    la t6, expect_trap_flag
    lw t5, 0(t6)
    beqz t5, .Lct_skip_\@
    inc_fail
.Lct_skip_\@:
    disarm_trap
.endm





.macro inc_pass
    get_tid t5
    bnez t5, .Lincp_h1_\@
    la t6, h0_pass_count
    j .Lincp_end_\@
.Lincp_h1_\@:
    la t6, h1_pass_count
.Lincp_end_\@:
    lw t4, 0(t6)
    addi t4, t4, 1
    sw t4, 0(t6)
.endm

.macro inc_fail
    get_tid t5
    bnez t5, .Lincf_h1_\@
    la t6, h0_fail_count
    j .Lincf_end_\@
.Lincf_h1_\@:
    la t6, h1_fail_count
.Lincf_end_\@:
    lw t4, 0(t6)
    addi t4, t4, 1
    sw t4, 0(t6)
.endm


.macro dbg_char c
    li t0, 0xd0580000
    li t1, \c
    sb t1, 0(t0)
.endm




.section .text
.global _start
.align 4

_start:
    csrw minstret, zero
    csrw minstreth, zero


    la t0, trap_entry
    csrw mtvec, t0


    li t0, 0x5f555555
    csrw 0x7c0, t0

    clear_all_pmp


    dbg_char '#'

    fork hart1_main




    la t0, h0_pass_count
    sw zero, 0(t0)
    la t0, h0_fail_count
    sw zero, 0(t0)
    la t0, h1_pass_count
    sw zero, 0(t0)
    la t0, h1_fail_count
    sw zero, 0(t0)
    la t0, h1_done_flag
    sw zero, 0(t0)
    la t0, expect_trap_flag
    sw zero, 0(t0)

    csrwi 0x7fc, 3
    li t0, 1
    la t1, h1_start_flag
    sw t0, 0(t1)

    jal ra, h0_run_tests


1: la t0, h1_done_flag
    lw t1, 0(t0)
    beqz t1, 1b

    jal ra, h0_report_and_finish
    j spin_forever




hart1_main:
    la t0, h1_pass_count
    sw zero, 0(t0)
    la t0, h1_fail_count
    sw zero, 0(t0)

1: la t0, h1_start_flag
    lw t1, 0(t0)
    beqz t1, 1b

    jal ra, h1_run_tests

    li t0, 1
    la t1, h1_done_flag
    sw t0, 0(t1)
    j spin_forever

spin_forever:
    j spin_forever







h0_run_tests:
    la t0, h0_saved_ra
    sw ra, 0(t0)
    jal ra, t_csr_basic
    jal ra, t_l0_mmode_bypass
    jal ra, t_locked_tor_load_store
    jal ra, t_locked_fetch_fault
    jal ra, t_napot_rw_fault
    jal ra, t_aoff_disabled
    jal ra, t_priority_entry0_wins
    jal ra, t_lock_and_tor_lock
    la t0, h0_saved_ra
    lw ra, 0(t0)
    ret




h1_run_tests:
    la t0, h1_saved_ra
    sw ra, 0(t0)
    jal ra, t_h1_na4_rw
    jal ra, t_h1_napot_16b
    jal ra, t_h1_chained_tor
    la t0, h1_saved_ra
    lw ra, 0(t0)
    ret





t_csr_basic:
    dbg_char '1'
    clear_all_pmp


    li t0, 0x12345
    csrw 0x3B3, t0
    csrr t2, 0x3B3
    bne t0, t2, 1f
    inc_pass
    j 2f
1: inc_fail
2:

    li t0, (((0)<<7)|((1)<<3)|((0)<<2)|((0)<<1)|(1))
    csrw 0x3A0, t0
    csrr t2, 0x3A0
    andi t2, t2, 0xFF
    bne t0, t2, 3f
    inc_pass
    j 4f
3: inc_fail
4:
    clear_all_pmp
    ret





t_l0_mmode_bypass:
    dbg_char '2'
    clear_all_pmp


    la t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw 0x3B0, t0
    li t0, (((0)<<7)|((1)<<3)|((0)<<2)|((0)<<1)|(0))
    csrw 0x3A0, t0
    pmp_sync

    la t0, guard_a_data
    lw t1, 0(t0)
    sw t1, 0(t0)
    inc_pass

    clear_all_pmp
    ret





t_locked_tor_load_store:
    dbg_char '3'
    clear_all_pmp


    la t0, guard_a_data
    srli t0, t0, 2
    csrw 0x3B0, t0


    la t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw 0x3B1, t0


    li t0, ((((1)<<7)|((1)<<3)|((0)<<2)|((0)<<1)|(1)) << 8)
    csrw 0x3A0, t0
    pmp_sync

    la t0, guard_a_data
    lw t1, 0(t0)
    inc_pass

    arm_trap 7
    la t0, guard_a_data
    li t1, 0xdeadbeef
    sw t1, 0(t0)
    check_trapped

    clear_all_pmp
    ret
# 372 "/users/ddegimli/Cores-VeeR-EH2-PD-UPC/testbench/asm/pmp_test.s"
t_locked_fetch_fault:
    dbg_char '4'


    la t0, fetch4_saved_ra
    sw ra, 0(t0)

    clear_all_pmp


    li t0, (((1)<<7)|((2)<<3)|((0)<<2)|((0)<<1)|(1))
    csrw 0x3A0, t0
    la t0, fetch_island
    srli t0, t0, 2
    csrw 0x3B0, t0
    pmp_sync





    arm_trap 1
    la t0, fetch_island
    jalr ra, t0, 0
    check_trapped


    la t0, fetch4_saved_ra
    lw ra, 0(t0)
    clear_all_pmp
    ret






t_napot_rw_fault:
    dbg_char '5'
    clear_all_pmp

    li t0, (((1)<<7)|((3)<<3)|((0)<<2)|((0)<<1)|(1))
    csrw 0x3A0, t0
    la t0, guard_a_data
    srli t0, t0, 2
    csrw 0x3B0, t0
    pmp_sync

    la t0, guard_a_data
    lw t1, 0(t0)
    inc_pass

    arm_trap 7
    la t0, guard_a_data
    li t1, 0xA5A5A5A5
    sw t1, 0(t0)
    check_trapped

    clear_all_pmp
    ret





t_aoff_disabled:
    dbg_char '6'
    clear_all_pmp


    li t0, (((1)<<7)|((0)<<3)|((0)<<2)|((0)<<1)|(0))
    csrw 0x3A0, t0
    la t0, guard_a_data
    srli t0, t0, 2
    csrw 0x3B0, t0
    pmp_sync


    la t0, guard_a_data
    li t1, 0x12345678
    sw t1, 0(t0)
    lw t2, 0(t0)
    bne t1, t2, 1f
    inc_pass
    j 2f
1: inc_fail
2:
    clear_all_pmp
    ret






t_priority_entry0_wins:
    dbg_char '7'
    clear_all_pmp

    la t3, guard_a_data
    srli t3, t3, 2



    li t0, (((1)<<7)|((2)<<3)|((0)<<2)|((0)<<1)|(1))
    li t1, ((((1)<<7)|((1)<<3)|((1)<<2)|((1)<<1)|(1)) << 8)
    or t0, t0, t1
    csrw 0x3A0, t0


    csrw 0x3B0, t3


    la t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw 0x3B1, t0
    pmp_sync


    la t0, guard_a_data
    lw t1, 0(t0)
    inc_pass


    arm_trap 7
    li t1, 0x55
    sw t1, 0(t0)
    check_trapped

    clear_all_pmp
    ret





t_lock_and_tor_lock:
    dbg_char '8'
    clear_all_pmp


    la t0, guard_a_data
    srli t0, t0, 2
    csrw 0x3B0, t0

    la t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw 0x3B1, t0

    li t0, ((((1)<<7)|((1)<<3)|((0)<<2)|((0)<<1)|(1)) << 8)
    csrw 0x3A0, t0
    pmp_sync


    csrw 0x3A0, zero
    csrr t2, 0x3A0
    li t3, ((((1)<<7)|((1)<<3)|((0)<<2)|((0)<<1)|(1)) << 8)
    bne t2, t3, 1f
    inc_pass
    j 2f
1: inc_fail
2:

    la t3, guard_a_data
    srli t3, t3, 2
    li t0, 0xFFFFF
    csrw 0x3B0, t0
    csrr t2, 0x3B0
    bne t2, t3, 3f
    inc_pass
    j 4f
3: inc_fail
4:

    la t3, guard_a_data
    addi t3, t3, 64
    srli t3, t3, 2
    csrw 0x3B1, zero
    csrr t2, 0x3B1
    bne t2, t3, 5f
    inc_pass
    j 6f
5: inc_fail
6:

    clear_all_pmp
    csrr t2, 0x3A0
    li t3, ((((1)<<7)|((1)<<3)|((0)<<2)|((0)<<1)|(1)) << 8)
    bne t2, t3, 7f
    inc_pass
    j 8f
7: inc_fail
8:
    ret




t_h1_na4_rw:
    dbg_char 'a'
    clear_all_pmp


    la t0, guard_b_data
    srli t0, t0, 2
    csrw 0x3B0, t0
    li t0, (((1)<<7)|((2)<<3)|((0)<<2)|((1)<<1)|(1))
    csrw 0x3A0, t0
    pmp_sync

    la t0, guard_b_data
    li t1, 0x12345678
    sw t1, 0(t0)
    lw t2, 0(t0)
    bne t1, t2, 1f
    inc_pass
    j 2f
1: inc_fail
2:
    clear_all_pmp
    ret






t_h1_napot_16b:
    dbg_char 'b'
    clear_all_pmp

    li t0, (((1)<<7)|((3)<<3)|((0)<<2)|((0)<<1)|(1))
    csrw 0x3A0, t0
    la t0, guard_b_data
    srli t0, t0, 2
    ori t0, t0, 0x1
    csrw 0x3B0, t0
    pmp_sync

    la t0, guard_b_data
    lw t1, 0(t0)
    inc_pass

    arm_trap 7
    la t0, guard_b_data
    li t1, 0xDEADC0DE
    sw t1, 0(t0)
    check_trapped

    clear_all_pmp
    ret




t_h1_chained_tor:
    dbg_char 'c'
    clear_all_pmp

    la t0, guard_b_data
    srli t0, t0, 2
    csrw 0x3B0, t0

    la t0, guard_b_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw 0x3B1, t0

    li t0, ((((1)<<7)|((1)<<3)|((0)<<2)|((1)<<1)|(1)) << 8)
    csrw 0x3A0, t0
    pmp_sync

    la t0, guard_b_data
    li t1, 0xa5a5a5a5
    sw t1, 0(t0)
    lw t2, 0(t0)
    bne t1, t2, 1f
    inc_pass
    j 2f
1: inc_fail
2:
    clear_all_pmp
    ret




h0_report_and_finish:
    dbg_char 'E'

    la t0, h0_pass_count
    lw s1, 0(t0)
    la t0, h1_pass_count
    lw t1, 0(t0)
    add s1, s1, t1

    la t0, h0_fail_count
    lw s2, 0(t0)
    la t0, h1_fail_count
    lw t1, 0(t0)
    add s2, s2, t1

    la t0, msg_banner
    jal ra, print_str

    la t0, msg_pass
    jal ra, print_str
    mv a0, s1
    jal ra, print_dec

    la t0, msg_fail
    jal ra, print_str
    mv a0, s2
    jal ra, print_dec

    la t0, msg_nl
    jal ra, print_str

    bnez s2, test_fail
    li t0, 0xd0580000
    li t1, 0xff
    sb t1, 0(t0)
1: j 1b

test_fail:
    li t0, 0xd0580000
    li t1, 0x01
    sb t1, 0(t0)
2: j 2b




print_str:
    li t1, 0xd0580000
1: lbu t2, 0(t0)
    beqz t2, 2f
    sb t2, 0(t1)
    addi t0, t0, 1
    j 1b
2: ret




print_dec:
    li t1, 0xd0580000
    li t2, 10
    blt a0, t2, 1f
    div t3, a0, t2
    rem a0, a0, t2
    addi t3, t3, '0'
    sb t3, 0(t1)
1: addi a0, a0, '0'
    sb a0, 0(t1)
    ret




.align 6
trap_entry:
    csrr t0, mcause
    csrr t1, mtval
    csrr t2, mepc


    la t3, trap_cause_val
    sw t0, 0(t3)
    la t3, trap_mtval_val
    sw t1, 0(t3)
    la t3, trap_epc_val
    sw t2, 0(t3)

    la t3, expect_trap_flag
    lw t4, 0(t3)
    beqz t4, trap_unexpected


    sw zero, 0(t3)
    la t3, expect_cause_val
    lw t4, 0(t3)
    bne t0, t4, trap_wrong_cause


    get_tid t5
    bnez t5, trap_h1_pass
    la t6, h0_pass_count
    j trap_inc_pass
trap_h1_pass:
    la t6, h1_pass_count
trap_inc_pass:
    lw t4, 0(t6)
    addi t4, t4, 1
    sw t4, 0(t6)
    addi t2, t2, 4
    csrw mepc, t2
    mret

trap_wrong_cause:

    get_tid t5
    bnez t5, trap_wc_h1
    la t6, h0_fail_count
    j trap_wc_inc
trap_wc_h1:
    la t6, h1_fail_count
trap_wc_inc:
    lw t4, 0(t6)
    addi t4, t4, 1
    sw t4, 0(t6)
    addi t2, t2, 4
    csrw mepc, t2
    mret

trap_unexpected:


    get_tid t5
    bnez t5, trap_unx_h1
    la t6, h0_fail_count
    j trap_unx_inc
trap_unx_h1:
    la t6, h1_fail_count
trap_unx_inc:
    lw t4, 0(t6)
    addi t4, t4, 1
    sw t4, 0(t6)
    addi t2, t2, 4
    csrw mepc, t2
    mret
# 813 "/users/ddegimli/Cores-VeeR-EH2-PD-UPC/testbench/asm/pmp_test.s"
.option push
.option norvc
.align 2
fetch_island:
    addi a0, a0, 1
    jalr x0, 0(ra)
.option pop




.section .data


.align 2
h0_pass_count: .word 0
h0_fail_count: .word 0
h1_pass_count: .word 0
h1_fail_count: .word 0
h1_done_flag: .word 0
h1_start_flag: .word 0
expect_trap_flag: .word 0
expect_cause_val: .word 0


h0_saved_ra: .word 0
h1_saved_ra: .word 0
fetch4_saved_ra: .word 0


trap_cause_val: .word 0
trap_mtval_val: .word 0
trap_epc_val: .word 0


.align 3
guard_a_data: .fill 64, 1, 0


.align 4
guard_b_data: .fill 64, 1, 0


msg_banner: .asciz "\nVeeR-EH2 PMP test\n"
msg_pass: .asciz "  pass="
msg_fail: .asciz " fail="
msg_nl: .asciz "\n"
