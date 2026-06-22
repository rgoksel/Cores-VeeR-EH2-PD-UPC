// SPDX-License-Identifier: Apache-2.0
// Reset-isolated PMP hart1 TOR test for VeeR-EH2.
//
// Checkpoint:
//      h1.chained_tor_rw

#include "defines.h"

#define STDOUT          RV_SERIALIO

// PMP CSR addresses
#define PMPCFG0         0x3A0
#define PMPCFG1         0x3A1
#define PMPCFG2         0x3A2
#define PMPCFG3         0x3A3
#define PMPADDR0        0x3B0
#define PMPADDR1        0x3B1

// A-field encoding
#define A_OFF           0
#define A_TOR           1
#define A_NA4           2
#define A_NAPOT         3

#define cfg_byte(l,a,x,w,r)  (((l)<<7)|((a)<<3)|((x)<<2)|((w)<<1)|(r))

// Per-checkpoint record layout (4 words = 16 bytes):
//   +0  status  (0=not-run 1=pass 2=fail-value 3=fail-cause 4=fail-no-trap 5=unexpected-trap)
//   +4  d0      (value: got      / cause: got-cause   / unexpected: cause)
//   +8  d1      (value: exp      / cause: exp-cause    / unexpected: mtval)
//   +12 d2      (                  cause: mtval        / unexpected: mepc)
#define REC_SHIFT       4
#define ST_PASS         1
#define ST_FAILVAL      2
#define ST_FAILCAUSE    3
#define ST_NOTRAP       4
#define ST_UNEXP        5

#define H1_NUM_CP       1

// ---------------------------------------------------------------------------
// Generic macros
// ---------------------------------------------------------------------------

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
    csrw PMPCFG0, zero
    csrw PMPCFG1, zero
    csrw PMPCFG2, zero
    csrw PMPCFG3, zero
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

.macro dbg_char c
    li   t0, STDOUT
    li   t1, \c
    sb   t1, 0(t0)
.endm

// ---------------------------------------------------------------------------
// Result-recording macros
// ---------------------------------------------------------------------------

.macro set_cp id
    li   t0, \id
    get_tid t1
    bnez t1, .Lsc_h1_\@
    la   t1, h0_cur_id
    j    .Lsc_w_\@
.Lsc_h1_\@:
    la   t1, h1_cur_id
.Lsc_w_\@:
    sw   t0, 0(t1)
.endm

.macro rec_addr_t6
    get_tid t6
    bnez t6, .Lrat_h1_\@
    la   t6, h0_records
    la   t5, h0_cur_id
    j    .Lrat_l_\@
.Lrat_h1_\@:
    la   t6, h1_records
    la   t5, h1_cur_id
.Lrat_l_\@:
    lw   t5, 0(t5)
    slli t5, t5, REC_SHIFT
    add  t6, t6, t5
.endm

.macro rec_pass
    rec_addr_t6
    lw   t5, 0(t6)
    bnez t5, .Lrp_skip_\@
    li   t5, ST_PASS
    sw   t5, 0(t6)
.Lrp_skip_\@:
.endm

.macro rec_failval g, e
    mv   t4, \e
    mv   t3, \g
    rec_addr_t6
    lw   t5, 0(t6)
    bnez t5, .Lrfv_skip_\@
    li   t5, ST_FAILVAL
    sw   t5, 0(t6)
    sw   t3, 4(t6)
    sw   t4, 8(t6)
.Lrfv_skip_\@:
.endm

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
.section .text
.global _start
.align 4

_start:
    csrw minstret, zero
    csrw minstreth, zero

    la   t0, trap_entry
    csrw mtvec, t0

    li   t0, 0x5f555555
    csrw 0x7c0, t0

    clear_all_pmp

    dbg_char '#'

    fork hart1_main

    // Zero the result/flag region before releasing hart1.
    la   t0, zero_region_start
    la   t1, zero_region_end
1:  sw   zero, 0(t0)
    addi t0, t0, 4
    bltu t0, t1, 1b

    csrwi 0x7fc, 3              // mhartstart: enable hart1
    li   t0, 1
    la   t1, h1_start_flag
    sw   t0, 0(t1)              // release hart1

1:  la   t0, h1_done_flag       // wait for hart1
    lw   t1, 0(t0)
    beqz t1, 1b

    jal  ra, h0_report_and_finish
    j    spin_forever

// ---------------------------------------------------------------------------
// Hart 1 entry
// ---------------------------------------------------------------------------
hart1_main:
    dbg_char 'H'
1:  la   t0, h1_start_flag
    lw   t1, 0(t0)
    beqz t1, 1b

    jal  ra, h1_run_tests

    li   t0, 1
    la   t1, h1_done_flag
    sw   t0, 0(t1)
    j    spin_forever

spin_forever:
    j spin_forever

// ---------------------------------------------------------------------------
// Hart 1 test suite
// ---------------------------------------------------------------------------
h1_run_tests:
    la   t0, h1_saved_ra
    sw   ra, 0(t0)
    jal  ra, t_h1_chained_tor
    la   t0, h1_saved_ra
    lw   ra, 0(t0)
    ret

// ---------------------------------------------------------------------------
// Hart 1 - Chained TOR R=1 W=1       (h1 cp0 = h1.chained_tor_rw)
// ---------------------------------------------------------------------------
t_h1_chained_tor:
    dbg_char 'c'
    clear_all_pmp

    la   t0, guard_b_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    la   t0, guard_b_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR1, t0
    li   t0, (cfg_byte(1, A_TOR, 0, 1, 1) << 8)
    csrw PMPCFG0, t0
    pmp_sync

    set_cp 0
    la   t0, guard_b_data
    li   t1, 0xa5a5a5a5
    sw   t1, 0(t0)              // W=1 -> succeed
    lw   t2, 0(t0)              // R=1 -> succeed
    beq  t1, t2, 1f
    rec_failval t2, t1
    j    2f
1:  rec_pass
2:
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Report and finish
// ---------------------------------------------------------------------------
h0_report_and_finish:
    dbg_char 'E'

    la   t0, msg_banner
    jal  ra, print_str

    la   t0, msg_h1hdr
    jal  ra, print_str
    la   a0, h1_records
    la   a1, h1_names
    li   a2, H1_NUM_CP
    jal  ra, print_report

    la   t0, msg_total
    jal  ra, print_str
    la   t0, total_pass
    lw   a0, 0(t0)
    jal  ra, print_dec
    la   t0, msg_total_fail
    jal  ra, print_str
    la   t0, total_fail
    lw   a0, 0(t0)
    jal  ra, print_dec
    la   t0, msg_nl
    jal  ra, print_str

    la   t0, total_fail
    lw   t1, 0(t0)
    bnez t1, test_fail
    li   t0, STDOUT
    li   t1, 0xff               // TEST_PASSED
    sb   t1, 0(t0)
1:  j 1b
test_fail:
    li   t0, STDOUT
    li   t1, 0x01               // TEST_FAILED
    sb   t1, 0(t0)
2:  j 2b

// ---------------------------------------------------------------------------
// print_report: a0=&records, a1=&name_ptr_table, a2=count.
// ---------------------------------------------------------------------------
print_report:
    la   t0, pr_saved_ra
    sw   ra, 0(t0)
    mv   s1, a0                 // records base
    mv   s2, a1                 // names base
    mv   s3, a2                 // count
    li   s4, 0                  // index
pr_loop:
    bge  s4, s3, pr_done

    slli t0, s4, 2
    add  t0, s2, t0
    lw   t0, 0(t0)
    jal  ra, print_str
    la   t0, msg_colon
    jal  ra, print_str

    slli t0, s4, REC_SHIFT
    add  s5, s1, t0             // &record
    lw   s6, 0(s5)              // status

    li   t1, ST_PASS
    bne  s6, t1, pr_tally_fail
    la   t1, total_pass
    lw   t2, 0(t1)
    addi t2, t2, 1
    sw   t2, 0(t1)
    j    pr_dispatch
pr_tally_fail:
    beqz s6, pr_dispatch
    la   t1, total_fail
    lw   t2, 0(t1)
    addi t2, t2, 1
    sw   t2, 0(t1)

pr_dispatch:
    li   t1, ST_PASS
    beq  s6, t1, pr_pass
    li   t1, ST_FAILVAL
    beq  s6, t1, pr_failval
    li   t1, ST_FAILCAUSE
    beq  s6, t1, pr_failcause
    li   t1, ST_NOTRAP
    beq  s6, t1, pr_notrap
    li   t1, ST_UNEXP
    beq  s6, t1, pr_unexp
    la   t0, msg_notrun
    jal  ra, print_str
    j    pr_next

pr_pass:
    la   t0, msg_pass_w
    jal  ra, print_str
    j    pr_next

pr_failval:
    la   t0, msg_failval
    jal  ra, print_str
    lw   a0, 4(s5)
    jal  ra, print_hex
    la   t0, msg_exp
    jal  ra, print_str
    lw   a0, 8(s5)
    jal  ra, print_hex
    j    pr_next

pr_failcause:
    la   t0, msg_failcause
    jal  ra, print_str
    lw   a0, 4(s5)
    jal  ra, print_hex
    la   t0, msg_exp
    jal  ra, print_str
    lw   a0, 8(s5)
    jal  ra, print_hex
    la   t0, msg_mtval
    jal  ra, print_str
    lw   a0, 12(s5)
    jal  ra, print_hex
    j    pr_next

pr_notrap:
    la   t0, msg_notrap
    jal  ra, print_str
    j    pr_next

pr_unexp:
    la   t0, msg_unexp
    jal  ra, print_str
    lw   a0, 4(s5)
    jal  ra, print_hex
    la   t0, msg_mtval
    jal  ra, print_str
    lw   a0, 8(s5)
    jal  ra, print_hex
    la   t0, msg_mepc
    jal  ra, print_str
    lw   a0, 12(s5)
    jal  ra, print_hex
    j    pr_next

pr_next:
    la   t0, msg_nl
    jal  ra, print_str
    addi s4, s4, 1
    j    pr_loop

pr_done:
    la   t0, pr_saved_ra
    lw   ra, 0(t0)
    ret

print_str:
    li   t1, STDOUT
1:  lbu  t2, 0(t0)
    beqz t2, 2f
    sb   t2, 0(t1)
    addi t0, t0, 1
    j    1b
2:  ret

print_dec:
    li   t1, STDOUT
    li   t2, 10
    blt  a0, t2, 1f
    div  t3, a0, t2
    rem  a0, a0, t2
    addi t3, t3, '0'
    sb   t3, 0(t1)
1:  addi a0, a0, '0'
    sb   a0, 0(t1)
    ret

print_hex:
    li   t1, STDOUT
    li   t2, '0'
    sb   t2, 0(t1)
    li   t2, 'x'
    sb   t2, 0(t1)
    li   t4, 28
1:  srl  t2, a0, t4
    andi t2, t2, 0xF
    li   t3, 10
    blt  t2, t3, 2f
    addi t2, t2, 'a'-10
    j    3f
2:  addi t2, t2, '0'
3:  sb   t2, 0(t1)
    addi t4, t4, -4
    bgez t4, 1b
    ret

// ---------------------------------------------------------------------------
// Trap vector: records unexpected traps for this non-faulting test.
// ---------------------------------------------------------------------------
.align 6
trap_entry:
    csrr t0, mcause
    csrr t1, mtval
    csrr t2, mepc

    get_tid t5
    bnez t5, .Lte_u_h1
    la   t6, h0_records
    la   t4, h0_cur_id
    j    .Lte_u_w
.Lte_u_h1:
    la   t6, h1_records
    la   t4, h1_cur_id
.Lte_u_w:
    lw   t4, 0(t4)
    slli t4, t4, REC_SHIFT
    add  t6, t6, t4
    li   t5, ST_UNEXP
    sw   t5, 0(t6)
    sw   t0, 4(t6)              // cause
    sw   t1, 8(t6)              // mtval
    sw   t2, 12(t6)             // mepc
    addi t2, t2, 4
    csrw mepc, t2
    mret

// ---------------------------------------------------------------------------
// Data section
// ---------------------------------------------------------------------------
.section .data

.align 2
zero_region_start:
total_pass:       .word 0
total_fail:       .word 0
h1_done_flag:     .word 0
h1_start_flag:    .word 0
h0_cur_id:        .word 0
h1_cur_id:        .word 0
h0_records:       .fill 4, 4, 0
h1_records:       .fill H1_NUM_CP*4, 4, 0
zero_region_end:

.align 2
h1_saved_ra:      .word 0
pr_saved_ra:      .word 0

// Guard region
.align 4
guard_b_data:     .fill 64, 1, 0

// Checkpoint name pointer table
.align 2
h1_names:
    .word s_h1_00

// Checkpoint name string
s_h1_00: .asciz "h1.chained_tor_rw"

// Report strings
msg_banner:     .asciz "\nVeeR-EH2 PMP hart1 TOR test (verbose)\n"
msg_h1hdr:      .asciz "[hart1]\n"
msg_colon:      .asciz ": "
msg_pass_w:     .asciz "PASS"
msg_failval:    .asciz "FAIL value got="
msg_failcause:  .asciz "FAIL cause got="
msg_exp:        .asciz " exp="
msg_mtval:      .asciz " mtval="
msg_mepc:       .asciz " mepc="
msg_notrap:     .asciz "FAIL no-trap (expected fault did not occur)"
msg_unexp:      .asciz "FAIL unexpected-trap cause="
msg_notrun:     .asciz "NOT-RUN"
msg_total:      .asciz "\nTOTAL pass="
msg_total_fail: .asciz " fail="
msg_nl:         .asciz "\n"
