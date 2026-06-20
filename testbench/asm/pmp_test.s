// SPDX-License-Identifier: Apache-2.0
// Deep PMP integration test for VeeR-EH2 (2-hart capable)
//
// Exercises: CSR read/write, TOR, NA4, NAPOT, L-bit bypass (M-mode),
// locked R/W/X faults, fetch/load/store faults, multi-byte span,
// lock/TOR-lock WARL behavior, entry priority, dual-hart isolation.

#include "defines.h"

#define STDOUT          RV_SERIALIO
#define PMPCFG0         0x3A0
#define PMPCFG1         0x3A1
#define PMPCFG2         0x3A2
#define PMPCFG3         0x3A3
#define PMPADDR0        0x3B0

#define A_OFF           0
#define A_TOR           1
#define A_NA4           2
#define A_NAPOT         3

// Protected data windows (outside main .text at 0x0)
#define GUARD_A_BASE    0x00030000
#define GUARD_A_END     0x00030100
#define GUARD_B_BASE    0x00040000
#define GUARD_B_END     0x00040100
#define SPAN_BASE       0x00050000
#define SPAN_END        0x00050008   // 8-byte NAPOT window

#define FETCH_ISLAND    0x00004000

#define RESULTS_BASE    (RV_DCCM_EADR - 512)
#define H0_PASS         (RESULTS_BASE + 0)
#define H0_FAIL         (RESULTS_BASE + 4)
#define H1_PASS         (RESULTS_BASE + 8)
#define H1_FAIL         (RESULTS_BASE + 12)
#define TRAP_CAUSE      (RESULTS_BASE + 16)
#define TRAP_MTVAL      (RESULTS_BASE + 20)
#define TRAP_EPC        (RESULTS_BASE + 24)
#define EXPECT_TRAP     (RESULTS_BASE + 28)
#define EXPECT_CAUSE    (RESULTS_BASE + 32)
#define LOCK_SEMAPHORE  (RV_DCCM_EADR - 8)
#define H1_DONE         (RV_DCCM_EADR - 4)
#define H1_START        (RV_DCCM_EADR - 12)

.macro get_tid reg=a0
    csrr \reg, mhartid
    andi \reg, \reg, 0xf
.endm

.macro fork targ, reg=a0
    get_tid \reg
    bnez \reg, \targ
.endm

.macro cfg_byte l, a, x, w, r
    ((\l << 7) | (\a << 3) | (\x << 2) | (\w << 1) | (\r))
.endm

.macro inc_pass
    la   t6, h_pass_ptr
    lw   t5, 0(t6)
    addi t5, t5, 1
    sw   t5, 0(t6)
.endm

.macro inc_fail
    la   t6, h_fail_ptr
    lw   t5, 0(t6)
    addi t5, t5, 1
    sw   t5, 0(t6)
.endm

// After an expected-trap instruction: fail if EXPECT_TRAP is still set.
.macro check_trapped
    la   t6, EXPECT_TRAP
    lw   t5, 0(t6)
    bnez t5, 1f
    j    2f
1:  inc_fail
2:  disarm_trap
.endm

// branch to 1f (fail label) if any reg != zero
.macro assert_zero reg, faillbl=1f
    bnez \reg, \faillbl
.endm

.macro assert_eq got, want, faillbl=1f
    bne  \got, \want, \faillbl
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
    li   t0, PMPADDR0
    li   t1, 16
1:  csrw t0, zero
    addi t0, t0, 1
    addi t1, t1, -1
    bnez t1, 1b
    pmp_sync
.endm

.macro arm_trap expect_cause
    li   t0, 1
    la   t1, EXPECT_TRAP
    sw   t0, 0(t1)
    li   t0, \expect_cause
    la   t1, EXPECT_CAUSE
    sw   t0, 0(t1)
.endm

.macro disarm_trap
    la   t1, EXPECT_TRAP
    sw   zero, 0(t1)
.endm

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

    fork hart1_main

    // --- Hart 0 setup ---
    la   t0, h0_pass_ptr
    la   t1, H0_PASS
    sw   t1, 0(t0)
    la   t0, h0_fail_ptr
    la   t1, H0_FAIL
    sw   t1, 0(t0)

    la   t0, H0_PASS
    sw   zero, 0(t0)
    la   t0, H0_FAIL
    sw   zero, 0(t0)
    la   t0, H1_DONE
    sw   zero, 0(t0)
    la   t0, H1_START
    sw   zero, 0(t0)

    li   t0, LOCK_SEMAPHORE
    sw   zero, 0(t0)

    csrwi 0x7fc, 3        // mhartstart: enable hart1
    li   t0, 1
    la   t1, H1_START
    sw   t0, 0(t1)

    jal  ra, h0_run_tests

    // Wait for hart1
1:  la   t0, H1_DONE
    lw   t1, 0(t0)
    beqz t1, 1b

    jal  ra, h0_report_and_finish
    j    spin_forever

hart1_main:
    la   t0, h1_pass_ptr
    la   t1, H1_PASS
    sw   t1, 0(t0)
    la   t0, h1_fail_ptr
    la   t1, H1_FAIL
    sw   t1, 0(t0)
    la   t0, H1_PASS
    sw   zero, 0(t0)
    la   t0, H1_FAIL
    sw   zero, 0(t0)

1:  la   t0, H1_START
    lw   t1, 0(t0)
    beqz t1, 1b

    jal  ra, h1_run_tests

    li   t0, 1
    la   t1, H1_DONE
    sw   t0, 0(t1)
    j    spin_forever

spin_forever:
    wfi
    j spin_forever

// ---------------------------------------------------------------------------
// Hart 0 test groups
// ---------------------------------------------------------------------------
h0_run_tests:
    jal  ra, t_csr_basic
    jal  ra, t_l0_mmode_bypass
    jal  ra, t_locked_tor_load_store
    jal  ra, t_locked_fetch_fault
    jal  ra, t_napot_span_fault
    jal  ra, t_lock_and_tor_lock
    jal  ra, t_priority_entry0_wins
    ret

// ---------------------------------------------------------------------------
// Hart 1 test groups (different physical window)
// ---------------------------------------------------------------------------
h1_run_tests:
    jal  ra, t_h1_na4_and_napot
    jal  ra, t_h1_chained_tor
    ret

// --- CSR smoke: write/read pmpaddr3 and packed pmpcfg0 ---
t_csr_basic:
    clear_all_pmp
    li   t0, (GUARD_A_END >> 2)
    li   t1, PMPADDR0 + 3
    csrw t1, t0
    csrr t2, t1
    bne  t0, t2, 1f
    inc_pass
    j    2f
1:  inc_fail
2:  ret

// --- L=0 in M-mode: R=0/W=0/X=0 still allows access ---
t_l0_mmode_bypass:
    clear_all_pmp
    li   t0, cfg_byte(0, A_TOR, 0, 0, 0)
    csrw PMPCFG0, t0
    li   t0, (GUARD_A_END >> 2)
    csrw PMPADDR0, t0
    la   t0, guard_a_data
    lw   t1, 0(t0)
    sw   t1, 0(t0)
    inc_pass
    ret

// --- L=1 TOR [GUARD_A_BASE, GUARD_A_END): R=1 W=0 X=0 ---
t_locked_tor_load_store:
    clear_all_pmp
    li   t0, (GUARD_A_BASE >> 2)
    csrw PMPADDR0, t0
    li   t0, (cfg_byte(1, A_TOR, 0, 0, 1) << 8)   // entry1 in pmpcfg0[15:8]
    csrw PMPCFG0, t0
    li   t0, (GUARD_A_END >> 2)
    li   t1, PMPADDR0 + 1
    csrw t1, t0
    pmp_sync

    la   t0, guard_a_data
    lw   t1, 0(t0)
    bnez t1, 1f
    inc_pass
    j    2f
1:  inc_fail
2:
    arm_trap 7
    li   t1, 0xdeadbeef
    sw   t1, 0(t0)
    check_trapped
    ret

// --- L=1 NA4 on fetch island with X=0 ---
t_locked_fetch_fault:
    clear_all_pmp
    li   t0, cfg_byte(1, A_NA4, 0, 0, 1)
    csrw PMPCFG0, t0
    li   t0, (FETCH_ISLAND >> 2)
    csrw PMPADDR0, t0
    pmp_sync

    arm_trap 1
    li   t0, FETCH_ISLAND
    jalr ra, t0
    check_trapped
    ret

// --- 8B NAPOT: word spanning outside region must fault ---
t_napot_span_fault:
    clear_all_pmp
    li   t0, cfg_byte(1, A_NAPOT, 0, 0, 1)
    csrw PMPCFG0, t0
    li   t0, (SPAN_BASE >> 2)   // 8B NAPOT encoding (base aligned, size=8)
    csrw PMPADDR0, t0
    pmp_sync

    li   t0, SPAN_BASE + 4
    arm_trap 5
    lw   t1, 0(t0)
    check_trapped

    li   t0, SPAN_BASE
    lw   t1, 0(t0)
    inc_pass
    ret

// --- Lock bit blocks pmpcfg rewrite; TOR-lock blocks pmpaddr[i-1] ---
t_lock_and_tor_lock:
    clear_all_pmp
    li   t0, (GUARD_A_BASE >> 2)
    csrw PMPADDR0, t0
    li   t0, (cfg_byte(1, A_TOR, 0, 0, 1) << 8)
    csrw PMPCFG0, t0
    li   t0, (GUARD_A_END >> 2)
    li   t1, PMPADDR0 + 1
    csrw t1, t0
    pmp_sync

    li   t0, 0x00000001
    li   t1, PMPADDR0
    csrw t1, t0
    csrr t2, t1
    li   t3, (GUARD_A_BASE >> 2)
    bne  t2, t3, 1f
    inc_pass
    j    2f
1:  inc_fail
2:
    li   t0, 0
    csrw PMPCFG0, t0
    csrr t2, PMPCFG0
    srli t2, t2, 8
    andi t2, t2, 0xff
    li   t3, cfg_byte(1, A_TOR, 0, 0, 1)
    bne  t2, t3, 3f
    inc_pass
    j    4f
3:  inc_fail
4:  ret

// --- Entry0 NA4 R-only beats entry1 TOR RWX on overlap ---
t_priority_entry0_wins:
    clear_all_pmp
    la   t0, guard_a_data
    srli t4, t0, 2

    li   t0, cfg_byte(1, A_NA4, 0, 0, 1)
    li   t1, (cfg_byte(1, A_TOR, 1, 1, 1) << 8)
    or   t0, t0, t1
    csrw PMPCFG0, t0
    csrw PMPADDR0, t4
    li   t0, (GUARD_A_END >> 2)
    li   t1, PMPADDR0 + 1
    csrw t1, t0
    pmp_sync

    la   t0, guard_a_data
    lw   t1, 0(t0)
    inc_pass

    arm_trap 7
    li   t1, 0x55
    sw   t1, 0(t0)
    check_trapped
    ret

// ---------------------------------------------------------------------------
// Hart1: NA4 + NAPOT on GUARD_B
// ---------------------------------------------------------------------------
t_h1_na4_and_napot:
    clear_all_pmp
    li   t0, cfg_byte(1, A_NA4, 0, 1, 1)
    csrw PMPCFG0, t0
    li   t0, (GUARD_B_BASE >> 2)
    csrw PMPADDR0, t0
    pmp_sync

    la   t0, guard_b_data
    li   t1, 0x12345678
    sw   t1, 0(t0)
    lw   t2, 0(t0)
    bne  t1, t2, 1f
    inc_pass
    j    2f
1:  inc_fail
2:
    li   t0, cfg_byte(1, A_NAPOT, 0, 0, 1)
    csrw PMPCFG0, t0
    li   t0, ((GUARD_B_BASE + 0x80) >> 2)   // 128B napot: set low bits
    ori  t0, t0, 0x1F
    csrw PMPADDR0, t0
    pmp_sync

    la   t0, guard_b_data
    addi t0, t0, 0x80
    lw   t1, 0(t0)
    inc_pass
    ret

// --- Hart1 chained TOR ---
t_h1_chained_tor:
    clear_all_pmp
    li   t0, (GUARD_B_BASE >> 2)
    csrw PMPADDR0, t0
    li   t0, (cfg_byte(1, A_TOR, 0, 1, 1) << 8)
    csrw PMPCFG0, t0
    li   t0, (GUARD_B_END >> 2)
    li   t1, PMPADDR0 + 1
    csrw t1, t0
    pmp_sync

    la   t0, guard_b_data
    li   t1, 0xa5a5a5a5
    sw   t1, 0(t0)
    lw   t2, 0(t0)
    bne  t1, t2, 1f
    inc_pass
    j    2f
1:  inc_fail
2:  ret

// ---------------------------------------------------------------------------
// Report + terminate (hart0 only)
// ---------------------------------------------------------------------------
h0_report_and_finish:
    la   t0, H0_PASS
    lw   t1, 0(t0)
    la   t0, H0_FAIL
    lw   t2, 0(t0)
    la   t0, H1_PASS
    lw   t3, 0(t0)
    la   t0, H1_FAIL
    lw   t4, 0(t0)

    add  t1, t1, t3
    add  t2, t2, t4

    la   t0, msg_banner
    jal  ra, print_str

    la   t0, msg_pass
    jal  ra, print_str
    mv   a0, t1
    jal  ra, print_dec

    la   t0, msg_fail
    jal  ra, print_str
    mv   a0, t2
    jal  ra, print_dec

    bnez t2, test_fail
    li   t0, STDOUT
    li   t1, 0xff
    sb   t1, 0(t0)
1:  j 1b

test_fail:
    li   t0, STDOUT
    li   t1, 0x01
    sb   t1, 0(t0)
2:  j 2b

print_str:
    li   t1, STDOUT
1:  lbu  t2, 0(t0)
    beqz t2, 2f
    sb   t2, 0(t1)
    addi t0, t0, 1
    j    1b
2:  ret

print_dec:
    li   t1, 10
    beqz a0, show0
    mv   t2, a0
    mv   t3, zero
1:  beqz t2, 2f
    rem  t4, t2, t1
    addi t4, t4, '0'
    addi sp, sp, -4
    sw   t4, 0(sp)
    addi t3, t3, 1
    div  t2, t2, t1
    j    1b
2:  beqz t3, 3f
    lw   t4, 0(sp)
    addi sp, sp, 4
    li   t5, STDOUT
    sb   t4, 0(t5)
    addi t3, t3, -1
    j    2b
3:  ret
show0:
    li   t4, '0'
    li   t5, STDOUT
    sb   t4, 0(t5)
    ret

// ---------------------------------------------------------------------------
// Trap vector
// ---------------------------------------------------------------------------
.align 6
trap_entry:
    csrr t0, mcause
    csrr t1, mtval
    csrr t2, mepc

    la   t3, TRAP_CAUSE
    sw   t0, 0(t3)
    la   t3, TRAP_MTVAL
    sw   t1, 0(t3)
    la   t3, TRAP_EPC
    sw   t2, 0(t3)

    la   t3, EXPECT_TRAP
    lw   t4, 0(t3)
    beqz t4, trap_unexpected

    sw   zero, 0(t3)
    la   t3, EXPECT_CAUSE
    lw   t4, 0(t3)
    bne  t0, t4, trap_unexpected

    get_tid t5
    bnez t5, trap_h1_pass
    la   t6, h0_pass_ptr
    j    trap_inc
trap_h1_pass:
    la   t6, h1_pass_ptr
trap_inc:
    lw   t4, 0(t6)
    addi t4, t4, 1
    sw   t4, 0(t6)

    addi t2, t2, 4
    csrw mepc, t2
    mret

trap_unexpected:
    get_tid t5
    bnez t5, trap_h1_fail
    la   t6, h0_fail_ptr
    j    trap_inc_fail
trap_h1_fail:
    la   t6, h1_fail_ptr
trap_inc_fail:
    lw   t4, 0(t6)
    addi t4, t4, 1
    sw   t4, 0(t6)
    j    spin_forever

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------
.org FETCH_ISLAND
fetch_island:
    addi a0, a0, 1
    ret

.section .data
.align 4
h0_pass_ptr:
    .word 0
h0_fail_ptr:
    .word 0
h1_pass_ptr:
    .word 0
h1_fail_ptr:
    .word 0

guard_a_data:
    .space 64
guard_b_data:
    .space 256

msg_banner:
    .ascii "VeeR-EH2 PMP deep test\n"
    .byte 0
msg_pass:
    .ascii "  pass="
    .byte 0
msg_fail:
    .ascii " fail="
    .byte 0
