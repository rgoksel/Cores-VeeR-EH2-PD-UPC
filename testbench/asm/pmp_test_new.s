// SPDX-License-Identifier: Apache-2.0
// PMP integration test for VeeR-EH2 (2-hart capable) - VERBOSE / SELF-DIAGNOSING
//
// Same coverage as the original (CSR R/W, TOR, NA4, NAPOT, L-bit bypass,
// locked R/W/X faults, fetch fault, lock/TOR-lock WARL, entry priority,
// A=OFF disabled, dual-hart isolation) but instead of only printing aggregate
// pass/fail counts it records a per-checkpoint result table and prints, for
// each named checkpoint:
//      <name>: PASS
//      <name>: FAIL value got=0x.... exp=0x....
//      <name>: FAIL cause got=0x.... exp=0x.... mtval=0x....
//      <name>: FAIL no-trap (expected fault did not occur)
//      <name>: FAIL unexpected-trap cause=0x.... mtval=0x.... mepc=0x....
//      <name>: NOT-RUN
//
// Only hart0 prints, and only after hart1 has finished (h1_done_flag), so the
// two harts never interleave characters in the report.  The single-char debug
// markers (#,H,1..8,a..c,E) still stream live and may interleave - that is fine.
//
// NOTE: a real bug in the original test was fixed here - see "prio.store" below.

#include "defines.h"

#define STDOUT          RV_SERIALIO

// PMP CSR addresses (used as immediate literals - CPP expands before GAS sees them)
#define PMPCFG0         0x3A0
#define PMPCFG1         0x3A1
#define PMPCFG2         0x3A2
#define PMPCFG3         0x3A3
#define PMPADDR0        0x3B0
#define PMPADDR1        0x3B1
#define PMPADDR2        0x3B2
#define PMPADDR3        0x3B3

// A-field encoding
#define A_OFF           0
#define A_TOR           1
#define A_NA4           2
#define A_NAPOT         3

#define cfg_byte(l,a,x,w,r)  (((l)<<7)|((a)<<3)|((x)<<2)|((w)<<1)|(r))

// NAPOT size encoding helpers
#define NAPOT_8B        0x0
#define NAPOT_16B       0x1
#define NAPOT_32B       0x3
#define NAPOT_128B      0xF
#define NAPOT_256B      0x1F

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

#define H0_NUM_CP       16
#define H1_NUM_CP       5

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

.macro arm_trap expect_cause
    get_tid t1
    slli t1, t1, 2
    la   t0, expect_trap_flag
    add  t1, t1, t0
    li   t0, 1
    sw   t0, 0(t1)

    get_tid t1
    slli t1, t1, 2
    la   t0, expect_cause_val
    add  t1, t1, t0
    li   t0, \expect_cause
    sw   t0, 0(t1)
.endm

.macro disarm_trap
    get_tid t1
    slli t1, t1, 2
    la   t0, expect_trap_flag
    add  t1, t1, t0
    sw   zero, 0(t1)
.endm

.macro dbg_char c
    li   t0, STDOUT
    li   t1, \c
    sb   t1, 0(t0)
.endm

// ---------------------------------------------------------------------------
// Result-recording macros (hart-agnostic: they pick the current hart's table
// and current-checkpoint id at run time).
// ---------------------------------------------------------------------------

// Select the current checkpoint id for the running hart. Clobbers t0,t1.
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

// Put &record (for current hart + current cp id) into t6. Clobbers t5,t6.
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

// Record PASS, but never overwrite an already-set status (so an unexpected
// trap recorded by the handler at this cp survives). Clobbers t5,t6.
.macro rec_pass
    rec_addr_t6
    lw   t5, 0(t6)
    bnez t5, .Lrp_skip_\@
    li   t5, ST_PASS
    sw   t5, 0(t6)
.Lrp_skip_\@:
.endm

// Record a value mismatch. got=\g exp=\e (neither may be t4/t5/t6).
// Clobbers t3,t4,t5,t6.
.macro rec_failval g, e
    mv   t4, \e
    mv   t3, \g
    rec_addr_t6
    li   t5, ST_FAILVAL
    sw   t5, 0(t6)
    sw   t3, 4(t6)
    sw   t4, 8(t6)
.endm

// After a faulting instruction: if expect_trap_flag is still set the trap did
// NOT happen -> record no-trap fail. Otherwise the handler already recorded.
.macro check_trapped
    get_tid t6
    slli t6, t6, 2
    la   t5, expect_trap_flag
    add  t6, t6, t5
    lw   t5, 0(t6)
    beqz t5, .Lct_skip_\@
    rec_addr_t6
    li   t5, ST_NOTRAP
    sw   t5, 0(t6)
.Lct_skip_\@:
    disarm_trap
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

    // -----------------------------------------------------------------------
    // Hart 0 initialisation
    // -----------------------------------------------------------------------
    // Zero the entire result/flag region (also zeroes both record tables and
    // both cur_id words) before releasing hart1.
    la   t0, zero_region_start
    la   t1, zero_region_end
1:  sw   zero, 0(t0)
    addi t0, t0, 4
    bltu t0, t1, 1b

    csrwi 0x7fc, 3              // mhartstart: enable hart1
    li   t0, 1
    la   t1, h1_start_flag
    sw   t0, 0(t1)              // release hart1

    jal  ra, h0_run_tests

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
// Hart 0 / Hart 1 test suites (non-leaf: save/restore ra around the jal chain)
// ---------------------------------------------------------------------------
h0_run_tests:
    la   t0, h0_saved_ra
    sw   ra, 0(t0)
    jal  ra, t_csr_basic
    jal  ra, t_l0_mmode_bypass
    jal  ra, t_locked_tor_load_store
    jal  ra, t_locked_fetch_fault
    jal  ra, t_napot_rw_fault
    jal  ra, t_aoff_disabled
    jal  ra, t_priority_entry0_wins
    jal  ra, t_locked_read_deny     // leaves entry4 locked over guard_c (in pmpcfg1)
    jal  ra, t_lock_and_tor_lock    // MUST be last: leaves locked entries
    la   t0, h0_saved_ra
    lw   ra, 0(t0)
    ret

h1_run_tests:
    la   t0, h1_saved_ra
    sw   ra, 0(t0)
    jal  ra, t_h1_na4_rw
    jal  ra, t_h1_napot_16b
    jal  ra, t_h1_chained_tor
    jal  ra, t_h1_read_deny
    la   t0, h1_saved_ra
    lw   ra, 0(t0)
    ret

// ---------------------------------------------------------------------------
// Test 1 - CSR read/write roundtrip       (cp0 = csr.pmpaddr3, cp1 = csr.pmpcfg0)
// ---------------------------------------------------------------------------
t_csr_basic:
    dbg_char '1'
    clear_all_pmp

    set_cp 0
    li   t0, 0x12345
    csrw PMPADDR3, t0
    csrr t2, PMPADDR3
    beq  t0, t2, 1f
    rec_failval t2, t0
    j    2f
1:  rec_pass
2:
    set_cp 1
    li   t0, cfg_byte(0, A_TOR, 0, 0, 1)
    csrw PMPCFG0, t0
    csrr t2, PMPCFG0
    andi t2, t2, 0xFF
    beq  t0, t2, 3f
    rec_failval t2, t0
    j    4f
3:  rec_pass
4:
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 2 - L=0 M-mode bypass               (cp2 = l0_mmode_bypass)
// ---------------------------------------------------------------------------
t_l0_mmode_bypass:
    dbg_char '2'
    clear_all_pmp

    la   t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR0, t0
    li   t0, cfg_byte(0, A_TOR, 0, 0, 0)
    csrw PMPCFG0, t0
    pmp_sync

    set_cp 2
    la   t0, guard_a_data
    lw   t1, 0(t0)              // M-mode, L=0 -> must succeed
    sw   t1, 0(t0)             // M-mode, L=0 -> must succeed
    rec_pass                    // (handler records unexpected-trap if either faults)

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 3 - L=1 TOR R=1 W=0    (cp3 = ltor.load, cp4 = ltor.store_fault c=7)
// ---------------------------------------------------------------------------
t_locked_tor_load_store:
    dbg_char '3'
    clear_all_pmp

    la   t0, guard_a_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    la   t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR1, t0
    li   t0, (cfg_byte(1, A_TOR, 0, 0, 1) << 8)
    csrw PMPCFG0, t0
    pmp_sync

    set_cp 3
    la   t0, guard_a_data
    lw   t1, 0(t0)              // R=1 -> succeed
    rec_pass

    set_cp 4
    arm_trap 7
    la   t0, guard_a_data
    li   t1, 0xdeadbeef
    sw   t1, 0(t0)              // W=0 L=1 -> fault (cause 7)
    check_trapped

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 4 - L=1 NA4 X=0 fetch fault         (cp5 = fetch_fault.X0 c=1)
// ---------------------------------------------------------------------------
t_locked_fetch_fault:
    dbg_char '4'

    la   t0, fetch4_saved_ra
    sw   ra, 0(t0)

    clear_all_pmp

    li   t0, cfg_byte(1, A_NA4, 0, 0, 1)
    csrw PMPCFG0, t0
    la   t0, fetch_island
    srli t0, t0, 2
    csrw PMPADDR0, t0
    pmp_sync

    set_cp 5
    arm_trap 1                  // expect instruction access fault
    la   t0, fetch_island
    jalr ra, t0, 0
    check_trapped

    la   t0, fetch4_saved_ra
    lw   ra, 0(t0)
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 5 - L=1 NAPOT(8B) R=1 W=0  (cp6 = napot8.load, cp7 = napot8.store_fault c=7)
// ---------------------------------------------------------------------------
t_napot_rw_fault:
    dbg_char '5'
    clear_all_pmp

    li   t0, cfg_byte(1, A_NAPOT, 0, 0, 1)
    csrw PMPCFG0, t0
    la   t0, guard_a_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    pmp_sync

    set_cp 6
    la   t0, guard_a_data
    lw   t1, 0(t0)              // R=1 -> succeed
    rec_pass

    set_cp 7
    arm_trap 7
    la   t0, guard_a_data
    li   t1, 0xA5A5A5A5
    sw   t1, 0(t0)              // W=0 L=1 -> fault
    check_trapped

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 6 - A=OFF disabled even when L=1     (cp8 = aoff_disabled)
// ---------------------------------------------------------------------------
t_aoff_disabled:
    dbg_char '6'
    clear_all_pmp

    li   t0, cfg_byte(1, A_OFF, 0, 0, 0)
    csrw PMPCFG0, t0
    la   t0, guard_a_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    pmp_sync

    set_cp 8
    la   t0, guard_a_data
    li   t1, 0x12345678
    sw   t1, 0(t0)
    lw   t2, 0(t0)
    beq  t1, t2, 1f
    rec_failval t2, t1
    j    2f
1:  rec_pass
2:
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 7 - Priority: entry0 (NA4 R-only) beats entry1 (TOR RWX)
//   cp9 = prio.load_entry0, cp10 = prio.store_fault_entry0 (c=7)
//
//   *** FIX vs original ***: the original did NOT reload t0 with guard_a_data
//   after arm_trap (arm_trap clobbers t0, leaving t0 = the expected-cause value
//   7).  The store then targeted address 0x7 instead of guard_a_data, so the
//   PMP region was never exercised - a guaranteed false failure.  Reloading t0
//   below makes this checkpoint actually test entry-0 priority.
// ---------------------------------------------------------------------------
t_priority_entry0_wins:
    dbg_char '7'
    clear_all_pmp

    la   t3, guard_a_data
    srli t3, t3, 2

    li   t0, cfg_byte(1, A_NA4, 0, 0, 1)
    li   t1, (cfg_byte(1, A_TOR, 1, 1, 1) << 8)
    or   t0, t0, t1
    csrw PMPCFG0, t0

    csrw PMPADDR0, t3
    la   t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR1, t0
    pmp_sync

    set_cp 9
    la   t0, guard_a_data
    lw   t1, 0(t0)              // entry0 R=1 -> succeed
    rec_pass

    set_cp 10
    arm_trap 7
    la   t0, guard_a_data       // FIX: arm_trap clobbered t0; reload base
    li   t1, 0x55
    sw   t1, 0(t0)              // entry0 W=0 L=1 -> fault
    check_trapped

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 8 - Lock / WARL (MUST be last; leaves locked entries)
//   cp11 = lock.cfg_warl, cp12 = lock.pmpaddr0_warl,
//   cp13 = lock.pmpaddr1_warl, cp14 = lock.survives_clear
// ---------------------------------------------------------------------------
t_lock_and_tor_lock:
    dbg_char '8'
    clear_all_pmp

    la   t0, guard_a_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    la   t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR1, t0
    li   t0, (cfg_byte(1, A_TOR, 0, 0, 1) << 8)
    csrw PMPCFG0, t0
    pmp_sync

    set_cp 11
    csrw PMPCFG0, zero
    csrr t2, PMPCFG0
    // entry1 = the TOR byte this test programmed; entry0 = the locked NA4 left
    // by test 7, which a write-zero cannot clear.  Both bytes persist (L=1), so
    // the read-back is 0x8991, not 0x8900.  This is the lock working correctly.
    li   t3, ((cfg_byte(1, A_TOR, 0, 0, 1) << 8) | cfg_byte(1, A_NA4, 0, 0, 1))
    beq  t2, t3, 1f
    rec_failval t2, t3
    j    2f
1:  rec_pass
2:
    set_cp 12
    la   t3, guard_a_data
    srli t3, t3, 2
    li   t0, 0xFFFFF
    csrw PMPADDR0, t0
    csrr t2, PMPADDR0
    beq  t2, t3, 3f
    rec_failval t2, t3
    j    4f
3:  rec_pass
4:
    set_cp 13
    la   t3, guard_a_data
    addi t3, t3, 64
    srli t3, t3, 2
    csrw PMPADDR1, zero
    csrr t2, PMPADDR1
    beq  t2, t3, 5f
    rec_failval t2, t3
    j    6f
5:  rec_pass
6:
    set_cp 14
    clear_all_pmp
    csrr t2, PMPCFG0
    // same surviving pair as cp11: locked TOR byte1 + locked NA4 entry0 (0x8991)
    li   t3, ((cfg_byte(1, A_TOR, 0, 0, 1) << 8) | cfg_byte(1, A_NA4, 0, 0, 1))
    beq  t2, t3, 7f
    rec_failval t2, t3
    j    8f
7:  rec_pass
8:
    ret

// ---------------------------------------------------------------------------
// Test 9 - locked region with R=0: M-mode LOAD must fault (cp15 = lock.load_deny_r0)
//
//   Coverage gap closer: every other locked region in this suite has R=1, so
//   read *denial* was never exercised.  This checks whether a locked R=0 region
//   denies a load the same way the W=0 regions should deny a store.
//
//   Uses PMP entry4 (pmpcfg1 byte0 / pmpaddr4) over a private guard_c region so
//   it (a) does not disturb the guard_a locks left by test 7, and (b) never
//   appears in the pmpcfg0 reads that test 8's checkpoints inspect.  The locked
//   entry4 persists afterwards but is harmless (lives in pmpcfg1, covers only
//   guard_c, which nothing else touches).
// ---------------------------------------------------------------------------
t_locked_read_deny:
    dbg_char '9'
    clear_all_pmp                  // entry0/1 (guard_a) locked -> survive; entry4 is clean

    la   t0, guard_c_data
    srli t0, t0, 2
    csrw 0x3B4, t0                 // pmpaddr4 = guard_c >> 2 (NAPOT 8B, 8-byte aligned)
    li   t0, cfg_byte(1, A_NAPOT, 0, 0, 0)   // L=1 A=NAPOT R=0 W=0 X=0
    csrw PMPCFG1, t0               // entry4 cfg = byte0 of pmpcfg1
    pmp_sync

    set_cp 15
    arm_trap 5                     // expect load access fault (cause 5)
    la   t0, guard_c_data          // reload base (arm_trap clobbers t0)
    lw   t1, 0(t0)                 // R=0 L=1 -> must fault
    check_trapped

    ret                            // leave entry4 locked (harmless)

// ---------------------------------------------------------------------------
// Hart 1 Test A - NA4 R=1 W=1                (h1 cp0 = h1.na4_rw)
// ---------------------------------------------------------------------------
t_h1_na4_rw:
    dbg_char 'a'
    clear_all_pmp

    la   t0, guard_b_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    li   t0, cfg_byte(1, A_NA4, 0, 1, 1)
    csrw PMPCFG0, t0
    pmp_sync

    set_cp 0
    la   t0, guard_b_data
    li   t1, 0x12345678
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
// Hart 1 Test B - NAPOT(16B) R=1 W=0   (h1 cp1 = h1.napot16.load,
//                                       h1 cp2 = h1.napot16.store_fault c=7)
// ---------------------------------------------------------------------------
t_h1_napot_16b:
    dbg_char 'b'
    clear_all_pmp

    li   t0, cfg_byte(1, A_NAPOT, 0, 0, 1)
    csrw PMPCFG0, t0
    la   t0, guard_b_data
    srli t0, t0, 2
    ori  t0, t0, NAPOT_16B
    csrw PMPADDR0, t0
    pmp_sync

    set_cp 1
    la   t0, guard_b_data
    lw   t1, 0(t0)              // R=1 -> succeed
    rec_pass

    set_cp 2
    arm_trap 7
    la   t0, guard_b_data
    li   t1, 0xDEADC0DE
    sw   t1, 0(t0)              // W=0 L=1 -> fault
    check_trapped

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Hart 1 Test C - Chained TOR R=1 W=1       (h1 cp3 = h1.chained_tor_rw)
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

    set_cp 3
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
// Hart 1 Test D - locked R=0 load deny       (h1 cp4 = h1.load_deny_r0)
//   Hart1's PMP is clean here (its previous test cleared), so this uses entry0
//   over the shared guard_c region.  PMP is per-hart, so hart0's entry4 lock on
//   guard_c and this entry0 lock are independent; both only attempt loads (no
//   writes), so the shared backing memory is never modified.
// ---------------------------------------------------------------------------
t_h1_read_deny:
    dbg_char 'd'
    clear_all_pmp

    la   t0, guard_c_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    li   t0, cfg_byte(1, A_NAPOT, 0, 0, 0)   // L=1 A=NAPOT R=0 W=0 X=0
    csrw PMPCFG0, t0
    pmp_sync

    set_cp 4
    arm_trap 5                     // expect load access fault (cause 5)
    la   t0, guard_c_data          // reload base (arm_trap clobbers t0)
    lw   t1, 0(t0)                 // R=0 L=1 -> must fault
    check_trapped

    clear_all_pmp
    ret
// ---------------------------------------------------------------------------
h0_report_and_finish:
    dbg_char 'E'

    la   t0, msg_banner
    jal  ra, print_str

    la   t0, msg_h0hdr
    jal  ra, print_str
    la   a0, h0_records
    la   a1, h0_names
    li   a2, H0_NUM_CP
    jal  ra, print_report

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
// Prints one line per checkpoint and tallies total_pass/total_fail.
// Saves its own ra (calls print_str/dec/hex which clobber ra). Uses s1..s6
// (callee-saved; the print helpers don't touch s-regs so they survive).
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

    // name
    slli t0, s4, 2
    add  t0, s2, t0
    lw   t0, 0(t0)
    jal  ra, print_str
    la   t0, msg_colon
    jal  ra, print_str

    // status
    slli t0, s4, REC_SHIFT
    add  s5, s1, t0             // &record
    lw   s6, 0(s5)             // status

    // tally
    li   t1, ST_PASS
    bne  s6, t1, pr_tally_fail
    la   t1, total_pass
    lw   t2, 0(t1)
    addi t2, t2, 1
    sw   t2, 0(t1)
    j    pr_dispatch
pr_tally_fail:
    beqz s6, pr_dispatch        // status 0 (not-run) counts as neither
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
    la   t0, msg_notrun         // status 0
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

// ---------------------------------------------------------------------------
// print_str: null-terminated string at t0
// ---------------------------------------------------------------------------
print_str:
    li   t1, STDOUT
1:  lbu  t2, 0(t0)
    beqz t2, 2f
    sb   t2, 0(t1)
    addi t0, t0, 1
    j    1b
2:  ret

// ---------------------------------------------------------------------------
// print_dec: a0 (0-99) to STDOUT
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// print_hex: a0 as "0x" + 8 hex digits. Clobbers t1..t5.
// ---------------------------------------------------------------------------
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
// Trap vector (direct mode). Records into the current hart/cp record. Does NOT
// touch ra (no calls), so the interrupted test's ra is preserved.
// ---------------------------------------------------------------------------
.align 6
trap_entry:
    csrr t0, mcause
    csrr t1, mtval
    csrr t2, mepc

    get_tid t3
    slli t3, t3, 2

    la   t4, trap_cause_val
    add  t4, t4, t3
    sw   t0, 0(t4)

    la   t4, trap_mtval_val
    add  t4, t4, t3
    sw   t1, 0(t4)

    la   t4, trap_epc_val
    add  t4, t4, t3
    sw   t2, 0(t4)

    la   t4, expect_trap_flag
    add  t3, t3, t4
    lw   t4, 0(t3)
    beqz t4, trap_unexpected

    sw   zero, 0(t3)            // clear expect flag

    get_tid t3
    slli t3, t3, 2
    la   t4, expect_cause_val
    add  t3, t3, t4
    lw   t4, 0(t3)              // expected cause
    bne  t0, t4, trap_wrong_cause

    // expected & correct -> status PASS
    get_tid t5
    bnez t5, .Lte_p_h1
    la   t6, h0_records
    la   t4, h0_cur_id
    j    .Lte_p_w
.Lte_p_h1:
    la   t6, h1_records
    la   t4, h1_cur_id
.Lte_p_w:
    lw   t4, 0(t4)
    slli t4, t4, REC_SHIFT
    add  t6, t6, t4
    li   t5, ST_PASS
    sw   t5, 0(t6)
    addi t2, t2, 4
    csrw mepc, t2
    mret

trap_wrong_cause:
    mv   t3, t4                 // expected cause
    get_tid t5
    bnez t5, .Lte_wc_h1
    la   t6, h0_records
    la   t4, h0_cur_id
    j    .Lte_wc_w
.Lte_wc_h1:
    la   t6, h1_records
    la   t4, h1_cur_id
.Lte_wc_w:
    lw   t4, 0(t4)
    slli t4, t4, REC_SHIFT
    add  t6, t6, t4
    li   t5, ST_FAILCAUSE
    sw   t5, 0(t6)
    sw   t0, 4(t6)             // got cause
    sw   t3, 8(t6)             // exp cause
    sw   t1, 12(t6)            // mtval
    addi t2, t2, 4
    csrw mepc, t2
    mret

trap_unexpected:
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
    sw   t0, 4(t6)             // cause
    sw   t1, 8(t6)             // mtval
    sw   t2, 12(t6)            // mepc
    addi t2, t2, 4
    csrw mepc, t2
    mret

// ---------------------------------------------------------------------------
// Fetch island (norvc so each instruction is exactly 4 bytes)
// ---------------------------------------------------------------------------
.option push
.option norvc
.align 2
fetch_island:
    addi a0, a0, 1
    jalr x0, 0(ra)
.option pop

// ---------------------------------------------------------------------------
// Data section
// ---------------------------------------------------------------------------
.section .data

// ---- zeroed-at-startup region (flags, cur ids, both record tables) ----
.align 2
zero_region_start:
total_pass:       .word 0
total_fail:       .word 0
h1_done_flag:     .word 0
h1_start_flag:    .word 0
expect_trap_flag: .word 0, 0    // indexed by mhartid
expect_cause_val: .word 0, 0    // indexed by mhartid
h0_cur_id:        .word 0
h1_cur_id:        .word 0
h0_records:       .fill H0_NUM_CP*4, 4, 0
h1_records:       .fill H1_NUM_CP*4, 4, 0
zero_region_end:

// ---- not zeroed by the loop (written before use at run time) ----
.align 2
h0_saved_ra:      .word 0
h1_saved_ra:      .word 0
fetch4_saved_ra:  .word 0
pr_saved_ra:      .word 0
trap_cause_val:   .word 0, 0    // indexed by mhartid
trap_mtval_val:   .word 0, 0    // indexed by mhartid
trap_epc_val:     .word 0, 0    // indexed by mhartid

// Guard regions
.align 3
guard_a_data:     .fill 64, 1, 0
.align 4
guard_b_data:     .fill 64, 1, 0
.align 3
guard_c_data:     .fill 64, 1, 0

// Checkpoint name pointer tables
.align 2
h0_names:
    .word s_h0_00, s_h0_01, s_h0_02, s_h0_03, s_h0_04
    .word s_h0_05, s_h0_06, s_h0_07, s_h0_08, s_h0_09
    .word s_h0_10, s_h0_11, s_h0_12, s_h0_13, s_h0_14
    .word s_h0_15
h1_names:
    .word s_h1_00, s_h1_01, s_h1_02, s_h1_03, s_h1_04

// Checkpoint name strings
s_h0_00: .asciz "csr.pmpaddr3"
s_h0_01: .asciz "csr.pmpcfg0"
s_h0_02: .asciz "l0_mmode_bypass"
s_h0_03: .asciz "ltor.load"
s_h0_04: .asciz "ltor.store_fault"
s_h0_05: .asciz "fetch_fault.X0"
s_h0_06: .asciz "napot8.load"
s_h0_07: .asciz "napot8.store_fault"
s_h0_08: .asciz "aoff_disabled"
s_h0_09: .asciz "prio.load_entry0"
s_h0_10: .asciz "prio.store_fault_entry0"
s_h0_11: .asciz "lock.cfg_warl"
s_h0_12: .asciz "lock.pmpaddr0_warl"
s_h0_13: .asciz "lock.pmpaddr1_warl"
s_h0_14: .asciz "lock.survives_clear"
s_h0_15: .asciz "lock.load_deny_r0"

s_h1_00: .asciz "h1.na4_rw"
s_h1_01: .asciz "h1.napot16.load"
s_h1_02: .asciz "h1.napot16.store_fault"
s_h1_03: .asciz "h1.chained_tor_rw"
s_h1_04: .asciz "h1.load_deny_r0"

// Report strings
msg_banner:     .asciz "\nVeeR-EH2 PMP test (verbose)\n"
msg_h0hdr:      .asciz "[hart0]\n"
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