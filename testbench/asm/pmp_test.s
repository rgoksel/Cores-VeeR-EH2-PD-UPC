// SPDX-License-Identifier: Apache-2.0
// PMP integration test for VeeR-EH2 (2-hart capable)
//
// Tests: CSR read/write, TOR, NA4, NAPOT, L-bit bypass (M-mode),
//        locked R/W/X faults, fetch fault, lock/TOR-lock WARL,
//        entry priority, A=OFF disabled, dual-hart isolation.

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

// cfg_byte MUST be a C-preprocessor #define so CPP evaluates it to a constant
// before GAS sees the li instruction.  A GAS .macro is NOT valid as an li operand.
#define cfg_byte(l,a,x,w,r)  (((l)<<7)|((a)<<3)|((x)<<2)|((w)<<1)|(r))

// NAPOT size encoding helpers: pmpaddr = (base >> 2) | NAPOT_MASK(size)
// Formula: (size/8 - 1) trailing 1s in pmpaddr => region = 2^(k+3) bytes where k=trailing-1s
#define NAPOT_8B        0x0   // 0 trailing 1s => 8B  (base must be 8B-aligned)
#define NAPOT_16B       0x1   // 1 trailing 1  => 16B (base must be 16B-aligned)
#define NAPOT_32B       0x3   // 2 trailing 1s => 32B
#define NAPOT_128B      0xF   // 4 trailing 1s => 128B
#define NAPOT_256B      0x1F  // 5 trailing 1s => 256B

// fetch_island lives in .text (after the last function) with .option norvc so each
// instruction is exactly 4 bytes.  The test loads its address via  la t0, fetch_island
// at runtime, avoiding the linker-relaxation issue that breaks fixed .org offsets.

// ---------------------------------------------------------------------------
// Macros
// ---------------------------------------------------------------------------

// Read hart ID into reg (default a0)
.macro get_tid reg=a0
    csrr \reg, mhartid
    andi \reg, \reg, 0xf
.endm

// Branch hart1 to targ
.macro fork targ, reg=a0
    get_tid \reg
    bnez \reg, \targ
.endm

// 8 nops to let PMP CSR writes propagate through the pipeline
.macro pmp_sync
    .rept 8
    nop
    .endr
.endm

// Clear ALL 16 PMP entries.  Each CSR address must be a literal immediate;
// a register cannot be used as a CSR specifier in RISC-V.
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

// Set expect_trap_flag=1 and expect_cause_val=cause before a fault instruction
.macro arm_trap expect_cause
    li   t0, 1
    la   t1, expect_trap_flag
    sw   t0, 0(t1)
    li   t0, \expect_cause
    la   t1, expect_cause_val
    sw   t0, 0(t1)
.endm

// Clear the expect flag (used after a trap sequence completes or is skipped)
.macro disarm_trap
    la   t1, expect_trap_flag
    sw   zero, 0(t1)
.endm

// After a faulting instruction: if expect_trap_flag is still set the trap
// did NOT happen -> count as failure.
.macro check_trapped
    la   t6, expect_trap_flag
    lw   t5, 0(t6)
    beqz t5, .Lct_skip_\@
    inc_fail
.Lct_skip_\@:
    disarm_trap
.endm

// Hart-aware pass/fail increment.  Uses get_tid to pick the correct counter.
// Clobbers t4, t5, t6.
// \@ gives a unique integer per macro invocation so labels never collide,
// even when inc_pass/inc_fail are expanded next to each other.
.macro inc_pass
    get_tid t5
    bnez t5, .Lincp_h1_\@
    la   t6, h0_pass_count
    j    .Lincp_end_\@
.Lincp_h1_\@:
    la   t6, h1_pass_count
.Lincp_end_\@:
    lw   t4, 0(t6)
    addi t4, t4, 1
    sw   t4, 0(t6)
.endm

.macro inc_fail
    get_tid t5
    bnez t5, .Lincf_h1_\@
    la   t6, h0_fail_count
    j    .Lincf_end_\@
.Lincf_h1_\@:
    la   t6, h1_fail_count
.Lincf_end_\@:
    lw   t4, 0(t6)
    addi t4, t4, 1
    sw   t4, 0(t6)
.endm

// Write a single-char debug marker to STDOUT (visible in console.log)
.macro dbg_char c
    li   t0, STDOUT
    li   t1, \c
    sb   t1, 0(t0)
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

    // Install trap handler
    la   t0, trap_entry
    csrw mtvec, t0

    // Enable caches via MRAC
    li   t0, 0x5f555555
    csrw 0x7c0, t0

    clear_all_pmp

    // Debug: signal simulation start
    dbg_char '#'

    fork hart1_main

    // -----------------------------------------------------------------------
    // Hart 0 initialisation
    // -----------------------------------------------------------------------
    la   t0, h0_pass_count
    sw   zero, 0(t0)
    la   t0, h0_fail_count
    sw   zero, 0(t0)
    la   t0, h1_pass_count
    sw   zero, 0(t0)
    la   t0, h1_fail_count
    sw   zero, 0(t0)
    la   t0, h1_done_flag
    sw   zero, 0(t0)
    la   t0, expect_trap_flag
    sw   zero, 0(t0)

    csrwi 0x7fc, 3          // mhartstart: enable hart1
    li   t0, 1
    la   t1, h1_start_flag
    sw   t0, 0(t1)          // signal hart1 to start

    jal  ra, h0_run_tests

    // Wait for hart1 to finish
1:  la   t0, h1_done_flag
    lw   t1, 0(t0)
    beqz t1, 1b

    jal  ra, h0_report_and_finish
    j    spin_forever

// ---------------------------------------------------------------------------
// Hart 1 entry
// ---------------------------------------------------------------------------
hart1_main:
    la   t0, h1_pass_count
    sw   zero, 0(t0)
    la   t0, h1_fail_count
    sw   zero, 0(t0)

1:  la   t0, h1_start_flag
    lw   t1, 0(t0)
    beqz t1, 1b             // spin until hart0 signals start

    jal  ra, h1_run_tests

    li   t0, 1
    la   t1, h1_done_flag
    sw   t0, 0(t1)
    j    spin_forever

spin_forever:
    j spin_forever

// ---------------------------------------------------------------------------
// Hart 0 test suite
// ---------------------------------------------------------------------------
// h0_run_tests is a non-leaf function: it calls tests with jal ra, which
// overwrites ra.  Save/restore ra around all calls so the final ret works.
// ---------------------------------------------------------------------------
h0_run_tests:
    la   t0, h0_saved_ra
    sw   ra, 0(t0)              // save caller's ra before first jal overwrites it
    jal  ra, t_csr_basic
    jal  ra, t_l0_mmode_bypass
    jal  ra, t_locked_tor_load_store
    jal  ra, t_locked_fetch_fault
    jal  ra, t_napot_rw_fault
    jal  ra, t_aoff_disabled
    jal  ra, t_priority_entry0_wins
    jal  ra, t_lock_and_tor_lock    // MUST be last: leaves locked entries
    la   t0, h0_saved_ra
    lw   ra, 0(t0)              // restore caller's ra
    ret

// ---------------------------------------------------------------------------
// Hart 1 test suite (same non-leaf fix)
// ---------------------------------------------------------------------------
h1_run_tests:
    la   t0, h1_saved_ra
    sw   ra, 0(t0)
    jal  ra, t_h1_na4_rw
    jal  ra, t_h1_napot_16b
    jal  ra, t_h1_chained_tor
    la   t0, h1_saved_ra
    lw   ra, 0(t0)
    ret

// ---------------------------------------------------------------------------
// Test 1 - CSR read/write roundtrip
//   pmpaddr3 (0x3B3) and pmpcfg0 byte 0 must store and return correctly.
// ---------------------------------------------------------------------------
t_csr_basic:
    dbg_char '1'
    clear_all_pmp

    // pmpaddr3 roundtrip
    li   t0, 0x12345
    csrw PMPADDR3, t0           // pmpaddr3 = 0x3B3 (literal immediate)
    csrr t2, PMPADDR3
    bne  t0, t2, 1f
    inc_pass
    j    2f
1:  inc_fail
2:
    // pmpcfg0 byte-0 roundtrip (unlocked TOR, R=1)
    li   t0, cfg_byte(0, A_TOR, 0, 0, 1)
    csrw PMPCFG0, t0
    csrr t2, PMPCFG0
    andi t2, t2, 0xFF
    bne  t0, t2, 3f
    inc_pass
    j    4f
3:  inc_fail
4:
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 2 - L=0 in M-mode: M-mode always bypasses an unlocked entry
//   Even if the entry says R=0 W=0, M-mode ignores it when L=0.
// ---------------------------------------------------------------------------
t_l0_mmode_bypass:
    dbg_char '2'
    clear_all_pmp

    // TOR entry 0: covers [0, guard_a_data+64), L=0, R=0, W=0, X=0
    la   t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR0, t0
    li   t0, cfg_byte(0, A_TOR, 0, 0, 0)
    csrw PMPCFG0, t0
    pmp_sync

    la   t0, guard_a_data
    lw   t1, 0(t0)              // load in M-mode with L=0: must succeed
    sw   t1, 0(t0)              // store in M-mode with L=0: must succeed
    inc_pass                    // getting here means neither faulted

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 3 - L=1 TOR [guard_a_data, guard_a_data+64): R=1, W=0
//   Load from region must succeed; store must raise fault (mcause=7).
// ---------------------------------------------------------------------------
t_locked_tor_load_store:
    dbg_char '3'
    clear_all_pmp

    // pmpaddr0 = lower TOR bound (pmpaddr[i-1] for entry 1)
    la   t0, guard_a_data
    srli t0, t0, 2
    csrw PMPADDR0, t0

    // pmpaddr1 = upper TOR bound
    la   t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR1, t0

    // entry 1 (pmpcfg0 byte 1 = bits [15:8]): L=1, TOR, R=1, W=0
    li   t0, (cfg_byte(1, A_TOR, 0, 0, 1) << 8)
    csrw PMPCFG0, t0
    pmp_sync

    la   t0, guard_a_data
    lw   t1, 0(t0)              // load: R=1 -> must succeed
    inc_pass

    arm_trap 7                  // expect store/AMO access fault
    la   t0, guard_a_data
    li   t1, 0xdeadbeef
    sw   t1, 0(t0)              // store: W=0 L=1 -> must fault
    check_trapped

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 4 - L=1 NA4 at fetch_island with X=0: instruction fetch must fault.
//   fetch_island is placed in .text with .option norvc (4-byte instructions).
//   We load its address at runtime, so this is immune to linker relaxation.
//
//   IMPORTANT: jalr ra, t0, 0 overwrites ra with (PC+4 = check_trapped address).
//   We must save/restore the caller's ra around this idiom, otherwise the
//   final ret loops back to check_trapped instead of returning to h0_run_tests.
// ---------------------------------------------------------------------------
t_locked_fetch_fault:
    dbg_char '4'

    // Save caller's ra before jalr ra,t0,0 overwrites it
    la   t0, fetch4_saved_ra
    sw   ra, 0(t0)

    clear_all_pmp

    // NA4 entry 0 at fetch_island (runtime address): L=1, R=1, W=0, X=0
    li   t0, cfg_byte(1, A_NA4, 0, 0, 1)
    csrw PMPCFG0, t0
    la   t0, fetch_island       // runtime symbol address (no .org dependency)
    srli t0, t0, 2
    csrw PMPADDR0, t0
    pmp_sync

    // jalr sets ra=PC+4 (=check_trapped addr), then tries to fetch from fetch_island
    // X=0 L=1 -> instruction access fault (mcause=1)
    // Trap handler: mepc=fetch_island, advances to fetch_island+4 (jalr x0,ra)
    // That 4-byte jalr x0,ra returns to ra (=check_trapped); so check_trapped runs.
    arm_trap 1                  // expect instruction access fault (mcause=1)
    la   t0, fetch_island
    jalr ra, t0, 0
    check_trapped

    // Restore caller's ra so ret goes to h0_run_tests (not back to check_trapped)
    la   t0, fetch4_saved_ra
    lw   ra, 0(t0)
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 5 - L=1 NAPOT(8B) at guard_a_data: R=1, W=0
//   Load from region must succeed; store must fault (mcause=7).
//   guard_a_data is .align 3 (8B aligned), so NAPOT encoding is exact.
// ---------------------------------------------------------------------------
t_napot_rw_fault:
    dbg_char '5'
    clear_all_pmp

    li   t0, cfg_byte(1, A_NAPOT, 0, 0, 1)
    csrw PMPCFG0, t0
    la   t0, guard_a_data       // 8B-aligned (see .data section)
    srli t0, t0, 2              // pmpaddr = base>>2 (0 trailing 1s = 8B NAPOT)
    csrw PMPADDR0, t0
    pmp_sync

    la   t0, guard_a_data
    lw   t1, 0(t0)              // load: R=1 -> succeed
    inc_pass

    arm_trap 7                  // expect store/AMO access fault
    la   t0, guard_a_data
    li   t1, 0xA5A5A5A5
    sw   t1, 0(t0)              // store: W=0 L=1 -> fault
    check_trapped

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 6 - A=OFF entry is disabled even when L=1
//   An entry with A_OFF never matches any address; M-mode accesses succeed.
// ---------------------------------------------------------------------------
t_aoff_disabled:
    dbg_char '6'
    clear_all_pmp

    // Entry 0: L=1, A=OFF (disabled), R=0, W=0, X=0
    li   t0, cfg_byte(1, A_OFF, 0, 0, 0)
    csrw PMPCFG0, t0
    la   t0, guard_a_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    pmp_sync

    // A=OFF means no matching -> M-mode always succeeds regardless of L bit
    la   t0, guard_a_data
    li   t1, 0x12345678
    sw   t1, 0(t0)
    lw   t2, 0(t0)
    bne  t1, t2, 1f
    inc_pass
    j    2f
1:  inc_fail
2:
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 7 - Priority: entry 0 (NA4 R-only) beats entry 1 (TOR RWX)
//   Both cover guard_a_data; the lower-indexed matching entry wins.
//   Load succeeds (entry 0 R=1) but store faults (entry 0 W=0).
// ---------------------------------------------------------------------------
t_priority_entry0_wins:
    dbg_char '7'
    clear_all_pmp

    la   t3, guard_a_data
    srli t3, t3, 2              // t3 = pmpaddr for NA4 and TOR lower bound

    // pmpcfg0 byte 0 (entry 0): L=1, NA4, R=1, W=0, X=0
    // pmpcfg0 byte 1 (entry 1): L=1, TOR, R=1, W=1, X=1
    li   t0, cfg_byte(1, A_NA4, 0, 0, 1)
    li   t1, (cfg_byte(1, A_TOR, 1, 1, 1) << 8)
    or   t0, t0, t1
    csrw PMPCFG0, t0

    // pmpaddr0 = NA4 address (also serves as TOR lower bound for entry 1)
    csrw PMPADDR0, t3

    // pmpaddr1 = TOR upper bound
    la   t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR1, t0
    pmp_sync

    // Load: entry 0 (NA4) matches first -> R=1 -> succeed
    la   t0, guard_a_data
    lw   t1, 0(t0)
    inc_pass

    // Store: entry 0 (NA4) matches first -> W=0 L=1 -> fault
    arm_trap 7
    li   t1, 0x55
    sw   t1, 0(t0)
    check_trapped

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Test 8 - Lock/WARL: locked entries resist CSR writes (MUST be last test)
//   After this test, some PMP entries remain permanently locked for the run.
// ---------------------------------------------------------------------------
t_lock_and_tor_lock:
    dbg_char '8'
    clear_all_pmp

    // Set up entry 1: L=1, TOR -> this also locks pmpaddr0 (the TOR lower bound)
    la   t0, guard_a_data
    srli t0, t0, 2
    csrw PMPADDR0, t0           // pmpaddr0 = TOR lower bound (will be locked)

    la   t0, guard_a_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR1, t0           // pmpaddr1 = TOR upper bound

    li   t0, (cfg_byte(1, A_TOR, 0, 0, 1) << 8)
    csrw PMPCFG0, t0
    pmp_sync

    // 1. Attempt to write pmpcfg0 (entry 1 locked): value must not change
    csrw PMPCFG0, zero
    csrr t2, PMPCFG0
    li   t3, (cfg_byte(1, A_TOR, 0, 0, 1) << 8)
    bne  t2, t3, 1f
    inc_pass
    j    2f
1:  inc_fail
2:
    // 2. Attempt to write pmpaddr0 (locked because entry1 is L=1 TOR): must not change
    la   t3, guard_a_data
    srli t3, t3, 2              // t3 = original value of pmpaddr0
    li   t0, 0xFFFFF
    csrw PMPADDR0, t0           // try to clobber
    csrr t2, PMPADDR0
    bne  t2, t3, 3f
    inc_pass
    j    4f
3:  inc_fail
4:
    // 3. Attempt to write pmpaddr1 (locked directly as entry1 has L=1): must not change
    la   t3, guard_a_data
    addi t3, t3, 64
    srli t3, t3, 2              // t3 = original value of pmpaddr1
    csrw PMPADDR1, zero
    csrr t2, PMPADDR1
    bne  t2, t3, 5f
    inc_pass
    j    6f
5:  inc_fail
6:
    // 4. Verify locked entry survives a second clear_all_pmp attempt
    clear_all_pmp
    csrr t2, PMPCFG0
    li   t3, (cfg_byte(1, A_TOR, 0, 0, 1) << 8)
    bne  t2, t3, 7f
    inc_pass
    j    8f
7:  inc_fail
8:
    ret                         // NOTE: locked entries persist after this test

// ---------------------------------------------------------------------------
// Hart 1 Test A - NA4 with R=1 W=1 (both RW succeed)
// ---------------------------------------------------------------------------
t_h1_na4_rw:
    dbg_char 'a'
    clear_all_pmp

    // NA4 at guard_b_data: L=1, R=1, W=1
    la   t0, guard_b_data
    srli t0, t0, 2
    csrw PMPADDR0, t0
    li   t0, cfg_byte(1, A_NA4, 0, 1, 1)
    csrw PMPCFG0, t0
    pmp_sync

    la   t0, guard_b_data
    li   t1, 0x12345678
    sw   t1, 0(t0)              // store: W=1 -> succeed
    lw   t2, 0(t0)              // load:  R=1 -> succeed
    bne  t1, t2, 1f
    inc_pass
    j    2f
1:  inc_fail
2:
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Hart 1 Test B - NAPOT(16B) at guard_b_data: R=1, W=0
//   Load succeeds; store faults (mcause=7).
//   guard_b_data is .align 4 (16B aligned); pmpaddr = base>>2 | NAPOT_16B.
// ---------------------------------------------------------------------------
t_h1_napot_16b:
    dbg_char 'b'
    clear_all_pmp

    li   t0, cfg_byte(1, A_NAPOT, 0, 0, 1)
    csrw PMPCFG0, t0
    la   t0, guard_b_data       // 16B-aligned (see .data section)
    srli t0, t0, 2
    ori  t0, t0, NAPOT_16B      // set 1 trailing 1 for 16B NAPOT
    csrw PMPADDR0, t0
    pmp_sync

    la   t0, guard_b_data
    lw   t1, 0(t0)              // load: R=1 -> succeed
    inc_pass

    arm_trap 7
    la   t0, guard_b_data
    li   t1, 0xDEADC0DE
    sw   t1, 0(t0)              // store: W=0 L=1 -> fault
    check_trapped

    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Hart 1 Test C - Chained TOR [guard_b_data, guard_b_data+64): R=1, W=1
// ---------------------------------------------------------------------------
t_h1_chained_tor:
    dbg_char 'c'
    clear_all_pmp

    la   t0, guard_b_data
    srli t0, t0, 2
    csrw PMPADDR0, t0           // pmpaddr0 = TOR lower bound

    la   t0, guard_b_data
    addi t0, t0, 64
    srli t0, t0, 2
    csrw PMPADDR1, t0           // pmpaddr1 = TOR upper bound

    li   t0, (cfg_byte(1, A_TOR, 0, 1, 1) << 8)   // entry 1: L=1, TOR, R=1, W=1
    csrw PMPCFG0, t0
    pmp_sync

    la   t0, guard_b_data
    li   t1, 0xa5a5a5a5
    sw   t1, 0(t0)              // store: W=1 -> succeed
    lw   t2, 0(t0)              // load:  R=1 -> succeed
    bne  t1, t2, 1f
    inc_pass
    j    2f
1:  inc_fail
2:
    clear_all_pmp
    ret

// ---------------------------------------------------------------------------
// Reporting and finish (hart 0 only)
// ---------------------------------------------------------------------------
h0_report_and_finish:
    dbg_char 'E'

    la   t0, h0_pass_count
    lw   s1, 0(t0)
    la   t0, h1_pass_count
    lw   t1, 0(t0)
    add  s1, s1, t1             // s1 = total passes

    la   t0, h0_fail_count
    lw   s2, 0(t0)
    la   t0, h1_fail_count
    lw   t1, 0(t0)
    add  s2, s2, t1             // s2 = total fails

    la   t0, msg_banner
    jal  ra, print_str

    la   t0, msg_pass
    jal  ra, print_str
    mv   a0, s1
    jal  ra, print_dec

    la   t0, msg_fail
    jal  ra, print_str
    mv   a0, s2
    jal  ra, print_dec

    la   t0, msg_nl
    jal  ra, print_str

    bnez s2, test_fail
    li   t0, STDOUT
    li   t1, 0xff               // 0xff -> testbench signals TEST_PASSED
    sb   t1, 0(t0)
1:  j 1b

test_fail:
    li   t0, STDOUT
    li   t1, 0x01               // 0x01 -> testbench signals TEST_FAILED
    sb   t1, 0(t0)
2:  j 2b

// ---------------------------------------------------------------------------
// Utility: print null-terminated string at t0 to STDOUT
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
// Utility: print decimal number in a0 to STDOUT (handles 0-99; no stack used)
// ---------------------------------------------------------------------------
print_dec:
    li   t1, STDOUT
    li   t2, 10
    blt  a0, t2, 1f             // single digit
    div  t3, a0, t2             // tens digit
    rem  a0, a0, t2             // units digit
    addi t3, t3, '0'
    sb   t3, 0(t1)
1:  addi a0, a0, '0'
    sb   a0, 0(t1)
    ret

// ---------------------------------------------------------------------------
// Trap vector (must be aligned; direct mode, mtvec[1:0]=0b00)
// ---------------------------------------------------------------------------
.align 6
trap_entry:
    csrr t0, mcause
    csrr t1, mtval
    csrr t2, mepc

    // Save cause, mtval, epc to memory for post-mortem inspection
    la   t3, trap_cause_val
    sw   t0, 0(t3)
    la   t3, trap_mtval_val
    sw   t1, 0(t3)
    la   t3, trap_epc_val
    sw   t2, 0(t3)

    la   t3, expect_trap_flag
    lw   t4, 0(t3)
    beqz t4, trap_unexpected    // flag=0 means we never armed -> unexpected

    // Trap was expected: check cause matches
    sw   zero, 0(t3)            // clear expect_trap_flag
    la   t3, expect_cause_val
    lw   t4, 0(t3)
    bne  t0, t4, trap_wrong_cause

    // Cause matches: increment pass counter for correct hart
    get_tid t5
    bnez t5, trap_h1_pass
    la   t6, h0_pass_count
    j    trap_inc_pass
trap_h1_pass:
    la   t6, h1_pass_count
trap_inc_pass:
    lw   t4, 0(t6)
    addi t4, t4, 1
    sw   t4, 0(t6)
    addi t2, t2, 4              // advance past faulting instruction (32-bit)
    csrw mepc, t2
    mret

trap_wrong_cause:
    // Expected a trap but got the wrong cause: count as fail, continue
    get_tid t5
    bnez t5, trap_wc_h1
    la   t6, h0_fail_count
    j    trap_wc_inc
trap_wc_h1:
    la   t6, h1_fail_count
trap_wc_inc:
    lw   t4, 0(t6)
    addi t4, t4, 1
    sw   t4, 0(t6)
    addi t2, t2, 4
    csrw mepc, t2
    mret

trap_unexpected:
    // No trap was expected: increment fail, advance PC and continue
    // (Do NOT spin forever; let the test finish and report all failures.)
    get_tid t5
    bnez t5, trap_unx_h1
    la   t6, h0_fail_count
    j    trap_unx_inc
trap_unx_h1:
    la   t6, h1_fail_count
trap_unx_inc:
    lw   t4, 0(t6)
    addi t4, t4, 1
    sw   t4, 0(t6)
    addi t2, t2, 4
    csrw mepc, t2
    mret

// ---------------------------------------------------------------------------
// Fetch island: placed here in .text with norvc so every instruction is
// exactly 4 bytes.  NA4 covers exactly [fetch_island, fetch_island+4).
// Instruction fetch at fetch_island faults when X=0 is locked (mcause=1).
// Trap handler advances mepc to fetch_island+4 (the jalr x0,ra = ret);
// that instruction uses ra (saved by the test's jalr) to return to the test.
// ---------------------------------------------------------------------------
.option push
.option norvc
.align 2
fetch_island:
    addi a0, a0, 1              // +0: 4 bytes, covered by NA4, X=0 -> fetch fault
    jalr x0, 0(ra)              // +4: 4 bytes, outside NA4, executed after trap advance
.option pop

// ---------------------------------------------------------------------------
// Data section
// ---------------------------------------------------------------------------
.section .data

// 4-byte counters and flags (word-aligned)
.align 2
h0_pass_count:   .word 0
h0_fail_count:   .word 0
h1_pass_count:   .word 0
h1_fail_count:   .word 0
h1_done_flag:    .word 0
h1_start_flag:   .word 0
expect_trap_flag: .word 0
expect_cause_val: .word 0

// Saved return addresses for non-leaf suite functions and the fetch-fault jalr
h0_saved_ra:     .word 0
h1_saved_ra:     .word 0
fetch4_saved_ra: .word 0

// Trap diagnostic (filled by trap_entry for post-mortem debugging)
trap_cause_val:  .word 0
trap_mtval_val:  .word 0
trap_epc_val:    .word 0

// Guard region A: 8B-aligned for NAPOT(8B) test, 64 bytes
.align 3
guard_a_data:    .fill 64, 1, 0

// Guard region B: 16B-aligned for NAPOT(16B) test, 64 bytes
.align 4
guard_b_data:    .fill 64, 1, 0

// Strings (null-terminated)
msg_banner: .asciz "\nVeeR-EH2 PMP test\n"
msg_pass:   .asciz "  pass="
msg_fail:   .asciz " fail="
msg_nl:     .asciz "\n"
