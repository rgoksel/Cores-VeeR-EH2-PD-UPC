`timescale 1ns/1ps

module tb_eh2_pmp;

    localparam int NUM_ENTRIES = 16;

    logic [7:0]  pmpcfg  [NUM_ENTRIES];
    logic [31:0] pmpaddr [NUM_ENTRIES];
    logic [33:0] addr;
    logic [1:0]  access_type;
    logic [1:0]  priv_mode;
    logic        allow;
    logic        fault;
    logic [2:0]  access_size;
    logic [4:0]  fault_cause;

    localparam [1:0] FETCH = 2'b00;
    localparam [1:0] LOAD  = 2'b01;
    localparam [1:0] STORE = 2'b10;
    localparam [1:0] BAD_ACCESS = 2'b11;

    localparam [1:0] U_MODE = 2'b00;
    localparam [1:0] S_MODE = 2'b01;
    localparam [1:0] M_MODE = 2'b11;

    localparam [1:0] A_OFF   = 2'b00;
    localparam [1:0] A_TOR   = 2'b01;
    localparam [1:0] A_NA4   = 2'b10;
    localparam [1:0] A_NAPOT = 2'b11;

    int total = 0;
    int passed = 0;

    eh2_pmp #(
    .NUM_ENTRIES(NUM_ENTRIES),
    .PADDR_WIDTH(34),
    .G(0)
            ) dut (
    .pmpcfg(pmpcfg),
    .pmpaddr(pmpaddr),
    .addr(addr),
    .access_size(access_size),
    .access_type(access_type),
    .priv_mode(priv_mode),
    .allow(allow),
    .fault(fault),
    .fault_cause(fault_cause)
);

    function automatic [7:0] cfg(
    input [1:0] mode,
    input logic x,
    input logic w,
    input logic r
);
    cfg = {1'b0, 2'b00, mode, x, w, r};
endfunction

function automatic [7:0] cfg_locked(
    input [1:0] mode,
    input logic x,
    input logic w,
    input logic r
);
    cfg_locked = {1'b1, 2'b00, mode, x, w, r};
endfunction

    function automatic [31:0] napot_addr(
        input [33:0] base,
        input int unsigned size_bytes
    );
        // Works with the simplified checker’s encoding.
        napot_addr = (base[33:2] | ((size_bytes >> 3) - 1));
    endfunction

    task automatic clear_pmp;
        int i;
        begin
            access_size = 3'd0; // 1 byte by default
            for (i = 0; i < NUM_ENTRIES; i++) begin
                pmpcfg[i]  = 8'h00;
                pmpaddr[i] = 32'h00000000;
            end
            addr        = 34'h0;
            access_type = LOAD;
            priv_mode   = M_MODE;
        end
    endtask

    task automatic check(
        input string name,
        input logic expected_allow
    );
        begin
            #1;
            total++;
            if (allow === expected_allow && fault === ~expected_allow) begin
                passed++;
                $display("[PASS] %s", name);
            end else begin
                $display("[FAIL] %s | expected allow=%0b fault=%0b, got allow=%0b fault=%0b",
                         name, expected_allow, ~expected_allow, allow, fault);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_eh2_pmp.vcd");
        $dumpvars;

        $display("Starting PMP checker tests...");

        // GROUP A: No active PMP entries
        //   - No match behavior
        //   - M-mode default allow
        //   - U-mode default deny
        begin : group_a_no_match
            clear_pmp();

            addr        = 34'h0000_1000;
            access_type = LOAD;

            priv_mode = M_MODE;
            check("A1 No match, M-mode load: allow", 1'b1);

            priv_mode = U_MODE;
            check("A2 No match, U-mode load: fault", 1'b0);
        end

        // GROUP B: NA4 region
        //   - Exact 4-byte region matching
        //   - Lower boundary, last byte, outside region
        begin : group_b_na4
            clear_pmp();

            pmpcfg[0]  = cfg(A_NA4, 1'b1, 1'b1, 1'b1); // RWX
            pmpaddr[0] = 32'h0000_3000 >> 2;
            priv_mode  = U_MODE;

            addr        = 34'h0000_3000;
            access_type = LOAD;
            check("B1 NA4 load at base: allow", 1'b1);

            addr        = 34'h0000_3003;
            access_type = FETCH;
            check("B2 NA4 fetch at last byte: allow", 1'b1);

            addr        = 34'h0000_3004;
            access_type = LOAD;
            check("B3 NA4 just after region: fault", 1'b0);

            addr        = 34'h0000_2FFF;
            access_type = LOAD;
            check("B4 NA4 just before region: fault", 1'b0);
        end

        // GROUP C: TOR, entry 0
        //   - Region [0, top)
        //   - Inclusive lower bound, exclusive upper bound
        //   - Permission check inside matched region
        begin : group_c_tor_entry0
            clear_pmp();

            pmpcfg[0]  = cfg(A_TOR, 1'b0, 1'b0, 1'b1); // R only
            pmpaddr[0] = 32'h0000_1000 >> 2;
            priv_mode  = U_MODE;

            addr        = 34'h0000_0000;
            access_type = LOAD;
            check("C1 TOR [0,0x1000): load at lower bound: allow", 1'b1);

            addr        = 34'h0000_0FFF;
            access_type = LOAD;
            check("C2 TOR [0,0x1000): load at upper-1: allow", 1'b1);

            addr        = 34'h0000_1000;
            access_type = LOAD;
            check("C3 TOR [0,0x1000): load at exclusive upper: fault", 1'b0);

            addr        = 34'h0000_0800;
            access_type = STORE;
            check("C4 TOR [0,0x1000): store in R-only region: fault", 1'b0);
        end

        // GROUP D: Chained TOR
        //   - Entry 1 covers [pmpaddr0<<2, pmpaddr1<<2)
        //   - Tests lower/upper boundaries
        begin : group_d_chained_tor
            clear_pmp();

            // Entry 0 defines lower bound for entry 1.
            pmpcfg[0]  = cfg(A_OFF, 1'b0, 1'b0, 1'b0);
            pmpaddr[0] = 32'h0000_1000 >> 2;

            // Entry 1 covers [0x1000, 0x2000), R/W
            pmpcfg[1]  = cfg(A_TOR, 1'b0, 1'b1, 1'b1);
            pmpaddr[1] = 32'h0000_2000 >> 2;
            priv_mode  = U_MODE;

            addr        = 34'h0000_1000;
            access_type = LOAD;
            check("D1 Chained TOR load at lower bound: allow", 1'b1);

            addr        = 34'h0000_1FFF;
            access_type = STORE;
            check("D2 Chained TOR store at upper-1: allow", 1'b1);

            addr        = 34'h0000_0FFF;
            access_type = LOAD;
            check("D3 Chained TOR just below lower: fault", 1'b0);

            addr        = 34'h0000_2000;
            access_type = LOAD;
            check("D4 Chained TOR at exclusive upper: fault", 1'b0);

            addr        = 34'h0000_1800;
            access_type = FETCH;
            check("D5 Chained TOR fetch with X=0: fault", 1'b0);
        end

        // GROUP E: NAPOT 8-byte region
        //   - Smallest NAPOT region supported by this checker
        //   - Boundary checks
        //   - Permission checks
        begin : group_e_napot_8b
            clear_pmp();

            pmpcfg[0]  = cfg(A_NAPOT, 1'b0, 1'b1, 1'b1); // R/W
            pmpaddr[0] = napot_addr(34'h0000_2000, 8);
            priv_mode  = U_MODE;

            addr        = 34'h0000_2000;
            access_type = LOAD;
            check("E1 NAPOT 8B load at base: allow", 1'b1);

            addr        = 34'h0000_2007;
            access_type = STORE;
            check("E2 NAPOT 8B store at last byte: allow", 1'b1);

            addr        = 34'h0000_2004;
            access_type = FETCH;
            check("E3 NAPOT 8B fetch with X=0: fault", 1'b0);

            addr        = 34'h0000_1FFF;
            access_type = LOAD;
            check("E4 NAPOT 8B just before base: fault", 1'b0);

            addr        = 34'h0000_2008;
            access_type = LOAD;
            check("E5 NAPOT 8B just after region: fault", 1'b0);
        end

        // GROUP F: More NAPOT sizes
        //   - 16B, 64B, 4KB regions
        begin : group_f_more_napot_sizes
            clear_pmp();
            priv_mode = U_MODE;

            // 16-byte region: [0x4000, 0x4010)
            pmpcfg[0]  = cfg(A_NAPOT, 1'b1, 1'b1, 1'b1);
            pmpaddr[0] = napot_addr(34'h0000_4000, 16);

            addr        = 34'h0000_4000;
            access_type = LOAD;
            check("F1 NAPOT 16B load at base: allow", 1'b1);

            addr        = 34'h0000_400F;
            access_type = STORE;
            check("F2 NAPOT 16B store at last byte: allow", 1'b1);

            addr        = 34'h0000_4010;
            access_type = LOAD;
            check("F3 NAPOT 16B just after region: fault", 1'b0);

            // 64-byte region: [0x5000, 0x5040)
            pmpcfg[0]  = cfg(A_NAPOT, 1'b1, 1'b0, 1'b1); // R/X only
            pmpaddr[0] = napot_addr(34'h0000_5000, 64);

            addr        = 34'h0000_5000;
            access_type = FETCH;
            check("F4 NAPOT 64B fetch at base: allow", 1'b1);

            addr        = 34'h0000_503F;
            access_type = LOAD;
            check("F5 NAPOT 64B load at last byte: allow", 1'b1);

            addr        = 34'h0000_5040;
            access_type = LOAD;
            check("F6 NAPOT 64B just after region: fault", 1'b0);

            addr        = 34'h0000_5020;
            access_type = STORE;
            check("F7 NAPOT 64B store with W=0: fault", 1'b0);

            // 4KB region: [0x80000000, 0x80001000)
            pmpcfg[0]  = cfg(A_NAPOT, 1'b1, 1'b1, 1'b1);
            pmpaddr[0] = napot_addr(34'h0_8000_0000, 4096);

            addr        = 34'h0_8000_0000;
            access_type = LOAD;
            check("F8 NAPOT 4KB load at base: allow", 1'b1);

            addr        = 34'h0_8000_0FFF;
            access_type = FETCH;
            check("F9 NAPOT 4KB fetch at last byte: allow", 1'b1);

            addr        = 34'h0_8000_1000;
            access_type = LOAD;
            check("F10 NAPOT 4KB just after region: fault", 1'b0);
        end

        // GROUP G: Permission combinations
        //   - Checks R-only, W-only, X-only, RW, RX, RWX, none.
        //   - W only: reserved in RISC-V PMP because R=0, W=1.
        begin : group_g_permissions
            clear_pmp();
            priv_mode  = U_MODE;
            pmpaddr[0] = napot_addr(34'h0000_6000, 8);
            addr       = 34'h0000_6000;

            // R only
            pmpcfg[0] = cfg(A_NAPOT, 1'b0, 1'b0, 1'b1);
            access_type = LOAD;  check("G1 R-only: LOAD allow", 1'b1);
            access_type = STORE; check("G2 R-only: STORE fault", 1'b0);
            access_type = FETCH; check("G3 R-only: FETCH fault", 1'b0);

            // W only
            pmpcfg[0] = cfg(A_NAPOT, 1'b0, 1'b1, 1'b0);
            access_type = LOAD;  check("G4 W-only reserved: LOAD fault", 1'b0);
            access_type = STORE; check("G5 W-only reserved: STORE fault", 1'b0);
            access_type = FETCH; check("G6 W-only reserved: FETCH fault", 1'b0);

            // X only
            pmpcfg[0] = cfg(A_NAPOT, 1'b1, 1'b0, 1'b0);
            access_type = LOAD;  check("G7 X-only: LOAD fault", 1'b0);
            access_type = STORE; check("G8 X-only: STORE fault", 1'b0);
            access_type = FETCH; check("G9 X-only: FETCH allow", 1'b1);

            // RW
            pmpcfg[0] = cfg(A_NAPOT, 1'b0, 1'b1, 1'b1);
            access_type = LOAD;  check("G10 RW: LOAD allow", 1'b1);
            access_type = STORE; check("G11 RW: STORE allow", 1'b1);
            access_type = FETCH; check("G12 RW: FETCH fault", 1'b0);

            // RX
            pmpcfg[0] = cfg(A_NAPOT, 1'b1, 1'b0, 1'b1);
            access_type = LOAD;  check("G13 RX: LOAD allow", 1'b1);
            access_type = STORE; check("G14 RX: STORE fault", 1'b0);
            access_type = FETCH; check("G15 RX: FETCH allow", 1'b1);

            // RWX
            pmpcfg[0] = cfg(A_NAPOT, 1'b1, 1'b1, 1'b1);
            access_type = LOAD;  check("G16 RWX: LOAD allow", 1'b1);
            access_type = STORE; check("G17 RWX: STORE allow", 1'b1);
            access_type = FETCH; check("G18 RWX: FETCH allow", 1'b1);

            // No permissions
            pmpcfg[0] = cfg(A_NAPOT, 1'b0, 1'b0, 1'b0);
            access_type = LOAD;  check("G19 none: LOAD fault", 1'b0);
            access_type = STORE; check("G20 none: STORE fault", 1'b0);
            access_type = FETCH; check("G21 none: FETCH fault", 1'b0);
        end

        // GROUP H: Priority with overlapping entries
        //   - Entry 0 denies while entry 1 allows
        //   - Entry 0 allows while entry 1 denies
        //   - Confirms lower entry number wins.
        begin : group_h_priority
            clear_pmp();
            priv_mode = U_MODE;

            // Entry 0: NA4 at 0x7000, R only
            pmpcfg[0]  = cfg(A_NA4, 1'b0, 1'b0, 1'b1);
            pmpaddr[0] = 32'h0000_7000 >> 2;

            // Entry 1: TOR [0, 0x8000), RWX
            pmpcfg[1]  = cfg(A_TOR, 1'b1, 1'b1, 1'b1);
            pmpaddr[1] = 32'h0000_8000 >> 2;

            addr        = 34'h0000_7000;
            access_type = STORE;
            check("H1 Priority: entry0 R-only beats entry1 RWX, STORE fault", 1'b0);

            access_type = LOAD;
            check("H2 Priority: entry0 R-only beats entry1 RWX, LOAD allow", 1'b1);

            // Reverse: entry 0 allows, entry 1 denies
            clear_pmp();
            priv_mode = U_MODE;

            pmpcfg[0]  = cfg(A_NA4, 1'b1, 1'b1, 1'b1); // RWX
            pmpaddr[0] = 32'h0000_7000 >> 2;

            pmpcfg[1]  = cfg(A_TOR, 1'b0, 1'b0, 1'b0); // no perms
            pmpaddr[1] = 32'h0000_8000 >> 2;

            addr        = 34'h0000_7000;
            access_type = STORE;
            check("H3 Priority: entry0 RWX beats entry1 none, STORE allow", 1'b1);

            // Partial-overlap multi-byte priority case.
            //
            // Entry 0 covers [0x7004, 0x7008).
            // Entry 1 covers [0x7000, 0x7010).
            // The 8-byte load accesses [0x7000, 0x7008).
            //
            // Entry 0 is lower numbered and matches part of the load.
            // It does not cover all bytes, so the specification says
            // that the complete operation must fault.
            clear_pmp();
            priv_mode = U_MODE;

            pmpcfg[0]  = cfg(A_NA4, 1'b1, 1'b1, 1'b1);
            pmpaddr[0] = 32'h0000_7004 >> 2;

            pmpcfg[1]  = cfg(A_NAPOT, 1'b1, 1'b1, 1'b1);
            pmpaddr[1] = napot_addr(34'h0000_7000, 16);

            addr        = 34'h0000_7000;
            access_size = 3'd3;
            access_type = LOAD;
            check("H4 Partial overlap: entry0 matches part of 8B load, fault", 1'b0);
        end

        // GROUP I: Invalid access type
        //   - access_type=2'b11 should never be allowed.
        begin : group_i_invalid_access
            clear_pmp();

            pmpcfg[0]  = cfg(A_NA4, 1'b1, 1'b1, 1'b1);
            pmpaddr[0] = 32'h0000_9000 >> 2;
            priv_mode  = U_MODE;

            addr        = 34'h0000_9000;
            access_type = BAD_ACCESS;
            check("I1 Invalid access_type=11 inside RWX region: fault", 1'b0);
        end

        // GROUP J: M-mode and L-bit behavior
        //   - If L=0 and access is M-mode, matched PMP entries should not restrict access.
        //   - If L=1, M-mode must obey R/W/X permissions.
        begin : group_j_mmode_lock_bit
        clear_pmp();

        // Entry 0: NA4 at 0xA000, R only, unlocked.
        // M-mode store should still be allowed because L=0.
        pmpcfg[0]  = cfg(A_NA4, 1'b0, 1'b0, 1'b1);
        pmpaddr[0] = 32'h0000_A000 >> 2;

        addr        = 34'h0000_A000;
        access_type = STORE;
        priv_mode   = M_MODE;
        check("J1 M-mode matched entry, L=0, W=0: STORE allow", 1'b1);

        // Same entry, but locked.
        // Now M-mode must obey permissions, so store should fault.
        pmpcfg[0]  = cfg_locked(A_NA4, 1'b0, 1'b0, 1'b1);

        addr        = 34'h0000_A000;
        access_type = STORE;
        priv_mode   = M_MODE;
        check("J2 M-mode matched entry, L=1, W=0: STORE fault", 1'b0);

        // Locked entry with X=1 should allow fetch.
        pmpcfg[0]  = cfg_locked(A_NA4, 1'b1, 1'b0, 1'b1);

        addr        = 34'h0000_A000;
        access_type = FETCH;
        priv_mode   = M_MODE;
        check("J3 M-mode matched entry, L=1, X=1: FETCH allow", 1'b1);
        end

        // GROUP K: 16-entry support
        //   - Confirms that NUM_ENTRIES=16 is really being used.
        //   - Tests entry 15.
        //   - Confirms entry 0 still has higher priority than entry 15.
        begin : group_k_16_entries
            clear_pmp();
            priv_mode = U_MODE;

            // Entry 15: TOR [0, 0xB000), RWX
            pmpcfg[15]  = cfg(A_TOR, 1'b1, 1'b1, 1'b1);
            pmpaddr[15] = 32'h0000_B000 >> 2;

            addr        = 34'h0000_A800;
            access_type = LOAD;
            check("K1 Entry 15 TOR match: LOAD allow", 1'b1);

            addr        = 34'h0000_A800;
            access_type = STORE;
            check("K2 Entry 15 TOR match: STORE allow", 1'b1);

            // Entry 0 overlaps and is higher priority.
            // Entry 0 is R-only at 0xA800, so STORE should now fault.
            pmpcfg[0]  = cfg(A_NA4, 1'b0, 1'b0, 1'b1);
            pmpaddr[0] = 32'h0000_A800 >> 2;

            addr        = 34'h0000_A800;
            access_type = STORE;
            check("K3 Entry 0 overrides entry 15: STORE fault", 1'b0);

            access_type = LOAD;
            check("K4 Entry 0 overrides entry 15: LOAD allow", 1'b1);
        end

        // GROUP S: S-mode permission decisions
        //   - No active entries: S-mode access must fault (at least one
        //     entry is implemented, so the "no PMP entry matches" default
        //     applies, unlike M-mode).
        //   - NA4 region: R-only and X-only permission checks in S-mode.
        //   - Confirms the L bit does not change S-mode behavior: an
        //     unlocked entry restricts S-mode exactly like a locked one.
        //   - TOR and NAPOT boundary checks in S-mode.
        //   - Priority: the lowest-numbered matching entry still decides
        //     the result when the access is from S-mode.
        begin : group_s_smode
            clear_pmp();
            priv_mode = S_MODE;

            addr        = 34'h0000_D000;
            access_type = LOAD;
            check("S1 No active entries, S-mode load: fault", 1'b0);

            // NA4 region, R only.
            pmpcfg[0]  = cfg(A_NA4, 1'b0, 1'b0, 1'b1);
            pmpaddr[0] = 32'h0000_D000 >> 2;

            addr        = 34'h0000_D000;
            access_type = LOAD;
            check("S2 NA4 R-only region, S-mode load: allow", 1'b1);

            access_type = STORE;
            check("S3 NA4 R-only region, S-mode store: fault", 1'b0);

            // Same region, X only, unlocked: S-mode must still obey X/R,
            // unlike M-mode where L=0 would bypass the check entirely.
            pmpcfg[0] = cfg(A_NA4, 1'b1, 1'b0, 1'b0);

            access_type = FETCH;
            check("S4 NA4 X-only unlocked, S-mode fetch: allow", 1'b1);

            access_type = LOAD;
            check("S5 NA4 X-only unlocked, S-mode load: fault", 1'b0);

            // Same permissions, now locked: result must be identical to
            // the unlocked case above, since L only affects M-mode.
            pmpcfg[0] = cfg_locked(A_NA4, 1'b1, 1'b0, 1'b0);

            access_type = FETCH;
            check("S6 NA4 X-only locked, S-mode fetch: allow", 1'b1);

            access_type = LOAD;
            check("S7 NA4 X-only locked, S-mode load: fault", 1'b0);

            // TOR region [0, 0xD000), RWX, entry 0.
            clear_pmp();
            priv_mode  = S_MODE;
            pmpcfg[0]  = cfg(A_TOR, 1'b1, 1'b1, 1'b1);
            pmpaddr[0] = 32'h0000_D000 >> 2;

            addr        = 34'h0000_0000;
            access_type = LOAD;
            check("S8 TOR [0,0xD000): S-mode load at lower bound: allow", 1'b1);

            addr        = 34'h0000_CFFF;
            access_type = STORE;
            check("S9 TOR [0,0xD000): S-mode store at upper-1: allow", 1'b1);

            addr        = 34'h0000_D000;
            access_type = LOAD;
            check("S10 TOR [0,0xD000): S-mode load at exclusive upper: fault", 1'b0);

            // NAPOT 16-byte region, RWX.
            clear_pmp();
            priv_mode  = S_MODE;
            pmpcfg[0]  = cfg(A_NAPOT, 1'b1, 1'b1, 1'b1);
            pmpaddr[0] = napot_addr(34'h0000_D000, 16);

            addr        = 34'h0000_D000;
            access_type = FETCH;
            check("S11 NAPOT 16B RWX, S-mode fetch: allow", 1'b1);

            addr        = 34'h0000_D010;
            access_type = LOAD;
            check("S12 NAPOT 16B just after region, S-mode load: fault", 1'b0);

            // Priority: entry 0 (R-only) outranks entry 1 (RWX) in S-mode too.
            clear_pmp();
            priv_mode  = S_MODE;

            pmpcfg[0]  = cfg(A_NA4, 1'b0, 1'b0, 1'b1);
            pmpaddr[0] = 32'h0000_D000 >> 2;

            pmpcfg[1]  = cfg(A_TOR, 1'b1, 1'b1, 1'b1);
            pmpaddr[1] = 32'h0000_E000 >> 2;

            addr        = 34'h0000_D000;
            access_type = STORE;
            check("S13 Priority: entry0 R-only beats entry1 RWX, S-mode store: fault", 1'b0);

            access_type = LOAD;
            check("S14 Priority: entry0 R-only beats entry1 RWX, S-mode load: allow", 1'b1);
        end

        // GROUP M: Multi-byte access spanning a region boundary
        //   - A single access whose byte span leaves the matched region must
        //     fault, even when its start address is inside the region. This
        //     exercises the "match all bytes" requirement against a single
        //     entry, complementing the two-entry partial-overlap case (H4).
        begin : group_m_span
            clear_pmp();
            priv_mode = U_MODE;

            // NA4 region covering [0x3000, 0x3004), RWX.
            pmpcfg[0]  = cfg(A_NA4, 1'b1, 1'b1, 1'b1);
            pmpaddr[0] = 32'h0000_3000 >> 2;

            // 4-byte load fully inside the region: allow.
            addr        = 34'h0000_3000;
            access_size = 3'd2;
            access_type = LOAD;
            check("M1 4B load fully inside NA4: allow", 1'b1);

            // 2-byte load straddling the upper edge [0x3003, 0x3005): fault.
            addr        = 34'h0000_3003;
            access_size = 3'd1;
            access_type = LOAD;
            check("M2 2B load straddles NA4 upper edge: fault", 1'b0);

            // 4-byte load starting one byte past base [0x3001, 0x3005): fault.
            addr        = 34'h0000_3001;
            access_size = 3'd2;
            access_type = LOAD;
            check("M3 misaligned 4B load leaves NA4 region: fault", 1'b0);

            // NAPOT 8-byte region [0x4000, 0x4008), RWX.
            clear_pmp();
            priv_mode  = U_MODE;
            pmpcfg[0]  = cfg(A_NAPOT, 1'b1, 1'b1, 1'b1);
            pmpaddr[0] = napot_addr(34'h0000_4000, 8);

            // 8-byte load fully inside: allow.
            addr        = 34'h0000_4000;
            access_size = 3'd3;
            access_type = LOAD;
            check("M4 8B load fully inside NAPOT 8B: allow", 1'b1);

            // 8-byte load starting at 0x4004 spans [0x4004, 0x400C): fault.
            addr        = 34'h0000_4004;
            access_size = 3'd3;
            access_type = LOAD;
            check("M5 8B load spills past NAPOT 8B region: fault", 1'b0);
        end

        // GROUP N: Empty TOR range
        //   - When pmpaddr[i-1] >= pmpaddr[i] and entry i is TOR, the entry
        //     matches no address. An access in the would-be range must fall
        //     through to the no-match default (deny in U-mode).
        begin : group_n_empty_tor
            clear_pmp();
            priv_mode = U_MODE;

            // Entry 0 sets the lower bound above entry 1's upper bound.
            pmpcfg[0]  = cfg(A_OFF, 1'b0, 1'b0, 1'b0);
            pmpaddr[0] = 32'h0000_2000 >> 2;

            // Entry 1 TOR with upper bound below the lower bound: empty range.
            pmpcfg[1]  = cfg(A_TOR, 1'b1, 1'b1, 1'b1);
            pmpaddr[1] = 32'h0000_1000 >> 2;

            addr        = 34'h0000_1800;
            access_size = 3'd2;
            access_type = LOAD;
            check("N1 Empty TOR (hi<lo) matches nothing, U-mode load: fault", 1'b0);

            // Sanity: a normal non-empty TOR at entry 1 still matches.
            clear_pmp();
            priv_mode  = U_MODE;
            pmpcfg[0]  = cfg(A_OFF, 1'b0, 1'b0, 1'b0);
            pmpaddr[0] = 32'h0000_1000 >> 2;
            pmpcfg[1]  = cfg(A_TOR, 1'b1, 1'b1, 1'b1);
            pmpaddr[1] = 32'h0000_2000 >> 2;

            addr        = 34'h0000_1800;
            access_size = 3'd2;
            access_type = LOAD;
            check("N2 Non-empty TOR [0x1000,0x2000) load: allow", 1'b1);
        end

        /*
        // GROUP L: 64-entry support
        //   - Confirms that NUM_ENTRIES=64 is really being used.
        //   - Tests entry 63.
        //   - Confirms entry 0 still has higher priority than entry 63.
        begin : group_l_64_entries
            clear_pmp();
            priv_mode = U_MODE;

            // Entry 63: TOR [0, 0xC000), RWX
            pmpcfg[63]  = cfg(A_TOR, 1'b1, 1'b1, 1'b1);
            pmpaddr[63] = 32'h0000_C000 >> 2;

            addr        = 34'h0000_B800;
            access_type = LOAD;
            check("L1 Entry 63 TOR match: LOAD allow", 1'b1);

            addr        = 34'h0000_B800;
            access_type = STORE;
            check("L2 Entry 63 TOR match: STORE allow", 1'b1);

            addr        = 34'h0000_C000;
            access_type = LOAD;
            check("L3 Entry 63 TOR exclusive upper bound: LOAD fault", 1'b0);

            // Entry 0 overlaps and must win.
            // Entry 0 is R-only at 0xB800, so STORE should now fault.
            pmpcfg[0]  = cfg(A_NA4, 1'b0, 1'b0, 1'b1);
            pmpaddr[0] = 32'h0000_B800 >> 2;

            addr        = 34'h0000_B800;
            access_type = STORE;
            check("L4 Entry 0 overrides entry 63: STORE fault", 1'b0);

            access_type = LOAD;
            check("L5 Entry 0 overrides entry 63: LOAD allow", 1'b1);
        end
        */

        $display("");
        $display("PMP checker results: %0d / %0d passed", passed, total);
        if (passed == total)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", total - passed);
        $finish;
    end

endmodule