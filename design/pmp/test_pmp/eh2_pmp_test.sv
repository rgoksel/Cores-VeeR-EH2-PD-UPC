//////////////////////////////7
///
///pmp
/////////////////////////////////
module eh2_pmp
#(
    // Spec allows only 0, 16, or 64.
    parameter int NUM_ENTRIES  = 16,
        // Physical address width in bits.
    //   RV32: 34 (pmpaddr CSR encodes addr[33:2] in 32 bits)
    //   RV64: 56
    parameter int PADDR_WIDTH  = 34,
        // PMP granularity exponent G 
    //   Grain size = 2^(G+2) bytes.  G=0 : 4-byte minimum
    parameter int G            = 0
)(
    // PMP CSR contents (from TLU/CSR block) 
    input  logic [7:0]              pmpcfg  [NUM_ENTRIES],  // 8-bit config per entry
    input  logic [PADDR_WIDTH-3:0]  pmpaddr [NUM_ENTRIES],  // encodes addr[PADDR_WIDTH-1:2]

    // Memory access descriptor
    input  logic [PADDR_WIDTH-1:0]  addr,         // Physical byte address of the access
    input  logic [2:0]              access_size,  // 0=1B, 1=2B, 2=4B, 3=8B
    input  logic [1:0]              access_type,  // 00=FETCH, 01=LOAD, 10=STORE
    input  logic [1:0]              priv_mode,    // 11=M-mode, 01=S-mode, 00=U-mode

    // Results
    output logic                    allow,        // 1 = access permitted
    output logic                    fault,        // 1 = access denied : raise exception
    output logic [4:0]              fault_cause   // mcause code (valid when fault=1)
);

    // Address-matching mode encodings (pmpcfg[4:3])
    localparam [1:0] A_OFF   = 2'b00;   // Entry disabled — matches nothing
    localparam [1:0] A_TOR   = 2'b01;   // Top-of-Range
    localparam [1:0] A_NA4   = 2'b10;   // Naturally-Aligned 4-byte region
    localparam [1:0] A_NAPOT = 2'b11;   // Naturally-Aligned Power-of-Two (≥8 B)

    // Privilege mode encodings
    localparam [1:0] PRIV_U = 2'b00;
    localparam [1:0] PRIV_S = 2'b01;
    localparam [1:0] PRIV_M = 2'b11;

    // Access type encodings
    localparam [1:0] ACC_FETCH = 2'b00;
    localparam [1:0] ACC_LOAD  = 2'b01;
    localparam [1:0] ACC_STORE = 2'b10;

    // Exception cause codes
    localparam [4:0] CAUSE_INST_ACCESS_FAULT  = 5'h01;
    localparam [4:0] CAUSE_LOAD_ACCESS_FAULT  = 5'h05;
    localparam [4:0] CAUSE_STORE_ACCESS_FAULT = 5'h07;

    // Width of a pmpaddr CSR field (= PADDR_WIDTH - 2)
    localparam int PA_BITS = PADDR_WIDTH - 2;

    // Granularity masks
    // When G ≥ 1, A=OFF/TOR: pmpaddr[G-1:0] reads as 0  : mask out in TOR logic
    // When G ≥ 2, A=NAPOT  : pmpaddr[G-2:0] reads as 1  : force on in NAPOT logic

    // We compute the masks via a safe shift on 64-bit integers to avoid overflow
    localparam int G_TOR   = (G >= 1) ? G   : 0;
    localparam int G_NAPOT = (G >= 2) ? G-1 : 0;

    localparam [PA_BITS-1:0] GRAN_MASK_TOR   = (G_TOR   > 0) ? PA_BITS'((64'h1 << G_TOR)   - 64'h1) : {PA_BITS{1'b0}};
    localparam [PA_BITS-1:0] GRAN_MASK_NAPOT = (G_NAPOT > 0) ? PA_BITS'((64'h1 << G_NAPOT) - 64'h1) : {PA_BITS{1'b0}};

    // "The matching PMP entry must match ALL bytes of an access."
    logic [PADDR_WIDTH-1:0] size_m1;
    logic [PADDR_WIDTH-1:0] addr_last;

    always_comb begin
        size_m1 = '0;
        case (access_size)
            3'd1:    size_m1 = {{(PADDR_WIDTH-1){1'b0}}, 1'b1};
            3'd2:    size_m1 = {{(PADDR_WIDTH-2){1'b0}}, 2'b11};
            3'd3:    size_m1 = {{(PADDR_WIDTH-3){1'b0}}, 3'b111};
            default: size_m1 = '0;     
        endcase
    end

    assign addr_last = addr + size_m1;

    genvar i;
    logic pmp_allow;   // final allow before output assignment

    generate

        if (NUM_ENTRIES == 0) begin : gen_no_pmp
            assign pmp_allow = (priv_mode == PRIV_M);
        end else begin : gen_with_pmp

            // match_any[i] : entry i's region overlaps at least one byte of the access
            // match_all[i] : entry i's region covers every byte of the access
            // Per Section 3.7.1.3, the lowest-numbered entry with match_any=1 is the
            // one that decides the result. That entry must also have match_all=1, or
            // the access fails regardless of its L/R/W/X bits.
            logic match_any [NUM_ENTRIES];
            logic match_all [NUM_ENTRIES];
            logic perm_ok   [NUM_ENTRIES];   // entry permits this access type

            for (i = 0; i < NUM_ENTRIES; i++) begin : gen_entry
                // Config field extraction
                logic        lock_i;          // pmpcfg[7]   - Lock bit
                logic [1:0]  mode_i;          // pmpcfg[4:3] - Address-matching mode
                logic        x_i, w_i, r_i;  // pmpcfg[2:0] - Execute/Write/Read

                assign lock_i = pmpcfg[i][7];
                assign mode_i = pmpcfg[i][4:3];
                assign x_i   = pmpcfg[i][2];
                assign w_i   = pmpcfg[i][1];
                assign r_i   = pmpcfg[i][0];

                // Granularity-adjusted pmpaddr values
                //   TOR/OFF entries : force lower G bits to 0
                //   NAPOT entries   : force lower G-1 bits to 1
                logic [PA_BITS-1:0] pmpaddr_tor_eff;
                logic [PA_BITS-1:0] pmpaddr_napot_eff;

                assign pmpaddr_tor_eff   = pmpaddr[i] & ~GRAN_MASK_TOR;
                assign pmpaddr_napot_eff = pmpaddr[i] |  GRAN_MASK_NAPOT;

                // NAPOT mask and base
                // trailing-ones in pmpaddr encode region size
                logic [PA_BITS-1:0]     trailing_mask;
                logic [PADDR_WIDTH-1:0] napot_mask;
                logic [PADDR_WIDTH-1:0] napot_base;
                logic [PADDR_WIDTH-1:0] napot_hi;     // inclusive upper bound

                assign trailing_mask = pmpaddr_napot_eff ^ (pmpaddr_napot_eff + 1'b1);
                assign napot_mask    = {trailing_mask, 2'b11};
                assign napot_base    = {pmpaddr_napot_eff, 2'b00} & ~napot_mask;
                assign napot_hi      = napot_base | napot_mask;

                // NA4 region bounds: exact 4-byte aligned region.
                logic [PADDR_WIDTH-1:0] na4_base;
                logic [PADDR_WIDTH-1:0] na4_hi;        // inclusive upper bound

                assign na4_base = {pmpaddr[i], 2'b00};
                assign na4_hi   = {pmpaddr[i], 2'b11};

                // TOR bounds (half-open interval [tor_lo, tor_hi))
                //  Entry i matches addr y if  pmpaddr[i-1] ≤ y < pmpaddr[i].
                //   Entry 0 uses 0 as its lower bound.
                //   The lower bound uses pmpaddr[i-1] regardless of pmpcfg[i-1].A.
                logic [PADDR_WIDTH-1:0] tor_lo, tor_hi;
                logic tor_match_any;
                logic tor_match_all;

                assign tor_hi = {pmpaddr_tor_eff, 2'b00};
                if (i == 0) begin : gen_tor_lo_entry0
                assign tor_lo        = '0;
                assign tor_match_any = (addr < tor_hi);
                assign tor_match_all = (addr_last < tor_hi);
                end else begin : gen_tor_lo_entryN
                assign tor_lo = {(pmpaddr[i-1] & ~GRAN_MASK_TOR), 2'b00};
                assign tor_match_any = (addr < tor_hi) && (addr_last >= tor_lo);
                assign tor_match_all = (addr >= tor_lo) && (addr_last < tor_hi);
                end

                // Address match logic
                //   match_any : the entry's region overlaps any byte of [addr, addr_last]
                //   match_all : the entry's region covers every byte of [addr, addr_last]
                always_comb begin
                    case (mode_i)

                        A_OFF: begin
                            match_any[i] = 1'b0;
                            match_all[i] = 1'b0;
                        end

                        A_TOR: begin
                            // Half-open region [tor_lo, tor_hi). Empty when tor_lo >= tor_hi
                            // (e.g. pmpaddr[i-1] >= pmpaddr[i]); such an entry matches nothing.
                            if (tor_lo < tor_hi) begin
                                match_any[i] = tor_match_any;
                                match_all[i] = tor_match_all;
                            end else begin
                                match_any[i] = 1'b0;
                                match_all[i] = 1'b0;
                            end
                        end

                        A_NA4: begin
                            // NA4 is unavailable when G ≥ 1.
                            if (G >= 1) begin
                                match_any[i] = 1'b0;
                                match_all[i] = 1'b0;
                            end else begin
                                match_any[i] = (addr      <= na4_hi)   && (addr_last >= na4_base);
                                match_all[i] = (addr      >= na4_base) && (addr_last <= na4_hi);
                            end
                        end

                        A_NAPOT: begin
                            match_any[i] = (addr      <= napot_hi)   && (addr_last >= napot_base);
                            match_all[i] = (addr      >= napot_base) && (addr_last <= napot_hi);
                        end

                        default: begin
                            match_any[i] = 1'b0;
                            match_all[i] = 1'b0;
                        end
                    endcase
                end

                // Priority and Matching Logic
                //   If the L bit is clear and the privilege mode of the access is M, the access succeeds.
                //   Otherwise (L=1 OR S/U-mode): check R, W, X bits.
                //   R=0, W=1 is RESERVED : deny access
                always_comb begin
                    if (!lock_i && (priv_mode == PRIV_M)) begin
                        // Unlocked M-mode: unconditional pass (R/W/X ignored).
                        perm_ok[i] = 1'b1;

                    end else if (r_i == 1'b0 && w_i == 1'b1) begin
                        // R=0, W=1: reserved combination : deny.
                        perm_ok[i] = 1'b0;

                    end else begin
                        // Locked entry, or S/U-mode: check permission bits.
                        case (access_type)
                            ACC_FETCH: perm_ok[i] = x_i;   // instruction fetch : X bit
                            ACC_LOAD:  perm_ok[i] = r_i;   // load              : R bit
                            ACC_STORE: perm_ok[i] = w_i;   // store             : W bit
                            default:   perm_ok[i] = 1'b0;
                        endcase
                    end
                end

            end : gen_entry


            // The lowest-numbered PMP entry that matches any byte of an
            //   access determines whether that access succeeds or fails.
            //   If that entry does not match every byte of the access, the
            //   access fails irrespective of L/R/W/X (Section 3.7.1.3).
            logic any_match;
            logic sel_match_all;
            logic sel_perm_ok;

            always_comb begin
                any_match     = 1'b0;
                sel_match_all = 1'b0;
                sel_perm_ok   = 1'b0;
                for (int j = NUM_ENTRIES-1; j >= 0; j--) begin
                    if (match_any[j]) begin
                        any_match     = 1'b1;
                        sel_match_all = match_all[j];
                        sel_perm_ok   = perm_ok[j];
                    end
                end
            end


            //   No match, M    : allow (M-mode default when no entry covers the addr).
            //   No match, S/U  : deny.
            //   Match found    : succeeds only if the selected entry covers the whole
            //                    access AND its permission bits allow this access type.
            assign pmp_allow = any_match ? (sel_match_all && sel_perm_ok) : (priv_mode == PRIV_M);

        end : gen_with_pmp

    endgenerate

    assign allow = pmp_allow;
    assign fault = ~pmp_allow;

    //   instruction fetch fault : mcause = 1
    //   Load access fault       : mcause = 5
    //   store access fault  : mcause = 7
    always_comb begin
        if (fault) begin
            case (access_type)
                ACC_FETCH: fault_cause = CAUSE_INST_ACCESS_FAULT;    // 5'h01
                ACC_LOAD:  fault_cause = CAUSE_LOAD_ACCESS_FAULT;    // 5'h05
                ACC_STORE: fault_cause = CAUSE_STORE_ACCESS_FAULT;   // 5'h07
                default:   fault_cause = 5'h00;
            endcase
        end else begin
            fault_cause = 5'h00;
        end
    end

endmodule
