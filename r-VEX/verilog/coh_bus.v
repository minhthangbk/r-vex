//======================================================================
// r-VEX | Atomic snooping bus between N L1 D-caches and the shared L2
//----------------------------------------------------------------------
// Round-robin arbiter. A granted transaction owns the bus until it
// completes, so transactions are serialised (this is what makes MSI on a
// blocking cache simple). In the grant cycle every other cache snoops the
// request; an M copy supplies the line (cache-to-cache) and that data is
// also written to L2, as a flush.
//   WB         : L2 write of the victim
//   UPGR       : no data; done the next cycle
//   RD/RDX + M : L2 write of the flushed line, requester gets it
//   RD/RDX     : L2 read
// FAST = 1 (2026-11-04 r12, reviews/contracts/bus.md BUS-03): completion
// (m_done, m_rline) is combinational in the cycle l2_done / the UPGR end is
// seen, and in the gap cycle after it only the master that just finished may
// be granted (its WB -> RD/RDX chain or a prefetch goes on; every other master
// is still kept out for one cycle). FAST = 0 is the r11 bus, cycle for cycle.
//======================================================================
module coh_bus #(
    parameter N  = 2,
    parameter AW = 16,
    parameter FAST = 0
) (
    input  wire              clk,
    input  wire              reset,
    // masters (packed, master i at [i])
    input  wire [N-1:0]      m_req,
    input  wire [N-1:0]      m_pf,        // request i is a prefetch (r11): granted only if no demand waits
    input  wire [2*N-1:0]    m_op,
    input  wire [AW*N-1:0]   m_addr,
    input  wire [128*N-1:0]  m_wline,
    output wire [N-1:0]      m_gnt,       // combinational, in the grant cycle
    output wire [N-1:0]      m_done,
    output wire [127:0]      m_rline,
    // snoop broadcast (combinational in the grant cycle)
    output wire [N-1:0]      s_valid,     // to cache i: snoop (not the requester)
    output wire [1:0]        s_op,
    output wire [AW-1:0]     s_addr,
    input  wire [N-1:0]      s_hit_m,
    input  wire [128*N-1:0]  s_line,
    // L2
    output reg               l2_req,
    output reg               l2_we,
    output reg  [AW-1:0]     l2_addr,
    output reg  [127:0]      l2_wline,
    input  wire              l2_done,
    input  wire [127:0]      l2_rline
);
    localparam OP_RD = 2'd0, OP_RDX = 2'd1, OP_UPGR = 2'd2, OP_WB = 2'd3;

    reg busy;
    reg [$clog2(N > 1 ? N : 2)-1:0] owner, last;
    reg [1:0]   op;
    reg         flushed;
    reg [127:0] flush_line;
    reg         upgr_wait;

    reg [31:0] n_txn, n_c2c;
    reg [31:0] n_busy, n_wait;      // r12 statistics: cycles owned, master-cycles waiting for a grant
    reg [N-1:0]  done_r;            // registered completion (FAST = 0)
    reg [127:0]  rline_r;
    // FAST: completion in the cycle the L2 (or the UPGR) finishes
    wire         fin = busy && (upgr_wait || l2_done);
    wire [N-1:0] done_c = fin ? ({{(N-1){1'b0}}, 1'b1} << owner) : {N{1'b0}};
    assign m_done  = FAST ? done_c : done_r;
    assign m_rline = FAST ? (flushed ? flush_line : l2_rline) : rline_r;

    // ---- arbitration (combinational): first requester after `last` ----
    reg        pick_v;
    reg [$clog2(N > 1 ? N : 2)-1:0] pick;
    integer i, j;
    // demand requests first (r11, prefetch.md DPF-09; TI C66x practice: a prefetch never wins over a
    // demand), round-robin inside each class
    wire [N-1:0] m_dem = m_req & ~m_pf;
    always @(*) begin
        pick_v = 1'b0; pick = 0;
        for (i = 1; i <= N; i = i + 1) begin
            j = (last + i) % N;
            if (!pick_v && m_dem[j]) begin pick_v = 1'b1; pick = j; end
        end
        for (i = 1; i <= N; i = i + 1) begin
            j = (last + i) % N;
            if (!pick_v && m_req[j]) begin pick_v = 1'b1; pick = j; end
        end
    end
    // Not in the cycle m_done is out: the requester installs its line at the
    // end of that cycle, so a transaction granted then would snoop the old
    // state and miss the new copy (found by tb_coherence: M in one L1, S in
    // the other).
    // Nor in the cycle after: that is when the requester's core retries and
    // hits. Without this gap another cache's RDX could invalidate the fresh
    // line before it is used, every time (livelock, also found by
    // tb_coherence). With it every fill is used at least once.
    reg  done_d;
    // BUS-03: FAST re-grants only the finishing master (`last`) in the gap cycle
    wire grant = !busy && !(|m_done) && (!done_d || (FAST && pick == last)) && pick_v;

    assign s_op   = m_op[2*pick +: 2];
    assign s_addr = m_addr[AW*pick +: AW];
    genvar g;
    generate for (g = 0; g < N; g = g + 1) begin : snp
        assign s_valid[g] = grant && (pick != g);
    end endgenerate

    reg         any_m;
    reg [127:0] m_line;
    integer q;
    always @(*) begin
        any_m = 1'b0; m_line = 128'd0;
        for (q = 0; q < N; q = q + 1)
            if (s_valid[q] && s_hit_m[q]) begin any_m = 1'b1; m_line = s_line[128*q +: 128]; end
    end

    integer w;
    reg [$clog2(N + 1)-1:0] nw;
    always @(*) begin
        nw = 0;
        for (w = 0; w < N; w = w + 1) if (m_req[w] && !m_gnt[w]) nw = nw + 1;
    end
    always @(posedge clk) begin
        done_r <= {N{1'b0}};
        done_d <= |m_done;
        if (reset) begin
            busy <= 1'b0; owner <= 0; last <= N - 1; op <= OP_RD; flushed <= 1'b0;
            l2_req <= 1'b0; l2_we <= 1'b0; upgr_wait <= 1'b0; n_txn <= 0; n_c2c <= 0;
            n_busy <= 0; n_wait <= 0; done_d <= 1'b0;
        end else begin
            if (busy) n_busy <= n_busy + 1;
            n_wait <= n_wait + nw;
        end
        if (reset) begin
        end else if (grant) begin
            busy <= 1'b1; owner <= pick; last <= pick; op <= s_op;
            n_txn <= n_txn + 1;
            flushed <= any_m; flush_line <= m_line;
            if (any_m) n_c2c <= n_c2c + 1;
            l2_addr <= s_addr;
            case (s_op)
                OP_WB:   begin l2_req <= 1'b1; l2_we <= 1'b1; l2_wline <= m_wline[128*pick +: 128]; end
                OP_UPGR: begin l2_req <= 1'b0; upgr_wait <= 1'b1; end
                default: begin
                    l2_req <= 1'b1; l2_we <= any_m; l2_wline <= m_line;
                end
            endcase
        end else if (busy) begin
            if (upgr_wait) begin
                upgr_wait <= 1'b0; busy <= 1'b0; done_r[owner] <= 1'b1;
            end else if (l2_done) begin
                l2_req <= 1'b0; busy <= 1'b0; done_r[owner] <= 1'b1;
                rline_r <= flushed ? flush_line : l2_rline;
            end
        end
    end

    // the requester sees its grant in the same cycle the bus latches the op
    generate for (g = 0; g < N; g = g + 1) begin : gn
        assign m_gnt[g] = grant && (pick == g);
    end endgenerate
endmodule
