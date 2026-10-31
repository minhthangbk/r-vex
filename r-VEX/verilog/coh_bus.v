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
//======================================================================
module coh_bus #(
    parameter N  = 2,
    parameter AW = 16
) (
    input  wire              clk,
    input  wire              reset,
    // masters (packed, master i at [i])
    input  wire [N-1:0]      m_req,
    input  wire [2*N-1:0]    m_op,
    input  wire [AW*N-1:0]   m_addr,
    input  wire [128*N-1:0]  m_wline,
    output wire [N-1:0]      m_gnt,       // combinational, in the grant cycle
    output reg  [N-1:0]      m_done,
    output reg  [127:0]      m_rline,
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

    // ---- arbitration (combinational): first requester after `last` ----
    reg        pick_v;
    reg [$clog2(N > 1 ? N : 2)-1:0] pick;
    integer i, j;
    always @(*) begin
        pick_v = 1'b0; pick = 0;
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
    wire grant = !busy && !(|m_done) && !done_d && pick_v;

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

    always @(posedge clk) begin
        m_done <= {N{1'b0}};
        done_d <= |m_done;
        if (reset) begin
            busy <= 1'b0; owner <= 0; last <= N - 1; op <= OP_RD; flushed <= 1'b0;
            l2_req <= 1'b0; l2_we <= 1'b0; upgr_wait <= 1'b0; n_txn <= 0; n_c2c <= 0;
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
                upgr_wait <= 1'b0; busy <= 1'b0; m_done[owner] <= 1'b1;
            end else if (l2_done) begin
                l2_req <= 1'b0; busy <= 1'b0; m_done[owner] <= 1'b1;
                m_rline <= flushed ? flush_line : l2_rline;
            end
        end
    end

    // the requester sees its grant in the same cycle the bus latches the op
    generate for (g = 0; g < N; g = g + 1) begin : gn
        assign m_gnt[g] = grant && (pick == g);
    end endgenerate
endmodule
