//======================================================================
// r-VEX | L1 data cache with MSI snooping (one per core)
//----------------------------------------------------------------------
// Direct-mapped, 4-word (16-byte) lines, write-back + write-allocate,
// blocking: one miss at a time and the core stalls (c_ready = 0) until the
// access hits. States: I, S, M (no E: a read miss always lands in S).
//
// Core port is combinational: a hit (load, or store to an M line) completes
// in the cycle it is presented, so the core's single-cycle packet timing
// is unchanged on hits. A miss goes out on the shared atomic bus
// (coh_bus.v) as one of
//   RD   : read miss            -> line in S
//   RDX  : write miss           -> line in M
//   UPGR : write hit on S line  -> M (other copies invalidated, no data)
//   WB   : evict a dirty victim (sent first, then the RD/RDX)
// The op is recomputed from the current line state every cycle until the
// bus grants it, so a line invalidated while waiting turns UPGR into RDX.
//
// Snooping: in the grant cycle of another cache's transaction, a matching
// line supplies its data if M (s_hit_m / s_line) and moves M->S on RD or
// to I on RDX/UPGR. A core access to a line being snooped in that cycle is
// held for one cycle, so a store cannot slip in after the flushed copy.
//
// DPF > 0 (2026-11-04 r11, contract prefetch.md DPF-01..08): stream
// prefetch for vector-style (unit-stride) data. A fully associative buffer
// of DPF lines sits beside the cache. A demand RD/RDX of line L whose line
// L-1 is among the last DPF_HIST demand misses allocates prefetches of
// L+1..L+DPF_DEG (FIFO replacement). A pending entry is fetched with a
// normal RD when the core has no demand miss and the line is not cached;
// the buffer holds it as clean shared data. A demand miss that finds its
// line in the buffer installs it in S in one cycle without the bus (victim
// clean, no RDX/UPGR of that line snooped in that cycle) and re-arms the
// entry with line+DPF_DEG. A snooped RDX/UPGR removes a buffered line.
// While the cache's own prefetch owns the bus, core hits still complete
// (no snoop can arrive then and the fill writes only the buffer).
// DPF = 0 is the r10 cache, cycle for cycle.
//======================================================================
module l1d_cache #(
    parameter AW   = 16,          // physical byte-address bits
    parameter SETS = 64,          // lines (power of two)
    parameter DPF      = 0,       // prefetch buffer lines (0 = no prefetch)
    parameter DPF_DEG  = 2,       // lines prefetched ahead of a stream
    parameter DPF_HIST = 4        // demand-miss history entries (allocation filter)
) (
    input  wire         clk,
    input  wire         reset,
    // core side
    input  wire         c_req,
    input  wire         c_we,
    input  wire [31:0]  c_addr,
    input  wire [31:0]  c_wdata,
    output wire [31:0]  c_rdata,
    output wire         c_ready,
    // bus master side
    output wire         b_req,
    output wire         b_pf,          // the presented request is a prefetch (r11, coh_bus demand first)
    output wire [1:0]   b_op,
    output wire [AW-1:0] b_addr,       // line address (low 4 bits 0)
    output wire [127:0] b_wline,       // victim data for WB
    input  wire         b_gnt,         // 1-cycle grant of the presented op
    input  wire         b_done,        // 1-cycle completion of the granted op
    input  wire [127:0] b_rline,
    // snoop side (another cache's granted transaction)
    input  wire         s_valid,
    input  wire [1:0]   s_op,
    input  wire [AW-1:0] s_addr,
    output wire         s_hit_m,
    output wire [127:0] s_line
);
    localparam OP_RD = 2'd0, OP_RDX = 2'd1, OP_UPGR = 2'd2, OP_WB = 2'd3;
    localparam ST_I = 2'd0, ST_S = 2'd1, ST_M = 2'd2;
    localparam IW = (SETS > 1) ? $clog2(SETS) : 1;
    // SETS must be a power of two >= 2 (a 1-line cache has no index bits)
    initial if (SETS < 2 || (SETS & (SETS - 1)) != 0) begin
        $display("FATAL %m: SETS=%0d must be a power of two >= 2", SETS); $finish;
    end
    localparam TW = AW - 4 - IW;
    localparam LA = AW - 4;                        // line address bits
    localparam NP = (DPF > 0) ? DPF : 1;
    localparam NH = (DPF_HIST > 0) ? DPF_HIST : 1;
    initial if (DPF > 0 && (DPF_DEG < 1 || DPF_HIST < 1 || DPF_DEG > DPF)) begin
        $display("FATAL %m: DPF=%0d needs 1 <= DPF_DEG (%0d) <= DPF and DPF_HIST (%0d) >= 1",
                 DPF, DPF_DEG, DPF_HIST); $finish;
    end

    reg [1:0]    st  [0:SETS-1];
    reg [TW-1:0] tg  [0:SETS-1];
    reg [31:0]   dat [0:SETS*4-1];

    // statistics
    reg [31:0] n_hit, n_miss, n_wb, n_upgr, n_inval, n_flush;
    reg [31:0] n_pf, n_pfhit, n_pfkill;
    reg [31:0] n_pfuseless, n_pflate;   // valid entry overwritten unused / demand miss waited for its prefetch

    // ---- prefetch buffer (DPF) ----
    reg          pv [0:NP-1];                      // line data valid
    reg          pp [0:NP-1];                      // allocated, RD not yet granted
    reg [LA-1:0] pa [0:NP-1];
    reg [127:0]  pd [0:NP-1];
    reg [LA-1:0] hist [0:NH-1];
    reg          hv   [0:NH-1];
    reg [$clog2(NP > 1 ? NP : 2)-1:0] rp;          // FIFO allocation pointer
    reg          cur_pf;                           // the granted transaction is a prefetch
    reg [$clog2(NP > 1 ? NP : 2)-1:0] cur_pb;

    // ---- core lookup ----
    wire [IW-1:0] c_idx = c_addr[4+IW-1:4];
    wire [TW-1:0] c_tag = c_addr[AW-1:4+IW];
    wire [1:0]    c_off = c_addr[3:2];
    wire          c_hit = (st[c_idx] != ST_I) && (tg[c_idx] == c_tag);
    wire          c_ok  = c_hit && (!c_we || st[c_idx] == ST_M);

    // ---- snoop lookup ----
    wire [IW-1:0] s_idx   = s_addr[4+IW-1:4];
    wire [TW-1:0] s_tag   = s_addr[AW-1:4+IW];
    wire          s_match = s_valid && (s_op != OP_WB) &&
                            (st[s_idx] != ST_I) && (tg[s_idx] == s_tag);
    assign s_hit_m = s_match && (st[s_idx] == ST_M);
    assign s_line  = {dat[s_idx*4+3], dat[s_idx*4+2], dat[s_idx*4+1], dat[s_idx*4+0]};
    wire   s_block = s_match && (s_idx == c_idx);    // hold the core this cycle
    wire [LA-1:0] c_line = c_addr[AW-1:4];
    wire [LA-1:0] s_la   = s_addr[AW-1:4];
    wire          s_kill = s_valid && (s_op == OP_RDX || s_op == OP_UPGR);

    // ---- miss engine ----
    reg        busy;                  // granted, waiting for b_done
    reg [1:0]  cur_op;
    reg [AW-1:0] cur_addr;

    // DPF-07: hits go on while the cache's own prefetch owns the bus
    assign c_ready = c_req && c_ok && (!busy || cur_pf) && !s_block;
    assign c_rdata = dat[c_idx*4 + c_off];

    wire need   = c_req && !c_ok && !busy;
    wire dirty_victim = !c_hit && (st[c_idx] == ST_M);

    // buffer lookup of the demand line, and the first pending entry
    reg          bh, pq;
    reg [$clog2(NP > 1 ? NP : 2)-1:0] bhi, pqi;
    // per-entry wires (constant indexes, so the always @(*) below sees them)
    wire [NP-1:0] e_hit, e_pend;
    wire [NH-1:0] e_seq;
    genvar gb;
    generate
        for (gb = 0; gb < NP; gb = gb + 1) begin : pe
            assign e_hit[gb]  = (DPF > 0) && pv[gb] && pa[gb] == c_line;
            assign e_pend[gb] = (DPF > 0) && pp[gb];
        end
        for (gb = 0; gb < NH; gb = gb + 1) begin : he
            assign e_seq[gb] = (DPF > 0) && hv[gb] && hist[gb] == c_line - 1'b1;
        end
    endgenerate
    integer b;
    always @(*) begin
        bh = 1'b0; bhi = 0; pq = 1'b0; pqi = 0;
        for (b = NP - 1; b >= 0; b = b - 1) begin
            if (e_hit[b])  begin bh = 1'b1; bhi = b; end
            if (e_pend[b]) begin pq = 1'b1; pqi = b; end
        end
    end
    wire [LA-1:0] p_line = pa[pqi];
    wire [IW-1:0] p_idx  = p_line[IW-1:0];
    wire [TW-1:0] p_tag  = p_line[LA-1:IW];
    wire          p_cached = (st[p_idx] != ST_I) && (tg[p_idx] == p_tag);
    // DPF-05: install from the buffer (no bus) when the victim is clean and no
    // RDX/UPGR of the line is granted to another cache this cycle
    wire buf_use = need && !c_hit && bh;                 // the miss is the buffer's
    wire inst    = buf_use && !dirty_victim && !(s_kill && s_la == c_line) && !s_block;
    wire dreq    = need && !(buf_use && !dirty_victim);  // demand transaction on the bus
    // DPF-07: a prefetch RD only when the core has no demand miss
    wire preq    = (DPF > 0) && !busy && !need && pq && !p_cached;
    wire pf_sel  = !dreq && preq;
    assign b_req  = dreq || preq;
    assign b_pf   = pf_sel;
    assign b_op   = pf_sel           ? OP_RD   :
                    (c_hit && c_we)  ? OP_UPGR :
                    dirty_victim     ? OP_WB   :
                    c_we             ? OP_RDX  : OP_RD;
    assign b_addr = pf_sel ? {p_line, 4'b0} :
                    (b_op == OP_WB) ? {tg[c_idx], c_idx, 4'b0} : {c_tag, c_idx, 4'b0};

    // DPF-06: allocation filter (line L-1 among the last DPF_HIST demand misses)
    wire seq = |e_seq;
    // is line x already cached or buffered (valid or pending)?
    function have;
        input [LA-1:0] x;
        integer q;
        begin
            have = (st[x[IW-1:0]] != ST_I) && (tg[x[IW-1:0]] == x[LA-1:IW]);
            for (q = 0; q < NP; q = q + 1) if ((pv[q] || pp[q]) && pa[q] == x) have = 1'b1;
        end
    endfunction
    assign b_wline = {dat[c_idx*4+3], dat[c_idx*4+2], dat[c_idx*4+1], dat[c_idx*4+0]};

    wire [IW-1:0] cur_idx = cur_addr[4+IW-1:4];
    wire [TW-1:0] cur_tag = cur_addr[AW-1:4+IW];

    integer k;
    always @(posedge clk) begin
        if (reset) begin
            for (k = 0; k < SETS; k = k + 1) begin st[k] <= ST_I; tg[k] <= {TW{1'b0}}; end
            busy <= 1'b0; cur_op <= OP_RD; cur_addr <= {AW{1'b0}};
            n_hit <= 0; n_miss <= 0; n_wb <= 0; n_upgr <= 0; n_inval <= 0; n_flush <= 0;
            n_pf <= 0; n_pfhit <= 0; n_pfkill <= 0; n_pfuseless <= 0; n_pflate <= 0; rp <= 0; cur_pf <= 1'b0; cur_pb <= 0;
            for (k = 0; k < NP; k = k + 1) begin pv[k] <= 1'b0; pp[k] <= 1'b0; pa[k] <= {LA{1'b0}}; end
            for (k = 0; k < NH; k = k + 1) begin hv[k] <= 1'b0; hist[k] <= {LA{1'b0}}; end
        end else begin
            // core hit: store data / count
            if (c_ready) begin
                n_hit <= n_hit + 1;
                if (c_we) dat[c_idx*4 + c_off] <= c_wdata;
            end
            // snoop state change (another cache's transaction)
            if (s_match) begin
                if (st[s_idx] == ST_M) n_flush <= n_flush + 1;
                if (s_op == OP_RD) st[s_idx] <= ST_S;
                else begin st[s_idx] <= ST_I; n_inval <= n_inval + 1; end
            end
            // DPF-04: a snooped RDX/UPGR removes the line from the buffer
            if (DPF > 0 && s_kill)
                for (k = 0; k < NP; k = k + 1)
                    if ((pv[k] || pp[k]) && pa[k] == s_la) begin
                        pv[k] <= 1'b0; pp[k] <= 1'b0; if (pv[k]) n_pfkill <= n_pfkill + 1;
                    end
            // DPF-05: install a buffered line in S, re-arm the entry with line + DEG
            if (DPF > 0 && inst) begin
                st[c_idx] <= ST_S; tg[c_idx] <= c_tag;
                dat[c_idx*4+0] <= pd[bhi][31:0];  dat[c_idx*4+1] <= pd[bhi][63:32];
                dat[c_idx*4+2] <= pd[bhi][95:64]; dat[c_idx*4+3] <= pd[bhi][127:96];
                n_pfhit <= n_pfhit + 1;
                pv[bhi] <= 1'b0; pa[bhi] <= c_line + DPF_DEG;
                pp[bhi] <= !have(c_line + DPF_DEG);
            end
            // a pending entry whose line got cached is dropped
            if (DPF > 0 && !busy && !need && pq && p_cached) pp[pqi] <= 1'b0;
            // own transaction
            if (b_gnt) begin
                busy <= 1'b1; cur_op <= b_op; cur_addr <= b_addr; cur_pf <= pf_sel; cur_pb <= pqi;
                if (pf_sel) begin pp[pqi] <= 1'b0; n_pf <= n_pf + 1; end
                else if (b_op == OP_WB) n_wb <= n_wb + 1;
                else if (b_op == OP_UPGR) n_upgr <= n_upgr + 1;
                else begin
                    n_miss <= n_miss + 1;
                    if (DPF > 0) begin : alloc
                        integer j, n;
                        // a pending entry for this line is no longer needed
                        for (k = 0; k < NP; k = k + 1) if (pp[k] && pa[k] == c_line) pp[k] <= 1'b0;
                        // DPF-06: sequential miss -> prefetch L+1..L+DEG
                        n = 0;
                        if (seq) for (j = 1; j <= DPF_DEG; j = j + 1)
                            if (!have(c_line + j)) begin
                                if (pv[(rp + n) % NP]) n_pfuseless <= n_pfuseless + 1;
                                pa[(rp + n) % NP] <= c_line + j; pp[(rp + n) % NP] <= 1'b1;
                                pv[(rp + n) % NP] <= 1'b0; n = n + 1;
                            end
                        rp <= (rp + n) % NP;
                        for (k = NH - 1; k > 0; k = k - 1) begin hist[k] <= hist[k-1]; hv[k] <= hv[k-1]; end
                        hist[0] <= c_line; hv[0] <= 1'b1;
                    end
                end
            end
            if (busy && b_done && cur_pf) begin
                busy <= 1'b0; cur_pf <= 1'b0;
                if (c_req && !c_ok && cur_addr[AW-1:4] == c_line) n_pflate <= n_pflate + 1;
                pd[cur_pb] <= b_rline; pv[cur_pb] <= 1'b1;
            end else if (busy && b_done) begin
                busy <= 1'b0;
                case (cur_op)
                    OP_WB  : st[cur_idx] <= ST_I;
                    OP_UPGR: st[cur_idx] <= ST_M;
                    default: begin
                        st[cur_idx] <= (cur_op == OP_RDX) ? ST_M : ST_S;
                        tg[cur_idx] <= cur_tag;
                        dat[cur_idx*4+0] <= b_rline[31:0];
                        dat[cur_idx*4+1] <= b_rline[63:32];
                        dat[cur_idx*4+2] <= b_rline[95:64];
                        dat[cur_idx*4+3] <= b_rline[127:96];
                    end
                endcase
            end
        end
    end

    // coherent peek for testbenches: 1 + data if this cache holds the word in M
    function [32:0] peek_m;
        input [AW-1:0] a;
        reg [IW-1:0] i; reg [TW-1:0] t;
        begin
            i = a[4+IW-1:4]; t = a[AW-1:4+IW];
            if (st[i] == ST_M && tg[i] == t) peek_m = {1'b1, dat[i*4 + a[3:2]]};
            else peek_m = 33'd0;
        end
    endfunction
endmodule
