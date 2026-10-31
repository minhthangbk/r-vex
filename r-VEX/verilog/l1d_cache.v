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
//======================================================================
module l1d_cache #(
    parameter AW   = 16,          // physical byte-address bits
    parameter SETS = 64           // lines (power of two)
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
    localparam IW = $clog2(SETS);
    localparam TW = AW - 4 - IW;

    reg [1:0]    st  [0:SETS-1];
    reg [TW-1:0] tg  [0:SETS-1];
    reg [31:0]   dat [0:SETS*4-1];

    // statistics
    reg [31:0] n_hit, n_miss, n_wb, n_upgr, n_inval, n_flush;

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

    // ---- miss engine ----
    reg        busy;                  // granted, waiting for b_done
    reg [1:0]  cur_op;
    reg [AW-1:0] cur_addr;

    assign c_ready = c_req && c_ok && !busy && !s_block;
    assign c_rdata = dat[c_idx*4 + c_off];

    wire need   = c_req && !c_ok && !busy;
    wire dirty_victim = !c_hit && (st[c_idx] == ST_M);
    assign b_req  = need;
    assign b_op   = (c_hit && c_we)  ? OP_UPGR :
                    dirty_victim     ? OP_WB   :
                    c_we             ? OP_RDX  : OP_RD;
    assign b_addr = (b_op == OP_WB) ? {tg[c_idx], c_idx, 4'b0} : {c_tag, c_idx, 4'b0};
    assign b_wline = {dat[c_idx*4+3], dat[c_idx*4+2], dat[c_idx*4+1], dat[c_idx*4+0]};

    wire [IW-1:0] cur_idx = cur_addr[4+IW-1:4];
    wire [TW-1:0] cur_tag = cur_addr[AW-1:4+IW];

    integer k;
    always @(posedge clk) begin
        if (reset) begin
            for (k = 0; k < SETS; k = k + 1) begin st[k] <= ST_I; tg[k] <= {TW{1'b0}}; end
            busy <= 1'b0; cur_op <= OP_RD; cur_addr <= {AW{1'b0}};
            n_hit <= 0; n_miss <= 0; n_wb <= 0; n_upgr <= 0; n_inval <= 0; n_flush <= 0;
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
            // own transaction
            if (b_gnt) begin
                busy <= 1'b1; cur_op <= b_op; cur_addr <= b_addr;
                if (b_op == OP_WB) n_wb <= n_wb + 1;
                else if (b_op == OP_UPGR) n_upgr <= n_upgr + 1;
                else n_miss <= n_miss + 1;
            end
            if (busy && b_done) begin
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
