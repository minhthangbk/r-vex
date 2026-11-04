//======================================================================
// tb_bus_lat: directed timing test of the r12 bus-latency and write-buffer
// features (reviews/contracts/bus.md). One l1d_cache (port 0; port 1 idle) on
// coh_bus + l2_cache (4-line L1, 8-line L2, L2_LAT 4, MEM_LAT 20).
// Each access counts its stall cycles (cycles c_req is high and c_ready low)
// and is compared with the latency model of the contract:
//   L2 miss, clean victims  : MEM_LAT + L2_LAT + 4 - 2*FAST - FWD   (BUS-04/05)
//   L2 hit                  : L2_LAT + 4 - 2*FAST - FWD             (BUS-04)
//   UPGR                    : 3 - FAST - FWD                        (BUS-03, FWD-01)
//   dirty L1 victim, WBB    : no WB before the RD: = L2 miss        (WBB-01)
//   dirty L2 victim, WBUF   : = L2 miss with a clean victim          (L2WB-01)
//   L2 hit during the write behind: = L2 hit                         (L2WB-02)
//   L2 miss during the write behind: waits (> L2 miss, n_mwait > 0)  (L2WB-02)
// Data: every load is checked against the last store (FWD-01, WBB-02).
// DPF = 4 (with WBB) adds two prefetch scenarios (WBB-05): a stream allocates the
// line held in the write-back buffer (it must never be fetched stale from L2),
// and a prefetch-buffer hit whose victim is dirty parks it and installs at once.
// Run with -P tb_bus_lat.FAST=.. FWD=.. WBB=.. WBUF=.. DPF=..; prints RESULT.
//======================================================================
`timescale 1ns/1ps
module tb_bus_lat;
    parameter FAST = 0, FWD = 0, WBB = 0, WBUF = 0, DPF = 0;
    localparam AW = 16, L2_LAT = 4, MEM_LAT = 20;
    reg clk = 0, reset = 1;
    always #5 clk = ~clk;

    reg  [1:0]     c_req, c_we;
    reg  [31:0]    c_addr [0:1];
    reg  [31:0]    c_wdata [0:1];
    wire [31:0]    c_rdata [0:1];
    wire [1:0]     c_ready;
    wire [1:0]     b_req, b_pf, b_gnt, b_done, s_valid, s_hit_m;
    wire [3:0]     b_op;
    wire [2*AW-1:0] b_addr;
    wire [255:0]   b_wline, s_line;
    wire [127:0]   b_rline;
    wire [1:0]     s_op;
    wire [AW-1:0]  s_addr;
    wire           l2_req, l2_we, l2_done;
    wire [AW-1:0]  l2_addr;
    wire [127:0]   l2_wline, l2_rline;

    genvar g;
    generate for (g = 0; g < 2; g = g + 1) begin : c
        l1d_cache #(.AW(AW), .SETS(4), .FWD(FWD), .WBB(WBB), .DPF(DPF), .DPF_DEG(1), .DPF_HIST(4)) l1d (
            .clk(clk), .reset(reset),
            .c_req(c_req[g]), .c_we(c_we[g]), .c_addr(c_addr[g]), .c_wdata(c_wdata[g]),
            .c_rdata(c_rdata[g]), .c_ready(c_ready[g]),
            .b_req(b_req[g]), .b_pf(b_pf[g]), .b_op(b_op[2*g +: 2]), .b_addr(b_addr[AW*g +: AW]),
            .b_wline(b_wline[128*g +: 128]), .b_gnt(b_gnt[g]), .b_done(b_done[g]),
            .b_rline(b_rline), .s_valid(s_valid[g]), .s_op(s_op), .s_addr(s_addr),
            .s_hit_m(s_hit_m[g]), .s_line(s_line[128*g +: 128]));
    end endgenerate
    coh_bus #(.N(2), .AW(AW), .FAST(FAST)) bus (
        .clk(clk), .reset(reset),
        .m_req(b_req), .m_pf(b_pf), .m_op(b_op), .m_addr(b_addr), .m_wline(b_wline),
        .m_gnt(b_gnt), .m_done(b_done), .m_rline(b_rline),
        .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr), .s_hit_m(s_hit_m), .s_line(s_line),
        .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wline(l2_wline),
        .l2_done(l2_done), .l2_rline(l2_rline));
    l2_cache #(.AW(AW), .SETS(8), .L2_LAT(L2_LAT), .MEM_LAT(MEM_LAT), .FAST(FAST), .WBUF(WBUF)) l2 (
        .clk(clk), .reset(reset), .req(l2_req), .we(l2_we), .addr(l2_addr),
        .wline(l2_wline), .done(l2_done), .rline(l2_rline));

    integer errors = 0, stall, k;
    reg [31:0] golden [0:255];
    reg [31:0] got;
    // L1 index = addr[5:4] (4 sets), L2 index = addr[6:4] (8 sets)
    localparam [31:0] A = 32'h1000, B = 32'h1040, C = 32'h1080, D = 32'h10C0;
    function integer gi; input [31:0] a; begin gi = (a - 32'h1000) >> 2; end endfunction

    // one access of port 0: present at a clock edge, hold until ready, count stall cycles.
    // 3 idle cycles first, so every access is timed alone (with FAST = 0 the gap cycle after a
    // completion blocks every master, which a FWD access followed at once by a miss would pay)
    reg gap = 1;                     // 0: present at once (steps 7/8 run inside the write behind)
    task access; input we; input [31:0] a; input [31:0] d; output integer st;
        begin
            if (gap) begin repeat (3) @(posedge clk); #1; end
            c_req[0] = 1; c_we[0] = we; c_addr[0] = a; c_wdata[0] = d; st = 0;
            #1;
            while (!c_ready[0]) begin @(posedge clk); #1; st = st + 1; end
            if (!we && c_rdata[0] !== golden[gi(a)]) begin
                $display("DATA MISMATCH addr=%h got=%h exp=%h", a, c_rdata[0], golden[gi(a)]);
                errors = errors + 1;
            end
            @(posedge clk);
            if (we) golden[gi(a)] = d;
            c_req[0] = 0; #1;
        end
    endtask
    task chk; input [8*40-1:0] name; input integer got_st; input integer exp_st;
        begin
            $display("%0s stall=%0d expected=%0d", name, got_st, exp_st);
            if (got_st != exp_st) begin $display("TIMING MISMATCH %0s", name); errors = errors + 1; end
        end
    endtask
    task idle; input integer n; begin repeat (n) @(posedge clk); #1; end endtask

    integer s_miss, s_hit, s_upgr, s4, s6, s7, s8, mw0;
    initial begin
        for (k = 0; k < (1 << (AW - 2)); k = k + 1) l2.mem[k] = 32'd0;
        for (k = 0; k < 256; k = k + 1) begin golden[k] = 32'hA5000000 + k; l2.mem[(32'h1000 >> 2) + k] = golden[k]; end
        c_req = 0; c_we = 0; c_addr[0] = 0; c_addr[1] = 0; c_wdata[0] = 0; c_wdata[1] = 0;
        repeat (3) @(posedge clk); #1;
        reset = 0;
        s_miss = MEM_LAT + L2_LAT + 4 - 2 * FAST - FWD;
        s_hit  = L2_LAT + 4 - 2 * FAST - FWD;
        s_upgr = 3 - FAST - FWD;
        access(0, A, 0, stall);         chk("1 L2 miss (clean)", stall, s_miss);
        access(0, A + 4, 0, stall);     chk("2 L1 hit", stall, 0);
        access(1, A, 32'h11111111, stall); chk("3 UPGR", stall, s_upgr);
        // 4: B evicts dirty A from the L1 (B misses L2, L2 victims clean)
        access(0, B, 0, stall); s4 = stall;
        if (WBB) chk("4 dirty L1 victim (WBB)", stall, s_miss);
        else begin
            $display("4 dirty L1 victim (WB first) stall=%0d (> %0d)", stall, s_miss);
            if (stall <= s_miss) begin $display("TIMING MISMATCH 4"); errors = errors + 1; end
        end
        idle(40);                       // background WB of A (WBB) finishes
        if (WBB && c[0].l1d.wv) begin $display("WBB not drained"); errors = errors + 1; end
        access(0, A, 0, stall);         chk("5 L2 hit", stall, s_hit);
        // 6: C misses L1 (victim A clean) and L2 (victim A dirty in L2 index 0)
        access(0, C, 0, stall); s6 = stall;
        chk("6 dirty L2 victim", stall, WBUF ? s_miss : s_miss + MEM_LAT);
        // 7: B hits L2 while A is written behind; 8: D misses L2 and must wait for the write
        mw0 = l2.n_mwait; gap = 0;
        // (FWD without FAST: the access right after a forwarded completion meets the gap cycle, +1)
        access(0, B, 0, stall); s7 = stall; chk("7 L2 hit in write-behind", stall, s_hit + (FWD && !FAST));
        access(0, D, 0, stall); s8 = stall;
        if (WBUF) begin
            $display("8 L2 miss in write-behind stall=%0d (> %0d) mwait=%0d", stall, s_miss, l2.n_mwait - mw0);
            if (stall <= s_miss || l2.n_mwait == mw0) begin $display("TIMING MISMATCH 8"); errors = errors + 1; end
        end else chk("8 L2 miss", stall, s_miss + (FWD && !FAST));
        // data after all the evictions (A was written to memory through the buffers)
        gap = 1; idle(60);
        access(0, A, 0, stall);
        access(0, A + 8, 0, stall);
        if (DPF > 0 && WBB > 0) begin
            // WBB-05 a: V (line 0x112) dirty, parked by 0x116; misses 0x110, 0x111 (back to back, so the
            // buffer cannot drain) make a stream that allocates 0x112 = the buffered line
            access(1, 32'h1120, 32'h5555AAAA, stall);
            gap = 0;
            access(0, 32'h1160, 0, stall);
            access(0, 32'h1100, 0, stall);
            access(0, 32'h1110, 0, stall);
            gap = 1; idle(60);
            access(0, 32'h1120, 0, stall);          // must read 5555AAAA, not the L2 copy
            // WBB-05 b: W (0x126) dirty; stream 0x120, 0x121 prefetches 0x122 (same set as W);
            // the buffer hit parks W and installs 0x122 without a bus transaction: stall 1
            access(1, 32'h1260, 32'h0BADF00D, stall);
            access(0, 32'h1200, 0, stall);
            access(0, 32'h1210, 0, stall);
            idle(60);
            access(0, 32'h1220, 0, stall); chk("9 prefetch hit, dirty victim", stall, 1);
            idle(60);
            access(0, 32'h1260, 0, stall);          // W back through the write-back buffer
        end
        $display("BUSLAT fast=%0d fwd=%0d wbb=%0d wbuf=%0d miss=%0d hit=%0d upgr=%0d dirtyL1=%0d dirtyL2=%0d errors=%0d",
                 FAST, FWD, WBB, WBUF, s_miss, s_hit, s_upgr, s4, s6, errors);
        if (errors == 0) $display("RESULT: BUSLAT PASSED"); else $display("RESULT: BUSLAT FAILED");
        $finish;
    end
    initial begin #2000000; $display("TIMEOUT"); $display("RESULT: BUSLAT FAILED"); $finish; end
endmodule
