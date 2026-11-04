//======================================================================
// tb_l1i_pf: directed test of the L1I next-line prefetch buffer
// (reviews/contracts/prefetch.md IPF-01..06, round 2026-11-04-r11).
// Two caches see the same fetch trace, one with IPF = 0 (r10) and one with
// IPF = 1; each has its own fetch driver (a core that advances its trace on
// i_ready). Every delivered packet is compared with the ROM (IPF-02).
//   seq   : 8 lines straight-line from packet 0. IPF = 0: every line stalls
//           FILL_LAT+1. IPF = 1: line 0 stalls FILL_LAT+1, every later line
//           stalls FILL_LAT-3 (prefetch started in the first cycle of the
//           previous line; the miss waits for it: IPF-03, IPF-04).
//   branch: 2 packets of line 0, then a jump to line 10: the target miss
//           aborts the prefetch of line 1 and stalls FILL_LAT+1 as with
//           IPF = 0 (IPF-05).
//   loop  : lines 0..3 five times: no miss after the first pass, and line 4
//           (prefetched at the end of every pass) is never in the cache
//           (IPF-06).
//======================================================================
`timescale 1ns/1ps
module tb_l1i_pf;
    localparam PC_W = 12, SETS = 16, FL = 8, NT = 128;
    reg clk = 0, reset = 1;
    always #5 clk = ~clk;

    reg  [PC_W-1:0] trace [0:NT-1];
    integer ntr, errors;
    integer pos [0:1], stall [0:1], lstall [0:1][0:15];
    reg  [1:0] run;
    wire [127:0] rd [0:1];
    wire [1:0]   rdy;
    wire [PC_W-1:0] pc0 = trace[pos[0]], pc1 = trace[pos[1]];

    l1i_cache #(.PC_W(PC_W), .SETS(SETS), .FILL_LAT(FL), .IPF(0)) c0 (
        .clk(clk), .reset(reset), .i_req(run[0]), .i_addr(pc0), .i_rdata(rd[0]), .i_ready(rdy[0]));
    l1i_cache #(.PC_W(PC_W), .SETS(SETS), .FILL_LAT(FL), .IPF(1)) c1 (
        .clk(clk), .reset(reset), .i_req(run[1]), .i_addr(pc1), .i_rdata(rd[1]), .i_ready(rdy[1]));

    integer k, t;
    // fetch drivers: advance on i_ready, count stalls per line of the trace
    always @(posedge clk) if (!reset) begin
        for (t = 0; t < 2; t = t + 1) if (run[t]) begin
            if ((t == 0 ? rdy[0] : rdy[1])) begin
                if ((t == 0 ? rd[0] : rd[1]) !== (t == 0 ? c0.rom[pc0] : c1.rom[pc1])) begin
                    $display("FAIL IPF-02 cache %0d pc %0d: wrong packet", t, trace[pos[t]]);
                    errors = errors + 1;
                end
                if (pos[t] + 1 >= ntr) run[t] <= 1'b0;
                pos[t] <= pos[t] + 1;
            end else begin
                stall[t] <= stall[t] + 1;
                lstall[t][trace[pos[t]] >> 2] <= lstall[t][trace[pos[t]] >> 2] + 1;
            end
        end
    end

    task go;      // reset both caches, run the trace to the end on both
        integer w;
        begin
            reset = 1; run = 2'b00; pos[0] = 0; pos[1] = 0; stall[0] = 0; stall[1] = 0;
            for (k = 0; k < 16; k = k + 1) begin lstall[0][k] = 0; lstall[1][k] = 0; end
            repeat (2) @(posedge clk);
            #1 reset = 0; run = 2'b11;
            w = 0;
            while (run != 2'b00 && w < 5000) begin @(posedge clk); #1 w = w + 1; end
            if (run != 2'b00) begin $display("FAIL timeout"); errors = errors + 1; end
        end
    endtask

    task expect_eq;
        input [8*32-1:0] what; input integer got, exp;
        begin
            if (got !== exp) begin $display("FAIL %0s: got %0d expected %0d", what, got, exp); errors = errors + 1; end
            else $display("ok   %0s = %0d", what, got);
        end
    endtask

    initial begin
        errors = 0;
        for (k = 0; k < (1 << PC_W); k = k + 1) begin
            c0.rom[k] = {4{k[31:0] ^ 32'h5a5a0000}}; c1.rom[k] = {4{k[31:0] ^ 32'h5a5a0000}};
        end
        // ---- seq: 32 packets straight-line ----
        ntr = 32; for (k = 0; k < ntr; k = k + 1) trace[k] = k;
        go;
        expect_eq("seq IPF=0 stall", stall[0], 8 * (FL + 1));
        expect_eq("seq IPF=1 line0 stall", lstall[1][0], FL + 1);
        for (k = 1; k < 8; k = k + 1) expect_eq("seq IPF=1 later line", lstall[1][k], FL - 3);
        expect_eq("seq IPF=1 pfhit", c1.n_pfhit, 7);
        // ---- branch: 0,1 then 40..43 (line 10) ----
        ntr = 6; trace[0] = 0; trace[1] = 1; for (k = 0; k < 4; k = k + 1) trace[2 + k] = 40 + k;
        go;
        expect_eq("branch IPF=0 target stall", lstall[0][10], FL + 1);
        expect_eq("branch IPF=1 target stall", lstall[1][10], FL + 1);
        expect_eq("branch IPF=1 abort", c1.n_pfabort, 1);
        // ---- loop: lines 0..3 five times ----
        ntr = 0; for (t = 0; t < 5; t = t + 1) for (k = 0; k < 16; k = k + 1) begin trace[ntr] = k; ntr = ntr + 1; end
        go;
        expect_eq("loop IPF=0 misses", c0.n_miss, 4);
        expect_eq("loop IPF=1 fills", c1.n_miss + c1.n_pfhit, 4);
        expect_eq("loop IPF=1 line4 cached", c1.vld[4], 0);
        if (errors == 0) $display("RESULT: L1I PREFETCH PASSED");
        else $display("RESULT: L1I PREFETCH FAILED (%0d)", errors);
        $finish;
    end
endmodule
