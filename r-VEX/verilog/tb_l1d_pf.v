//======================================================================
// tb_l1d_pf: directed test of the L1D stream prefetch buffer
// (reviews/contracts/prefetch.md DPF-03..07, round 2026-11-04-r11).
// 2 x l1d_cache (DPF = 4, DEG = 2 or 1, HIST = 4, 16 sets) + coh_bus + l2_cache.
//   seq    : core 0 reads lines 0..7 with idle gaps: 2 demand misses, then
//            every line comes from the buffer (DPF-06).
//   random : lines with no L-1 in the history: no prefetch at all (DPF-06).
//   hitpf  : hits complete with no stall while core 0's prefetch owns the
//            bus (DPF-07).
//   kill   : core 1 buffers lines 22/23, core 0 writes line 22: core 1's
//            copy is removed and its read returns the new value (DPF-04).
//   race   : core 1's install of a buffered line and core 0's RDX of that
//            line in the same cycle: the install is held (DPF-05), SWMR.
//   2 streams: interleaved streams keep separate entries (FIFO pointer, DPF-06).
//   victim : a buffered line whose set holds a dirty line: WB first, then
//            install (DPF-05); a store to a buffered line: install S, UPGR.
// Every load is checked against a golden memory.
//======================================================================
`timescale 1ns/1ps
module tb_l1d_pf;
    localparam AW = 16, L1S = 16, NW = 1024;
    parameter DEG = 2;                     // -Ptb_l1d_pf.DEG=1 runs the same cases with degree 1
    parameter STR = 0;                     // r23: -Ptb_l1d_pf.STR=1 = stride targets (prefetch.md DPF-11..13)
    parameter AGE = 0;                     // r23: -Ptb_l1d_pf.AGE=1 = oldest pending first (DPF-14, BUG-240)
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
        l1d_cache #(.AW(AW), .SETS(L1S), .DPF(4), .DPF_DEG(DEG), .DPF_HIST(4), .DPF_STR(STR), .DPF_AGE(AGE)) l1d (
            .clk(clk), .reset(reset),
            .c_req(c_req[g]), .c_we(c_we[g]), .c_addr(c_addr[g]), .c_wdata(c_wdata[g]),
            .c_rdata(c_rdata[g]), .c_ready(c_ready[g]),
            .b_req(b_req[g]), .b_pf(b_pf[g]), .b_op(b_op[2*g +: 2]), .b_addr(b_addr[AW*g +: AW]),
            .b_wline(b_wline[128*g +: 128]), .b_gnt(b_gnt[g]), .b_done(b_done[g]),
            .b_rline(b_rline), .s_valid(s_valid[g]), .s_op(s_op), .s_addr(s_addr),
            .s_hit_m(s_hit_m[g]), .s_line(s_line[128*g +: 128]));
    end endgenerate
    coh_bus #(.N(2), .AW(AW)) bus (
        .clk(clk), .reset(reset),
        .m_req(b_req), .m_pf(b_pf), .m_op(b_op), .m_addr(b_addr), .m_wline(b_wline),
        .m_gnt(b_gnt), .m_done(b_done), .m_rline(b_rline),
        .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr), .s_hit_m(s_hit_m), .s_line(s_line),
        .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wline(l2_wline),
        .l2_done(l2_done), .l2_rline(l2_rline));
    l2_cache #(.AW(AW), .SETS(64), .L2_LAT(3), .MEM_LAT(7)) l2 (
        .clk(clk), .reset(reset), .req(l2_req), .we(l2_we), .addr(l2_addr),
        .wline(l2_wline), .done(l2_done), .rline(l2_rline));

    reg [31:0] golden [0:NW-1];
    integer errors, k, stalls, cyc;
    function [31:0] la; input integer line, word; begin la = 32'h2000 + 16 * line + 4 * word; end endfunction

    // one access on port p, held until ready; returns its stall cycles
    task acc;
        input integer p, we, line, word; input [31:0] wd;
        integer w;
        begin
            c_req[p] = 1; c_we[p] = we; c_addr[p] = la(line, word); c_wdata[p] = wd;
            #1; w = 0;
            while (!c_ready[p]) begin @(posedge clk); #1 w = w + 1; end
            if (!we && c_rdata[p] !== golden[(c_addr[p] - 32'h2000) >> 2]) begin
                $display("FAIL load port %0d line %0d word %0d got %h exp %h", p, line, word,
                         c_rdata[p], golden[(c_addr[p] - 32'h2000) >> 2]);
                errors = errors + 1;
            end
            if (we) golden[(c_addr[p] - 32'h2000) >> 2] = wd;
            stalls = w;
            @(posedge clk); #1 c_req[p] = 0;
        end
    endtask
    task idle; input integer n; begin repeat (n) @(posedge clk); #1; end endtask
    task expect_eq;
        input [8*32-1:0] what; input integer got, exp;
        begin
            if (got !== exp) begin $display("FAIL %0s: got %0d expected %0d", what, got, exp); errors = errors + 1; end
            else $display("ok   %0s = %0d", what, got);
        end
    endtask
    // SWMR every cycle
    integer sw;
    always @(posedge clk) if (!reset) for (sw = 0; sw < L1S; sw = sw + 1)
        if (c[0].l1d.st[sw] != 0 && c[1].l1d.st[sw] != 0 && c[0].l1d.tg[sw] == c[1].l1d.tg[sw] &&
            (c[0].l1d.st[sw] == 2 || c[1].l1d.st[sw] == 2)) begin
            $display("FAIL SWMR set %0d st0=%0d st1=%0d", sw, c[0].l1d.st[sw], c[1].l1d.st[sw]);
            errors = errors + 1;
            if (errors > 20) begin $display("RESULT: L1D PREFETCH FAILED (too many)"); $finish; end
        end

    // r23 DPF-14 invariant (Codex R46 F2): with AGE = 1 the entry chosen for the next prefetch became pending no
    // later than any other pending entry (32-bit cycle stamps kept by the testbench: no wrap); every cycle, both cores
    integer cyc_n = 0, ps0 [0:3], ps1 [0:3], m;
    reg [3:0] opp0 = 0, opp1 = 0;
    reg [AW-5:0] opa0 [0:3], opa1 [0:3];
    always @(negedge clk) if (!reset) begin
        cyc_n = cyc_n + 1;
        for (m = 0; m < 4; m = m + 1) begin   // stamps first (values after this cycle's update), then the check
            if (c[0].l1d.pp[m] && (!opp0[m] || c[0].l1d.pa[m] != opa0[m])) ps0[m] = cyc_n;
            if (c[1].l1d.pp[m] && (!opp1[m] || c[1].l1d.pa[m] != opa1[m])) ps1[m] = cyc_n;
            opp0[m] = c[0].l1d.pp[m]; opa0[m] = c[0].l1d.pa[m]; opp1[m] = c[1].l1d.pp[m]; opa1[m] = c[1].l1d.pa[m];
        end
        if (AGE) for (m = 0; m < 4; m = m + 1) begin
            if (c[0].l1d.pq && c[0].l1d.pp[m] && ps0[m] < ps0[c[0].l1d.pqi]) begin
                $display("FAIL DPF-14 core 0: entry %0d (pending since %0d) is older than the chosen %0d (since %0d)",
                         m, ps0[m], c[0].l1d.pqi, ps0[c[0].l1d.pqi]); errors = errors + 1; end
            if (c[1].l1d.pq && c[1].l1d.pp[m] && ps1[m] < ps1[c[1].l1d.pqi]) begin
                $display("FAIL DPF-14 core 1: entry %0d older than the chosen %0d", m, c[1].l1d.pqi); errors = errors + 1; end
        end
    end
    integer pf0, hit0, miss0, kill1, hit1, wb0, up0, i, useless;
    initial begin
        errors = 0; c_req = 0; c_we = 0;
        for (k = 0; k < (1 << (AW - 2)); k = k + 1) l2.mem[k] = 32'd0;
        for (k = 0; k < NW; k = k + 1) begin golden[k] = 32'h1000_0000 + 3 * k; l2.mem[(32'h2000 >> 2) + k] = golden[k]; end
        repeat (3) @(posedge clk); #1 reset = 0;

        // ---- seq ----
        for (i = 0; i < 8; i = i + 1) begin acc(0, 0, i, 0, 0); idle(40); end
        expect_eq("seq demand misses", c[0].l1d.n_miss, 2);
        expect_eq("seq buffer hits", c[0].l1d.n_pfhit, 6);
        // ---- random (lines 40, 45, 42, 49, 54, 47, 51) ----
        pf0 = c[0].l1d.n_pf;
        acc(0, 0, 40, 1, 0); acc(0, 0, 45, 1, 0); acc(0, 0, 42, 1, 0); acc(0, 0, 49, 1, 0);
        acc(0, 0, 54, 1, 0); acc(0, 0, 47, 1, 0); acc(0, 0, 51, 1, 0); idle(40);
        expect_eq("random prefetches", c[0].l1d.n_pf - pf0, 0);
        // ---- hitpf: miss 60, 61 (allocates 62, 63), then hits on line 61 ----
        acc(0, 0, 60, 0, 0); acc(0, 0, 61, 0, 0);
        pf0 = c[0].l1d.n_pf; k = 0;
        for (i = 0; i < 30; i = i + 1) begin acc(0, 0, 61, i % 4, 0); k = k + stalls; end
        expect_eq("hitpf prefetches during hits", c[0].l1d.n_pf - pf0 > 0, 1);
        expect_eq("hitpf hit stalls", k, 0);
        idle(40);
        // ---- kill: core 1 reads 20, 21 -> buffers 22, 23 ----
        acc(1, 0, 20, 0, 0); acc(1, 0, 21, 0, 0); idle(60);
        expect_eq("kill buffered", c[1].l1d.pv[0] + c[1].l1d.pv[1] + c[1].l1d.pv[2] + c[1].l1d.pv[3] >= DEG, 1);
        kill1 = c[1].l1d.n_pfkill; hit1 = c[1].l1d.n_pfhit;
        acc(0, 1, 22, 2, 32'hdead_0022); idle(5);
        expect_eq("kill removed", c[1].l1d.n_pfkill - kill1, 1);
        acc(1, 0, 22, 2, 0);
        expect_eq("kill no buffer hit", c[1].l1d.n_pfhit - hit1, 0);
        idle(40);
        // ---- race: core 1 buffers 82/83 (last grant = core 1), then core 0 store
        //      miss on 82 and core 1 load of 82 in the same cycle ----
        acc(1, 0, 80, 0, 0); acc(1, 0, 81, 0, 0); idle(60);
        hit1 = c[1].l1d.n_pfhit;
        c_req = 2'b11; c_we = 2'b01;
        c_addr[0] = la(82, 1); c_wdata[0] = 32'hbeef_0082; c_addr[1] = la(82, 1);
        #1 expect_eq("race grant to core 0", b_gnt[0], 1);
        expect_eq("race install held", c[1].l1d.inst, 0);
        // let both finish (the store completes first: its RDX owns the bus)
        cyc = 0;
        while (c_req != 0 && cyc < 400) begin
            @(posedge clk);
            if (c_ready[0] && c_req[0]) begin golden[(la(82, 1) - 32'h2000) >> 2] = 32'hbeef_0082; c_req[0] <= 0; end
            if (c_ready[1] && c_req[1]) begin
                if (c_rdata[1] !== 32'hbeef_0082) begin
                    $display("FAIL race: core 1 read %h, expected the new value", c_rdata[1]); errors = errors + 1;
                end
                c_req[1] <= 0;
            end
            #1 cyc = cyc + 1;
        end
        expect_eq("race buffer hit", c[1].l1d.n_pfhit - hit1, 0);
        idle(40);
        // ---- victim: core 0 dirties line 102 (set 6), buffers 116/117 (sets 4, 5)...
        //      line 118 is set 6: buffered via 116, 117 misses, then read it ----
        acc(0, 1, 102, 0, 32'h0102_0102);
        acc(0, 0, 116, 0, 0); acc(0, 0, 117, 0, 0); idle(60);
        wb0 = c[0].l1d.n_wb; hit0 = c[0].l1d.n_pfhit;
        acc(0, 0, 118, 3, 0);
        expect_eq("victim write-back", c[0].l1d.n_wb - wb0, 1);
        expect_eq("victim buffer hit", c[0].l1d.n_pfhit - hit0, 1);
        acc(0, 0, 102, 0, 0);           // the dirty line went to L2 intact
        // store to a buffered line (119 was prefetched by the 117 miss)
        idle(60);
        up0 = c[0].l1d.n_upgr; hit0 = c[0].l1d.n_pfhit;
        acc(0, 1, 119, 2, 32'h0119_0119);
        expect_eq("store buffer hit", c[0].l1d.n_pfhit - hit0, 1);
        expect_eq("store upgrade", c[0].l1d.n_upgr - up0, 1);
        acc(0, 0, 119, 2, 0);
        acc(1, 0, 119, 2, 0);           // core 1 sees it through the bus flush

        // ---- two interleaved streams (DPF-06 FIFO replacement): A = lines 200.., B = lines 230..;
        //      each keeps its own entries, so both next lines come from the buffer ----
        idle(40);
        acc(0, 0, 200, 0, 0); acc(0, 0, 230, 0, 0); acc(0, 0, 201, 0, 0); acc(0, 0, 231, 0, 0); idle(80);
        hit0 = c[0].l1d.n_pfhit;
        acc(0, 0, 202, 0, 0); acc(0, 0, 232, 0, 0);
        expect_eq("two streams buffer hits", c[0].l1d.n_pfhit - hit0, 2);

        // ---- r23 (DPF-11..13, BUG-134): read-modify-write stream with a stride of 5 words (20 B: the stream
        //      skips one line in five). Stride targets: every prefetched line is used except the last DEG ones
        //      (beyond the end) and the r22 allocation at the start; next-line targets fetch the skipped lines ----
        idle(40);
        pf0 = c[0].l1d.n_pf; hit0 = c[0].l1d.n_pfhit; miss0 = c[0].l1d.n_miss;
        for (i = 0; i < 32; i = i + 1) begin
            acc(0, 0, 130 + (5 * i) / 4, (5 * i) % 4, 0);
            acc(0, 1, 130 + (5 * i) / 4, (5 * i) % 4, 32'h5000_0000 + i);
            idle(12);
        end
        idle(40);
        useless = (c[0].l1d.n_pf - pf0) - (c[0].l1d.n_pfhit - hit0);
        $display("     stride5: prefetches %0d, buffer hits %0d, demand misses %0d, useless %0d",
                 c[0].l1d.n_pf - pf0, c[0].l1d.n_pfhit - hit0, c[0].l1d.n_miss - miss0, useless);
        if (STR) begin
            expect_eq("stride5 useless <= DEG+1 (DPF-11)", useless <= DEG + 1, 1);
            expect_eq("stride5 demand misses <= 3 (DPF-12)", c[0].l1d.n_miss - miss0 <= 3, 1);
        end else
            expect_eq("stride5 next-line fetches skipped lines (r22)", useless >= 4, 1);
        // ---- r23 (DPF-13): alternating deltas (two fields of a 20-byte struct, +0 and +4): no confident stride,
        //      so the next-line rule holds: every line after the first two comes from the buffer ----
        idle(40);
        hit0 = c[0].l1d.n_pfhit; miss0 = c[0].l1d.n_miss;
        for (i = 0; i < 24; i = i + 1) begin
            acc(0, 0, 180 + (5 * i) / 4, (5 * i) % 4, 0);
            acc(0, 0, 180 + (5 * i + 1) / 4, (5 * i + 1) % 4, 0);
            idle(12);
        end
        $display("     struct20: buffer hits %0d, demand misses %0d", c[0].l1d.n_pfhit - hit0, c[0].l1d.n_miss - miss0);
        // DPF-14 (BUG-240): with DEG = 2 and lowest-index order the re-armed entry overtakes the older pending line
        if (AGE || DEG == 1) expect_eq("struct20 demand misses (DPF-13 next-line, DPF-14 order)", c[0].l1d.n_miss - miss0, 2);
        else expect_eq("struct20 lowest-index order misses lines (r22, BUG-240)", c[0].l1d.n_miss - miss0 > 2, 1);
        // ---- r23 (DPF-13): unit-stride word walk (delta 4 B < one line): no stride target, the next-line rule holds
        //      (DPF-M21: a 4-byte stride would re-arm the line in use and stop the stream) ----
        idle(40);
        hit0 = c[0].l1d.n_pfhit; miss0 = c[0].l1d.n_miss;
        for (i = 0; i < 64; i = i + 1) begin acc(0, 0, 212 + i / 4, i % 4, 0); idle(4); end
        $display("     words: buffer hits %0d, demand misses %0d", c[0].l1d.n_pfhit - hit0, c[0].l1d.n_miss - miss0);
        if (AGE || DEG == 1) expect_eq("word walk demand misses (DPF-13 next-line)", c[0].l1d.n_miss - miss0, 2);
        if (errors == 0) $display("RESULT: L1D PREFETCH PASSED");
        else $display("RESULT: L1D PREFETCH FAILED (%0d)", errors);
        $finish;
    end
endmodule
