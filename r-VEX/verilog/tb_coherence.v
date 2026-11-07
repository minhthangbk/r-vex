//======================================================================
// tb_coherence: constrained-random MSI test of l1d_cache x2 + coh_bus +
// l2_cache, no cores. Two traffic ports hammer 16 lines that collide in a
// 4-line L1 and an 8-line L2 (constant evictions, write-backs, upgrades,
// cache-to-cache flushes). Checked against a golden memory:
//   * every completed load returns the last completed store (SC order:
//     the bus is atomic and the caches block, so completion order is the
//     global order);
//   * SWMR, every cycle: a line valid in both L1s is S in both;
//   * data-value, every cycle: every valid L1 line equals golden;
//   * at the end, peek of every word equals golden.
//   * DPF > 0 (r11, prefetch.md DPF-03/04): every valid prefetch-buffer
//     line equals golden, every cycle; half of the requests walk to the next
//     word, so sequential line misses (the prefetch trigger) are common.
//   * r12 (bus.md): FAST/FWD/WBB/WBUF parameters; the write-back buffer line
//     equals golden and is in no array or prefetch buffer; GAP rule of the
//     bus (BUS-03). WBB also walks, so parked victims are touched again.
// +SEED=n +CYCLES=n [+SPARSE]
//======================================================================
`timescale 1ns/1ps
module tb_coherence;
    localparam AW = 16;
    // r18 (geofuzz.py, ppa.md PPA-10): geometry and latencies are parameters; the defaults are the r10..r17 bench
    parameter L1S = 4, L2S = 8, NW = 64;             // NW words = 16 lines
    parameter L2LAT = 3, MLAT = 7;
    localparam L1IW = (L1S > 1) ? $clog2(L1S) : 1;   // r18 Codex R36 F2: the checks index any L1S, not only 4 sets
    parameter DPF = 0, DPF_DEG = 2, DPF_HIST = 4;    // L1D prefetch (r11, prefetch.md DPF-03)
    parameter FAST = 0, FWD = 0, WBB = 0, WBUF = 0;  // r12 bus latency / write buffers (bus.md BUS-02)
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
        l1d_cache #(.AW(AW), .SETS(L1S), .DPF(DPF), .DPF_DEG(DPF_DEG), .DPF_HIST(DPF_HIST),
                    .FWD(FWD), .WBB(WBB)) l1d (
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
    l2_cache #(.AW(AW), .SETS(L2S), .L2_LAT(L2LAT), .MEM_LAT(MLAT), .FAST(FAST), .WBUF(WBUF)) l2 (
        .clk(clk), .reset(reset), .req(l2_req), .we(l2_we), .addr(l2_addr),
        .wline(l2_wline), .done(l2_done), .rline(l2_rline));

    reg [31:0] golden [0:NW-1];
    integer seed, seed0, ncyc, cyc, p, k, w, errors, loads, stores, i;
    integer ndchk = 0;     // r18 Codex R36 F2: every-cycle data checks that ran (0 = the check was skipped)
    integer wait_cnt [0:1];
    integer lastw [0:1];
    reg sparse;
    reg [32:0] pm0, pm1;

    // pool word k -> byte address 0x1000 + 4k: 16 consecutive lines, so 4
    // lines share every L1 index and 2 share every L2 index
    function [31:0] waddr; input integer kk; begin waddr = 32'h1000 + 4 * kk; end endfunction

    task check_invariants;
        integer s, ww, t, a;
        reg [15:0] la;
        begin
            for (s = 0; s < L1S; s = s + 1)
                if (c[0].l1d.st[s] != 0 && c[1].l1d.st[s] != 0 &&
                    c[0].l1d.tg[s] == c[1].l1d.tg[s] &&
                    (c[0].l1d.st[s] == 2 || c[1].l1d.st[s] == 2)) begin
                    $display("SWMR VIOLATION cyc=%0d set=%0d st0=%0d st1=%0d", cyc, s,
                             c[0].l1d.st[s], c[1].l1d.st[s]);
                    errors = errors + 1;
                end
            for (t = 0; t < 2; t = t + 1)
                for (s = 0; s < L1S; s = s + 1)
                    if ((t == 0 ? c[0].l1d.st[s] : c[1].l1d.st[s]) != 0)
                        for (ww = 0; ww < 4; ww = ww + 1) begin
                            la = ((t == 0 ? c[0].l1d.tg[s] : c[1].l1d.tg[s]) << (4 + L1IW)) + (s << 4) + 4 * ww;   // Codex R36 F2: any L1S
                            a = (la - 16'h1000) >> 2;
                            if (la >= 16'h1000 && a < NW) ndchk = ndchk + 1;
                            if (la >= 16'h1000 && a < NW &&
                                (t == 0 ? c[0].l1d.dat[s*4+ww] : c[1].l1d.dat[s*4+ww]) !== golden[a]) begin
                                $display("DATA VIOLATION cyc=%0d cache=%0d set=%0d word=%0d", cyc, t, s, ww);
                                errors = errors + 1;
                            end
                        end
            // DPF-10 (r11 research G7): a valid buffered line is not also in the same cache's array,
            // and no cache buffers a line the other cache holds in M
            for (t = 0; t < 2; t = t + 1)
                for (s = 0; s < DPF; s = s + 1)
                    if (t == 0 ? c[0].l1d.pv[s] : c[1].l1d.pv[s]) begin
                        la = {(t == 0 ? c[0].l1d.pa[s] : c[1].l1d.pa[s]), 4'b0};
                        if ((t == 0 ? c[0].l1d.st[((la >> 4) % L1S)] : c[1].l1d.st[((la >> 4) % L1S)]) != 0 &&
                            (t == 0 ? c[0].l1d.tg[((la >> 4) % L1S)] : c[1].l1d.tg[((la >> 4) % L1S)]) == (la >> (4 + L1IW))) begin
                            $display("PREFETCH EXCLUSION VIOLATION cyc=%0d cache=%0d entry=%0d", cyc, t, s);
                            errors = errors + 1;
                        end
                        if ((t == 0 ? c[1].l1d.st[((la >> 4) % L1S)] : c[0].l1d.st[((la >> 4) % L1S)]) == 2 &&
                            (t == 0 ? c[1].l1d.tg[((la >> 4) % L1S)] : c[0].l1d.tg[((la >> 4) % L1S)]) == (la >> (4 + L1IW))) begin
                            $display("PREFETCH SWMR VIOLATION cyc=%0d cache=%0d entry=%0d", cyc, t, s);
                            errors = errors + 1;
                        end
                    end
            // WBB-02 (r12, bus.md): the write-back buffer holds the owner copy: its data equals golden,
            // its line is in no array (neither cache) and in no prefetch buffer
            for (t = 0; t < 2; t = t + 1)
                if (WBB > 0 && (t == 0 ? c[0].l1d.wv : c[1].l1d.wv)) begin
                    la = {(t == 0 ? c[0].l1d.wa : c[1].l1d.wa), 4'b0};
                    for (ww = 0; ww < 4; ww = ww + 1) begin
                        a = ((la + 4 * ww) - 16'h1000) >> 2;
                        if (la >= 16'h1000 && a < NW &&
                            ((t == 0 ? c[0].l1d.wd : c[1].l1d.wd) >> (32 * ww)) % 33'h100000000 !== golden[a]) begin
                            $display("WBB DATA VIOLATION cyc=%0d cache=%0d word=%0d", cyc, t, ww);
                            errors = errors + 1;
                        end
                    end
                    if ((c[0].l1d.st[((la >> 4) % L1S)] != 0 && c[0].l1d.tg[((la >> 4) % L1S)] == (la >> (4 + L1IW))) ||
                        (c[1].l1d.st[((la >> 4) % L1S)] != 0 && c[1].l1d.tg[((la >> 4) % L1S)] == (la >> (4 + L1IW)))) begin
                        $display("WBB EXCLUSION VIOLATION cyc=%0d cache=%0d (line in an array)", cyc, t);
                        errors = errors + 1;
                    end
                    for (s = 0; s < DPF; s = s + 1)
                        if ((c[0].l1d.pv[s] && c[0].l1d.pa[s] == la[15:4]) ||
                            (c[1].l1d.pv[s] && c[1].l1d.pa[s] == la[15:4])) begin
                            $display("WBB EXCLUSION VIOLATION cyc=%0d cache=%0d (line in a prefetch buffer)", cyc, t);
                            errors = errors + 1;
                        end
                end
            if (WBB > 0 && c[0].l1d.wv && c[1].l1d.wv && c[0].l1d.wa == c[1].l1d.wa) begin
                $display("WBB SWMR VIOLATION cyc=%0d", cyc); errors = errors + 1;
            end
            // DPF-03: a valid buffered line is never stale
            for (t = 0; t < 2; t = t + 1)
                for (s = 0; s < DPF; s = s + 1)
                    if (t == 0 ? c[0].l1d.pv[s] : c[1].l1d.pv[s])
                        for (ww = 0; ww < 4; ww = ww + 1) begin
                            la = {(t == 0 ? c[0].l1d.pa[s] : c[1].l1d.pa[s]), 4'b0} + 4 * ww;
                            a = (la - 16'h1000) >> 2;
                            if (la >= 16'h1000 && a < NW &&
                                ((t == 0 ? c[0].l1d.pd[s] : c[1].l1d.pd[s]) >> (32 * ww)) % 33'h100000000 !== golden[a]) begin
                                $display("PREFETCH DATA VIOLATION cyc=%0d cache=%0d entry=%0d word=%0d", cyc, t, s, ww);
                                errors = errors + 1;
                            end
                        end
        end
    endtask

    initial begin
        if (!$value$plusargs("SEED=%d", seed)) seed = 1;
        seed0 = seed;
        if (!$value$plusargs("CYCLES=%d", ncyc)) ncyc = 100000;
        sparse = $test$plusargs("SPARSE");   // 1 request in 8 cycles: idle gaps for prefetches
        errors = 0; loads = 0; stores = 0;
        for (k = 0; k < (1 << (AW - 2)); k = k + 1) l2.mem[k] = 32'd0;
        for (k = 0; k < NW; k = k + 1) begin
            golden[k] = $random(seed);
            l2.mem[waddr(k) >> 2] = golden[k];
        end
        c_req = 0; c_we = 0; wait_cnt[0] = 0; wait_cnt[1] = 0; lastw[0] = 0; lastw[1] = 0;
        repeat (3) @(posedge clk);
        reset = 0;
        // after ncyc cycles no new request is presented and the run drains: the final check
        // must not see a transaction in flight (r11: a flush M->S granted in the last cycles
        // had not reached L2 yet, a false FINAL MISMATCH found with DPF_DEG = 1, seed 3)
        for (cyc = 0; cyc < ncyc || ((c_req != 0 || bus.busy || |b_done || |b_req) && cyc < ncyc + 5000); cyc = cyc + 1) begin
            // present / keep requests (a core holds its request until ready)
            for (p = 0; p < 2; p = p + 1)
                if (cyc < ncyc && !c_req[p] && (sparse ? ($random(seed) & 7) == 0 : ($random(seed) & 3) != 0)) begin
                    w = {$random(seed)} % NW;
                    if ((DPF > 0 || WBB > 0) && ($random(seed) & 1)) w = (lastw[p] + 1) % NW;   // walk
                    lastw[p] = w;
                    c_req[p] = 1; c_we[p] = $random(seed) & 1;
                    c_addr[p] = waddr(w); c_wdata[p] = $random(seed);
                    wait_cnt[p] = 0;
                end
            #1;
            // completions this cycle: check loads against golden first
            for (p = 0; p < 2; p = p + 1)
                if (c_ready[p] && !c_we[p]) begin
                    k = (c_addr[p] - 32'h1000) >> 2;
                    if (c_rdata[p] !== golden[k]) begin
                        $display("LOAD MISMATCH cyc=%0d port=%0d addr=%h got=%h exp=%h",
                                 cyc, p, c_addr[p], c_rdata[p], golden[k]);
                        errors = errors + 1;
                    end
                    loads = loads + 1;
                end
            @(posedge clk);
            #1;
            check_invariants;
            if (errors > 10) begin $display("too many errors"); $finish; end
        end
        // (a pending prefetch presents b_req, so the loop also waits for the prefetch buffers)
        if (c_req != 0 || bus.busy || |b_req) begin $display("DRAIN TIMEOUT"); errors = errors + 1; end
        // WBB-04 (r12): with no demand the write-back buffers drain
        if (WBB > 0 && (c[0].l1d.wv || c[1].l1d.wv)) begin $display("WBB NOT DRAINED"); errors = errors + 1; end
        // final: coherent view == golden
        for (k = 0; k < NW; k = k + 1) begin
            pm0 = c[0].l1d.peek_m(waddr(k));
            pm1 = c[1].l1d.peek_m(waddr(k));
            if ((pm0[32] && pm0[31:0] !== golden[k]) || (pm1[32] && pm1[31:0] !== golden[k]) ||
                (pm0[32] && pm1[32]) ||
                (!pm0[32] && !pm1[32] && l2.peek(waddr(k)) !== golden[k])) begin
                $display("FINAL MISMATCH word %0d", k); errors = errors + 1;
            end
        end
        $display("COHERENCE seed=%0d cycles=%0d loads=%0d stores=%0d bus_txn=%0d c2c=%0d l1miss=%0d/%0d upgr=%0d/%0d wb=%0d/%0d inval=%0d/%0d l2hit=%0d l2miss=%0d l2wb=%0d errors=%0d",
                 seed0, ncyc, loads, stores, bus.n_txn, bus.n_c2c,
                 c[0].l1d.n_miss, c[1].l1d.n_miss, c[0].l1d.n_upgr, c[1].l1d.n_upgr,
                 c[0].l1d.n_wb, c[1].l1d.n_wb, c[0].l1d.n_inval, c[1].l1d.n_inval,
                 l2.n_hit, l2.n_miss, l2.n_wb, errors);
        if (DPF > 0)
            $display("PREFETCH pf=%0d/%0d pfhit=%0d/%0d pfkill=%0d/%0d",
                     c[0].l1d.n_pf, c[1].l1d.n_pf, c[0].l1d.n_pfhit, c[1].l1d.n_pfhit,
                     c[0].l1d.n_pfkill, c[1].l1d.n_pfkill);
        $display("BUSR12 fast=%0d fwd=%0d wbb=%0d wbuf=%0d n_busy=%0d n_wait=%0d fwd=%0d/%0d wbb=%0d/%0d wbbsnp=%0d/%0d l2wbuf=%0d mwait=%0d",
                 FAST, FWD, WBB, WBUF, bus.n_busy, bus.n_wait, c[0].l1d.n_fwd, c[1].l1d.n_fwd,
                 c[0].l1d.n_wbb, c[1].l1d.n_wbb, c[0].l1d.n_wbbsnp, c[1].l1d.n_wbbsnp, l2.n_wbuf, l2.n_mwait);
        $display("DATACHECKS %0d", ndchk);
        if (ndchk == 0) begin $display("CHECK COVERAGE: no data check ran"); errors = errors + 1; end
        if (errors == 0) $display("RESULT: COHERENCE PASSED");
        else $display("RESULT: COHERENCE FAILED");
        $finish;
    end

    always @(posedge clk) if (!reset && $test$plusargs("TRACE"))
        $display("T%0d req=%b/%b we=%b%b a0=%h a1=%h rdy=%b breq=%b gnt=%b done=%b busy=%b op0=%0d op1=%0d st1[%0d]=%0d tg=%h busy1=%b",
                 cyc, c_req[0], c_req[1], c_we[0], c_we[1], c_addr[0], c_addr[1], c_ready, b_req, b_gnt, b_done,
                 bus.busy, b_op[1:0], b_op[3:2], c_addr[1][5:4], c[1].l1d.st[c_addr[1][5:4]], c[1].l1d.tg[c_addr[1][5:4]], c[1].l1d.busy);

    // BUS-03 (r12): no grant in a completion cycle; in the gap cycle after it only the master that
    // just finished (FAST) or none (FAST = 0)
    reg [1:0] done_q;
    always @(posedge clk) if (!reset) begin
        if (|b_gnt && |b_done) begin
            $display("GAP VIOLATION cyc=%0d grant in the done cycle gnt=%b done=%b", cyc, b_gnt, b_done);
            errors = errors + 1;
        end
        if (|b_gnt && |done_q && (FAST == 0 || (b_gnt & ~done_q) != 0)) begin
            $display("GAP VIOLATION cyc=%0d gnt=%b after done=%b", cyc, b_gnt, done_q);
            errors = errors + 1;
        end
        done_q <= b_done;
    end
    initial done_q = 0;

    // DPF-09 (r11): a prefetch is never granted while the other cache presents a demand request
    always @(posedge clk) if (!reset)
        if ((b_gnt[0] && b_pf[0] && b_req[1] && !b_pf[1]) || (b_gnt[1] && b_pf[1] && b_req[0] && !b_pf[0])) begin
            $display("PRIORITY VIOLATION cyc=%0d gnt=%b pf=%b req=%b", cyc, b_gnt, b_pf, b_req);
            errors = errors + 1;
        end

    // apply stores / retire requests on the clock edge (same edge the cache writes)
    always @(posedge clk) if (!reset) begin
        for (i = 0; i < 2; i = i + 1)
            if (c_ready[i]) begin
                if (c_we[i]) begin
                    golden[(c_addr[i] - 32'h1000) >> 2] <= c_wdata[i];
                    stores <= stores + 1;
                end
                c_req[i] <= 1'b0;
            end else if (c_req[i]) begin
                wait_cnt[i] <= wait_cnt[i] + 1;
                if (wait_cnt[i] > 2000) begin
                    $display("DEADLOCK port %0d addr %h", i, c_addr[i]); errors <= errors + 100;
                end
            end
    end
endmodule
