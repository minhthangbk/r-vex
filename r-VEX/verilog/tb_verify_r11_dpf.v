//======================================================================
// tb_verify_r11_dpf: final verify r11, L1D stream prefetch corners inside
// MSI (reviews/contracts/prefetch.md DPF-03..07, DPF-10) that tb_l1d_pf,
// tb_codex_r22_dpf_edges and tb_coherence do not drive deliberately.
// 2 x l1d_cache (DPF = 4, DEG, HIST = 4, L1S sets) + coh_bus + l2_cache.
//   K1 rearm-vs-UPGR : core 1 installs buffered line 52 (re-arming 53) in the
//      same cycle core 0's UPGR of 53 is granted. The new pending entry for 53
//      survives; its prefetch RD, granted after the UPGR is done, must take the
//      flushed M data (core 0's store), not the L2 copy.
//   K2 held install  : core 1 misses on buffered line X whose victim V is
//      dirty (WB presented) while core 0's RD (K2a) / RDX (K2b) of V is
//      granted: the install is held (s_block), V is flushed by the snoop, and
//      the retry installs X with no write-back (V clean / invalid now).
//   K3 pending/in-flight + store: core 1 stores to its pending line before the
//      prefetch is granted (K3a: demand RDX, pending dropped, no prefetch) and
//      while it is in flight (K3b: waits, n_pflate, install S, UPGR).
//   K4 snooped RD    : core 0 reads a line core 1 buffers: no kill (DPF-04).
//   K5 useless       : FIFO reallocation of unused valid entries (n_pfuseless).
//   K6 two streams, one writer: core 0 streams stores, core 1 streams loads
//      over the same 32 lines, random gaps (+SEED), SC load check.
// Every cycle: SWMR, L1 data == golden, valid buffer data == golden,
// EXCLUSION (buffer vs own array), buffer vs M in the other cache.
// -Ptb_verify_r11_dpf.L1S=4 / .DEG=2 run the same cases on other shapes
// (K1-K5 expectations hold for L1S >= 16; with L1S < 16 only K6 + invariants).
//======================================================================
`timescale 1ns/1ps
module tb_verify_r11_dpf;
    parameter L1S = 16, DEG = 1;
    localparam AW = 16, NW = 2048, IW = $clog2(L1S);
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
        l1d_cache #(.AW(AW), .SETS(L1S), .DPF(4), .DPF_DEG(DEG), .DPF_HIST(4)) l1d (
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

    localparam [31:0] BASE = 32'h2000;
    reg [31:0] golden [0:NW-1];
    integer errors, k, stalls, cyc, seed;
    function [31:0] la; input integer line, word; begin la = BASE + 16 * line + 4 * word; end endfunction
    function integer gi; input [31:0] a; begin gi = (a - BASE) >> 2; end endfunction

    // golden memory: a store is applied on the edge it completes (the cache writes on the same edge)
    always @(posedge clk) if (!reset) begin : gold
        integer p;
        for (p = 0; p < 2; p = p + 1)
            if (c_req[p] && c_ready[p] && c_we[p]) golden[gi(c_addr[p])] <= c_wdata[p];
    end
    // loads: checked in the cycle they complete (SC: atomic bus, blocking caches)
    always @(negedge clk) if (!reset) begin : ldchk
        integer p;
        for (p = 0; p < 2; p = p + 1)
            if (c_req[p] && c_ready[p] && !c_we[p] && c_rdata[p] !== golden[gi(c_addr[p])]) begin
                $display("FAIL load port %0d addr %h got %h exp %h (t=%0t)", p, c_addr[p], c_rdata[p],
                         golden[gi(c_addr[p])], $time);
                errors = errors + 1;
            end
    end

    // ---- invariants, every cycle (after the edge's updates) ----
    `define INV_CACHE(T, O) \
        for (s = 0; s < L1S; s = s + 1) if (c[T].l1d.st[s] != 0) \
            for (w = 0; w < 4; w = w + 1) begin \
                a = c[T].l1d.tg[s] * (L1S * 16) + s * 16 + 4 * w; \
                if (a >= BASE && gi(a) < NW && c[T].l1d.dat[s*4+w] !== golden[gi(a)]) begin \
                    $display("FAIL DATA cache %0d set %0d word %0d t=%0t", T, s, w, $time); errors = errors + 1; end \
            end \
        for (e = 0; e < 4; e = e + 1) if (c[T].l1d.pv[e]) begin \
            a = c[T].l1d.pa[e] * 16; s = c[T].l1d.pa[e] % L1S; \
            if (c[T].l1d.st[s] != 0 && c[T].l1d.tg[s] == c[T].l1d.pa[e] / L1S) begin \
                $display("FAIL EXCLUSION cache %0d entry %0d line %h t=%0t", T, e, c[T].l1d.pa[e], $time); errors = errors + 1; end \
            if (c[O].l1d.st[s] == 2 && c[O].l1d.tg[s] == c[T].l1d.pa[e] / L1S) begin \
                $display("FAIL BUFFER-vs-M cache %0d entry %0d line %h t=%0t", T, e, c[T].l1d.pa[e], $time); errors = errors + 1; end \
            for (w = 0; w < 4; w = w + 1) \
                if (a + 4 * w >= BASE && gi(a + 4 * w) < NW && \
                    ((c[T].l1d.pd[e] >> (32 * w)) & 128'hffffffff) !== golden[gi(a + 4 * w)]) begin \
                    $display("FAIL BUFFER DATA cache %0d entry %0d word %0d line %h t=%0t", T, e, w, c[T].l1d.pa[e], $time); \
                    errors = errors + 1; end \
        end
    always @(negedge clk) if (!reset) begin : inv
        integer s, w, e, a;
        for (s = 0; s < L1S; s = s + 1)
            if (c[0].l1d.st[s] != 0 && c[1].l1d.st[s] != 0 && c[0].l1d.tg[s] == c[1].l1d.tg[s] &&
                (c[0].l1d.st[s] == 2 || c[1].l1d.st[s] == 2)) begin
                $display("FAIL SWMR set %0d st0=%0d st1=%0d t=%0t", s, c[0].l1d.st[s], c[1].l1d.st[s], $time);
                errors = errors + 1;
            end
        `INV_CACHE(0, 1)
        `INV_CACHE(1, 0)
        if (errors > 30) begin $display("RESULT: VERIFY R11 DPF FAILED (too many)"); $finish; end
    end

    // one access on port p, held until ready; returns its stall cycles
    task acc;
        input integer p, we, line, word; input [31:0] wd;
        integer w;
        begin
            c_req[p] = 1; c_we[p] = we; c_addr[p] = la(line, word); c_wdata[p] = wd;
            #1; w = 0;
            while (!c_ready[p]) begin @(posedge clk); #1 w = w + 1; end
            stalls = w;
            @(posedge clk); #1 c_req[p] = 0;
        end
    endtask
    task idle; input integer n; begin repeat (n) @(posedge clk); #1; end endtask
    task expect_eq;
        input [8*40-1:0] what; input integer got, exp;
        begin
            if (got !== exp) begin $display("FAIL %0s: got %0d expected %0d", what, got, exp); errors = errors + 1; end
            else $display("ok   %0s = %0d", what, got);
        end
    endtask
    // both ports finish their presented requests
    task drain2;
        integer n;
        begin
            n = 0;
            while (c_req != 0 && n < 1000) begin
                @(posedge clk);
                if (c_req[0] && c_ready[0]) c_req[0] <= 0;
                if (c_req[1] && c_ready[1]) c_req[1] <= 0;
                #1 n = n + 1;
            end
            if (c_req != 0) begin $display("FAIL drain timeout"); errors = errors + 1; c_req = 0; end
        end
    endtask
    function integer pend_of; input integer t, line; integer e; begin
        pend_of = 0;
        for (e = 0; e < 4; e = e + 1)
            if ((t == 0 ? c[0].l1d.pp[e] : c[1].l1d.pp[e]) && (t == 0 ? c[0].l1d.pa[e] : c[1].l1d.pa[e]) == (BASE >> 4) + line)
                pend_of = 1;
    end endfunction
    function integer valid_of; input integer t, line; integer e; begin
        valid_of = 0;
        for (e = 0; e < 4; e = e + 1)
            if ((t == 0 ? c[0].l1d.pv[e] : c[1].l1d.pv[e]) && (t == 0 ? c[0].l1d.pa[e] : c[1].l1d.pa[e]) == (BASE >> 4) + line)
                valid_of = 1;
    end endfunction

    integer d0, d1, d2, d3, i, nv, wpos, rpos, gap0, gap1;
    initial begin
        errors = 0; c_req = 0; c_we = 0;
        if (!$value$plusargs("SEED=%d", seed)) seed = 1;
        for (k = 0; k < (1 << (AW - 2)); k = k + 1) l2.mem[k] = 32'd0;
        for (k = 0; k < NW; k = k + 1) begin golden[k] = 32'h5000_0000 + 7 * k; l2.mem[(BASE >> 2) + k] = golden[k]; end
        repeat (3) @(posedge clk); #1 reset = 0;

        if (L1S >= 16 && DEG == 1) begin
            // ---- K1: re-arm in the grant cycle of the other cache's UPGR of the same line ----
            acc(0, 0, 53, 0, 0);                         // core 0: 53 in S
            acc(1, 0, 50, 0, 0); acc(1, 0, 51, 0, 0); idle(60);   // core 1 buffers 52
            expect_eq("K1 52 buffered", valid_of(1, 52), 1);
            c_req = 2'b11; c_we = 2'b01;
            c_addr[0] = la(53, 2); c_wdata[0] = 32'hc0de_0053; c_addr[1] = la(52, 1);
            #1;
            expect_eq("K1 core 1 installs 52", c[1].l1d.inst, 1);
            expect_eq("K1 core 0 UPGR granted", b_gnt[0] && b_op[1:0] == 2, 1);
            @(posedge clk); #1;
            expect_eq("K1 pending 53 survives the UPGR", pend_of(1, 53), 1);
            drain2; idle(60);
            expect_eq("K1 53 buffered (flushed copy)", valid_of(1, 53), 1);
            d0 = c[1].l1d.n_pfhit;
            acc(1, 0, 53, 2, 0);                         // must read c0de_0053 (load check)
            expect_eq("K1 53 from the buffer", c[1].l1d.n_pfhit - d0, 1);
            idle(40);

            // ---- K2a: install held by a snooped RD of the dirty victim ----
            acc(1, 1, 66, 1, 32'hd1d1_0066);             // core 1: line 66 (set 2) in M
            acc(1, 0, 80, 0, 0); acc(1, 0, 81, 0, 0); idle(60);   // buffers 82 (set 2)
            expect_eq("K2a 82 buffered", valid_of(1, 82), 1);
            d0 = c[1].l1d.n_wb; d1 = c[1].l1d.n_pfhit;
            c_req = 2'b11; c_we = 2'b00; c_addr[0] = la(66, 1); c_addr[1] = la(82, 3);
            #1;
            expect_eq("K2a core 0 RD granted", b_gnt[0], 1);
            expect_eq("K2a core 1 install held", !c[1].l1d.inst && c[1].l1d.s_block, 1);
            drain2;
            expect_eq("K2a no write-back (victim flushed)", c[1].l1d.n_wb - d0, 0);
            expect_eq("K2a install from the buffer", c[1].l1d.n_pfhit - d1, 1);
            acc(1, 0, 66, 1, 0);                         // the flushed store is still visible
            idle(40);
            // ---- K2b: the same with core 0's RDX of the victim ----
            acc(1, 1, 98, 0, 32'hd2d2_0098);             // set 2 again, M
            acc(1, 0, 112, 0, 0); acc(1, 0, 113, 0, 0); idle(60);  // buffers 114 (set 2)
            expect_eq("K2b 114 buffered", valid_of(1, 114), 1);
            d0 = c[1].l1d.n_wb; d1 = c[1].l1d.n_pfhit;
            c_req = 2'b11; c_we = 2'b01; c_addr[0] = la(98, 3); c_wdata[0] = 32'he0e0_0098; c_addr[1] = la(114, 0);
            #1;
            expect_eq("K2b core 0 RDX granted", b_gnt[0] && b_op[1:0] == 1, 1);
            expect_eq("K2b core 1 install held", !c[1].l1d.inst, 1);
            drain2;
            expect_eq("K2b no write-back", c[1].l1d.n_wb - d0, 0);
            expect_eq("K2b install from the buffer", c[1].l1d.n_pfhit - d1, 1);
            acc(1, 0, 98, 0, 0); acc(1, 0, 98, 3, 0);
            idle(40);

            // ---- K3a: store to a pending line before its prefetch is granted ----
            acc(1, 0, 130, 0, 0); acc(1, 0, 131, 0, 0);  // allocates 132 (pending, not yet granted)
            d0 = c[1].l1d.n_pf; d1 = c[1].l1d.n_miss;
            expect_eq("K3a 132 pending", pend_of(1, 132), 1);
            acc(1, 1, 132, 1, 32'hf00d_0132);
            expect_eq("K3a demand RDX", c[1].l1d.n_miss - d1, 1);
            expect_eq("K3a pending dropped", pend_of(1, 132), 0);
            idle(60);
            expect_eq("K3a only 133 prefetched", c[1].l1d.n_pf - d0, 1);
            expect_eq("K3a 133 buffered", valid_of(1, 133), 1);
            acc(0, 0, 132, 1, 0);
            idle(60);
            // ---- K3b: store to a line whose prefetch is in flight ----
            acc(1, 0, 140, 0, 0); acc(1, 0, 141, 0, 0); // allocates 142
            idle(4);
            expect_eq("K3b prefetch of 142 in flight", c[1].l1d.busy && c[1].l1d.cur_pf, 1);
            d0 = c[1].l1d.n_pflate; d1 = c[1].l1d.n_pfhit; d2 = c[1].l1d.n_upgr;
            acc(1, 1, 142, 2, 32'hf00d_0142);
            expect_eq("K3b late", c[1].l1d.n_pflate - d0, 1);
            expect_eq("K3b buffer hit", c[1].l1d.n_pfhit - d1, 1);
            expect_eq("K3b upgrade", c[1].l1d.n_upgr - d2, 1);
            acc(0, 0, 142, 2, 0);
            idle(60);

            // ---- K4: a snooped RD does not remove a buffered line ----
            acc(1, 0, 160, 0, 0); acc(1, 0, 161, 0, 0); idle(60);   // buffers 162
            d0 = c[1].l1d.n_pfkill; d1 = c[1].l1d.n_pfhit;
            acc(0, 0, 162, 0, 0);
            expect_eq("K4 no kill on RD", c[1].l1d.n_pfkill - d0, 0);
            expect_eq("K4 still buffered", valid_of(1, 162), 1);
            acc(1, 0, 162, 0, 0);
            expect_eq("K4 buffer hit after the RD", c[1].l1d.n_pfhit - d1, 1);
            idle(40);

            // ---- K5: FIFO reallocation of unused entries ----
            acc(1, 0, 170, 0, 0); acc(1, 0, 171, 0, 0); idle(60);   // buffers 172
            nv = c[1].l1d.pv[0] + c[1].l1d.pv[1] + c[1].l1d.pv[2] + c[1].l1d.pv[3];
            d0 = c[1].l1d.n_pfuseless;
            for (i = 0; i < 4; i = i + 1) begin acc(1, 0, 180 + 10 * i, 0, 0); acc(1, 0, 181 + 10 * i, 0, 0); idle(60); end
            expect_eq("K5 useless = valid entries overwritten", c[1].l1d.n_pfuseless - d0, nv);
            idle(20);
        end

        // ---- K6: core 0 streams stores, core 1 streams loads over lines 300..331 ----
        wpos = 0; rpos = 0; gap0 = 0; gap1 = 0; d0 = c[1].l1d.n_pfkill; d1 = c[1].l1d.n_pfhit;
        d2 = c[0].l1d.n_pfhit; d3 = c[1].l1d.n_pf;
        for (cyc = 0; cyc < 40000; cyc = cyc + 1) begin
            if (!c_req[0]) begin
                if (gap0 > 0) gap0 = gap0 - 1;
                else begin
                    c_req[0] = 1; c_we[0] = ($random(seed) & 3) != 0;   // 3/4 stores, 1/4 loads
                    c_addr[0] = la(300 + (wpos >> 2) % 32, wpos & 3); c_wdata[0] = $random(seed);
                    wpos = wpos + 1; gap0 = {$random(seed)} % 4;
                end
            end
            if (!c_req[1]) begin
                if (gap1 > 0) gap1 = gap1 - 1;
                else begin
                    c_req[1] = 1; c_we[1] = 0;
                    c_addr[1] = la(300 + (rpos >> 2) % 32, rpos & 3);
                    rpos = rpos + 1 + ((($random(seed) & 15) == 0) ? 4 : 0);   // sometimes skip a line
                    gap1 = {$random(seed)} % 3;
                end
            end
            @(posedge clk);
            if (c_req[0] && c_ready[0]) c_req[0] <= 0;
            if (c_req[1] && c_ready[1]) c_req[1] <= 0;
            #1;
        end
        drain2; idle(200);
        $display("K6 stores/loads issued %0d/%0d core1 pf=%0d pfhit=%0d pfkill=%0d core0 pfhit=%0d",
                 wpos, rpos, c[1].l1d.n_pf - d3, c[1].l1d.n_pfhit - d1, c[1].l1d.n_pfkill - d0, c[0].l1d.n_pfhit - d2);
        if (c[1].l1d.n_pfhit - d1 == 0) begin $display("FAIL K6 reader never hit its buffer"); errors = errors + 1; end
        if (c[1].l1d.n_pfkill - d0 == 0) begin $display("FAIL K6 writer never killed a buffered line"); errors = errors + 1; end
        if (bus.busy || b_req != 0) begin $display("FAIL K6 drain"); errors = errors + 1; end
        // coherent view == golden
        for (k = 0; k < 32 * 4; k = k + 1) begin : fin
            reg [32:0] p0, p1; reg [31:0] a;
            a = la(300 + k / 4, k % 4);
            p0 = c[0].l1d.peek_m(a); p1 = c[1].l1d.peek_m(a);
            if ((p0[32] && p0[31:0] !== golden[gi(a)]) || (p1[32] && p1[31:0] !== golden[gi(a)]) || (p0[32] && p1[32]) ||
                (!p0[32] && !p1[32] && l2.peek(a) !== golden[gi(a)])) begin
                $display("FAIL FINAL word %h", a); errors = errors + 1;
            end
        end
        if (errors == 0) $display("RESULT: VERIFY R11 DPF PASSED");
        else $display("RESULT: VERIFY R11 DPF FAILED (%0d)", errors);
        $finish;
    end
endmodule
