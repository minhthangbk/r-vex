//======================================================================
// tb_verify_r12_wbb: final-verify r12 directed races of the L1D write-back
// buffer (WBB) with the stream prefetch buffer (DPF) and the FAST bus gap
// re-grant (reviews/contracts/bus.md). Two l1d_cache (FWD WBB DPF=4 DEG=1,
// 4 sets) on coh_bus FAST + l2_cache FAST WBUF (8 sets, L2_LAT 2, MEM_LAT 5).
// Scenarios (each swept over the relative start offset of the two cores):
//   S1 second dirty victim while the buffer is full (WBB-03)
//   S2 background drain against a stream of the other core's demands (WBB-04, BUS-07)
//   S3 demand miss that waits for its own in-flight prefetch (FWD-02)
//   S4 store stream that fills the L2 with dirty lines (L2WB-01/02)
//   S5 the buffered line wanted by the other core while its owner misses on it (WBB-02/03)
//   S6 DPF buffer hit with a dirty victim while the other core snoops the victim
//      or the installed line (WBB-05/06)
// Monitors, every cycle (contract items, not the code):
//   OWNER    one owner per line: an M line or a buffered line is valid nowhere else (WBB-02)
//   WB-PARK  with WBB = 1 a dirty victim is parked, never written by a WB of the array slot (WBB-01/03)
//   PRIO     a background drain is never granted while the other master presents a demand (WBB-04, BUS-07)
//   INFLIGHT the buffered line stays the owner copy (peek_m) until its WB completes (WBB-04)
//   FWD      no core miss completes while the own prefetch or the buffer WB owns the bus (FWD-02)
//   MWAIT    a waiting L2 memory access starts in the cycle the write behind ends (L2WB-02)
// Data: every load is checked against the value the contract allows.
//======================================================================
`timescale 1ns/1ps
module tb_verify_r12_wbb;
    localparam AW = 16;
    reg clk = 0, reset = 1; always #5 clk = ~clk;
    reg [1:0] c_req, c_we;
    reg [31:0] c_addr [0:1], c_wdata [0:1];
    wire [31:0] c_rdata [0:1]; wire [1:0] c_ready;
    wire [1:0] b_req, b_pf, b_gnt, b_done, s_valid, s_hit_m;
    wire [3:0] b_op; wire [2*AW-1:0] b_addr; wire [255:0] b_wline, s_line;
    wire [127:0] b_rline; wire [1:0] s_op; wire [AW-1:0] s_addr;
    wire l2_req, l2_we, l2_done; wire [AW-1:0] l2_addr; wire [127:0] l2_wline, l2_rline;
    integer errors = 0, cyc = 0, k, k1, d;
    integer n_contend = 0, n_inflight = 0, n_park2 = 0, n_s6park = 0, n_s6blk = 0;

    genvar g;
    generate for (g = 0; g < 2; g = g + 1) begin : c
        l1d_cache #(.AW(AW), .SETS(4), .FWD(1), .WBB(1), .DPF(4), .DPF_DEG(1), .DPF_HIST(4)) l1d (
            .clk(clk), .reset(reset), .c_req(c_req[g]), .c_we(c_we[g]), .c_addr(c_addr[g]),
            .c_wdata(c_wdata[g]), .c_rdata(c_rdata[g]), .c_ready(c_ready[g]),
            .b_req(b_req[g]), .b_pf(b_pf[g]), .b_op(b_op[2*g +: 2]), .b_addr(b_addr[AW*g +: AW]),
            .b_wline(b_wline[128*g +: 128]), .b_gnt(b_gnt[g]), .b_done(b_done[g]), .b_rline(b_rline),
            .s_valid(s_valid[g]), .s_op(s_op), .s_addr(s_addr), .s_hit_m(s_hit_m[g]),
            .s_line(s_line[128*g +: 128]));
    end endgenerate
    coh_bus #(.N(2), .AW(AW), .FAST(1)) bus (
        .clk(clk), .reset(reset), .m_req(b_req), .m_pf(b_pf), .m_op(b_op), .m_addr(b_addr),
        .m_wline(b_wline), .m_gnt(b_gnt), .m_done(b_done), .m_rline(b_rline),
        .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr), .s_hit_m(s_hit_m), .s_line(s_line),
        .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wline(l2_wline),
        .l2_done(l2_done), .l2_rline(l2_rline));
    l2_cache #(.AW(AW), .SETS(8), .L2_LAT(2), .MEM_LAT(5), .FAST(1), .WBUF(1)) l2 (
        .clk(clk), .reset(reset), .req(l2_req), .we(l2_we), .addr(l2_addr), .wline(l2_wline),
        .done(l2_done), .rline(l2_rline));

    function [31:0] init; input [31:0] a; init = 32'h90000000 + (a >> 2); endfunction

    // one core access; returns the load data
    task automatic acc;
        input integer p; input we; input [31:0] a, dw; output [31:0] r;
        begin
            @(negedge clk); c_req[p] = 1; c_we[p] = we; c_addr[p] = a; c_wdata[p] = dw;
            #1; while (!c_ready[p]) begin @(negedge clk); #1; end
            r = c_rdata[p];
            @(posedge clk); #1; c_req[p] = 0;
        end
    endtask
    task automatic ld;   // load and compare with one or two allowed values
        input integer p; input [31:0] a, e0, e1; input [8*24-1:0] what;
        reg [31:0] r;
        begin
            acc(p, 0, a, 0, r);
            if (r !== e0 && r !== e1) begin
                $display("DATA MISMATCH %0s cyc=%0d d=%0d core=%0d addr=%h got=%h exp=%h|%h",
                         what, cyc, d, p, a, r, e0, e1);
                errors = errors + 1;
            end
        end
    endtask
    task automatic st; input integer p; input [31:0] a, v; reg [31:0] r; begin acc(p, 1, a, v, r); end endtask
    task idle; input integer n; begin repeat (n) @(posedge clk); end endtask
    task quiesce; begin   // both buffers drained, bus and L2 idle
        k = 0;
        while ((c[0].l1d.wv || c[1].l1d.wv || c[0].l1d.busy || c[1].l1d.busy || bus.busy) && k < 2000) begin
            @(posedge clk); k = k + 1;
        end
        idle(12);
        if (k >= 2000) begin $display("QUIESCE TIMEOUT cyc=%0d", cyc); errors = errors + 1; end
    end endtask

    // ------------------------------------------------------------------ monitors
    integer t, s, u;
    reg [AW-5:0] la;
    reg [32:0] pk;
    reg pr_mwait;
    always @(posedge clk) if (!reset) begin
        cyc <= cyc + 1;
        // OWNER (WBB-02, SWMR incl. the buffers)
        for (t = 0; t < 2; t = t + 1) for (s = 0; s < 4; s = s + 1) begin
            u = 1 - t;
            if ((t == 0 ? c[0].l1d.st[s] : c[1].l1d.st[s]) == 2) begin
                la = {(t == 0 ? c[0].l1d.tg[s] : c[1].l1d.tg[s]), s[1:0]};
                if ((u == 0 ? c[0].l1d.st[s] : c[1].l1d.st[s]) != 0 &&
                    (u == 0 ? c[0].l1d.tg[s] : c[1].l1d.tg[s]) == la[AW-5:2]) begin
                    $display("OWNER VIOLATION cyc=%0d line %h M in cache %0d and valid in %0d", cyc, la, t, u);
                    errors = errors + 1;
                end
                if ((u == 0 ? c[0].l1d.wv : c[1].l1d.wv) && (u == 0 ? c[0].l1d.wa : c[1].l1d.wa) == la) begin
                    $display("OWNER VIOLATION cyc=%0d line %h M in cache %0d and buffered in %0d", cyc, la, t, u);
                    errors = errors + 1;
                end
            end
        end
        for (t = 0; t < 2; t = t + 1)
            if (t == 0 ? c[0].l1d.wv : c[1].l1d.wv) begin
                la = (t == 0 ? c[0].l1d.wa : c[1].l1d.wa);
                for (u = 0; u < 2; u = u + 1)
                    if ((u == 0 ? c[0].l1d.st[la[1:0]] : c[1].l1d.st[la[1:0]]) != 0 &&
                        (u == 0 ? c[0].l1d.tg[la[1:0]] : c[1].l1d.tg[la[1:0]]) == la[AW-5:2]) begin
                        $display("OWNER VIOLATION cyc=%0d buffered line %h of cache %0d valid in array %0d", cyc, la, t, u);
                        errors = errors + 1;
                    end
            end
        // WB-PARK (WBB-01/03): every WB granted with WBB = 1 is the buffer's
        for (t = 0; t < 2; t = t + 1)
            if (b_gnt[t] && b_op[2*t +: 2] == 2'd3 && !(t == 0 ? c[0].l1d.wsel : c[1].l1d.wsel)) begin
                $display("WB-PARK VIOLATION cyc=%0d cache %0d writes back an array victim (buffer full: %0d)",
                         cyc, t, (t == 0 ? c[0].l1d.wv : c[1].l1d.wv));
                errors = errors + 1;
            end
        // PRIO (WBB-04, BUS-07)
        for (t = 0; t < 2; t = t + 1) begin
            u = 1 - t;
            if ((t == 0 ? (c[0].l1d.wdr_bg && !c[0].l1d.dreq) : (c[1].l1d.wdr_bg && !c[1].l1d.dreq)) &&
                b_req[u] && !b_pf[u]) begin
                n_contend <= n_contend + 1;
                if (b_gnt[t]) begin
                    $display("PRIO VIOLATION cyc=%0d background drain of cache %0d granted over a demand of %0d", cyc, t, u);
                    errors = errors + 1;
                end
            end
        end
        // INFLIGHT (WBB-04: the buffer empties at the WB completion, it is the owner copy until then)
        if (c[0].l1d.busy && c[0].l1d.cur_wbb) begin
            pk = c[0].l1d.peek_m(c[0].l1d.cur_addr); n_inflight <= n_inflight + 1;
            if (!pk[32]) begin $display("INFLIGHT VIOLATION cyc=%0d cache 0 line %h", cyc, c[0].l1d.cur_addr); errors = errors + 1; end
        end
        if (c[1].l1d.busy && c[1].l1d.cur_wbb) begin
            pk = c[1].l1d.peek_m(c[1].l1d.cur_addr); n_inflight <= n_inflight + 1;
            if (!pk[32]) begin $display("INFLIGHT VIOLATION cyc=%0d cache 1 line %h", cyc, c[1].l1d.cur_addr); errors = errors + 1; end
        end
        // FWD (FWD-02)
        if (c_ready[0] && !c[0].l1d.c_ok && c[0].l1d.busy && (c[0].l1d.cur_pf || c[0].l1d.cur_wbb)) begin
            $display("FWD VIOLATION cyc=%0d cache 0 miss completed by a prefetch/WB fill", cyc); errors = errors + 1;
        end
        if (c_ready[1] && !c[1].l1d.c_ok && c[1].l1d.busy && (c[1].l1d.cur_pf || c[1].l1d.cur_wbb)) begin
            $display("FWD VIOLATION cyc=%0d cache 1 miss completed by a prefetch/WB fill", cyc); errors = errors + 1;
        end
        // MWAIT (L2WB-02): having waited while memory was busy, the access must leave MWAIT
        // in the cycle the write behind ends, not one cycle later
        if (l2.state == 3'd4 && !l2.wb_busy && pr_mwait) begin
            $display("MWAIT VIOLATION cyc=%0d memory idle while an access waits", cyc); errors = errors + 1;
        end
        pr_mwait <= (l2.state == 3'd4) && l2.wb_busy;
        if (b_gnt[0] && c[0].l1d.wsel && c[0].l1d.dreq) n_park2 <= n_park2 + 1;
    end
    initial pr_mwait = 0;

    // ------------------------------------------------------------------ scenarios
    reg [31:0] A, B, C, X, L0, base, r0, r1;
    integer np0, nf0, nb0, ns0;
    initial begin
        c_req = 0; c_we = 0; c_addr[0] = 0; c_addr[1] = 0; c_wdata[0] = 0; c_wdata[1] = 0;
        for (k = 0; k < (1 << (AW - 2)); k = k + 1) l2.mem[k] = 32'h90000000 + k;
        repeat (3) @(posedge clk); #1; reset = 0;
        d = 0;

        // S1 (WBB-03): three dirty lines of one set back to back; the second dirty victim finds
        // the buffer full: the buffer is written back first (demand class), then the victim parked
        for (d = 0; d < 3; d = d + 1) begin
            base = 32'h1000 + d * 32'h400; A = base; B = base + 32'h40; C = base + 32'h80;
            nb0 = c[0].l1d.n_wbb;
            st(0, A + 4, 32'hA1000000 + d); st(0, B + 8, 32'hB1000000 + d); st(0, C + 12, 32'hC1000000 + d);
            if (c[0].l1d.n_wbb != nb0 + 2) begin
                $display("S1 FAIL d=%0d parked %0d victims (expected 2)", d, c[0].l1d.n_wbb - nb0); errors = errors + 1;
            end
            ld(1, A + 4, 32'hA1000000 + d, 32'hA1000000 + d, "S1 A");
            ld(1, B + 8, 32'hB1000000 + d, 32'hB1000000 + d, "S1 B");
            ld(1, C + 12, 32'hC1000000 + d, 32'hC1000000 + d, "S1 C");
            ld(1, A, init(A), init(A), "S1 A0");
            quiesce;
        end

        // S2 (WBB-04, BUS-07): core 0 parks D and goes idle while core 1 streams demand misses
        for (d = 0; d < 4; d = d + 1) begin
            base = 32'h3000 + d * 32'h400; A = base; B = base + 32'h40;
            st(0, A, 32'hD2000000 + d);
            fork
                st(0, B, 32'hE2000000 + d);
                begin
                    idle(d);
                    for (k1 = 0; k1 < 10; k1 = k1 + 1) ld(1, 32'h8000 + d * 32'h400 + k1 * 32'h40, init(32'h8000 + d * 32'h400 + k1 * 32'h40),
                                                       init(32'h8000 + d * 32'h400 + k1 * 32'h40), "S2 stream");
                end
            join
            quiesce;
            ld(1, A, 32'hD2000000 + d, 32'hD2000000 + d, "S2 D");
            ld(1, B, 32'hE2000000 + d, 32'hE2000000 + d, "S2 E");
            quiesce;
        end

        // S3 (FWD-02): stream L0, L1 (allocates the prefetch of L2), k hits, then load L2
        for (d = 0; d < 8; d = d + 1) begin
            L0 = 32'h5000 + d * 32'h400;
            ld(0, L0, init(L0), init(L0), "S3 L0");
            ld(0, L0 + 16, init(L0 + 16), init(L0 + 16), "S3 L1");
            for (k = 0; k < d; k = k + 1) ld(0, L0 + 20, init(L0 + 20), init(L0 + 20), "S3 hit");
            ld(0, L0 + 36, init(L0 + 36), init(L0 + 36), "S3 L2");
            quiesce;
        end

        // S4 (L2WB-01/02): both cores store over 40 lines (L2 holds 8): dirty L2 victims are
        // posted while the next misses arrive
        fork
            for (k = 0; k < 40; k = k + 1) st(0, 32'h6000 + k * 16, 32'h64000000 + k);
            for (k1 = 0; k1 < 40; k1 = k1 + 1) st(1, 32'h7000 + k1 * 16 + 4, 32'h74000000 + k1);
        join
        quiesce;
        for (k = 0; k < 40; k = k + 4) begin
            ld(1, 32'h6000 + k * 16, 32'h64000000 + k, 32'h64000000 + k, "S4 c0");
            ld(0, 32'h7000 + k * 16 + 4, 32'h74000000 + k, 32'h74000000 + k, "S4 c1");
        end
        quiesce;

        // S5 (WBB-02/03): core 0 parks A behind its store miss B and then loads A (own access to the
        // buffered line); core 1 starts d cycles after core 0's store B with a store (even d) or a
        // load (odd d) of A word 0
        for (d = 0; d < 16; d = d + 1) begin
            base = 32'h9000 + d * 32'h200; A = base; B = base + 32'h40;
            for (k = 0; k < 4; k = k + 1) st(0, A + 4 * k, 32'h50000000 + d * 16 + k);
            fork
                begin st(0, B, 32'h5B000000 + d); ld(0, A + 8, 32'h50000002 + d * 16, 32'h50000002 + d * 16, "S5 own A2");
                      ld(0, A, 32'h50000000 + d * 16, (d % 2 == 0) ? 32'h5C000000 + d : 32'h50000000 + d * 16, "S5 own A0"); end
                begin idle(d);
                      if (d % 2 == 0) st(1, A, 32'h5C000000 + d);
                      else ld(1, A, 32'h50000000 + d * 16, 32'h50000000 + d * 16, "S5 other A0"); end
            join
            for (k = 1; k < 4; k = k + 1)
                ld(1, A + 4 * k, 32'h50000000 + d * 16 + k, 32'h50000000 + d * 16 + k, "S5 final A");
            ld(1, A, (d % 2 == 0) ? 32'h5C000000 + d : 32'h50000000 + d * 16,
                     (d % 2 == 0) ? 32'h5C000000 + d : 32'h50000000 + d * 16, "S5 final A0 c1");
            ld(0, A, (d % 2 == 0) ? 32'h5C000000 + d : 32'h50000000 + d * 16,
                     (d % 2 == 0) ? 32'h5C000000 + d : 32'h50000000 + d * 16, "S5 final A0 c0");
            ld(1, B, 32'h5B000000 + d, 32'h5B000000 + d, "S5 final B");
            quiesce;
        end

        // S6 (WBB-05/06): X dirty in set 2 of core 0; stream L0 (set 0), L1 (set 1) allocates the
        // prefetch of L2 (set 2); once it is in the buffer core 0 loads L2 (buffer hit, dirty victim X:
        // park + install in one cycle) while core 1, d cycles later, stores X (snoops the victim)
        // or loads L2 (snoops the installed line)
        for (d = 0; d < 12; d = d + 1) begin
            L0 = 32'hC000 + d * 32'h100;          // sets 0,1,2 for L0, L0+16, L0+32
            X  = L0 + 32'h20 + 32'h40;            // set 2, other tag
            st(0, X + 4, 32'h60000000 + d);
            np0 = c[0].l1d.n_pf; nf0 = c[0].l1d.n_pfhit;
            ld(0, L0, init(L0), init(L0), "S6 L0");
            ld(0, L0 + 16, init(L0 + 16), init(L0 + 16), "S6 L1");
            k = 0; while (c[0].l1d.n_pf == np0 && k < 100) begin @(posedge clk); k = k + 1; end
            k = 0; while (c[0].l1d.busy && k < 100) begin @(posedge clk); k = k + 1; end
            nb0 = c[0].l1d.n_wbb; ns0 = c[0].l1d.n_wbbsnp;
            fork
                ld(0, L0 + 32, init(L0 + 32), init(L0 + 32), "S6 L2");
                begin idle(d / 2);
                      if (d % 2 == 0) st(1, X + 4, 32'h6C000000 + d);
                      else ld(1, L0 + 32, init(L0 + 32), init(L0 + 32), "S6 other L2"); end
            join
            if (c[0].l1d.n_wbb != nb0) n_s6park = n_s6park + 1; else n_s6blk = n_s6blk + 1;
            ld(1, X + 4, (d % 2 == 0) ? 32'h6C000000 + d : 32'h60000000 + d,
                         (d % 2 == 0) ? 32'h6C000000 + d : 32'h60000000 + d, "S6 final X c1");
            ld(0, X + 4, (d % 2 == 0) ? 32'h6C000000 + d : 32'h60000000 + d,
                         (d % 2 == 0) ? 32'h6C000000 + d : 32'h60000000 + d, "S6 final X c0");
            ld(0, X, init(X), init(X), "S6 final X0");
            quiesce;
        end

        // coverage of the races (a vacuous run must fail)
        $display("VERIFY R12 WBB coverage: contend=%0d inflight=%0d demand-drains=%0d pflate=%0d/%0d mwait=%0d wbuf=%0d s6 park/blocked=%0d/%0d wbbsnp=%0d/%0d",
                 n_contend, n_inflight, n_park2, c[0].l1d.n_pflate, c[1].l1d.n_pflate, l2.n_mwait, l2.n_wbuf,
                 n_s6park, n_s6blk, c[0].l1d.n_wbbsnp, c[1].l1d.n_wbbsnp);
        if (n_contend == 0 || n_inflight == 0 || n_park2 == 0 || c[0].l1d.n_pflate == 0 || l2.n_mwait == 0 ||
            n_s6park == 0 || c[0].l1d.n_wbbsnp == 0) begin
            $display("COVERAGE FAIL: a race was not reached"); errors = errors + 1;
        end
        if (errors == 0) $display("RESULT: VERIFY R12 WBB PASSED");
        else $display("RESULT: VERIFY R12 WBB FAILED (%0d)", errors);
        $finish;
    end
    initial begin #3000000; $display("TIMEOUT"); $display("RESULT: VERIFY R12 WBB FAILED"); $finish; end
endmodule
