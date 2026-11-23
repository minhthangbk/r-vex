`timescale 1ns/1ps
`include "rvex_defs.vh"
`include "slot8.vh"

// tb_ldo.v (round r32 B2, reviews/contracts/ldo.md LO-02..07): 256-bit loads (ldo, opcode 0x1D) in slots 2 and 3, the
// cores of tb_ldq.v: bare (b), external D port with chosen misses (e), second D port that always hits (d), second port
// with port-1 misses (m). Word i of the data rows sits at 0x100 + 4 i and holds 0xA0000000 + i. Two programs (the core
// has 64 GRs; the cores are reset between them):
//   phase 1  A slot-3 ldo 0[r2] (EXT: word 2 misses); A2 slot-2 ldo 32[r2] (word 0 misses); QQ slot-2 ldo -1024[r6]
//            (the smallest offset) + slot-3 ldo 1020[r7] (the largest): 16 accesses (misses: slot-2 word 5, slot-3
//            word 7); P-SL slot-2 stw of word 39 + slot-3 ldo of words 32..39 (reads the old word; EXT: the ldo first;
//            miss word 1); P-LS slot-2 ldo of words 40..47 + slot-3 stw of word 45 (miss ldo word 6 and the store)
//   phase 2  MA misaligned base: all eight destinations 0, MISALIGN once; IO0 word 0 in the PIU window, IO7 word 7 only
//            (0xFDE4), IOT 0xFFF0 (words 0..3 in the hw_sem page, 4..7 past 64 KB): no destination written, no external
//            access (the I/O pages are one 512-byte run: any I/O word of an ldo makes word 0 or word 7 one); R0 quad A
//            = r0..r3: r0 stays 0
//   PIU      LD counts each ldo once, MEMSEQ = the holds (e: nacc - 1 per packet; d: two accesses per cycle), DSTALL 3
//            per miss, cycles = bare + MEMSEQ + DSTALL
//   DRAIN    8 cycles after done no D request; SEQ one packet per shape on a single-port EXT core (s) and a DUAL_D
//            core (t): ldo2, ldo3, ldo + store, store + ldo, ldq + ldo, two ldo (16 accesses), one miss at every access
//            index and all hit; the request log must be the LO-06 order exactly once
module tb_ldo;
    reg clk = 0, reset = 1, start = 0;
    always #5 clk = ~clk;
    integer pass = 0, fail = 0, k, cyc = 0;

    // ---- bare core ----
    wire done_b; wire [31:0] cycles_b;
    rvex_core #(.EXT_DMEM(0)) b(.clk(clk), .reset(reset), .start(start), .done(done_b), .cycles(cycles_b),
        .d_req(), .d_we(), .d_addr(), .d_wdata(), .d_word(), .d_rdata(32'd0), .d_ready(1'b1),
        .d2_req(), .d2_we(), .d2_addr(), .d2_wdata(), .d2_rdata(32'd0), .d2_hit(1'b0), .stall_cycles(),
        .i_req(), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles());

    // ---- external D port core + memory model with misses ----
    wire done_e, d_req, d_we; wire [31:0] cycles_e, d_addr, d_wdata, stall_e;
    reg  [31:0] xm [0:255];
    wire [31:0] d_rdata = xm[d_addr[9:2]];
    reg  [3:0]  wait_n = 0;
    reg  [31:0] wait_addr = 32'hFFFF_FFFF;
    function is_miss; input [31:0] a; begin
        is_miss = a == 32'h108 || a == 32'h120 || a == 32'h154 || a == 32'h17C || a == 32'h184 || a == 32'h1B8 ||
                  a == 32'h1B4; end endfunction
    wire        miss_addr = is_miss(d_addr);
    integer     miss_e = 0, miss_m = 0;
    wire        d_ready = !(d_req && miss_addr && !(wait_addr == d_addr && wait_n == 0));
    integer     io_acc = 0;                                                            // external accesses to 0xFE00.. / past 64 KB
    rvex_core #(.EXT_DMEM(1)) e(.clk(clk), .reset(reset), .start(start), .done(done_e), .cycles(cycles_e),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(), .d_rdata(d_rdata),
        .d_ready(d_ready), .d2_req(), .d2_we(), .d2_addr(), .d2_wdata(), .d2_rdata(32'd0), .d2_hit(1'b0),
        .stall_cycles(stall_e), .i_req(), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles());
    always @(posedge clk) if (!reset) begin
        if (d_req && miss_addr && wait_addr != d_addr) begin wait_addr <= d_addr; wait_n <= 4'd2; miss_e = miss_e + 1; end   // 3 cycles held
        else if (d_req && miss_addr && wait_n != 0) wait_n <= wait_n - 1;
        if (d_req && d_ready && d_we) xm[d_addr[9:2]] <= d_wdata;
        if (d_req && (d_addr[15:8] == 8'hFE || &d_addr[15:8] || d_addr[31:16] != 0)) io_acc = io_acc + 1;
    end

    // ---- dual-port core: both ports always hit ----
    wire done_d, q_req, q_we, q2_req, q2_we; wire [31:0] cycles_d, q_addr, q_wdata, q2_addr, q2_wdata;
    reg  [31:0] ym [0:255];
    rvex_core #(.EXT_DMEM(1), .DUAL_D(1)) d(.clk(clk), .reset(reset), .start(start), .done(done_d), .cycles(cycles_d),
        .d_req(q_req), .d_we(q_we), .d_addr(q_addr), .d_wdata(q_wdata), .d_word(), .d_rdata(ym[q_addr[9:2]]),
        .d_ready(1'b1), .d2_req(q2_req), .d2_we(q2_we), .d2_addr(q2_addr), .d2_wdata(q2_wdata),
        .d2_rdata(ym[q2_addr[9:2]]), .d2_hit(1'b1), .stall_cycles(), .i_req(), .i_addr(), .i_rdata(256'd0),
        .i_ready(1'b1), .istall_cycles());
    always @(posedge clk) if (!reset) begin
        if (q_req && q_we) ym[q_addr[9:2]] <= q_wdata;
        if (q2_req && q2_we) ym[q2_addr[9:2]] <= q2_wdata;
    end

    // ---- dual-port core with port-1 misses (Codex R57 F4): port 2 always hits ----
    wire done_m, m_req, m_we, m2_req, m2_we; wire [31:0] cycles_m, m_addr, m_wdata, m2_addr, m2_wdata, stall_m;
    reg  [31:0] zm [0:255];
    reg  [3:0]  mwait_n = 0;
    reg  [31:0] mwait_addr = 32'hFFFF_FFFF;
    wire        m_miss = is_miss(m_addr);
    wire        m_ready = !(m_req && m_miss && !(mwait_addr == m_addr && mwait_n == 0));
    rvex_core #(.EXT_DMEM(1), .DUAL_D(1)) m(.clk(clk), .reset(reset), .start(start), .done(done_m), .cycles(cycles_m),
        .d_req(m_req), .d_we(m_we), .d_addr(m_addr), .d_wdata(m_wdata), .d_word(), .d_rdata(zm[m_addr[9:2]]),
        .d_ready(m_ready), .d2_req(m2_req), .d2_we(m2_we), .d2_addr(m2_addr), .d2_wdata(m2_wdata),
        .d2_rdata(zm[m2_addr[9:2]]), .d2_hit(1'b1), .stall_cycles(stall_m), .i_req(), .i_addr(), .i_rdata(256'd0),
        .i_ready(1'b1), .istall_cycles());
    always @(posedge clk) if (!reset) begin
        if (m_req && m_miss && mwait_addr != m_addr) begin mwait_addr <= m_addr; mwait_n <= 4'd2; miss_m = miss_m + 1; end
        else if (m_req && m_miss && mwait_n != 0) mwait_n <= mwait_n - 1;
        if (m_req && m_ready && m_we) zm[m_addr[9:2]] <= m_wdata;
        if (m_req && m_ready && m2_req && m2_we) zm[m2_addr[9:2]] <= m2_wdata;   // port 2 completes with port 1
    end

    // ---- SEQ matrix cores (Codex R58 F3): own reset, one miss address per case, request logs ----
    reg rs = 1, st = 0;
    reg  [31:0] smem [0:255], tmem [0:255];
    reg  [31:0] s_miss = 32'hFFFF_FFFF, t_miss = 32'hFFFF_FFFF;
    reg  [3:0]  s_wn = 0, t_wn = 0;
    reg  [31:0] s_wa = 32'hFFFF_FFFF, t_wa = 32'hFFFF_FFFF;
    wire done_s, s_req, s_we, done_t, t_req, t_we, t2_req, t2_we;
    wire [31:0] cycles_s, s_addr, s_wdata, stall_s, cycles_t, t_addr, t_wdata, t2_addr, t2_wdata, stall_t;
    wire s_ready = !(s_req && s_addr == s_miss && !(s_wa == s_addr && s_wn == 0));
    wire t_ready = !(t_req && t_addr == t_miss && !(t_wa == t_addr && t_wn == 0));
    rvex_core #(.EXT_DMEM(1)) s(.clk(clk), .reset(rs), .start(st), .done(done_s), .cycles(cycles_s),
        .d_req(s_req), .d_we(s_we), .d_addr(s_addr), .d_wdata(s_wdata), .d_word(), .d_rdata(smem[s_addr[9:2]]),
        .d_ready(s_ready), .d2_req(), .d2_we(), .d2_addr(), .d2_wdata(), .d2_rdata(32'd0), .d2_hit(1'b0),
        .stall_cycles(stall_s), .i_req(), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles());
    rvex_core #(.EXT_DMEM(1), .DUAL_D(1)) t(.clk(clk), .reset(rs), .start(st), .done(done_t), .cycles(cycles_t),
        .d_req(t_req), .d_we(t_we), .d_addr(t_addr), .d_wdata(t_wdata), .d_word(), .d_rdata(tmem[t_addr[9:2]]),
        .d_ready(t_ready), .d2_req(t2_req), .d2_we(t2_we), .d2_addr(t2_addr), .d2_wdata(t2_wdata),
        .d2_rdata(tmem[t2_addr[9:2]]), .d2_hit(1'b1), .stall_cycles(stall_t), .i_req(), .i_addr(), .i_rdata(256'd0),
        .i_ready(1'b1), .istall_cycles());
    integer ns = 0, nt = 0, mis_s = 0, mis_t = 0;
    reg [31:0] la_s [0:15], la_t [0:15];
    reg [1:0]  lp_t [0:15];
    reg        lw_s [0:15], lw_t [0:15];   // r32: 16 = two ldo
    always @(posedge clk) if (!rs) begin
        if (s_req && s_addr == s_miss && s_wa != s_addr) begin s_wa <= s_addr; s_wn <= 4'd2; mis_s = mis_s + 1; end
        else if (s_req && s_addr == s_miss && s_wn != 0) s_wn <= s_wn - 1;
        if (t_req && t_addr == t_miss && t_wa != t_addr) begin t_wa <= t_addr; t_wn <= 4'd2; mis_t = mis_t + 1; end
        else if (t_req && t_addr == t_miss && t_wn != 0) t_wn <= t_wn - 1;
        if (s_req && s_ready) begin
            if (ns < 16) begin la_s[ns] = s_addr; lw_s[ns] = s_we; end
            ns = ns + 1;
            if (s_we) smem[s_addr[9:2]] <= s_wdata;
        end
        if (t_req && t_ready) begin
            if (nt < 16) begin la_t[nt] = t_addr; lw_t[nt] = t_we; lp_t[nt] = 2'd1; end
            nt = nt + 1;
            if (t_we) tmem[t_addr[9:2]] <= t_wdata;
            if (t2_req) begin                                      // port 2 completes with port 1 (d2_hit)
                if (nt < 16) begin la_t[nt] = t2_addr; lw_t[nt] = t2_we; lp_t[nt] = 2'd2; end
                nt = nt + 1;
                if (t2_we) tmem[t2_addr[9:2]] <= t2_wdata;
            end
        end
    end

    // LO-01: quad A = 4 x syl[22:19], base = syl[16:11], quad B = 4 x syl[10:7],
    // offset / 4 = {syl[24:23], syl[18:17], syl[6:2]} (signed, -256..255)
    function [31:0] ldo; input [5:0] qa, qb, a; input integer off; reg [8:0] o; begin
        o = off / 4;
        ldo = {`MEM_LDO, o[8:7], qa[5:2], o[6:5], a, qb[5:2], o[4:0], 2'b00}; end endfunction
    function [31:0] ldq; input [5:0] pa, pb, a; input integer off; reg [6:0] o; begin
        o = off / 4;
        ldq = {`MEM_LDQ, o[6:5], pa[5:1], o[4], a, pb[5:1], o[3], o[2:0], 2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] dd, a; input [8:0] im; begin ri = {op, `SHORT_IMM, dd, a, im, 2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s, a; input [8:0] off; begin mst = {op, `NO_IMM, s, a, off, 2'b00}; end endfunction
    localparam [31:0] NOP = 0, PRE = 32'h5EED_0000, SV = 32'h5566_7788;
    task P; input [11:0] n; input [31:0] s2, s3; begin
        b.imem[n] = pk8(s3, s2, NOP, NOP); e.imem[n] = pk8(s3, s2, NOP, NOP); d.imem[n] = pk8(s3, s2, NOP, NOP);
        m.imem[n] = pk8(s3, s2, NOP, NOP); end endtask
    task STOPAT; input [11:0] n; begin
        b.imem[n] = {224'd0, `OP_STOP, 25'd0}; e.imem[n] = b.imem[n]; d.imem[n] = b.imem[n]; m.imem[n] = b.imem[n]; end endtask
    task chk; input [511:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("PASS %0s got=%0h", nm, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%0h expected=%0h", nm, got, exp); end
    end endtask
    task setw; input [31:0] a, v; begin b.dmem[a >> 2] = v; xm[a[9:2]] = v; ym[a[9:2]] = v; zm[a[9:2]] = v; end endtask
    task setr; input [5:0] r; input [31:0] v; begin b.gr[r] = v; e.gr[r] = v; d.gr[r] = v; m.gr[r] = v; end endtask
    task chk3; input [511:0] nm; input [5:0] r; input [31:0] exp; begin
        chk({nm, " bare"}, b.gr[r], exp); chk({nm, " EXT"}, e.gr[r], exp); chk({nm, " dual"}, d.gr[r], exp);
        chk({nm, " dual+miss"}, m.gr[r], exp); end endtask
    task chk8; input [511:0] nm; input [5:0] qa, qb; input integer w; integer i; begin   // qa..qa+3, qb..qb+3 = words w..w+7
        for (i = 0; i < 4; i = i + 1) chk3(nm, qa + i, W(w + i));
        for (i = 0; i < 4; i = i + 1) chk3(nm, qb + i, W(w + 4 + i)); end endtask
    task chkm; input [511:0] nm; input [31:0] a, exp; begin
        chk({nm, " bare"}, b.dmem[a >> 2], exp); chk({nm, " EXT"}, xm[a[9:2]], exp); chk({nm, " dual"}, ym[a[9:2]], exp);
        chk({nm, " dual+miss"}, zm[a[9:2]], exp); end endtask
    function [31:0] W; input integer i; begin W = 32'hA000_0000 + i; end endfunction
    task go; begin   // reset the four cores, wait for STOP of all four
        reset = 1; repeat (2) @(posedge clk); reset = 0; @(posedge clk); #1;
    end endtask
    task run; begin
        @(posedge clk); start = 1; @(posedge clk); start = 0;
        cyc = 0; while (!(done_b && done_e && done_d && done_m) && cyc < 600) begin @(posedge clk); cyc = cyc + 1; end
        chk("all four cores end by STOP within the cap", done_b && done_e && done_d && done_m, 1);
        repeat (8) @(posedge clk); #1;
        chk("DRAIN no D request 8 cycles after done (RR-11)", {d_req, q_req, q2_req, m_req, m2_req}, 5'd0);
    end endtask

    // ---- SEQ matrix ----
    reg [31:0] ea [0:15]; reg ew [0:15]; integer na, sh, pm, j, cyc_s, cyc_t, okt, hold_s, hold_t, c0s, c0t;
    reg [31:0] s2q, s3q;
    reg [8*8-1:0] shn;
    reg [511:0] nmb;
    task shape; input integer n; begin   // packet slots 2 / 3 and the expected access order (base r2 = 0x100)
        for (j = 0; j < 16; j = j + 1) begin ea[j] = 32'h100 + 4 * (j % 8); ew[j] = 1'b0; end
        case (n)
        0: begin shn = "ldo2"; s2q = ldo(8, 12, 2, 0); s3q = NOP; na = 8; end
        1: begin shn = "ldo3"; s2q = NOP; s3q = ldo(8, 12, 2, 0); na = 8; end
        2: begin shn = "ldo+st"; s2q = ldo(8, 12, 2, 0); s3q = mst(`MEM_STW, 62, 2, 64); na = 9;
                 ea[8] = 32'h140; ew[8] = 1'b1; end                                   // slot 2 first
        3: begin shn = "st+ldo"; s2q = mst(`MEM_STW, 62, 2, 64); s3q = ldo(8, 12, 2, 0); na = 9;
                 ea[8] = 32'h140; ew[8] = 1'b1; end                                   // slot 3 first: slot 2 stores
        4: begin shn = "ldq+ldo"; s2q = ldq(20, 22, 2, 64); s3q = ldo(8, 12, 2, 0); na = 12;
                 for (j = 0; j < 4; j = j + 1) ea[j] = 32'h140 + 4 * j; for (j = 4; j < 12; j = j + 1) ea[j] = 32'h100 + 4 * (j - 4); end
        default: begin shn = "ldo+ldo"; s2q = ldo(8, 12, 2, 0); s3q = ldo(16, 20, 2, 32); na = 16;
                 for (j = 8; j < 16; j = j + 1) ea[j] = 32'h120 + 4 * (j - 8); end
        endcase
    end endtask
    task seq_run; input integer p; begin   // one case: miss at access index p (-1: all hit)
        rs = 1; repeat (2) @(posedge clk);
        for (j = 0; j < 256; j = j + 1) begin smem[j] = (j >= 64) ? W(j - 64) : 32'd0; tmem[j] = smem[j]; end
        s.imem[0] = pk8(s3q, s2q, NOP, NOP); t.imem[0] = s.imem[0];
        s.imem[1] = {224'd0, `OP_STOP, 25'd0}; t.imem[1] = s.imem[1];
        s_miss = p < 0 ? 32'hFFFF_FFFF : ea[p]; t_miss = s_miss; s_wa = 32'hFFFF_FFFF; t_wa = 32'hFFFF_FFFF;
        s_wn = 0; t_wn = 0; ns = 0; nt = 0; mis_s = 0; mis_t = 0;
        rs = 0; @(posedge clk); #1;
        s.gr[2] = 32'h100; t.gr[2] = 32'h100; s.gr[62] = SV; t.gr[62] = SV;
        for (j = 8; j < 24; j = j + 1) begin s.gr[j] = PRE; t.gr[j] = PRE; end
        @(posedge clk); st = 1; @(posedge clk); st = 0;
        cyc = 0; while (!(done_s && done_t) && cyc < 200) begin @(posedge clk); cyc = cyc + 1; end
        repeat (8) @(posedge clk); #1;
        cyc_s = cycles_s; cyc_t = cycles_t;
        okt = done_s && done_t && ns == na;
        for (j = 0; j < na; j = j + 1) okt = okt && la_s[j] == ea[j] && lw_s[j] == ew[j];
        $sformat(nmb, "SEQ %0s p%0d e: order once", shn, p); chk(nmb, okt, 1);
        okt = done_t && nt == na;
        for (j = 0; j < na; j = j + 1) okt = okt && la_t[j] == ea[j] && lw_t[j] == ew[j] && lp_t[j] == (j % 2 ? 2 : 1);
        $sformat(nmb, "SEQ %0s p%0d dual: order once", shn, p); chk(nmb, okt, 1);
        $sformat(nmb, "SEQ %0s p%0d e: miss, DSTALL", shn, p); chk(nmb, {mis_s[3:0], stall_s[7:0]}, p < 0 ? 12'd0 : {4'd1, 8'd3});
        $sformat(nmb, "SEQ %0s p%0d dual: miss, DSTALL", shn, p); chk(nmb, {mis_t[3:0], stall_t[7:0]},
            (p < 0 || p % 2) ? 12'd0 : {4'd1, 8'd3});            // an odd index goes on port 2 (always hits)
        okt = 1;
        for (j = 0; j < 8; j = j + 1) okt = okt && s.gr[8 + j] == W(j) && t.gr[8 + j] == W(j);
        if (sh == 5) for (j = 0; j < 8; j = j + 1) okt = okt && s.gr[16 + j] == W(8 + j) && t.gr[16 + j] == W(8 + j);
        if (sh == 4) for (j = 0; j < 4; j = j + 1) okt = okt && s.gr[20 + j] == W(16 + j) && t.gr[20 + j] == W(16 + j);
        if (sh == 2 || sh == 3) okt = okt && smem[8'h50] == SV && tmem[8'h50] == SV;
        $sformat(nmb, "SEQ %0s p%0d values", shn, p); chk(nmb, okt, 1);
        if (p < 0) begin
            c0s = cyc_s; c0t = cyc_t; hold_s = na - 1; hold_t = (na + 1) / 2 - 1;
            $sformat(nmb, "SEQ %0s all hit: e - dual = extra holds", shn); chk(nmb, c0s - c0t, hold_s - hold_t);
        end else begin
            $sformat(nmb, "SEQ %0s p%0d e: cycles = all hit + 3", shn, p); chk(nmb, cyc_s, c0s + 3);
            $sformat(nmb, "SEQ %0s p%0d dual: cycles", shn, p); chk(nmb, cyc_t, c0t + ((p % 2) ? 0 : 3));
        end
        $sformat(nmb, "SEQ %0s p%0d: no request after done", shn, p); chk(nmb, {s_req, t_req, t2_req}, 3'd0);
    end endtask
    task seq_matrix; begin
        for (sh = 0; sh < 6; sh = sh + 1) begin
            shape(sh);
            for (pm = -1; pm < na; pm = pm + 1) seq_run(pm);
        end
    end endtask

    integer i1, cb1;
    initial begin
        for (k = 0; k < 256; k = k + 1) begin xm[k] = 32'd0; ym[k] = 32'd0; zm[k] = 32'd0; end
        for (k = 0; k < 72; k = k + 1) setw(32'h100 + 4 * k, W(k));
        // ---- phase 1 ----
        P(0, NOP,                        ldo(8, 12, 2, 0));                // A: words 0..7
        P(1, ldo(16, 20, 2, 32),         NOP);                             // A2: words 8..15
        P(2, ldo(24, 28, 6, -1024),      ldo(32, 36, 7, 1020));            // QQ: words 16..23 (r6 = 0x540), 24..31
        P(3, mst(`MEM_STW, 62, 2, 156),  ldo(40, 44, 2, 128));             // P-SL: store word 39, ldo words 32..39
        P(4, ldo(48, 52, 2, 160),        mst(`MEM_STW, 62, 2, 180));       // P-LS: ldo words 40..47, store word 45
        P(5, ri(`MEM_LDW, 57, 5, 9'h24), NOP);                             // PIU: LD
        P(6, ri(`MEM_LDW, 58, 5, 9'h6C), NOP);                             // PIU: MEMSEQ
        STOPAT(7);
        go;
        setr(2, 32'h100); setr(5, 32'hFE00); setr(6, 32'h540); setr(7, 32'h160 - 1020); setr(62, SV);
        run;
        chk8("A    slot-3 ldo (EXT miss on word 2)", 8, 12, 0);
        chk8("A2   slot-2 ldo (miss on word 0)", 16, 20, 8);
        chk8("NEG  offset -1024 (QQ slot 2)", 24, 28, 16);
        chk8("OFF  offset 1020 (QQ slot 3)", 32, 36, 24);
        for (k = 0; k < 7; k = k + 1) chk3("P-SL ldo words 32..38", 40 + k, W(32 + k));
        chk3("P-SL ldo reads the old word 39", 47, W(39));
        chkm("P-SL the store lands", 32'h19C, SV);
        for (k = 0; k < 8; k = k + 1) if (k != 5) chk3("P-LS ldo words 40..47", 48 + k, W(40 + k));
        chk3("P-LS ldo reads the old word 45", 53, W(45));
        chkm("P-LS the store lands", 32'h1B4, SV);
        chk3("PIU  LD counts each ldo once (LO-07)", 57, 32'd6);
        // MEMSEQ e (nacc - 1): A 7, A2 7, QQ 15, P-SL 8, P-LS 8 = 45; d (ceil(nacc / 2) - 1): 3 + 3 + 7 + 4 + 4 = 21
        chk("PIU  MEMSEQ bare = 0", b.gr[58], 32'd0);
        chk("PIU  MEMSEQ EXT = one hold per access after the first (LO-06)", e.gr[58], 32'd45);
        chk("PIU  MEMSEQ dual = two accesses per cycle (LO-06)", d.gr[58], 32'd21);
        chk("MISS EXT: 7 miss words, word 45 twice (P-LS ldo word 5 and the store)", miss_e, 32'd8);
        chk("DSTALL EXT = 3 cycles per miss", stall_e, 32'd24);
        chk("cycles EXT = bare + MEMSEQ + DSTALL", cycles_e, cycles_b + 45 + 24);
        chk("MISS dual+miss: 4 miss words on port 1", miss_m, 32'd4);   // A w2, A2 w0, P-LS w6 and the P-LS store
        chk("DSTALL dual+miss = 3 cycles per port-1 miss", stall_m, 3 * miss_m);
        chk("PIU  MEMSEQ dual+miss = MEMSEQ dual", m.gr[58], 32'd21);
        chk("cycles dual+miss = bare + MEMSEQ + DSTALL", cycles_m, cycles_b + 21 + stall_m);
        chk("cycles dual = bare + MEMSEQ", cycles_d, cycles_b + 21);
        // ---- phase 2 ----
        miss_e = 0; miss_m = 0;
        P(0, NOP,                        ldo(8, 12, 3, 0));                // MA: r3 = 0x102
        P(1, NOP,                        ldo(16, 20, 56, 0));              // IO0: r56 = 0xFE00
        P(2, ldo(16, 20, 57, 0),         NOP);                             // IO7: r57 = 0xFDE4 (slot 2)
        P(3, NOP,                        ldo(16, 20, 60, 0));              // IOT: r60 = 0xFFF0 (outside the MA quads)
        P(4, NOP,                        ldo(0, 24, 4, 0));                // R0: words 0..7 into r0..r3, r24..r27
        P(5, ri(`MEM_LDW, 28, 5, 9'h24), ri(`MEM_LDW, 29, 5, 9'h60));      // PIU: LD, MISALIGN
        P(6, ri(`MEM_LDW, 30, 5, 9'h6C), NOP);                             // PIU: MEMSEQ
        STOPAT(7);
        go;
        setr(3, 32'h102); setr(4, 32'h100); setr(5, 32'hFE00); setr(56, 32'hFE00); setr(57, 32'hFDE4); setr(60, 32'hFFF0);
        for (k = 8; k < 24; k = k + 1) setr(k, PRE);
        io_acc = 0;
        run;
        for (k = 8; k < 16; k = k + 1) chk3("MA   misaligned: destination 0", k, 32'd0);
        for (k = 16; k < 24; k = k + 1) chk3("IO   any word in the I/O page: destination kept", k, PRE);
        chk("IO   no external access to the I/O page (EXT)", io_acc, 32'd0);
        chk3("R0   $r0.0 stays 0", 0, 32'd0); chk3("R0   $r0.1 = word 1", 1, W(1));
        chk3("R0   $r0.2 = word 2", 2, W(2)); chk3("R0   $r0.3 = word 3", 3, W(3));
        for (k = 0; k < 4; k = k + 1) chk3("R0   quad B = words 4..7", 24 + k, W(4 + k));
        chk3("PIU  LD counts the I/O and misaligned ldo too (LO-07)", 28, 32'd5);
        chk3("PIU  MISALIGN counts the misaligned ldo once (LO-03)", 29, 32'd1);
        chk("PIU  MEMSEQ EXT phase 2 = R0 7 holds", e.gr[30], 32'd7);
        chk("PIU  MEMSEQ dual phase 2 = 3", d.gr[30], 32'd3);
        seq_matrix;
        if (fail) $display("RESULT: LDO FAILED (%0d of %0d)", fail, pass + fail);
        else $display("RESULT: LDO PASSED (%0d checks)", pass);
        $finish;
    end
endmodule
