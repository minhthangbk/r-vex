`timescale 1ns/1ps
`include "rvex_defs.vh"
`include "slot8.vh"

// tb_ldq.v (round r30 B2, reviews/contracts/ldq.md LQ-02..07): 128-bit loads (ldq, opcode 0x1B, encoding P2) in
// slots 2 and 3, the same program on three cores as tb_ldd.v: the bare core (b), a core with the external D port and
// chosen misses (e, DUAL_D = 0) and one with the second D port that always hits (d, DUAL_D = 1). Word i of the data
// rows sits at 0x100 + 4 i and holds 0xA0000000 + i.
//   A    slot-3 ldq 0[r2] (EXT: word 2 misses)    A2   slot-2 ldq 16[r2]
//   NEG  offset -256 (the smallest)                 OFF  offset 252 (the largest)
//   QQ   two ldq in one packet (8 accesses; EXT: word 3 of the slot-3 ldq misses)
//   QD   slot-2 ldd + slot-3 ldq (6 accesses)
//   P-SL slot-2 stw of word 27 + slot-3 ldq of words 24..27 (the ldq reads the old word; EXT: the ldq goes first)
//   P-LS slot-2 ldq of words 28..31 + slot-3 stw of word 30 (the ldq reads the old word)
//   MA   misaligned base: all four destinations 0, MISALIGN counts it once
//   IOk  words k..3 in the PIU window (k = 0..3), IOT words 0, 1 in the hw_sem page and words 2, 3 past 64 KB
//        (0xFFF8): no destination written, no external access (the I/O pages are one 512-byte run, so any I/O word
//        of an ldq makes word 0 or word 3 an I/O word: IO3 and IOT kill a check of one end only, LQ-M4)
//   R0   pair A = $r0.0:$r0.1: $r0.0 stays 0, $r0.1 and pair B written
//   PIU  LD counts each ldq once, MEMSEQ = the holds between accesses (e: nacc - 1 per packet; d: two accesses per
//        cycle), DSTALL = 3 cycles per miss, cycles = packets + MEMSEQ + DSTALL
//   MISS (Codex R57 F4) core e misses words 0..3 of the ldq phases in slot 2 and slot 3 and in every order: A word 2,
//        A2 word 0 (slot 2), QQ slot-2 word 1 and slot-3 word 3, QD ldq word 1 (next to an ldd), P-SL ldq word 1
//        (store first in the order), P-LS ldq word 0 and the store's word 30 (ldq + store); core m (DUAL_D = 1) has the
//        same misses on port 1 while port 2 always hits: port 2's access completes only with port 1's (dual_ok)
//   DRAIN (Codex R58 F6, RR-11) 8 cycles after done no core has a D request (port 1 or 2) asserted
//   SEQ  (Codex R58 F3) one packet per case on a single-port EXT core (s) and a DUAL_D core (t, port 2 hits): shapes
//        slot-2 ldq, slot-3 ldq, ldq + store, store + ldq, ldd + ldq, two ldq, each with one miss at every access
//        index and all hit; every accepted request (port, address, we) is logged and must be the LDQ-06 / LDD-06
//        order exactly once (slot 3 first when slot 2 stores, else slot 2; words in order; DUAL_D: the next access on
//        port 2), the destinations and the store land, one miss on a port-1 access (DSTALL 3), cycles = all-hit + 3,
//        all-hit e - t cycles = the extra holds, and no request after done
module tb_ldq;
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
        is_miss = a == 32'h108 || a == 32'h13C || a == 32'h110 || a == 32'h124 || a == 32'h15C || a == 32'h164 ||
                  a == 32'h170 || a == 32'h178; end endfunction
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
    reg        lw_s [0:15], lw_t [0:15];
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

    // LQ-01 (P2): pair A = 2 x syl[22:18], base = syl[16:11], pair B = 2 x syl[10:6],
    // offset / 4 = {syl[24:23], syl[17], syl[5], syl[4:2]} (signed, -64..63)
    function [31:0] ldq; input [5:0] pa, pb, a; input integer off; reg [6:0] o; begin
        o = off / 4;
        ldq = {`MEM_LDQ, o[6:5], pa[5:1], o[4], a, pb[5:1], o[3], o[2:0], 2'b00}; end endfunction
    function [31:0] ldd; input [5:0] d1, d2, a; input [6:0] off; begin   // ldd.md LDD-01 (E2), off in bytes
        ldd = {`MEM_LDD, off[6:5], d1, a, d2, off[4:2], 2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] dd, a; input [8:0] im; begin ri = {op, `SHORT_IMM, dd, a, im, 2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s, a; input [8:0] off; begin mst = {op, `NO_IMM, s, a, off, 2'b00}; end endfunction
    localparam [31:0] NOP = 0, PRE = 32'h5EED_0000, SV = 32'h5566_7788;
    task P; input [11:0] n; input [31:0] s2, s3; begin
        b.imem[n] = pk8(s3, s2, NOP, NOP); e.imem[n] = pk8(s3, s2, NOP, NOP); d.imem[n] = pk8(s3, s2, NOP, NOP);
        m.imem[n] = pk8(s3, s2, NOP, NOP); end endtask
    task chk; input [511:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("PASS %0s got=%0h", nm, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%0h expected=%0h", nm, got, exp); end
    end endtask
    task setw; input [31:0] a, v; begin b.dmem[a >> 2] = v; xm[a[9:2]] = v; ym[a[9:2]] = v; zm[a[9:2]] = v; end endtask
    task setr; input [5:0] r; input [31:0] v; begin b.gr[r] = v; e.gr[r] = v; d.gr[r] = v; m.gr[r] = v; end endtask
    task chk3; input [511:0] nm; input [5:0] r; input [31:0] exp; begin
        chk({nm, " bare"}, b.gr[r], exp); chk({nm, " EXT"}, e.gr[r], exp); chk({nm, " dual"}, d.gr[r], exp);
        chk({nm, " dual+miss"}, m.gr[r], exp); end endtask
    task chk4; input [511:0] nm; input [5:0] r; input integer w; begin   // r..r+3 = words w..w+3
        chk3({nm, " A0"}, r, W(w)); chk3({nm, " A1"}, r + 1, W(w + 1));
        chk3({nm, " B0"}, r + 2, W(w + 2)); chk3({nm, " B1"}, r + 3, W(w + 3)); end endtask
    task chkm; input [511:0] nm; input [31:0] a, exp; begin
        chk({nm, " bare"}, b.dmem[a >> 2], exp); chk({nm, " EXT"}, xm[a[9:2]], exp); chk({nm, " dual"}, ym[a[9:2]], exp);
        chk({nm, " dual+miss"}, zm[a[9:2]], exp); end endtask
    function [31:0] W; input integer i; begin W = 32'hA000_0000 + i; end endfunction

    // ---- SEQ matrix (Codex R58 F3) ----
    reg [31:0] ea [0:7]; reg ew [0:7]; integer na, sh, pm, j, cyc_s, cyc_t, okt, hold_s, hold_t, c0s, c0t;
    reg [31:0] s2q, s3q;
    reg [8*8-1:0] shn;
    reg [511:0] nmb;
    task shape; input integer n; begin   // packet slots 2 / 3 and the expected access order (base r2 = 0x100)
        for (j = 0; j < 8; j = j + 1) begin ea[j] = 32'h100 + 4 * (j % 4); ew[j] = 1'b0; end
        case (n)
        0: begin shn = "ldq2"; s2q = ldq(10, 12, 2, 0); s3q = NOP; na = 4; end
        1: begin shn = "ldq3"; s2q = NOP; s3q = ldq(10, 12, 2, 0); na = 4; end
        2: begin shn = "ldq+st"; s2q = ldq(10, 12, 2, 0); s3q = mst(`MEM_STW, 62, 2, 32); na = 5;
                 ea[4] = 32'h120; ew[4] = 1'b1; end                                   // slot 2 first
        3: begin shn = "st+ldq"; s2q = mst(`MEM_STW, 62, 2, 32); s3q = ldq(10, 12, 2, 0); na = 5;
                 ea[4] = 32'h120; ew[4] = 1'b1; end                                   // slot 3 first: slot 2 stores
        4: begin shn = "ldd+ldq"; s2q = ldd(20, 21, 2, 32); s3q = ldq(10, 12, 2, 0); na = 6;
                 ea[0] = 32'h120; ea[1] = 32'h124; for (j = 2; j < 6; j = j + 1) ea[j] = 32'h100 + 4 * (j - 2); end
        default: begin shn = "ldq+ldq"; s2q = ldq(10, 12, 2, 0); s3q = ldq(14, 16, 2, 16); na = 8;
                 for (j = 4; j < 8; j = j + 1) ea[j] = 32'h110 + 4 * (j - 4); end
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
        for (j = 10; j < 22; j = j + 1) begin s.gr[j] = PRE; t.gr[j] = PRE; end
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
        for (j = 0; j < 4; j = j + 1) okt = okt && s.gr[10 + j] == W(j) && t.gr[10 + j] == W(j);
        if (sh == 5) for (j = 0; j < 4; j = j + 1) okt = okt && s.gr[14 + j] == W(4 + j) && t.gr[14 + j] == W(4 + j);
        if (sh == 4) okt = okt && s.gr[20] == W(8) && s.gr[21] == W(9) && t.gr[20] == W(8) && t.gr[21] == W(9);
        if (sh == 2 || sh == 3) okt = okt && smem[8'h48] == SV && tmem[8'h48] == SV;
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

    initial begin
        for (k = 0; k < 256; k = k + 1) begin xm[k] = 32'd0; ym[k] = 32'd0; zm[k] = 32'd0; end
        for (k = 0; k < 72; k = k + 1) setw(32'h100 + 4 * k, W(k));
        P(0,  NOP,                       ldq(10, 12, 2, 0));               // A: words 0..3
        P(1,  ldq(14, 16, 2, 16),        NOP);                             // A2: words 4..7
        P(2,  NOP,                       ldq(18, 20, 6, -256));            // NEG: r6 = 0x240, words 16..19
        P(3,  NOP,                       ldq(22, 24, 2, 252));             // OFF: words 63..66
        P(4,  ldq(26, 28, 2, 32),        ldq(30, 32, 2, 48));              // QQ: words 8..11, 12..15
        P(5,  ldd(34, 35, 2, 80),        ldq(36, 38, 2, 88));              // QD: words 20, 21 and 22..25
        P(6,  mst(`MEM_STW, 62, 2, 108), ldq(40, 42, 2, 96));              // P-SL: store word 27, ldq words 24..27
        P(7,  ldq(44, 46, 2, 112),       mst(`MEM_STW, 62, 2, 120));       // P-LS: ldq words 28..31, store word 30
        P(8,  NOP,                       ldq(48, 50, 3, 0));               // MA: r3 = 0x102
        P(9,  NOP,                       ldq(52, 54, 56, 0));              // IO0: r56 = 0xFE00
        P(10, ldq(52, 54, 57, 0),        NOP);                             // IO1: r57 = 0xFDFC (slot 2)
        P(11, NOP,                       ldq(52, 54, 58, 0));              // IO2: r58 = 0xFDF8
        P(12, NOP,                       ldq(52, 54, 59, 0));              // IO3: r59 = 0xFDF4
        P(13, NOP,                       ldq(52, 54, 9, 0));               // IOT: r9 = 0xFFF8
        P(14, NOP,                       ldq(0, 60, 2, 128));              // R0: words 32..35 into r0, r1, r60, r61
        P(15, ri(`MEM_LDW, 4, 5, 9'h24), ri(`MEM_LDW, 7, 5, 9'h60));       // PIU: LD, MISALIGN
        P(16, ri(`MEM_LDW, 8, 5, 9'h6C), NOP);                             // PIU: MEMSEQ
        P(17, NOP, NOP); b.imem[17] = {224'd0, `OP_STOP, 25'd0}; e.imem[17] = b.imem[17]; d.imem[17] = b.imem[17];
        m.imem[17] = b.imem[17];
        repeat (2) @(posedge clk);
        reset = 0;
        @(posedge clk); #1;                     // after the last reset edge (its <= would clear the registers)
        setr(2, 32'h100); setr(3, 32'h102); setr(5, 32'hFE00); setr(6, 32'h240); setr(62, SV);
        setr(56, 32'hFE00); setr(57, 32'hFDFC); setr(58, 32'hFDF8); setr(59, 32'hFDF4); setr(9, 32'hFFF8);
        for (k = 48; k < 56; k = k + 1) setr(k, PRE);
        @(posedge clk); start = 1; @(posedge clk); start = 0;
        while (!(done_b && done_e && done_d && done_m) && cyc < 600) begin @(posedge clk); cyc = cyc + 1; end
        chk("all four cores end by STOP within the cap", done_b && done_e && done_d && done_m, 1);
        repeat (8) @(posedge clk); #1;
        chk("DRAIN no D request 8 cycles after done (RR-11)", {d_req, q_req, q2_req, m_req, m2_req}, 5'd0);
        chk4("A    slot-3 ldq (EXT miss on word 2)", 10, 0);
        chk4("A2   slot-2 ldq", 14, 4);
        chk4("NEG  offset -256", 18, 16);
        chk4("OFF  offset 252", 22, 63);
        chk4("QQ   slot 2", 26, 8);
        chk4("QQ   slot 3 (EXT miss on word 3)", 30, 12);
        chk3("QD   ldd dst1", 34, W(20)); chk3("QD   ldd dst2", 35, W(21));
        chk4("QD   ldq", 36, 22);
        chk3("P-SL ldq A0", 40, W(24)); chk3("P-SL ldq A1", 41, W(25)); chk3("P-SL ldq B0", 42, W(26));
        chk3("P-SL ldq reads the old word 27", 43, W(27));
        chkm("P-SL the store lands", 32'h16C, SV);
        chk3("P-LS ldq A0", 44, W(28)); chk3("P-LS ldq A1", 45, W(29));
        chk3("P-LS ldq reads the old word 30", 46, W(30)); chk3("P-LS ldq B1", 47, W(31));
        chkm("P-LS the store lands", 32'h178, SV);
        for (k = 48; k < 52; k = k + 1) chk3("MA   misaligned: destination 0", k, 32'd0);
        for (k = 52; k < 56; k = k + 1) chk3("IO   any word in the I/O page: destination kept", k, PRE);
        chk("IO   no external access to the I/O page (EXT)", io_acc, 32'd0);
        chk3("R0   $r0.0 stays 0", 0, 32'd0); chk3("R0   $r0.1 = word 33", 1, W(33));
        chk3("R0   pair B = words 34, 35 (B0)", 60, W(34)); chk3("R0   pair B (B1)", 61, W(35));
        // LD: 17 before packet 15 (A, A2, NEG, OFF, QQ 2, QD 2, P-SL, P-LS, MA, IO 5, R0)
        chk3("PIU  LD counts each ldq once (LQ-07)", 4, 32'd17);
        chk3("PIU  MISALIGN counts the misaligned ldq once (LQ-03)", 7, 32'd1);
        // MEMSEQ e (nacc - 1): A 3, A2 3, NEG 3, OFF 3, QQ 7, QD 5, P-SL 4, P-LS 4, R0 3 (MA, IO, IOT: no access) = 35;
        // d (ceil(nacc / 2) - 1): A 1, A2 1, NEG 1, OFF 1, QQ 3, QD 2, P-SL 2, P-LS 2, R0 1 = 14
        chk("PIU  MEMSEQ bare = 0", b.gr[8], 32'd0);
        chk("PIU  MEMSEQ EXT = one hold per access after the first (LQ-06)", e.gr[8], 32'd35);
        chk("PIU  MEMSEQ dual = two accesses per cycle (LQ-06)", d.gr[8], 32'd14);
        chk("MISS EXT: each of the 8 miss words missed once", miss_e, 32'd8);
        chk("DSTALL EXT = 3 cycles per miss", stall_e, 32'd24);
        chk("cycles EXT = bare + MEMSEQ + DSTALL", cycles_e, cycles_b + 35 + 24);
        chk("MISS dual+miss: 4 miss words on port 1", miss_m, 32'd4);   // the other 4 go out on port 2 (hits)
        chk("DSTALL dual+miss = 3 cycles per port-1 miss", stall_m, 3 * miss_m);
        chk("PIU  MEMSEQ dual+miss = MEMSEQ dual", m.gr[8], 32'd14);   // a port-1 miss holds, it splits no pair
        chk("cycles dual+miss = bare + MEMSEQ + DSTALL", cycles_m, cycles_b + 14 + stall_m);
        chk("cycles dual = bare + MEMSEQ", cycles_d, cycles_b + 14);
        seq_matrix;
        if (fail) $display("RESULT: LDQ FAILED (%0d of %0d)", fail, pass + fail);
        else $display("RESULT: LDQ PASSED (%0d checks)", pass);
        $finish;
    end
endmodule
