`timescale 1ns/1ps
`include "rvex_defs.vh"
`include "slot8.vh"

// tb_ldd.v (round r29 B2, reviews/contracts/ldd.md LDD-01..07): 64-bit loads (ldd, opcode 0x1A, encoding E2) in
// slots 2 and 3, the same program on three cores: the bare core (b), a core with the external D port and chosen
// misses (e, DUAL_D = 0) and one with the second D port that always hits (d, DUAL_D = 1). Word i of the data rows
// sits at 0x100 + 4 i and holds 0xA0000000 + i.
//   A    slot-3 ldd 0[r2]            A2  slot-2 ldd 8[r2]          OFF  offset 124 (the largest)
//   DD   two ldd in one packet (4 accesses)                      R0   dst1 = $r0.0, dst2 written
//   P-SL slot-2 stw of word 1 + slot-3 ldd (the ldd reads the old word; EXT: the ldd goes first)
//   P-LS slot-2 ldd + slot-3 stw of word 0 (the ldd reads the old word)
//   MA   misaligned base: both destinations 0, MISALIGN counts it      IO  ea + 4 in the PIU window: no write
//   MISS EXT miss on word 1 of a slot-3 ldd                       MISS2 EXT miss on word 0 of a slot-2 ldd (+ slot-3 ldd)
//   PIU  LD counts each ldd once, MEMSEQ = the holds between accesses (e: one per access after the first; d: two
//        accesses per cycle), DSTALL = the two 3-cycle misses, cycles = packets + MEMSEQ + DSTALL
module tb_ldd;
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
    wire        miss_addr = (d_addr == 32'h0000_013C) || (d_addr == 32'h0000_0140);   // MISS word 1, MISS2 word 0
    wire        d_ready = !(d_req && miss_addr && !(wait_addr == d_addr && wait_n == 0));
    rvex_core #(.EXT_DMEM(1)) e(.clk(clk), .reset(reset), .start(start), .done(done_e), .cycles(cycles_e),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(), .d_rdata(d_rdata),
        .d_ready(d_ready), .d2_req(), .d2_we(), .d2_addr(), .d2_wdata(), .d2_rdata(32'd0), .d2_hit(1'b0),
        .stall_cycles(stall_e), .i_req(), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles());
    always @(posedge clk) if (!reset) begin
        if (d_req && miss_addr && wait_addr != d_addr) begin wait_addr <= d_addr; wait_n <= 4'd2; end   // 3 cycles held
        else if (d_req && miss_addr && wait_n != 0) wait_n <= wait_n - 1;
        if (d_req && d_ready && d_we) xm[d_addr[9:2]] <= d_wdata;
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

    function [31:0] ldd; input [5:0] d1, d2, a; input [6:0] off; begin   // LDD-01 (E2), off in bytes
        ldd = {`MEM_LDD, off[6:5], d1, a, d2, off[4:2], 2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] dd, a; input [8:0] im; begin ri = {op, `SHORT_IMM, dd, a, im, 2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s, a; input [8:0] off; begin mst = {op, `NO_IMM, s, a, off, 2'b00}; end endfunction
    localparam [31:0] NOP = 0, PRE = 32'h5EED_0000;
    task P; input [11:0] n; input [31:0] s2, s3; begin
        b.imem[n] = pk8(s3, s2, NOP, NOP); e.imem[n] = pk8(s3, s2, NOP, NOP); d.imem[n] = pk8(s3, s2, NOP, NOP); end endtask
    task chk; input [511:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("PASS %0s got=%0h", nm, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%0h expected=%0h", nm, got, exp); end
    end endtask
    task setw; input [31:0] a, v; begin b.dmem[a >> 2] = v; xm[a[9:2]] = v; ym[a[9:2]] = v; end endtask
    task setr; input [5:0] r; input [31:0] v; begin b.gr[r] = v; e.gr[r] = v; d.gr[r] = v; end endtask
    task chk3; input [511:0] nm; input [5:0] r; input [31:0] exp; begin
        chk({nm, " bare"}, b.gr[r], exp); chk({nm, " EXT"}, e.gr[r], exp); chk({nm, " dual"}, d.gr[r], exp); end endtask
    task chkm; input [511:0] nm; input [31:0] a, exp; begin
        chk({nm, " bare"}, b.dmem[a >> 2], exp); chk({nm, " EXT"}, xm[a[9:2]], exp); chk({nm, " dual"}, ym[a[9:2]], exp); end endtask
    function [31:0] W; input integer i; begin W = 32'hA000_0000 + i; end endfunction

    initial begin
        for (k = 0; k < 256; k = k + 1) begin xm[k] = 32'd0; ym[k] = 32'd0; end
        for (k = 0; k < 40; k = k + 1) setw(32'h100 + 4 * k, W(k));
        P(0,  NOP,                       ldd(10, 11, 2, 0));               // A
        P(1,  ldd(12, 13, 2, 8),         NOP);                             // A2
        P(2,  NOP,                       ldd(14, 15, 2, 124));             // OFF: words 31, 32
        P(3,  ldd(16, 17, 2, 16),        ldd(18, 19, 2, 24));              // DD: words 4..7
        P(4,  mst(`MEM_STW, 20, 2, 36),  ldd(21, 22, 2, 32));              // P-SL: store word 9, ldd words 8, 9
        P(5,  ldd(23, 24, 2, 40),        mst(`MEM_STW, 20, 2, 40));        // P-LS: ldd words 10, 11, store word 10
        P(6,  NOP,                       ldd(25, 26, 3, 0));               // MA: r3 = 0x102
        P(7,  NOP,                       ldd(27, 28, 4, 0));               // IO: r4 = 0xFDFC (ea + 4 = 0xFE00)
        P(8,  NOP,                       ldd(0, 29, 2, 48));               // R0: word 13 into r29
        P(9,  NOP,                       ldd(30, 31, 2, 56));              // MISS: word 15 (0x13C) misses
        P(10, ldd(32, 33, 2, 64),        ldd(34, 35, 2, 72));              // MISS2: word 16 (0x140) misses
        P(11, ri(`MEM_LDW, 40, 5, 9'h24), ri(`MEM_LDW, 41, 5, 9'h60));     // PIU: LD, MISALIGN
        P(12, ri(`MEM_LDW, 42, 5, 9'h6C), NOP);                            // PIU: MEMSEQ
        P(13, NOP, NOP); b.imem[13] = {224'd0, `OP_STOP, 25'd0}; e.imem[13] = b.imem[13]; d.imem[13] = b.imem[13];
        repeat (2) @(posedge clk);
        reset = 0;
        @(posedge clk); #1;                     // after the last reset edge (its <= would clear the registers)
        setr(2, 32'h100); setr(3, 32'h102); setr(4, 32'hFDFC); setr(5, 32'hFE00); setr(20, 32'h5566_7788);
        setr(25, PRE); setr(26, PRE); setr(27, PRE); setr(28, PRE);
        @(posedge clk); start = 1; @(posedge clk); start = 0;
        while (!(done_b && done_e && done_d) && cyc < 400) begin @(posedge clk); cyc = cyc + 1; end
        chk("all three cores end by STOP within the cap", done_b && done_e && done_d, 1);
        chk3("A    slot-3 ldd dst1 = word 0", 10, W(0));   chk3("A    slot-3 ldd dst2 = word 1", 11, W(1));
        chk3("A2   slot-2 ldd dst1", 12, W(2));            chk3("A2   slot-2 ldd dst2", 13, W(3));
        chk3("OFF  offset 124 dst1", 14, W(31));           chk3("OFF  offset 124 dst2", 15, W(32));
        chk3("DD   slot 2 dst1", 16, W(4)); chk3("DD   slot 2 dst2", 17, W(5));
        chk3("DD   slot 3 dst1", 18, W(6)); chk3("DD   slot 3 dst2", 19, W(7));
        chk3("P-SL ldd word 0", 21, W(8)); chk3("P-SL ldd reads the old word 1", 22, W(9));
        chkm("P-SL the store lands", 32'h124, 32'h5566_7788);
        chk3("P-LS ldd reads the old word 0", 23, W(10)); chk3("P-LS ldd word 1", 24, W(11));
        chkm("P-LS the store lands", 32'h128, 32'h5566_7788);
        chk3("MA   misaligned dst1 = 0", 25, 32'd0); chk3("MA   misaligned dst2 = 0", 26, 32'd0);
        chk3("IO   I/O page: dst1 kept", 27, PRE); chk3("IO   I/O page: dst2 kept", 28, PRE);
        chk3("R0   $r0.0 stays 0", 0, 32'd0); chk3("R0   dst2 written", 29, W(13));
        chk3("MISS miss on word 1: dst1", 30, W(14)); chk3("MISS miss on word 1: dst2", 31, W(15));
        chk3("MISS2 slot 2 dst1 (miss)", 32, W(16)); chk3("MISS2 slot 2 dst2", 33, W(17));
        chk3("MISS2 slot 3 dst1", 34, W(18)); chk3("MISS2 slot 3 dst2", 35, W(19));
        // LD: 13 ldd before packet 11 (A, A2, OFF, DD 2, P-SL, P-LS, MA, IO, R0, MISS, MISS2 2)
        chk3("PIU  LD counts each ldd once (LDD-07)", 40, 32'd13);
        chk3("PIU  MISALIGN counts the misaligned ldd (LDD-03)", 41, 32'd1);
        // MEMSEQ e: A 1, A2 1, OFF 1, DD 3, P-SL 2, P-LS 2, R0 1, MISS 1, MISS2 3 = 15; d: DD 1, P-SL 1, P-LS 1, MISS2 1
        chk("PIU  MEMSEQ bare = 0", b.gr[42], 32'd0);
        chk("PIU  MEMSEQ EXT = one hold per access after the first (LDD-06)", e.gr[42], 32'd15);
        chk("PIU  MEMSEQ dual = two accesses per cycle (LDD-06)", d.gr[42], 32'd4);
        chk("DSTALL EXT = the two 3-cycle misses", stall_e, 32'd6);
        chk("cycles EXT = bare + MEMSEQ + DSTALL", cycles_e, cycles_b + 15 + 6);
        chk("cycles dual = bare + MEMSEQ", cycles_d, cycles_b + 4);
        if (fail) $display("RESULT: LDD FAILED (%0d of %0d)", fail, pass + fail);
        else $display("RESULT: LDD PASSED (%0d checks)", pass);
        $finish;
    end
endmodule
