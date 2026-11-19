`timescale 1ns/1ps
`include "rvex_defs.vh"
`include "slot8.vh"

// tb_mem2.v (round r28, reviews/contracts/mem2.md MEM2-01..05, research r28 7.1 rows C-01..C-17): two memory ops per
// packet (slots 2 and 3), the same program on the bare core (EXT_DMEM = 0) and on a core with the external D port
// (EXT_DMEM = 1, a memory model with chosen misses). Word k sits at 0x100 + 16 k (A_k = 0x11223344) and B_k at
// A_k + 4 (0xAABBCCDD).
//   C-01 ld+st one word   C-02 st+ld one word   C-03 stw+stw   C-04 stb+stb   C-05 sth+stb   C-06 ld+ld
//   C-07 WAR as compiled  C-08 st+ld other words  C-09 misaligned in slot 2  C-10 PIU reads  C-11 PIU store pair
//   C-15 EXT miss in the first access  C-16 EXT miss in the second  C-17 FP-12 hold + slot-2 store
//   C-14 (hit + hit) is every two-access row of the EXT core: one MEMSEQ cycle each.
// The console (C-12) is checked by tb_function.v rows, hw_sem (C-13) by the system bench.
module tb_mem2;
    reg clk = 0, reset = 1, start = 0;
    always #5 clk = ~clk;
    integer pass = 0, fail = 0, k, cyc_b = 0, cyc_e = 0, st14 = 0, ord_ok = 1, nreq = 0;

    // ---- bare core ----
    wire done_b; wire [31:0] cycles_b;
    rvex_core #(.EXT_DMEM(0)) b(.clk(clk), .reset(reset), .start(start), .done(done_b), .cycles(cycles_b),
        .d_req(), .d_we(), .d_addr(), .d_wdata(), .d_word(), .d_rdata(32'd0), .d_ready(1'b1), .stall_cycles(),
        .i_req(), .i_addr(), .i_rdata(128'd0), .i_ready(1'b1), .istall_cycles());

    // ---- external D port core + memory model ----
    wire done_e, d_req, d_we, d_word; wire [31:0] cycles_e, d_addr, d_wdata;
    reg  [31:0] xm [0:255];             // words 0x000..0x3FC
    wire [31:0] d_rdata = xm[d_addr[9:2]];
    reg  [3:0]  wait_n = 0;             // stall cycles left for the access on the port
    reg  [31:0] wait_addr = 32'hFFFF_FFFF;
    wire        miss_addr = (d_addr == 32'h0000_01C0) || (d_addr == 32'h0000_01D4);   // A12 (C-15), B13 (C-16)
    wire        d_ready = !(d_req && miss_addr && !(wait_addr == d_addr && wait_n == 0));
    rvex_core #(.EXT_DMEM(1)) e(.clk(clk), .reset(reset), .start(start), .done(done_e), .cycles(cycles_e),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(d_word), .d_rdata(d_rdata),
        .d_ready(d_ready), .stall_cycles(), .i_req(), .i_addr(), .i_rdata(128'd0), .i_ready(1'b1), .istall_cycles());
    always @(posedge clk) if (!reset) begin
        if (d_req && miss_addr && wait_addr != d_addr) begin wait_addr <= d_addr; wait_n <= 4'd2; end   // 3 cycles held
        else if (d_req && miss_addr && wait_n != 0) wait_n <= wait_n - 1;
        if (d_req && d_ready) begin
            nreq = nreq + 1;
            if (d_we) xm[d_addr[9:2]] <= d_wdata;
            if (d_we && d_addr == 32'h0000_01E0) st14 = st14 + 1;          // C-17: the store is presented once
            // C-02 (A2 = 0x120): the slot-3 load goes before the slot-2 store
            if (d_addr == 32'h0000_0120 && d_we && e.seq_ph == 1'b0) ord_ok = 0;
        end
    end

    function [31:0] ri;  input [6:0] op; input [5:0] d, a; input [8:0] im; begin ri = {op, `SHORT_IMM, d, a, im, 2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s, a; input [8:0] off; begin mst = {op, `NO_IMM, s, a, off, 2'b00}; end endfunction
    function [31:0] rr;  input [6:0] op; input [5:0] d, a, bb; begin rr = {op, `NO_IMM, d, a, bb, 5'b0}; end endfunction
    localparam [31:0] NOP = 0;
    task P; input [11:0] n; input [31:0] s0, s1, s2, s3; begin b.imem[n] = pk8(s3, s2, s1, s0); e.imem[n] = pk8(s3, s2, s1, s0); end endtask
    task chk; input [511:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("PASS %0s got=%0h", nm, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%0h expected=%0h", nm, got, exp); end
    end endtask
    task setw; input [31:0] a, v; begin b.dmem[a >> 2] = v; xm[a[9:2]] = v; end endtask
    function [31:0] getw_b; input [31:0] a; begin getw_b = b.dmem[a >> 2]; end endfunction
    function [31:0] getw_e; input [31:0] a; begin getw_e = xm[a[9:2]]; end endfunction

    // r2 = 0x100 base of the rows: A_k offset 16 k, B_k 16 k + 4; r5 = 0xFE00 (PIU)
    initial begin
        for (k = 0; k < 256; k = k + 1) xm[k] = 32'd0;
        for (k = 0; k < 16; k = k + 1) begin setw(32'h100 + 16 * k, 32'h11223344); setw(32'h104 + 16 * k, 32'hAABBCCDD); end
        b.dmem[16'hFE00 >> 2] = 32'hDEAD_BEEF;
        P(0,  NOP, NOP, ri(`MEM_LDW, 10, 2, 16),        mst(`MEM_STW, 20, 2, 16));          // C-01
        P(1,  NOP, NOP, mst(`MEM_STW, 20, 2, 32),       ri(`MEM_LDW, 11, 2, 32));           // C-02
        P(2,  NOP, NOP, mst(`MEM_STW, 21, 2, 48),       mst(`MEM_STW, 22, 2, 48));          // C-03
        P(3,  NOP, NOP, mst(`MEM_STB, 23, 2, 64),       mst(`MEM_STB, 24, 2, 65));          // C-04
        P(4,  NOP, NOP, mst(`MEM_STH, 25, 2, 80),       mst(`MEM_STB, 26, 2, 80));          // C-05
        P(5,  NOP, NOP, ri(`MEM_LDW, 12, 2, 96),        ri(`MEM_LDH, 13, 2, 98));           // C-06
        P(6,  NOP, NOP, mst(`MEM_STW, 27, 2, 112),      ri(`MEM_LDW, 14, 2, 112));          // C-07 (program order: load, store)
        P(7,  NOP, NOP, mst(`MEM_STW, 27, 2, 128),      ri(`MEM_LDW, 15, 2, 132));          // C-08
        P(8,  NOP, NOP, mst(`MEM_STH, 25, 2, 147),      ri(`MEM_LDH, 16, 2, 147));          // C-09
        P(9,  NOP, NOP, ri(`MEM_LDW, 17, 5, 4),         ri(`MEM_LDW, 18, 5, 16));           // C-10 ID, PKT
        P(10, NOP, NOP, mst(`MEM_STW, 0, 5, 0),         mst(`MEM_STW, 28, 5, 0));           // C-11 EN 0 then 1
        P(11, NOP, NOP, ri(`MEM_LDW, 29, 2, 192),       ri(`MEM_LDW, 30, 2, 196));          // C-15 A12 misses first
        P(12, NOP, NOP, ri(`MEM_LDW, 31, 2, 208),       ri(`MEM_LDW, 32, 2, 212));          // C-16 B13 misses second
        P(13, NOP, rr(`FPU_FADD, 33, 34, 35), mst(`MEM_STW, 27, 2, 224), NOP);              // C-17
        P(14, {`OP_STOP, 25'd0}, NOP, NOP, NOP);
        repeat (2) @(posedge clk);
        reset = 0; #1;
        b.gr[2] = 32'h100; e.gr[2] = 32'h100; b.gr[5] = 32'hFE00; e.gr[5] = 32'hFE00;
        b.gr[20] = 32'h55667788; e.gr[20] = 32'h55667788; b.gr[21] = 32'h01010101; e.gr[21] = 32'h01010101;
        b.gr[22] = 32'h02020202; e.gr[22] = 32'h02020202; b.gr[23] = 32'h55; e.gr[23] = 32'h55;
        b.gr[24] = 32'h66; e.gr[24] = 32'h66; b.gr[25] = 32'h7788; e.gr[25] = 32'h7788;
        b.gr[26] = 32'h99; e.gr[26] = 32'h99; b.gr[27] = 32'h5; e.gr[27] = 32'h5; b.gr[28] = 32'h1; e.gr[28] = 32'h1;
        b.gr[34] = 32'h3FC00000; e.gr[34] = 32'h3FC00000; b.gr[35] = 32'h40100000; e.gr[35] = 32'h40100000;
        start = 1;
        while (!(done_b && done_e) && cyc_b < 300) begin @(posedge clk); cyc_b = cyc_b + 1; end
        #1;
        chk("both cores end by STOP within the cap", done_b && done_e, 1);
        // ---- values: the same on both cores ----
        chk("C-01 bare ld+st: load reads the old word", b.gr[10], 32'h11223344);
        chk("C-01 EXT  ld+st: load reads the old word", e.gr[10], 32'h11223344);
        chk("C-01 bare store applied", getw_b(32'h110), 32'h55667788);
        chk("C-01 EXT  store applied", getw_e(32'h110), 32'h55667788);
        chk("C-02 bare st+ld: load reads the old word", b.gr[11], 32'h11223344);
        chk("C-02 EXT  st+ld: load reads the old word", e.gr[11], 32'h11223344);
        chk("C-02 EXT  slot-3 load presented before the slot-2 store", ord_ok, 1);
        chk("C-02 bare store applied", getw_b(32'h120), 32'h55667788);
        chk("C-02 EXT  store applied", getw_e(32'h120), 32'h55667788);
        chk("C-03 bare stw+stw: slot 3 wins", getw_b(32'h130), 32'h02020202);
        chk("C-03 EXT  stw+stw: slot 3 wins", getw_e(32'h130), 32'h02020202);
        chk("C-04 bare stb+stb merge", getw_b(32'h140), 32'h55663344);
        chk("C-04 EXT  stb+stb merge", getw_e(32'h140), 32'h55663344);
        chk("C-05 bare sth+stb: slot 3 byte on top", getw_b(32'h150), 32'h99883344);
        chk("C-05 EXT  sth+stb: slot 3 byte on top", getw_e(32'h150), 32'h99883344);
        chk("C-06 bare ld+ld ldw", b.gr[12], 32'h11223344);
        chk("C-06 bare ld+ld ldh", b.gr[13], 32'h00003344);
        chk("C-06 EXT  ld+ld ldw", e.gr[12], 32'h11223344);
        chk("C-06 EXT  ld+ld ldh", e.gr[13], 32'h00003344);
        chk("C-07 bare WAR: load old", b.gr[14], 32'h11223344);
        chk("C-07 EXT  WAR: load old", e.gr[14], 32'h11223344);
        chk("C-07 bare WAR: store applied", getw_b(32'h170), 32'h5);
        chk("C-08 bare st+ld other words: load", b.gr[15], 32'hAABBCCDD);
        chk("C-08 EXT  st+ld other words: load", e.gr[15], 32'hAABBCCDD);
        chk("C-08 EXT  store applied", getw_e(32'h180), 32'h5);
        chk("C-09 bare crossing sth in slot 2 leaves the word", getw_b(32'h190), 32'h11223344);
        chk("C-09 EXT  crossing sth in slot 2 leaves the word", getw_e(32'h190), 32'h11223344);
        chk("C-09 bare crossing ldh reads 0", b.gr[16], 32'd0);
        chk("C-09 bare MISALIGN counts both slots", b.u_piu.misalign, 2);
        chk("C-09 EXT  MISALIGN counts both slots", e.u_piu.misalign, 2);
        chk("C-10 bare slot-2 PIU read (ID)", b.gr[17], 32'h50495501);
        chk("C-10 EXT  slot-2 PIU read (ID)", e.gr[17], 32'h50495501);
        chk("C-10 bare slot-3 PKT read (9 packets before)", b.gr[18], 9);
        chk("C-10 EXT  slot-3 PKT read (9 packets before)", e.gr[18], 9);
        chk("C-11 bare PIU store pair: slot 3 wins (EN = 1)", b.u_piu.en, 1);
        chk("C-11 EXT  PIU store pair: slot 3 wins (EN = 1)", e.u_piu.en, 1);
        chk("C-11 bare DMEM 0xFE00 untouched", b.dmem[16'hFE00 >> 2], 32'hDEAD_BEEF);
        chk("C-15 EXT  miss first: both loads", {e.gr[29][15:0], e.gr[30][15:0]}, {16'h3344, 16'hCCDD});
        chk("C-16 EXT  miss second: both loads", {e.gr[31][15:0], e.gr[32][15:0]}, {16'h3344, 16'hCCDD});
        chk("C-17 bare FP hold + slot-2 store", getw_b(32'h1E0), 32'h5);
        chk("C-17 EXT  the store is presented once", st14, 1);
        chk("C-17 fadd committed", e.gr[33], 32'h40700000);
        // ---- PIU accounting (MEM2-05) ----
        chk("MEM2-05 bare LD counts both slots", b.u_piu.ld, 13);
        chk("MEM2-05 bare ST counts both slots", b.u_piu.st, 14);
        chk("MEM2-05 bare MEMSEQ = 0", b.u_piu.memseq, 0);
        chk("MEM2-05 EXT  MEMSEQ = two-access packets (C-14: 9 + C-15 + C-16)", e.u_piu.memseq, 11);
        chk("MEM2-05 EXT  DSTALL = the two 3-cycle misses", e.u_piu.dstall, 6);
        chk("MEM2-05 EXT  FPSTALL = 1", e.u_piu.fpstall, 1);
        chk("MEM2-05 bare cycle law", b.u_piu.cyc[31:0], b.u_piu.pkt + b.u_piu.xnop + b.u_piu.istall + b.u_piu.dstall + b.u_piu.fpstall + b.u_piu.memseq + 1);   // PIU-13: + the STOP cycle
        chk("MEM2-05 EXT  cycle law", e.u_piu.cyc[31:0], e.u_piu.pkt + e.u_piu.xnop + e.u_piu.istall + e.u_piu.dstall + e.u_piu.fpstall + e.u_piu.memseq + 1);   // PIU-13: + the STOP cycle
        chk("MEM2-05 bare = EXT packets", b.u_piu.pkt, e.u_piu.pkt);
        if (fail == 0) $display("RESULT: MEM2 PASSED (%0d checks)", pass);
        else $display("RESULT: MEM2 FAILED (%0d of %0d)", fail, pass + fail);
        $finish;
    end
endmodule
