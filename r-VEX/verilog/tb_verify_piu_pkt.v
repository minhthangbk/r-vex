//======================================================================
// r-VEX | Final-verify (round 2026-10-31-r5): PIU accesses in packets that
// also change the PC or follow a stall (rvex_core integration)
//----------------------------------------------------------------------
// Contract: reviews/contracts/piu.md. Expected values are derived from the
// contract, not from the RTL:
//   PIU-03  a CTRL store takes effect at the end of ITS (commit) cycle; the
//           events of that cycle use the old EN. Cycles in which the packet
//           is held (XNOP bubble before it, I-fetch miss) are not its cycle.
//   PIU-04  CLR clears at the end of the store cycle: the next packet reads 0.
//   PIU-05  no dmem write, no d_req, never stalls, also with a taken branch,
//           call or return in the same packet.
//   PIU-10  BRT counts taken br / call / return.
// Part A (bare core):
//    0: r10=0x7F | r11=3 | cmpeq b1 = r0==r0
//    1: r10 = 0xFE00
//    2: xnop 3                               (3 bubble cycles)
//    3: stw CTRL = r0  (EN-only 0)           counted with old EN = 1
//       -> CYCLE = 3 + 3 + 1 = 7, PKT = 4, XNOP = 3 (frozen)
//    4..6: ldw r20 = CYCLE, r21 = XNOP, r22 = PKT
//    7: goto 9 | stw CTRL = r11 (CLR+EN)     not counted (old EN = 0)
//    8: poison r30
//    9: ldw r23 = PKT                        -> 0 (PIU-04)
//   10: ldw r24 = CYCLE                      -> 1
//   11: br b1 -> 13 | ldw r25 = BRT          -> 0, taken
//   12: poison r31
//   13: ldw r26 = BRT                        -> 1
//   14: call 20 (GR63 = 15) | ldw r27 = PKT  -> 4 (9,10,11,13)
//   20: ldw r28 = CTRL (=1) | r29 = r63 (=15)
//   21: return | stw CTRL = r0               counted (old EN = 1), then EN = 0
//   15: ldw r32 = PKT                        -> 7 (9,10,11,13,14,20,21)
//   16: ldw r33 = CTRL                       -> 0
//   17: STOP (not counted, EN = 0)
// Part B (EXT_IMEM + EXT_DMEM core): the packet "stw CTRL = r0" is held by
//   a 3-cycle I-fetch miss, then delivered. ISTALL = 3 and the commit cycle
//   are counted with EN = 1; afterwards CYCLE stays 4. d_req never rises.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_verify_piu_pkt;
    reg clk = 0, reset = 1, start = 0;
    integer pass = 0, fail = 0, k;
    always #5 clk = ~clk;

    // ---------------- part A: bare core ----------------
    wire done_a; wire [31:0] cycles_a;
    rvex_core a(.clk(clk), .reset(reset), .start(start), .done(done_a), .cycles(cycles_a));

    // ---------------- part B: external I/D ports ----------------
    reg  [127:0] i_rdata = 0;
    reg  i_ready = 0;
    wire done_b, d_req, d_we, d_word, i_req;
    wire [31:0] cycles_b, d_addr, d_wdata;
    wire [11:0] i_addr;
    rvex_core #(.EXT_IMEM(1), .EXT_DMEM(1)) b(
        .clk(clk), .reset(reset), .start(start), .done(done_b), .cycles(cycles_b),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata),
        .d_word(d_word), .d_rdata(32'h0bad_0bad), .d_ready(1'b1),
        .stall_cycles(), .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata),
        .i_ready(i_ready), .istall_cycles());
    integer dreq_seen = 0;
    always @(posedge clk) if (!reset && d_req) dreq_seen = dreq_seen + 1;

    function [31:0] rrb; input [6:0] op; input [5:0] a_,b_; input [2:0] dbr; begin
        rrb = {op,`NO_IMM,6'd0,a_,b_,dbr,2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] d,a_; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a_,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s,b_; input [8:0] off; begin
        mst = {op,`NO_IMM,s,b_,off,2'b00}; end endfunction
    function [31:0] ctl; input [6:0] op; input [5:0] dst; input [11:0] off; input [2:0] bi; begin
        ctl = {op,2'b00,dst,off,bi,2'b00}; end endfunction
    localparam [31:0] NOP = 32'd0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    task P; input [11:0] n; input [31:0] s0,s1,s2,s3; begin a.imem[n] = {s3,s2,s1,s0}; end endtask
    task chk; input [255:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s : %0d", nm, got); end
        else begin fail = fail + 1; $display("  FAIL %0s : got %0d (0x%0h) exp %0d (0x%0h)", nm, got, got, exp, exp); end
    end endtask

    initial begin
        for (k = 0; k < 4096; k = k + 1) a.imem[k] = 128'd0;
        for (k = 0; k < (1 << (a.DMEM_AW - 2)); k = k + 1) a.dmem[k] = 32'd0;
        a.dmem[16'hFE00 >> 2] = 32'hDEAD_BEEF;
        P(0,  ri(`ALU_ADD,6'd10,6'd0,9'h7F), ri(`ALU_ADD,6'd11,6'd0,9'd3), rrb(`ALU_CMPEQ,6'd0,6'd0,3'd1), NOP);
        P(1,  ri(`ALU_SHL,6'd10,6'd10,9'd9), NOP, NOP, NOP);
        P(2,  ctl(`CTRL_XNOP,6'd0,12'd3,3'd0), NOP, NOP, NOP);
        P(3,  NOP, NOP, NOP, mst(`MEM_STW,6'd0,6'd10,9'h00));
        P(4,  NOP, NOP, NOP, ri(`MEM_LDW,6'd20,6'd10,9'h08));
        P(5,  NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd10,9'h38));
        P(6,  NOP, NOP, NOP, ri(`MEM_LDW,6'd22,6'd10,9'h10));
        P(7,  ctl(`CTRL_GOTO,6'd0,12'd9,3'd0), NOP, NOP, mst(`MEM_STW,6'd11,6'd10,9'h00));
        P(8,  ri(`ALU_ADD,6'd30,6'd0,9'hAA), NOP, NOP, NOP);
        P(9,  NOP, NOP, NOP, ri(`MEM_LDW,6'd23,6'd10,9'h10));
        P(10, NOP, NOP, NOP, ri(`MEM_LDW,6'd24,6'd10,9'h08));
        P(11, ctl(`CTRL_BR,6'd0,12'd13,3'd1), NOP, NOP, ri(`MEM_LDW,6'd25,6'd10,9'h30));
        P(12, ri(`ALU_ADD,6'd31,6'd0,9'hBB), NOP, NOP, NOP);
        P(13, NOP, NOP, NOP, ri(`MEM_LDW,6'd26,6'd10,9'h30));
        P(14, ctl(`CTRL_CALL,6'd63,12'd20,3'd0), NOP, NOP, ri(`MEM_LDW,6'd27,6'd10,9'h10));
        P(15, NOP, NOP, NOP, ri(`MEM_LDW,6'd32,6'd10,9'h10));
        P(16, NOP, NOP, NOP, ri(`MEM_LDW,6'd33,6'd10,9'h00));
        P(17, STOP, NOP, NOP, NOP);
        P(20, NOP, ri(`ALU_ADD,6'd29,6'd63,9'd0), NOP, ri(`MEM_LDW,6'd28,6'd10,9'h00));
        P(21, ctl(`CTRL_RETURN,6'd0,12'd0,3'd0), NOP, NOP, mst(`MEM_STW,6'd0,6'd10,9'h00));

        repeat (2) @(posedge clk);
        reset = 0; start = 1;
        #1 b.gr[10] = 32'h0000_FE00;        // part B base register (after the reset edge)
        // part B: "stw CTRL = r0" held by a 3-cycle I-miss
        i_rdata = {mst(`MEM_STW,6'd0,6'd10,9'h00), 96'd0};
        i_ready = 0;
        repeat (4) @(posedge clk);          // 1st edge only sets running, then 3 miss cycles
        #1 i_ready = 1;
        @(posedge clk);
        #1 i_rdata = 128'd0;                // then empty packets, EN = 0
        repeat (3) @(posedge clk);
        #1 i_rdata = {96'd0, STOP};
        k = 0;
        while (!(done_a === 1'b1 && done_b === 1'b1) && k < 200) begin @(posedge clk); k = k + 1; end
        repeat (2) @(posedge clk);

        $display("========== verify: PIU access with PC change / after stall ==========");
        chk("A done",                        done_a, 1);
        chk("A PIU-03 CYCLE after XNOP+STOP", a.gr[20], 7);
        chk("A PIU-11 XNOP held store",       a.gr[21], 3);
        chk("A PIU-07 PKT frozen",            a.gr[22], 4);
        chk("A PIU-04 CLR with goto: PKT",    a.gr[23], 0);
        chk("A PIU-04 CYCLE one packet later", a.gr[24], 1);
        chk("A PIU-10 BRT read in br packet", a.gr[25], 0);
        chk("A PIU-10 BRT after taken br",    a.gr[26], 1);
        chk("A PIU-02 load in call packet",   a.gr[27], 4);
        chk("A call link GR63",               a.gr[29], 15);
        chk("A PIU-02 CTRL EN in callee",     a.gr[28], 1);
        chk("A PIU-03 PKT incl. return+STOP", a.gr[32], 7);
        chk("A PIU-03 EN off after return",   a.gr[33], 0);
        chk("A goto poison skipped",          a.gr[30], 0);
        chk("A br poison skipped",            a.gr[31], 0);
        chk("A PIU-05 dmem 0xFE00 untouched", a.dmem[16'hFE00 >> 2], 32'hDEAD_BEEF);
        chk("A PIU-10 BRT final",             a.u_piu.brt, 3);
        chk("A PIU-10 CTRL_OPS final",        a.u_piu.ctl, 3);
        chk("A PIU-09 LD final",              a.u_piu.ld, 6);
        chk("A PIU-09 ST final",              a.u_piu.st, 1);
        chk("A PIU-06 CYCLE final",           a.u_piu.cyc, 7);
        chk("B done",                         done_b, 1);
        chk("B PIU-11 ISTALL held store",     b.u_piu.istall, 3);
        chk("B PIU-06 CYCLE frozen",          b.u_piu.cyc, 4);
        chk("B PIU-07 PKT",                   b.u_piu.pkt, 1);
        chk("B PIU-09 ST",                    b.u_piu.st, 1);
        chk("B PIU-03 EN off",                b.u_piu.en, 0);
        chk("B PIU-05 no d_req",              dreq_seen, 0);
        $display("=====================================================================");
        $display("VERIFY PIU PKT: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: VERIFY PIU PKT PASSED"); else $display("RESULT: VERIFY PIU PKT FAILED");
        $finish;
    end
endmodule
