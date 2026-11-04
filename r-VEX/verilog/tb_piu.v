//======================================================================
// r-VEX | Directed test of the Performance Indicator Unit on the bare core
//----------------------------------------------------------------------
// Contract: reviews/contracts/piu.md. Expected values are derived by hand
// from the contract items (named in each check), not from the RTL.
// Program: build 0xFE00 in r10, START (CLR+EN), a measured region with
// known contents (ALU/MUL16/MUL32/LD/ST/CTRL/taken branch/NOP packet/XNOP),
// STOP, read the frozen counters, then a sub-word START, an EN-only write
// and a final STOP.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_piu;
    reg clk = 0, reset = 1, start = 0;
    wire done; wire [31:0] cycles;
    integer pass = 0, fail = 0, k;

    rvex_core dut(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles));
    always #5 clk = ~clk;

    function [31:0] rr;  input [6:0] op; input [5:0] d,a,b; begin
        rr = {op,`NO_IMM,d,a,b,3'b000,2'b00}; end endfunction
    function [31:0] rrb; input [6:0] op; input [5:0] a,b; input [2:0] dbr; begin
        rrb = {op,`NO_IMM,6'd0,a,b,dbr,2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s,b; input [8:0] off; begin
        mst = {op,`NO_IMM,s,b,off,2'b00}; end endfunction
    function [31:0] jbr; input [6:0] op; input [11:0] off; input [2:0] bidx; begin
        jbr = {op,8'b0,off,bidx,2'b0}; end endfunction
    function [31:0] j12; input [6:0] op; input [11:0] n; begin
        j12 = {op,8'b0,n,5'b0}; end endfunction
    localparam [31:0] NOP = 32'd0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    task P; input [11:0] a; input [31:0] s0,s1,s2,s3; begin
        dut.imem[a] = {s3,s2,s1,s0}; end endtask
    task chk; input [255:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s : %0d", nm, got); end
        else begin fail = fail + 1; $display("  FAIL %0s : got %0d (0x%0h) exp %0d (0x%0h)", nm, got, got, exp, exp); end
    end endtask

    initial begin
        for (k = 0; k < 4096; k = k + 1) dut.imem[k] = 128'd0;
        for (k = 0; k < (1 << (dut.DMEM_AW - 2)); k = k + 1) dut.dmem[k] = 32'd0;
        //  0: r10 = 0x7F | r11 = 3 | r32 = 0x1FF | r12 = 1
        P(0,  ri(`ALU_ADD,6'd10,6'd0,9'h7F), ri(`ALU_ADD,6'd11,6'd0,9'd3),
              ri(`ALU_ADD,6'd32,6'd0,9'h1FF), ri(`ALU_ADD,6'd12,6'd0,9'd1));
        //  1: r10 = 0xFE00
        P(1,  ri(`ALU_SHL,6'd10,6'd10,9'd9), NOP, NOP, NOP);
        //  2: START: stw CTRL = 3 (CLR + EN); not counted (PIU-03/04)
        P(2,  NOP, NOP, NOP, mst(`MEM_STW,6'd11,6'd10,9'h00));
        // ---- measured region: packets 3..13 ----
        //  3: r1 = 5 | r2 = 7 | r3 = 9 | ldw r20 = ID          ALU 3, LD 1
        P(3,  ri(`ALU_ADD,6'd1,6'd0,9'd5), ri(`ALU_ADD,6'd2,6'd0,9'd7),
              ri(`ALU_ADD,6'd3,6'd0,9'd9), ri(`MEM_LDW,6'd20,6'd10,9'h04));
        //  4: mpyll r4 | mpylu r5                               MUL16 1, MUL32 1
        P(4,  NOP, rr(`MUL_MPYLL,6'd4,6'd1,6'd2), rr(`MUL_MPYLU,6'd5,6'd1,6'd2), NOP);
        //  5: empty packet                                       NOPPKT 1
        P(5,  NOP, NOP, NOP, NOP);
        //  6: xnop 2                                             CTRL 1, XNOP 2 cycles
        P(6,  j12(`CTRL_XNOP,12'd2), NOP, NOP, NOP);
        //  7: br1 = r1 < r2 | mpylu r8 (slot 1) | mpyhh r9 (slot 2)   ALU 1, MUL32 1, MUL16 1
        P(7,  rrb(`ALU_CMPLT,6'd1,6'd2,3'd1), rr(`MUL_MPYLU,6'd8,6'd1,6'd2), rr(`MUL_MPYHH,6'd9,6'd1,6'd2), NOP);
        //  8: br br1 -> 10 (taken) | r6 = 1                       CTRL 1, BRT 1, ALU 1
        P(8,  jbr(`CTRL_BR,12'd10,3'd1), ri(`ALU_ADD,6'd6,6'd0,9'd1), NOP, NOP);
        //  9: poison (skipped)
        P(9,  ri(`ALU_ADD,6'd7,6'd0,9'hAA), NOP, NOP, NOP);
        // 10: brf br1 -> 9 (not taken)                            CTRL 1
        P(10, jbr(`CTRL_BRF,12'd9,3'd1), NOP, NOP, NOP);
        // 11: ldh r21 = 6[r10] (low half of ID)                    LD 1
        P(11, NOP, NOP, NOP, ri(`MEM_LDH,6'd21,6'd10,9'h06));
        // 12: stw 8[r10] = r1: store to a read-only register       ST 1 (no effect, PIU-05)
        P(12, NOP, NOP, NOP, mst(`MEM_STW,6'd1,6'd10,9'h08));
        // 13: STOP: stw CTRL = 0; counted with the old EN (PIU-03) ST 1
        P(13, NOP, NOP, NOP, mst(`MEM_STW,6'd0,6'd10,9'h00));
        // ---- frozen ----
        // 14..16: read PKT, CYCLE, offset 0x60 (first hole since PIU-18 SEMFAIL took 0x5C) while EN = 0
        P(14, NOP, NOP, NOP, ri(`MEM_LDW,6'd30,6'd10,9'h10));
        P(15, NOP, NOP, NOP, ri(`MEM_LDW,6'd31,6'd10,9'h08));
        P(16, NOP, NOP, NOP, ri(`MEM_LDW,6'd32,6'd10,9'h60));
        // 17: stb 3[r10] = r11 (byte lane 3 = 3: CLR + EN), sub-word store (PIU-05)
        P(17, NOP, NOP, NOP, mst(`MEM_STB,6'd11,6'd10,9'h03));
        // 18: stw CTRL = 0 (counted: PKT 1)
        P(18, NOP, NOP, NOP, mst(`MEM_STW,6'd0,6'd10,9'h00));
        // 19: ldw r33 = PKT (frozen, = 1)
        P(19, NOP, NOP, NOP, ri(`MEM_LDW,6'd33,6'd10,9'h10));
        // 20: stw CTRL = r12 (EN only, no clear); not counted (old EN = 0)
        P(20, NOP, NOP, NOP, mst(`MEM_STW,6'd12,6'd10,9'h00));
        // 21: empty packet (counted)    22: stw CTRL = 0 (counted)
        P(21, NOP, NOP, NOP, NOP);
        P(22, NOP, NOP, NOP, mst(`MEM_STW,6'd0,6'd10,9'h00));
        // 23: STOP (EN = 0: the STOP cycle is not counted)
        P(23, STOP, NOP, NOP, NOP);

        repeat (2) @(posedge clk);
        // PIU-01: reset values
        chk("PIU-01 reset EN",      dut.u_piu.en, 1);
        chk("PIU-01 reset CYCLE",   dut.u_piu.cyc, 0);
        chk("PIU-01 reset PKT",     dut.u_piu.pkt, 0);
        reset = 0; start = 1;
        k = 0;
        while (done !== 1'b1 && k < 200) begin @(posedge clk); k = k + 1; end
        repeat (2) @(posedge clk);

        $display("============= PIU directed test (bare core) =============");
        chk("done",                  done, 1);
        chk("PIU-01/02 ID ldw",      dut.gr[20], 32'h50495501);
        chk("PIU-02 ldh lane",       dut.gr[21], 32'h00005501);
        chk("PIU-02 hole reads 0",   dut.gr[32], 0);
        chk("PIU-07 PKT frozen",     dut.gr[30], 10);
        chk("PIU-06 CYCLE frozen",   dut.gr[31], 12);
        chk("PIU-05 sub-word START", dut.gr[33], 1);
        chk("PIU-05 dmem untouched 0xFE00", dut.dmem[16'hFE00 >> 2], 0);
        chk("PIU-05 dmem untouched 0xFE08", dut.dmem[16'hFE08 >> 2], 0);
        chk("poison skipped",        dut.gr[7], 0);
        chk("mpyll",                 dut.gr[4], 35);
        // final counters: region 17..22 = packets 18, 21, 22 (EN-only write 20 not counted)
        chk("PIU-03 EN-only PKT",    dut.u_piu.pkt, 3);
        chk("PIU-06 CYCLE final",    dut.u_piu.cyc, 3);
        chk("PIU-07 OPS final",      dut.u_piu.ops, 2);
        chk("PIU-09 ST final",       dut.u_piu.st, 2);
        chk("PIU-07 NOPPKT final",   dut.u_piu.noppkt, 1);
        chk("PIU-03 EN after STOP",  dut.u_piu.en, 0);
        $display("=========================================================");
        $display("PIU TEST: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: PIU PASSED"); else $display("RESULT: PIU FAILED");
        $finish;
    end

    // Region counters at the cycle packet 14 issues (EN = 0 since packet 13).
    reg checked = 0;
    always @(posedge clk) if (!reset && dut.pc == 12'd14 && !checked && dut.running) begin
        checked <= 1;
        $display("  region snapshot at packet 14:");
        chk("PIU-07 PKT",     dut.u_piu.pkt, 10);
        chk("PIU-06 CYCLE",   dut.u_piu.cyc, 12);
        chk("PIU-07 OPS",     dut.u_piu.ops, 16);
        chk("PIU-12 ALU",     dut.u_piu.alu, 5);
        chk("PIU-08 MUL16",   dut.u_piu.mul16, 2);
        chk("PIU-08 MUL32",   dut.u_piu.mul32, 2);
        chk("PIU-09 LD",      dut.u_piu.ld, 2);
        chk("PIU-09 ST",      dut.u_piu.st, 2);
        chk("PIU-10 CTRL",    dut.u_piu.ctl, 3);
        chk("PIU-10 BRT",     dut.u_piu.brt, 1);
        chk("PIU-07 NOPPKT",  dut.u_piu.noppkt, 1);
        chk("PIU-11 XNOP",    dut.u_piu.xnop, 2);
        chk("PIU-11 ISTALL",  dut.u_piu.istall, 0);
        chk("PIU-11 DSTALL",  dut.u_piu.dstall, 0);
        chk("PIU-07 SLOT0",   dut.u_piu.slot[0], 5);
        chk("PIU-07 SLOT1",   dut.u_piu.slot[1], 4);
        chk("PIU-07 SLOT2",   dut.u_piu.slot[2], 3);
        chk("PIU-07 SLOT3",   dut.u_piu.slot[3], 4);
        chk("PIU-16 MULPKT",  dut.u_piu.mulpkt, 2);
        chk("PIU-17 SEMSTALL", dut.u_piu.semstall, 0);
        chk("PIU-03 EN off",  dut.u_piu.en, 0);
    end
endmodule
