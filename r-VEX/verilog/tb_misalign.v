//======================================================================
// r-VEX | Misaligned accesses (round 2026-11-10-r21, BUG-182)
//----------------------------------------------------------------------
// Contract: reviews/contracts/misalign.md (MA-01..MA-04) and piu.md PIU-19.
// Expected values are derived by hand from the contract items, not from the RTL.
// Part 1: mem_unit alone, every opcode x position: the merged store word, the
//         loaded value and the misal flag.
// Part 2: a program on the bare core: sth/ldh/ldhu at byte offsets 1 and 3,
//         misaligned ldw/stw, aligned accesses, and a PIU read of MISALIGN
//         (0xFE60) from the program; the final counter value.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_misalign;
    integer pass = 0, fail = 0, k;
    task chk; input [255:0] nm; input [31:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s : 0x%08h", nm, got); end
        else begin fail = fail + 1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", nm, got, exp); end
    end endtask

    // ---------------- part 1: mem_unit ----------------
    reg  [6:0]  eop; reg [31:0] emem, ereg; reg [1:0] epos;
    wire [31:0] eload, estore; wire eisl, eiss, eval, emis;
    mem_unit u_mem(.opcode(eop),.mem_val(emem),.reg_val(ereg),.pos(epos),.load_data(eload),.store_data(estore),
                   .is_load(eisl),.is_store(eiss),.out_valid(eval),.misal(emis));
    task drive; input [6:0] op; input [31:0] mv, rv; input [1:0] p; begin
        eop = op; emem = mv; ereg = rv; epos = p; #1; end endtask

    // ---------------- part 2: bare core ----------------
    reg clk = 0, reset = 1, start = 0;
    wire done; wire [31:0] cycles;
    rvex_core dut(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles));
    always #5 clk = ~clk;
    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s,b; input [8:0] off; begin
        mst = {op,`NO_IMM,s,b,off,2'b00}; end endfunction
    localparam [31:0] NOP = 32'd0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};
    task P; input [11:0] a; input [31:0] s0,s1,s2,s3; begin dut.imem[a] = {s3,s2,s1,s0}; end endtask

    initial begin
        $display("============= misaligned access test (misalign.md) =============");
        // MA-01: sth at pos 3 leaves the word unchanged (BUG-182: was 0xFFFFFFFF)
        drive(`MEM_STH, 32'h11223344, 32'hAAAA00AB, 2'b11); chk("MA-01 sth pos3 word unchanged", estore, 32'h11223344);
        chk("MA-03 sth pos3 misal", emis, 1);
        // MA-02: sth pos 1 writes bytes 1..2; aligned positions unchanged behaviour
        drive(`MEM_STH, 32'h11223344, 32'hAAAA00AB, 2'b01); chk("MA-02 sth pos1 bytes 1..2", estore, 32'h1100AB44);
        chk("MA-03 sth pos1 misal", emis, 1);
        drive(`MEM_STH, 32'h11223344, 32'hAAAA00AB, 2'b00); chk("MA-02 sth pos0", estore, 32'h00AB3344);
        chk("MA-03 sth pos0 aligned", emis, 0);
        drive(`MEM_STH, 32'h11223344, 32'hAAAA00AB, 2'b10); chk("MA-02 sth pos2", estore, 32'h112200AB);
        chk("MA-03 sth pos2 aligned", emis, 0);
        // MA-02: ldh/ldhu at pos 3 read 0 (crossing half); pos 1 reads bytes 1..2
        drive(`MEM_LDH,  32'h1122C3C4, 0, 2'b11); chk("MA-02 ldh pos3 = 0", eload, 32'h0); chk("MA-03 ldh pos3 misal", emis, 1);
        drive(`MEM_LDHU, 32'h1122C3C4, 0, 2'b11); chk("MA-02 ldhu pos3 = 0", eload, 32'h0); chk("MA-03 ldhu pos3 misal", emis, 1);
        drive(`MEM_LDH,  32'h11C3C444, 0, 2'b01); chk("MA-02 ldh pos1 bytes 1..2", eload, 32'hFFFFC3C4);
        chk("MA-03 ldh pos1 misal", emis, 1);
        drive(`MEM_LDHU, 32'h11C3C444, 0, 2'b01); chk("MA-02 ldhu pos1", eload, 32'h0000C3C4);
        // MA-02: ldw/stw ignore address bits [1:0]
        for (k = 1; k < 4; k = k + 1) begin
            drive(`MEM_LDW, 32'hDEADBEEF, 0, k[1:0]); chk("MA-02 ldw pos!=0 whole word", eload, 32'hDEADBEEF);
            chk("MA-03 ldw pos!=0 misal", emis, 1);
            drive(`MEM_STW, 32'h0, 32'hCAFEF00D, k[1:0]); chk("MA-02 stw pos!=0 whole word", estore, 32'hCAFEF00D);
            chk("MA-03 stw pos!=0 misal", emis, 1);
        end
        drive(`MEM_LDW, 32'hDEADBEEF, 0, 2'b00); chk("MA-03 ldw pos0 aligned", emis, 0);
        // MA-03: byte accesses are never misaligned; PFT never
        for (k = 0; k < 4; k = k + 1) begin
            drive(`MEM_LDB, 32'h11223344, 0, k[1:0]); chk("MA-03 ldb aligned", emis, 0);
            drive(`MEM_STB, 32'h11223344, 32'h55, k[1:0]); chk("MA-03 stb aligned", emis, 0);
            drive(`MEM_PFT, 32'h11223344, 0, k[1:0]); chk("MA-03 pft not counted", emis, 0);
        end

        // ---------------- part 2 ----------------
        for (k = 0; k < 4096; k = k + 1) dut.imem[k] = 128'd0;
        for (k = 0; k < (1 << (dut.DMEM_AW - 2)); k = k + 1) dut.dmem[k] = 32'd0;
        dut.dmem[16'h100 >> 2] = 32'h11223344;
        dut.dmem[16'h104 >> 2] = 32'h55667788;
        dut.dmem[16'h108 >> 2] = 32'h99AABBCC;
        //  0: r10 = 0x100 | r11 = 0xCD | r13 = 0x7F
        P(0, ri(`ALU_ADD,6'd10,6'd0,9'h100), ri(`ALU_ADD,6'd11,6'd0,9'hCD), ri(`ALU_ADD,6'd13,6'd0,9'h7F), NOP);
        //  1: r13 = 0xFE00 (PIU window)
        P(1, ri(`ALU_SHL,6'd13,6'd13,9'd9), NOP, NOP, NOP);
        //  2: sth 3[r10] = r11            MA-01: 0x100 unchanged                    MISALIGN 1
        P(2, NOP, NOP, NOP, mst(`MEM_STH,6'd11,6'd10,9'd3));
        //  3: sth 1[r10] = r11            MA-02: 0x100 = 0x1100CD44                 MISALIGN 2
        P(3, NOP, NOP, NOP, mst(`MEM_STH,6'd11,6'd10,9'd1));
        //  4: sth 6[r10] = r11            aligned: 0x104 = 0x556600CD
        P(4, NOP, NOP, NOP, mst(`MEM_STH,6'd11,6'd10,9'd6));
        //  5: ldh r20 = 3[r10]            MA-02: 0                                  MISALIGN 3
        P(5, NOP, NOP, NOP, ri(`MEM_LDH,6'd20,6'd10,9'd3));
        //  6: ldhu r21 = 1[r10]           MA-02: bytes 1..2 of 0x1100CD44 = 0x00CD   MISALIGN 4
        P(6, NOP, NOP, NOP, ri(`MEM_LDHU,6'd21,6'd10,9'd1));
        //  7: ldw r22 = 2[r10]            MA-02: word 0x100                         MISALIGN 5
        P(7, NOP, NOP, NOP, ri(`MEM_LDW,6'd22,6'd10,9'd2));
        //  8: stw 11[r10] = r11           MA-02: word 0x108 = 0xCD                  MISALIGN 6
        P(8, NOP, NOP, NOP, mst(`MEM_STW,6'd11,6'd10,9'd11));
        //  9: ldb r23 = 3[r10] (aligned byte)  10: ldh r24 = 6[r10] (aligned half)
        P(9, NOP, NOP, NOP, ri(`MEM_LDB,6'd23,6'd10,9'd3));
        P(10, NOP, NOP, NOP, ri(`MEM_LDH,6'd24,6'd10,9'd6));
        // 11: ldw r25 = 0x60[r13]         PIU-19: MISALIGN read by the program = 6
        P(11, NOP, NOP, NOP, ri(`MEM_LDW,6'd25,6'd13,9'h60));
        // 12: ldh r26 = 0x63[r13]         a misaligned PIU read counts too (MA-03): MISALIGN 7 after it
        P(12, NOP, NOP, NOP, ri(`MEM_LDH,6'd26,6'd13,9'h63));
        P(13, STOP, NOP, NOP, NOP);
        repeat (2) @(posedge clk);
        reset = 0; start = 1;
        k = 0;
        while (done !== 1'b1 && k < 200) begin @(posedge clk); k = k + 1; end
        repeat (2) @(posedge clk);
        chk("done", done, 1);
        chk("MA-01 core sth pos3: 0x100 after packets 2..3", dut.dmem[16'h100 >> 2], 32'h1100CD44);
        chk("MA-02 core aligned sth: 0x104", dut.dmem[16'h104 >> 2], 32'h556600CD);
        chk("MA-02 core stw 11[r10] -> word 0x108", dut.dmem[16'h108 >> 2], 32'h000000CD);
        chk("MA-02 core ldh pos3 = 0", dut.gr[20], 32'h0);
        chk("MA-02 core ldhu pos1", dut.gr[21], 32'h000000CD);
        chk("MA-02 core ldw pos2 = word", dut.gr[22], 32'h1100CD44);
        chk("aligned ldb", dut.gr[23], 32'h44);
        chk("aligned ldh", dut.gr[24], 32'h000000CD);
        chk("PIU-19 MISALIGN read by the program", dut.gr[25], 6);
        chk("PIU-19 misaligned PIU ldh reads 0 (crossing half)", dut.gr[26], 0);
        chk("PIU-19 MISALIGN final", dut.u_piu.misalign, 7);
        chk("PIU-12 LD+ST unchanged by MISALIGN", dut.u_piu.ld + dut.u_piu.st, 11);
        $display("=================================================================");
        $display("MISALIGN TEST: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: MISALIGN PASSED"); else $display("RESULT: MISALIGN FAILED");
        $finish;
    end
endmodule
