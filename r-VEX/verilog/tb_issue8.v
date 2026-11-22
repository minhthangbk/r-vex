// tb_issue8.v (round r29, reviews/contracts/issue8.md IS8-01..07, IS8-10): the 8-slot packet on the bare core.
// Rows S1..S11 of the contract; every expected value is written from the contract, not read from the RTL.
`timescale 1ns/1ps
`include "rvex_defs.vh"
module tb_issue8;
    reg clk = 0, reset = 1, start = 0;
    wire done; wire [31:0] cycles, st, ist;
    rvex_core #(.PC_W(8), .DMEM_AW(16)) dut(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(), .d_we(), .d_addr(), .d_wdata(), .d_word(), .d_rdata(32'd0), .d_ready(1'b1),
        .d2_req(), .d2_we(), .d2_addr(), .d2_wdata(), .d2_rdata(32'd0), .d2_hit(1'b0), .stall_cycles(st),
        .i_req(), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles(ist));
    always #5 clk = ~clk;
    integer pass = 0, fail = 0, k, p;
    localparam [31:0] NOP = 32'd0, STOP = {`OP_STOP, 25'd0};
    function [31:0] rr;  input [6:0] op; input [5:0] d, a, b; begin rr = {op, `NO_IMM, d, a, b, 5'd0}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] d, a; input [8:0] im; begin ri = {op, `SHORT_IMM, d, a, im, 2'b00}; end endfunction
    function [31:0] mem; input [6:0] op; input [5:0] d, base; input [8:0] off; begin mem = {op, `NO_IMM, d, base, off, 2'b00}; end endfunction
    function [31:0] rrb; input [6:0] op; input [5:0] a, b; input [2:0] dbr; begin rrb = {op, `NO_IMM, 6'd0, a, b, dbr, 2'b00}; end endfunction
    function [31:0] jbr; input [6:0] op; input [11:0] off; input [2:0] b; input n; begin jbr = {op, 2'b00, 5'b0, n, off, b, 2'b0}; end endfunction
    task P; input [7:0] n; input [31:0] s0, s1, s2, s3, s4, s5, s6, s7;
        begin dut.imem[n] = {s7, s6, s5, s4, s3, s2, s1, s0}; end endtask
    task chk; input [319:0] name; input [31:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s : 0x%08h", name, got); end
        else begin fail = fail + 1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", name, got, exp); end
    end endtask
    // the packets this program commits (path: 0 1 2 3 4 5 7 9 11), for the PIU sums of S10
    reg [7:0] path [0:8];
    reg [31:0] eslot [0:7]; reg [31:0] eops;
    initial begin
        for (k = 0; k < 256; k = k + 1) dut.imem[k] = 256'd0;
        // S1 (IS8-01 / IS8-02): 8 ALU ops, one per slot, 8 distinct GRs and values
        P(0, ri(`ALU_ADD, 6'd10, 6'd0, 9'd3), ri(`ALU_ADD, 6'd11, 6'd0, 9'd6), ri(`ALU_ADD, 6'd12, 6'd0, 9'd9),
             ri(`ALU_ADD, 6'd13, 6'd0, 9'd12), ri(`ALU_ADD, 6'd14, 6'd0, 9'd15), ri(`ALU_ADD, 6'd15, 6'd0, 9'd18),
             ri(`ALU_ADD, 6'd16, 6'd0, 9'd21), ri(`ALU_ADD, 6'd17, 6'd0, 9'd24));
        // S2 (IS8-01): CTRL (goto 2) + ALU + 2 MEM + 4 MUL-class ops in one packet (no joint rule)
        P(1, jbr(`CTRL_GOTO, 12'd2, 3'd0, 1'b0), ri(`ALU_ADD, 6'd20, 6'd0, 9'd7),
             mem(`MEM_LDW, 6'd21, 6'd30, 9'd0), mem(`MEM_LDW, 6'd22, 6'd30, 9'd4),
             rr(`MUL_MPYLL, 6'd23, 6'd2, 6'd3), rr(`MUL_MPYLL, 6'd24, 6'd2, 6'd2),
             rr(`MUL_MPYLL, 6'd25, 6'd3, 6'd3), rr(`SM_DOT2, 6'd26, 6'd4, 6'd5));
        // S3 (IS8-01): multiply-class ops in slots 0-3 write nothing; r31 FP-13: an FPU op in slot 6 is defined
        P(2, rr(`MUL_MPYLL, 6'd40, 6'd2, 6'd3), rr(`SM_DOT2, 6'd41, 6'd4, 6'd5), rr(`MUL_MPYLL, 6'd42, 6'd2, 6'd3),
             rr(`FPU_FADD, 6'd43, 6'd8, 6'd8), NOP, NOP, rr(`FPU_FADD, 6'd44, 6'd8, 6'd8), NOP);
        // S4 (IS8-01): memory opcodes in slots 4 and 5 do not access memory
        P(3, NOP, NOP, NOP, NOP, mem(`MEM_LDW, 6'd46, 6'd30, 9'd0), mem(`MEM_STW, 6'd2, 6'd30, 9'd8), NOP, NOP);
        // S6 (IS8-03): slots 1 and 7 write one GR: slot 7's value
        P(4, NOP, ri(`ALU_ADD, 6'd47, 6'd0, 9'd1), NOP, NOP, NOP, NOP, NOP, ri(`ALU_ADD, 6'd47, 6'd0, 9'd2));
        // S8 (IS8-05): a compare to BR 3 in slot 6 reaches brn in slot 0 (taken to 7, packet 6 skipped)
        P(5, jbr(`CTRL_BR, 12'd7, 3'd3, 1'b1), NOP, NOP, NOP, NOP, NOP, rrb(`ALU_CMPEQ, 6'd2, 6'd2, 3'd3), NOP);
        P(6, ri(`ALU_ADD, 6'd48, 6'd0, 9'd1), NOP, NOP, NOP, NOP, NOP, NOP, NOP);
        // S9 (IS8-05): compares to BR 4 in slots 4 (false) and 7 (true): brn takes slot 7's value (to 9)
        P(7, jbr(`CTRL_BR, 12'd9, 3'd4, 1'b1), NOP, NOP, NOP, rrb(`ALU_CMPEQ, 6'd2, 6'd3, 3'd4), NOP, NOP,
             rrb(`ALU_CMPEQ, 6'd2, 6'd2, 3'd4));
        P(8, ri(`ALU_ADD, 6'd49, 6'd0, 9'd1), NOP, NOP, NOP, NOP, NOP, NOP, NOP);
        // S7 (IS8-04): a slot-1 goto (to 11) with ALU ops in the 7 other slots
        P(9, ri(`ALU_ADD, 6'd50, 6'd0, 9'd1), jbr(`CTRL_GOTO, 12'd11, 3'd0, 1'b0), ri(`ALU_ADD, 6'd51, 6'd0, 9'd2),
             ri(`ALU_ADD, 6'd52, 6'd0, 9'd3), ri(`ALU_ADD, 6'd53, 6'd0, 9'd4), ri(`ALU_ADD, 6'd54, 6'd0, 9'd5),
             ri(`ALU_ADD, 6'd55, 6'd0, 9'd6), ri(`ALU_ADD, 6'd56, 6'd0, 9'd7));
        P(10, ri(`ALU_ADD, 6'd57, 6'd0, 9'd1), NOP, NOP, NOP, NOP, NOP, NOP, NOP);
        // S11 (IS8-07, BUG-358): two half stores to one dmem word whose addresses differ above bit DMEM_AW
        // (0x200 and 0x10202): both halves land (slot 3's store merges into slot 2's word)
        P(11, NOP, NOP, mem(`MEM_STH, 6'd6, 6'd31, 9'd0), mem(`MEM_STH, 6'd7, 6'd32, 9'd2), NOP, NOP, NOP, NOP);
        P(12, STOP, NOP, NOP, NOP, NOP, NOP, NOP, NOP);
        path[0] = 0; path[1] = 1; path[2] = 2; path[3] = 3; path[4] = 4; path[5] = 5; path[6] = 7; path[7] = 9; path[8] = 11;
        for (k = 0; k < 16384; k = k + 1) dut.dmem[k] = 32'd0;
        dut.dmem[32'h100 >> 2] = 32'h1122_3344; dut.dmem[32'h104 >> 2] = 32'h5566_7788;
        repeat (2) @(posedge clk); reset = 0;
        @(negedge clk);   // after the reset edge's nonblocking clears
        dut.gr[2] = 32'd5; dut.gr[3] = 32'd7; dut.gr[4] = 32'h0003_0004; dut.gr[5] = 32'h0005_0006;
        dut.gr[6] = 32'h0000_AAAA; dut.gr[7] = 32'h0000_BBBB; dut.gr[8] = 32'h3F80_0000;
        dut.gr[30] = 32'h100; dut.gr[31] = 32'h200; dut.gr[32] = 32'h1_0200;
        for (k = 40; k <= 49; k = k + 1) dut.gr[k] = 32'h0000_0011;
        dut.gr[57] = 32'h0000_0011;
        @(negedge clk); start = 1; @(negedge clk); start = 0;
        for (k = 0; k < 200 && !done; k = k + 1) @(posedge clk);
        if (!done) $display("TIMEOUT tb_issue8: no STOP after 200 cycles");   // RR-11 (Codex r31 batch 2 F5)
        #1;
        $display("============= tb_issue8 (issue8.md, r29) =============");
        chk("S1 done", {31'd0, done}, 1);
        for (k = 0; k < 8; k = k + 1) chk("S1 slot k ALU (r10+k = 3(k+1))", dut.gr[10 + k], 3 * (k + 1));
        chk("S2 slot 1 ALU", dut.gr[20], 7);
        chk("S2 slot 2 ldw", dut.gr[21], 32'h1122_3344);
        chk("S2 slot 3 ldw", dut.gr[22], 32'h5566_7788);
        chk("S2 slot 4 mpyll", dut.gr[23], 35);
        chk("S2 slot 5 mpyll", dut.gr[24], 25);
        chk("S2 slot 6 mpyll", dut.gr[25], 49);
        chk("S2 slot 7 dot2 3*5+4*6", dut.gr[26], 39);
        chk("S3 mpyll in slot 0 writes nothing", dut.gr[40], 32'h11);
        chk("S3 dot2 in slot 1 writes nothing", dut.gr[41], 32'h11);
        chk("S3 mpyll in slot 2 writes nothing", dut.gr[42], 32'h11);
        chk("S3 fadd in slot 3 writes nothing", dut.gr[43], 32'h11);
        chk("S3 fadd in slot 6 writes its result (r31 FP-13: FPUs in slots 4-7; 1.0 + 1.0)", dut.gr[44], 32'h4000_0000);
        chk("S4 ldw in slot 4 writes nothing", dut.gr[46], 32'h11);
        chk("S4 stw in slot 5 stores nothing", dut.dmem[32'h108 >> 2], 32'd0);
        chk("S6 slot 7 wins over slot 1", dut.gr[47], 2);
        chk("S8 brn sees slot 6 compare", dut.gr[48], 32'h11);
        chk("S9 brn sees slot 7 last", dut.gr[49], 32'h11);
        chk("S9 BR 4 after the packet", {31'd0, dut.br[4]}, 1);
        for (k = 0; k < 7; k = k + 1) chk("S7 ALU next to slot-1 goto", dut.gr[50 + k], k + 1);
        chk("S7 slot-1 goto taken", dut.gr[57], 32'h11);
        chk("S11 both halves land", dut.dmem[32'h200 >> 2], 32'hAAAA_BBBB);
        // S10 (IS8-06): OPS and SLOT0..7 over the committed packets
        eops = 0; for (k = 0; k < 8; k = k + 1) eslot[k] = 0;
        for (p = 0; p < 9; p = p + 1)
            for (k = 0; k < 8; k = k + 1)
                if (dut.imem[path[p]][32*k +: 32] != 32'd0) begin eslot[k] = eslot[k] + 1; eops = eops + 1; end
        chk("S10 PIU OPS over 8 slots", dut.u_piu.ops, eops);
        chk("S10 PIU PKT", dut.u_piu.pkt, 9);
        for (k = 0; k < 8; k = k + 1) chk("S10 PIU SLOTk", dut.u_piu.slot[k], eslot[k]);
        $display("tb_issue8: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: ISSUE8 PASSED"); else $display("RESULT: ISSUE8 FAILED");
        $finish;
    end
endmodule
