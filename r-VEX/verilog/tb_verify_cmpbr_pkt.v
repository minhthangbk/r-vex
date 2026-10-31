//======================================================================
// r-VEX | final verify r6: BRIMM compares in packets with other BR writers,
// a D-cache stall, a taken branch and a load that overwrites the source GR
//----------------------------------------------------------------------
// Contract reviews/contracts/cmpbr.md (CB-01, CB-05) and the EXT_DMEM hold
// rule (rvex_core.v header: on a miss the WHOLE packet is held, no GR/BR/PC
// update). Expected values from the contract / C semantics:
//  P0 (held 4 cycles): b1 = (r3 == 7) [old r3=7 -> 1] | mtb b2 = r4[0] (1)
//      | b3 = r5 < r6 (reg compare, 1) | ldw r3 = [0] (400).  While held,
//      b1/b2/b3 stay 0 and r3 stays 7.
//  P1 (held): br b1 -> P3 (taken) | addcg r7,b4 = r8 + r9 + b0 (0xffffffff+1+1:
//      r7 = 1, b4 = 1) | b5 = r3 gtu 300 (reads the new r3 = 400 -> 1)
//      | ldw r10 = [4] (3)
//  P2: r11 = 1 (skipped)
//  P3: b6 = r10 lt 5 (1, slot 1) | b0 = r10 ne 3 (0, slot 2)
//      | b7 = r7 ge 2 (0, slot 3 ALU; b7 preset 1)
//  P4: brf b0 -> P6 (taken: b0 written 0 by P3) ; P5: r12 = 1 (skipped) ; P6 STOP
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_verify_cmpbr_pkt;
    reg clk = 0, reset = 1, start = 0;
    wire done, d_req, d_we, d_word; wire [31:0] cycles, d_addr, d_wdata;
    reg [31:0] dm [0:3];
    reg [2:0] cnt = 0;
    wire d_ready = d_req && (cnt == 3'd4);
    wire [31:0] d_rdata = dm[d_addr[3:2]];
    integer pass = 0, fail = 0, k, held = 0;
    localparam [31:0] NOP = 0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    rvex_core #(.EXT_DMEM(1)) dut(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(d_word),
        .d_rdata(d_rdata), .d_ready(d_ready), .i_rdata(128'd0), .i_ready(1'b1));
    always #5 clk = ~clk;
    always @(posedge clk) cnt <= (d_req && cnt != 3'd4) ? cnt + 3'd1 : 3'd0;

    function [31:0] bi;  input [6:0] op; input [2:0] b; input [5:0] a; input [8:0] im; begin
        bi = {op,2'b11,3'b000,b,a,im,2'b00}; end endfunction
    function [31:0] rrb; input [6:0] op; input [5:0] a,b; input [2:0] dbr; begin
        rrb = {op,`NO_IMM,6'd0,a,b,dbr,2'b00}; end endfunction
    function [31:0] mtb; input [5:0] a; input [2:0] dbr; begin
        mtb = {`ALU_MTB,`NO_IMM,6'd0,a,6'd0,dbr,2'b00}; end endfunction
    function [31:0] addcg; input [5:0] d,a,b; input [2:0] sb,db; begin
        addcg = {{`ALU_ADDCG4,sb},`NO_IMM,d,a,b,db,2'b00}; end endfunction
    function [31:0] ldw; input [5:0] d,a; input [8:0] off; begin
        ldw = {`MEM_LDW,`NO_IMM,d,a,off,2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] jbr; input [6:0] op; input [11:0] off; input [2:0] b; begin
        jbr = {op,8'b0,off,b,2'b0}; end endfunction

    task chk; input [255:0] nm; input [31:0] got, exp; begin
        if (got === exp) pass = pass + 1;
        else begin fail = fail + 1; $display("  FAIL %0s : got %0h exp %0h", nm, got, exp); end
    end endtask

    // while P0 is held: no BR/GR effect of the packet
    always @(posedge clk) if (!reset && dut.running && !done && dut.pc == 0 && d_req && !d_ready) begin
        held = held + 1;
        chk("P0 held: b1", dut.br[1], 0); chk("P0 held: b2", dut.br[2], 0);
        chk("P0 held: b3", dut.br[3], 0); chk("P0 held: r3", dut.gr[3], 7);
    end

    initial begin
        dm[0] = 400; dm[1] = 3; dm[2] = 0; dm[3] = 0;
        for (k = 0; k < 4096; k = k + 1) dut.imem[k] = 128'd0;
        dut.imem[0] = {ldw(6'd3, 6'd0, 9'd0), rrb(`ALU_CMPLT, 6'd5, 6'd6, 3'd3), mtb(6'd4, 3'd2),
                       bi(`ALU_CMPEQ, 3'd1, 6'd3, 9'd7)};
        dut.imem[1] = {ldw(6'd10, 6'd0, 9'd4), bi(`ALU_CMPGTU, 3'd5, 6'd3, 9'd300),
                       addcg(6'd7, 6'd8, 6'd9, 3'd0, 3'd4), jbr(`CTRL_BR, 12'd3, 3'd1)};
        dut.imem[2] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd11, 6'd0, 9'd1)};
        dut.imem[3] = {bi(`ALU_CMPGE, 3'd7, 6'd7, 9'd2), bi(`ALU_CMPNE, 3'd0, 6'd10, 9'd3),
                       bi(`ALU_CMPLT, 3'd6, 6'd10, 9'd5), NOP};
        dut.imem[4] = {NOP, NOP, NOP, jbr(`CTRL_BRF, 12'd6, 3'd0)};
        dut.imem[5] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd12, 6'd0, 9'd1)};
        dut.imem[6] = {NOP, NOP, NOP, STOP};
        repeat (2) @(posedge clk);
        reset = 0;
        @(posedge clk);
        dut.gr[3] = 7; dut.gr[4] = 1; dut.gr[5] = 2; dut.gr[6] = 9;
        dut.gr[8] = 32'hffffffff; dut.gr[9] = 1;
        dut.br[0] = 1; dut.br[7] = 1;
        start = 1;
        k = 0;
        while (done !== 1'b1 && k < 200) begin @(posedge clk); k = k + 1; end
        chk("STOP reached", done, 1);
        chk("P0 was held >= 3 cycles", held >= 3, 1);
        chk("b1 = old r3 eq 7", dut.br[1], 1);
        chk("b2 = mtb r4", dut.br[2], 1);
        chk("b3 = r5 lt r6", dut.br[3], 1);
        chk("r3 loaded", dut.gr[3], 400);
        chk("r7 addcg", dut.gr[7], 1);
        chk("b4 addcg carry", dut.br[4], 1);
        chk("b5 = new r3 gtu 300", dut.br[5], 1);
        chk("r10 loaded", dut.gr[10], 3);
        chk("P2 skipped (br on BRIMM b1)", dut.gr[11], 0);
        chk("b6 = r10 lt 5", dut.br[6], 1);
        chk("b0 = r10 ne 3", dut.br[0], 0);
        chk("b7 = r7 ge 2 (slot 3)", dut.br[7], 0);
        chk("P5 skipped (brf on BRIMM b0)", dut.gr[12], 0);
        $display("VERIFY CMPBR PKT: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: VERIFY CMPBR PKT PASSED"); else $display("RESULT: VERIFY CMPBR PKT FAILED");
        $finish;
    end
endmodule
