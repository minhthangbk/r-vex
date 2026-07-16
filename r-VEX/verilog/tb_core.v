//======================================================================
// r-VEX | Integrated core testbench
//----------------------------------------------------------------------
// Loads a small VLIW program that exercises, end-to-end through the
// core (fetch->decode->execute->writeback):
//   - ALU immediate ops         (ADDi)
//   - MUL on slot 1             (MPYLL)
//   - STORE then sign-extended LOAD (STW / LDB / LDBU)  <- bug-fix path
//   - XNOP n multicycle stall   <- new feature
//   - taken GOTO branch (skips a poison packet)
//   - STOP
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_core;
    reg clk=0, reset=1, start=0;
    wire done; wire [31:0] cycles;
    integer pass=0, fail=0;

    rvex_core dut(.clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles));
    always #5 clk = ~clk;

    // ---- syllable encoders (readable field assembly) ----
    function [31:0] rr;  input [6:0] op; input [5:0] d,a,b; begin
        rr = {op,2'b00,d,a,b,3'b000,2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,2'b01,d,a,im,2'b00}; end endfunction
    function [31:0] mld; input [6:0] op; input [5:0] d,base; input [8:0] off; begin
        mld = {op,2'b00,d,base,off,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] dreg,base; input [8:0] off; begin
        mst = {op,2'b00,dreg,base,off,2'b00}; end endfunction
    function [31:0] j12; input [6:0] op; input [11:0] off; begin
        j12 = {op,8'b0,off,5'b0}; end endfunction

    localparam [31:0] NOP = 32'd0;

    task P; input [7:0] a; input [31:0] s0,s1,s2,s3; begin
        dut.imem[a] = {s3,s2,s1,s0}; end
    endtask

    task chk; input [127:0] nm; input [31:0] got,exp; begin
        if (got===exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h", nm, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", nm, got, exp); end
    end endtask

    integer k;
    initial begin
        // clear imem
        for (k=0;k<256;k=k+1) dut.imem[k] = 128'd0;

        // ---- program ----
        // 0: r1 = 5 ; r2 = 0x80
        P(8'd0, ri(`ALU_ADD,6'd1,6'd0,9'd5), ri(`ALU_ADD,6'd2,6'd0,9'h080), NOP, NOP);
        // 1: r3 = r1 * r2 (MPYLL on slot1) = 5*128 = 640
        P(8'd1, NOP, rr(`MUL_MPYLL,6'd3,6'd1,6'd2), NOP, NOP);
        // 2: STW dmem[0] = r2 (0x80)   (slot3, base r0, off 0)
        P(8'd2, NOP, NOP, NOP, mst(`MEM_STW,6'd2,6'd0,9'd0));
        // 3: XNOP 3  (stall 3 cycles)
        P(8'd3, j12(`CTRL_XNOP,12'd3), NOP, NOP, NOP);
        // 4: r5 = LDB [byte 3 of word0] -> 0x80 sign-extended = 0xFFFFFF80
        P(8'd4, NOP, NOP, NOP, mld(`MEM_LDB,6'd5,6'd0,9'd3));
        // 5: r6 = LDBU same byte -> 0x00000080
        P(8'd5, NOP, NOP, NOP, mld(`MEM_LDBU,6'd6,6'd0,9'd3));
        // 6: GOTO 8 (skip poison at 7)
        P(8'd6, j12(`CTRL_GOTO,12'd8), NOP, NOP, NOP);
        // 7: poison: r7 = 0xAA  (must be skipped)
        P(8'd7, ri(`ALU_ADD,6'd7,6'd0,9'h0AA), NOP, NOP, NOP);
        // 8: STOP  (opcode must sit in syllable[31:25])
        P(8'd8, {`OP_STOP,25'd0}, NOP, NOP, NOP);

        // ---- run ----
        reset=1; start=0; repeat(2) @(posedge clk);
        reset=0; start=1;

        // wait for done (with timeout)
        k=0;
        while (done!==1'b1 && k<200) begin @(posedge clk); k=k+1; end

        $display("================ r-VEX core test ================");
        $display("  done=%b  cycles=%0d  (timeout counter=%0d)", done, cycles, k);
        chk("gr1 (=5)",        dut.gr[1],  32'd5);
        chk("gr2 (=0x80)",     dut.gr[2],  32'h00000080);
        chk("gr3 (MPYLL 640)", dut.gr[3],  32'd640);
        chk("dmem0 (STW)",     dut.dmem[0],32'h00000080);
        chk("gr5 (LDB sign)",  dut.gr[5],  32'hFFFFFF80);
        chk("gr6 (LDBU zero)", dut.gr[6],  32'h00000080);
        chk("gr7 (skipped=0)", dut.gr[7],  32'd0);
        // 8 packets issued (0,1,2,3,4,5,6,8) + 3 XNOP stall cycles = 11 running cycles
        chk("cycles incl XNOP-3 stall", cycles, 32'd11);
        $display("=================================================");
        $display("CORE TEST: %0d passed, %0d failed", pass, fail);
        if (fail==0 && done===1'b1) $display("RESULT: CORE TEST PASSED");
        else $display("RESULT: CORE TEST FAILED");
        $finish;
    end
endmodule
