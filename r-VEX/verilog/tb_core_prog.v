//======================================================================
// r-VEX | Second integrated core program (control-flow + VLIW packing)
//----------------------------------------------------------------------
// Complements tb_core.v (which covered ALU-imm / MUL / mem / XNOP / GOTO).
// This program drives the paths tb_core.v left out, end-to-end:
//   - two-slot VLIW parallel issue in a single packet (ALU || ALU, ALU || MUL)
//   - register-register ADD
//   - CMPLT writing a Branch Register  (dst=0 -> BR write)
//   - conditional BR taken on that BR   (skips two poison packets)
//   - SLCT selecting on a BR value      (opcode low-3 bits = BR index)
//   - CALL writing a link register, then RETURN through that link
//   - STOP
//
// Field layout matches rvex_core.v decode:
//   op[31:25] imt[24:23] dst[22:17] r1a[16:11] r2a[10:5] dbr[4:2] [1:0]
//   short-imm value = syl[10:2] ; branch offset off0 = syl[16:5]
//   SLCT/ADDCG/DIVS source-BR index = opcode[2:0]
//   BR/BRF source-BR index          = syl[4:2]
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_core_prog;
    reg clk=0, reset=1, start=0;
    wire done; wire [31:0] cycles;
    integer pass=0, fail=0;

    rvex_core dut(.clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles));
    always #5 clk = ~clk;

    // ---- syllable encoders ----
    // register-register (dbr = 0)
    function [31:0] rr;  input [6:0] op; input [5:0] d,a,b; begin
        rr = {op,`NO_IMM,d,a,b,3'b000,2'b00}; end endfunction
    // register-register writing a BR (compare with dst=0, dbr set)
    function [31:0] rrb; input [6:0] op; input [5:0] d,a,b; input [2:0] dbr; begin
        rrb = {op,`NO_IMM,d,a,b,dbr,2'b00}; end endfunction
    // register-immediate (9-bit short immediate in syl[10:2])
    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    // CALL/GOTO/XNOP: 12-bit offset in syl[16:5], dst in syl[22:17]
    function [31:0] jc;  input [6:0] op; input [5:0] d; input [11:0] off; begin
        jc = {op,`NO_IMM,d,off,5'b0}; end endfunction
    // BR/BRF: 12-bit offset + source-BR index in syl[4:2]
    function [31:0] jbr; input [6:0] op; input [11:0] off; input [2:0] bidx; begin
        jbr = {op,8'b0,off,bidx,2'b0}; end endfunction
    // RETURN: r1a = link register (read as lr); dst = 0 (no link writeback)
    function [31:0] jret; input [6:0] op; input [5:0] lreg; begin
        jret = {op,`NO_IMM,6'd0,lreg,6'd0,3'b0,2'b0}; end endfunction

    localparam [31:0] NOP = 32'd0;

    task P; input [7:0] a; input [31:0] s0,s1,s2,s3; begin
        dut.imem[a] = {s3,s2,s1,s0}; end
    endtask
    task chk; input [127:0] nm; input [31:0] got,exp; begin
        if (got===exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h", nm, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", nm, got, exp); end
    end endtask
    task chkb; input [127:0] nm; input got,exp; begin
        if (got===exp) begin pass=pass+1; $display("  PASS %0s : %b", nm, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got %b exp %b", nm, got, exp); end
    end endtask

    // SLCT opcode carrying source-BR index 1 in its low 3 bits
    localparam [6:0] SLCT_BR1 = {`ALU_SLCT4, 3'd1};

    integer k;
    initial begin
        for (k=0;k<256;k=k+1) dut.imem[k] = 128'd0;

        // 0: r1 = 10  ||  r2 = 3     (VLIW: ALU-imm on slot0 AND slot1)
        P(8'd0, ri(`ALU_ADD,6'd1,6'd0,9'd10), ri(`ALU_ADD,6'd2,6'd0,9'd3), NOP, NOP);
        // 1: r3 = r1+r2 (=13)  ||  r4 = r1*r2 MPYLL (=30)   (ALU || MUL)
        P(8'd1, rr(`ALU_ADD,6'd3,6'd1,6'd2), rr(`MUL_MPYLL,6'd4,6'd1,6'd2), NOP, NOP);
        // 2: BR1 = (r2 < r1) signed = 1      (CMPLT, dst=0 -> writes BR1)
        P(8'd2, rrb(`ALU_CMPLT,6'd0,6'd2,6'd1,3'd1), NOP, NOP, NOP);
        // 3: BR BR1 -> goto 6                (taken, skips poison 4,5)
        P(8'd3, jbr(`CTRL_BR,12'd6,3'd1), NOP, NOP, NOP);
        // 4: poison  r5 = 0xAA               (must be skipped)
        P(8'd4, ri(`ALU_ADD,6'd5,6'd0,9'h0AA), NOP, NOP, NOP);
        // 5: poison  r5 = 0xBB               (must be skipped)
        P(8'd5, ri(`ALU_ADD,6'd5,6'd0,9'h0BB), NOP, NOP, NOP);
        // 6: r6 = BR1 ? r1 : r2   (=r1=10)   (SLCT on BR1)
        P(8'd6, rr(SLCT_BR1,6'd6,6'd1,6'd2), NOP, NOP, NOP);
        // 7: CALL 10, link -> r63 (=8)       (taken to 10)
        P(8'd7, jc(`CTRL_CALL,6'd63,12'd10), NOP, NOP, NOP);
        // 8: (return target) r7 = 0x55
        P(8'd8, ri(`ALU_ADD,6'd7,6'd0,9'h055), NOP, NOP, NOP);
        // 9: STOP
        P(8'd9, {`OP_STOP,25'd0}, NOP, NOP, NOP);
        // 10: RETURN via r63 (-> 8)  ||  r8 = 0x99  (CTRL || ALU-imm)
        P(8'd10, jret(`CTRL_RETURN,6'd63), ri(`ALU_ADD,6'd8,6'd0,9'h099), NOP, NOP);

        // ---- run ----
        reset=1; start=0; repeat(2) @(posedge clk);
        reset=0; start=1;
        k=0;
        while (done!==1'b1 && k<200) begin @(posedge clk); k=k+1; end

        $display("============= r-VEX core program #2 =============");
        $display("  done=%b  cycles=%0d  (timeout counter=%0d)", done, cycles, k);
        chk ("gr1 (=10)",          dut.gr[1],  32'd10);
        chk ("gr2 (=3)",           dut.gr[2],  32'd3);
        chk ("gr3 (r1+r2=13)",     dut.gr[3],  32'd13);
        chk ("gr4 (MPYLL 30)",     dut.gr[4],  32'd30);
        chkb("br1 (r2<r1)",        dut.br[1],  1'b1);
        chk ("gr5 (poison skip=0)",dut.gr[5],  32'd0);
        chk ("gr6 (SLCT->r1=10)",  dut.gr[6],  32'd10);
        chk ("gr63 (CALL link=8)", dut.gr[63], 32'd8);
        chk ("gr7 (post-RET 0x55)",dut.gr[7],  32'h00000055);
        chk ("gr8 (subr 0x99)",    dut.gr[8],  32'h00000099);
        $display("=================================================");
        $display("CORE PROG #2: %0d passed, %0d failed", pass, fail);
        if (fail==0 && done===1'b1) $display("RESULT: CORE PROGRAM #2 PASSED");
        else $display("RESULT: CORE PROGRAM #2 FAILED");
        $finish;
    end
endmodule
