//======================================================================
// r-VEX | Core program #3 : ADDCG carry-chain + memory round-trip
//----------------------------------------------------------------------
// Exercises two integration paths not covered by tb_core / tb_core_prog:
//
//  1) A 64-bit add built from two chained ADDCG syllables. The low ADDCG
//     produces a carry into a Branch Register (dbr = syl[4:2]); the high
//     ADDCG consumes it as its carry-in (source BR index = opcode[2:0]).
//        A = 0x00000020_80000000 , B = 0x00000010_80000000
//        sum = 0x00000031_00000000   (low=0 with carry, high=0x31)
//
//  2) A store/load round-trip through real data memory:
//        STH r16[15:0]=0xABCD into dmem[4].[31:16]  -> 0xABCD3344
//        LDHU -> 0x0000ABCD ,  LDH -> 0xFFFFABCD (sign)
//        STB r17[7:0]=0x77   into dmem[4].[7:0]      -> 0xABCD3377
//        LDBU -> 0x00000077
//
// Field layout identical to rvex_core.v decode (see tb_core_prog.v).
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_core_carry;
    reg clk=0, reset=1, start=0;
    wire done; wire [31:0] cycles;
    integer pass=0, fail=0;

    rvex_core dut(.clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles));
    always #5 clk = ~clk;

    // ---- encoders ----
    function [31:0] rr;  input [6:0] op; input [5:0] d,a,b; begin
        rr = {op,`NO_IMM,d,a,b,3'b000,2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    // ADDCG: source-BR index in opcode[2:0], dest-carry BR in syl[4:2]
    function [31:0] addcg; input [5:0] d,a,b; input [2:0] sbr,dbr; begin
        addcg = {{`ALU_ADDCG4,sbr},`NO_IMM,d,a,b,dbr,2'b00}; end endfunction
    // memory load : d=[22:17] base=[16:11] off9(bytes)=[10:2]
    function [31:0] mld; input [6:0] op; input [5:0] d,base; input [8:0] off; begin
        mld = {op,`NO_IMM,d,base,off,2'b00}; end endfunction
    // memory store: store-src=[22:17] base=[16:11] off9=[10:2]
    function [31:0] mst; input [6:0] op; input [5:0] sreg,base; input [8:0] off; begin
        mst = {op,`NO_IMM,sreg,base,off,2'b00}; end endfunction

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

    integer k;
    initial begin
        for (k=0;k<256;k=k+1) begin dut.imem[k]=128'd0; dut.dmem[k]=32'd0; end
        dut.dmem[4] = 32'h11223344;   // seed word for the memory round-trip

        //--------- build operands ---------
        // 0: r10 = 1
        P(8'd0, ri(`ALU_ADD,6'd10,6'd0,9'd1), NOP, NOP, NOP);
        // 1: r11 = r10 << 31 = 0x80000000   (low word of both A and B)
        P(8'd1, ri(`ALU_SHL,6'd11,6'd10,9'd31), NOP, NOP, NOP);
        // 2: r12 = 0x20 (A_hi)  ||  r13 = 0x10 (B_hi)
        P(8'd2, ri(`ALU_ADD,6'd12,6'd0,9'h020), ri(`ALU_ADD,6'd13,6'd0,9'h010), NOP, NOP);

        //--------- 64-bit add via chained ADDCG ---------
        // 3: ADDCG r14 = r11 + r11 + br0(0) ; carry -> br7   (=0, carry 1)
        P(8'd3, addcg(6'd14,6'd11,6'd11,3'd0,3'd7), NOP, NOP, NOP);
        // 4: ADDCG r15 = r12 + r13 + br7(1) ; carry -> br6   (=0x31)
        P(8'd4, addcg(6'd15,6'd12,6'd13,3'd7,3'd6), NOP, NOP, NOP);

        //--------- build 0xABCD and 0x77 for the memory test ---------
        // 5: r18 = 0xAB
        P(8'd5, ri(`ALU_ADD,6'd18,6'd0,9'h0AB), NOP, NOP, NOP);
        // 6: r19 = r18 << 8 = 0xAB00
        P(8'd6, ri(`ALU_SHL,6'd19,6'd18,9'd8), NOP, NOP, NOP);
        // 7: r16 = r19 | 0xCD = 0xABCD  ||  r17 = 0x77
        P(8'd7, ri(`ALU_OR,6'd16,6'd19,9'h0CD), ri(`ALU_ADD,6'd17,6'd0,9'h077), NOP, NOP);

        //--------- memory round-trip (slot 3) ---------
        // 8: STH dmem[4].[31:16] = r16[15:0]   (base r0, byte off 16, pos0)
        P(8'd8, NOP, NOP, NOP, mst(`MEM_STH,6'd16,6'd0,9'd16));
        // 9: r20 = LDHU dmem[4] pos0 -> 0x0000ABCD
        P(8'd9, NOP, NOP, NOP, mld(`MEM_LDHU,6'd20,6'd0,9'd16));
        // 10: r21 = LDH  dmem[4] pos0 -> 0xFFFFABCD (sign)
        P(8'd10, NOP, NOP, NOP, mld(`MEM_LDH,6'd21,6'd0,9'd16));
        // 11: STB dmem[4].[7:0] = r17[7:0]     (byte off 19, pos3)
        P(8'd11, NOP, NOP, NOP, mst(`MEM_STB,6'd17,6'd0,9'd19));
        // 12: r22 = LDBU dmem[4] pos3 -> 0x00000077
        P(8'd12, NOP, NOP, NOP, mld(`MEM_LDBU,6'd22,6'd0,9'd19));
        // 13: STOP
        P(8'd13, {`OP_STOP,25'd0}, NOP, NOP, NOP);

        // ---- run ----
        reset=1; start=0; repeat(2) @(posedge clk);
        reset=0; start=1;
        k=0;
        while (done!==1'b1 && k<300) begin @(posedge clk); k=k+1; end

        $display("========== r-VEX core program #3 (ADDCG + mem) ==========");
        $display("  done=%b  cycles=%0d", done, cycles);
        // --- ADDCG carry chain ---
        chk ("r11 (0x80000000)",     dut.gr[11], 32'h80000000);
        chk ("r14 ADDCG-lo sum",     dut.gr[14], 32'h00000000);
        chkb("br7 carry-out",        dut.br[7],  1'b1);
        chk ("r15 ADDCG-hi (0x31)",  dut.gr[15], 32'h00000031);
        chkb("br6 hi carry-out",     dut.br[6],  1'b0);
        // --- memory round-trip ---
        chk ("r16 (0xABCD)",         dut.gr[16], 32'h0000ABCD);
        chk ("dmem[4] after STB",    dut.dmem[4],32'hABCD3377);
        chk ("r20 LDHU (0x0000ABCD)",dut.gr[20], 32'h0000ABCD);
        chk ("r21 LDH  (0xFFFFABCD)",dut.gr[21], 32'hFFFFABCD);
        chk ("r22 LDBU (0x00000077)",dut.gr[22], 32'h00000077);
        $display("=========================================================");
        $display("CORE PROG #3: %0d passed, %0d failed", pass, fail);
        if (fail==0 && done===1'b1) $display("RESULT: CORE PROGRAM #3 PASSED");
        else $display("RESULT: CORE PROGRAM #3 FAILED");
        $finish;
    end
endmodule
