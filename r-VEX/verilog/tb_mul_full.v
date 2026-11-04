//======================================================================
// r-VEX | Exhaustive multiplier directed testbench
//----------------------------------------------------------------------
// Covers every mul.v opcode, including the ones tb_units.v skipped:
//   - all 16x16->32 variants (MPYLL/LLU/LH/LHU/HH/HHU), signed vs unsigned
//   - the 32x16 variants (MPYL/MPYLU/MPYHU/MPYH/MPYHS), truncating mod 2^32
//     (VEX ISA, reviews/contracts/mul32.md; r10: they saturated before),
//     in range and wrapping, and the overflow flag that stays 0
//   - OP_NOP -> 0, and invalid opcode -> out_valid=0
// The wrap cases use product == 2^32 (0x1_0000_0000), the old saturation point.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_mul_full;
    integer pass = 0, fail = 0;

    reg  [6:0]  mop;
    reg  [31:0] ms1, ms2;
    wire [31:0] mres;
    wire        mov, mval;

    mul u_mul(.mulop(mop),.src1(ms1),.src2(ms2),
              .result(mres),.overflow(mov),.out_valid(mval));

    task chk; input [127:0] name; input [31:0] got, exp; begin
        if (got === exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h", name, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", name, got, exp); end
    end endtask
    task chkb; input [127:0] name; input got, exp; begin
        if (got === exp) begin pass=pass+1; $display("  PASS %0s : %b", name, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got %b exp %b", name, got, exp); end
    end endtask

    task drive; input [6:0] op; input [31:0] s1,s2; begin
        mop=op; ms1=s1; ms2=s2; #1;
    end endtask

    initial begin
        $display("================ r-VEX MUL full tests ================");

        //---------------------------------------- 16x16 -> 32
        $display("[16x16->32]");
        drive(`MUL_MPYLL ,32'h00000003,32'h00000004); chk("MPYLL 3*4"       ,mres,32'd12);
        drive(`MUL_MPYLL ,32'hFFFFFFFE,32'h00000003); chk("MPYLL -2*3 signed",mres,32'hFFFFFFFA);
        drive(`MUL_MPYLLU,32'h0000FFFF,32'h00000002); chk("MPYLLU 0xFFFF*2"  ,mres,32'h0001FFFE);
        drive(`MUL_MPYLH ,32'h00000003,32'h00040000); chk("MPYLH lo*hi 3*4"  ,mres,32'd12);
        drive(`MUL_MPYLHU,32'h0000FFFF,32'h00020000); chk("MPYLHU 0xFFFF*2"  ,mres,32'h0001FFFE);
        drive(`MUL_MPYHH ,32'h80000000,32'h80000000); chk("MPYHH (-2^15)^2"  ,mres,32'h40000000); // signed
        drive(`MUL_MPYHHU,32'h80000000,32'h00010000); chk("MPYHHU 0x8000*1"  ,mres,32'h00008000); // unsigned

        //---------------------------------------- 32x16, truncating mod 2^32 (mul32.md MU-01..06)
        // r10: the products 2^32 and 2^32+... that used to saturate now wrap to their low word
        $display("[MPYL 32 x INT16(s2) truncating]");
        drive(`MUL_MPYL,32'h00000003,32'h00000004); chk ("MPYL 3*4"            ,mres,32'd12);
        drive(`MUL_MPYL,32'hFFFFFFFE,32'h00000003); chk ("MPYL -2*3 signed"    ,mres,32'hFFFFFFFA);
        drive(`MUL_MPYL,32'h00100000,32'h00001000); chk ("MPYL 2^32 wraps"     ,mres,32'h00000000);
        drive(`MUL_MPYL,32'h00100000,32'h00001000); chkb("MPYL no overflow"    ,mov,1'b0);
        drive(`MUL_MPYL,32'h7FFFFFFF,32'h1234FFFF); chk ("MPYL max*-1 (s2 hi ignored)",mres,32'h80000001);

        $display("[MPYLU 32 x UINT16(s2) truncating]");
        drive(`MUL_MPYLU,32'h00000003,32'h00000004); chk ("MPYLU 3*4"          ,mres,32'd12);
        drive(`MUL_MPYLU,32'h00100000,32'h00001000); chk ("MPYLU 2^32 wraps"   ,mres,32'h00000000);
        drive(`MUL_MPYLU,32'h00100000,32'h00001000); chkb("MPYLU no overflow"  ,mov,1'b0);
        drive(`MUL_MPYLU,32'hFFFFFFFF,32'h0000FFFF); chk ("MPYLU -1*65535"     ,mres,32'hFFFF0001);
        drive(`MUL_MPYLU,32'hFFFFFFFD,32'h00000007); chk ("MPYLU -3*7 (BUG-008)",mres,32'hFFFFFFEB);

        $display("[MPYHU 32 x UINT16(s2>>16) truncating]");
        drive(`MUL_MPYHU,32'h00000001,32'h80000000); chk ("MPYHU 1*0x8000"     ,mres,32'h00008000);
        drive(`MUL_MPYHU,32'h00100000,32'h10000000); chk ("MPYHU 2^32 wraps"   ,mres,32'h00000000);
        drive(`MUL_MPYHU,32'h00100000,32'h10000000); chkb("MPYHU no overflow"  ,mov,1'b0);
        drive(`MUL_MPYHU,32'hFFFFFFFF,32'hFFFF1234); chk ("MPYHU -1*0xFFFF"    ,mres,32'hFFFF0001);

        $display("[MPYH 32 x INT16(s2>>16) truncating]");
        drive(`MUL_MPYH,32'h00000003,32'h00020000); chk ("MPYH 3*hi(2)"        ,mres,32'd6);
        drive(`MUL_MPYH,32'h00100000,32'h10000000); chk ("MPYH 2^32 wraps"     ,mres,32'h00000000);
        drive(`MUL_MPYH,32'h00100000,32'h10000000); chkb("MPYH no overflow"    ,mov,1'b0);
        drive(`MUL_MPYH,32'h00000005,32'hFFFE0000); chk ("MPYH 5*hi(-2)"       ,mres,32'hFFFFFFF6);

        $display("[MPYHS (32 x INT16(s2>>16)) << 16 truncating]");
        drive(`MUL_MPYHS,32'h00000001,32'h00010000); chk ("MPYHS 1*1 <<16"     ,mres,32'h00010000);
        drive(`MUL_MPYHS,32'h00000003,32'h00020000); chk ("MPYHS 3*2 <<16"     ,mres,32'h00060000);
        drive(`MUL_MPYHS,32'h00100000,32'h10000000); chk ("MPYHS 2^48 wraps"   ,mres,32'h00000000);
        drive(`MUL_MPYHS,32'h00100000,32'h10000000); chkb("MPYHS no overflow"  ,mov,1'b0);
        drive(`MUL_MPYHS,32'h12345678,32'h00030000); chk ("MPYHS 0x12345678*3" ,mres,32'h03680000);
        drive(`MUL_MPYHS,32'hFFFFFFFF,32'h00010000); chk ("MPYHS -1*1 <<16"    ,mres,32'hFFFF0000);
        drive(`MUL_MPYHS,32'h00000007,32'hFFFFFFFD); chk ("MPYHS 7*hi(-3) <<16",mres,32'hFFF90000);

        // MU-07 identity on the two words of -3*7 / 7*-3: mpylu + mpyhs == a*b mod 2^32
        drive(`MUL_MPYLU,32'h00000007,32'hFFFFFFFD); chk ("MPYLU 7*lo(-3)"     ,mres,32'h0006FFEB);

        //---------------------------------------- NOP / invalid
        $display("[nop/invalid]");
        drive(`OP_NOP ,32'hDEAD,32'hBEEF); chk ("NOP -> 0"      ,mres,32'd0);
        drive(`OP_NOP ,32'hDEAD,32'hBEEF); chkb("NOP valid"     ,mval,1'b1);
        drive(`OP_STOP,32'hDEAD,32'hBEEF); chkb("bad op invalid",mval,1'b0);

        $display("======================================================");
        $display("MUL FULL: %0d passed, %0d failed", pass, fail);
        if (fail==0) $display("RESULT: ALL MUL FULL TESTS PASSED");
        else         $display("RESULT: %0d MUL FULL TEST FAILURES", fail);
        $finish;
    end
endmodule
