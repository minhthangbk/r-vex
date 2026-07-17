//======================================================================
// r-VEX | Exhaustive ALU directed testbench
//----------------------------------------------------------------------
// Covers every ALU opcode path in alu.v that the original tb_units.v
// left untested: full arithmetic/logical set, all 10 compares, the four
// logical-boolean ops (NANDL/NORL/ORL/ANDL), SHxADD, shifts, sign/zero
// extends, ADDCG (result + carry, both carry-in and carry-out), the
// DIVS division step (both sign branches), SLCT/SLCTF on both select
// values, MTB set/clear, and the invalid-opcode -> out_valid=0 path.
// Golden values are derived from the ISA semantics documented in alu.v.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_alu_full;
    integer pass = 0, fail = 0;

    reg  [6:0]  aop;
    reg  [31:0] as1, as2;
    reg         acin;
    wire [31:0] ares;
    wire        acout, aval;

    alu u_alu(.aluop(aop),.src1(as1),.src2(as2),.cin(acin),
              .result(ares),.cout(acout),.out_valid(aval));

    task chk; input [127:0] name; input [31:0] got, exp; begin
        if (got === exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h", name, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", name, got, exp); end
    end endtask
    task chkb; input [127:0] name; input got, exp; begin
        if (got === exp) begin pass=pass+1; $display("  PASS %0s : %b", name, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got %b exp %b", name, got, exp); end
    end endtask

    // helper: drive op/src and settle
    task drive; input [6:0] op; input [31:0] s1,s2; input c; begin
        aop=op; as1=s1; as2=s2; acin=c; #1;
    end endtask

    initial begin
        $display("================ r-VEX ALU full tests ================");

        //---------------------------------------- arithmetic / logical
        $display("[arith/logical]");
        drive(`ALU_ADD ,32'd7        ,32'd35       ,0); chk("ADD 7+35"     ,ares,32'd42);
        drive(`ALU_SUB ,32'd50       ,32'd8        ,0); chk("SUB 50-8"     ,ares,32'd42);
        drive(`ALU_AND ,32'h0000F0F0 ,32'h0000FF00 ,0); chk("AND"          ,ares,32'h0000F000);
        drive(`ALU_ANDC,32'h00000F0F ,32'h0000FF00 ,0); chk("ANDC ~s1&s2"  ,ares,32'h0000F000);
        drive(`ALU_OR  ,32'h000000F0 ,32'h00000F00 ,0); chk("OR"           ,ares,32'h00000FF0);
        drive(`ALU_ORC ,32'h0000000F ,32'h00000000 ,0); chk("ORC ~s1|s2"   ,ares,32'hFFFFFFF0);
        drive(`ALU_XOR ,32'h0000FF00 ,32'h00000FF0 ,0); chk("XOR"          ,ares,32'h0000F0F0);
        drive(`ALU_MOV ,32'hCAFEBABE ,32'd0        ,0); chk("MOV s1"       ,ares,32'hCAFEBABE);

        //---------------------------------------- min/max signed + unsigned
        $display("[min/max]");
        drive(`ALU_MAX ,32'hFFFFFFFB ,32'd3 ,0); chk("MAX  s(-5,3)"  ,ares,32'd3);          // signed
        drive(`ALU_MAXU,32'hFFFFFFFB ,32'd3 ,0); chk("MAXU u(-5,3)"  ,ares,32'hFFFFFFFB);   // unsigned
        drive(`ALU_MIN ,32'hFFFFFFFB ,32'd3 ,0); chk("MIN  s(-5,3)"  ,ares,32'hFFFFFFFB);
        drive(`ALU_MINU,32'hFFFFFFFB ,32'd3 ,0); chk("MINU u(-5,3)"  ,ares,32'd3);

        //---------------------------------------- SHxADD + shifts
        $display("[shift/shadd]");
        drive(`ALU_SH1ADD,32'd5,32'd3,0); chk("SH1ADD (5<<1)+3",ares,32'd13);
        drive(`ALU_SH2ADD,32'd5,32'd3,0); chk("SH2ADD (5<<2)+3",ares,32'd23);
        drive(`ALU_SH3ADD,32'd5,32'd3,0); chk("SH3ADD (5<<3)+3",ares,32'd43);
        drive(`ALU_SH4ADD,32'd5,32'd3,0); chk("SH4ADD (5<<4)+3",ares,32'd83);
        drive(`ALU_SHL ,32'd1        ,32'd4,0); chk("SHL 1<<4"    ,ares,32'd16);
        drive(`ALU_SHR ,32'hFFFFFFF0 ,32'd4,0); chk("SHR arith"   ,ares,32'hFFFFFFFF);
        drive(`ALU_SHRU,32'hFFFFFFF0 ,32'd4,0); chk("SHRU logical",ares,32'h0FFFFFFF);

        //---------------------------------------- extends
        $display("[sxt/zxt]");
        drive(`ALU_SXTB,32'h00000080,0,0); chk("SXTB 0x80"  ,ares,32'hFFFFFF80);
        drive(`ALU_SXTH,32'h00008000,0,0); chk("SXTH 0x8000",ares,32'hFFFF8000);
        drive(`ALU_ZXTB,32'h12345678,0,0); chk("ZXTB"       ,ares,32'h00000078);
        drive(`ALU_ZXTH,32'h12345678,0,0); chk("ZXTH"       ,ares,32'h00005678);

        //---------------------------------------- compares (result 1/0)
        $display("[compares]");
        drive(`ALU_CMPEQ ,32'd5,32'd5,0); chk("CMPEQ 5==5"  ,ares,32'd1);
        drive(`ALU_CMPEQ ,32'd5,32'd6,0); chk("CMPEQ 5==6"  ,ares,32'd0);
        drive(`ALU_CMPNE ,32'd5,32'd6,0); chk("CMPNE 5!=6"  ,ares,32'd1);
        drive(`ALU_CMPGE ,32'd3,32'hFFFFFFFB,0); chk("CMPGE  s 3>=-5",ares,32'd1);
        drive(`ALU_CMPGE ,32'hFFFFFFFB,32'd3,0); chk("CMPGE  s -5>=3",ares,32'd0);
        drive(`ALU_CMPGEU,32'd3,32'hFFFFFFFB,0); chk("CMPGEU u 3>=big",ares,32'd0);
        drive(`ALU_CMPGT ,32'd3,32'hFFFFFFFB,0); chk("CMPGT  s 3>-5" ,ares,32'd1);
        drive(`ALU_CMPGTU,32'hFFFFFFFB,32'd3,0); chk("CMPGTU u big>3",ares,32'd1);
        drive(`ALU_CMPLE ,32'hFFFFFFFB,32'd3,0); chk("CMPLE  s -5<=3",ares,32'd1);
        drive(`ALU_CMPLEU,32'd3,32'hFFFFFFFB,0); chk("CMPLEU u 3<=big",ares,32'd1);
        drive(`ALU_CMPLT ,32'hFFFFFFFB,32'd3,0); chk("CMPLT  s -5<3" ,ares,32'd1);
        drive(`ALU_CMPLTU,32'd3,32'hFFFFFFFB,0); chk("CMPLTU u 3<big",ares,32'd1);

        //---------------------------------------- boolean logicals
        $display("[bool logicals]");
        drive(`ALU_NANDL,32'd0,32'd5,0); chk("NANDL(0,5)=1",ares,32'd1);
        drive(`ALU_NANDL,32'd5,32'd5,0); chk("NANDL(5,5)=0",ares,32'd0);
        drive(`ALU_NORL ,32'd0,32'd0,0); chk("NORL(0,0)=1" ,ares,32'd1);
        drive(`ALU_NORL ,32'd0,32'd5,0); chk("NORL(0,5)=0" ,ares,32'd0);
        drive(`ALU_ORL  ,32'd0,32'd0,0); chk("ORL(0,0)=0"  ,ares,32'd0);
        drive(`ALU_ORL  ,32'd0,32'd5,0); chk("ORL(0,5)=1"  ,ares,32'd1);
        drive(`ALU_ANDL ,32'd5,32'd5,0); chk("ANDL(5,5)=1" ,ares,32'd1);
        drive(`ALU_ANDL ,32'd0,32'd5,0); chk("ANDL(0,5)=0" ,ares,32'd0);

        //---------------------------------------- MTB (BR write via cout)
        $display("[MTB]");
        drive(`ALU_MTB,32'h00000001,0,0); chkb("MTB set  s1[0]=1",acout,1'b1);
        drive(`ALU_MTB,32'h00000000,0,0); chkb("MTB clr  s1[0]=0",acout,1'b0);

        //---------------------------------------- ADDCG (result + carry)
        $display("[ADDCG]");
        drive(`ALU_ADDCG,32'd5,32'd3,1);          chk ("ADDCG 5+3+ci"    ,ares,32'd9);
        drive(`ALU_ADDCG,32'd5,32'd3,1);          chkb("ADDCG no carry"  ,acout,1'b0);
        drive(`ALU_ADDCG,32'hFFFFFFFF,32'd1,0);   chk ("ADDCG wrap res"  ,ares,32'd0);
        drive(`ALU_ADDCG,32'hFFFFFFFF,32'd1,0);   chkb("ADDCG carry-out" ,acout,1'b1);

        //---------------------------------------- DIVS (division step)
        $display("[DIVS]");
        // s1[31]=0 -> result = (s1<<1)+ci - s2 ; cout = s1[31]
        drive(`ALU_DIVS,32'd4,32'd2,0);           chk ("DIVS +: 8-2"     ,ares,32'd6);
        drive(`ALU_DIVS,32'd4,32'd2,0);           chkb("DIVS + cout"     ,acout,1'b0);
        // s1[31]=1 -> result = (s1[30:0]<<1)+ci + s2 ; cout = 1
        drive(`ALU_DIVS,32'h80000004,32'd2,1);    chk ("DIVS -: 9+2"     ,ares,32'd11);
        drive(`ALU_DIVS,32'h80000004,32'd2,1);    chkb("DIVS - cout"     ,acout,1'b1);

        //---------------------------------------- SLCT / SLCTF (both paths)
        $display("[SLCT/SLCTF]");
        drive(`ALU_SLCT ,32'hAAAA,32'h5555,1); chk("SLCT  cin=1->s1" ,ares,32'h0000AAAA);
        drive(`ALU_SLCT ,32'hAAAA,32'h5555,0); chk("SLCT  cin=0->s2" ,ares,32'h00005555);
        drive(`ALU_SLCTF,32'hAAAA,32'h5555,1); chk("SLCTF cin=1->s2" ,ares,32'h00005555);
        drive(`ALU_SLCTF,32'hAAAA,32'h5555,0); chk("SLCTF cin=0->s1" ,ares,32'h0000AAAA);

        //---------------------------------------- validity
        $display("[valid/invalid]");
        drive(`ALU_ADD,32'd1,32'd1,0); chkb("valid op -> out_valid=1"  ,aval,1'b1);
        drive(`OP_STOP,32'd1,32'd1,0); chkb("bad op   -> out_valid=0"  ,aval,1'b0);
        chk("bad op result cleared", ares, 32'd0);

        $display("======================================================");
        $display("ALU FULL: %0d passed, %0d failed", pass, fail);
        if (fail==0) $display("RESULT: ALL ALU FULL TESTS PASSED");
        else         $display("RESULT: %0d ALU FULL TEST FAILURES", fail);
        $finish;
    end
endmodule
