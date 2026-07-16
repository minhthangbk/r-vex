//======================================================================
// r-VEX | Directed self-checking testbench for the execution units
//----------------------------------------------------------------------
// Proves each bug fix and new instruction at the datapath level.
// Golden values are derived from the ISA semantics and the NotebookLM
// guideline formulas (sign-extend {{n{msb}},x}; unsigned '*' vs $signed).
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_units;
    integer pass = 0, fail = 0;

    // ---- ALU ----
    reg  [6:0] aop; reg [31:0] as1, as2; reg acin;
    wire [31:0] ares; wire acout, aval;
    alu u_alu(.aluop(aop),.src1(as1),.src2(as2),.cin(acin),.result(ares),.cout(acout),.out_valid(aval));

    // ---- MUL ----
    reg  [6:0] mop; reg [31:0] ms1, ms2;
    wire [31:0] mres; wire mov, mval;
    mul u_mul(.mulop(mop),.src1(ms1),.src2(ms2),.result(mres),.overflow(mov),.out_valid(mval));

    // ---- MEM ----
    reg  [6:0] eop; reg [31:0] emem, ereg; reg [1:0] epos;
    wire [31:0] eload, estore; wire eisl, eiss, eval;
    mem_unit u_mem(.opcode(eop),.mem_val(emem),.reg_val(ereg),.pos(epos),
                   .load_data(eload),.store_data(estore),.is_load(eisl),.is_store(eiss),.out_valid(eval));

    task chk; input [127:0] name; input [31:0] got, exp; begin
        if (got === exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h", name, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", name, got, exp); end
    end endtask
    task chkb; input [127:0] name; input got, exp; begin
        if (got === exp) begin pass=pass+1; $display("  PASS %0s : %b", name, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got %b exp %b", name, got, exp); end
    end endtask

    initial begin
        $display("================ r-VEX unit tests ================");

        //------------------------------------------------ ALU sanity
        $display("[ALU]");
        aop=`ALU_ADD;  as1=32'd7;         as2=32'd35;        acin=0; #1 chk("ADD 7+35",        ares,32'd42);
        aop=`ALU_SUB;  as1=32'd50;        as2=32'd8;         acin=0; #1 chk("SUB 50-8",        ares,32'd42);
        aop=`ALU_SHR;  as1=32'hFFFFFFF0;  as2=32'd4;         acin=0; #1 chk("SHR arith",       ares,32'hFFFFFFFF);
        aop=`ALU_SHRU; as1=32'hFFFFFFF0;  as2=32'd4;         acin=0; #1 chk("SHRU logical",    ares,32'h0FFFFFFF);
        aop=`ALU_SXTB; as1=32'h00000080;  as2=32'd0;         acin=0; #1 chk("SXTB 0x80",       ares,32'hFFFFFF80);
        aop=`ALU_CMPLT;as1=-32'sd5;       as2=32'sd3;        acin=0; #1 chk("CMPLT -5<3",      ares,32'd1);
        aop=`ALU_ADDCG;as1=32'hFFFFFFFF;  as2=32'd1;         acin=0; #1 chkb("ADDCG carry",    acout,1'b1);
        aop=`ALU_MTB;  as1=32'h00000001;  as2=32'd0;         acin=0; #1 chkb("MTB bit",        acout,1'b1);
        aop=`ALU_SLCT; as1=32'hAAAA;      as2=32'h5555;      acin=1; #1 chk("SLCT true",       ares,32'hAAAA);
        aop=`ALU_SLCTF;as1=32'hAAAA;      as2=32'h5555;      acin=1; #1 chk("SLCTF true->s2",  ares,32'h5555);

        //------------------------------------------------ MUL fixes
        $display("[MUL] (bug fixes)");
        // MPYLL sanity (signed low*low)
        mop=`MUL_MPYLL;  ms1=32'h00000003; ms2=32'h00000004; #1 chk("MPYLL 3*4", mres,32'd12);
        // MPYHHU: unsigned high*high. high1=0x8000 high2=0x0001 -> 0x8000 (signed would be 0xFFFF8000)
        mop=`MUL_MPYHHU; ms1=32'h80000000; ms2=32'h00010000; #1 chk("MPYHHU 0x8000*1 unsigned", mres,32'h00008000);
        // MPYHU: unsigned 32x(high16). s1=1, s2 high=0x8000 -> 0x8000 (signed would be 0xFFFF8000)
        mop=`MUL_MPYHU;  ms1=32'h00000001; ms2=32'h80000000; #1 chk("MPYHU 1*0x8000 unsigned", mres,32'h00008000);
        // MPYHS: signed 32 x high16 then <<16. s1=1, s2 high=1 -> (1)<<16 = 0x00010000 (buggy MPYH gave 1)
        mop=`MUL_MPYHS;  ms1=32'h00000001; ms2=32'h00010000; #1 chk("MPYHS 1*1 <<16", mres,32'h00010000);
        // MPYHH still signed (regression guard): (-32768)*(-32768)=+0x40000000
        mop=`MUL_MPYHH;  ms1=32'h80000000; ms2=32'h80000000; #1 chk("MPYHH signed sq", mres,32'h40000000);

        //------------------------------------------------ MEM fixes + PFT
        $display("[MEM] (bug fixes + PFT)");
        // LDB sign-extend: pos=11 -> mem[7:0]=0x80 -> 0xFFFFFF80
        eop=`MEM_LDB;  emem=32'h00000080; ereg=0; epos=2'b11; #1 chk("LDB  sign 0x80",  eload,32'hFFFFFF80);
        eop=`MEM_LDBU; emem=32'h00000080; ereg=0; epos=2'b11; #1 chk("LDBU zero 0x80",  eload,32'h00000080);
        // LDH sign-extend: pos=10 -> mem[15:0]=0x8000 -> 0xFFFF8000
        eop=`MEM_LDH;  emem=32'h00008000; ereg=0; epos=2'b10; #1 chk("LDH  sign 0x8000",eload,32'hFFFF8000);
        eop=`MEM_LDHU; emem=32'h00008000; ereg=0; epos=2'b10; #1 chk("LDHU zero 0x8000",eload,32'h00008000);
        // STW sanity
        eop=`MEM_STW;  emem=32'h0; ereg=32'hDEADBEEF; epos=2'b00; #1 chk("STW",estore,32'hDEADBEEF);
        // PFT: valid no-op, no load/store
        eop=`MEM_PFT;  emem=32'h12345678; ereg=0; epos=2'b00; #1 begin
            chkb("PFT valid",   eval, 1'b1);
            chkb("PFT no-load",  eisl, 1'b0);
            chkb("PFT no-store", eiss, 1'b0);
        end

        $display("==================================================");
        $display("UNIT TESTS: %0d passed, %0d failed", pass, fail);
        if (fail==0) $display("RESULT: ALL UNIT TESTS PASSED");
        else         $display("RESULT: %0d UNIT TEST FAILURES", fail);
        $finish;
    end
endmodule
