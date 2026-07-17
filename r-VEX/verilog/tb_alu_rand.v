//======================================================================
// r-VEX | Constrained-random ALU testbench (self-checking vs reference)
//----------------------------------------------------------------------
// Drives N random (opcode, src1, src2, cin) vectors into alu.v and checks
// result / cout / out_valid against an independent behavioural reference
// (alu_ref) that encodes the same ISA semantics documented in alu.v.
//
// The opcode table includes the four "top-nibble" ops (ADDCG/DIVS/SLCT/
// SLCTF, BR index 0) and two deliberately-invalid opcodes so the
// out_valid=0 path is exercised too. Seed is fixed for reproducibility.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_alu_rand;
    localparam integer N = 4000;
    integer seed;
    integer it, checks=0, fail=0;

    reg  [6:0]  aop;  reg [31:0] as1, as2; reg acin;
    wire [31:0] ares; wire acout, aval;
    alu u_alu(.aluop(aop),.src1(as1),.src2(as2),.cin(acin),
              .result(ares),.cout(acout),.out_valid(aval));

    // ---- opcode table (44 entries incl. 2 invalid) ----
    localparam integer NOPS = 44;
    reg [6:0] optab [0:NOPS-1];

    // ---- independent behavioural reference ----
    task alu_ref;
        input  [6:0] op; input [31:0] s1,s2; input c;
        output [31:0] r; output rc; output rv;
        reg [4:0]  sh; reg [32:0] add33; reg [31:0] dshift;
        begin
            r=32'd0; rc=1'b0; rv=1'b1; sh=s2[4:0];
            if (op[6:3]==`ALU_ADDCG4) begin
                add33 = {1'b0,s1} + {1'b0,s2} + {32'b0,c}; r=add33[31:0]; rc=add33[32];
            end else if (op[6:3]==`ALU_DIVS4) begin
                dshift = {s1[30:0],1'b0} + {31'b0,c};
                if (s1[31]) r = dshift + s2; else r = dshift - s2;
                rc = s1[31];
            end else if (op[6:3]==`ALU_SLCT4)  r = c ? s1 : s2;
            else if (op[6:3]==`ALU_SLCTF4)     r = c ? s2 : s1;
            else case (op)
                `ALU_ADD  : r = s1 + s2;
                `ALU_AND  : r = s1 & s2;
                `ALU_ANDC : r = (~s1) & s2;
                `ALU_OR   : r = s1 | s2;
                `ALU_ORC  : r = (~s1) | s2;
                `ALU_XOR  : r = s1 ^ s2;
                `ALU_MAX  : r = ($signed(s1) >= $signed(s2)) ? s1 : s2;
                `ALU_MAXU : r = (s1 >= s2) ? s1 : s2;
                `ALU_MIN  : r = ($signed(s1) <= $signed(s2)) ? s1 : s2;
                `ALU_MINU : r = (s1 <= s2) ? s1 : s2;
                `ALU_SH1ADD: r = (s1 << 1) + s2;
                `ALU_SH2ADD: r = (s1 << 2) + s2;
                `ALU_SH3ADD: r = (s1 << 3) + s2;
                `ALU_SH4ADD: r = (s1 << 4) + s2;
                `ALU_SHL  : r = s1 << sh;
                `ALU_SHR  : r = $signed(s1) >>> sh;
                `ALU_SHRU : r = s1 >> sh;
                `ALU_SUB  : r = s1 - s2;
                `ALU_SXTB : r = {{24{s1[7]}},  s1[7:0]};
                `ALU_SXTH : r = {{16{s1[15]}}, s1[15:0]};
                `ALU_ZXTB : r = s1 & 32'h0000_00FF;
                `ALU_ZXTH : r = s1 & 32'h0000_FFFF;
                `ALU_MOV  : r = s1;
                `ALU_CMPEQ : r = (s1 == s2) ? 32'd1 : 32'd0;
                `ALU_CMPNE : r = (s1 != s2) ? 32'd1 : 32'd0;
                `ALU_CMPGE : r = ($signed(s1) >= $signed(s2)) ? 32'd1 : 32'd0;
                `ALU_CMPGEU: r = (s1 >= s2) ? 32'd1 : 32'd0;
                `ALU_CMPGT : r = ($signed(s1) >  $signed(s2)) ? 32'd1 : 32'd0;
                `ALU_CMPGTU: r = (s1 >  s2) ? 32'd1 : 32'd0;
                `ALU_CMPLE : r = ($signed(s1) <= $signed(s2)) ? 32'd1 : 32'd0;
                `ALU_CMPLEU: r = (s1 <= s2) ? 32'd1 : 32'd0;
                `ALU_CMPLT : r = ($signed(s1) <  $signed(s2)) ? 32'd1 : 32'd0;
                `ALU_CMPLTU: r = (s1 <  s2) ? 32'd1 : 32'd0;
                `ALU_NANDL : r = ((s1==0)||(s2==0)) ? 32'd1 : 32'd0;
                `ALU_NORL  : r = ((s1==0)&&(s2==0)) ? 32'd1 : 32'd0;
                `ALU_ORL   : r = ((s1==0)&&(s2==0)) ? 32'd0 : 32'd1;
                `ALU_ANDL  : r = ((s1==0)||(s2==0)) ? 32'd0 : 32'd1;
                `ALU_MTB   : begin r = 32'd0; rc = s1[0]; end
                default    : begin r = 32'd0; rv = 1'b0; end
            endcase
        end
    endtask

    reg [31:0] xr; reg xc, xv;
    initial begin
        optab[0]=`ALU_ADD;   optab[1]=`ALU_AND;   optab[2]=`ALU_ANDC;  optab[3]=`ALU_MAX;
        optab[4]=`ALU_MAXU;  optab[5]=`ALU_MIN;   optab[6]=`ALU_MINU;  optab[7]=`ALU_OR;
        optab[8]=`ALU_ORC;   optab[9]=`ALU_SH1ADD;optab[10]=`ALU_SH2ADD;optab[11]=`ALU_SH3ADD;
        optab[12]=`ALU_SH4ADD;optab[13]=`ALU_SHL; optab[14]=`ALU_SHR;  optab[15]=`ALU_SHRU;
        optab[16]=`ALU_SUB;  optab[17]=`ALU_SXTB; optab[18]=`ALU_SXTH; optab[19]=`ALU_ZXTB;
        optab[20]=`ALU_ZXTH; optab[21]=`ALU_XOR;  optab[22]=`ALU_MOV;  optab[23]=`ALU_CMPEQ;
        optab[24]=`ALU_CMPGE;optab[25]=`ALU_CMPGEU;optab[26]=`ALU_CMPGT;optab[27]=`ALU_CMPGTU;
        optab[28]=`ALU_CMPLE;optab[29]=`ALU_CMPLEU;optab[30]=`ALU_CMPLT;optab[31]=`ALU_CMPLTU;
        optab[32]=`ALU_CMPNE;optab[33]=`ALU_NANDL;optab[34]=`ALU_NORL; optab[35]=`ALU_ORL;
        optab[36]=`ALU_MTB;  optab[37]=`ALU_ANDL; optab[38]=`ALU_ADDCG;optab[39]=`ALU_DIVS;
        optab[40]=`ALU_SLCT; optab[41]=`ALU_SLCTF;
        optab[42]=`OP_STOP;  optab[43]=`OP_SEND;   // invalid -> out_valid must be 0

        seed = 32'h1234_5678;
        $display("========= r-VEX ALU constrained-random (%0d vectors) =========", N);

        for (it=0; it<N; it=it+1) begin
            aop  = optab[{$random(seed)} % NOPS];
            as1  = $random(seed);
            as2  = $random(seed);
            acin = $random(seed);
            #1;
            alu_ref(aop, as1, as2, acin, xr, xc, xv);
            checks = checks + 3;
            if (aval !== xv) begin
                fail=fail+1;
                $display("  FAIL[%0d] op=%b valid: dut=%b ref=%b (s1=%h s2=%h c=%b)",
                         it, aop, aval, xv, as1, as2, acin);
            end
            // result & cout only meaningful when the op is valid
            if (xv) begin
                if (ares !== xr) begin fail=fail+1;
                    $display("  FAIL[%0d] op=%b result: dut=%h ref=%h (s1=%h s2=%h c=%b)",
                             it, aop, ares, xr, as1, as2, acin); end
                if (acout !== xc) begin fail=fail+1;
                    $display("  FAIL[%0d] op=%b cout: dut=%b ref=%b (s1=%h s2=%h c=%b)",
                             it, aop, acout, xc, as1, as2, acin); end
            end
        end

        $display("==============================================================");
        $display("ALU RANDOM: %0d checks, %0d failed", checks, fail);
        if (fail==0) $display("RESULT: ALU RANDOM PASSED");
        else         $display("RESULT: %0d ALU RANDOM FAILURES", fail);
        $finish;
    end
endmodule
