//======================================================================
// r-VEX | Constrained-random multiplier testbench (self-checking)
//----------------------------------------------------------------------
// Drives N random (opcode, src1, src2) vectors into mul.v and checks
// result / overflow / out_valid against an independent behavioural
// reference (mul_ref) encoding the ISA semantics from mul.v, including
// the 48-bit saturating variants and the (positive-only) overflow rule
// $signed(product[47:32]) > 0.
//
// Random operands are biased toward small magnitudes half the time so
// both the in-range and the saturating paths get frequent coverage.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_mul_rand;
    localparam integer N = 4000;
    integer seed;
    integer it, checks=0, fail=0;

    reg  [6:0]  mop; reg [31:0] ms1, ms2;
    wire [31:0] mres; wire mov, mval;
    mul u_mul(.mulop(mop),.src1(ms1),.src2(ms2),
              .result(mres),.overflow(mov),.out_valid(mval));

    localparam integer NOPS = 13;
    reg [6:0] optab [0:NOPS-1];

    // ---- independent behavioural reference ----
    task mul_ref;
        input  [6:0] op; input [31:0] s1,s2;
        output [31:0] r; output ov; output rv;
        reg signed [47:0] ps; reg [47:0] pu;
        begin
            r=32'd0; ov=1'b0; rv=1'b1; ps=48'sd0; pu=48'd0;
            case (op)
                `MUL_MPYLL : r = $signed(s1[15:0])  * $signed(s2[15:0]);
                `MUL_MPYLLU: r = s1[15:0]           * s2[15:0];
                `MUL_MPYLH : r = $signed(s1[15:0])  * $signed(s2[31:16]);
                `MUL_MPYLHU: r = s1[15:0]           * s2[31:16];
                `MUL_MPYHH : r = $signed(s1[31:16]) * $signed(s2[31:16]);
                `MUL_MPYHHU: r = s1[31:16]          * s2[31:16];
                `MUL_MPYL : begin
                    ps = $signed(s1) * $signed(s2[15:0]);
                    if ($signed(ps[47:32]) > 0) begin ov=1'b1; r = ps[47] ? 32'h8000_0000 : 32'h7FFF_FFFF; end
                    else r = ps[31:0];
                end
                `MUL_MPYH : begin
                    ps = $signed(s1) * $signed(s2[31:16]);
                    if ($signed(ps[47:32]) > 0) begin ov=1'b1; r = ps[47] ? 32'h8000_0000 : 32'h7FFF_FFFF; end
                    else r = ps[31:0];
                end
                `MUL_MPYLU: begin
                    pu = s1 * s2[15:0];
                    if ($signed(pu[47:32]) > 0) begin ov=1'b1; r = 32'hFFFF_FFFF; end
                    else r = pu[31:0];
                end
                `MUL_MPYHU: begin
                    pu = s1 * s2[31:16];
                    if ($signed(pu[47:32]) > 0) begin ov=1'b1; r = 32'hFFFF_FFFF; end
                    else r = pu[31:0];
                end
                `MUL_MPYHS: begin
                    ps = $signed(s1) * $signed(s2[31:16]);
                    if ($signed(ps[47:32]) > 0) begin ov=1'b1; r = 32'hFFFF_FFFF; end
                    else r = ps[15:0] << 16;
                end
                `OP_NOP : begin r=32'd0; ov=1'b0; end
                default : rv = 1'b0;
            endcase
        end
    endtask

    reg [31:0] xr; reg xo, xv;
    reg [1:0] mag;
    initial begin
        optab[0]=`MUL_MPYLL;  optab[1]=`MUL_MPYLLU; optab[2]=`MUL_MPYLH; optab[3]=`MUL_MPYLHU;
        optab[4]=`MUL_MPYHH;  optab[5]=`MUL_MPYHHU; optab[6]=`MUL_MPYL;  optab[7]=`MUL_MPYH;
        optab[8]=`MUL_MPYLU;  optab[9]=`MUL_MPYHU;  optab[10]=`MUL_MPYHS;
        optab[11]=`OP_NOP;    optab[12]=`OP_STOP;   // NOP + invalid

        seed = 32'h0BADC0DE;
        $display("========= r-VEX MUL constrained-random (%0d vectors) =========", N);

        for (it=0; it<N; it=it+1) begin
            mop = optab[{$random(seed)} % NOPS];
            ms1 = $random(seed);
            ms2 = $random(seed);
            // half the time squeeze operands small so non-saturating paths hit often
            mag = $random(seed);
            if (mag[0]) ms1 = ms1 & 32'h0000_00FF;
            if (mag[1]) ms2 = ms2 & 32'h00FF_00FF;
            #1;
            mul_ref(mop, ms1, ms2, xr, xo, xv);
            checks = checks + 3;
            if (mval !== xv) begin fail=fail+1;
                $display("  FAIL[%0d] op=%b valid: dut=%b ref=%b (s1=%h s2=%h)", it, mop, mval, xv, ms1, ms2); end
            if (xv) begin
                if (mres !== xr) begin fail=fail+1;
                    $display("  FAIL[%0d] op=%b result: dut=%h ref=%h (s1=%h s2=%h)", it, mop, mres, xr, ms1, ms2); end
                if (mov !== xo) begin fail=fail+1;
                    $display("  FAIL[%0d] op=%b ovf: dut=%b ref=%b (s1=%h s2=%h)", it, mop, mov, xo, ms1, ms2); end
            end
        end

        $display("==============================================================");
        $display("MUL RANDOM: %0d checks, %0d failed", checks, fail);
        if (fail==0) $display("RESULT: MUL RANDOM PASSED");
        else         $display("RESULT: %0d MUL RANDOM FAILURES", fail);
        $finish;
    end
endmodule
