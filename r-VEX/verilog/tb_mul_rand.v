//======================================================================
// r-VEX | Constrained-random multiplier testbench (self-checking)
//----------------------------------------------------------------------
// Drives N random (opcode, src1, src2) vectors into mul.v and checks
// result / overflow / out_valid against an independent behavioural
// reference (mul_ref) encoding the VEX ISA semantics (reviews/contracts/mul32.md): the 32x16 ops
// truncate mod 2^32 and never set overflow (r10, BUG-076; they saturated before). It also checks the
// identity a*b mod 2^32 == mpylu(a,b) + mpyhs(a,b) on the DUT (MU-07) for every vector.
//
// Random operands are biased toward small magnitudes half the time so
// both small and wrapping products get frequent coverage.
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
        reg [63:0] p64;
        begin
            r=32'd0; ov=1'b0; rv=1'b1; p64=64'd0;
            case (op)
                `MUL_MPYLL : r = $signed(s1[15:0])  * $signed(s2[15:0]);
                `MUL_MPYLLU: r = s1[15:0]           * s2[15:0];
                `MUL_MPYLH : r = $signed(s1[15:0])  * $signed(s2[31:16]);
                `MUL_MPYLHU: r = s1[15:0]           * s2[31:16];
                `MUL_MPYHH : r = $signed(s1[31:16]) * $signed(s2[31:16]);
                `MUL_MPYHHU: r = s1[31:16]          * s2[31:16];
                // 32x16 ops truncate mod 2^32 (VEX ISA, reviews/contracts/mul32.md MU-01..06):
                // the reference forms the full 64-bit product and keeps the low word
                `MUL_MPYL : begin p64 = {{32{s1[31]}}, s1} * {{48{s2[15]}}, s2[15:0]};  r = p64[31:0]; end
                `MUL_MPYLU: begin p64 = {32'd0, s1} * {48'd0, s2[15:0]};                r = p64[31:0]; end
                `MUL_MPYH : begin p64 = {{32{s1[31]}}, s1} * {{48{s2[31]}}, s2[31:16]}; r = p64[31:0]; end
                `MUL_MPYHU: begin p64 = {32'd0, s1} * {48'd0, s2[31:16]};               r = p64[31:0]; end
                `MUL_MPYHS: begin p64 = {{32{s1[31]}}, s1} * {{48{s2[31]}}, s2[31:16]}; r = {p64[15:0], 16'd0}; end
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

        // MU-07: a*b == mpylu(a,b) + mpyhs(a,b) mod 2^32, on the DUT itself
        for (it=0; it<N; it=it+1) begin
            ms1 = $random(seed); ms2 = $random(seed);
            if (it % 4 == 1) ms1 = 32'h8000_0000; if (it % 4 == 2) ms2 = 32'hFFFF_FFFF;
            mop = `MUL_MPYLU; #1; xr = mres;
            mop = `MUL_MPYHS; #1; xr = xr + mres;
            checks = checks + 1;
            if (xr !== ms1 * ms2) begin fail=fail+1;
                $display("  FAIL[id %0d] mpylu+mpyhs=%h a*b=%h (a=%h b=%h)", it, xr, ms1 * ms2, ms1, ms2); end
        end

        $display("==============================================================");
        $display("MUL RANDOM: %0d checks, %0d failed", checks, fail);
        if (fail==0) $display("RESULT: MUL RANDOM PASSED");
        else         $display("RESULT: %0d MUL RANDOM FAILURES", fail);
        $finish;
    end
endmodule
