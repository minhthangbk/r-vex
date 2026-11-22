//======================================================================
// r-VEX | FP family of opcode 0x0F with imm type 01 / 11 (round r32, fpu.md FP-18..FP-22)
//----------------------------------------------------------------------
// One per FPU slot (4-7), next to fpu.v. Stage 1 loads the operands when the packet with the op commits (ld, as FP-13);
// stage 2 computes from the registers, so the result is written at the next clock edge like the scalar FPU (latency
// 2 for the compiler, FP-14 interlock). Two fused lanes (fma32.v) compute:
//   imm type 01, sub-op syl[4:2]:
//     0 fmadd   rD        = rD + rA * rB             (fused, one rounding)
//     1 fmsub   rD        = rD - rA * rB
//     2 fmrsub  rD        = rA * rB - rD
//     3 fadd2   rD:rD+1   = rA:rA+1 + rB:rB+1        (lane-wise; pair fields even)
//     4 fsub2   rD:rD+1   = rA:rA+1 - rB:rB+1
//     5 fmul2   rD:rD+1   = rA:rA+1 * rB:rB+1
//     6 fmadd2  rD:rD+1  += rA:rA+1 * rB:rB+1        (fused per lane)
//     7 fmsub2  rD:rD+1  -= rA:rA+1 * rB:rB+1
//   imm type 11 (complex cross-lane forms, user decision 2026-11-22 "Thêm dạng complex cross-lane"):
//     4 fmul2x  rD:rD+1   = (rA * rB+1, rA+1 * rB)  (the cross products of a complex multiply)
//     5 fhsa2   rD:rD+1   = (rA - rA+1, rB + rB+1)  (re = ac - bd from fmul2, im = ad + bc from fmul2x)
//     6 faddsub2 rD:rD+1  = (rA - rB, rA+1 + rB+1)
//     0..3, 7  undefined (0..3 reserved for the Q15 complex family of r33): the core writes nothing
// The pair-form lane identities: x + y = fma(x, 1.0, y), x - y = fma(x, 1.0, -y), x * y = fma(x, y, -0) (FP-20).
//======================================================================
`ifndef RVEX_FPX_V
`define RVEX_FPX_V
module fpx (
    input  wire        clk,
    input  wire        ld,
    input  wire [1:0]  imt,
    input  wire [2:0]  sub,
    input  wire [31:0] a0, a1, b0, b1, d0, d1,
    output wire [31:0] lo,
    output wire [31:0] hi
);
    localparam [31:0] ONE = 32'h3F800000, NZ = 32'h80000000;
    reg [1:0] imt_q; reg [2:0] sub_q; reg [31:0] a0_q, a1_q, b0_q, b1_q, d0_q, d1_q;
    always @(posedge clk) if (ld) begin
        imt_q <= imt; sub_q <= sub; a0_q <= a0; a1_q <= a1; b0_q <= b0; b1_q <= b1; d0_q <= d0; d1_q <= d1;
    end
    reg [31:0] x0, y0, z0, x1, y1, z1; reg np0, nc0, np1, nc1;
    always @(*) begin
        x0 = a0_q; y0 = b0_q; z0 = d0_q; np0 = 1'b0; nc0 = 1'b0;
        x1 = a1_q; y1 = b1_q; z1 = d1_q; np1 = 1'b0; nc1 = 1'b0;
        if (imt_q == 2'b01) begin
            case (sub_q)
                3'd0: ;                                                         // fmadd
                3'd1: np0 = 1'b1;                                               // fmsub
                3'd2: nc0 = 1'b1;                                               // fmrsub
                3'd3: begin y0 = ONE; z0 = b0_q; y1 = ONE; z1 = b1_q; end      // fadd2
                3'd4: begin y0 = ONE; z0 = b0_q; nc0 = 1'b1; y1 = ONE; z1 = b1_q; nc1 = 1'b1; end   // fsub2
                3'd5: begin z0 = NZ; z1 = NZ; end                               // fmul2
                3'd6: ;                                                         // fmadd2
                3'd7: begin np0 = 1'b1; np1 = 1'b1; end                        // fmsub2
            endcase
        end else begin
            case (sub_q)
                3'd4: begin y0 = b1_q; z0 = NZ; y1 = b0_q; z1 = NZ; end        // fmul2x
                3'd5: begin y0 = ONE; z0 = a1_q; nc0 = 1'b1; x1 = b0_q; y1 = ONE; z1 = b1_q; end   // fhsa2
                3'd6: begin y0 = ONE; z0 = b0_q; nc0 = 1'b1; y1 = ONE; z1 = b1_q; end   // faddsub2
                default: ;
            endcase
        end
    end
    fma32 u_l0(.a(x0), .b(y0), .c(z0), .np(np0), .nc(nc0), .r(lo));
    fma32 u_l1(.a(x1), .b(y1), .c(z1), .np(np1), .nc(nc1), .r(hi));
endmodule
`endif
