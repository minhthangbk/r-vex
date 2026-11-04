//======================================================================
// r-VEX | Multiplier (Verilog port of mul.vhd + mul_operations.vhd)
//----------------------------------------------------------------------
// 2026-11-04 r10: the 32x16 ops truncate (VEX ISA, contract mul32.md);
// the VHDL saturated them. BUG FIXES vs the VHDL reference (see review):
//   * MPYHHU : uses UNSIGNED high*high (VHDL f_MPYHHU wrongly used signed)
//   * MPYHU  : wired to the UNSIGNED 32x16 multiply
//              (mul.vhd wrongly dispatched it to f_MPYH / signed)
//   * MPYHS  : wired to the shift-left-16 variant
//              (mul.vhd wrongly dispatched it to f_MPYH)
// No op saturates; `overflow` is always 0 (MU-06).
//======================================================================
`include "rvex_defs.vh"

module mul (
    input  wire [6:0]  mulop,
    input  wire [31:0] src1,
    input  wire [31:0] src2,
    output reg  [31:0] result,
    output reg         overflow,
    output reg         out_valid
);

    always @(*) begin
        result    = 32'd0;
        overflow  = 1'b0;
        out_valid = 1'b1;

        case (mulop)
            // ---- 16x16 -> 32, no saturation ----
            `MUL_MPYLL : result = $signed(src1[15:0])  * $signed(src2[15:0]);
            `MUL_MPYLLU: result = src1[15:0]           * src2[15:0];
            `MUL_MPYLH : result = $signed(src1[15:0])  * $signed(src2[31:16]);
            `MUL_MPYLHU: result = src1[15:0]           * src2[31:16];
            `MUL_MPYHH : result = $signed(src1[31:16]) * $signed(src2[31:16]);
            `MUL_MPYHHU: result = src1[31:16]          * src2[31:16];   // FIX: unsigned

            // ---- 32x16, truncating mod 2^32 (VEX ISA; reviews/contracts/mul32.md MU-01..06) ----
            // r10 (BUG-076): the r-VEX VHDL saturated these when product bits [47:32] > 0;
            // the VEX manual defines them as 32-bit C products, so a*b = mpylu(a,b) + mpyhs(a,b).
            `MUL_MPYL : result = src1 * {{16{src2[15]}}, src2[15:0]};    // s1 * INT16(s2)
            `MUL_MPYLU: result = src1 * {16'd0, src2[15:0]};             // s1 * UINT16(s2)
            `MUL_MPYH : result = src1 * {{16{src2[31]}}, src2[31:16]};   // s1 * INT16(s2 >> 16)
            `MUL_MPYHU: result = src1 * {16'd0, src2[31:16]};            // s1 * UINT16(s2 >> 16)
            `MUL_MPYHS: result = {src1[15:0] * src2[31:16], 16'd0};      // (s1 * INT16(s2 >> 16)) << 16

            `OP_NOP: begin result = 32'd0; overflow = 1'b0; end
            default: out_valid = 1'b0;
        endcase
    end
endmodule
