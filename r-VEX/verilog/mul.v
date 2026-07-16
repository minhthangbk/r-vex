//======================================================================
// r-VEX | Multiplier (Verilog port of mul.vhd + mul_operations.vhd)
//----------------------------------------------------------------------
// BUG FIXES vs the VHDL reference (see review):
//   * MPYHHU : uses UNSIGNED high*high (VHDL f_MPYHHU wrongly used signed)
//   * MPYHU  : wired to the UNSIGNED 48-bit saturating multiply
//              (mul.vhd wrongly dispatched it to f_MPYH / signed)
//   * MPYHS  : wired to the shift-left-16 variant
//              (mul.vhd wrongly dispatched it to f_MPYH)
// Saturation/overflow logic mirrors the VHDL: for the 48-bit variants the
// upper 16 product bits are tested ( $signed(tmp[47:32]) > 0 ).
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
    reg signed [47:0] ps;   // signed 48-bit product
    reg        [47:0] pu;   // unsigned 48-bit product

    always @(*) begin
        result    = 32'd0;
        overflow  = 1'b0;
        out_valid = 1'b1;
        ps = 48'sd0;
        pu = 48'd0;

        case (mulop)
            // ---- 16x16 -> 32, no saturation ----
            `MUL_MPYLL : result = $signed(src1[15:0])  * $signed(src2[15:0]);
            `MUL_MPYLLU: result = src1[15:0]           * src2[15:0];
            `MUL_MPYLH : result = $signed(src1[15:0])  * $signed(src2[31:16]);
            `MUL_MPYLHU: result = src1[15:0]           * src2[31:16];
            `MUL_MPYHH : result = $signed(src1[31:16]) * $signed(src2[31:16]);
            `MUL_MPYHHU: result = src1[31:16]          * src2[31:16];   // FIX: unsigned

            // ---- 32x16 -> 48, signed saturating ----
            `MUL_MPYL : begin
                ps = $signed(src1) * $signed(src2[15:0]);
                if ($signed(ps[47:32]) > 0) begin
                    overflow = 1'b1;
                    result   = ps[47] ? 32'h8000_0000 : 32'h7FFF_FFFF;
                end else result = ps[31:0];
            end
            `MUL_MPYH : begin
                ps = $signed(src1) * $signed(src2[31:16]);
                if ($signed(ps[47:32]) > 0) begin
                    overflow = 1'b1;
                    result   = ps[47] ? 32'h8000_0000 : 32'h7FFF_FFFF;
                end else result = ps[31:0];
            end

            // ---- 32x16 -> 48, unsigned saturating ----
            `MUL_MPYLU: begin
                pu = src1 * src2[15:0];
                if ($signed(pu[47:32]) > 0) begin
                    overflow = 1'b1;
                    result   = 32'hFFFF_FFFF;
                end else result = pu[31:0];
            end
            `MUL_MPYHU: begin                                  // FIX: unsigned + right wiring
                pu = src1 * src2[31:16];
                if ($signed(pu[47:32]) > 0) begin
                    overflow = 1'b1;
                    result   = 32'hFFFF_FFFF;
                end else result = pu[31:0];
            end

            // ---- 32x16 signed, shift left 16 (FIX: right wiring) ----
            `MUL_MPYHS: begin
                ps = $signed(src1) * $signed(src2[31:16]);
                if ($signed(ps[47:32]) > 0) begin
                    overflow = 1'b1;
                    result   = 32'hFFFF_FFFF;
                end else begin
                    result = ps[15:0] << 16;   // (product << 16)[31:0], per f_MPYHS
                end
            end

            `OP_NOP: begin result = 32'd0; overflow = 1'b0; end
            default: out_valid = 1'b0;
        endcase
    end
endmodule
