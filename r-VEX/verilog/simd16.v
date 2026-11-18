//======================================================================
// r-VEX | SIMD16 dual-MAC unit (round r28, reviews/contracts/simd16.md SM-01..07), one per MUL slot (1, 2)
//   hi(x) = signed x[31:16], lo(x) = signed x[15:0]; every result mod 2^32 (no saturation, no flags)
//   dot2   = hi(a)*hi(b) + lo(a)*lo(b)      dotn2   = hi(a)*hi(b) - lo(a)*lo(b)
//   xdot2  = hi(a)*lo(b) + lo(a)*hi(b)      xdotn2  = lo(a)*hi(b) - hi(a)*lo(b)
//   the *a forms add the accumulator acc (GR[dst] of the packet, read before the packet's writes)
//======================================================================
`ifndef RVEX_SIMD16_V
`define RVEX_SIMD16_V
module simd16 (
    input  wire [6:0]  op,
    input  wire [31:0] a, b, acc,
    output reg  [31:0] result,
    output reg         out_valid
);
    wire signed [15:0] ah = a[31:16], al = a[15:0], bh = b[31:16], bl = b[15:0];
    wire signed [31:0] hh = ah * bh, ll = al * bl, hl = ah * bl, lh = al * bh;
    always @(*) begin
        out_valid = 1'b1;
        case (op)
            `SM_DOT2:    result = hh + ll;
            `SM_DOTN2:   result = hh - ll;
            `SM_XDOT2:   result = hl + lh;
            `SM_XDOTN2:  result = lh - hl;
            `SM_DOT2A:   result = acc + hh + ll;
            `SM_DOTN2A:  result = acc + hh - ll;
            `SM_XDOT2A:  result = acc + hl + lh;
            `SM_XDOTN2A: result = acc + lh - hl;
            default: begin result = 32'd0; out_valid = 1'b0; end
        endcase
    end
endmodule
`endif
