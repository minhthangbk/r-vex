//======================================================================
// r-VEX | SIMD16 4-lane MAC unit (round r30 B2, reviews/contracts/simd4.md SQ-01..06), one per MUL slot (4..7)
//   opcode SQ_SIMD4 (0x0F), sub-op s = syl[4:2]; A pair (a0, a1) = GR[srcA], GR[srcA + 1], B pair (b0, b1) likewise
//   per word the simd16.md formulas (hi / lo = signed halves), the two word results added, every result mod 2^32:
//   s 0 dot4  = dot2(a0, b0) + dot2(a1, b1)     s 1 dotn4  = dotn2(a0, b0) + dotn2(a1, b1)
//   s 2 xdot4 = xdot2(a0, b0) + xdot2(a1, b1)   s 3 xdotn4 = xdotn2(a0, b0) + xdotn2(a1, b1)
//   s 4..7: the same plus the accumulator acc (GR[dst] of the packet, read before the packet's writes)
//   the core decides validity (slot 4..7, imt 00, even pair fields); this unit only computes
//======================================================================
`ifndef RVEX_SIMD4_V
`define RVEX_SIMD4_V
module simd4 (
    input  wire [2:0]  sub,
    input  wire [31:0] a0, a1, b0, b1, acc,
    output wire [31:0] result
);
    function [31:0] lane; input [1:0] k; input [31:0] a, b; reg signed [15:0] ah, al, bh, bl; begin
        ah = a[31:16]; al = a[15:0]; bh = b[31:16]; bl = b[15:0];
        case (k)
            2'd0: lane = ah * bh + al * bl;      // dot2
            2'd1: lane = ah * bh - al * bl;      // dotn2
            2'd2: lane = ah * bl + al * bh;      // xdot2
            default: lane = al * bh - ah * bl;   // xdotn2
        endcase
    end endfunction
    assign result = lane(sub[1:0], a0, b0) + lane(sub[1:0], a1, b1) + (sub[2] ? acc : 32'd0);
endmodule
`endif
