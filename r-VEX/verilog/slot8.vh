// r29 (issue8.md IS8-01): helper for the 4-slot testbenches written before the 8-issue core. pk8 places an old
// 4-slot packet {s3, s2, s1, s0} on the 8-slot core: a multiply-class syllable (MUL / FPU opcodes 000xxxx other than
// NOP, SIMD16 0x69..0x6F, 0x40) of the old MUL slots 1 and 2 moves to slot 4 and 5 (the FPU slots, also MUL slots);
// every other syllable keeps its slot, slots 6-7 stay NOP. The ops and their results are the same; only their slot
// follows the new map (tb_issue8.v tests that a multiply-class op in slots 0-3 writes nothing).
`ifndef RVEX_SLOT8_VH
`define RVEX_SLOT8_VH
function automatic pk8_mulcls; input [31:0] s; reg [6:0] o; begin
    o = s[31:25];
    pk8_mulcls = (o[6:4] == 3'b000 && o != 7'd0) || (o >= 7'h69 && o <= 7'h6F) || (o == 7'h40);
end endfunction
function automatic [255:0] pk8; input [31:0] s3, s2, s1, s0; reg [31:0] t1, t2, t4, t5; begin
    t1 = pk8_mulcls(s1) ? 32'd0 : s1;  t4 = pk8_mulcls(s1) ? s1 : 32'd0;
    t2 = pk8_mulcls(s2) ? 32'd0 : s2;  t5 = pk8_mulcls(s2) ? s2 : 32'd0;
    pk8 = {32'd0, 32'd0, t5, t4, s3, t2, t1, s0};
end endfunction
`endif
