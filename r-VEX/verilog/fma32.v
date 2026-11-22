//======================================================================
// r-VEX | binary32 fused multiply-add lane (round r32, fpu.md FP-18 / FP-20)
//----------------------------------------------------------------------
// r = round32((-1)^np * a * b + (-1)^nc * c) with ONE rounding (RNE), IEEE 754-2008 fusedMultiplyAdd, the special
// cases of f32ref.fma: any NaN input or inf * 0 gives the canonical QNAN 0x7FC00000; an infinite product plus an
// infinite c of the other sign gives QNAN; an exact zero sum is +0 unless the product and c are both zeros of sign 1;
// gradual underflow, overflow to infinity, no flags.
// Datapath (reference form, per-cycle work: synthesis / area are out of scope by the user rule "per-cycle first",
// CTX-20261122-02): the product and c are placed exactly on one integer grid of weight 2^-300 (a binary32 value is
// m * 2^(x - 150), x >= 1, so the product is P * 2^(xa + xb - 300) and c is mc * 2^(xc - 150)); their signed sum is exact
// in 562 bits, then it is normalised and rounded once.
// The pair forms (FP-20) use the identities fadd = fma(a, 1.0, b), fsub = fma(a, 1.0, -b), fmul = fma(a, b, -0)
// (f32ref selftest row "lane identities").
//======================================================================
`ifndef RVEX_FMA32_V
`define RVEX_FMA32_V
module fma32 (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [31:0] c,
    input  wire        np,         // negate the product
    input  wire        nc,         // negate c
    output reg  [31:0] r
);
    localparam [31:0] QNAN = 32'h7FC00000;
    localparam integer W = 562;
    wire        sa = a[31], sb = b[31];
    wire        sc = c[31] ^ nc;
    wire        sp = sa ^ sb ^ np;
    wire [7:0]  ea = a[30:23], eb = b[30:23], ec = c[30:23];
    wire [22:0] fa = a[22:0], fb = b[22:0], fc = c[22:0];
    wire        nan_a = &ea && |fa, nan_b = &eb && |fb, nan_c = &ec && |fc;
    wire        inf_a = &ea && !fa, inf_b = &eb && !fb, inf_c = &ec && !fc;
    wire        zer_a = !ea && !fa, zer_b = !eb && !fb, zer_c = !ec && !fc;
    wire [23:0] ma = {|ea, fa}, mb = {|eb, fb}, mc = {|ec, fc};
    wire [8:0]  xa = |ea ? {1'b0, ea} : 9'd1, xb = |eb ? {1'b0, eb} : 9'd1, xc = |ec ? {1'b0, ec} : 9'd1;
    wire [47:0] P  = ma * mb;

    reg [W-1:0] Pg, Cg, M;
    reg         s, rup;
    integer     m, e, q, k;
    reg [W-1:0] lo;
    reg         g, st;
    always @(*) begin
        Pg = {{(W-48){1'b0}}, P} << (xa + xb);
        Cg = {{(W-24){1'b0}}, mc} << (xc + 9'd150);
        if (sp == sc) begin M = Pg + Cg; s = sp; end
        else if (Pg >= Cg) begin M = Pg - Cg; s = sp; end
        else begin M = Cg - Pg; s = sc; end
        // leading one
        m = -1;
        for (k = 0; k < W; k = k + 1) if (M[k]) m = k;
        e = m - 300;                                    // M = 1.xxx * 2^e
        q = (e < -126) ? 151 : (m - 23);                // the quantum position (2^-149 below the normal range)
        lo = M >> q;
        g = (q >= 1) ? M[q - 1] : 1'b0;
        st = 1'b0;
        for (k = 0; k < W; k = k + 1) if (k < q - 1 && M[k]) st = 1'b1;
        rup = g && (st || lo[0]);
        lo = lo + rup;
        if (e >= -126 && lo[24]) begin lo = lo >> 1; e = e + 1; end
        if (nan_a || nan_b || nan_c || (inf_a && zer_b) || (zer_a && inf_b))
            r = QNAN;
        else if (inf_a || inf_b)
            r = (inf_c && sc != sp) ? QNAN : {sp, 8'hFF, 23'd0};
        else if (inf_c)
            r = {sc, 8'hFF, 23'd0};
        else if (M == {W{1'b0}})
            r = {((zer_a || zer_b) && zer_c && sp && sc), 31'd0};
        else if (e < -126)
            r = {s, lo[30:0]};                           // subnormal (lo = 2^23 is the smallest normal)
        else if (e > 127)
            r = {s, 8'hFF, 23'd0};
        else
            r = {s, e[7:0] + 8'd127, lo[22:0]};
    end
endmodule
`endif
