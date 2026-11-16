//======================================================================
// r-VEX | binary32 floating-point unit, slots 1 and 2 (round r27)
//----------------------------------------------------------------------
// Contract: reviews/contracts/fpu.md (FP-01..). PIPE = 0: combinational.
// PIPE = 1 (the core, r27 user decision after the fmax A/B): one register
// between stage 1 (unpack, swap, align, add / 24x24 product, special cases)
// and stage 2 (normalise, round, pack); the register loads in the cycle
// the core holds a packet with an FP op (ld, FP-12), so the result is
// still visible in the next packet (compiler latency 1).
//   FADD rd = a + b, FSUB rd = a - b, FMUL rd = a * b on IEEE-754 binary32
//   patterns held in GRs. One rounding, to nearest, ties to even; gradual
//   underflow (subnormal inputs and results, no flush to zero); overflow
//   gives infinity; an invalid operation (inf - inf, 0 * inf) or a NaN
//   input gives the canonical quiet NaN 0x7FC00000; an exact zero sum is
//   +0 unless both addends are -0. No exception flags.
// Reference model: toolchain/sim/f32ref.py (tb_fpu.v runs its vectors).
//======================================================================
`include "rvex_defs.vh"
`ifndef RVEX_FPU_V
`define RVEX_FPU_V
module fpu #(
    parameter PIPE = 0
) (
    input  wire        clk,
    input  wire        ld,         // PIPE = 1: load the stage register (stage-1 cycle)
    input  wire [6:0]  fpuop,
    input  wire [31:0] src1,
    input  wire [31:0] src2,
    output reg  [31:0] result,
    output wire        out_valid
);
    localparam [31:0] QNAN = 32'h7FC00000;
    assign out_valid = (fpuop == `FPU_FADD) || (fpuop == `FPU_FSUB) || (fpuop == `FPU_FMUL);

    // ---- unpack ----
    wire        sa = src1[31];
    wire        sb = src2[31] ^ (fpuop == `FPU_FSUB);
    wire [7:0]  ea = src1[30:23], eb = src2[30:23];
    wire [22:0] fa = src1[22:0],  fb = src2[22:0];
    wire        nan_a = &ea && |fa, nan_b = &eb && |fb;
    wire        inf_a = &ea && !fa, inf_b = &eb && !fb;
    wire        zer_a = !ea && !fa, zer_b = !eb && !fb;
    wire [23:0] ma = {|ea, fa}, mb = {|eb, fb};          // hidden bit 0 for a subnormal
    wire [7:0]  xa = |ea ? ea : 8'd1, xb = |eb ? eb : 8'd1;   // a subnormal has exponent 1

    // count of leading zeros of a 48-bit value (48 if zero)
    function [5:0] lzc48; input [47:0] v; integer k; reg f; begin
        lzc48 = 6'd48; f = 1'b0;
        for (k = 47; k >= 0; k = k - 1)
            if (!f && v[k]) begin lzc48 = 6'd47 - k[5:0]; f = 1'b1; end
    end endfunction

    // =================== adder ===================
    // |A| >= |B| after the swap (the exponent/fraction fields compare as integers)
    wire        swp = {ea, fa} < {eb, fb};
    wire        sA = swp ? sb : sa, sB = swp ? sa : sb;
    wire [7:0]  xA = swp ? xb : xa, xB = swp ? xa : xb;
    wire [23:0] mA = swp ? mb : ma, mB = swp ? ma : mb;
    wire [7:0]  d  = xA - xB;
    // mB << 3 (guard, round, sticky positions) shifted right by d; bits that fall off go into the sticky bit
    // (d >= 26 puts every bit of mB below the round bit: shifting by 26 already does that, so d is clamped there)
    wire [49:0] bx = {mB, 26'd0} >> (d > 8'd26 ? 8'd26 : d);
    wire        bst = |bx[23:0];
    wire [26:0] Bal = {bx[49:24], bst};
    wire [26:0] Aal = {mA, 3'b000};
    wire        effsub = sA ^ sB;
    wire [27:0] S1 = effsub ? {1'b0, Aal} - {1'b0, Bal} : {1'b0, Aal} + {1'b0, Bal};
    // special results of the adder, decided in stage 1
    wire        add_sp1 = nan_a || nan_b || inf_a || inf_b;
    wire [31:0] add_sv1 = (nan_a || nan_b) ? QNAN :
                          (inf_a && inf_b) ? ((sa == sb) ? {sa, 8'hFF, 23'd0} : QNAN) :
                          inf_a ? {sa, 8'hFF, 23'd0} : {sb, 8'hFF, 23'd0};
    // multiplier stage 1: the 24 x 24 product and the special results
    wire        sm1 = src1[31] ^ src2[31];
    wire [47:0] P1  = ma * mb;
    wire        mul_sp1 = nan_a || nan_b || inf_a || inf_b || zer_a || zer_b;
    wire [31:0] mul_sv1 = (nan_a || nan_b) ? QNAN :
                          ((inf_a && zer_b) || (zer_a && inf_b)) ? QNAN :
                          (inf_a || inf_b) ? {sm1, 8'hFF, 23'd0} : {sm1, 31'd0};
    wire [1:0]  sel1 = ((fpuop == `FPU_FADD) || (fpuop == `FPU_FSUB)) ? 2'd1 : (fpuop == `FPU_FMUL) ? 2'd2 : 2'd0;

    // =================== stage boundary (PIPE = 1: one register, loaded when ld) ===================
    reg  [27:0] S_q;  reg [7:0] xA_q, xa_q, xb_q; reg sA_q, sB_q, sm_q, add_sp_q, mul_sp_q;
    reg  [31:0] add_sv_q, mul_sv_q; reg [47:0] P_q; reg [1:0] sel_q;
    always @(posedge clk) if (ld) begin
        S_q <= S1; xA_q <= xA; sA_q <= sA; sB_q <= sB; add_sp_q <= add_sp1; add_sv_q <= add_sv1;
        P_q <= P1; xa_q <= xa; xb_q <= xb; sm_q <= sm1; mul_sp_q <= mul_sp1; mul_sv_q <= mul_sv1; sel_q <= sel1;
    end
    wire [27:0] S    = PIPE ? S_q    : S1;
    wire [7:0]  xA2  = PIPE ? xA_q   : xA;
    wire        sA2  = PIPE ? sA_q   : sA;
    wire        sB2  = PIPE ? sB_q   : sB;
    wire        add_sp = PIPE ? add_sp_q : add_sp1;
    wire [31:0] add_sv = PIPE ? add_sv_q : add_sv1;
    wire [47:0] P    = PIPE ? P_q    : P1;
    wire [7:0]  xa2  = PIPE ? xa_q   : xa;
    wire [7:0]  xb2  = PIPE ? xb_q   : xb;
    wire        sm   = PIPE ? sm_q   : sm1;
    wire        mul_sp = PIPE ? mul_sp_q : mul_sp1;
    wire [31:0] mul_sv = PIPE ? mul_sv_q : mul_sv1;
    wire [1:0]  sel  = PIPE ? sel_q  : sel1;

    // =================== adder stage 2 ===================
    // normalise: a carry shifts right by one; otherwise shift left by the leading zeros, but never below exponent 1
    wire [5:0]  lz  = lzc48({S[26:0], 21'd0});            // leading zeros of S[26:0] (27 bits)
    wire [7:0]  lim = xA2 - 8'd1;                          // the most the exponent may drop
    wire [5:0]  sh  = ({2'b0, lz} > lim) ? lim[5:0] : lz;
    wire [26:0] Sr  = {S[27:2], S[1] | S[0]};             // carry case, sticky kept
    wire [26:0] Sl  = S[26:0] << sh;
    wire [26:0] N   = S[27] ? Sr : Sl;
    wire [8:0]  En  = S[27] ? {1'b0, xA2} + 9'd1 : {1'b0, xA2} - {3'b0, sh};
    // N[26:3] = mantissa with the hidden bit at 26 (0 when the result is subnormal), N[2] guard, N[1:0] round/sticky
    wire        rup = N[2] && (N[1] || N[0] || N[3]);
    wire [24:0] Mr  = {1'b0, N[26:3]} + {24'd0, rup};
    wire [8:0]  Ef  = Mr[24] ? En + 9'd1 : (En == 9'd1 && !Mr[23]) ? 9'd0 : En;
    wire [22:0] Ff  = Mr[24] ? Mr[23:1] : Mr[22:0];
    wire        add_zero = (S == 28'd0);
    wire [31:0] add_r = add_zero ? {sA2 & sB2, 31'd0} :
                        (Ef >= 9'd255) ? {sA2, 8'hFF, 23'd0} : {sA2, Ef[7:0], Ff};
    wire [31:0] add_res = add_sp ? add_sv : add_r;

    // =================== multiplier stage 2 ===================
    wire [5:0]  plz = lzc48(P);
    wire [47:0] Pn = P << plz;                             // msb at bit 47 (P != 0 here)
    // biased exponent of 1.Pn[46:24] x 2^E: E = xa + xb - 127 + 1 - plz
    wire signed [10:0] Em = $signed({3'b0, xa2}) + $signed({3'b0, xb2}) - 11'sd126 - $signed({5'b0, plz});
    // below the normal range: shift right by 1 - Em more (sticky), exponent field 0
    wire [10:0] rsh = (Em < 11'sd1) ? (11'sd1 - Em) : 11'd0;
    wire [71:0] Px  = {Pn, 24'd0} >> (rsh > 11'd48 ? 11'd48 : rsh);
    wire        pst = |Px[46:0] || (rsh > 11'd48);
    wire [23:0] Pm  = Px[71:48];                           // mantissa with the hidden bit at 23
    wire        pg  = Px[47];
    wire        prup = pg && (pst || Pm[0]);
    wire [24:0] Pr  = {1'b0, Pm} + {24'd0, prup};
    wire [10:0] Eb  = (Em < 11'sd1) ? 11'd0 : Em;
    wire [10:0] Ep  = Pr[24] ? Eb + 11'd1 : (Eb == 11'd0 && Pr[23]) ? 11'd1 : Eb;
    wire [22:0] Fp  = Pr[24] ? Pr[23:1] : Pr[22:0];
    wire [31:0] mul_r = ($signed(Ep) >= 11'sd255) ? {sm, 8'hFF, 23'd0} : {sm, Ep[7:0], Fp};
    wire [31:0] mul_res = mul_sp ? mul_sv : mul_r;

    always @(*) begin
        case (sel)
            2'd1:    result = add_res;
            2'd2:    result = mul_res;
            default: result = 32'd0;
        endcase
    end
endmodule
`endif
