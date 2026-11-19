//======================================================================
// r-VEX | Integrated execution core (equivalent Verilog port)
//----------------------------------------------------------------------
// SCOPE / DEVIATION (documented in the review):
//   The original VHDL uses a multi-FSM, clock-level handshake spread over
//   fetch/decode/execute/mem/writeback plus a ~700-line top. That handshake
//   is NOT cycle-reproduced here. Instead this is a clean, synchronous
//   "one VLIW packet per active issue cycle" core that preserves:
//     - the slot resource model; since r29 (issue8.md IS8-01) 8 slots: ALU x8, CTRL slot 0, second branch
//       slot 1, MEM slots 2-3 (mem2.md), MUL / SIMD16 slots 4-7, FPU slots 4-5
//     - the syllable field layout of decode.vhd
//     - VEX packet semantics (all source reads happen before writes)
//   This is sufficient to drive every fixed/new instruction end-to-end and
//   observe GR / BR / data-memory / PC / XNOP-stall effects.
//
//   GR file is 64 deep (6-bit addresses) so $r0.63-style link regs are legal;
//   the VHDL constant GR_DEPTH=32 was a latent limitation. GR[0] is hardwired 0.
//
// MEMORY SYSTEM (2026-10-31):
//   PC_W     : PC width. 12 = the whole absolute branch field syl[16:5], so
//              imem holds 2**PC_W packets (was 8 bits / 256 packets).
//   DMEM_AW  : byte-address width of the internal data memory (was 10).
//   EXT_DMEM : 0 = internal single-cycle dmem (default, used by every
//              existing testbench). 1 = slot-3 loads/stores go out on the
//              d_* port (to an L1 D-cache). The port is combinational: on
//              d_ready the access completes in this cycle; without it the
//              WHOLE packet is held (no GR/BR/PC/memory update), i.e. the
//              core stalls on a cache miss and the compiler-visible
//              schedule is unchanged.
//   EXT_IMEM : 1 = packets come from the i_* port (an L1 I-cache); a fetch
//              miss holds the core the same way.
//
// PERFORMANCE INDICATOR UNIT (2026-10-31 r5, reviews/contracts/piu.md):
//   PIU      : 1 = slot-3 accesses with addr[15:8] == 8'hFE go to this core's
//              rvex_piu (event counters) instead of dmem / the D port. They
//              complete in one cycle and never stall.
//======================================================================
`include "rvex_defs.vh"
`include "rvex_piu.v"
`include "fpu.v"
`include "simd16.v"

module rvex_core #(
    parameter PC_W     = 12,
    parameter DMEM_AW  = 16,
    parameter EXT_DMEM = 0,
    parameter EXT_IMEM = 0,
    parameter PIU      = 1,
    parameter DUAL_D   = 0,    // r28 (mem2.md MEM2-10): second data port d2_* to a dual-port L1D (hit only)
    parameter SIMD16   = 1,    // r28 (simd16.md): dual 16x16 MAC ops in the MUL slots (4-7 since r29); 0 = codes undefined (write nothing)
    parameter FPU      = 1     // r27 (fpu.md): binary32 FADD/FSUB/FMUL in slots 4-5 since r29; 0 = no FPU (codes 12..14 undefined: write nothing, BUG-320)
) (
    input  wire clk,
    input  wire reset,
    input  wire start,
    output reg  done,
    output reg [31:0] cycles,
    // external data port (EXT_DMEM=1)
    output wire        d_req,
    output wire        d_we,
    output wire [31:0] d_addr,     // byte address, word aligned
    output wire [31:0] d_wdata,    // whole merged word (sub-word stores)
    output wire        d_word,     // the access is ldw/stw (hw_sem accepts only these)
    input  wire [31:0] d_rdata,
    input  wire        d_ready,
    // second data port (DUAL_D=1, r28 MEM2-10): the second access of a packet, completes only with a port-1 hit
    output wire        d2_req,
    output wire        d2_we,
    output wire [31:0] d2_addr,
    output wire [31:0] d2_wdata,
    input  wire [31:0] d2_rdata,
    input  wire        d2_hit,
    output reg  [31:0] stall_cycles,     // cycles held by D-side misses
    // external instruction port (EXT_IMEM=1, to an L1 I-cache)
    output wire              i_req,
    output wire [PC_W-1:0]   i_addr,
    input  wire [255:0]      i_rdata,        // r29 (issue8.md IS8-02): 8 syllables, slot k in [32k+31:32k]
    input  wire              i_ready,
    output reg  [31:0]       istall_cycles  // cycles held by I-side misses
);
    // ---- state ----
    reg [255:0] imem [0:(1<<PC_W)-1];      // instruction memory (8 syllables per packet, IS8-02)
    reg [31:0]  gr   [0:63];       // general registers ($r0.0 = 0)
    reg         br   [0:7];        // branch registers
    reg [31:0]  dmem [0:(1<<(DMEM_AW-2))-1]; // data memory (word addressed)
    reg [PC_W-1:0] pc;
    reg [11:0]  stall_cnt;
    reg         running;
    integer     i;

    // ---- current packet & syllables ----
    wire [255:0] packet = EXT_IMEM ? i_rdata : imem[pc];
    wire [31:0]  syl0 = packet[31:0];
    wire [31:0]  syl1 = packet[63:32];
    wire [31:0]  syl2 = packet[95:64];
    wire [31:0]  syl3 = packet[127:96];
    wire [31:0]  syl4 = packet[159:128];     // r29 IS8-01: slots 4-7 = ALU + MUL / SIMD16 (FPU in 4-5)
    wire [31:0]  syl5 = packet[191:160];
    wire [31:0]  syl6 = packet[223:192];
    wire [31:0]  syl7 = packet[255:224];

    // ---- field extraction ----
    wire [6:0] op0=syl0[31:25], op1=syl1[31:25], op2=syl2[31:25], op3=syl3[31:25];
    wire [1:0] imt0=syl0[24:23], imt1=syl1[24:23], imt2=syl2[24:23], imt3=syl3[24:23];
    wire [5:0] dst0=syl0[22:17], dst1=syl1[22:17], dst2=syl2[22:17], dst3=syl3[22:17];
    wire [5:0] r1a0=syl0[16:11], r1a1=syl1[16:11], r1a2=syl2[16:11], r1a3=syl3[16:11];
    wire [5:0] r2a0=syl0[10:5],  r2a1=syl1[10:5];
    // r28 (mem2.md MEM2-02): a slot-2 store reads its data GR from the dst field, as slot 3
    wire       st2 = (op2 == `MEM_STW) || (op2 == `MEM_STH) || (op2 == `MEM_STB);
    wire [5:0] r2a2 = st2 ? syl2[22:17] : syl2[10:5];
    // Stores read the data GR from the dst field; every other slot-3 op
    // (ALU, load) reads src2 from [10:5] like the other slots.
    wire       st3 = (op3 == `MEM_STW) || (op3 == `MEM_STH) || (op3 == `MEM_STB);
    wire [5:0] r2a3 = st3 ? syl3[22:17] : syl3[10:5];
    wire [2:0] dbr0=syl0[4:2],   dbr1=syl1[4:2],   dbr2=syl2[4:2],   dbr3=syl3[4:2];
    wire [6:0] op4=syl4[31:25], op5=syl5[31:25], op6=syl6[31:25], op7=syl7[31:25];
    wire [1:0] imt4=syl4[24:23], imt5=syl5[24:23], imt6=syl6[24:23], imt7=syl7[24:23];
    wire [5:0] dst4=syl4[22:17], dst5=syl5[22:17], dst6=syl6[22:17], dst7=syl7[22:17];
    wire [5:0] r1a4=syl4[16:11], r1a5=syl5[16:11], r1a6=syl6[16:11], r1a7=syl7[16:11];
    wire [5:0] r2a4=syl4[10:5],  r2a5=syl5[10:5],  r2a6=syl6[10:5],  r2a7=syl7[10:5];
    wire [2:0] dbr4=syl4[4:2],   dbr5=syl5[4:2],   dbr6=syl6[4:2],   dbr7=syl7[4:2];
    wire [2:0] sbr0=syl0[27:25], sbr1=syl1[27:25], sbr2=syl2[27:25];
    wire [11:0] off0=syl0[16:5];

    // ---- helper: is a compare/logic op that may target BR (CMPEQ..ANDL) ----
    // CLZ (0x65) and MTB (0x67) sit inside the CMPEQ..ANDL opcode range but are not compares: CLZ writes
    // only a GR (clz.md), so `clz $r0.0 = x` must write nothing (BUG-181, pfuzz.md PZ-02, found by pfuzz r17:
    // it wrote BR[syl[4:2]] = clz(x)[0] and counted as a brn writer). MTB keeps its own writeback branch.
    function is_cmp; input [6:0] op; begin
        is_cmp = (op >= `ALU_CMPEQ) && (op <= `ALU_ANDL) && (op != `ALU_CLZ) && (op != `ALU_MTB); end
    endfunction
    function is_mul; input [6:0] op; begin
        is_mul = (op[6:4] == 3'b000) && (op != `OP_NOP); end
    endfunction
    function is_fpu; input [6:0] op; begin   // fpu.md FP-01: FADD / FSUB / FMUL (slots 1-2 only)
        is_fpu = FPU && ((op == `FPU_FADD) || (op == `FPU_FSUB) || (op == `FPU_FMUL)); end
    endfunction
    function is_simd; input [6:0] op; begin   // simd16.md SM-07: 0x69..0x6F, 0x40 (slots 1-2 only)
        is_simd = SIMD16 && (((op >= `SM_DOT2) && (op <= `SM_XDOT2A)) || (op == `SM_XDOTN2A)); end
    endfunction
    function is_addcg_divs; input [6:0] op; begin
        is_addcg_divs = (op[6:3]==`ALU_ADDCG4) || (op[6:3]==`ALU_DIVS4); end
    endfunction

    // ---- operand read (combinational) ----
    wire [31:0] gr_r1_0 = gr[r1a0], gr_r2_0 = gr[r2a0];
    wire [31:0] gr_r1_1 = gr[r1a1], gr_r2_1 = gr[r2a1];
    wire [31:0] gr_r1_2 = gr[r1a2], gr_r2_2 = gr[r2a2];
    wire [31:0] gr_r1_3 = gr[r1a3], gr_r2_3 = gr[r2a3];
    wire [31:0] gr_r1_4 = gr[r1a4], gr_r2_4 = gr[r2a4];
    wire [31:0] gr_r1_5 = gr[r1a5], gr_r2_5 = gr[r2a5];
    wire [31:0] gr_r1_6 = gr[r1a6], gr_r2_6 = gr[r2a6];
    wire [31:0] gr_r1_7 = gr[r1a7], gr_r2_7 = gr[r2a7];

    // BRIMM (reviews/contracts/cmpbr.md): a compare/logic op with imm type
    // 2'b11 writes BR syl[19:17] = GR syl[16:11] <op> zext(syl[10:2]) and no
    // GR. A BR compare leaves the GR dst field free, so the BR index moves
    // there and syl[10:2] holds the immediate as for SHORT_IMM.
    function is_brimm; input [6:0] op; input [1:0] imt; begin
        is_brimm = (imt == `LONG_IMM) &&
                   (((op >= `ALU_CMPEQ) && (op <= `ALU_NORL)) || op == `ALU_ORL || op == `ALU_ANDL); end
    endfunction
    wire bi0 = is_brimm(op0, imt0), bi1 = is_brimm(op1, imt1),
         bi2 = is_brimm(op2, imt2), bi3 = is_brimm(op3, imt3),
         bi4 = is_brimm(op4, imt4), bi5 = is_brimm(op5, imt5),
         bi6 = is_brimm(op6, imt6), bi7 = is_brimm(op7, imt7);

    function [31:0] op2sel; input [1:0] imt; input bi; input [31:0] regv; input [31:0] syl; begin
        case (imt)
            `SHORT_IMM : op2sel = {23'b0, syl[10:2]};
            `BRANCH_IMM: op2sel = {20'b0, syl[16:5]};
            `LONG_IMM  : op2sel = bi ? {23'b0, syl[10:2]} : regv;
            default    : op2sel = regv;
        endcase
    end endfunction

    wire [31:0] a2_0 = op2sel(imt0, bi0, gr_r2_0, syl0);
    wire [31:0] a2_1 = op2sel(imt1, bi1, gr_r2_1, syl1);
    wire [31:0] a2_2 = op2sel(imt2, bi2, gr_r2_2, syl2);
    wire [31:0] a2_3 = op2sel(imt3, bi3, gr_r2_3, syl3);
    wire [31:0] a2_4 = op2sel(imt4, bi4, gr_r2_4, syl4);
    wire [31:0] a2_5 = op2sel(imt5, bi5, gr_r2_5, syl5);
    wire [31:0] a2_6 = op2sel(imt6, bi6, gr_r2_6, syl6);
    wire [31:0] a2_7 = op2sel(imt7, bi7, gr_r2_7, syl7);

    // BR operand for ALU (SLCT/SLCTF/ADDCG/DIVS use rb=syl[27:25])
    wire cin0 = br[sbr0], cin1 = br[sbr1], cin2 = br[sbr2];
    wire cin3 = br[syl3[27:25]];
    wire cin4 = br[syl4[27:25]], cin5 = br[syl5[27:25]], cin6 = br[syl6[27:25]], cin7 = br[syl7[27:25]];

    // ---- functional units ----
    // r29 (issue8.md IS8-01): an ALU in every slot; MUL / SIMD16 in slots 4-7; FPU in slots 4-5
    wire [31:0] alu0_r,alu1_r,alu2_r,alu3_r,alu4_r,alu5_r,alu6_r,alu7_r;
    wire        alu0_c,alu1_c,alu2_c,alu3_c,alu4_c,alu5_c,alu6_c,alu7_c;
    wire        alu0_v,alu1_v,alu2_v,alu3_v,alu4_v,alu5_v,alu6_v,alu7_v;
    alu u_alu0(.aluop(op0),.src1(gr_r1_0),.src2(a2_0),.cin(cin0),.result(alu0_r),.cout(alu0_c),.out_valid(alu0_v));
    alu u_alu1(.aluop(op1),.src1(gr_r1_1),.src2(a2_1),.cin(cin1),.result(alu1_r),.cout(alu1_c),.out_valid(alu1_v));
    alu u_alu2(.aluop(op2),.src1(gr_r1_2),.src2(a2_2),.cin(cin2),.result(alu2_r),.cout(alu2_c),.out_valid(alu2_v));
    alu u_alu3(.aluop(op3),.src1(gr_r1_3),.src2(a2_3),.cin(cin3),.result(alu3_r),.cout(alu3_c),.out_valid(alu3_v));
    alu u_alu4(.aluop(op4),.src1(gr_r1_4),.src2(a2_4),.cin(cin4),.result(alu4_r),.cout(alu4_c),.out_valid(alu4_v));
    alu u_alu5(.aluop(op5),.src1(gr_r1_5),.src2(a2_5),.cin(cin5),.result(alu5_r),.cout(alu5_c),.out_valid(alu5_v));
    alu u_alu6(.aluop(op6),.src1(gr_r1_6),.src2(a2_6),.cin(cin6),.result(alu6_r),.cout(alu6_c),.out_valid(alu6_v));
    alu u_alu7(.aluop(op7),.src1(gr_r1_7),.src2(a2_7),.cin(cin7),.result(alu7_r),.cout(alu7_c),.out_valid(alu7_v));

    wire [31:0] mul4_r, mul5_r, mul6_r, mul7_r; wire mul4_v, mul5_v, mul6_v, mul7_v, mul4_o, mul5_o, mul6_o, mul7_o;
    mul u_mul4(.mulop(op4),.src1(gr_r1_4),.src2(a2_4),.result(mul4_r),.overflow(mul4_o),.out_valid(mul4_v));
    mul u_mul5(.mulop(op5),.src1(gr_r1_5),.src2(a2_5),.result(mul5_r),.overflow(mul5_o),.out_valid(mul5_v));
    mul u_mul6(.mulop(op6),.src1(gr_r1_6),.src2(a2_6),.result(mul6_r),.overflow(mul6_o),.out_valid(mul6_v));
    mul u_mul7(.mulop(op7),.src1(gr_r1_7),.src2(a2_7),.result(mul7_r),.overflow(mul7_o),.out_valid(mul7_v));
    // simd16.md SM-01..05 (r28): the accumulate forms read GR[dst] of the packet (before its writes)
    wire [31:0] sm4_r, sm5_r, sm6_r, sm7_r; wire sm4_v, sm5_v, sm6_v, sm7_v;
    simd16 u_sm4(.op(op4),.a(gr_r1_4),.b(a2_4),.acc(gr[dst4]),.result(sm4_r),.out_valid(sm4_v));
    simd16 u_sm5(.op(op5),.a(gr_r1_5),.b(a2_5),.acc(gr[dst5]),.result(sm5_r),.out_valid(sm5_v));
    simd16 u_sm6(.op(op6),.a(gr_r1_6),.b(a2_6),.acc(gr[dst6]),.result(sm6_r),.out_valid(sm6_v));
    simd16 u_sm7(.op(op7),.a(gr_r1_7),.b(a2_7),.acc(gr[dst7]),.result(sm7_r),.out_valid(sm7_v));
    wire [31:0] fpu4_r, fpu5_r; wire fpu4_v, fpu5_v;
    wire        fp_wait;                 // FP-12, below
    generate if (FPU) begin : g_fpu
        // FP-12 (r27): two stages; the stage register loads while the packet is held one cycle (fp_wait)
        fpu #(.PIPE(1)) u_fpu4(.clk(clk),.ld(fp_wait),.fpuop(op4),.src1(gr_r1_4),.src2(a2_4),.result(fpu4_r),.out_valid(fpu4_v));
        fpu #(.PIPE(1)) u_fpu5(.clk(clk),.ld(fp_wait),.fpuop(op5),.src1(gr_r1_5),.src2(a2_5),.result(fpu5_r),.out_valid(fpu5_v));
    end else begin : g_nofpu
        assign fpu4_r = 32'd0; assign fpu5_r = 32'd0; assign fpu4_v = 1'b0; assign fpu5_v = 1'b0;
    end endgenerate

    // ---- 64-bit loads (r29 B2, reviews/contracts/ldd.md): ldd in slot 2 / 3, encoding E2 ----
    // dst1 = syl[22:17], base = syl[16:11], dst2 = syl[10:5], offset = {syl[24:23], syl[4:2]} x 4 (LDD-01)
    function io_pg; input [31:0] a; begin   // LDD-04: the PIU window or the hw_sem page
        io_pg = (a[DMEM_AW-1:8] == 8'hFE) || (&a[DMEM_AW-1:8]); end endfunction
    wire        ldd3     = (op3 == `MEM_LDD);
    wire        ldd2     = (op2 == `MEM_LDD);
    wire [31:0] ldd_ea3  = gr_r1_3 + {25'b0, syl3[24:23], syl3[4:2], 2'b00};
    wire [31:0] ldd_ea2  = gr_r1_2 + {25'b0, syl2[24:23], syl2[4:2], 2'b00};
    wire        ldd_io3  = io_pg(ldd_ea3) || io_pg(ldd_ea3 + 32'd4);
    wire        ldd_io2  = io_pg(ldd_ea2) || io_pg(ldd_ea2 + 32'd4);
    wire        ldd_mis3 = (ldd_ea3[1:0] != 2'b00);               // LDD-03: both destinations 0
    wire        ldd_mis2 = (ldd_ea2[1:0] != 2'b00);
    wire        ldd_ok3  = ldd3 && !ldd_io3 && !ldd_mis3;         // LDD-02: two word reads
    wire        ldd_ok2  = ldd2 && !ldd_io2 && !ldd_mis2;
    wire [DMEM_AW-3:0] ldd_wa3 = ldd_ea3[DMEM_AW-1:2];
    wire [DMEM_AW-3:0] ldd_wa2 = ldd_ea2[DMEM_AW-1:2];

    // ---- memory unit (slot 3) ----
    wire [31:0] mem_ea        = gr_r1_3 + {23'b0, syl3[10:2]};
    wire [DMEM_AW-1:0] dmem_byte_addr = mem_ea[DMEM_AW-1:0];
    wire [DMEM_AW-3:0] dmem_word_addr = dmem_byte_addr[DMEM_AW-1:2];
    wire [1:0] dmem_pos       = mem_ea[1:0];
    // PIU window = 0xFE00..0xFEFF exactly (piu.md): bits [DMEM_AW-1:8] == 0..0FE, zero-extended
    // (R21-2: [15:8] alone also mapped 0x1FE00 to the PIU when DMEM_AW = 17)
    wire        piu_hit       = PIU && (op3[6:4] == 3'b001) && !ldd3 && mem_ea[DMEM_AW-1:8] == 8'hFE;
    wire [31:0] piu_rdata;
    wire [31:0] dmem_word     = piu_hit ? piu_rdata : EXT_DMEM ? ext_word3 : dmem[dmem_word_addr];
    wire [31:0] dmem_word2, dmem_word3;   // r28 (mem2.md): the words the two memory units see
    wire [31:0] mem_load, mem_store; wire mem_is_load, mem_is_store, mem_v, mem_misal;
    mem_unit u_mem(.opcode(op3),.mem_val(dmem_word3),.reg_val(gr_r2_3),.pos(dmem_pos),
                   .load_data(mem_load),.store_data(mem_store),
                   .is_load(mem_is_load),.is_store(mem_is_store),.out_valid(mem_v),.misal(mem_misal));

    // ---- second memory unit (slot 2), r28 reviews/contracts/mem2.md ----
    wire [31:0] mem_ea2       = gr_r1_2 + {23'b0, syl2[10:2]};
    wire [DMEM_AW-1:0] dmem_byte_addr2 = mem_ea2[DMEM_AW-1:0];
    wire [DMEM_AW-3:0] dmem_word_addr2 = dmem_byte_addr2[DMEM_AW-1:2];
    wire [1:0] dmem_pos2      = mem_ea2[1:0];
    wire        piu_hit2      = PIU && (op2[6:4] == 3'b001) && !ldd2 && mem_ea2[DMEM_AW-1:8] == 8'hFE;
    wire [31:0] piu_rdata2;
    wire [31:0] mem_load2, mem_store2; wire mem2_is_load, mem2_is_store, mem2_v, mem2_misal;
    mem_unit u_mem2(.opcode(op2),.mem_val(dmem_word2),.reg_val(gr_r2_2),.pos(dmem_pos2),
                    .load_data(mem_load2),.store_data(mem_store2),
                    .is_load(mem2_is_load),.is_store(mem2_is_store),.out_valid(mem2_v),.misal(mem2_misal));
    wire mem2_acc  = (op2[6:4] == 3'b001) && (mem2_is_load || mem2_is_store);
    wire mem3_acc  = (op3[6:4] == 3'b001) && (mem_is_load || mem_is_store);
    // MEM2-03: loads read the memory before the packet; stores apply slot 2 then slot 3, so a slot-3 store to the word
    // slot 2 stores merges into slot 2's word (bare: dmem; EXT: the phase order of MEM2-04 gives the same)
    // r29 IS8-07 (A1, BUG-358): the bare core indexes dmem by the DMEM_AW-bit word address, so two stores whose
    // addresses agree modulo 2^DMEM_AW hit one dmem word; the EXT port sees the whole address
    wire same_word = mem2_acc && mem3_acc && (piu_hit2 == piu_hit) &&
                     (EXT_DMEM ? (mem_ea2[31:2] == mem_ea[31:2]) : (dmem_word_addr2 == dmem_word_addr));
    // MEM2-04: two external accesses run one after the other on the d_* port; a slot-3 load goes first when slot 2
    // stores, else slot 2 goes first. r29 (ldd.md LDD-06): an ldd is two accesses (word 0, then word 1), so a packet
    // has up to 4: the order is slot 2's then slot 3's, or slot 3's first when slot 2 stores. seq_k = the accesses
    // completed (their words kept in sv0..sv2); a packet without ldd runs the r28 sequence cycle for cycle
    wire x2a = EXT_DMEM && ((mem2_acc && !piu_hit2) || ldd_ok2);
    wire x2b = EXT_DMEM && ldd_ok2;
    wire x3a = EXT_DMEM && ((mem3_acc && !piu_hit) || ldd_ok3);
    wire x3b = EXT_DMEM && ldd_ok3;
    wire ext2 = x2a;
    wire ext3 = x3a;
    wire first3 = mem2_is_store && (mem_is_load || ldd3);  // the slot-3 load (or ldd) goes first
    wire [2:0] n2 = {2'b0, x2a} + {2'b0, x2b};
    wire [2:0] n3 = {2'b0, x3a} + {2'b0, x3b};
    wire [2:0] nacc = n2 + n3;                            // external accesses of the packet
    reg  [1:0] seq_k;
    wire seq_ph = (seq_k != 2'd0);                        // r28 name (tb_mem2): the first access completed
    reg [31:0] sv0, sv1, sv2;
    wire [2:0] i2a = first3 ? n3 : 3'd0;                  // index of each access in the order
    wire [2:0] i2b = i2a + 3'd1;
    wire [2:0] i3a = first3 ? 3'd0 : n2;
    wire [2:0] i3b = i3a + 3'd1;
    wire [2:0] k0 = {1'b0, seq_k};
    wire [2:0] k1 = k0 + 3'd1;
    // (pure functions: a function in a continuous assignment is re-evaluated only when its arguments change)
    function acc_is3; input [2:0] j; input f3; input [2:0] m2, m3; begin acc_is3 = f3 ? (j < m3) : (j >= m2); end endfunction
    wire cur3   = (nacc == 3'd0) ? 1'b0 : acc_is3(k0, first3, n2, n3);   // the access on the port now is slot 3's
    wire cur_w1 = cur3 ? (x3b && k0 == i3b) : (x2b && k0 == i2b);   // ... and the second word of its ldd
    // MEM2-10 (r28, DUAL_D): the next access of the order goes out on port 2 at the same time; when the dual-port
    // L1D reports a hit for it (both plain hits, different words) both complete in this cycle (LDD-06: every phase)
    wire dual_try = (DUAL_D != 0) && (k1 < nacc) && !(same_word && mem2_is_store && mem_is_store);   // a load next to a store of its word reads the old word
    wire p2_is3   = acc_is3(k1, first3, n2, n3);                          // port 2 carries the access after the one on port 1
    wire p2_w1    = p2_is3 ? (x3b && k1 == i3b) : (x2b && k1 == i2b);
    function [31:0] accw; input [2:0] j, ka, kb; input dt; input [31:0] r1, r2, s0, s1, s2; begin   // word of access j
        accw = (j == ka) ? r1 : (dt && j == kb) ? r2 : (j == 3'd0) ? s0 : (j == 3'd1) ? s1 : s2;
    end endfunction
    wire [31:0] ext_word2 = accw(i2a, k0, k1, dual_try, d_rdata, d2_rdata, sv0, sv1, sv2);
    wire [31:0] ext_word3 = accw(i3a, k0, k1, dual_try, d_rdata, d2_rdata, sv0, sv1, sv2);
    wire [31:0] ext_ldd2b = accw(i2b, k0, k1, dual_try, d_rdata, d2_rdata, sv0, sv1, sv2);
    wire [31:0] ext_ldd3b = accw(i3b, k0, k1, dual_try, d_rdata, d2_rdata, sv0, sv1, sv2);
    wire [31:0] ldd3_w0 = EXT_DMEM ? ext_word3 : dmem[ldd_wa3];
    wire [31:0] ldd3_w1 = EXT_DMEM ? ext_ldd3b : dmem[ldd_wa3 + 1'b1];
    wire [31:0] ldd2_w0 = EXT_DMEM ? ext_word2 : dmem[ldd_wa2];
    wire [31:0] ldd2_w1 = EXT_DMEM ? ext_ldd2b : dmem[ldd_wa2 + 1'b1];
    wire [31:0] acc_a2 = ldd2 ? ldd_ea2 : mem_ea2;        // the address of each slot's access (word 0)
    wire [31:0] acc_a3 = ldd3 ? ldd_ea3 : mem_ea;
    wire [31:0] base_word2 = piu_hit2 ? piu_rdata2 : EXT_DMEM ? ext_word2 : dmem[dmem_word_addr2];
    assign dmem_word2 = base_word2;
    // bare (or the PIU window) only: a slot-3 store merges into slot 2's stored word; in EXT mode the port already returns it
    assign dmem_word3 = (same_word && mem2_is_store && mem_is_store && (piu_hit || !EXT_DMEM)) ? mem_store2 : dmem_word;

    // ---- control unit (slot 0) ----
    wire [PC_W-1:0] c_pc_goto; wire [31:0] c_link; wire c_taken, c_wlink, c_xnop, c_rfi, c_v;
    wire [11:0] c_xnop_n;
    // BR/BRF read rb = syl[4:2]. With syl[17] set (brn/brfn,
    // reviews/contracts/brnew.md) the branch takes the value that slot 1..3
    // of this packet writes to that BR (slot 3 last, as in the writeback).
    function brw_hit; input [6:0] op; input [1:0] imt; input [31:0] syl; input [2:0] b; begin
        if (op == `ALU_MTB || is_addcg_divs(op))       brw_hit = (syl[4:2] == b);
        else if (is_brimm(op, imt))                    brw_hit = (syl[19:17] == b);
        else if (is_cmp(op) && syl[22:17] == 6'd0)     brw_hit = (syl[4:2] == b);
        else                                           brw_hit = 1'b0;
    end endfunction
    function brw_val; input [6:0] op; input [31:0] r; input c; begin
        brw_val = (op == `ALU_MTB || is_addcg_divs(op)) ? c : r[0];
    end endfunction
    wire [2:0] c_bi = syl0[4:2];
    wire c_new = syl0[17] && (op0 == `CTRL_BR || op0 == `CTRL_BRF);
    // r29 (IS8-05): slot 7 .. 1, the highest slot last (as the writeback); MEM ops of slots 2-3 write no BR
    wire c_br_in = !c_new ? br[c_bi] :
                   brw_hit(op7, imt7, syl7, c_bi) ? brw_val(op7, alu7_r, alu7_c) :
                   brw_hit(op6, imt6, syl6, c_bi) ? brw_val(op6, alu6_r, alu6_c) :
                   brw_hit(op5, imt5, syl5, c_bi) ? brw_val(op5, alu5_r, alu5_c) :
                   brw_hit(op4, imt4, syl4, c_bi) ? brw_val(op4, alu4_r, alu4_c) :
                   brw_hit(op3, imt3, syl3, c_bi) && op3[6:4] != 3'b001 ? brw_val(op3, alu3_r, alu3_c) :
                   brw_hit(op2, imt2, syl2, c_bi) && op2[6:4] != 3'b001 ? brw_val(op2, alu2_r, alu2_c) :
                   brw_hit(op1, imt1, syl1, c_bi) ? brw_val(op1, alu1_r, alu1_c) : br[c_bi];
    // The link register is implicit ($l0.0 = GR63), as in VEX: syl[16:5] is
    // the offset of return/rfi (SP + offset), so it cannot also name a GR.
    ctrl_unit #(.PC_W(PC_W)) u_ctrl(.opcode(op0),.pc(pc),.lr(gr[63]),.sp(gr[1]),.offset(off0),.br(c_br_in),
                     .pc_goto(c_pc_goto),.link_val(c_link),.taken(c_taken),.writes_link(c_wlink),
                     .is_xnop(c_xnop),.xnop_n(c_xnop_n),.is_rfi(c_rfi),.out_valid(c_v));

    wire slot0_is_ctrl = (op0[6:4] == 3'b010);
    wire slot0_is_stop = (op0 == `OP_STOP);

    // ---- second branch unit (slot 1), reviews/contracts/mwbr.md MW-01..04 (round r24, BUG-103) ----
    // Only br/brf/brn/brfn/goto act in slot 1; the slot-1 branch counts only when slot 0 does not change the PC
    // (taken branch, goto, call, return, rfi) and is no xnop (MW-02, program order). brn/brfn in slot 1 take the BR
    // that slot 0, 2 or 3 of this packet writes (slot 3 last, MW-03). A control op in slot 1 writes nothing (MW-04).
    wire slot1_is_ctrl = (op1[6:4] == 3'b010);
    wire slot1_is_br   = (op1 == `CTRL_BR || op1 == `CTRL_BRF || op1 == `CTRL_GOTO);
    // MW-01/04: STOP (0x1f, not class 010) in slot 1 is inert too (Codex R48 F1, BUG-257)
    wire slot1_inert   = slot1_is_ctrl || (op1 == `OP_STOP);
    wire [2:0] c1_bi   = syl1[4:2];
    wire c1_new = syl1[17] && (op1 == `CTRL_BR || op1 == `CTRL_BRF);
    wire c1_br_in = !c1_new ? br[c1_bi] :
                    brw_hit(op7, imt7, syl7, c1_bi) ? brw_val(op7, alu7_r, alu7_c) :
                    brw_hit(op6, imt6, syl6, c1_bi) ? brw_val(op6, alu6_r, alu6_c) :
                    brw_hit(op5, imt5, syl5, c1_bi) ? brw_val(op5, alu5_r, alu5_c) :
                    brw_hit(op4, imt4, syl4, c1_bi) ? brw_val(op4, alu4_r, alu4_c) :
                    brw_hit(op3, imt3, syl3, c1_bi) && op3[6:4] != 3'b001 ? brw_val(op3, alu3_r, alu3_c) :
                    brw_hit(op2, imt2, syl2, c1_bi) && op2[6:4] != 3'b001 ? brw_val(op2, alu2_r, alu2_c) :
                    brw_hit(op0, imt0, syl0, c1_bi) && !slot0_is_ctrl ? brw_val(op0, alu0_r, alu0_c) : br[c1_bi];
    wire c1_cond  = (op1 == `CTRL_GOTO) ? 1'b1 : (op1 == `CTRL_BR) ? c1_br_in : ~c1_br_in;
    wire slot0_redirect = slot0_is_ctrl && (c_taken || c_xnop);
    wire c1_taken = slot1_is_br && c1_cond && !slot0_redirect;
    wire [PC_W-1:0] c1_target = syl1[5+PC_W-1:5];

    // ---- per-slot GR result select (ALU vs MUL) ----
    // r29 (IS8-01): slots 1-3 have only the ALU (and MEM in 2-3); MUL / SIMD16 in 4-7, FPU in 4-5
    wire [31:0] res1 = alu1_r;
    wire [31:0] res2 = alu2_r;
    wire [31:0] res4 = is_fpu(op4) ? fpu4_r : is_simd(op4) ? sm4_r : is_mul(op4) ? mul4_r : alu4_r;
    wire [31:0] res5 = is_fpu(op5) ? fpu5_r : is_simd(op5) ? sm5_r : is_mul(op5) ? mul5_r : alu5_r;
    wire [31:0] res6 = is_simd(op6) ? sm6_r : is_mul(op6) ? mul6_r : alu6_r;
    wire [31:0] res7 = is_simd(op7) ? sm7_r : is_mul(op7) ? mul7_r : alu7_r;
    wire        slot2_is_mem = (op2[6:4] == 3'b001);      // r28 (MEM2-01): the slot-2 memory unit owns the 001xxxx range
    // BUG-320 (r27, review rule RR-03): the slot's unit must decode the op, as slot 0 does (alu0_v): an undefined
    // opcode writes nothing (it wrote the unit's 0 result before r27); r29: a MUL / SIMD / FPU op outside its slots is
    // undefined there (the ALU does not decode it)
    wire        val1 = alu1_v;
    wire        val2 = slot2_is_mem ? 1'b0 : alu2_v;
    wire        val4 = is_fpu(op4) ? fpu4_v : is_simd(op4) ? sm4_v : is_mul(op4) ? mul4_v : alu4_v;
    wire        val5 = is_fpu(op5) ? fpu5_v : is_simd(op5) ? sm5_v : is_mul(op5) ? mul5_v : alu5_v;
    wire        val6 = is_simd(op6) ? sm6_v : is_mul(op6) ? mul6_v : alu6_v;
    wire        val7 = is_simd(op7) ? sm7_v : is_mul(op7) ? mul7_v : alu7_v;

    // ---- external data port / miss stall ----
    assign i_req   = EXT_IMEM && running && !done && stall_cnt == 0;
    assign i_addr  = pc;
    wire   fstall  = i_req && !i_ready;          // packet not fetched yet
    // FP-12 (r27, user decision after the fmax A/B): a packet with an FP op in slot 1 or 2 is held for one cycle
    // (fp_wait: stage 1 of the FPU fills its register; nothing commits, no memory access is presented), then issues
    // with fp_ph = 1 (stage 2 result written back). Its operands cannot change while it is held (no commit).
    reg    fp_ph;
    wire   fp_pkt  = is_fpu(op4) || is_fpu(op5);   // r29: the FPU slots 4-5
    assign fp_wait = running && !done && stall_cnt == 0 && !fstall && !slot0_is_stop && fp_pkt && !fp_ph;
    wire issue   = running && !done && stall_cnt == 0 && !fstall && !slot0_is_stop && !fp_wait;
    wire mem_acc = mem3_acc;
    assign d_req   = issue && (k0 < nacc);
    assign d_we    = cur3 ? mem_is_store : mem2_is_store;
    assign d_addr  = {(cur3 ? acc_a3[31:2] : acc_a2[31:2]) + {29'b0, cur_w1}, 2'b00};
    assign d_wdata = cur3 ? mem_store : mem_store2;
    assign d_word  = cur3 ? ((op3 == `MEM_LDW) || (op3 == `MEM_STW)) : ((op2 == `MEM_LDW) || (op2 == `MEM_STW));
    wire   dstall  = d_req && !d_ready;
    // MEM2-04 / MEM2-05: the first of two external accesses completed: hold the packet one cycle (MEMSEQ)
    assign d2_req   = issue && dual_try;
    assign d2_we    = p2_is3 ? mem_is_store : mem2_is_store;
    assign d2_addr  = {(p2_is3 ? acc_a3[31:2] : acc_a2[31:2]) + {29'b0, p2_w1}, 2'b00};
    assign d2_wdata = p2_is3 ? mem_store : mem_store2;
    wire   dual_ok = dual_try && d2_hit;
    wire [2:0] seq_step = dual_ok ? 3'd2 : 3'd1;
    wire   memseq  = d_req && d_ready && (k0 + seq_step < nacc);

    // ---- performance indicator unit ----
    wire   active  = running && !done;
    wire   commit  = issue && !dstall && !memseq;
    rvex_piu u_piu(.clk(clk), .reset(reset),
        .active(active), .commit(commit), .xnop_c(active && stall_cnt != 0), .fpstall_c(fp_wait),
        .istall_c(active && stall_cnt == 0 && fstall), .dstall_c(dstall),
        .semstall_c(cur3 ? &acc_a3[DMEM_AW-1:8] : &acc_a2[DMEM_AW-1:8]), .memseq_c(memseq),
        .opv({op7, op6, op5, op4, op3, op2, op1, op0}),
        .ctrl0(slot0_is_ctrl), .taken0(slot0_is_ctrl && c_taken && !c_xnop),
        .ctrl1(slot1_is_ctrl), .taken1(c1_taken),   // mwbr.md MW-05
        // r29 (IS8-06): the MUL / SIMD slots 4-7 and the FPU slots 4-5
        .mulv({is_mul(op7), is_mul(op6), is_mul(op5), is_mul(op4)}),
        .fpv({is_fpu(op5), is_fpu(op4)}),            // PIU-20 FLOP (fpu.md FP-07)
        .smv({is_simd(op7), is_simd(op6), is_simd(op5), is_simd(op4)}),   // simd16.md SM-06: MUL16 and MULPKT (r28)
        .ld3((mem_acc && mem_is_load) || ldd3), .st3(mem_acc && mem_is_store),          // LDD-07: an ldd is one LD
        .ld2((mem2_acc && mem2_is_load) || ldd2), .st2(mem2_acc && mem2_is_store),   // MEM2-05 (r28)
        // PIU-18: a test-and-set ldw on hw_sem (system only: EXT_DMEM) that found the lock taken
        // window decoded like rvex_system (d_addr[AW-1:8] all ones; Codex R20 C-1: [15:8] was wrong for AW != 16)
        .semfail3(EXT_DMEM && op3 == `MEM_LDW && &mem_ea[DMEM_AW-1:8] && dmem_word3[0]),
        .semfail2(EXT_DMEM && op2 == `MEM_LDW && &mem_ea2[DMEM_AW-1:8] && dmem_word2[0]),
        // PIU-19 (misalign.md MA-03, r21 BUG-182): a committed slot-3 access that is not naturally aligned
        .misal3((mem_acc && mem_misal) || (ldd3 && !ldd_io3 && ldd_mis3)),   // LDD-03
        .misal2((mem2_acc && mem2_misal) || (ldd2 && !ldd_io2 && ldd_mis2)),
        .addr(mem_ea[7:2]), .rdata(piu_rdata),
        .we(commit && piu_hit && mem_is_store), .wdata(mem_store),
        .addr2(mem_ea2[7:2]), .rdata2(piu_rdata2), .we2(commit && piu_hit2 && mem2_is_store), .wdata2(mem_store2));

    // ================= sequential execution =================
    always @(posedge clk) begin
        if (reset) begin
            pc <= {PC_W{1'b0}}; stall_cnt <= 12'd0; running <= 1'b0; done <= 1'b0; cycles <= 32'd0;
            stall_cycles <= 32'd0; istall_cycles <= 32'd0; fp_ph <= 1'b0; seq_k <= 2'd0;
            sv0 <= 32'd0; sv1 <= 32'd0; sv2 <= 32'd0;
            for (i=0;i<64;i=i+1) gr[i] <= 32'd0;
            for (i=0;i<8;i=i+1)  br[i] <= 1'b0;
        end else begin
            if (start) running <= 1'b1;
            if (running && !done) begin
                cycles <= cycles + 1;
                if (stall_cnt != 0) begin
                    stall_cnt <= stall_cnt - 1;         // XNOP bubble
                end else if (fstall) begin
                    istall_cycles <= istall_cycles + 1;  // I-cache miss: no packet yet
                end else if (dstall) begin
                    stall_cycles <= stall_cycles + 1;    // D-cache miss: hold the packet
                end else if (memseq) begin
                    seq_k <= seq_k + seq_step[1:0];        // MEM2-04 / LDD-06: keep the words done
                    case (seq_k) 2'd0: sv0 <= d_rdata; 2'd1: sv1 <= d_rdata; default: sv2 <= d_rdata; endcase
                    if (dual_ok) case (seq_k) 2'd0: sv1 <= d2_rdata; default: sv2 <= d2_rdata; endcase
                end else if (fp_wait) begin
                    fp_ph <= 1'b1;                       // FP-12: FPU stage 1, hold the packet one cycle
                end else if (slot0_is_stop) begin
                    done <= 1'b1;                        // STOP
                end else begin
                    //--------------- writeback slot 0 ---------------
                    if (slot0_is_ctrl) begin
                        if (c_wlink && dst0 != 6'd0) gr[dst0] <= c_link;   // CALL/RETURN link
                    end else if (op0 == `ALU_MTB) begin
                        br[dbr0] <= alu0_c;
                    end else if (bi0) begin
                        br[syl0[19:17]] <= alu0_r[0];
                    end else if (is_addcg_divs(op0)) begin
                        if (dst0 != 6'd0) gr[dst0] <= alu0_r; br[dbr0] <= alu0_c;
                    end else if (is_cmp(op0) && dst0 == 6'd0) begin
                        br[dbr0] <= alu0_r[0];
                    end else if (op0 != `OP_NOP && alu0_v && dst0 != 6'd0) begin
                        gr[dst0] <= alu0_r;
                    end

                    //--------------- writeback slot 1 ---------------
                    if (slot1_inert) begin end              // MW-04: a slot-1 control op or STOP writes no GR / BR
                    else if (op1 == `ALU_MTB) br[dbr1] <= alu1_c;
                    else if (bi1) br[syl1[19:17]] <= alu1_r[0];
                    else if (is_addcg_divs(op1)) begin
                        if (dst1 != 6'd0) gr[dst1] <= alu1_r; br[dbr1] <= alu1_c;
                    end else if (is_cmp(op1) && dst1 == 6'd0) br[dbr1] <= alu1_r[0];
                    else if (op1 != `OP_NOP && val1 && dst1 != 6'd0) gr[dst1] <= res1;

                    //--------------- writeback slot 2 ---------------
                    if (slot2_is_mem) begin                 // r28 (MEM2-01): slot-2 memory unit
                        if (ldd2) begin                     // ldd.md LDD-02..04 (dst2 after dst1)
                            if (!ldd_io2 && dst2 != 6'd0) gr[dst2] <= ldd_mis2 ? 32'd0 : ldd2_w0;
                            if (!ldd_io2 && syl2[10:5] != 6'd0) gr[syl2[10:5]] <= ldd_mis2 ? 32'd0 : ldd2_w1;
                        end else if (mem2_is_store && !EXT_DMEM && !piu_hit2) dmem[dmem_word_addr2] <= mem_store2;
                        else if (mem2_is_load && dst2 != 6'd0) gr[dst2] <= mem_load2;
                    end else if (op2 == `ALU_MTB) br[dbr2] <= alu2_c;
                    else if (bi2) br[syl2[19:17]] <= alu2_r[0];
                    else if (is_addcg_divs(op2)) begin
                        if (dst2 != 6'd0) gr[dst2] <= alu2_r; br[dbr2] <= alu2_c;
                    end else if (is_cmp(op2) && dst2 == 6'd0) br[dbr2] <= alu2_r[0];
                    else if (op2 != `OP_NOP && val2 && dst2 != 6'd0) gr[dst2] <= res2;

                    //--------------- writeback slot 3 (ALU or MEM) ---------------
                    if (op3[6:4] == 3'b001) begin           // MEM range
                        if (ldd3) begin                     // ldd.md LDD-02..04 (dst2 after dst1)
                            if (!ldd_io3 && dst3 != 6'd0) gr[dst3] <= ldd_mis3 ? 32'd0 : ldd3_w0;
                            if (!ldd_io3 && syl3[10:5] != 6'd0) gr[syl3[10:5]] <= ldd_mis3 ? 32'd0 : ldd3_w1;
                        end else if (mem_is_store && !EXT_DMEM && !piu_hit) dmem[dmem_word_addr] <= mem_store;
                        else if (mem_is_load && dst3 != 6'd0) gr[dst3] <= mem_load;
                        // PFT / SYL_FOLLOW -> no writeback
                    end else if (op3 == `ALU_MTB) br[dbr3] <= alu3_c;
                    else if (bi3) br[syl3[19:17]] <= alu3_r[0];
                    else if (is_addcg_divs(op3)) begin
                        if (dst3 != 6'd0) gr[dst3] <= alu3_r; br[dbr3] <= alu3_c;
                    end else if (is_cmp(op3) && dst3 == 6'd0) br[dbr3] <= alu3_r[0];
                    else if (op3 != `OP_NOP && alu3_v && dst3 != 6'd0) gr[dst3] <= alu3_r;

                    //--------------- writeback slots 4..7 (ALU / MUL / SIMD16; FPU 4-5), r29 IS8-01 ---------------
                    if (op4 == `ALU_MTB) br[dbr4] <= alu4_c;
                    else if (bi4) br[syl4[19:17]] <= alu4_r[0];
                    else if (is_addcg_divs(op4)) begin
                        if (dst4 != 6'd0) gr[dst4] <= alu4_r; br[dbr4] <= alu4_c;
                    end else if (is_cmp(op4) && dst4 == 6'd0) br[dbr4] <= alu4_r[0];
                    else if (op4 != `OP_NOP && val4 && dst4 != 6'd0) gr[dst4] <= res4;
                    if (op5 == `ALU_MTB) br[dbr5] <= alu5_c;
                    else if (bi5) br[syl5[19:17]] <= alu5_r[0];
                    else if (is_addcg_divs(op5)) begin
                        if (dst5 != 6'd0) gr[dst5] <= alu5_r; br[dbr5] <= alu5_c;
                    end else if (is_cmp(op5) && dst5 == 6'd0) br[dbr5] <= alu5_r[0];
                    else if (op5 != `OP_NOP && val5 && dst5 != 6'd0) gr[dst5] <= res5;
                    if (op6 == `ALU_MTB) br[dbr6] <= alu6_c;
                    else if (bi6) br[syl6[19:17]] <= alu6_r[0];
                    else if (is_addcg_divs(op6)) begin
                        if (dst6 != 6'd0) gr[dst6] <= alu6_r; br[dbr6] <= alu6_c;
                    end else if (is_cmp(op6) && dst6 == 6'd0) br[dbr6] <= alu6_r[0];
                    else if (op6 != `OP_NOP && val6 && dst6 != 6'd0) gr[dst6] <= res6;
                    if (op7 == `ALU_MTB) br[dbr7] <= alu7_c;
                    else if (bi7) br[syl7[19:17]] <= alu7_r[0];
                    else if (is_addcg_divs(op7)) begin
                        if (dst7 != 6'd0) gr[dst7] <= alu7_r; br[dbr7] <= alu7_c;
                    end else if (is_cmp(op7) && dst7 == 6'd0) br[dbr7] <= alu7_r[0];
                    else if (op7 != `OP_NOP && val7 && dst7 != 6'd0) gr[dst7] <= res7;

                    //--------------- PC / stall update ---------------
                    fp_ph <= 1'b0;                          // FP-12: the next packet starts in stage 1
                    seq_k <= 2'd0;                          // MEM2-04: the next packet starts with its first access
                    if (slot0_is_ctrl && c_xnop) begin
                        stall_cnt <= c_xnop_n;              // NEW: XNOP stall n cycles
                        pc <= pc + 1'b1;
                    end else if (slot0_is_ctrl && c_taken) begin
                        pc <= c_pc_goto;                    // branch / call / return
                    end else if (c1_taken) begin
                        pc <= c1_target;                    // MW-02: slot-1 branch (slot 0 did not redirect)
                    end else begin
                        pc <= pc + 1'b1;
                    end
                end
            end
            gr[0] <= 32'd0; // r0 hardwired
        end
    end
endmodule
