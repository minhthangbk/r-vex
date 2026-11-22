//======================================================================
// r-VEX | Performance Indicator Unit (PIU), one per core (2026-10-31 r5)
//----------------------------------------------------------------------
// Contract: reviews/contracts/piu.md (items PIU-01..15).
// Event counters, memory-mapped at 0xFE00..0xFEFF of the core's data space
// (rvex_core decodes addr[15:8] == 8'hFE on slot 3; the access never reaches
// dmem, the D port or the caches, and never stalls).
//   0x00 CTRL   bit0 EN (reset 1), bit1 CLR (write 1 = clear all; reads 0)
//   0x04 ID     0x50495501
//   0x08 CYCLE  0x0C CYCLEH  0x10 PKT  0x14 OPS  0x18 ALU  0x1C MUL16
//   0x20 MUL32  0x24 LD  0x28 ST  0x2C CTRL_OPS  0x30 BRT  0x34 NOPPKT
//   0x38 XNOP   0x3C ISTALL  0x40 DSTALL  0x44..0x50 SLOT0..SLOT3
//   0x54 MULPKT (packets with a multiply)  0x58 SEMSTALL (DSTALL cycles waiting for hw_sem)
//   0x5C SEMFAIL (committed ldw to hw_sem 0xFFxx that returned 1 = lock taken, 2026-11-04 r10, PIU-18)
//   0x60 MISALIGN (committed slot-3 accesses not naturally aligned, 2026-11-10 r21, PIU-19, misalign.md MA-03)
//   0x64 FLOP (committed FADD / FSUB / FMUL ops, one FLOP each, 2026-11-16 r27, PIU-20, fpu.md FP-07)
//   0x68 FPSTALL (r27, PIU-21)  0x6C MEMSEQ (cycles a packet is held between its two external accesses, r28 mem2.md MEM2-05)
//   0x70..0x7C SLOT4..SLOT7 (r29 issue8.md IS8-06: non-NOP syllables of the committed packets in slots 4..7)
// Two window ports (r28, mem2.md MEM2-01): slot 2 and slot 3 read and write the window; two stores of one packet apply
// in slot order (slot 3 last).
// Events of a cycle are counted with the EN value held during that cycle;
// a CTRL store takes effect at the end of its cycle, and CLR wins over the
// increments of that cycle.
//======================================================================
`ifndef RVEX_PIU_V
`define RVEX_PIU_V
module rvex_piu (
    input  wire        clk,
    input  wire        reset,
    // cycle classes (exclusive; at most one of commit/xnop/istall/dstall/stop)
    input  wire        active,     // running && !done
    input  wire        commit,     // the packet commits this cycle
    input  wire        xnop_c,     // XNOP bubble cycle
    input  wire        fpstall_c,  // FP-12 / PIU-21 (r27): a packet with an FP op held for FPU stage 1
    input  wire        istall_c,   // held by an I-fetch miss
    input  wire        dstall_c,   // held by a D miss / hw_sem wait
    input  wire        semstall_c, // ... of which: waiting for hw_sem (address 0xFFxx)
    input  wire        semfail3,   // slot 3 is an ldw to hw_sem whose answer is 1 (busy); counted on commit
    input  wire        misal3,     // slot 3 access is not naturally aligned (PIU-19); counted on commit
    input  wire        semfail2,   // r28 (mem2.md MEM2-05): the same for slot 2
    input  wire        misal2,
    input  wire        memseq_c,   // r28 (MEM2-05): held after the first of two external accesses completed
    // committed packet contents
    input  wire [55:0] opv,        // r29 (IS8-06): the 8 opcodes, slot k in [7k+6:7k]
    input  wire        ctrl0,      // slot 0 holds a control op
    input  wire        taken0,     // ... and it changes the PC
    input  wire        ctrl1,      // slot 1 holds a control op (mwbr.md MW-05, r24)
    input  wire        taken1,     // ... a slot-1 branch changed the PC (slot 0 did not)
    input  wire [3:0]  mulv,       // r29: slot 4..7 op is in the multiply range (000xxxx, not NOP)
    input  wire [3:0]  fpv,        // slot 4..7 op is an FPU op (PIU-20: not a multiply, not ALU; r31 FP-13: 4 FPUs)
    input  wire [3:0]  smv,        // r28 (simd16.md SM-06): slot 4..7 op is a SIMD16 dual MAC (one MUL16 op each)
    input  wire        ld3, st3,   // slot 3 op is a load / a store
    input  wire        ld2, st2,   // r28 (MEM2-05): slot 2 op is a load / a store
    // register port (slot 3)
    input  wire [7:2]  addr,       // word offset in the window
    output reg  [31:0] rdata,
    input  wire        we,         // store to the window commits this cycle
    input  wire [31:0] wdata,      // merged store word
    // register port (slot 2, r28 mem2.md MEM2-01)
    input  wire [7:2]  addr2,
    output reg  [31:0] rdata2,
    input  wire        we2,
    input  wire [31:0] wdata2
);
    reg        en;
    reg [63:0] cyc;
    reg [31:0] pkt, ops, alu, mul16, mul32, ld, st, ctl, brt, noppkt, xnop, istall, dstall, mulpkt, semstall, semfail, misalign, flop, fpstall, memseq;
    reg [31:0] slot [0:7];
    integer k;

    // 16x16 opcodes are 1..6, 32x16 partial products 7..11 (rvex_defs.vh); the undefined
    // 000xxxx opcodes 12..15 are not multiplies for the PIU (PIU-08, Codex R10) and count as ALU
    wire [6:0] op [0:7];
    genvar gs;
    generate for (gs = 0; gs < 8; gs = gs + 1) begin : g_op
        assign op[gs] = opv[7*gs+6:7*gs];
    end endgenerate
    wire [3:0] md, m16;
    generate for (gs = 0; gs < 4; gs = gs + 1) begin : g_md
        assign md[gs]  = (mulv[gs] && op[gs+4] <= 7'd11) || smv[gs];
        assign m16[gs] = (mulv[gs] && op[gs+4] <= 7'd6) || smv[gs];
    end endgenerate
    wire [7:0] nz = {op[7] != 7'd0, op[6] != 7'd0, op[5] != 7'd0, op[4] != 7'd0,
                     op[3] != 7'd0, op[2] != 7'd0, op[1] != 7'd0, op[0] != 7'd0};
    wire [3:0] n_ops = nz[0] + nz[1] + nz[2] + nz[3] + nz[4] + nz[5] + nz[6] + nz[7];
    wire [2:0] n_m16 = m16[0] + m16[1] + m16[2] + m16[3];
    wire [2:0] n_m32 = (md[0] && !m16[0]) + (md[1] && !m16[1]) + (md[2] && !m16[2]) + (md[3] && !m16[3]);
    wire [2:0] n_md  = md[0] + md[1] + md[2] + md[3];
    wire [2:0] n_fp  = fpv[0] + fpv[1] + fpv[2] + fpv[3];
    wire [3:0] n_alu = n_ops - n_md - n_fp - ld3 - st3 - ld2 - st2 - ctrl0 - ctrl1;
    wire       cnt = active && en;
    wire       cpk = cnt && commit;

    // two read ports (r28, mem2.md MEM2-01); written out twice: an always @(*) that calls a function is not
    // sensitive to the counters the function reads
    always @(*) begin
        case (addr)
            6'h00: rdata = {31'd0, en};
            6'h01: rdata = 32'h5049_5501;
            6'h02: rdata = cyc[31:0];
            6'h03: rdata = cyc[63:32];
            6'h04: rdata = pkt;
            6'h05: rdata = ops;
            6'h06: rdata = alu;
            6'h07: rdata = mul16;
            6'h08: rdata = mul32;
            6'h09: rdata = ld;
            6'h0A: rdata = st;
            6'h0B: rdata = ctl;
            6'h0C: rdata = brt;
            6'h0D: rdata = noppkt;
            6'h0E: rdata = xnop;
            6'h0F: rdata = istall;
            6'h10: rdata = dstall;
            6'h11: rdata = slot[0];
            6'h12: rdata = slot[1];
            6'h13: rdata = slot[2];
            6'h14: rdata = slot[3];
            6'h15: rdata = mulpkt;
            6'h16: rdata = semstall;
            6'h17: rdata = semfail;
            6'h18: rdata = misalign;
            6'h19: rdata = flop;
            6'h1A: rdata = fpstall;     // PIU-21 (r27): 0xFE68
            6'h1B: rdata = memseq;      // MEM2-05 (r28): 0xFE6C
            6'h1C: rdata = slot[4];     // IS8-06 (r29): 0xFE70..0xFE7C
            6'h1D: rdata = slot[5];
            6'h1E: rdata = slot[6];
            6'h1F: rdata = slot[7];
            default: rdata = 32'd0;
        endcase
    end
    always @(*) begin
        case (addr2)
            6'h00: rdata2 = {31'd0, en};
            6'h01: rdata2 = 32'h5049_5501;
            6'h02: rdata2 = cyc[31:0];
            6'h03: rdata2 = cyc[63:32];
            6'h04: rdata2 = pkt;
            6'h05: rdata2 = ops;
            6'h06: rdata2 = alu;
            6'h07: rdata2 = mul16;
            6'h08: rdata2 = mul32;
            6'h09: rdata2 = ld;
            6'h0A: rdata2 = st;
            6'h0B: rdata2 = ctl;
            6'h0C: rdata2 = brt;
            6'h0D: rdata2 = noppkt;
            6'h0E: rdata2 = xnop;
            6'h0F: rdata2 = istall;
            6'h10: rdata2 = dstall;
            6'h11: rdata2 = slot[0];
            6'h12: rdata2 = slot[1];
            6'h13: rdata2 = slot[2];
            6'h14: rdata2 = slot[3];
            6'h15: rdata2 = mulpkt;
            6'h16: rdata2 = semstall;
            6'h17: rdata2 = semfail;
            6'h18: rdata2 = misalign;
            6'h19: rdata2 = flop;
            6'h1A: rdata2 = fpstall;     // PIU-21 (r27): 0xFE68
            6'h1B: rdata2 = memseq;      // MEM2-05 (r28): 0xFE6C
            6'h1C: rdata2 = slot[4];     // IS8-06 (r29): 0xFE70..0xFE7C
            6'h1D: rdata2 = slot[5];
            6'h1E: rdata2 = slot[6];
            6'h1F: rdata2 = slot[7];
            default: rdata2 = 32'd0;
        endcase
    end

    always @(posedge clk) begin
        if (reset || (we && addr == 6'h00 && wdata[1]) || (we2 && addr2 == 6'h00 && wdata2[1])) begin
            cyc <= 64'd0; pkt <= 0; ops <= 0; alu <= 0; mul16 <= 0; mul32 <= 0; ld <= 0; st <= 0;
            ctl <= 0; brt <= 0; noppkt <= 0; xnop <= 0; istall <= 0; dstall <= 0; mulpkt <= 0; semstall <= 0; semfail <= 0;
            misalign <= 0; flop <= 0; fpstall <= 0; memseq <= 0;
            for (k = 0; k < 8; k = k + 1) slot[k] <= 0;
        end else begin
            if (cnt) cyc <= cyc + 64'd1;
            if (cnt && xnop_c)   xnop   <= xnop + 1;
            if (cnt && fpstall_c) fpstall <= fpstall + 1;
            if (cnt && memseq_c) memseq <= memseq + 1;
            if (cnt && istall_c) istall <= istall + 1;
            if (cnt && dstall_c) dstall <= dstall + 1;
            if (cnt && dstall_c && semstall_c) semstall <= semstall + 1;
            if (cpk) begin
                pkt   <= pkt + 1;
                ops   <= ops + n_ops;
                alu   <= alu + n_alu;
                mul16 <= mul16 + n_m16;
                mul32 <= mul32 + n_m32;
                ld    <= ld + ld3 + ld2;
                st    <= st + st3 + st2;
                ctl   <= ctl + ctrl0 + ctrl1;
                brt   <= brt + (taken0 | taken1);
                if (nz == 8'd0) noppkt <= noppkt + 1;
                if (md != 4'd0) mulpkt <= mulpkt + 1;
                semfail <= semfail + semfail3 + semfail2;
                misalign <= misalign + misal3 + misal2;
                flop  <= flop + n_fp;
                for (k = 0; k < 8; k = k + 1) if (nz[k]) slot[k] <= slot[k] + 1;
            end
        end
        if (reset) en <= 1'b1;
        else if (we && addr == 6'h00) en <= wdata[0];          // slot 3 last (MEM2-03)
        else if (we2 && addr2 == 6'h00) en <= wdata2[0];
    end
endmodule
`endif
