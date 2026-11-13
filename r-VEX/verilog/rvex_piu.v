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
    input  wire        istall_c,   // held by an I-fetch miss
    input  wire        dstall_c,   // held by a D miss / hw_sem wait
    input  wire        semstall_c, // ... of which: waiting for hw_sem (address 0xFFxx)
    input  wire        semfail3,   // slot 3 is an ldw to hw_sem whose answer is 1 (busy); counted on commit
    input  wire        misal3,     // slot 3 access is not naturally aligned (PIU-19); counted on commit
    // committed packet contents
    input  wire [6:0]  op0, op1, op2, op3,
    input  wire        ctrl0,      // slot 0 holds a control op
    input  wire        taken0,     // ... and it changes the PC
    input  wire        ctrl1,      // slot 1 holds a control op (mwbr.md MW-05, r24)
    input  wire        taken1,     // ... a slot-1 branch changed the PC (slot 0 did not)
    input  wire        mul1, mul2, // slot 1/2 op is a multiply
    input  wire        ld3, st3,   // slot 3 op is a load / a store
    // register port (slot 3)
    input  wire [7:2]  addr,       // word offset in the window
    output reg  [31:0] rdata,
    input  wire        we,         // store to the window commits this cycle
    input  wire [31:0] wdata       // merged store word
);
    reg        en;
    reg [63:0] cyc;
    reg [31:0] pkt, ops, alu, mul16, mul32, ld, st, ctl, brt, noppkt, xnop, istall, dstall, mulpkt, semstall, semfail, misalign;
    reg [31:0] slot [0:3];
    integer k;

    // 16x16 opcodes are 1..6, 32x16 partial products 7..11 (rvex_defs.vh); the undefined
    // 000xxxx opcodes 12..15 are not multiplies for the PIU (PIU-08, Codex R10) and count as ALU
    wire md1 = mul1 && op1 <= 7'd11, md2 = mul2 && op2 <= 7'd11;
    wire m16_1 = md1 && op1 <= 7'd6, m16_2 = md2 && op2 <= 7'd6;
    wire [3:0] nz = {op3 != 7'd0, op2 != 7'd0, op1 != 7'd0, op0 != 7'd0};
    wire [2:0] n_ops = nz[0] + nz[1] + nz[2] + nz[3];
    wire [1:0] n_m16 = m16_1 + m16_2;
    wire [1:0] n_m32 = (md1 && !m16_1) + (md2 && !m16_2);
    wire [2:0] n_alu = n_ops - (md1 + md2) - ld3 - st3 - ctrl0 - ctrl1;
    wire       cnt = active && en;
    wire       cpk = cnt && commit;

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
            default: rdata = 32'd0;
        endcase
    end

    always @(posedge clk) begin
        if (reset || (we && addr == 6'h00 && wdata[1])) begin
            cyc <= 64'd0; pkt <= 0; ops <= 0; alu <= 0; mul16 <= 0; mul32 <= 0; ld <= 0; st <= 0;
            ctl <= 0; brt <= 0; noppkt <= 0; xnop <= 0; istall <= 0; dstall <= 0; mulpkt <= 0; semstall <= 0; semfail <= 0;
            misalign <= 0;
            for (k = 0; k < 4; k = k + 1) slot[k] <= 0;
        end else begin
            if (cnt) cyc <= cyc + 64'd1;
            if (cnt && xnop_c)   xnop   <= xnop + 1;
            if (cnt && istall_c) istall <= istall + 1;
            if (cnt && dstall_c) dstall <= dstall + 1;
            if (cnt && dstall_c && semstall_c) semstall <= semstall + 1;
            if (cpk) begin
                pkt   <= pkt + 1;
                ops   <= ops + n_ops;
                alu   <= alu + n_alu;
                mul16 <= mul16 + n_m16;
                mul32 <= mul32 + n_m32;
                ld    <= ld + ld3;
                st    <= st + st3;
                ctl   <= ctl + ctrl0 + ctrl1;
                brt   <= brt + (taken0 | taken1);
                if (nz == 4'd0) noppkt <= noppkt + 1;
                if (md1 || md2) mulpkt <= mulpkt + 1;
                if (semfail3) semfail <= semfail + 1;
                if (misal3) misalign <= misalign + 1;
                for (k = 0; k < 4; k = k + 1) if (nz[k]) slot[k] <= slot[k] + 1;
            end
        end
        if (reset) en <= 1'b1;
        else if (we && addr == 6'h00) en <= wdata[0];
    end
endmodule
`endif
