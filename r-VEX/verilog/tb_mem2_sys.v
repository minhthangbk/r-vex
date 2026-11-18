//======================================================================
// r-VEX | two memory ops per packet on the dual-port L1D (round r28)
//----------------------------------------------------------------------
// Contract: reviews/contracts/mem2.md MEM2-10 (L1D_DUAL = 1, user decision "L1D 2 cổng khi hit").
// Core 0 warms two lines (A in M, B in S), then runs one packet per case:
//   D1 two loads, both hit                      -> one cycle (dual), no MEMSEQ
//   D2 load hit + store hit to an M line        -> dual
//   D3 store + load of the same word            -> dual, the load reads the old word
//   D4 store + sub-word store of the same word  -> sequenced (MEMSEQ), slot 3's byte on top
//   D5 load + store to an S line                -> sequenced (the store needs an upgrade)
//   D6 load hit + load miss                     -> sequenced (port 2 serves hits only)
// r28 Codex R55 F5 (D7..D11, counted apart from D1..D6):
//   D7 core 0: 24 packets "load A | store E_k" (port 2 = the store, lines E_k in M) while core 1 stores to the same
//      lines (RDX / snoops): a snoop on the port-2 index in a would-be dual cycle must sequence it (s_block2; the
//      cycles are counted, >= 1 is required); every store of both cores lands (3 words x 8 lines + core 1's word)
//   D8 load miss of D (same index as the M line A, a fill forwarded to port 1) + store to A on port 2 -> sequenced
//   D9 cached load + PIU / hw_sem load on port 2 (MMIO is never dual): the values of the window
//   D10 misaligned on a dual candidate: crossing sth on port 2 (word unchanged), pos-1 sth next to a load of its word
//   D11 an xnop before a dual packet
//   D12 (final verify r28 F2) busy hw_sem ldw in slot 2 next to a cached load: SEMFAIL counts slot 2 (MEM2-05)
// After STOP (RR-11): drain with a cap: no core demand, prefetch, write-back buffer or bus request in flight.
// Checks: the loaded values, the memory words (coherent peek), the number of dual commits (3) and of MEMSEQ
// cycles (3) of D1..D6, D8..D10 sequenced, D11 dual, and that the run ends.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_mem2_sys;
    reg clk = 0, reset = 1, start0 = 0, start1 = 0;
    wire done0, done1; wire [31:0] cycles0, cycles1;
    integer pass = 0, fail = 0, k, j, ndual = 0, nseq = 0, nd7 = 0, nsnoop2 = 0, ndual2 = 0, nseq2 = 0, nseqall = 0, nseq3 = 0;
    reg [31:0] pc0;

    rvex_system dut(.clk(clk), .reset(reset), .start0(start0), .start1(start1),
        .done0(done0), .done1(done1), .cycles0(cycles0), .cycles1(cycles1));
    always #5 clk = ~clk;

    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s,b; input [8:0] off; begin
        mst = {op,`NO_IMM,s,b,off,2'b00}; end endfunction
    localparam [31:0] NOP = 32'd0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};
    function [31:0] xnop; input [11:0] n; begin xnop = {`CTRL_XNOP,8'd0,n,5'd0}; end endfunction
    localparam [31:0] A0 = 32'hA0A0_0000, A1 = 32'hA1A1_1111, A2 = 32'hA2A2_2222, A3 = 32'hA3A3_3333;
    localparam [31:0] B0 = 32'hB0B0_0000, B1 = 32'hB1B1_1111, C0 = 32'hC0C0_0000;

    task P0; input [11:0] a; input [31:0] s0,s1,s2,s3; begin dut.cpu[0].l1i.rom[a] = {s3,s2,s1,s0}; end endtask
    task chk; input [255:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s : %0h", nm, got); end
        else begin fail = fail + 1; $display("  FAIL %0s : got %0h exp %0h", nm, got, exp); end
    end endtask

    always @(posedge clk) begin
        pc0 = dut.cpu[0].core.pc;
        if (dut.cpu[0].core.memseq) nseqall = nseqall + 1;
        if (pc0 < 11) begin
            if (dut.cpu[0].core.dual_ok && dut.cpu[0].core.commit) ndual = ndual + 1;
            if (dut.cpu[0].core.memseq) nseq = nseq + 1;
        end else if (pc0 >= 40 && pc0 < 64) begin
            if (dut.cpu[0].core.dual_ok && dut.cpu[0].core.commit) nd7 = nd7 + 1;
            // D7: a would-be dual cycle (both plain hits, cache free) held by a snoop of the port-2 index
            if (dut.cpu[0].l1d.c2_req && dut.cpu[0].l1d.c2_ok && dut.cpu[0].l1d.c_req && dut.cpu[0].l1d.c_ok &&
                !dut.cpu[0].l1d.busy && !dut.cpu[0].l1d.s_block && dut.cpu[0].l1d.s_block2) nsnoop2 = nsnoop2 + 1;
        end else if (pc0 >= 64 && pc0 < 72) begin
            if (dut.cpu[0].core.dual_ok && dut.cpu[0].core.commit) ndual2 = ndual2 + 1;
            if (dut.cpu[0].core.memseq) nseq2 = nseq2 + 1;
        end else if (pc0 >= 72) begin
            if (dut.cpu[0].core.memseq) nseq3 = nseq3 + 1;          // D12
        end
    end

    initial begin
        for (k = 0; k < 4096; k = k + 1) begin dut.cpu[0].l1i.rom[k] = 128'd0; dut.cpu[1].l1i.rom[k] = 128'd0; end
        for (k = 0; k < 16384; k = k + 1) dut.l2.mem[k] = 32'd0;
        dut.l2.mem[16'h1000 >> 2] = A0; dut.l2.mem[16'h1004 >> 2] = A1;
        dut.l2.mem[16'h1008 >> 2] = A2; dut.l2.mem[16'h100C >> 2] = A3;
        dut.l2.mem[16'h1100 >> 2] = B0; dut.l2.mem[16'h1104 >> 2] = B1; dut.l2.mem[16'h1200 >> 2] = C0;
        P0(0, ri(`ALU_ADD,6'd12,6'd0,9'h10), ri(`ALU_ADD,6'd14,6'd0,9'h11), ri(`ALU_ADD,6'd16,6'd0,9'h12), ri(`ALU_ADD,6'd30,6'd0,9'h55));
        P0(1, ri(`ALU_SHL,6'd12,6'd12,9'd8), ri(`ALU_SHL,6'd14,6'd14,9'd8), ri(`ALU_SHL,6'd16,6'd16,9'd8), ri(`ALU_ADD,6'd31,6'd0,9'h66));
        P0(2, NOP, NOP, NOP, ri(`MEM_LDW,6'd20,6'd12,9'd0));                              // line A -> S
        P0(3, NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd14,9'd0));                              // line B -> S
        P0(4, NOP, NOP, NOP, mst(`MEM_STW,6'd30,6'd12,9'd8));                             // A2 = 0x55, line A -> M
        P0(5, NOP, NOP, ri(`MEM_LDW,6'd22,6'd12,9'd4), ri(`MEM_LDW,6'd23,6'd14,9'd0));     // D1
        P0(6, NOP, NOP, ri(`MEM_LDW,6'd24,6'd14,9'd0), mst(`MEM_STW,6'd31,6'd12,9'd4));    // D2: A1 = 0x66
        P0(7, NOP, NOP, mst(`MEM_STW,6'd30,6'd12,9'd4), ri(`MEM_LDW,6'd25,6'd12,9'd4));    // D3: r25 = 0x66, A1 = 0x55
        P0(8, NOP, NOP, mst(`MEM_STW,6'd30,6'd12,9'd12), mst(`MEM_STB,6'd31,6'd12,9'd12));   // D4: A3 = 0x66000055
        P0(9, NOP, NOP, ri(`MEM_LDW,6'd26,6'd12,9'd0), mst(`MEM_STW,6'd30,6'd14,9'd4));    // D5: B1 = 0x55 (upgrade)
        P0(10, NOP, NOP, ri(`MEM_LDW,6'd27,6'd12,9'd0), ri(`MEM_LDW,6'd28,6'd16,9'd0));    // D6: line C misses
        // D7..D11 setup: r40 = 0x1400 (lines E_0..E_7), r41 = 0x2000 (D, the index of A), r42 = 0xFF00 (hw_sem),
        // r43 = 0x1300 (line F), r44 = 0x77, r39 = 0xFE00 (PIU)
        P0(11, ri(`ALU_ADD,6'd40,6'd0,9'h14), ri(`ALU_ADD,6'd41,6'd0,9'h20), ri(`ALU_ADD,6'd43,6'd0,9'h13), ri(`ALU_ADD,6'd44,6'd0,9'h77));
        P0(12, ri(`ALU_SHL,6'd40,6'd40,9'd8), ri(`ALU_SHL,6'd41,6'd41,9'd8), ri(`ALU_SHL,6'd43,6'd43,9'd8), ri(`ALU_ADD,6'd42,6'd0,9'hFF));
        P0(13, ri(`ALU_SHL,6'd42,6'd42,9'd8), ri(`ALU_ADD,6'd39,6'd0,9'hFE), NOP, ri(`MEM_LDW,6'd45,6'd43,9'd0));   // F -> S
        P0(14, ri(`ALU_SHL,6'd39,6'd39,9'd8), NOP, NOP, mst(`MEM_STW,6'd30,6'd43,9'd4));                            // F -> M
        for (j = 0; j < 8; j = j + 1) P0(15 + j, NOP, NOP, NOP, mst(`MEM_STW,6'd30,6'd40,j*16));                    // E_j -> M
        for (k = 23; k < 40; k = k + 1) P0(k, NOP, NOP, NOP, NOP);
        // D7: line by line, three packets per line store word 0 / 4 / 12 (values r30 / r31 / r44): after a miss the
        // next stores to the line are dual hits until core 1's RDX snoops it
        for (j = 0; j < 24; j = j + 1)
            P0(40 + j, NOP, NOP, ri(`MEM_LDW,6'd46,6'd12,9'd0),
               mst(`MEM_STW, j % 3 == 0 ? 6'd30 : j % 3 == 1 ? 6'd31 : 6'd44, 6'd40,
                   (j / 3) * 16 + (j % 3 == 0 ? 0 : j % 3 == 1 ? 4 : 12)));
        P0(64, NOP, NOP, ri(`MEM_LDW,6'd47,6'd41,9'd0), mst(`MEM_STW,6'd44,6'd12,9'd0));    // D8: D misses, A0 = 0x77
        P0(65, NOP, NOP, ri(`MEM_LDW,6'd48,6'd43,9'd4), ri(`MEM_LDW,6'd49,6'd42,9'd0));     // D9: r49 = hw_sem lock 0 (free: 0)
        P0(66, NOP, NOP, ri(`MEM_LDW,6'd50,6'd43,9'd4), ri(`MEM_LDW,6'd51,6'd42,9'd0));     // D9: r51 = 1 (taken)
        P0(67, NOP, NOP, ri(`MEM_LDW,6'd52,6'd43,9'd4), ri(`MEM_LDW,6'd53,6'd39,9'd4));     // D9: r53 = PIU ID (0xFE04, core-local)
        P0(68, NOP, NOP, ri(`MEM_LDW,6'd54,6'd43,9'd4), mst(`MEM_STH,6'd44,6'd43,9'd7));    // D10: crossing sth, F1 kept
        P0(69, NOP, NOP, mst(`MEM_STH,6'd44,6'd43,9'd9), ri(`MEM_LDW,6'd55,6'd43,9'd8));    // D10: r55 = F2 old, F2 = 0x7700
        P0(70, xnop(12'd40), NOP, NOP, NOP);                                                 // D8's prefetch drains (busy: no port 2)
        P0(71, NOP, NOP, ri(`MEM_LDW,6'd56,6'd43,9'd4), ri(`MEM_LDW,6'd57,6'd43,9'd12));    // D11: dual after an xnop (F3 = 0)
        // D12 (final verify r28 F2, BUG-352): a busy hw_sem ldw in slot 2 (lock 0, taken since D9) next to a cached
        // load in slot 3: SEMFAIL counts slot 2 too (MEM2-05, PIU-18); MMIO never takes port 2, so it is sequenced
        P0(72, NOP, NOP, ri(`MEM_LDW,6'd58,6'd42,9'd0), ri(`MEM_LDW,6'd59,6'd43,9'd4));
        P0(73, STOP, NOP, NOP, NOP);
        // core 1: a delay, then 6 passes over E_0..E_7 storing word 8 (RDX of the lines core 0 keeps in M)
        dut.cpu[1].l1i.rom[0] = {NOP, NOP, NOP, ri(`ALU_ADD,6'd40,6'd0,9'h14)};
        dut.cpu[1].l1i.rom[1] = {NOP, NOP, ri(`ALU_ADD,6'd33,6'd0,9'h99), ri(`ALU_SHL,6'd40,6'd40,9'd8)};
        dut.cpu[1].l1i.rom[2] = {NOP, NOP, NOP, xnop(12'd150)};
        for (j = 0; j < 48; j = j + 1) dut.cpu[1].l1i.rom[3 + j] = {mst(`MEM_STW,6'd33,6'd40,(j % 8) * 16 + 8), NOP, NOP, NOP};
        dut.cpu[1].l1i.rom[51] = {NOP, NOP, NOP, STOP};

        repeat (2) @(posedge clk);
        reset = 0; start0 = 1; start1 = 1;
        k = 0;
        while (!(done0 === 1'b1 && done1 === 1'b1) && k < 6000) begin @(posedge clk); k = k + 1; end
        // RR-11 drain: no demand, prefetch, write-back buffer or bus request in flight (cap 2000 cycles)
        j = 0;
        while ((dut.cpu[0].l1d.busy || dut.cpu[1].l1d.busy || dut.cpu[0].l1d.b_req || dut.cpu[1].l1d.b_req ||
                dut.cpu[0].l1d.wv || dut.cpu[1].l1d.wv || dut.bus.busy) && j < 2000) begin @(posedge clk); j = j + 1; end
        if (j >= 2000) $display("DRAIN TIMEOUT");
        chk("RR-11 drained (no demand, prefetch, WBB or bus request)", j < 2000, 1);
        repeat (2) @(posedge clk);
        $display("tb_mem2_sys: core 0 %0d cycles, %0d dual commits, %0d MEMSEQ cycles", cycles0, ndual, nseq);
        chk("run ends", done0 && done1, 1);
        chk("D1 slot-2 load (A1)", dut.cpu[0].core.gr[22], A1);
        chk("D1 slot-3 load (B0)", dut.cpu[0].core.gr[23], B0);
        chk("D2 load next to a store (B0)", dut.cpu[0].core.gr[24], B0);
        chk("D3 load reads the word before the packet's store", dut.cpu[0].core.gr[25], 32'h66);
        chk("D5 load (A0)", dut.cpu[0].core.gr[26], A0);
        chk("D6 hit load (A0)", dut.cpu[0].core.gr[27], A0);
        chk("D6 miss load (C0)", dut.cpu[0].core.gr[28], C0);
        chk("A1 after D2, D3 (slot-2 store last)", dut.peek(16'h1004), 32'h55);
        chk("A2 after the warm-up store", dut.peek(16'h1008), 32'h55);
        chk("D4 A3: slot-3 byte on top of slot-2 word", dut.peek(16'h100C), 32'h6600_0055);
        chk("D5 B1 after the upgrade", dut.peek(16'h1104), 32'h55);
        chk("MEM2-10 dual commits (D1, D2, D3)", ndual, 3);
        chk("MEM2-10 MEMSEQ cycles (D4, D5, D6)", nseq, 3);
        $display("tb_mem2_sys: D7 %0d dual commits, %0d snoop-held port-2 cycles; D8..D11 %0d dual, %0d MEMSEQ",
                 nd7, nsnoop2, ndual2, nseq2);
        for (j = 0; j < 8; j = j + 1) begin
            chk("D7 E_j word 0 (core 0 round 1)", dut.peek(16'h1400 + j*16), 32'h55);
            chk("D7 E_j word 4 (core 0 round 2)", dut.peek(16'h1404 + j*16), 32'h66);
            chk("D7 E_j word 8 (core 1)", dut.peek(16'h1408 + j*16), 32'h99);
            chk("D7 E_j word 12 (core 0 round 3)", dut.peek(16'h140C + j*16), 32'h77);
        end
        chk("D7 a snoop held a would-be dual port-2 store (reach)", nsnoop2 > 0, 1);
        chk("D8 load of D (fill)", dut.cpu[0].core.gr[47], 32'd0);
        chk("D8 A0 stored on the sequenced path", dut.peek(16'h1000), 32'h77);
        chk("D9 hw_sem free lock reads 0", dut.cpu[0].core.gr[49], 32'd0);
        chk("D9 hw_sem taken lock reads 1", dut.cpu[0].core.gr[51], 32'd1);
        chk("D9 PIU ID read next to a cached load", dut.cpu[0].core.gr[53], 32'h5049_5501);
        chk("D9 cached loads next to MMIO (F1)", {dut.cpu[0].core.gr[48], dut.cpu[0].core.gr[52]}, {32'h55, 32'h55});
        chk("D10 load next to a crossing sth (F1)", dut.cpu[0].core.gr[54], 32'h55);
        chk("D10 crossing sth leaves F1", dut.peek(16'h1304), 32'h55);
        chk("D10 load before the pos-1 sth of its word", dut.cpu[0].core.gr[55], 32'd0);
        chk("D10 pos-1 sth writes bytes 1..2", dut.peek(16'h1308), 32'h0000_7700);
        chk("D11 loads after an xnop", {dut.cpu[0].core.gr[56], dut.cpu[0].core.gr[57]}, {32'h55, 32'h0});
        chk("D8 and the two hw_sem packets of D9 sequenced (MEMSEQ cycles)", nseq2, 3);
        chk("D10 (2 packets) and D11 dual", ndual2, 3);
        chk("PIU MEMSEQ counter = held cycles", dut.cpu[0].core.u_piu.memseq, nseqall);
        chk("D12 slot-2 busy hw_sem ldw = 1", dut.cpu[0].core.gr[58], 32'd1);
        chk("D12 cached load next to it", dut.cpu[0].core.gr[59], 32'h55);
        chk("D12 SEMFAIL D9 s3 + D12 s2 = 2", dut.cpu[0].core.u_piu.semfail, 2);
        chk("D12 sequenced (MMIO, port 1)", nseq3, 1);
        if (fail == 0) $display("RESULT: MEM2 SYS PASSED (%0d checks)", pass);
        else $display("RESULT: MEM2 SYS FAILED (%0d of %0d)", fail, pass + fail);
        $finish;
    end
endmodule
