// tb_fpu_core.v (round r27, fpu.md FP-01/02/05/07/08, piu.md PIU-20): the FPU through rvex_core.
//   slot 1 and slot 2 ops in one packet, a dependent op in the next packet (latency 1), the immediate form,
//   FPU codes in slots 0 and 3 write nothing (undefined ops, FP-08; slot 3 wrote 0 before BUG-320), code 15 stays ALU for the PIU, FLOP counts,
//   and the same packets on an FPU=0 core write nothing (BUG-320; the r26 core wrote 0).
//   r31 FP-13 / FP-14 / PIU-21 (the r27 FP-12 hold is gone): an FP packet commits at once and its results land at the next
//   edge; the test drives each FP packet for two cycles, so the second is an interlock on its own pending destinations
//   (FPSTALL + 1 per FP packet; the FPU=0 core runs it twice and writes nothing).
`timescale 1ns/1ps
`include "rvex_defs.vh"
`include "slot8.vh"

module tb_fpu_core;
    reg clk = 0, reset = 1, start = 0;
    reg [255:0] i_rdata = 0;
    wire done, d_req, d_we, d_word, i_req, done0, d_req0, d_we0, d_word0, i_req0;
    wire [31:0] cycles, d_addr, d_wdata, cycles0, d_addr0, d_wdata0;
    wire [11:0] i_addr, i_addr0;
    integer pass = 0, fail = 0; reg [11:0] pc0;

    rvex_core #(.EXT_IMEM(1), .EXT_DMEM(1)) dut(
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(d_word), .d_rdata(32'd0),
        .d_ready(1'b1), .stall_cycles(), .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata), .i_ready(1'b1),
        .istall_cycles());
    rvex_core #(.EXT_IMEM(1), .EXT_DMEM(1), .FPU(0)) nofpu(
        .clk(clk), .reset(reset), .start(start), .done(done0), .cycles(cycles0),
        .d_req(d_req0), .d_we(d_we0), .d_addr(d_addr0), .d_wdata(d_wdata0), .d_word(d_word0), .d_rdata(32'd0),
        .d_ready(1'b1), .stall_cycles(), .i_req(i_req0), .i_addr(i_addr0), .i_rdata(i_rdata), .i_ready(1'b1),
        .istall_cycles());
    always #5 clk = ~clk;

    function [31:0] rr; input [6:0] op; input [5:0] d, a, b; begin rr = {op, 2'b00, d, a, b, 5'b0}; end endfunction
    function [31:0] ri; input [6:0] op; input [5:0] d, a; input [8:0] im; begin ri = {op, 2'b01, d, a, im, 2'b00}; end endfunction
    task chk; input [511:0] name; input [63:0] got, expected; begin
        if (got === expected) begin pass = pass + 1; $display("PASS %0s got=%08x", name, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%08x expected=%08x", name, got, expected); end
    end endtask

    initial begin
        repeat (2) @(posedge clk);
        reset = 0; start = 1; #1;
        i_rdata = 256'd0;                                        // one NOP packet: the core runs from the next edge
        @(posedge clk); #1;
        dut.gr[4] = 32'h3FC00000; dut.gr[5] = 32'h40100000;      // 1.5, 2.25
        nofpu.gr[4] = 32'h3FC00000; nofpu.gr[5] = 32'h40100000; nofpu.gr[6] = 32'h12345678; nofpu.gr[7] = 32'h12345678;
        // packet 1: slot 1 fadd r6 = 1.5 + 2.25 = 3.75, slot 2 fmul r7 = 1.5 * 2.25 = 3.375
        i_rdata = pk8(32'd0, rr(`FPU_FMUL, 6'd7, 6'd4, 6'd5), rr(`FPU_FADD, 6'd6, 6'd4, 6'd5), 32'd0);
        pc0 = i_addr;
        @(posedge clk); #1;
        // r31 FP-13 (supersedes the FP-12 hold): the FP packet commits in its first cycle, its results land at the
        // next edge; the bench drives it a second cycle, where its own destinations are pending: one FP-14 interlock
        chk("FP-13 the result is not written at the commit", dut.gr[6], 32'h0);
        chk("FP-13 no hold: the PC moves on", i_addr, pc0 + 12'd1);
        chk("FP-13 no hold: FPSTALL 0 after the commit cycle", dut.u_piu.fpstall, 0);
        chk("FP-13 the packet committed in its first cycle", dut.u_piu.pkt, 1);
        @(posedge clk); #1;
        chk("FP-01 slot 1 fadd", dut.gr[6], 32'h40700000);
        chk("FP-01 slot 2 fmul", dut.gr[7], 32'h40580000);
        // packet 2 (latency 1): slot 2 fsub r8 = r6 - r7 = 0.375, slot 1 fsub r9 = r7 - r6 = -0.375
        i_rdata = pk8(32'd0, rr(`FPU_FSUB, 6'd8, 6'd6, 6'd7), rr(`FPU_FSUB, 6'd9, 6'd7, 6'd6), 32'd0);
        @(posedge clk); #1; @(posedge clk); #1;
        chk("FP-05 next packet reads the result, slot 2 fsub", dut.gr[8], 32'h3EC00000);
        chk("FP-01 fsub operand order, slot 1", dut.gr[9], 32'hBEC00000);
        // packet 3: immediate form (the 9-bit field as a pattern: 0x1 = the smallest subnormal) and dst 0
        i_rdata = pk8(32'd0, rr(`FPU_FADD, 6'd0, 6'd4, 6'd5), ri(`FPU_FMUL, 6'd10, 6'd4, 9'd1), 32'd0);
        @(posedge clk); #1; @(posedge clk); #1;
        chk("FP-01 immediate operand pattern", dut.gr[10], 32'h00000002);   // 1.5 * 2^-149 = 1.5 ulp -> 2 (ties to even)
        chk("FP-01 dst 0 writes nothing", dut.gr[0], 32'h0);
        // packet 4: FPU codes in slot 0 and slot 3 (undefined there, FP-08: as before r27), code 15 in slot 1 (ALU)
        dut.gr[11] = 32'h12345678; dut.gr[12] = 32'h12345678; dut.gr[13] = 32'h12345678;
        i_rdata = pk8(rr(`FPU_FADD, 6'd12, 6'd4, 6'd5), 32'd0, rr(7'd15, 6'd13, 6'd4, 6'd5), rr(`FPU_FADD, 6'd11, 6'd4, 6'd5));
        @(posedge clk); #1;
        chk("FP-08 FADD code in slot 0 writes nothing (as before r27)", dut.gr[11], 32'h12345678);
        chk("FP-08 / BUG-320 FADD code in slot 3 writes nothing", dut.gr[12], 32'h12345678);
        chk("FP-08 / BUG-320 code 15 in slot 1 writes nothing", dut.gr[13], 32'h12345678);
        // PIU-20 / FP-07: 2 + 2 + 2 FPU ops in slots 1-2 so far; code 15 and the slot 0 / 3 codes are not FLOPs
        chk("PIU-20 FLOP counts slot 1-2 FPU ops", dut.u_piu.flop, 6);
        chk("PIU-21 FPSTALL = 3 FP packets", dut.u_piu.fpstall, 3);
        chk("PIU-21 packet 4 (no slot 1-2 FP op) is not held", dut.u_piu.pkt, 4);
        chk("PIU-12 OPS = ALU + ... + FLOP", dut.u_piu.ops, dut.u_piu.alu + dut.u_piu.mul16 + dut.u_piu.mul32 +
            dut.u_piu.ld + dut.u_piu.st + dut.u_piu.ctl + dut.u_piu.flop);
        // FPU = 0: no FPU, the same packets write nothing (BUG-320; the r26 core wrote 0) and count no FLOP
        chk("FP-05 / BUG-320 FPU=0 fadd writes nothing", nofpu.gr[6], 32'h12345678);
        chk("FP-05 / BUG-320 FPU=0 fmul writes nothing", nofpu.gr[7], 32'h12345678);
        chk("FP-05 FPU=0 no FLOP", nofpu.u_piu.flop, 0);
        // drain (rule RR-11, Codex R54 F7): STOP ends both cores within a cap, then no I or D request stays up
        i_rdata = {96'd0, `OP_STOP, 25'd0};
        repeat (4) begin if (!(done && done0)) @(posedge clk); end
        #1;
        chk("drain: STOP ends both cores (cap 4 cycles)", {done, done0}, 2'b11);
        chk("drain: no I or D request after STOP", {i_req, d_req, i_req0, d_req0}, 4'b0000);
        $display("FPU CORE: %0d passed, %0d failed", pass, fail);
        $display("RESULT: FPU CORE %s", fail == 0 ? "PASSED" : "FAILED");
        $finish;
    end
endmodule
