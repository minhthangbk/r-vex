// tb_fpx_core.v (round r32, fpu.md FP-18..FP-22, piu.md PIU-22): the FP family of opcode 0x0F (imm type 01 / 11,
// sub-op syl[4:2]) through rvex_core, packets driven one per cycle on the external I-port. Expected values come from
// toolchain/sim/f32ref.py (generated with this bench by the r32 K1 script).
//   S1 FP-18 scalar fmadd / fmsub / fmrsub (rD accumulate) in slot 4; S2 FP-18 fused: (1 + 2^-23)^2 - (1 + 2^-22) is
//   2^-46 fused, 0 unfused; S3 FP-20 every pair op (lane-wise and cross-lane) in slots 4-7; S4 FP-21 a reader of the
//   pending high half rD + 1 is interlocked; S5 FP-19 pair rD = 0, an odd pair field, undefined sub-ops (imm type 11
//   sub 0, imm type 10) and the family in slots 0-3 write nothing (Codex batch 3 F2: odd rD / rA / rB, slots 0 / 1 / 2 / 3
//   each a row); S6 FP-17 a higher slot's write of rD + 1 wins;
//   S7 FP-18 a chain of fmadd on one accumulator (RAW on rD) is interlocked; S8 PIU FLOP weights and FPOP (FP-22, PIU-22)
`timescale 1ns/1ps
`include "rvex_defs.vh"
module tb_fpx_core;
    reg clk = 0, reset = 1, start = 0;
    reg [255:0] i_rdata = 0;
    wire done, d_req, d_we, d_word, i_req;
    wire [31:0] cycles, d_addr, d_wdata;
    wire [11:0] i_addr;
    integer pass = 0, fail = 0, k;
    reg [31:0] s0;
    rvex_core #(.EXT_IMEM(1), .EXT_DMEM(1)) dut(
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(d_word), .d_rdata(32'd0),
        .d_ready(1'b1), .stall_cycles(), .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata), .i_ready(1'b1),
        .istall_cycles());
    always #5 clk = ~clk;
    // FP family syllable: {0x0F, imm type, rD, rA, rB, sub-op, 00}
    function [31:0] fx; input [1:0] imt; input [2:0] sub; input [5:0] d, a, b; begin fx = {`SQ_SIMD4, imt, d, a, b, sub, 2'b00}; end endfunction
    function [31:0] ri; input [6:0] op; input [5:0] d, a; input [8:0] im; begin ri = {op, 2'b01, d, a, im, 2'b00}; end endfunction
    function [31:0] rr; input [6:0] op; input [5:0] d, a, b; begin rr = {op, 2'b00, d, a, b, 5'b0}; end endfunction
    function [255:0] pk; input [31:0] s7, s6, s5, s4, s3, s2, s1, s0; begin pk = {s7, s6, s5, s4, s3, s2, s1, s0}; end endfunction
    task chk; input [511:0] name; input [63:0] got, expected; begin
        if (got === expected) begin pass = pass + 1; $display("PASS %0s got=%08x", name, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%08x expected=%08x", name, got, expected); end
    end endtask
    task cyc; begin @(posedge clk); #1; end endtask
    task nop2; begin i_rdata = 256'd0; cyc; cyc; end endtask
    task setop; begin   // operands: pairs r2:r3 (A), r4:r5 (B), accumulators r6:r7 (D)
        dut.gr[2] = 32'h3FC00000; dut.gr[3] = 32'h40100000; dut.gr[4] = 32'h40000000; dut.gr[5] = 32'hBF400000;
        dut.gr[6] = 32'h3E800000; dut.gr[7] = 32'h41200000; end endtask
    initial begin
        repeat (2) @(posedge clk);
        reset = 0; start = 1; #1;
        i_rdata = 256'd0; cyc;
        setop;
        dut.gr[10] = 32'h3E800000; dut.gr[12] = 32'h3E800000; dut.gr[14] = 32'h3E800000;
        i_rdata = pk(fx(2'b01, 3'd2, 6'd14, 6'd2, 6'd4), 32'd0, fx(2'b01, 3'd1, 6'd12, 6'd2, 6'd4), fx(2'b01, 3'd0, 6'd10, 6'd2, 6'd4), 32'd0, 32'd0, 32'd0, 32'd0); cyc;
        chk("FP-18 not written at the commit", dut.gr[10], 32'h3E800000);
        nop2;
        chk("FP-18 fmadd rD + rA*rB (slot 4)", dut.gr[10], 32'h40500000);
        chk("FP-18 fmsub rD - rA*rB (slot 5)", dut.gr[12], 32'hC0300000);
        chk("FP-18 fmrsub rA*rB - rD (slot 7)", dut.gr[14], 32'h40300000);
        dut.gr[16] = 32'h3F800001; dut.gr[17] = 32'hBF800002;
        i_rdata = pk(32'd0, fx(2'b01, 3'd0, 6'd17, 6'd16, 6'd16), 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0); cyc; nop2;
        chk("FP-18 fused: one rounding (unfused gives 00000000)", dut.gr[17], 32'h28800000);
        dut.gr[26] = 32'h3E800000; dut.gr[27] = 32'h41200000;
        i_rdata = pk(fx(2'b01, 3'd6, 6'd26, 6'd2, 6'd4), fx(2'b01, 3'd5, 6'd24, 6'd2, 6'd4), fx(2'b01, 3'd4, 6'd22, 6'd2, 6'd4), fx(2'b01, 3'd3, 6'd20, 6'd2, 6'd4), 32'd0, 32'd0, 32'd0, 32'd0); cyc; nop2;
        chk("FP-20 fadd2 low lane (slot 4)", dut.gr[20], 32'h40600000);
        chk("FP-20 fadd2 high lane", dut.gr[21], 32'h3FC00000);
        chk("FP-20 fsub2 low lane (slot 5)", dut.gr[22], 32'hBF000000);
        chk("FP-20 fsub2 high lane", dut.gr[23], 32'h40400000);
        chk("FP-20 fmul2 low lane (slot 6)", dut.gr[24], 32'h40400000);
        chk("FP-20 fmul2 high lane", dut.gr[25], 32'hBFD80000);
        chk("FP-20 fmadd2 low lane (slot 7)", dut.gr[26], 32'h40500000);
        chk("FP-20 fmadd2 high lane", dut.gr[27], 32'h41050000);
        dut.gr[28] = 32'h3E800000; dut.gr[29] = 32'h41200000;
        i_rdata = pk(fx(2'b11, 3'd6, 6'd34, 6'd2, 6'd4), fx(2'b11, 3'd5, 6'd32, 6'd2, 6'd4), fx(2'b11, 3'd4, 6'd30, 6'd2, 6'd4), fx(2'b01, 3'd7, 6'd28, 6'd2, 6'd4), 32'd0, 32'd0, 32'd0, 32'd0); cyc; nop2;
        chk("FP-20 fmsub2 low lane (slot 4)", dut.gr[28], 32'hC0300000);
        chk("FP-20 fmsub2 high lane", dut.gr[29], 32'h413B0000);
        chk("FP-20 fmul2x low lane (slot 5)", dut.gr[30], 32'hBF900000);
        chk("FP-20 fmul2x high lane", dut.gr[31], 32'h40900000);
        chk("FP-20 fhsa2 low lane (slot 6)", dut.gr[32], 32'hBF400000);
        chk("FP-20 fhsa2 high lane", dut.gr[33], 32'h3FA00000);
        chk("FP-20 faddsub2 low lane (slot 7)", dut.gr[34], 32'hBF000000);
        chk("FP-20 faddsub2 high lane", dut.gr[35], 32'h3FC00000);
        s0 = dut.u_piu.fpstall;
        i_rdata = pk(32'd0, 32'd0, 32'd0, fx(2'b01, 3'd3, 6'd40, 6'd2, 6'd4), 32'd0, 32'd0, 32'd0, 32'd0); cyc;
        i_rdata = pk(32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, rr(`ALU_ADD, 6'd42, 6'd41, 6'd0)); cyc; cyc;
        i_rdata = 256'd0; cyc;
        chk("FP-21 a reader of the pending high half is interlocked one cycle", dut.u_piu.fpstall, s0 + 1);
        chk("FP-21 the reader sees the high lane", dut.gr[42], 32'h3FC00000);
        dut.gr[1] = 32'h00008000; dut.gr[44] = 32'h11111111; dut.gr[45] = 32'h22222222; dut.gr[46] = 32'h33333333; dut.gr[47] = 32'h44444444; dut.gr[48] = 32'h55555555; dut.gr[49] = 32'h66666666;
        i_rdata = pk(fx(2'b11, 3'd0, 6'd46, 6'd2, 6'd4), fx(2'b01, 3'd3, 6'd44, 6'd3, 6'd4), fx(2'b01, 3'd3, 6'd0, 6'd2, 6'd4), fx(2'b10, 3'd0, 6'd48, 6'd2, 6'd4), 32'd0, fx(2'b01, 3'd0, 6'd49, 6'd2, 6'd4), 32'd0, 32'd0); cyc; nop2;
        chk("FP-19 pair rD = 0 writes nothing (SP kept)", dut.gr[1], 32'h00008000);
        chk("FP-19 an odd pair field writes nothing (low)", dut.gr[44], 32'h11111111);
        chk("FP-19 an odd pair field writes nothing (high)", dut.gr[45], 32'h22222222);
        chk("FP-19 imm type 11 sub-op 0 writes nothing", dut.gr[46], 32'h33333333);
        chk("FP-19 imm type 10 writes nothing (B2 dot8 later)", dut.gr[48], 32'h55555555);
        chk("FP-19 the family in slot 2 writes nothing", dut.gr[49], 32'h66666666);
        // Codex batch 3 F2 (r32): one row per FP-19 guard: odd rD (slot 4), odd rB (slot 6), the family in slots 0 / 1 / 3
        dut.gr[53] = 32'h77777777; dut.gr[36] = 32'h88888888; dut.gr[37] = 32'h99999999;
        dut.gr[54] = 32'hAAAAAAAA; dut.gr[55] = 32'hAAAAAAA1; dut.gr[56] = 32'hBBBBBBBB; dut.gr[57] = 32'hBBBBBBB1;
        dut.gr[58] = 32'hCCCCCCCC; dut.gr[59] = 32'hCCCCCCC1;
        i_rdata = pk(32'd0, fx(2'b01, 3'd3, 6'd36, 6'd2, 6'd5), 32'd0, fx(2'b01, 3'd3, 6'd53, 6'd2, 6'd4),
                     fx(2'b01, 3'd3, 6'd58, 6'd2, 6'd4), 32'd0, fx(2'b01, 3'd3, 6'd56, 6'd2, 6'd4), fx(2'b01, 3'd3, 6'd54, 6'd2, 6'd4)); cyc; nop2;
        chk("FP-19 an odd rD field writes nothing (slot 4)", dut.gr[53], 32'h77777777);
        chk("FP-19 an odd rB field writes nothing (low, slot 6)", dut.gr[36], 32'h88888888);
        chk("FP-19 an odd rB field writes nothing (high)", dut.gr[37], 32'h99999999);
        chk("FP-19 the family in slot 0 writes nothing (low)", dut.gr[54], 32'hAAAAAAAA);
        chk("FP-19 the family in slot 0 writes nothing (high)", dut.gr[55], 32'hAAAAAAA1);
        chk("FP-19 the family in slot 1 writes nothing (low)", dut.gr[56], 32'hBBBBBBBB);
        chk("FP-19 the family in slot 1 writes nothing (high)", dut.gr[57], 32'hBBBBBBB1);
        chk("FP-19 the family in slot 3 writes nothing (low)", dut.gr[58], 32'hCCCCCCCC);
        chk("FP-19 the family in slot 3 writes nothing (high)", dut.gr[59], 32'hCCCCCCC1);
        i_rdata = pk(ri(`ALU_ADD, 6'd51, 6'd0, 9'd7), 32'd0, 32'd0, fx(2'b01, 3'd3, 6'd50, 6'd2, 6'd4), 32'd0, 32'd0, 32'd0, 32'd0); cyc; nop2;
        chk("FP-17 / FP-21 slot 7 ALU write of rD + 1 wins", dut.gr[51], 32'd7);
        chk("FP-21 the low lane of that pair is written", dut.gr[50], 32'h40600000);
        dut.gr[52] = 32'h3E800000; s0 = dut.u_piu.fpstall;
        i_rdata = pk(32'd0, 32'd0, 32'd0, fx(2'b01, 3'd0, 6'd52, 6'd2, 6'd4), 32'd0, 32'd0, 32'd0, 32'd0); cyc; cyc; cyc; i_rdata = 256'd0; cyc; cyc;
        chk("FP-18 fmadd chain on rD: the second op reads the first result", dut.gr[52], 32'h40C80000);
        chk("FP-14 the chain is interlocked one cycle", dut.u_piu.fpstall, s0 + 1);
        chk("FP-22 PIU FLOP weights (1 scalar FPU, 2 FMA / pair, 4 fmadd2 / fmsub2)", dut.u_piu.flop, 38);   // + the pair op with rD = 0 of S5 (an op that writes nothing still counts)
        chk("PIU-22 FPOP counts the FP ops", dut.u_piu.fpop, 17);
        i_rdata = {224'd0, `OP_STOP, 25'd0};
        k = 0;
        while (!done && k < 20) begin @(posedge clk); k = k + 1; end
        #1;
        if (!done) $display("TIMEOUT tb_fpx_core: no STOP after 20 cycles");
        chk("drain: STOP ends the core", done, 1'b1);
        repeat (2) @(posedge clk); #1;
        chk("drain: no fetch after STOP", i_req, 1'b0);
        chk("drain: no data access after STOP", d_req, 1'b0);
        $display("FPX CORE: %0d passed, %0d failed", pass, fail);
        $display("RESULT: FPX CORE %s", fail == 0 ? "PASSED" : "FAILED");
        $finish;
    end
endmodule
