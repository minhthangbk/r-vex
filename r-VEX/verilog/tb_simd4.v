`timescale 1ns/1ps
`include "rvex_defs.vh"

// tb_simd4.v (round r30 B2, reviews/contracts/simd4.md SQ-02..07)
//   part 1: the simd4 unit against simd16ref.py vectors4 (simd4_vec.txt: corners of every sub-op + random)
//   part 2: the core: the eight forms in slots 4-7, accumulate forms read the old rD, a pair partner written in the
//           same packet is read old (all reads before writes), odd pair fields / imm type != 00 / slots 0-3 write
//           nothing, PIU MUL16 / MULPKT counts, a core with SIMD4 = 0 writes nothing
module tb_simd4;
    integer pass = 0, fail = 0, fd, n, nv = 0, nexp = -1;
    reg [31:0] sub_r, a0, a1, b0, b1, acc, exp_v;
    wire [31:0] res;
    simd4 u(.sub(sub_r[2:0]), .q(1'b0), .a2(32'd0), .a3(32'd0), .b2(32'd0), .b3(32'd0), .a0(a0), .a1(a1), .b0(b0), .b1(b1), .acc(acc), .result(res));
    task chk; input [511:0] nm; input [63:0] got, e; begin
        if (got === e) begin pass = pass + 1; $display("PASS %0s got=%0h", nm, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%0h expected=%0h", nm, got, e); end
    end endtask

    reg clk = 0, reset = 1, start = 0; always #5 clk = ~clk;
    wire done, done0; wire [31:0] cycles, cycles0;
    wire c_dq, c_dq2, c_iq, c0_dq, c0_dq2, c0_iq;   // Codex R58 F6 (RR-11): request ports checked idle after done
    rvex_core #(.EXT_DMEM(0)) c(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(c_dq), .d_we(), .d_addr(), .d_wdata(), .d_word(), .d_rdata(32'd0), .d_ready(1'b1), .stall_cycles(),
        .d2_req(c_dq2), .d2_rdata(32'd0), .d2_hit(1'b0),
        .i_req(c_iq), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles());
    rvex_core #(.EXT_DMEM(0), .SIMD4(0)) c0(.clk(clk), .reset(reset), .start(start), .done(done0), .cycles(cycles0),
        .d_req(c0_dq), .d_we(), .d_addr(), .d_wdata(), .d_word(), .d_rdata(32'd0), .d_ready(1'b1), .stall_cycles(),
        .d2_req(c0_dq2), .d2_rdata(32'd0), .d2_hit(1'b0),
        .i_req(c0_iq), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles());
    // a 4-lane op: dst, pair A base, pair B base, sub-op (syl[4:2]); imm type 00
    function [31:0] q4; input [5:0] d, x, y; input [2:0] s; begin q4 = {`SQ_SIMD4, `NO_IMM, d, x, y, s, 2'b00}; end endfunction
    function [31:0] ri; input [6:0] o; input [5:0] d, x; input [8:0] im; begin ri = {o, `SHORT_IMM, d, x, im, 2'b00}; end endfunction
    localparam [31:0] NOP = 0;
    task P; input [11:0] k; input [31:0] s0, s1, s2, s3, s4, s5, s6, s7; begin
        c.imem[k] = {s7, s6, s5, s4, s3, s2, s1, s0}; c0.imem[k] = {s7, s6, s5, s4, s3, s2, s1, s0}; end endtask
    integer cyc = 0, bad = 0;

    initial begin
        // ---- part 1 ----
        fd = $fopen("simd4_vec.txt", "r");
        if (fd == 0) begin $display("FAIL no simd4_vec.txt"); fail = fail + 1; end
        else begin
            while (!$feof(fd)) begin
                n = $fscanf(fd, "%h %h %h %h %h %h %h\n", sub_r, a0, a1, b0, b1, acc, exp_v);
                if (n == 7) begin
                    #1; nv = nv + 1;
                    if (res !== exp_v) begin
                        bad = bad + 1;
                        if (bad <= 10) $display("FAIL vector sub %0d a %08x:%08x b %08x:%08x acc %08x got %08x expected %08x",
                                                sub_r[2:0], a0, a1, b0, b1, acc, res, exp_v);
                    end
                end
            end
            $fclose(fd);
            chk("SQ-02..04 unit vectors match simd16ref.py vectors4 (count of mismatches)", bad, 0);
            fd = $fopen("simd4_vec.n", "r");   // the count the generator wrote: a truncated vector file fails
            if (fd != 0) begin n = $fscanf(fd, "%d", nexp); $fclose(fd); end
            chk("SQ unit vectors read (count)", nv == nexp && nexp >= 1000, 1);
        end
        // ---- part 2 ----
        P(0, NOP, NOP, NOP, NOP, q4(10, 2, 4, 0), q4(11, 2, 4, 1), q4(12, 2, 4, 2), q4(13, 2, 4, 3));
        P(1, NOP, NOP, NOP, NOP, q4(14, 2, 4, 4), q4(15, 2, 4, 5), q4(16, 2, 4, 6), q4(17, 2, 4, 7));
        // r3 and r5 (pair partners) written in the packet that reads them
        P(2, NOP, ri(`ALU_ADD, 5, 0, 0), ri(`ALU_ADD, 3, 0, 0), NOP, q4(18, 2, 4, 0), q4(19, 4, 2, 0), NOP, NOP);
        // undefined forms: odd A field, imm type 11 sub-op 0 (r32; imm type 01 is the FP family, FP-18), odd B field, slots 0-3; slot 6 valid
        // (Codex R57 F3: slots 1 and 2 too, mutants SQ-M17 / SQ-M18 open their write paths)
        P(3, q4(25, 6, 8, 0), q4(28, 6, 8, 0), q4(29, 6, 8, 0), q4(26, 6, 8, 0), q4(22, 7, 8, 0), {`SQ_SIMD4, 2'b11, 6'd23, 6'd6, 6'd8, 3'd0, 2'b00},   /* r32: imm type 01 is fmadd (fpu.md FP-18); 11 sub-op 0 stays undefined (FP-19) */
          q4(27, 6, 8, 0), q4(24, 6, 9, 0));
        P(4, {`OP_STOP, 25'd0}, NOP, NOP, NOP, NOP, NOP, NOP, NOP);
        repeat (2) @(posedge clk); reset = 0; #1;
        c.gr[2] = 32'h00020003; c.gr[3] = 32'h00040005; c.gr[4] = 32'h00050007; c.gr[5] = 32'h00060008;
        c.gr[6] = 32'h00010001; c.gr[7] = 32'h00000000; c.gr[8] = 32'h00030004; c.gr[9] = 32'h00000002;
        c.gr[14] = 32'd100; c.gr[15] = 32'd1000; c.gr[16] = 32'd5; c.gr[17] = 32'd0;
        c.gr[22] = 32'hCAFE; c.gr[23] = 32'hCAFE; c.gr[24] = 32'hCAFE; c.gr[25] = 32'hCAFE; c.gr[26] = 32'hCAFE; c.gr[28] = 32'hCAFE; c.gr[29] = 32'hCAFE;
        c0.gr[2] = 32'h00020003; c0.gr[3] = 32'h00040005; c0.gr[4] = 32'h00050007; c0.gr[5] = 32'h00060008;
        c0.gr[10] = 32'hBEEF;
        start = 1;
        while (!(done && done0) && cyc < 100) begin @(posedge clk); cyc = cyc + 1; end
        #1;
        if (!(done && done0)) $display("TIMEOUT tb_simd4: no STOP of both cores within 100 cycles (RR-11)");
        chk("core runs end by STOP", done && done0, 1);
        repeat (8) @(posedge clk); #1;
        chk("DRAIN no D / I request 8 cycles after done", {c_dq, c_dq2, c_iq, c0_dq, c0_dq2, c0_iq}, 6'd0);
        chk("SQ-02 slot 4 dot4", c.gr[10], 32'h5F);
        chk("SQ-02 slot 5 dotn4", c.gr[11], 32'hFFFFFFE5);
        chk("SQ-02 slot 6 xdot4", c.gr[12], 32'h5B);
        chk("SQ-02 slot 7 xdotn4", c.gr[13], 32'hFFFFFFFF);
        chk("SQ-03 slot 4 dot4a", c.gr[14], 32'd195);
        chk("SQ-03 slot 5 dotn4a", c.gr[15], 32'd973);
        chk("SQ-03 slot 6 xdot4a", c.gr[16], 32'd96);
        chk("SQ-03 slot 7 xdotn4a", c.gr[17], 32'hFFFFFFFF);
        chk("SQ-06 pair partner r3 written in the packet is read old", c.gr[18], 32'h5F);
        chk("SQ-06 pair partner r5 written in the packet is read old", c.gr[19], 32'h5F);
        chk("SQ-06 the packet's ALU writes land (r3)", c.gr[3], 32'd0);
        chk("SQ-05 odd A field writes nothing", c.gr[22], 32'hCAFE);
        chk("SQ-05 imm type 11 sub-op 0 writes nothing (r32 FP-19)", c.gr[23], 32'hCAFE);
        chk("SQ-05 odd B field writes nothing", c.gr[24], 32'hCAFE);
        chk("SQ-05 slot 0 writes nothing", c.gr[25], 32'hCAFE);
        chk("SQ-05 slot 3 writes nothing", c.gr[26], 32'hCAFE);
        chk("SQ-05 slot 1 writes nothing", c.gr[28], 32'hCAFE);
        chk("SQ-05 slot 2 writes nothing", c.gr[29], 32'hCAFE);
        chk("SQ-02 slot 6 valid next to undefined forms", c.gr[27], 32'd7);   // 1*3 + 1*4 + 0
        chk("SQ-07 PIU MUL16 counts the 11 defined 4-lane ops", c.u_piu.mul16, 11);
        chk("SQ-07 PIU MULPKT counts the 4 packets", c.u_piu.mulpkt, 4);
        chk("SQ-07 PIU-12 ops = alu + mul16 + mul32 + ld + st + ctrl + fpop (r32 PIU-22)", c.u_piu.ops,
            c.u_piu.alu + c.u_piu.mul16 + c.u_piu.mul32 + c.u_piu.ld + c.u_piu.st + c.u_piu.ctl + c.u_piu.fpop);
        chk("SQ-05 SIMD4 = 0: the op writes nothing", c0.gr[10], 32'hBEEF);
        if (fail == 0) $display("RESULT: SIMD4 PASSED (%0d checks, %0d vectors)", pass, nv);
        else $display("RESULT: SIMD4 FAILED (%0d of %0d)", fail, pass + fail);
        $finish;
    end
endmodule
