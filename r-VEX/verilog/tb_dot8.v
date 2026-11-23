`timescale 1ns/1ps
`include "rvex_defs.vh"

// tb_dot8.v (round r32 B2, reviews/contracts/simd8.md SD-01..SD-07)
//   part 1: the simd4 unit with q = 1 (the dot8 family) against simd16ref.py vectors8 (simd8_vec.txt)
//   part 2: the core: the eight dot8 forms (0x0F imm type 10) in slots 4-7 on GR quads, accumulate forms read the old
//           rD, a quad element written in the same packet is read old (all reads before writes), A / B fields that
//           are no quad (multiple of 4) and slots 0-3 write nothing, the FP-14 interlock matches a pending FP write
//           to a quad element (one held cycle, the new value read), PIU MUL16 / MULPKT counts, SIMD4 = 0 writes nothing
module tb_dot8;
    integer pass = 0, fail = 0, fd, n, nv = 0, nexp = -1;
    reg [31:0] sub_r, a0, a1, a2, a3, b0, b1, b2, b3, acc, exp_v;
    wire [31:0] res;
    simd4 u(.sub(sub_r[2:0]), .q(1'b1), .a0(a0), .a1(a1), .a2(a2), .a3(a3), .b0(b0), .b1(b1), .b2(b2), .b3(b3),
            .acc(acc), .result(res));
    task chk; input [511:0] nm; input [63:0] got, e; begin
        if (got === e) begin pass = pass + 1; $display("PASS %0s got=%0h", nm, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%0h expected=%0h", nm, got, e); end
    end endtask

    reg clk = 0, reset = 1, start = 0; always #5 clk = ~clk;
    wire done, done0; wire [31:0] cycles, cycles0;
    wire c_dq, c_dq2, c_iq, c0_dq, c0_dq2, c0_iq;
    rvex_core #(.EXT_DMEM(0)) c(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(c_dq), .d_we(), .d_addr(), .d_wdata(), .d_word(), .d_rdata(32'd0), .d_ready(1'b1), .stall_cycles(),
        .d2_req(c_dq2), .d2_rdata(32'd0), .d2_hit(1'b0),
        .i_req(c_iq), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles());
    rvex_core #(.EXT_DMEM(0), .SIMD4(0)) c0(.clk(clk), .reset(reset), .start(start), .done(done0), .cycles(cycles0),
        .d_req(c0_dq), .d_we(), .d_addr(), .d_wdata(), .d_word(), .d_rdata(32'd0), .d_ready(1'b1), .stall_cycles(),
        .d2_req(c0_dq2), .d2_rdata(32'd0), .d2_hit(1'b0),
        .i_req(c0_iq), .i_addr(), .i_rdata(256'd0), .i_ready(1'b1), .istall_cycles());
    // a dot8 op: dst, quad A base, quad B base, sub-op (syl[4:2]); imm type 10
    function [31:0] d8; input [5:0] d, x, y; input [2:0] s; begin d8 = {`SQ_SIMD4, 2'b10, d, x, y, s, 2'b00}; end endfunction
    function [31:0] ri; input [6:0] o; input [5:0] d, x; input [8:0] im; begin ri = {o, `SHORT_IMM, d, x, im, 2'b00}; end endfunction
    function [31:0] rr; input [6:0] o; input [5:0] d, x, y; begin rr = {o, `NO_IMM, d, x, y, 5'b0}; end endfunction
    localparam [31:0] NOP = 0;
    task P; input [11:0] k; input [31:0] s0, s1, s2, s3, s4, s5, s6, s7; begin
        c.imem[k] = {s7, s6, s5, s4, s3, s2, s1, s0}; c0.imem[k] = {s7, s6, s5, s4, s3, s2, s1, s0}; end endtask
    task Q; input [5:0] b; begin   // the quad A8 = (0x00020003, 0x00040005, 0x00010002, 0x00030004) at GR b..b+3
        c.gr[b] = 32'h00020003; c.gr[b + 1] = 32'h00040005; c.gr[b + 2] = 32'h00010002; c.gr[b + 3] = 32'h00030004;
        c0.gr[b] = 32'h00020003; c0.gr[b + 1] = 32'h00040005; c0.gr[b + 2] = 32'h00010002; c0.gr[b + 3] = 32'h00030004;
    end endtask
    integer cyc = 0, bad = 0, fps0;

    initial begin
        // ---- part 1 ----
        fd = $fopen("simd8_vec.txt", "r");
        if (fd == 0) begin $display("FAIL no simd8_vec.txt"); fail = fail + 1; end
        else begin
            while (!$feof(fd)) begin
                n = $fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h\n", sub_r, a0, a1, a2, a3, b0, b1, b2, b3, acc, exp_v);
                if (n == 11) begin
                    #1; nv = nv + 1;
                    if (res !== exp_v) begin
                        bad = bad + 1;
                        if (bad <= 10) $display("FAIL vector sub %0d a %08x:%08x:%08x:%08x b %08x:%08x:%08x:%08x acc %08x got %08x expected %08x",
                                                sub_r[2:0], a0, a1, a2, a3, b0, b1, b2, b3, acc, res, exp_v);
                    end
                end
            end
            $fclose(fd);
            chk("SD-03 unit vectors match simd16ref.py vectors8 (count of mismatches)", bad, 0);
            fd = $fopen("simd8_vec.n", "r");
            if (fd != 0) begin n = $fscanf(fd, "%d", nexp); $fclose(fd); end
            chk("SD unit vectors read (count)", nv == nexp && nexp >= 1000, 1);
        end
        // ---- part 2 ----
        P(0, NOP, NOP, NOP, NOP, d8(10, 16, 20, 0), d8(11, 16, 20, 1), d8(12, 16, 20, 2), d8(13, 16, 20, 3));
        P(1, NOP, NOP, NOP, NOP, d8(14, 16, 20, 4), d8(15, 16, 20, 5), d8(24, 16, 20, 6), d8(25, 16, 20, 7));
        // undefined forms: A field 18 and B field 22 (even, no quad), slots 0-3; slot 6 valid
        P(2, d8(28, 16, 20, 0), d8(29, 16, 20, 0), d8(30, 16, 20, 0), d8(31, 16, 20, 0), d8(26, 18, 20, 0), d8(27, 16, 22, 0),
          d8(32, 16, 20, 0), NOP);
        // a quad element (r47 of A quad 44..47) written in the packet that reads it: read old
        P(3, NOP, ri(`ALU_ADD, 47, 0, 0), NOP, NOP, d8(33, 44, 20, 0), NOP, NOP, NOP);
        // FP-14 / SD-05: fadd r23 (element 3 of B quad 20..23, outside the pair match of the field) = 1.0 + 1.0, then a dot8 reading the quad: held one cycle
        P(4, NOP, NOP, NOP, NOP, rr(`FPU_FADD, 23, 40, 41), NOP, NOP, NOP);
        P(5, NOP, NOP, NOP, NOP, NOP, d8(34, 16, 20, 0), NOP, NOP);
        P(6, {`OP_STOP, 25'd0}, NOP, NOP, NOP, NOP, NOP, NOP, NOP);
        repeat (2) @(posedge clk); reset = 0; #1;
        Q(16); Q(44);
        c.gr[20] = 32'h00050007; c.gr[21] = 32'h00060008; c.gr[22] = 32'h00010001; c.gr[23] = 32'h00020002;
        c0.gr[20] = 32'h00050007; c0.gr[21] = 32'h00060008; c0.gr[22] = 32'h00010001; c0.gr[23] = 32'h00020002;
        c.gr[14] = 32'd100; c.gr[15] = 32'd1000; c.gr[24] = 32'd5; c.gr[25] = 32'd0;
        c.gr[40] = 32'h3F800000; c.gr[41] = 32'h3F800000; c0.gr[40] = 32'h3F800000; c0.gr[41] = 32'h3F800000;
        c.gr[26] = 32'hCAFE; c.gr[27] = 32'hCAFE; c.gr[28] = 32'hCAFE; c.gr[29] = 32'hCAFE; c.gr[30] = 32'hCAFE; c.gr[31] = 32'hCAFE;
        c0.gr[10] = 32'hBEEF;
        fps0 = c.u_piu.fpstall;
        start = 1;
        while (!(done && done0) && cyc < 100) begin @(posedge clk); cyc = cyc + 1; end
        #1;
        chk("core runs end by STOP", done && done0, 1);
        repeat (8) @(posedge clk); #1;
        chk("DRAIN no D / I request 8 cycles after done", {c_dq, c_dq2, c_iq, c0_dq, c0_dq2, c0_iq}, 6'd0);
        chk("SD-03 slot 4 dot8", c.gr[10], 32'h70);
        chk("SD-03 slot 5 dotn8", c.gr[11], 32'hFFFFFFE2);
        chk("SD-03 slot 6 xdot8", c.gr[12], 32'h6C);
        chk("SD-03 slot 7 xdotn8", c.gr[13], 32'h2);
        chk("SD-03 slot 4 dot8a", c.gr[14], 32'hD4);
        chk("SD-03 slot 5 dotn8a", c.gr[15], 32'h3CA);
        chk("SD-03 slot 6 xdot8a", c.gr[24], 32'h71);
        chk("SD-03 slot 7 xdotn8a", c.gr[25], 32'h2);
        chk("SD-02 A field 18 (no quad) writes nothing", c.gr[26], 32'hCAFE);
        chk("SD-02 B field 22 (no quad) writes nothing", c.gr[27], 32'hCAFE);
        chk("SD-02 slot 0 writes nothing", c.gr[28], 32'hCAFE);
        chk("SD-02 slot 1 writes nothing", c.gr[29], 32'hCAFE);
        chk("SD-02 slot 2 writes nothing", c.gr[30], 32'hCAFE);
        chk("SD-02 slot 3 writes nothing", c.gr[31], 32'hCAFE);
        chk("SD-03 slot 6 valid next to undefined forms", c.gr[32], 32'h70);
        chk("SD-04 quad element r47 written in the packet is read old", c.gr[33], 32'h70);
        chk("SD-04 the packet's ALU write lands (r47)", c.gr[47], 32'd0);
        chk("SD-05 dot8 reads the FP result in its B quad", c.gr[34], 32'hC062);
        chk("SD-05 one FP interlock cycle", c.u_piu.fpstall - fps0, 1);
        chk("SD-06 PIU MUL16 counts the 11 defined dot8 ops", c.u_piu.mul16, 11);
        chk("SD-06 PIU MULPKT counts the 5 packets", c.u_piu.mulpkt, 5);
        chk("SD-06 PIU-12 ops = alu + mul16 + mul32 + ld + st + ctrl + fpop", c.u_piu.ops,
            c.u_piu.alu + c.u_piu.mul16 + c.u_piu.mul32 + c.u_piu.ld + c.u_piu.st + c.u_piu.ctl + c.u_piu.fpop);
        chk("SD-02 SIMD4 = 0: the op writes nothing", c0.gr[10], 32'hBEEF);
        if (fail == 0) $display("RESULT: DOT8 PASSED (%0d checks, %0d vectors)", pass, nv);
        else $display("RESULT: DOT8 FAILED (%0d of %0d)", fail, pass + fail);
        $finish;
    end
endmodule
