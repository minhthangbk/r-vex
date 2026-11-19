`timescale 1ns/1ps
`include "rvex_defs.vh"
`include "slot8.vh"

// tb_simd16.v (round r28, reviews/contracts/simd16.md SM-01..06)
//   part 1: the simd16 unit against simd16ref.py vectors (simd16_vec.txt: corners of every op + random)
//   part 2: the core: the ops in slots 1 and 2, accumulate forms read the old rD (all reads before writes), the ops in
//           slots 0 and 3 write nothing, PIU MUL16 / ALU counts
module tb_simd16;
    integer pass = 0, fail = 0, fd, n, nv = 0, nexp = -1;
    reg [31:0] op_r, a, b, acc, exp_v;
    wire [31:0] res; wire val;
    simd16 u(.op(op_r[6:0]), .a(a), .b(b), .acc(acc), .result(res), .out_valid(val));
    task chk; input [511:0] nm; input [63:0] got, e; begin
        if (got === e) begin pass = pass + 1; $display("PASS %0s got=%0h", nm, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%0h expected=%0h", nm, got, e); end
    end endtask

    reg clk = 0, reset = 1, start = 0; always #5 clk = ~clk;
    wire done; wire [31:0] cycles;
    rvex_core #(.EXT_DMEM(0)) c(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(), .d_we(), .d_addr(), .d_wdata(), .d_word(), .d_rdata(32'd0), .d_ready(1'b1), .stall_cycles(),
        .i_req(), .i_addr(), .i_rdata(128'd0), .i_ready(1'b1), .istall_cycles());
    function [31:0] rr; input [6:0] o; input [5:0] d, x, y; begin rr = {o, `NO_IMM, d, x, y, 5'b0}; end endfunction
    function [31:0] ri; input [6:0] o; input [5:0] d, x; input [8:0] im; begin ri = {o, `SHORT_IMM, d, x, im, 2'b00}; end endfunction
    localparam [31:0] NOP = 0;
    task P; input [11:0] k; input [31:0] s0, s1, s2, s3; begin c.imem[k] = pk8(s3, s2, s1, s0); end endtask
    integer cyc = 0, bad = 0;

    initial begin
        // ---- part 1 ----
        fd = $fopen("simd16_vec.txt", "r");
        if (fd == 0) begin $display("FAIL no simd16_vec.txt"); fail = fail + 1; end
        else begin
            while (!$feof(fd)) begin
                n = $fscanf(fd, "%h %h %h %h %h\n", op_r, a, b, acc, exp_v);
                if (n == 5) begin
                    #1; nv = nv + 1;
                    if (res !== exp_v || val !== 1'b1) begin
                        bad = bad + 1;
                        if (bad <= 10) $display("FAIL vector op %02x a %08x b %08x acc %08x got %08x expected %08x", op_r[6:0], a, b, acc, res, exp_v);
                    end
                end
            end
            $fclose(fd);
            chk("SM-01..05 unit vectors match simd16ref.py (count of mismatches)", bad, 0);
            // final verify r28 F9 (BUG-355): exactly the count the generator wrote (run_sim_all.sh / mutate.py write
            // simd16_vec.n), so a truncated vector file fails
            fd = $fopen("simd16_vec.n", "r");
            if (fd != 0) begin n = $fscanf(fd, "%d", nexp); $fclose(fd); end
            chk("SM unit vectors read (count)", nv == nexp && nexp >= 1000, 1);
        end
        op_r = 32'h41; #1; chk("SM-07 an ALU code is not a simd16 op", val, 0);
        // ---- part 2 ----
        P(0, NOP, rr(`SM_DOT2, 10, 2, 3), rr(`SM_DOTN2, 11, 2, 3), NOP);
        P(1, NOP, rr(`SM_DOT2A, 12, 2, 3), rr(`SM_XDOT2A, 13, 2, 3), NOP);
        P(2, NOP, rr(`SM_XDOTN2, 14, 2, 3), rr(`SM_XDOTN2A, 15, 2, 3), NOP);
        P(3, ri(`ALU_ADD, 18, 17, 0), rr(`SM_DOTN2A, 17, 2, 3), rr(`SM_XDOT2, 19, 2, 3), NOP);   // r18 = old r17
        P(4, rr(`SM_DOT2, 20, 2, 3), NOP, NOP, rr(`SM_DOT2, 21, 2, 3));                          // slots 0 / 3: nothing
        P(5, {`OP_STOP, 25'd0}, NOP, NOP, NOP);
        repeat (2) @(posedge clk); reset = 0; #1;
        c.gr[2] = 32'h00020003; c.gr[3] = 32'h00050007;
        c.gr[12] = 32'd100; c.gr[13] = 32'hFFFFFFFF; c.gr[15] = 32'd5; c.gr[17] = 32'd1000;
        c.gr[20] = 32'hCAFE; c.gr[21] = 32'hBEEF;
        start = 1;
        while (!done && cyc < 100) begin @(posedge clk); cyc = cyc + 1; end
        #1;
        chk("core run ends by STOP", done, 1);
        chk("SM-01 slot 1 dot2", c.gr[10], 32'd31);                    // 2*5 + 3*7
        chk("SM-02 slot 2 dotn2", c.gr[11], 32'hFFFFFFF5);             // 10 - 21
        chk("SM-05 slot 1 dot2a", c.gr[12], 32'd131);
        chk("SM-05 slot 2 xdot2a (wrap)", c.gr[13], 32'd28);           // -1 + 14 + 15
        chk("SM-04 slot 1 xdotn2", c.gr[14], 32'd1);                   // 3*5 - 2*7
        chk("SM-05 slot 2 xdotn2a", c.gr[15], 32'd6);
        chk("SM-05 dotn2a accumulates", c.gr[17], 32'd989);            // 1000 + 10 - 21
        chk("SM-05 all reads before writes: slot 0 reads the old rD", c.gr[18], 32'd1000);
        chk("SM-03 slot 2 xdot2", c.gr[19], 32'd29);
        chk("SM-06 slot 0 op writes nothing", c.gr[20], 32'hCAFE);
        chk("SM-06 slot 3 op writes nothing", c.gr[21], 32'hBEEF);
        chk("SM-06 PIU MUL16 counts the 8 slot-1/2 ops", c.u_piu.mul16, 8);
        chk("SM-06 PIU MULPKT counts the 4 packets", c.u_piu.mulpkt, 4);
        chk("SM-06 PIU-12 ops = alu + mul16 + mul32 + ld + st + ctrl + flop", c.u_piu.ops,
            c.u_piu.alu + c.u_piu.mul16 + c.u_piu.mul32 + c.u_piu.ld + c.u_piu.st + c.u_piu.ctl + c.u_piu.flop);
        if (fail == 0) $display("RESULT: SIMD16 PASSED (%0d checks, %0d vectors)", pass, nv);
        else $display("RESULT: SIMD16 FAILED (%0d of %0d)", fail, pass + fail);
        $finish;
    end
endmodule
