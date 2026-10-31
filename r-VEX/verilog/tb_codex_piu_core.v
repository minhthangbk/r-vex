// Codex round-10 PIU core-level edge cases, from reviews/contracts/piu.md.
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_codex_piu_core;
    reg clk = 0, reset = 1, start = 0;
    wire done_a, done_b, done_c;
    wire [31:0] cycles_a, cycles_b, cycles_c;
    integer pass = 0, fail = 0, k;

    rvex_core a(.clk(clk), .reset(reset), .start(start), .done(done_a), .cycles(cycles_a));
    rvex_core b(.clk(clk), .reset(reset), .start(start), .done(done_b), .cycles(cycles_b));
    rvex_core #(.PIU(0)) c(.clk(clk), .reset(reset), .start(start), .done(done_c), .cycles(cycles_c));
    always #5 clk = ~clk;

    function [31:0] ri; input [6:0] op; input [5:0] d, s; input [8:0] im;
        begin ri = {op,`SHORT_IMM,d,s,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s, base; input [8:0] off;
        begin mst = {op,`NO_IMM,s,base,off,2'b00}; end endfunction
    function [31:0] j12; input [6:0] op; input [11:0] target;
        begin j12 = {op,8'b0,target,5'b0}; end endfunction
    localparam [31:0] NOP = 0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    task PA; input [11:0] n; input [31:0] s0,s1,s2,s3;
        begin a.imem[n] = {s3,s2,s1,s0}; end endtask
    task PB; input [11:0] n; input [31:0] s0,s1,s2,s3;
        begin b.imem[n] = {s3,s2,s1,s0}; end endtask
    task PC; input [11:0] n; input [31:0] s0,s1,s2,s3;
        begin c.imem[n] = {s3,s2,s1,s0}; end endtask
    task chk; input [255:0] name; input [63:0] got, expected;
        begin
            if (got === expected) begin pass = pass + 1; $display("PASS %0s got=%0d", name, got); end
            else begin fail = fail + 1; $display("FAIL %0s got=%0d expected=%0d", name, got, expected); end
        end
    endtask

    initial begin
        for (k = 0; k < 4096; k = k + 1) begin
            a.imem[k] = 0; b.imem[k] = 0; c.imem[k] = 0;
        end

        // A: byte stores to CTRL lanes 0..2 must preserve EN. Then a taken
        // goto and a CTRL store share one packet: both commit, using old EN.
        PA(0, ri(`ALU_ADD,6'd10,6'd0,9'h7f), ri(`ALU_ADD,6'd11,6'd0,9'haa), NOP, NOP);
        PA(1, ri(`ALU_SHL,6'd10,6'd10,9'd9), NOP, NOP, NOP);
        PA(2, NOP, NOP, NOP, mst(`MEM_STB,6'd11,6'd10,9'd0));
        PA(3, NOP, NOP, NOP, mst(`MEM_STB,6'd11,6'd10,9'd1));
        PA(4, NOP, NOP, NOP, mst(`MEM_STB,6'd11,6'd10,9'd2));
        PA(5, j12(`CTRL_GOTO,12'd7), NOP, NOP, mst(`MEM_STW,6'd0,6'd10,9'd0));
        PA(6, ri(`ALU_ADD,6'd30,6'd0,9'd1), NOP, NOP, NOP);
        PA(7, STOP, NOP, NOP, NOP);

        // B: no syllable in a packet whose slot 0 is STOP commits.
        PB(0, NOP, ri(`ALU_ADD,6'd1,6'd0,9'd1), NOP, NOP);
        PB(1, STOP, ri(`ALU_ADD,6'd2,6'd0,9'd2), ri(`ALU_ADD,6'd3,6'd0,9'd3),
              ri(`ALU_ADD,6'd4,6'd0,9'd4));

        // C: PIU=0 means 0xFE00 is ordinary memory, not PIU MMIO.
        PC(0, ri(`ALU_ADD,6'd10,6'd0,9'h7f), NOP, NOP, NOP);
        PC(1, ri(`ALU_SHL,6'd10,6'd10,9'd9), NOP, NOP, NOP);
        PC(2, NOP, NOP, NOP, ri(`MEM_LDW,6'd20,6'd10,9'd0));
        PC(3, STOP, NOP, NOP, NOP);
        c.dmem[16'hfe00 >> 2] = 32'hdead_beef;

        repeat (2) @(posedge clk);
        reset = 0; start = 1;
        k = 0;
        while (!(done_a && done_b && done_c) && k < 100) begin @(posedge clk); k = k + 1; end
        #1;

        chk("A done", done_a, 1);
        chk("A lane stores preserve EN until word STOP", a.u_piu.pkt, 6);
        chk("A taken branch counted", a.u_piu.brt, 1);
        chk("A control counted", a.u_piu.ctl, 1);
        chk("A stores counted", a.u_piu.st, 4);
        chk("A poison skipped", a.gr[30], 0);
        chk("A final EN", a.u_piu.en, 0);
        chk("B STOP packet excluded", b.u_piu.pkt, 1);
        chk("B STOP companions excluded OPS", b.u_piu.ops, 1);
        chk("B STOP companions no writeback", b.gr[2] | b.gr[3] | b.gr[4], 0);
        chk("B STOP cycle included", b.u_piu.cyc, 2);
        chk("C PIU=0 reads dmem", c.gr[20], 32'hdead_beef);

        $display("CODEX PIU CORE: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: CODEX PIU CORE PASSED");
        else $display("RESULT: CODEX PIU CORE FAILED");
        $finish;
    end
endmodule
