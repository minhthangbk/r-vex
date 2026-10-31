`timescale 1ns/1ps
`include "rvex_defs.vh"

/* A packet reads the old BR value for control while another slot writes that
 * BR.  This is a legal WAR: packet reads precede its nonblocking writes. */
module tb_codex_cmpbr_oldbr;
    reg clk = 0, reset = 1, start = 0;
    wire done; wire [31:0] cycles;
    integer pass = 0, fail = 0, k;
    localparam [31:0] NOP = 0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    rvex_core dut(.clk(clk), .reset(reset), .start(start),
                  .done(done), .cycles(cycles));
    always #5 clk = ~clk;

    function [31:0] bi;
        input [6:0] op; input [2:0] b; input [5:0] a; input [8:0] im;
        begin bi = {op,2'b11,3'b000,b,a,im,2'b00}; end
    endfunction
    function [31:0] jbr;
        input [6:0] op; input [11:0] off; input [2:0] b;
        begin jbr = {op,8'b0,off,b,2'b0}; end
    endfunction
    function [31:0] ri;
        input [6:0] op; input [5:0] d,a; input [8:0] im;
        begin ri = {op,`SHORT_IMM,d,a,im,2'b00}; end
    endfunction
    task chk;
        input [127:0] name; input [31:0] got, exp;
        begin
            if (got === exp) pass = pass + 1;
            else begin fail = fail + 1;
                $display("FAIL %0s got=%0h exp=%0h", name, got, exp);
            end
        end
    endtask

    initial begin
        for (k = 0; k < 4096; k = k + 1) dut.imem[k] = 128'd0;
        /* Same r3 is compared with two constants in one packet. */
        dut.imem[0] = {NOP, NOP,
                       bi(`ALU_CMPLT, 3'd2, 6'd3, 9'd100),
                       bi(`ALU_CMPGT, 3'd1, 6'd3, 9'd200)};
        /* b2 is old-true here; slot 1 concurrently changes it to false. */
        dut.imem[1] = {NOP, NOP,
                       bi(`ALU_CMPGT, 3'd2, 6'd3, 9'd200),
                       jbr(`CTRL_BR, 12'd3, 3'd2)};
        dut.imem[2] = {NOP, NOP, NOP,
                       ri(`ALU_ADD, 6'd10, 6'd0, 9'd1)};
        dut.imem[3] = {NOP, NOP, NOP, STOP};

        repeat (2) @(posedge clk);
        reset = 0;
        @(posedge clk);
        dut.gr[3] = 32'd50;
        start = 1;
        k = 0;
        while (done !== 1'b1 && k < 20) begin @(posedge clk); k = k + 1; end
        chk("STOP", done, 1);
        chk("old BR branch", dut.gr[10], 0);
        chk("new b2 false", dut.br[2], 0);
        chk("other b1 false", dut.br[1], 0);
        $display("CODEX CMPBR OLD-BR: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: CODEX CMPBR OLD-BR PASSED");
        else $display("RESULT: CODEX CMPBR OLD-BR FAILED");
        $finish;
    end
endmodule
