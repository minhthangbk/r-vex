// Two cores commit different CTRL values on the same edge (PIU-14).
`timescale 1ns/1ps
`include "rvex_defs.vh"
module tb_codex_piu_dualctrl;
    reg clk = 0, reset = 1, start = 0;
    wire done0, done1; wire [31:0] cycles0, cycles1;
    integer k, simultaneous = 0, fail = 0;
    rvex_core c0(.clk(clk), .reset(reset), .start(start), .done(done0), .cycles(cycles0));
    rvex_core c1(.clk(clk), .reset(reset), .start(start), .done(done1), .cycles(cycles1));
    always #5 clk = ~clk;
    function [31:0] ri; input [6:0] op; input [5:0] d,s; input [8:0] im;
        begin ri={op,`SHORT_IMM,d,s,im,2'b0}; end endfunction
    function [31:0] st; input [5:0] s,b; begin
        st={`MEM_STW,`NO_IMM,s,b,9'd0,2'b0}; end endfunction
    localparam [31:0] NOP=0, STOP={`OP_STOP,25'd0};
    always @(posedge clk)
        if (c0.u_piu.we && c1.u_piu.we) simultaneous = simultaneous + 1;
    initial begin
        for (k=0;k<4096;k=k+1) begin c0.imem[k]=0; c1.imem[k]=0; end
        c0.imem[0]={NOP,NOP,NOP,ri(`ALU_ADD,6'd10,6'd0,9'h7f)};
        c1.imem[0]={NOP,NOP,ri(`ALU_ADD,6'd11,6'd0,9'd3),ri(`ALU_ADD,6'd10,6'd0,9'h7f)};
        c0.imem[1]={NOP,NOP,NOP,ri(`ALU_SHL,6'd10,6'd10,9'd9)};
        c1.imem[1]={NOP,NOP,NOP,ri(`ALU_SHL,6'd10,6'd10,9'd9)};
        c0.imem[2]={st(6'd0,6'd10),NOP,NOP,NOP};
        c1.imem[2]={st(6'd11,6'd10),NOP,NOP,NOP};
        c0.imem[3]={NOP,NOP,NOP,STOP}; c1.imem[3]={NOP,NOP,NOP,STOP};
        repeat(2) @(posedge clk);
        reset=0; start=1;
        k=0; while (!(done0 && done1) && k<30) begin @(posedge clk); k=k+1; end
        #1;
        if (simultaneous != 1) begin fail=fail+1; $display("FAIL simultaneous=%0d",simultaneous); end
        if (c0.u_piu.en != 0 || c0.u_piu.pkt != 3) begin fail=fail+1; $display("FAIL core0 en/pkt"); end
        if (c1.u_piu.en != 1 || c1.u_piu.pkt != 0 || c1.u_piu.cyc != 1) begin
            fail=fail+1; $display("FAIL core1 en/pkt/cycle");
        end
        $display("CODEX PIU DUALCTRL: %0d failed",fail);
        if (!fail) $display("RESULT: CODEX PIU DUALCTRL PASSED");
        $finish;
    end
endmodule
