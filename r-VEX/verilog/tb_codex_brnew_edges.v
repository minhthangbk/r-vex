`timescale 1ns/1ps
`include "rvex_defs.vh"

/* Thin BN edge cases not isolated by tb_brnew.v: EXT_IMEM hold, ADDCG whose
 * carry-in and output are the branch BR, a same-packet SLCT reading the old BR,
 * and the PIU taken counter for BRFN. */
module tb_codex_brnew_edges;
    reg clk = 0, reset = 1, start = 0, i_ready = 0;
    reg [2:0] mode = 0;
    reg [127:0] i_rdata;
    wire done, i_req;
    wire [7:0] i_addr;
    wire [31:0] cycles, stall_cycles, istall_cycles;
    integer errors = 0, k;
    localparam [31:0] NOP = 0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    rvex_core #(.PC_W(8), .EXT_IMEM(1)) dut(
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata), .i_ready(i_ready),
        .stall_cycles(stall_cycles), .istall_cycles(istall_cycles));
    always #5 clk = ~clk;

    function [31:0] ri;
        input [6:0] op; input [5:0] d,a; input [8:0] im;
        begin ri = {op,`SHORT_IMM,d,a,im,2'b00}; end
    endfunction
    function [31:0] cmpb;
        input [2:0] b; input [5:0] a,c;
        begin cmpb = {`ALU_CMPEQ,`NO_IMM,6'd0,a,c,b,2'b00}; end
    endfunction
    function [31:0] jbr;
        input [6:0] op; input [11:0] target; input [2:0] b;
        begin jbr = {op,2'b00,5'b0,1'b1,target,b,2'b00}; end
    endfunction
    function [31:0] addcg;
        input [2:0] b; input [5:0] d,a,c;
        begin addcg = {{`ALU_ADDCG4,b},`NO_IMM,d,a,c,b,2'b00}; end
    endfunction
    function [31:0] slct;
        input [2:0] b; input [5:0] d,a,c;
        begin slct = {{`ALU_SLCT4,b},`NO_IMM,d,a,c,3'b000,2'b00}; end
    endfunction

    always @(*) begin
        i_rdata = 128'd0;
        case (i_addr)
          0: case (mode)
               0: i_rdata = {NOP,NOP,cmpb(3'd2,6'd3,6'd4),jbr(`CTRL_BR,12'd3,3'd2)};
               1: i_rdata = {NOP,NOP,addcg(3'd2,6'd11,6'd3,6'd4),jbr(`CTRL_BR,12'd3,3'd2)};
               2: i_rdata = {NOP,slct(3'd2,6'd12,6'd5,6'd6),cmpb(3'd2,6'd3,6'd4),jbr(`CTRL_BR,12'd3,3'd2)};
               default: i_rdata = {NOP,NOP,cmpb(3'd2,6'd3,6'd4),jbr(`CTRL_BRF,12'd3,3'd2)};
             endcase
          1: i_rdata = {NOP,NOP,NOP,ri(`ALU_ADD,6'd10,6'd0,9'd1)};
          2: i_rdata = {NOP,NOP,NOP,STOP};
          3: i_rdata = {NOP,NOP,NOP,ri(`ALU_ADD,6'd10,6'd0,9'd2)};
          4: i_rdata = {NOP,NOP,NOP,STOP};
        endcase
    end

    task begin_case;
        input [2:0] which; input oldb;
        begin
            mode = which; reset = 1; start = 0; i_ready = 0;
            repeat (2) @(posedge clk); reset = 0; #1;
            dut.br[2] = oldb; dut.gr[3] = 32'd9; dut.gr[4] = 32'd9;
            dut.gr[5] = 32'd11; dut.gr[6] = 32'd22; dut.gr[10] = 0; dut.gr[12] = 0;
            start = 1;
        end
    endtask
    task finish_case;
        begin
            k = 0; while (done !== 1'b1 && k < 20) begin @(posedge clk); k = k + 1; end
            start = 0;
            if (!done || dut.gr[10] !== 2) begin
                $display("FAIL mode=%0d done=%b path=%0d",mode,done,dut.gr[10]); errors=errors+1;
            end
        end
    endtask

    initial begin
        /* I-miss: no branch or compare write before the fetched packet commits. */
        begin_case(0, 0);
        repeat (3) begin
            @(posedge clk); #1;
            if (dut.pc !== 0 || dut.br[2] !== 0 || dut.gr[10] !== 0) begin
                $display("FAIL I-hold pc=%0d b2=%b r10=%0d",dut.pc,dut.br[2],dut.gr[10]); errors=errors+1;
            end
        end
        i_ready = 1; finish_case;
        if (istall_cycles < 2) begin $display("FAIL istall=%0d",istall_cycles); errors=errors+1; end

        /* ADDCG reads old b2=0 as carry-in, writes b2=1, and BRN reads new 1. */
        begin_case(1, 0); dut.gr[3] = 32'hffff_ffff; dut.gr[4] = 1; i_ready = 1;
        finish_case;
        if (dut.br[2] !== 1 || dut.gr[11] !== 0) begin
            $display("FAIL addcg b2=%b r11=%h",dut.br[2],dut.gr[11]); errors=errors+1;
        end

        /* SLCT still reads old b2=0 while BRN sees the compare's new b2=1. */
        begin_case(2, 0); i_ready = 1; finish_case;
        if (dut.gr[12] !== 22 || dut.br[2] !== 1) begin
            $display("FAIL slct old-BR r12=%0d b2=%b",dut.gr[12],dut.br[2]); errors=errors+1;
        end

        /* Compare writes zero; BRFN takes on the new zero and PIU counts it. */
        begin_case(3, 1); dut.gr[4] = 10; i_ready = 1; finish_case;
        if (dut.br[2] !== 0 || dut.u_piu.brt !== 1) begin
            $display("FAIL brfn/PIU b2=%b brt=%0d",dut.br[2],dut.u_piu.brt); errors=errors+1;
        end

        if (errors == 0) $display("RESULT: CODEX BRNEW EDGES PASSED");
        else $display("RESULT: CODEX BRNEW EDGES FAILED errors=%0d",errors);
        $finish;
    end
endmodule
