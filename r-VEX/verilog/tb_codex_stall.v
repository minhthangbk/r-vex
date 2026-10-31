`timescale 1ns/1ps
`include "rvex_defs.vh"

// Directed audit of the external-memory hold contract.  A packet containing
// GR writes, a BR carry write, and a byte store is held on a D miss; XNOP and
// STOP are then held on their respective internal/I-side stall paths.
module tb_codex_stall;
    reg clk = 0, reset = 1, start = 0;
    reg [127:0] i_rdata;
    reg i_ready = 0, d_ready = 0;
    reg [31:0] d_rdata = 32'hdeadbeef;
    wire done, d_req, d_we, i_req;
    wire [31:0] cycles, d_addr, d_wdata, stall_cycles, istall_cycles;
    wire [7:0] i_addr;
    integer errors = 0, k;

    rvex_core #(.PC_W(8), .DMEM_AW(16), .EXT_DMEM(1), .EXT_IMEM(1)) dut (
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata),
        .d_rdata(d_rdata), .d_ready(d_ready), .stall_cycles(stall_cycles),
        .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata), .i_ready(i_ready),
        .istall_cycles(istall_cycles));
    always #5 clk = ~clk;

    function [31:0] ri; input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s,b; input [8:0] off; begin
        mst = {op,`NO_IMM,s,b,off,2'b00}; end endfunction
    function [31:0] addcg; input [5:0] d,a,b; input [2:0] sb,db; begin
        addcg = {{`ALU_ADDCG4,sb},`NO_IMM,d,a,b,db,2'b00}; end endfunction
    function [31:0] j12; input [6:0] op; input [11:0] n; begin
        j12 = {op,8'b0,n,5'b0}; end endfunction
    localparam [31:0] NOP = 0;

    always @(*) begin
        case (i_addr)
          0: i_rdata = {mst(`MEM_STB,6'd2,6'd0,9'd1), NOP,
                         ri(`ALU_ADD,6'd6,6'd0,9'd7),
                         addcg(6'd5,6'd2,6'd2,3'd0,3'd7)};
          1: i_rdata = {NOP,NOP,NOP,j12(`CTRL_XNOP,12'd2)};
          2: i_rdata = {NOP,NOP,NOP,{`OP_STOP,25'd0}};
          default: i_rdata = 0;
        endcase
    end

    task expect_hold;
        input [7:0] exp_pc;
        input [31:0] exp5, exp6;
        input expb7, expdone;
        begin
            if (dut.pc !== exp_pc || dut.gr[5] !== exp5 || dut.gr[6] !== exp6 ||
                dut.br[7] !== expb7 || done !== expdone) begin
                $display("HOLD FAIL pc=%0d r5=%h r6=%h b7=%b done=%b",
                         dut.pc, dut.gr[5], dut.gr[6], dut.br[7], done);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        repeat (2) @(posedge clk);
        reset = 0;
        #1; // let the final reset-edge nonblocking assignments settle
        // Seed visible state so an accidental packet issue is unmistakable.
        dut.gr[2] = 32'h000000aa;
        dut.gr[5] = 32'h55555555;
        dut.gr[6] = 32'h66666666;
        dut.br[0] = 1'b1;
        dut.br[7] = 1'b1;
        start = 1;

        // I-side miss at PC 0: nothing except cycle/stat counters may move.
        repeat (3) begin @(posedge clk); #1; expect_hold(0,32'h55555555,32'h66666666,1,0); end
        i_ready = 1;
        #1;
        if (!d_req || !d_we) begin $display("D request missing"); errors = errors + 1; end

        // D-side miss: the entire mixed packet must remain uncommitted.
        repeat (3) begin @(posedge clk); #1; expect_hold(0,32'h55555555,32'h66666666,1,0); end
        d_rdata = 32'h11223344;
        #1;
        if (d_wdata !== 32'h11aa3344) begin
            $display("MERGE FAIL got=%h exp=11aa3344", d_wdata); errors = errors + 1;
        end
        d_ready = 1;
        @(posedge clk); #1;
        if (dut.pc !== 1 || dut.gr[5] !== 32'h00000155 || dut.gr[6] !== 7 || dut.br[7] !== 0)
            begin $display("COMMIT FAIL pc=%0d r5=%h r6=%h b7=%b", dut.pc,dut.gr[5],dut.gr[6],dut.br[7]); errors=errors+1; end
        d_ready = 0;

        // XNOP issues once, then holds PC/architectural state for two cycles.
        @(posedge clk); #1;
        if (dut.pc !== 2 || dut.stall_cnt !== 2) begin $display("XNOP ISSUE FAIL"); errors=errors+1; end
        repeat (2) begin @(posedge clk); #1; expect_hold(2,32'h00000155,7,0,0); end

        // STOP itself must not retire while its I fetch is stalled.
        i_ready = 0;
        repeat (2) begin @(posedge clk); #1; expect_hold(2,32'h00000155,7,0,0); end
        i_ready = 1;
        @(posedge clk); #1;
        if (!done || dut.pc !== 2) begin $display("STOP FAIL done=%b pc=%0d",done,dut.pc); errors=errors+1; end

        if (errors == 0) $display("RESULT: CODEX STALL TEST PASSED");
        else $display("RESULT: CODEX STALL TEST FAILED errors=%0d", errors);
        $finish;
    end
endmodule
