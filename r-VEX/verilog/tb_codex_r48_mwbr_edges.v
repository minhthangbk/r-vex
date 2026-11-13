`timescale 1ns/1ps
`include "rvex_defs.vh"

/* Round 48 review-only gaps for mwbr.md MW-01..05.
 * This bench intentionally includes an encoded STOP in slot 1 with non-zero
 * destination bits.  MW-01 says it is inert; the r24 RTL currently lets it
 * reach ordinary slot-1 writeback, so that row is expected to fail until the
 * decoder suppresses every non-executable slot-1 control-class operation.
 */
module tb_codex_r48_mwbr_edges;
  reg clk = 0, reset = 1, start = 0;
  wire done; wire [31:0] cycles;
  integer pass = 0, fail = 0, k, seen;
  localparam [31:0] NOP = 0;
  localparam [31:0] STOP = {`OP_STOP,25'd0};

  rvex_core dut(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles));
  always #5 clk = ~clk;

  function [31:0] ctl;
    input [6:0] op; input [11:0] off; input [2:0] b; input [5:0] dst;
    begin ctl = {op,2'b00,dst,off,b,2'b00}; end
  endfunction
  function [31:0] ri;
    input [5:0] d; input [8:0] imm;
    begin ri = {`ALU_ADD,2'b01,d,6'd0,imm,2'b00}; end
  endfunction
  task chk;
    input [255:0] name; input [31:0] got, exp;
    begin
      if (got === exp) pass = pass + 1;
      else begin fail = fail + 1; $display("FAIL %0s got=%h exp=%h", name, got, exp); end
    end
  endtask
  task run;
    input [127:0] p; input [31:0] lr; input [31:0] sp;
    begin
      for (k=0;k<16;k=k+1) dut.imem[k]=0;
      dut.imem[0]=p;
      dut.imem[1]={NOP,NOP,NOP,ri(6'd10,9'd1)}; dut.imem[2]={NOP,NOP,NOP,STOP};
      dut.imem[3]={NOP,NOP,NOP,ri(6'd10,9'd3)}; dut.imem[4]={NOP,NOP,NOP,STOP};
      dut.imem[5]={NOP,NOP,NOP,ri(6'd10,9'd5)}; dut.imem[6]={NOP,NOP,NOP,STOP};
      reset=1; start=0; repeat(2) @(posedge clk); reset=0; @(posedge clk);
      dut.gr[1]=sp; dut.gr[63]=lr; dut.gr[12]=32'hcafe1234; dut.br[0]=0;
      start=1; seen=0;
      while (done !== 1'b1 && seen < 50) begin @(posedge clk); seen=seen+1; end
      start=0;
    end
  endtask

  reg [127:0] p;
  initial begin
    for (k=0;k<4096;k=k+1) dut.imem[k]=0;

    // Every remaining slot-0 redirect class defeats a taken slot-1 goto.
    p={NOP,NOP,ctl(`CTRL_GOTO,12'd5,0,0),ctl(`CTRL_IGOTO,0,0,0)};
    run(p,3,32'h100); chk("igoto priority",dut.gr[10],3);
    p={NOP,NOP,ctl(`CTRL_GOTO,12'd5,0,0),ctl(`CTRL_ICALL,0,0,6'd63)};
    run(p,3,32'h100); chk("icall priority",dut.gr[10],3); chk("icall link",dut.gr[63],1);
    p={NOP,NOP,ctl(`CTRL_GOTO,12'd5,0,0),ctl(`CTRL_RETURN,0,0,6'd1)};
    run(p,3,3); chk("return priority",dut.gr[10],3); chk("return SP",dut.gr[1],3);
    p={NOP,NOP,ctl(`CTRL_GOTO,12'd5,0,0),ctl(`CTRL_RFI,0,0,6'd1)};
    run(p,3,3); chk("rfi priority",dut.gr[10],3); chk("rfi SP",dut.gr[1],3);

    // Slot-1 RFI is inert, while an ordinary slot-1 ALU op always commits.
    p={NOP,NOP,ctl(`CTRL_RFI,12'd5,0,6'd1),ctl(`CTRL_BR,12'd3,0,0)};
    run(p,3,32'h100); chk("slot1 rfi inert",dut.gr[10],1); chk("slot1 rfi no SP",dut.gr[1],32'h100);
    p={NOP,NOP,ri(6'd12,9'd77),ctl(`CTRL_GOTO,12'd3,0,0)};
    run(p,9,32'h100); chk("slot1 ALU on taken slot0",dut.gr[12],77); chk("slot0 goto path",dut.gr[10],3);

    // A slot-0 STOP commits no other lane.
    p={NOP,NOP,ri(6'd12,9'd88),STOP};
    run(p,9,32'h100); chk("slot0 stop suppresses slot1 ALU",dut.gr[12],32'hcafe1234);

    // MW-01: STOP in slot 1 is inert even if its otherwise-destination bits are nonzero.
    p={NOP,NOP,{`OP_STOP,2'b00,6'd12,17'd0},ctl(`CTRL_BR,12'd3,0,0)};
    run(p,9,32'h100); chk("slot1 stop no GR write",dut.gr[12],32'hcafe1234);

    $display("CODEX R48 MWBR EDGES: %0d passed, %0d failed",pass,fail);
    if (!fail) $display("RESULT: CODEX R48 MWBR EDGES PASSED");
    else $display("RESULT: CODEX R48 MWBR EDGES FAILED");
    $finish;
  end
endmodule
