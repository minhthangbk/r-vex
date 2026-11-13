`timescale 1ns/1ps
`include "rvex_defs.vh"

/* MW-02/MW-05 hold-path row: a two-control packet is first held by EXT_IMEM,
 * then by an EXT_DMEM load.  PC/writeback and PIU packet events must occur
 * exactly once, on the eventual commit. */
module tb_codex_r48_mwbr_stall;
  reg clk=0, reset=1, start=0, i_ready=0, d_ready=0;
  reg [31:0] d_rdata=32'h89abcdef;
  wire done, i_req, d_req, d_we, d_word;
  wire [11:0] i_addr; wire [31:0] d_addr,d_wdata,cycles,stall_cycles,istall_cycles;
  reg [127:0] i_rdata;
  integer pass=0, fail=0;
  localparam [31:0] NOP=0;
  localparam [31:0] STOP={`OP_STOP,25'd0};

  function [31:0] br;
    input [6:0] op; input [11:0] off; input [2:0] b;
    begin br={op,2'b00,6'd0,off,b,2'b00}; end
  endfunction
  function [31:0] ldw;
    input [5:0] d,base; input [8:0] off;
    begin ldw={`MEM_LDW,2'b01,d,base,off,2'b00}; end
  endfunction
  wire [127:0] held_packet={ldw(6'd20,6'd0,9'd0),NOP,br(`CTRL_GOTO,12'd5,0),br(`CTRL_BR,12'd3,0)};
  always @(*) i_rdata = (i_addr==12'd0) ? held_packet : (i_addr==12'd5) ? {NOP,NOP,NOP,STOP} : 128'd0;

  rvex_core #(.EXT_IMEM(1),.EXT_DMEM(1)) dut(
    .clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles),
    .d_req(d_req),.d_we(d_we),.d_addr(d_addr),.d_wdata(d_wdata),.d_word(d_word),
    .d_rdata(d_rdata),.d_ready(d_ready),.stall_cycles(stall_cycles),
    .i_req(i_req),.i_addr(i_addr),.i_rdata(i_rdata),.i_ready(i_ready),.istall_cycles(istall_cycles));
  always #5 clk=~clk;

  task chk;
    input [255:0] name; input [31:0] got,exp;
    begin if(got===exp) pass=pass+1; else begin fail=fail+1; $display("FAIL %0s got=%h exp=%h",name,got,exp); end end
  endtask

  initial begin
    repeat(2) @(posedge clk); reset=0; start=1;
    repeat(4) @(posedge clk); #1;
    chk("I-hold PC",dut.pc,0); chk("I-hold pkt",dut.u_piu.pkt,0);
    chk("I-hold CTRL",dut.u_piu.ctl,0); chk("I-hold BRT",dut.u_piu.brt,0);

    i_ready=1; d_ready=0;
    repeat(4) @(posedge clk); #1;
    chk("D-hold PC",dut.pc,0); chk("D-hold load WB",dut.gr[20],0);
    chk("D-hold pkt",dut.u_piu.pkt,0); chk("D-hold CTRL",dut.u_piu.ctl,0);
    chk("D-hold BRT",dut.u_piu.brt,0);

    d_ready=1; @(posedge clk); #1;
    chk("commit PC slot1",dut.pc,5); chk("commit load WB",dut.gr[20],32'h89abcdef);
    chk("commit pkt once",dut.u_piu.pkt,1); chk("commit CTRL twice",dut.u_piu.ctl,2);
    chk("commit BRT once",dut.u_piu.brt,1); chk("slot1 control is not ALU",dut.u_piu.alu,0);
    @(posedge clk); #1; chk("target stop",done,1);

    $display("CODEX R48 MWBR STALL: %0d passed, %0d failed",pass,fail);
    if(!fail) $display("RESULT: CODEX R48 MWBR STALL PASSED");
    else $display("RESULT: CODEX R48 MWBR STALL FAILED");
    $finish;
  end
endmodule
