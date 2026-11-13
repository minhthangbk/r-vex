`timescale 1ns/1ps
`include "rvex_defs.vh"

/* Round 49 review probe: defined, reserved, and wrong-slot opcode behaviour in
 * slot 1.  This is deliberately not an ISA legality test; it records the RTL
 * side effects and PIU classification around mwbr.md MW-01..05. */
module tb_codex_r49_slot1_decode;
  reg clk=0, reset=1, start=0;
  wire done; wire [31:0] cycles;
  integer pass=0, fail=0, k, guard;
  localparam [31:0] NOP=0;
  localparam [31:0] STOP={`OP_STOP,25'd0};

  rvex_core dut(.clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles));
  always #5 clk=~clk;

  function [31:0] rr;
    input [6:0] op; input [5:0] dst; input [5:0] src1;
    begin rr={op,2'b00,dst,src1,6'd0,5'd0}; end
  endfunction
  task chk;
    input [255:0] name; input [31:0] got,exp;
    begin if(got===exp) pass=pass+1;
      else begin fail=fail+1; $display("FAIL %0s got=%h exp=%h",name,got,exp); end
    end
  endtask
  task run1;
    input [31:0] syl1;
    begin
      for(k=0;k<8;k=k+1) dut.imem[k]=0;
      dut.imem[0]={NOP,NOP,syl1,NOP}; dut.imem[1]={NOP,NOP,NOP,STOP};
      reset=1; start=0; repeat(2) @(posedge clk); reset=0; @(posedge clk);
      dut.gr[2]=32'd1; dut.gr[12]=32'hcafe1234; dut.br[3]=0;
      dut.dmem[0]=32'h11223344;
      start=1; guard=0;
      while(done!==1'b1 && guard<20) begin @(posedge clk); guard=guard+1; end
      start=0; #1;
    end
  endtask

  initial begin
    run1(rr(`OP_STOP,6'd12,6'd2));
    chk("STOP no GR",dut.gr[12],32'hcafe1234);
    chk("STOP OPS",dut.u_piu.ops,1); chk("STOP ALU",dut.u_piu.alu,1);
    chk("STOP CTRL",dut.u_piu.ctl,0); chk("STOP SLOT1",dut.u_piu.slot[1],1);

    run1(rr(`MEM_PFT,6'd12,6'd2));
    chk("wrong-slot PFT legacy zero WB",dut.gr[12],0);
    chk("wrong-slot PFT ALU",dut.u_piu.alu,1);

    run1(rr(`MEM_LDW,6'd12,6'd2));
    chk("wrong-slot load legacy zero WB",dut.gr[12],0);
    chk("wrong-slot load no LD",dut.u_piu.ld,0);

    run1(rr(`MEM_STW,6'd12,6'd2));
    chk("wrong-slot store legacy zero WB",dut.gr[12],0);
    chk("wrong-slot store no memory",dut.dmem[0],32'h11223344);
    chk("wrong-slot store no ST",dut.u_piu.st,0);

    run1(rr(`ALU_MTB,6'd0,6'd2) | (32'd3<<2));
    chk("MTB writes BR",dut.br[3],1); chk("MTB ALU",dut.u_piu.alu,1);

    run1(rr(7'h2c,6'd12,6'd2));
    chk("reserved CTRL-class inert",dut.gr[12],32'hcafe1234);
    chk("reserved CTRL-class CTRL",dut.u_piu.ctl,1);
    chk("reserved CTRL-class not ALU",dut.u_piu.alu,0);

    run1(rr(7'h69,6'd12,6'd2));
    chk("unknown legacy zero WB",dut.gr[12],0);
    chk("unknown ALU",dut.u_piu.alu,1);

    $display("CODEX R49 SLOT1 DECODE: %0d passed, %0d failed",pass,fail);
    if(!fail) $display("RESULT: CODEX R49 SLOT1 DECODE PASSED");
    else $display("RESULT: CODEX R49 SLOT1 DECODE FAILED");
    $finish;
  end
endmodule
