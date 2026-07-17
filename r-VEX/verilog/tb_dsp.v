//======================================================================
// r-VEX | DSP full-flow testbench (C -> asm -> binary -> core)
//----------------------------------------------------------------------
// Loads a program + data image produced by the external assembler
// (dsp/rvex_asm.py) via $readmemh -- the file-loader bridge the Verilog
// core previously lacked -- then runs the dot-product kernel end to end
// and checks the result against the host C golden (120 = 0x78).
//
//   dsp_imem.hex : 128-bit VLIW packets  ({syl3,syl2,syl1,syl0})
//   dsp_dmem.hex : 32-bit data words     (a[0..7] @0..7, b[0..7] @8..15)
//   result       : sum -> GR $r3 and data memory word 16 (byte 64)
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_dsp;
    reg clk=0, reset=1, start=0;
    wire done; wire [31:0] cycles;
    integer pass=0, fail=0, k;

    rvex_core dut(.clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles));
    always #5 clk = ~clk;

    localparam [31:0] GOLDEN = 32'd120;   // from dsp/dotprod.exe

    task chk; input [127:0] nm; input [31:0] got,exp; begin
        if (got===exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h (%0d)", nm, got, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", nm, got, exp); end
    end endtask

    initial begin
        // clear memories, then load the assembled images
        for (k=0;k<256;k=k+1) begin dut.imem[k]=128'd0; dut.dmem[k]=32'd0; end
        $readmemh("dsp_imem.hex", dut.imem);
        $readmemh("dsp_dmem.hex", dut.dmem);

        // run
        reset=1; start=0; repeat(2) @(posedge clk);
        reset=0; start=1;
        k=0;
        while (done!==1'b1 && k<1000) begin @(posedge clk); k=k+1; end

        $display("============ r-VEX DSP full-flow (dot product) ============");
        $display("  done=%b  cycles=%0d  (timeout counter=%0d)", done, cycles, k);
        $display("  host golden = %0d (0x%08h)", GOLDEN, GOLDEN);
        chk("GR $r3  sum",        dut.gr[3],    GOLDEN);
        chk("dmem[16] stored sum",dut.dmem[16], GOLDEN);
        // sanity: pointers walked the whole array (i reached n=8)
        chk("GR $r1  i==n",       dut.gr[1],    32'd8);
        $display("===========================================================");
        $display("DSP FLOW: %0d passed, %0d failed", pass, fail);
        if (fail==0 && done===1'b1) $display("RESULT: DSP FULL-FLOW PASSED (RTL sum == C golden)");
        else $display("RESULT: DSP FULL-FLOW FAILED");
        $finish;
    end
endmodule
