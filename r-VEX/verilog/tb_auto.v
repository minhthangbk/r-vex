//======================================================================
// r-VEX | AUTO-compiled full-flow testbench  (C -> vex_llc -> .s -> hex)
//----------------------------------------------------------------------
// Runs a program built by the automatic path -- clang -> vex_llc (which
// does all the VLIW packetisation and scheduling) -> s2hex.py, wrapped in
// the startup harness from toolchain/harness.py -- and checks the result
// against the host C golden.
//
// This differs from tb_dsp.v, which runs the HAND-SCHEDULED dotprod: here
// nothing about the schedule is written by hand.
//
// Driven entirely by plusargs so one testbench serves every kernel:
//   +imem=<file>    128-bit packet image   ({syl3,syl2,syl1,syl0})
//   +dmem=<file>    32-bit data words, loaded at word 0
//   +chk=<file>     lines "<word_index> <expected>" checked after halt
//   +ret=<file>     optional "<gr_index> <expected>" for a returned scalar
//   +name=<string>  label for the report
//   +maxcyc=<n>     timeout (default 20000)
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_auto;
    reg clk=0, reset=1, start=0;
    wire done; wire [31:0] cycles;
    integer pass=0, fail=0, k;
    integer fd, code, widx, wexp, ridx, rexp, nchk;
    integer maxcyc, ngr;
    reg [1023:0] f_imem, f_dmem, f_chk, f_ret, nm;

    rvex_core dut(.clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles));
    always #5 clk = ~clk;

    task chk32; input [255:0] nmx; input [31:0] got, exp; begin
        if (got === exp) begin
            pass = pass + 1;
            $display("  PASS %0s : %0d (0x%08h)", nmx, $signed(got), got);
        end else begin
            fail = fail + 1;
            $display("  FAIL %0s : got %0d (0x%08h)  expected %0d (0x%08h)",
                     nmx, $signed(got), got, $signed(exp), exp);
        end
    end endtask

    initial begin
        if (!$value$plusargs("imem=%s", f_imem)) begin
            $display("tb_auto: need +imem=<file>"); $finish;
        end
        if (!$value$plusargs("name=%s", nm)) nm = "kernel";
        if (!$value$plusargs("maxcyc=%d", maxcyc)) maxcyc = 20000;

        for (k=0;k<256;k=k+1) begin dut.imem[k]=128'd0; dut.dmem[k]=32'd0; end
        $readmemh(f_imem, dut.imem);
        if ($value$plusargs("dmem=%s", f_dmem)) $readmemh(f_dmem, dut.dmem);

        reset=1; start=0; repeat(2) @(posedge clk);
        reset=0; start=1;
        k=0;
        while (done!==1'b1 && k<maxcyc) begin @(posedge clk); k=k+1; end

        $display("========== r-VEX AUTO full-flow : %0s ==========", nm);
        $display("  done=%b  cycles=%0d  (limit %0d)", done, cycles, maxcyc);
        if (done!==1'b1) begin
            fail = fail + 1;
            $display("  FAIL halt : core never reached `stop` within %0d cycles", maxcyc);
        end

        // ---- data-memory expectations ------------------------------------
        nchk = 0;
        if ($value$plusargs("chk=%s", f_chk)) begin
            fd = $fopen(f_chk, "r");
            if (fd == 0) begin
                fail = fail + 1; $display("  FAIL chk : cannot open %0s", f_chk);
            end else begin
                code = $fscanf(fd, "%d %d\n", widx, wexp);
                while (code == 2) begin
                    nchk = nchk + 1;
                    $display("  -- dmem[%0d]", widx);
                    chk32("dmem word", dut.dmem[widx], wexp);
                    code = $fscanf(fd, "%d %d\n", widx, wexp);
                end
                $fclose(fd);
            end
        end

        // ---- returned scalar ($r0.3 per the VEX RTA) ----------------------
        if ($value$plusargs("ret=%s", f_ret)) begin
            fd = $fopen(f_ret, "r");
            if (fd != 0) begin
                code = $fscanf(fd, "%d %d\n", ridx, rexp);
                if (code == 2) begin
                    $display("  -- return value in $r0.%0d", ridx);
                    chk32("retval", dut.gr[ridx], rexp);
                end
                $fclose(fd);
            end
        end

        // ---- optional register dump, for debugging a kernel that misbehaves --
        if ($value$plusargs("dumpgr=%d", ngr)) begin
            $display("  -- general registers 0..%0d", ngr);
            for (k = 0; k <= ngr; k = k + 1)
                if (dut.gr[k] !== 32'd0)
                    $display("     $r0.%0d = %0d (0x%08h)", k, $signed(dut.gr[k]), dut.gr[k]);
            for (k = 0; k < 8; k = k + 1)
                $write("     $b0.%0d=%b", k, dut.br[k]);
            $display("\n     pc=%0d", dut.pc);
        end

        $display("-----------------------------------------------------------");
        $display("AUTO FLOW %0s: %0d passed, %0d failed  (%0d dmem checks)",
                 nm, pass, fail, nchk);
        if (fail==0 && pass>0)
            $display("RESULT: AUTO FULL-FLOW PASSED (%0s: RTL == C golden)", nm);
        else
            $display("RESULT: AUTO FULL-FLOW FAILED (%0s)", nm);
        $display("===========================================================");
        $finish;
    end
endmodule
