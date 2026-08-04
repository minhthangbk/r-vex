//======================================================================
// r-VEX SoC phase 1 | rvex_soc_top directed test (round 25)
//----------------------------------------------------------------------
// Exercises, against the NEW bus-attached core (rvex_core_bus + real
// SRAM macros + AHB-Lite DMEM), exactly the things this round's design
// claims to newly support/prove, that rvex_core.v's own suite could
// never exercise (see rvex_core_bus.v's header, docs item 22):
//   1. Basic ALU-imm / VLIW-packed / register-register ops (sanity that
//      the restructured F/E pipeline didn't break the untouched slot0-2
//      datapath logic).
//   2. STW then LDW through the real AHB-Lite DMEM bus.
//   3. THE central falsifiability claim: a load's consumer at bundle-
//      distance 1 (0 bubbles) MUST read the STALE pre-load value, and at
//      bundle-distance 2 (1 bubble, matching rvexSchedule.td's
//      IILoadStore latency=2) MUST read the FRESH loaded value. Both
//      directions are checked -- a design that stalls the core on every
//      load (masking the hazard) would fail the distance-1 check by
//      reading the fresh value early; a design with one cycle too much
//      latency (an earlier draft of rvex_core_bus.v had exactly this
//      bug -- see its header) would fail the distance-2 check.
//   4. Back-to-back memory-accessing bundles (structural DMEM_BUSY
//      stall): two consecutive LDW bundles, checking BOTH values land
//      correctly (the stall must not corrupt data, only cost a cycle).
//   5. STB read-modify-write: a sub-word store must preserve the other
//      3 bytes of the target word (needs the DMEM sub-FSM's 2-phase
//      read-then-write path -- untested by anything else in this file).
//   6. A taken branch (CTRL_BR): the two already-in-flight wrong-path
//      fetches (squash) must be discarded -- the "poison" packets must
//      never commit.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_soc_top;
    reg clk=0, reset=1, start=0;
    wire done; wire [31:0] cycles;
    integer pass=0, fail=0, k;

    rvex_soc_top dut(.clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles));
    always #5 clk = ~clk;

    function [31:0] rr;  input [6:0] op; input [5:0] d,a,b; begin
        rr = {op,`NO_IMM,d,a,b,3'b000,2'b00}; end endfunction
    function [31:0] rrb; input [6:0] op; input [5:0] d,a,b; input [2:0] dbr; begin
        rrb = {op,`NO_IMM,d,a,b,dbr,2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] jbr; input [6:0] op; input [11:0] off; input [2:0] bidx; begin
        jbr = {op,8'b0,off,bidx,2'b0}; end endfunction

    localparam [31:0] NOP = 32'd0;

    task P; input [7:0] a; input [31:0] s0,s1,s2,s3; begin
        dut.u_imem.mem[a] = {s3,s2,s1,s0}; end
    endtask
    task chk; input [127:0] nm; input [31:0] got,exp; begin
        if (got===exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h (%0d)", nm, got, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h (%0d) exp 0x%08h (%0d)", nm, got, got, exp, exp); end
    end endtask

    integer a;
    initial begin
        for (a=0;a<256;a=a+1) dut.u_imem.mem[a] = 128'd0;
        for (a=0;a<256;a=a+1) dut.u_dmem_ctrl.u_sram.mem[a] = 32'd0;

        // 0: r1=100 || r2=7   (VLIW-packed ALU-imm sanity)
        P(8'd0,  ri(`ALU_ADD,6'd1,6'd0,9'd100), ri(`ALU_ADD,6'd2,6'd0,9'd7), NOP, NOP);
        // 1: STW dmem[word0] = r1 (=100)
        P(8'd1,  NOP, NOP, NOP, ri(`MEM_STW,6'd1,6'd0,9'd0));
        // 2: STW dmem[word1] = r2 (=7)
        P(8'd2,  NOP, NOP, NOP, ri(`MEM_STW,6'd2,6'd0,9'd4));
        // 3: STW dmem[word2] = r1 (=100 = 0x00000064, RMW base value)
        P(8'd3,  NOP, NOP, NOP, ri(`MEM_STW,6'd1,6'd0,9'd8));
        // 4: DIAGNOSTIC: LDW r13 = dmem[word2] -- isolates whether the 3-deep
        //    back-to-back STW chain (packets 1,2,3) landed word2 correctly,
        //    independent of the STB test below (added while root-causing the
        //    STB failure: this confirms the write side is fine and the bug
        //    is specifically in the STB read-modify-write path).
        P(8'd4,  NOP, NOP, NOP, ri(`MEM_LDW,6'd13,6'd0,9'd8));
        // 5: r5 = 200 (sentinel: proves distance-1 prober reads STALE, not X/garbage)
        P(8'd5,  ri(`ALU_ADD,6'd5,6'd0,9'd200), NOP, NOP, NOP);
        // 6,7: spacers (let the sentinel settle well before the load)
        P(8'd6,  NOP, NOP, NOP, NOP);
        P(8'd7,  NOP, NOP, NOP, NOP);
        // 8: LDW r5 = dmem[word0]  (=100) -- THE load under test
        P(8'd8,  NOP, NOP, NOP, ri(`MEM_LDW,6'd5,6'd0,9'd0));
        // 9: distance-1 (0 bubbles): r6 = r5 + 0  -- MUST see STALE 200
        P(8'd9,  ri(`ALU_ADD,6'd6,6'd5,9'd0), NOP, NOP, NOP);
        // 10: distance-2 (1 bubble): r7 = r5 + 0   -- MUST see FRESH 100
        P(8'd10, ri(`ALU_ADD,6'd7,6'd5,9'd0), NOP, NOP, NOP);
        // 11: spacer before the back-to-back load test
        P(8'd11, NOP, NOP, NOP, NOP);
        // 12: LDW r8 = dmem[word0] (=100)
        P(8'd12, NOP, NOP, NOP, ri(`MEM_LDW,6'd8,6'd0,9'd0));
        // 13: LDW r9 = dmem[word1] (=7) -- back-to-back with 12: exercises DMEM_BUSY structural stall
        P(8'd13, NOP, NOP, NOP, ri(`MEM_LDW,6'd9,6'd0,9'd4));
        // 14-16: spacers (let both loads fully land)
        P(8'd14, NOP, NOP, NOP, NOP);
        P(8'd15, NOP, NOP, NOP, NOP);
        P(8'd16, NOP, NOP, NOP, NOP);
        // 17: r11 = 170 (0xAA) -- byte value for the STB read-modify-write test
        P(8'd17, ri(`ALU_ADD,6'd11,6'd0,9'd170), NOP, NOP, NOP);
        // 18: STB dmem byte @ word2,pos0 (byte offset 8) = r11 -- must preserve word2's low 3 bytes
        P(8'd18, NOP, NOP, NOP, ri(`MEM_STB,6'd11,6'd0,9'd8));
        // 19-20: spacers (STB is a 2-phase read-then-write, needs time to land)
        P(8'd19, NOP, NOP, NOP, NOP);
        P(8'd20, NOP, NOP, NOP, NOP);
        // 21: LDW r12 = dmem[word2] -- expect merged 0xAA000064
        P(8'd21, NOP, NOP, NOP, ri(`MEM_LDW,6'd12,6'd0,9'd8));
        // 22: spacer
        P(8'd22, NOP, NOP, NOP, NOP);
        // 23: BR1 = (r2 < r1) signed = (7<100) = 1
        P(8'd23, rrb(`ALU_CMPLT,6'd0,6'd2,6'd1,3'd1), NOP, NOP, NOP);
        // 24: BR BR1 -> goto 27 (taken; must squash the 2 poison packets 25,26)
        P(8'd24, jbr(`CTRL_BR,12'd27,3'd1), NOP, NOP, NOP);
        // 25: poison -- must NEVER commit
        P(8'd25, ri(`ALU_ADD,6'd21,6'd0,9'd111), NOP, NOP, NOP);
        // 26: poison -- must NEVER commit
        P(8'd26, ri(`ALU_ADD,6'd21,6'd0,9'd222), NOP, NOP, NOP);
        // 27: branch target
        P(8'd27, ri(`ALU_ADD,6'd22,6'd0,9'd55), NOP, NOP, NOP);
        // 28: spacer
        P(8'd28, NOP, NOP, NOP, NOP);
        // 29: STOP
        P(8'd29, {`OP_STOP,25'd0}, NOP, NOP, NOP);

        reset=1; start=0; repeat(2) @(posedge clk);
        reset=0; start=1;
        k=0;
        while (done!==1'b1 && k<2000) begin @(posedge clk); k=k+1; end

        $display("================= r-VEX SoC phase-1 directed test =================");
        $display("  done=%b  cycles=%0d  (timeout counter=%0d)", done, cycles, k);
        chk("gr1 (=100)",              dut.u_core.gr[1],  32'd100);
        chk("gr2 (=7)",                dut.u_core.gr[2],  32'd7);
        chk("gr13 diag: word2 pre-STB (=100)", dut.u_core.gr[13], 32'd100);
        chk("gr6  distance-1: STALE",  dut.u_core.gr[6],  32'd200);
        chk("gr7  distance-2: FRESH",  dut.u_core.gr[7],  32'd100);
        chk("gr8  b2b load #1 (=100)", dut.u_core.gr[8],  32'd100);
        chk("gr9  b2b load #2 (=7)",   dut.u_core.gr[9],  32'd7);
        chk("gr12 STB RMW merged",     dut.u_core.gr[12], 32'hAA000064);
        chk("gr21 poison (must be 0)", dut.u_core.gr[21], 32'd0);
        chk("gr22 branch target hit",  dut.u_core.gr[22], 32'd55);
        chk("dmem[word0] (=100)",      dut.u_dmem_ctrl.u_sram.mem[0], 32'd100);
        chk("dmem[word1] (=7)",        dut.u_dmem_ctrl.u_sram.mem[1], 32'd7);
        chk("dmem[word2] STB merged",  dut.u_dmem_ctrl.u_sram.mem[2], 32'hAA000064);
        $display("=====================================================================");
        $display("SOC PHASE-1 TEST: %0d passed, %0d failed", pass, fail);
        if (fail==0 && done===1'b1) $display("RESULT: SOC PHASE-1 TEST PASSED");
        else $display("RESULT: SOC PHASE-1 TEST FAILED");
        $finish;
    end
endmodule
