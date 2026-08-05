//======================================================================
// r-VEX SoC round 26 | rvex_soc_top_cache directed test
//----------------------------------------------------------------------
// Packets 0-27 are IDENTICAL to tb_soc_top.v's round-25 program (same
// encoding, same expected register values) -- reused deliberately: since
// this is a straight-line program with no address reuse except through
// word0/word1/word2 (which all map to the SAME D-Cache line, idx=0,
// tag=0), running it here naturally exercises the D-Cache's read-miss
// (packet 4's diagnostic LDW), read-hit (packets 8, 12, 13 -- word0/1
// already cached by then), write-around-on-cold-store (packets 1-3),
// and write-through-with-update-on-hit (packet 18's STB, then checked
// by packet 21's LDW reading the merged value straight back from cache)
// paths, all while re-proving round 25's core functional-correctness
// claims are unaffected by a cache sitting in the path. The I-Cache
// equivalently sees a fresh compulsory miss on almost every fetch here
// (no address is refetched in 0-27 except via the one branch, which
// jumps FORWARD only).
//
// Packets 28+ are NEW, added specifically to exercise what 0-27 don't:
//   28-33: a 2-iteration loop -- packet 30 (the loop body) is fetched
//     TWICE at the SAME address with nothing evicting its I-Cache line
//     in between, giving a genuine, explicit I-Cache HIT (plus
//     exercising the branch-squash / imem_stall interaction Codex asked
//     about in round 25's context, now with a real cache in the loop).
//   34-42: D-Cache eviction + write-through-consistency. LDWs to 7
//     distinct lines (words 4..28, one per remaining index) followed by
//     a LDW to word32 -- same index as word0 (idx=0) but a DIFFERENT
//     tag, which evicts the line holding word0-3's data. The final LDW
//     of word0 must therefore MISS again and correctly refetch 100 (its
//     original value, set all the way back in packet 1) from the
//     write-through memory, NOT read stale/wrong data -- the one
//     property write-around+write-through-on-hit is supposed to
//     guarantee and that only an actual eviction can falsify.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_soc_top_cache;
    reg clk=0, reset=1, start=0;
    wire done; wire [31:0] cycles;
    integer pass=0, fail=0, k;

    rvex_soc_top_cache dut(.clk(clk),.reset(reset),.start(start),.done(done),.cycles(cycles));
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
        dut.u_imem_ctrl.u_sram.mem[a] = {s3,s2,s1,s0}; end
    endtask
    task chk; input [127:0] nm; input [31:0] got,exp; begin
        if (got===exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h (%0d)", nm, got, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h (%0d) exp 0x%08h (%0d)", nm, got, got, exp, exp); end
    end endtask

    integer a;
    initial begin
        for (a=0;a<256;a=a+1) dut.u_imem_ctrl.u_sram.mem[a] = 128'd0;
        for (a=0;a<256;a=a+1) dut.u_dmem_ctrl.u_sram.mem[a] = 32'd0;

        // ---- packets 0-27: identical to tb_soc_top.v (round 25) ----
        P(8'd0,  ri(`ALU_ADD,6'd1,6'd0,9'd100), ri(`ALU_ADD,6'd2,6'd0,9'd7), NOP, NOP);
        P(8'd1,  NOP, NOP, NOP, ri(`MEM_STW,6'd1,6'd0,9'd0));
        P(8'd2,  NOP, NOP, NOP, ri(`MEM_STW,6'd2,6'd0,9'd4));
        P(8'd3,  NOP, NOP, NOP, ri(`MEM_STW,6'd1,6'd0,9'd8));
        P(8'd4,  NOP, NOP, NOP, ri(`MEM_LDW,6'd13,6'd0,9'd8));
        P(8'd5,  ri(`ALU_ADD,6'd5,6'd0,9'd200), NOP, NOP, NOP);
        P(8'd6,  NOP, NOP, NOP, NOP);
        P(8'd7,  NOP, NOP, NOP, NOP);
        P(8'd8,  NOP, NOP, NOP, ri(`MEM_LDW,6'd5,6'd0,9'd0));
        P(8'd9,  ri(`ALU_ADD,6'd6,6'd5,9'd0), NOP, NOP, NOP);
        P(8'd10, ri(`ALU_ADD,6'd7,6'd5,9'd0), NOP, NOP, NOP);
        P(8'd11, NOP, NOP, NOP, NOP);
        P(8'd12, NOP, NOP, NOP, ri(`MEM_LDW,6'd8,6'd0,9'd0));
        P(8'd13, NOP, NOP, NOP, ri(`MEM_LDW,6'd9,6'd0,9'd4));
        P(8'd14, NOP, NOP, NOP, NOP);
        P(8'd15, NOP, NOP, NOP, NOP);
        P(8'd16, NOP, NOP, NOP, NOP);
        P(8'd17, ri(`ALU_ADD,6'd11,6'd0,9'd170), NOP, NOP, NOP);
        P(8'd18, NOP, NOP, NOP, ri(`MEM_STB,6'd11,6'd0,9'd8));
        P(8'd19, NOP, NOP, NOP, NOP);
        P(8'd20, NOP, NOP, NOP, NOP);
        P(8'd21, NOP, NOP, NOP, ri(`MEM_LDW,6'd12,6'd0,9'd8));
        P(8'd22, NOP, NOP, NOP, NOP);
        P(8'd23, rrb(`ALU_CMPLT,6'd0,6'd2,6'd1,3'd1), NOP, NOP, NOP);
        P(8'd24, jbr(`CTRL_BR,12'd27,3'd1), NOP, NOP, NOP);
        P(8'd25, ri(`ALU_ADD,6'd21,6'd0,9'd111), NOP, NOP, NOP);
        P(8'd26, ri(`ALU_ADD,6'd21,6'd0,9'd222), NOP, NOP, NOP);
        P(8'd27, ri(`ALU_ADD,6'd22,6'd0,9'd55), NOP, NOP, NOP);

        // ---- packets 28-33: 2-iteration loop -- I-Cache HIT on packet 30's 2nd fetch ----
        P(8'd28, ri(`ALU_ADD,6'd29,6'd0,9'd2), NOP, NOP, NOP);      // r29 = 2 (loop bound)
        P(8'd29, ri(`ALU_ADD,6'd30,6'd0,9'd0), NOP, NOP, NOP);      // r30 = 0 (loop counter)
        P(8'd30, ri(`ALU_ADD,6'd30,6'd30,9'd1), NOP, NOP, NOP);     // (L) r30 = r30 + 1  -- fetched twice
        P(8'd31, rrb(`ALU_CMPNE,6'd0,6'd30,6'd29,3'd2), NOP, NOP, NOP); // br2 = (r30 != r29)
        P(8'd32, jbr(`CTRL_BR,12'd30,3'd2), NOP, NOP, NOP);         // BR br2 -> L (loop while r30!=2)
        P(8'd33, ri(`ALU_ADD,6'd31,6'd0,9'd255), NOP, NOP, NOP);    // r31 = 255 (loop-exit marker; 9-bit max)

        // ---- packets 34-42: D-Cache eviction + write-through-consistency ----
        P(8'd34, NOP, NOP, NOP, ri(`MEM_LDW,6'd51,6'd0,9'd16));  // word4  (idx1) cold miss
        P(8'd35, NOP, NOP, NOP, ri(`MEM_LDW,6'd52,6'd0,9'd32));  // word8  (idx2) cold miss
        P(8'd36, NOP, NOP, NOP, ri(`MEM_LDW,6'd53,6'd0,9'd48));  // word12 (idx3) cold miss
        P(8'd37, NOP, NOP, NOP, ri(`MEM_LDW,6'd54,6'd0,9'd64));  // word16 (idx4) cold miss
        P(8'd38, NOP, NOP, NOP, ri(`MEM_LDW,6'd55,6'd0,9'd80));  // word20 (idx5) cold miss
        P(8'd39, NOP, NOP, NOP, ri(`MEM_LDW,6'd56,6'd0,9'd96));  // word24 (idx6) cold miss
        P(8'd40, NOP, NOP, NOP, ri(`MEM_LDW,6'd57,6'd0,9'd112)); // word28 (idx7) cold miss
        P(8'd41, NOP, NOP, NOP, ri(`MEM_LDW,6'd58,6'd0,9'd128)); // word32 (idx0,tag1) EVICTS word0-3's line
        P(8'd42, NOP, NOP, NOP, ri(`MEM_LDW,6'd50,6'd0,9'd0));   // word0 (idx0,tag0) MUST miss+refetch: expect 100

        // ---- packets 43-49: WARMED-UP load-use-latency=2 falsifiability ----
        // The EARLY distance-1/distance-2 check (packets 8-10, gr6/gr7) is
        // NOT a reliable falsifiability proof once a cache is in the path:
        // in that straight-line, never-repeated-address program, every
        // surrounding I-Cache fetch is a compulsory miss, and Codex's
        // round-25 review already flagged that a stalled surrounding
        // bundle can let an already-issued load complete "for free" before
        // an under-scheduled consumer executes -- masking exactly the
        // class of bug this design exists to catch (see this file's final
        // checks below and reviews/rvex-round26-*.html section 3 for the
        // full analysis). This loop, by contrast, executes its body TWICE
        // at the SAME addresses with nothing evicting either cache's
        // lines in between: iteration 1 is cold (I$+D$ compulsory misses,
        // timing not meaningful), iteration 2 has EVERY fetch and the
        // load itself as a HIT -- genuine steady-state 1-bundle/cycle
        // throughput, restoring the exact condition round 25 established
        // is required for the property to hold. gr76/gr77 (distance-1/2)
        // reflect the LAST (steady-state) iteration's behavior.
        P(8'd43, NOP, NOP, NOP, ri(`MEM_STW,6'd1,6'd0,9'd160));       // pre-store: word40 = r1 (=100)
        P(8'd44, ri(`ALU_ADD,6'd61,6'd0,9'd0), ri(`ALU_ADD,6'd62,6'd0,9'd2), NOP, NOP); // r61=0 (counter), r62=2 (bound)
        P(8'd45, ri(`ALU_ADD,6'd60,6'd0,9'd111), NOP, NOP, NOP);      // r60 = 111 (sentinel, re-set each iter)
        P(8'd46, ri(`ALU_ADD,6'd61,6'd61,9'd1), NOP, NOP, ri(`MEM_LDW,6'd60,6'd0,9'd160)); // (L) counter++ || LDW r60=word40
        P(8'd47, ri(`ALU_ADD,6'd58,6'd60,9'd0), NOP, NOP, NOP);       // distance-1: r58 = r60 + 0 (r58 no longer needed post-eviction-test)
        P(8'd48, ri(`ALU_ADD,6'd59,6'd60,9'd0), NOP, NOP, NOP);       // distance-2: r59 = r60 + 0
        P(8'd49, rrb(`ALU_CMPNE,6'd0,6'd61,6'd62,3'd3), NOP, NOP, NOP); // br3 = (r61 != r62)
        P(8'd50, jbr(`CTRL_BR,12'd45,3'd3), NOP, NOP, NOP);           // BR br3 -> loop (L=45, re-arms sentinel)

        P(8'd51, {`OP_STOP,25'd0}, NOP, NOP, NOP);

        reset=1; start=0; repeat(2) @(posedge clk);
        reset=0; start=1;
        k=0;
        while (done!==1'b1 && k<5000) begin @(posedge clk); k=k+1; end
        // `done` (STOP) does not wait for bundle_stall (matches round 25's
        // core: see slot0_is_stop's priority over bundle_stall in
        // rvex_core_bus.v), so a load issued just before STOP can still be
        // in flight when done first asserts -- load_complete_now's write
        // is deliberately unconditional on `done` for exactly this case,
        // but it still needs a few MORE clock edges to actually land.
        // Drain those before checking final architectural state.
        repeat(20) @(posedge clk);

        $display("================= r-VEX SoC AXI+cache directed test =================");
        $display("  done=%b  cycles=%0d  (timeout counter=%0d)", done, cycles, k);
        chk("gr1 (=100)",              dut.u_core.gr[1],  32'd100);
        chk("gr2 (=7)",                dut.u_core.gr[2],  32'd7);
        chk("gr13 diag: word2 pre-STB (=100)", dut.u_core.gr[13], 32'd100);
        // gr6/gr7: EARLY (cold-cache) distance-1/2 probe -- documented,
        // NOT a falsifiability proof once a cache is present (see the
        // packet-43-51 comment above and reviews/rvex-round26-*.html
        // section 3): every surrounding I-Cache fetch here is a
        // compulsory miss, and the extra fetch-stall time lets the load
        // finish before EITHER prober executes, so BOTH read fresh data.
        // This is the real, load-bearing finding of this round, not a
        // bug -- the WARMED-UP loop below (gr76/gr77) is the actual
        // falsifiability proof.
        chk("gr6  distance-1 (cold cache -- see note, NOT falsifiable here)", dut.u_core.gr[6], 32'd100);
        chk("gr7  distance-2: FRESH",  dut.u_core.gr[7],  32'd100);
        chk("gr8  D$ hit load #1 (=100)", dut.u_core.gr[8],  32'd100);
        chk("gr9  D$ hit load #2 (=7)",   dut.u_core.gr[9],  32'd7);
        chk("gr12 STB RMW merged",     dut.u_core.gr[12], 32'hAA000064);
        chk("gr21 poison (must be 0)", dut.u_core.gr[21], 32'd0);
        chk("gr22 branch target hit",  dut.u_core.gr[22], 32'd55);
        chk("gr30 loop counter (=2)",  dut.u_core.gr[30], 32'd2);
        chk("gr31 loop-exit marker",   dut.u_core.gr[31], 32'd255);
        chk("gr50 word0 post-eviction (=100)", dut.u_core.gr[50], 32'd100);
        chk("gr61 warmed-up loop ran twice (=2)", dut.u_core.gr[61], 32'd2);
        chk("gr58 WARMED-UP distance-1: STALE (I$+D$ hit, real falsifiability proof)", dut.u_core.gr[58], 32'd111);
        chk("gr59 WARMED-UP distance-2: FRESH",   dut.u_core.gr[59], 32'd100);
        chk("dmem[word0] (=100)",      dut.u_dmem_ctrl.u_sram.mem[0], 32'd100);
        chk("dmem[word2] STB merged",  dut.u_dmem_ctrl.u_sram.mem[2], 32'hAA000064);
        $display("=======================================================================");
        $display("SOC AXI+CACHE TEST: %0d passed, %0d failed", pass, fail);
        if (fail==0 && done===1'b1) $display("RESULT: SOC AXI+CACHE TEST PASSED");
        else $display("RESULT: SOC AXI+CACHE TEST FAILED");
        $finish;
    end
endmodule
