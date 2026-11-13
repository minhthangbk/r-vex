`timescale 1ns/1ps
`include "rvex_defs.vh"

/* Two branches in one packet (reviews/contracts/mwbr.md MW-01..05, round r24, BUG-103).
 * Each case runs a 7-packet program on a fresh core:
 *   0: { slot3, slot2, slot1 = second branch (target 5), slot0 = first op (branch target 3) }
 *   1: add r10 = r0, 1   2: stop        (no branch taken)
 *   3: add r10 = r0, 2   4: stop        (slot 0 taken)
 *   5: add r10 = r0, 3   6: stop        (slot 1 taken)
 * and checks the path (r10), that a slot-1 control op wrote no GR (MW-04) and the PIU ctl / brt counters (MW-05).
 *   T1/T2 (MW-01/02): slot 0 br/brf x slot 1 br/brf/goto x both BR values x brn flags of slot 1 with a compare in
 *          slot 2 or 3 writing its BR (MW-03)
 *   T3 (MW-03): slot-1 brn with the writer in slot 0 (a lone slot-1 branch), slot 2, slot 3, or no writer
 *   T5 (MW-02): slot 0 goto / call / xnop / taken br with a taken slot-1 branch: slot 0 wins
 *   T6 (MW-01): call, igoto, return, xnop in slot 1 do nothing
 *   T4 (MW-04): dst bits of a slot-1 branch syllable set: GR[dst] unchanged */
module tb_mwbr;
    reg clk = 0, reset = 1, start = 0;
    wire done; wire [31:0] cycles;
    integer pass = 0, fail = 0, k, i0, i1, v0, v1, n1, ws, seen, expp, exp_ctl, exp_brt;
    localparam [31:0] NOP = 0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};
    reg [255:0] tag;

    rvex_core dut(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles));
    always #5 clk = ~clk;

    function [31:0] jbr;   // br/brf/goto (n = brn flag), d5 = bits [22:18] (the GR dst field of other syllables)
        input [6:0] op; input [11:0] off; input [2:0] b; input n; input [4:0] d5;
        begin jbr = {op,2'b00,d5,n,off,b,2'b0}; end
    endfunction
    function [31:0] ri;
        input [6:0] op; input [5:0] d,a; input [8:0] im;
        begin ri = {op,`SHORT_IMM,d,a,im,2'b00}; end
    endfunction
    function [31:0] cmpw;  // BR[w] = r3 == r4
        input [2:0] w;
        begin cmpw = {`ALU_CMPEQ,2'b00,6'd0,6'd3,6'd4,w,2'b00}; end
    endfunction
    task chk;
        input [255:0] name; input [31:0] got, exp;
        begin
            if (got === exp) pass = pass + 1;
            else begin fail = fail + 1;
                $display("FAIL %0s / %0s got=%0h exp=%0h (i0 %0d i1 %0d v0 %0d v1 %0d n1 %0d ws %0d)",
                         tag, name, got, exp, i0, i1, v0, v1, n1, ws);
            end
        end
    endtask

    // run packet p as packet 0 with BR[0] = b0, BR[1] = b1; r3/r4 make the compare write cmpv
    task run; input [127:0] p; input b0, b1, cmpv;
        begin
            for (k = 0; k < 8; k = k + 1) dut.imem[k] = 128'd0;
            dut.imem[0] = p;
            dut.imem[1] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd1)};
            dut.imem[2] = {NOP, NOP, NOP, STOP};
            dut.imem[3] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd2)};
            dut.imem[4] = {NOP, NOP, NOP, STOP};
            dut.imem[5] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd3)};
            dut.imem[6] = {NOP, NOP, NOP, STOP};
            reset = 1; start = 0;
            repeat (2) @(posedge clk);
            reset = 0;
            @(posedge clk);
            for (k = 0; k < 8; k = k + 1) dut.br[k] = 1'b0;
            dut.br[0] = b0; dut.br[1] = b1;
            dut.gr[3] = 32'd77; dut.gr[4] = cmpv ? 32'd77 : 32'd78;
            for (k = 40; k < 48; k = k + 1) dut.gr[k] = 32'hDEAD_0000 + k;
            dut.gr[63] = 32'h0000_0BAD;
            dut.u_piu.en = 1'b1;
            start = 1; seen = 0;
            while (done !== 1'b1 && seen < 40) begin @(posedge clk); seen = seen + 1; end
            start = 0;
        end
    endtask

    task check_gr_kept;
        begin
            for (k = 40; k < 48; k = k + 1) chk("MW-04 no GR write", dut.gr[k], 32'hDEAD_0000 + k);
        end
    endtask

    reg [6:0] op0, op1; reg c0, c1, cv; reg [127:0] p;
    initial begin
        for (k = 0; k < 4096; k = k + 1) dut.imem[k] = 128'd0;
        // ---- T1/T2/MW-03: first branch br/brf on BR0 (old value), second br/brf/goto on BR1 ----
        for (i0 = 0; i0 < 2; i0 = i0 + 1)
          for (i1 = 0; i1 < 3; i1 = i1 + 1)
            for (v0 = 0; v0 < 2; v0 = v0 + 1)
              for (v1 = 0; v1 < 2; v1 = v1 + 1)
                for (n1 = 0; n1 < 2; n1 = n1 + 1)
                  for (ws = 2; ws <= 3; ws = ws + 1) begin
                    tag = "T1/T2";
                    op0 = i0 ? `CTRL_BRF : `CTRL_BR;
                    op1 = i1 == 0 ? `CTRL_BR : i1 == 1 ? `CTRL_BRF : `CTRL_GOTO;
                    // the compare writes BR1 = !v1: brn reads the new value, br the old one
                    p = {NOP, NOP, jbr(op1, 12'd5, 3'd1, n1[0], 5'b10101), jbr(op0, 12'd3, 3'd0, 1'b0, 5'b0)};
                    if (ws == 2) p[95:64] = cmpw(3'd1); else p[127:96] = cmpw(3'd1);
                    run(p, v0[0], v1[0], !v1[0]);
                    c0 = i0 ? !v0[0] : v0[0];
                    cv = n1 ? !v1[0] : v1[0];
                    c1 = i1 == 2 ? 1'b1 : i1 == 1 ? !cv : cv;
                    expp = c0 ? 2 : c1 ? 3 : 1;
                    chk("path", dut.gr[10], expp);
                    chk("BR1 written by the compare", dut.br[1], !v1[0]);
                    chk("PIU ctl (two branches)", dut.u_piu.ctl, 2);
                    chk("PIU brt", dut.u_piu.brt, (c0 || c1) ? 1 : 0);
                    chk("PIU alu (compare only)", dut.u_piu.alu, 1 + 1 + 0);   // compare + the add of the path
                    check_gr_kept;
                  end
        // ---- T3: slot-1 brn writer in slot 0 (ALU compare, lone slot-1 branch), slot 2, slot 3, none ----
        for (ws = 0; ws <= 4; ws = ws + 1)
          for (v1 = 0; v1 < 2; v1 = v1 + 1) begin
            tag = "T3";
            if (ws == 1) ws = 2;   // slot 1 holds the branch
            p = {NOP, NOP, jbr(`CTRL_BR, 12'd5, 3'd1, 1'b1, 5'b0), NOP};
            if (ws == 0) p[31:0] = cmpw(3'd1); else if (ws == 2) p[95:64] = cmpw(3'd1); else if (ws == 3) p[127:96] = cmpw(3'd1);
            run(p, 1'b0, v1[0], !v1[0]);
            expp = (ws == 4) ? (v1 ? 3 : 1) : (!v1 ? 3 : 1);     // ws 4: no writer -> old BR
            chk("MW-03 forwarded BR", dut.gr[10], expp);
            chk("PIU ctl (one branch, slot 1)", dut.u_piu.ctl, 1);
            chk("PIU brt", dut.u_piu.brt, expp == 3 ? 1 : 0);
          end
        // ---- T5 (MW-02): slot 0 redirects (goto, call, taken br) or xnops; the taken slot-1 goto loses ----
        for (i0 = 0; i0 < 4; i0 = i0 + 1) begin
            tag = "T5";
            op0 = i0 == 0 ? `CTRL_GOTO : i0 == 1 ? `CTRL_CALL : i0 == 2 ? `CTRL_BR : `CTRL_XNOP;
            p = {NOP, NOP, jbr(`CTRL_GOTO, 12'd5, 3'd0, 1'b0, 5'b0),
                 i0 == 3 ? {`CTRL_XNOP, 2'b00, 6'd0, 12'd2, 5'd0} : jbr(op0, 12'd3, 3'd0, 1'b0, 5'b0)};
            run(p, 1'b1, 1'b0, 1'b0);
            chk("MW-02 slot 0 wins", dut.gr[10], i0 == 3 ? 1 : 2);
            chk("PIU brt", dut.u_piu.brt, i0 == 3 ? 0 : 1);
        end
        // ---- T6 (MW-01/04): call, igoto, return, xnop in slot 1 do nothing (and write no link) ----
        for (i1 = 0; i1 < 4; i1 = i1 + 1) begin
            tag = "T6";
            op1 = i1 == 0 ? `CTRL_CALL : i1 == 1 ? `CTRL_IGOTO : i1 == 2 ? `CTRL_RETURN : `CTRL_XNOP;
            p = {NOP, NOP, jbr(op1, 12'd5, 3'd0, 1'b0, 5'b10101), jbr(`CTRL_BR, 12'd3, 3'd0, 1'b0, 5'b0)};
            run(p, 1'b0, 1'b0, 1'b0);
            chk("MW-01 slot-1 non-branch does nothing", dut.gr[10], 1);
            chk("MW-04 no link write", dut.gr[63], 32'h0000_0BAD);
            chk("MW-01 no SP write", dut.gr[1], 32'd0);
            check_gr_kept;
        end

        $display("MWBR: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: MWBR PASSED");
        else $display("RESULT: MWBR FAILED");
        $finish;
    end
endmodule
