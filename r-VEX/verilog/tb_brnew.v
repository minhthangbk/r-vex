`timescale 1ns/1ps
`include "rvex_defs.vh"

/* brn / brfn: a branch that reads the BR written in its own packet
 * (reviews/contracts/brnew.md BN-01..06, BN-10).  Each case runs a 5-packet
 * program on a fresh core:
 *   0: { writer of BR[w] in slot S, branch on BR[b] (flag n) to 3 }
 *   1: add r10 = r0, 1     2: stop
 *   3: add r10 = r0, 2     4: stop
 * and checks the path taken (r10), the BR value after the packet and the
 * PIU taken-branch counter.  A second core with EXT_DMEM=1 holds a brn
 * packet on a D-miss (BN-06). */
module tb_brnew;
    reg clk = 0, reset = 1, start = 0;
    wire done; wire [31:0] cycles;
    integer pass = 0, fail = 0, k, s, b, form, pol, oldv, newv, nflag, other;
    localparam [31:0] NOP = 0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    rvex_core dut(.clk(clk), .reset(reset), .start(start),
                  .done(done), .cycles(cycles));
    always #5 clk = ~clk;

    function [31:0] jbr;   // br/brf (n = 0) or brn/brfn (n = 1)
        input [6:0] op; input [11:0] off; input [2:0] b; input n;
        begin jbr = {op,2'b00,5'b0,n,off,b,2'b0}; end
    endfunction
    function [31:0] ri;
        input [6:0] op; input [5:0] d,a; input [8:0] im;
        begin ri = {op,`SHORT_IMM,d,a,im,2'b00}; end
    endfunction
    // writer forms: 0 register compare (dst 0), 1 BRIMM compare, 2 mtb, 3 addcg carry, 4 divs carry
    function [31:0] wr;
        input integer f; input [2:0] w; input [2:0] ci;
        begin
            case (f)
                0: wr = {`ALU_CMPEQ,2'b00,6'd0,6'd3,6'd4,w,2'b00};          // BR[w] = r3 == r4
                1: wr = {`ALU_CMPEQ,2'b11,3'b000,w,6'd3,9'd77,2'b00};       // BR[w] = r3 == 77
                2: wr = {`ALU_MTB,2'b00,6'd0,6'd5,6'd0,w,2'b00};            // BR[w] = r5[0]
                3: wr = {`ALU_ADDCG4,ci,2'b00,6'd11,6'd6,6'd7,w,2'b00}; // r11, BR[w] = carry(r6 + r7 + BR[ci])
                default: wr = {`ALU_DIVS4,ci,2'b00,6'd11,6'd8,6'd7,w,2'b00};  // r11, BR[w] = r8[31] (divs, final verify F3)
            endcase
        end
    endfunction
    task chk;
        input [255:0] name; input [31:0] got, exp;
        begin
            if (got === exp) pass = pass + 1;
            else begin fail = fail + 1;
                $display("FAIL %0s got=%0h exp=%0h (slot %0d br %0d form %0d pol %0d old %0d new %0d n %0d other %0d)",
                         name, got, exp, s, b, form, pol, oldv, newv, nflag, other);
            end
        end
    endtask

    task run_case;
        reg [31:0] w; reg [2:0] wb, ci; reg [127:0] p; integer seen, exp_t;
        begin
            wb = other ? b[2:0] ^ 3'd1 : b[2:0];
            ci = b[2:0] + 3'd2;                                    // addcg carry-in: neither b nor wb
            w = wr(form, wb, ci);
            p = {NOP, NOP, NOP, jbr(pol ? `CTRL_BRF : `CTRL_BR, 12'd3, b[2:0], nflag[0])};
            if (s == 1) p[63:32] = w; else if (s == 2) p[95:64] = w; else p[127:96] = w;
            for (k = 0; k < 8; k = k + 1) dut.imem[k] = 128'd0;
            dut.imem[0] = p;
            dut.imem[1] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd1)};
            dut.imem[2] = {NOP, NOP, NOP, STOP};
            dut.imem[3] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd2)};
            dut.imem[4] = {NOP, NOP, NOP, STOP};
            reset = 1; start = 0;
            repeat (2) @(posedge clk);
            reset = 0;
            @(posedge clk);
            for (k = 0; k < 8; k = k + 1) dut.br[k] = (k == b) ? oldv[0] : ~oldv[0];
            if (form >= 3) dut.br[ci] = 1'b0;
            dut.u_piu.en = 1'b1;
            // operands that make the writer produce newv
            dut.gr[3] = newv ? 32'd77 : 32'd78;   // forms 0, 1
            dut.gr[4] = 32'd77;
            dut.gr[5] = newv ? 32'd3 : 32'd2;     // form 2
            dut.gr[6] = 32'hFFFF_FFFF;            // form 3
            dut.gr[7] = newv ? 32'd1 : 32'd0;
            dut.gr[8] = newv ? 32'h8000_0000 : 32'd5; // form 4
            start = 1;
            seen = 0;
            while (done !== 1'b1 && seen < 20) begin @(posedge clk); seen = seen + 1; end
            start = 0;
            // the decision value: new BR value for brn/brfn when the written BR is the named one
            exp_t = (nflag && !other) ? newv : oldv;
            if (pol) exp_t = !exp_t;
            chk("STOP", done, 1);
            chk("path r10", dut.gr[10], exp_t ? 2 : 1);
            chk("BR written", dut.br[wb], newv);
            if (other) chk("named BR kept", dut.br[b], oldv);
            chk("PIU brt", dut.u_piu.brt, exp_t);
            chk("PIU ctl", dut.u_piu.ctl, 1);          // BN-10: the brn/brfn packet counts as one CTRL op
        end
    endtask

    // ---- BN-06: a brn packet held by a D-miss ----
    reg reset2 = 1, start2 = 0, ready2 = 0;
    wire done2; wire [31:0] cycles2;
    wire d_req2, d_we2, d_word2; wire [31:0] d_addr2, d_wdata2;
    wire i_req2; wire [11:0] i_addr2; wire [31:0] st2, ist2;
    rvex_core #(.EXT_DMEM(1)) dut2(.clk(clk), .reset(reset2), .start(start2), .done(done2), .cycles(cycles2),
        .d_req(d_req2), .d_we(d_we2), .d_addr(d_addr2), .d_wdata(d_wdata2), .d_word(d_word2),
        .d_rdata(32'h1234_5678), .d_ready(ready2), .stall_cycles(st2),
        .i_req(i_req2), .i_addr(i_addr2), .i_rdata(128'd0), .i_ready(1'b1), .istall_cycles(ist2));

    initial begin
        for (k = 0; k < 4096; k = k + 1) dut.imem[k] = 128'd0;
        // BN-01..05 + BN-10 sweep
        for (s = 1; s <= 3; s = s + 1)
          for (b = 0; b < 8; b = b + 1)
            for (form = 0; form < 5; form = form + 1)
              for (pol = 0; pol < 2; pol = pol + 1)
                for (nflag = 0; nflag < 2; nflag = nflag + 1)
                  for (other = 0; other < 2; other = other + 1)
                    for (newv = 0; newv < 2; newv = newv + 1) begin
                      oldv = !newv;            // old and new BR differ: the decision shows which was read
                      run_case;
                    end
        // BN-05: brn without a writer reads the old BR (both values)
        s = 1; form = 0; other = 0;
        for (b = 0; b < 8; b = b + 1)
          for (oldv = 0; oldv < 2; oldv = oldv + 1) begin
            for (k = 0; k < 8; k = k + 1) dut.imem[k] = 128'd0;
            dut.imem[0] = {NOP, NOP, ri(`ALU_ADD, 6'd12, 6'd0, 9'd5), jbr(`CTRL_BR, 12'd3, b[2:0], 1'b1)};
            dut.imem[1] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd1)};
            dut.imem[2] = {NOP, NOP, NOP, STOP};
            dut.imem[3] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd2)};
            dut.imem[4] = {NOP, NOP, NOP, STOP};
            reset = 1; repeat (2) @(posedge clk); reset = 0; @(posedge clk);
            for (k = 0; k < 8; k = k + 1) dut.br[k] = (k == b) ? oldv[0] : ~oldv[0];
            start = 1; k = 0;
            while (done !== 1'b1 && k < 20) begin @(posedge clk); k = k + 1; end
            start = 0;
            chk("BN-05 no writer: old BR", dut.gr[10], oldv ? 2 : 1);
          end

        // BN-06: {ldw r20 = 0[r0] (slot 3), cmpeq b2 = r3 == r4 (slot 1), brn b2 -> 3}
        // held for 3 cycles by d_ready = 0: no branch, no BR write, no load until it commits.
        for (k = 0; k < 8; k = k + 1) dut2.imem[k] = 128'd0;
        dut2.imem[0] = {{`MEM_LDW,`NO_IMM,6'd20,6'd0,9'd0,2'b00}, NOP,
                        {`ALU_CMPEQ,2'b00,6'd0,6'd3,6'd4,3'd2,2'b00}, jbr(`CTRL_BR, 12'd3, 3'd2, 1'b1)};
        dut2.imem[1] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd1)};
        dut2.imem[2] = {NOP, NOP, NOP, STOP};
        dut2.imem[3] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd10, 6'd0, 9'd2)};
        dut2.imem[4] = {NOP, NOP, NOP, STOP};
        repeat (2) @(posedge clk); reset2 = 0; @(posedge clk);
        dut2.br[2] = 1'b0; dut2.gr[3] = 32'd9; dut2.gr[4] = 32'd9;
        start2 = 1; ready2 = 0;
        repeat (3) @(posedge clk);
        #1;
        chk("BN-06 held: pc", dut2.pc, 0);
        chk("BN-06 held: BR", dut2.br[2], 0);
        chk("BN-06 held: load", dut2.gr[20], 0);
        ready2 = 1;
        k = 0;
        while (done2 !== 1'b1 && k < 20) begin @(posedge clk); k = k + 1; end
        chk("BN-06 path", dut2.gr[10], 2);
        chk("BN-06 BR", dut2.br[2], 1);
        chk("BN-06 load", dut2.gr[20], 32'h1234_5678);
        chk("BN-06 stall cycles", st2, 3);

        $display("BRNEW: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: BRNEW PASSED");
        else $display("RESULT: BRNEW FAILED");
        $finish;
    end
endmodule
