//======================================================================
// Codex R20 (task c): PIU-18 SEMFAIL edge cases (reviews/contracts/piu.md PIU-18)
//----------------------------------------------------------------------
// dut (AW = 16, the delivered system):
//   both cores test-and-set lock 2 in the same packets: the loser of the
//   round-robin is held by a D-stall WITH rbit = 1 (lock already taken), so
//   a counter that counts per cycle instead of per committed ldw over-counts
//   (precondition checked: at least one such held cycle is seen);
//   core 0 then: an unaligned ldw 0xFF09 (word 0xFF08, a real test-and-set),
//   an ldh of the window (not counted), a busy ldw in an XNOP packet (counted
//   once), a busy ldw while EN = 0 (not counted), EN = 1 again, a busy ldw
//   (counted), a release and a re-acquire (not counted).
//   Checks: SEMFAIL per core, SEMFAIL0 + SEMFAIL1 == hw_sem n_spin - 1 (the
//   EN = 0 failure), the SEMFAIL readout at 0x5C.
// dut17 (AW = 17, a 128 KB system; rvex_system's AW parameter): the hw_sem
//   window is 0x1FF00..0x1FFFF, so 0xFF00 is ordinary cached memory. core 0
//   loads a word holding 1 from 0xFF00: PIU-18 ("ldw to the hw_sem window")
//   says SEMFAIL stays 0. rvex_core.v decodes the window as &mem_ea[15:8]
//   (fixed 16 bits) while rvex_system.v decodes &d_addr[AW-1:8].
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_codex_semfail;
    reg clk = 0, reset = 1, start0 = 0, start1 = 0, start17 = 0;
    wire done0, done1, d17_0, d17_1; wire [31:0] cycles0, cycles1, c17_0, c17_1;
    integer pass = 0, fail = 0, k, held_busy = 0;

    rvex_system dut(.clk(clk), .reset(reset), .start0(start0), .start1(start1),
        .done0(done0), .done1(done1), .cycles0(cycles0), .cycles1(cycles1));
    rvex_system #(.AW(17)) dut17(.clk(clk), .reset(reset), .start0(start17), .start1(1'b0),
        .done0(d17_0), .done1(d17_1), .cycles0(c17_0), .cycles1(c17_1));
    always #5 clk = ~clk;

    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s,b; input [8:0] off; begin
        mst = {op,`NO_IMM,s,b,off,2'b00}; end endfunction
    function [31:0] j12; input [6:0] op; input [11:0] n; begin
        j12 = {op,8'b0,n,5'b0}; end endfunction
    localparam [31:0] NOP = 32'd0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    task P0; input [11:0] a; input [31:0] s0,s1,s2,s3; begin dut.cpu[0].l1i.rom[a] = {s3,s2,s1,s0}; end endtask
    task P1; input [11:0] a; input [31:0] s0,s1,s2,s3; begin dut.cpu[1].l1i.rom[a] = {s3,s2,s1,s0}; end endtask
    task Q0; input [11:0] a; input [31:0] s0,s1,s2,s3; begin dut17.cpu[0].l1i.rom[a] = {s3,s2,s1,s0}; end endtask
    task chk; input [255:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s : %0d", nm, got); end
        else begin fail = fail + 1; $display("  FAIL %0s : got %0d (0x%0h) exp %0d (0x%0h)", nm, got, got, exp, exp); end
    end endtask

    // precondition: a test-and-set ldw held by the hw_sem arbitration while the lock reads 1
    always @(posedge clk) begin
        if (dut.cpu[0].core.dstall && dut.cpu[0].core.op3 == `MEM_LDW && &dut.cpu[0].core.mem_ea[15:8]
            && dut.cpu[0].d_rdata[0]) held_busy = held_busy + 1;
        if (dut.cpu[1].core.dstall && dut.cpu[1].core.op3 == `MEM_LDW && &dut.cpu[1].core.mem_ea[15:8]
            && dut.cpu[1].d_rdata[0]) held_busy = held_busy + 1;
    end

    integer exp0, exp1;
    initial begin
        for (k = 0; k < 4096; k = k + 1) begin
            dut.cpu[0].l1i.rom[k] = 128'd0; dut.cpu[1].l1i.rom[k] = 128'd0;
            dut17.cpu[0].l1i.rom[k] = 128'd0; dut17.cpu[1].l1i.rom[k] = 128'd0;
        end
        for (k = 0; k < 16384; k = k + 1) dut.l2.mem[k] = 32'd0;
        for (k = 0; k < 32768; k = k + 1) dut17.l2.mem[k] = 32'd0;
        dut17.l2.mem[16'hFF00 >> 2] = 32'd1;           // ordinary data at 0xFF00 in a 128 KB system

        // identical prologue + contention section on both cores (packets 0..6)
        for (k = 0; k < 2; k = k + 1) begin
            if (k == 0) begin
                P0(0, ri(`ALU_ADD,6'd10,6'd0,9'h7F), ri(`ALU_ADD,6'd11,6'd0,9'd1), ri(`ALU_ADD,6'd13,6'd0,9'hFF), NOP);
                P0(1, ri(`ALU_SHL,6'd10,6'd10,9'd9), NOP, ri(`ALU_SHL,6'd13,6'd13,9'd8), NOP);  // 0xFE00, 0xFF00
                P0(2, NOP, NOP, NOP, ri(`MEM_LDW,6'd20,6'd13,9'h08));    // acquire lock 2 (contended)
                P0(3, NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd13,9'h08));    // busy (contended, held with rbit 1)
                P0(4, NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd13,9'h08));    // busy
                P0(5, NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd13,9'h08));    // busy
            end else begin
                P1(0, ri(`ALU_ADD,6'd10,6'd0,9'h7F), ri(`ALU_ADD,6'd11,6'd0,9'd1), ri(`ALU_ADD,6'd13,6'd0,9'hFF), NOP);
                P1(1, ri(`ALU_SHL,6'd10,6'd10,9'd9), NOP, ri(`ALU_SHL,6'd13,6'd13,9'd8), NOP);
                P1(2, NOP, NOP, NOP, ri(`MEM_LDW,6'd20,6'd13,9'h08));
                P1(3, NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd13,9'h08));
                P1(4, NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd13,9'h08));
                P1(5, NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd13,9'h08));
            end
        end
        P1(6, STOP, NOP, NOP, NOP);
        // core 0 only
        P0(6, NOP, NOP, NOP, ri(`MEM_LDW,6'd22,6'd13,9'h09));          // unaligned ldw 0xFF09 -> lock 2: busy
        P0(7, NOP, NOP, NOP, ri(`MEM_LDH,6'd23,6'd13,9'h08));          // sub-word: reads 1s, not counted
        P0(8, j12(`CTRL_XNOP,12'd3), NOP, NOP, ri(`MEM_LDW,6'd24,6'd13,9'h08));   // busy in an XNOP packet
        P0(9, NOP, NOP, NOP, mst(`MEM_STW,6'd0,6'd10,9'h00));          // CTRL = 0: EN off
        P0(10, NOP, NOP, NOP, ri(`MEM_LDW,6'd25,6'd13,9'h08));         // busy, EN = 0: not counted
        P0(11, NOP, NOP, NOP, mst(`MEM_STW,6'd11,6'd10,9'h00));        // CTRL = 1: EN on (no clear)
        P0(12, NOP, NOP, NOP, ri(`MEM_LDW,6'd26,6'd13,9'h08));         // busy: counted
        P0(13, NOP, NOP, NOP, mst(`MEM_STW,6'd0,6'd13,9'h08));         // release lock 2
        P0(14, NOP, NOP, NOP, ri(`MEM_LDW,6'd27,6'd13,9'h08));         // acquired: not counted
        P0(15, NOP, NOP, NOP, ri(`MEM_LDW,6'd28,6'd10,9'h5C));         // read SEMFAIL
        P0(16, STOP, NOP, NOP, NOP);

        // dut17 core 0: ldw from 0xFF00 = ordinary memory holding 1
        Q0(0, ri(`ALU_ADD,6'd13,6'd0,9'hFF), NOP, NOP, NOP);
        Q0(1, ri(`ALU_SHL,6'd13,6'd13,9'd8), NOP, NOP, NOP);
        Q0(2, NOP, NOP, NOP, ri(`MEM_LDW,6'd20,6'd13,9'h00));
        Q0(3, STOP, NOP, NOP, NOP);

        repeat (2) @(posedge clk);
        reset = 0; start0 = 1; start1 = 1; start17 = 1;
        k = 0;
        while (!(done0 === 1'b1 && done1 === 1'b1 && d17_0 === 1'b1) && k < 4000) begin @(posedge clk); k = k + 1; end
        repeat (2) @(posedge clk);

        // the core that lost packet 2 (r20 = 1) gets one more failure
        exp0 = 3 + 3 + (dut.cpu[0].core.gr[20] == 1);   // pkts 3-5, unaligned, XNOP pkt, after EN=1 (+ lost pkt 2)
        exp1 = 3 + (dut.cpu[1].core.gr[20] == 1);
        $display("============= Codex R20 SEMFAIL (PIU-18) edges =============");
        chk("done (3 cores)", done0 && done1 && d17_0, 1);
        chk("pre: busy ldw held", held_busy > 0, 1);
        chk("one core acquired pkt 2", dut.cpu[0].core.gr[20] + dut.cpu[1].core.gr[20], 1);
        chk("unaligned ldw busy", dut.cpu[0].core.gr[22], 1);
        chk("ldh reads 1s", dut.cpu[0].core.gr[23], 32'hFFFFFFFF);
        chk("re-acquire after release", dut.cpu[0].core.gr[27], 0);
        chk("PIU-18 core0 SEMFAIL", dut.cpu[0].core.u_piu.semfail, exp0);
        chk("PIU-18 core1 SEMFAIL", dut.cpu[1].core.u_piu.semfail, exp1);
        chk("PIU-18 sum == n_spin - 1",
            dut.cpu[0].core.u_piu.semfail + dut.cpu[1].core.u_piu.semfail, dut.sem_spin - 1);
        chk("PIU-18 readout 0x5C", dut.cpu[0].core.gr[28], exp0);
        chk("AW17 load 0xFF00 == 1", dut17.cpu[0].core.gr[20], 1);
        chk("AW17 no hw_sem access", dut17.sem_spin + dut17.sem_acq, 0);
        chk("AW17 PIU-18 SEMFAIL 0", dut17.cpu[0].core.u_piu.semfail, 0);
        $display("CODEX SEMFAIL: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: CODEX SEMFAIL PASSED"); else $display("RESULT: CODEX SEMFAIL FAILED");
        $finish;
    end
endmodule
