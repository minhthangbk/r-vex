// Codex Round 42: system-level misalignment edges (misalign.md / PIU-19).
// Exercises crossing halfword accesses through L1D, a D-stalled store counted
// once, PIU and hw_sem windows, word align-down, and PIU EN gating on both cores.
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_codex_r42_misal_sys;
    reg clk = 0, reset = 1, start0 = 0, start1 = 0;
    wire done0, done1; wire [31:0] cycles0, cycles1;
    integer pass = 0, fail = 0, k, held0 = 0, held1 = 0;

    rvex_system dut(.clk(clk), .reset(reset), .start0(start0), .start1(start1),
        .done0(done0), .done1(done1), .cycles0(cycles0), .cycles1(cycles1));
    always #5 clk = ~clk;

    function [31:0] ri; input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s,b; input [8:0] off; begin
        mst = {op,`NO_IMM,s,b,off,2'b00}; end endfunction
    localparam [31:0] NOP = 32'd0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    task P0; input [11:0] a; input [31:0] s0,s1,s2,s3; begin dut.cpu[0].l1i.rom[a] = {s3,s2,s1,s0}; end endtask
    task P1; input [11:0] a; input [31:0] s0,s1,s2,s3; begin dut.cpu[1].l1i.rom[a] = {s3,s2,s1,s0}; end endtask
    task chk; input [255:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s: %0h", nm, got); end
        else begin fail = fail + 1; $display("  FAIL %0s: got %0h exp %0h", nm, got, exp); end
    end endtask

    // A store upgrade holds the crossing sth for several cycles. PIU-19 must
    // count the access on commit, not count these held presentations.
    always @(posedge clk) begin
        if (dut.cpu[0].core.dstall && dut.cpu[0].core.op3 == `MEM_STH && dut.cpu[0].core.mem_ea[1:0] == 3) held0 = held0 + 1;
        if (dut.cpu[1].core.dstall && dut.cpu[1].core.op3 == `MEM_STH && dut.cpu[1].core.mem_ea[1:0] == 3) held1 = held1 + 1;
    end

    initial begin
        for (k = 0; k < 4096; k = k + 1) begin dut.cpu[0].l1i.rom[k] = 0; dut.cpu[1].l1i.rom[k] = 0; end
        for (k = 0; k < 16384; k = k + 1) dut.l2.mem[k] = 0;
        dut.l2.mem[16'h1000 >> 2] = 32'h11223344;
        dut.l2.mem[16'h1004 >> 2] = 32'h0000AABB;
        dut.l2.mem[16'h2000 >> 2] = 32'h55667788;
        dut.l2.mem[16'h2004 >> 2] = 32'h0000CCDD;

        // r10=data base, r11=PIU 0xFE00, r12=hw_sem 0xFF00, r13=1.
        P0(0, ri(`ALU_ADD,10,0,9'h10), ri(`ALU_ADD,11,0,9'h7f), ri(`ALU_ADD,12,0,9'hff), ri(`ALU_ADD,13,0,1));
        P0(1, ri(`ALU_SHL,10,10,8), ri(`ALU_SHL,11,11,9), ri(`ALU_SHL,12,12,8), NOP);
        P0(2, NOP,NOP,NOP,ri(`MEM_LDW,20,10,4));
        P0(3, NOP,NOP,NOP,mst(`MEM_STH,20,10,3));       // crossing, store upgrade / D-stall
        P0(4, NOP,NOP,NOP,ri(`MEM_LDH,21,10,3));        // crossing -> 0
        P0(5, NOP,NOP,NOP,ri(`MEM_LDHU,22,10,1));       // in-word -> 0x2233
        P0(6, NOP,NOP,NOP,ri(`MEM_LDW,23,10,1));        // align-down -> 0x11223344
        P0(7, NOP,NOP,NOP,ri(`MEM_LDW,24,11,9'h61));    // PIU misaligned: reads old count 4, then counts itself
        P0(8, NOP,NOP,NOP,ri(`MEM_LDH,25,12,1));        // hw_sem subword -> -1, no acquire
        P0(9, NOP,NOP,NOP,ri(`MEM_LDW,26,12,1));        // align-down -> lock 0 acquire
        P0(10,NOP,NOP,NOP,mst(`MEM_STW,0,11,0));        // PIU EN=0
        P0(11,NOP,NOP,NOP,ri(`MEM_LDW,27,10,1));        // misaligned but disabled
        P0(12,NOP,NOP,NOP,mst(`MEM_STW,13,11,0));       // PIU EN=1
        P0(13,NOP,NOP,NOP,mst(`MEM_STH,20,11,3));       // crossing PIU store: CTRL unchanged
        P0(14,NOP,NOP,NOP,ri(`MEM_LDW,28,11,9'h60));    // MISALIGN = 8
        P0(15,STOP,NOP,NOP,NOP);

        P1(0, ri(`ALU_ADD,10,0,9'h20), ri(`ALU_ADD,11,0,9'h7f), NOP, NOP);
        P1(1, ri(`ALU_SHL,10,10,8), ri(`ALU_SHL,11,11,9), NOP, NOP);
        P1(2, NOP,NOP,NOP,ri(`MEM_LDW,20,10,4));
        P1(3, NOP,NOP,NOP,mst(`MEM_STH,20,10,3));
        P1(4, NOP,NOP,NOP,ri(`MEM_LDH,21,10,3));
        P1(5, NOP,NOP,NOP,ri(`MEM_LDHU,22,10,1));
        P1(6, NOP,NOP,NOP,ri(`MEM_LDW,23,10,1));
        P1(7, NOP,NOP,NOP,ri(`MEM_LDW,24,11,9'h61));    // old count 4; final count 5
        P1(8, STOP,NOP,NOP,NOP);

        repeat (2) @(posedge clk);
        reset = 0; start0 = 1; start1 = 1;
        k = 0;
        while (!(done0 && done1) && k < 5000) begin @(posedge clk); k = k + 1; end
        repeat (2) @(posedge clk);

        $display("============= Codex R42 misalign system =============");
        chk("both cores done", done0 && done1, 1);
        chk("core0 crossing sth unchanged", dut.peek(16'h1000), 32'h11223344);
        chk("core1 crossing sth unchanged", dut.peek(16'h2000), 32'h55667788);
        chk("core0 crossing ldh", dut.cpu[0].core.gr[21], 0);
        chk("core1 crossing ldh", dut.cpu[1].core.gr[21], 0);
        chk("core0 in-word ldhu", dut.cpu[0].core.gr[22], 32'h2233);
        chk("core1 in-word ldhu", dut.cpu[1].core.gr[22], 32'h6677);
        chk("core0 unaligned ldw align-down", dut.cpu[0].core.gr[23], 32'h11223344);
        chk("core1 unaligned ldw align-down", dut.cpu[1].core.gr[23], 32'h55667788);
        chk("both crossing stores D-stalled", held0 > 0 && held1 > 0, 1);
        chk("core0 PIU misaligned read sees prior count", dut.cpu[0].core.gr[24], 4);
        chk("core1 PIU misaligned read sees prior count", dut.cpu[1].core.gr[24], 4);
        chk("hw_sem ldh result", dut.cpu[0].core.gr[25], 32'hFFFFFFFF);
        chk("hw_sem ldw align-down acquires", dut.cpu[0].core.gr[26], 0);
        chk("disabled misaligned ldw still loads", dut.cpu[0].core.gr[27], 32'h11223344);
        chk("core0 MISALIGN read", dut.cpu[0].core.gr[28], 8);
        chk("core0 MISALIGN final", dut.cpu[0].core.u_piu.misalign, 8);
        chk("core1 MISALIGN final", dut.cpu[1].core.u_piu.misalign, 5);
        chk("PIU crossing sth leaves EN", dut.cpu[0].core.u_piu.en, 1);
        chk("hw_sem only word acquired", dut.sem_acq, 1);
        // The no-op store still obtains ownership: unchanged data in a dirty M line.
        chk("core0 unchanged line is M", dut.cpu[0].l1d.st[0], 2);
        chk("core1 unchanged line is M", dut.cpu[1].l1d.st[0], 2);
        $display("CODEX R42 MISAL SYS: %0d passed, %0d failed", pass, fail);
        if (!fail) $display("RESULT: CODEX R42 MISAL SYS PASSED");
        else $display("RESULT: CODEX R42 MISAL SYS FAILED");
        $finish;
    end
endmodule
