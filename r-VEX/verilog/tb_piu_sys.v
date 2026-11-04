//======================================================================
// r-VEX | PIU in the two-core system (L1I/L1D misses, hw_sem, per-core PIU)
//----------------------------------------------------------------------
// Contract: reviews/contracts/piu.md (PIU-05, -09, -11, -13, -14).
// Core 0 runs with EN = 1 for the whole run (no CTRL write): an I-miss per
// L1I line, a D-miss load, a store upgrade, a PIU load, a hw_sem
// acquire/release, an XNOP. Core 1 clears EN after three packets.
// Checks: CYCLE/ISTALL/DSTALL equal the core's own counters, the cycle
// classes add up (PIU-13), a PIU access never raises d_req (PIU-05), and the
// two PIUs are independent (PIU-14).
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_piu_sys;
    reg clk = 0, reset = 1, start0 = 0, start1 = 0;
    wire done0, done1; wire [31:0] cycles0, cycles1;
    integer pass = 0, fail = 0, k, piu_dreq = 0;

    rvex_system dut(.clk(clk), .reset(reset), .start0(start0), .start1(start1),
        .done0(done0), .done1(done1), .cycles0(cycles0), .cycles1(cycles1));
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
    task chk; input [255:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s : %0d", nm, got); end
        else begin fail = fail + 1; $display("  FAIL %0s : got %0d (0x%0h) exp %0d (0x%0h)", nm, got, got, exp, exp); end
    end endtask

    // PIU-05: an access in the PIU window never goes out on the D port
    always @(posedge clk) begin
        if (dut.cpu[0].core.piu_hit && dut.cpu[0].core.d_req) piu_dreq = piu_dreq + 1;
        if (dut.cpu[1].core.piu_hit && dut.cpu[1].core.d_req) piu_dreq = piu_dreq + 1;
    end

    initial begin
        for (k = 0; k < 4096; k = k + 1) begin dut.cpu[0].l1i.rom[k] = 128'd0; dut.cpu[1].l1i.rom[k] = 128'd0; end
        for (k = 0; k < 16384; k = k + 1) dut.l2.mem[k] = 32'd0;
        dut.l2.mem[16'h1000 >> 2] = 32'h1234_5678;
        // core 0 (packets 0-1 match core 1, so both reach the hw_sem load in packet 2 in the same cycle)
        P0(0, ri(`ALU_ADD,6'd10,6'd0,9'h7F), ri(`ALU_ADD,6'd11,6'd0,9'h10), ri(`ALU_ADD,6'd13,6'd0,9'hFF), NOP);
        P0(1, ri(`ALU_SHL,6'd10,6'd10,9'd9), ri(`ALU_SHL,6'd12,6'd11,9'd8), ri(`ALU_SHL,6'd13,6'd13,9'd8), NOP); // 0xFE00, 0x1000, 0xFF00
        P0(2, NOP, NOP, NOP, ri(`MEM_LDW,6'd22,6'd13,9'h00));                             // hw_sem acquire (contended)
        P0(3, NOP, NOP, NOP, ri(`MEM_LDW,6'd20,6'd10,9'h04));                             // PIU ID
        P0(4, NOP, NOP, NOP, ri(`MEM_LDW,6'd21,6'd12,9'h00));                             // D miss
        P0(5, NOP, NOP, NOP, mst(`MEM_STW,6'd21,6'd12,9'h04));                            // store (upgrade)
        P0(6, NOP, NOP, NOP, mst(`MEM_STW,6'd0,6'd13,9'h00));                             // hw_sem release
        P0(7, j12(`CTRL_XNOP,12'd3), NOP, NOP, NOP);
        P0(8, NOP, NOP, NOP, ri(`MEM_LDW,6'd23,6'd10,9'h00));                             // CTRL reads 1
        // PIU-18 (r10): a sub-word load of hw_sem reads 0xFFFFFFFF (not a test-and-set: not counted),
        // a free lock is acquired (not counted), the same lock again is busy (counted)
        P0(9, NOP, NOP, NOP, ri(`MEM_LDB,6'd24,6'd13,9'h04));                             // ldb 0xFF04
        P0(10, NOP, NOP, NOP, ri(`MEM_LDW,6'd25,6'd13,9'h04));                            // acquire lock 1
        P0(11, NOP, NOP, NOP, ri(`MEM_LDW,6'd26,6'd13,9'h04));                            // lock 1 busy
        P0(12, NOP, NOP, NOP, ri(`MEM_LDW,6'd27,6'd10,9'h5C));                            // read SEMFAIL
        P0(13, STOP, NOP, NOP, NOP);
        // core 1
        P1(0, ri(`ALU_ADD,6'd10,6'd0,9'h7F), ri(`ALU_ADD,6'd11,6'd0,9'h10), ri(`ALU_ADD,6'd13,6'd0,9'hFF), NOP);
        P1(1, ri(`ALU_SHL,6'd10,6'd10,9'd9), ri(`ALU_SHL,6'd12,6'd11,9'd8), ri(`ALU_SHL,6'd13,6'd13,9'd8), NOP);
        P1(2, NOP, NOP, NOP, ri(`MEM_LDW,6'd22,6'd13,9'h00));                             // loses round-robin: waits
        P1(3, NOP, NOP, NOP, mst(`MEM_STW,6'd0,6'd10,9'h00));                             // EN = 0 (counted)
        P1(4, ri(`ALU_ADD,6'd1,6'd0,9'd1), NOP, NOP, NOP);
        P1(5, ri(`ALU_ADD,6'd2,6'd0,9'd2), NOP, NOP, NOP);
        P1(6, STOP, NOP, NOP, NOP);

        repeat (2) @(posedge clk);
        reset = 0; start0 = 1; start1 = 1;
        k = 0;
        while (!(done0 === 1'b1 && done1 === 1'b1) && k < 2000) begin @(posedge clk); k = k + 1; end
        repeat (2) @(posedge clk);

        $display("============= PIU system test (2 cores, caches) =============");
        chk("done0", done0, 1);
        chk("done1", done1, 1);
        chk("core0 ID",               dut.cpu[0].core.gr[20], 32'h50495501);
        chk("core0 D-miss data",      dut.cpu[0].core.gr[21], 32'h12345678);
        chk("core0 sem acquired",     dut.cpu[0].core.gr[22], 0);
        chk("core0 CTRL reads EN",    dut.cpu[0].core.gr[23], 1);
        chk("PIU-05 no d_req",        piu_dreq, 0);
        chk("PIU-01 CYCLE == cycles",  dut.cpu[0].core.u_piu.cyc, cycles0);
        chk("PIU-13 ISTALL == core",   dut.cpu[0].core.u_piu.istall, dut.cpu[0].core.istall_cycles);
        chk("PIU-13 DSTALL == core",   dut.cpu[0].core.u_piu.dstall, dut.cpu[0].core.stall_cycles);
        chk("PIU-11 ISTALL > 0",       dut.cpu[0].core.u_piu.istall > 0, 1);
        chk("PIU-11 DSTALL > 0",       dut.cpu[0].core.u_piu.dstall > 0, 1);
        chk("PIU-07 PKT",              dut.cpu[0].core.u_piu.pkt, 13);
        chk("PIU-09 LD (+3 sem +SEMFAIL)", dut.cpu[0].core.u_piu.ld, 8);
        chk("PIU-09 ST (mem+sem)",     dut.cpu[0].core.u_piu.st, 2);
        chk("PIU-11 XNOP",             dut.cpu[0].core.u_piu.xnop, 3);
        chk("PIU-13 cycle classes",    dut.cpu[0].core.u_piu.cyc,
            dut.cpu[0].core.u_piu.pkt + dut.cpu[0].core.u_piu.xnop + dut.cpu[0].core.u_piu.istall
            + dut.cpu[0].core.u_piu.dstall + 1);
        chk("PIU-14 core0 EN kept",    dut.cpu[0].core.u_piu.en, 1);
        chk("PIU-14 core1 EN off",     dut.cpu[1].core.u_piu.en, 0);
        chk("PIU-14 core1 PKT",        dut.cpu[1].core.u_piu.pkt, 4);
        chk("PIU-14 core1 LD",         dut.cpu[1].core.u_piu.ld, 1);
        chk("core1 sem found taken",   dut.cpu[1].core.gr[22], 1);
        chk("PIU-17 core0 SEMSTALL",   dut.cpu[0].core.u_piu.semstall, 0);
        chk("PIU-17 core1 SEMSTALL",   dut.cpu[1].core.u_piu.semstall, 1);
        chk("PIU-17 core1 SEMSTALL <= DSTALL", dut.cpu[1].core.u_piu.semstall <= dut.cpu[1].core.u_piu.dstall, 1);
        chk("PIU-14 core1 ST",         dut.cpu[1].core.u_piu.st, 1);
        chk("core0 ldb hw_sem reads 1s", dut.cpu[0].core.gr[24], 32'hFFFFFFFF);
        chk("core0 lock 1 acquired",   dut.cpu[0].core.gr[25], 0);
        chk("core0 lock 1 busy",       dut.cpu[0].core.gr[26], 1);
        chk("PIU-18 core0 SEMFAIL (busy ldw only)", dut.cpu[0].core.u_piu.semfail, 1);
        chk("PIU-18 core1 SEMFAIL (lost the race)", dut.cpu[1].core.u_piu.semfail, 1);
        chk("PIU-18 SEMFAIL read at 0x5C", dut.cpu[0].core.gr[27], 1);
        $display("=============================================================");
        $display("PIU SYS TEST: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: PIU SYS PASSED"); else $display("RESULT: PIU SYS FAILED");
        $finish;
    end
endmodule
