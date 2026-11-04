//======================================================================
// Codex R21 (task b): the PIU window decode for a system with AW != 16
//----------------------------------------------------------------------
// R20 C-1 moved semfail3 / semstall_c to &mem_ea[DMEM_AW-1:8]. piu_hit
// (rvex_core.v:163) still decodes the fixed field mem_ea[15:8] == 8'hFE and
// ignores bits [DMEM_AW-1:16]. With rvex_system #(.AW(17)) (128 KB) the
// address 0x1FE00 is ordinary cached memory, but the core sends it to the PIU:
// a stw there writes the PIU CTRL register (here: value 6 = CLR + no EN) and is
// lost to memory; the ldw back reads the PIU, not the stored word.
// dut16 (AW = 16) runs the same program as a reference: there 0x1FE00 aliases
// 0xFE00 (the memory wraps at 64 KB), so the PIU access is correct.
// Checks (dut17): the word stored at 0x1FE00 reads back, the PIU stays enabled.
// Build: iverilog -g2012 -I . -o x.vvp rvex_defs.vh alu.v mul.v mem_unit.v
//   ctrl_unit.v rvex_core.v l1i_cache.v l1d_cache.v coh_bus.v l2_cache.v
//   hw_sem.v rvex_system.v tb_codex_r21_piuaw.v
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_codex_r21_piuaw;
    reg clk = 0, reset = 1, start = 0;
    wire a0, a1, b0, b1; wire [31:0] ca0, ca1, cb0, cb1;
    integer pass = 0, bad = 0, k;

    rvex_system dut16(.clk(clk), .reset(reset), .start0(start), .start1(1'b0),
        .done0(a0), .done1(a1), .cycles0(ca0), .cycles1(ca1));
    rvex_system #(.AW(17)) dut17(.clk(clk), .reset(reset), .start0(start), .start1(1'b0),
        .done0(b0), .done1(b1), .cycles0(cb0), .cycles1(cb1));
    always #5 clk = ~clk;

    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s,b; input [8:0] off; begin
        mst = {op,`NO_IMM,s,b,off,2'b00}; end endfunction
    localparam [31:0] NOP = 32'd0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};
    task chk; input [255:0] nm; input [63:0] got, exp; begin
        if (got === exp) begin pass = pass + 1; $display("  PASS %0s : %0d", nm, got); end
        else begin bad = bad + 1; $display("  FAIL %0s : got %0d (0x%0h) exp %0d (0x%0h)", nm, got, got, exp, exp); end
    end endtask
    task PR; input [11:0] a; input [31:0] s0,s1,s2,s3; begin
        dut16.cpu[0].l1i.rom[a] = {s3,s2,s1,s0}; dut17.cpu[0].l1i.rom[a] = {s3,s2,s1,s0}; end endtask

    initial begin
        for (k = 0; k < 4096; k = k + 1) begin
            dut16.cpu[0].l1i.rom[k] = 128'd0; dut16.cpu[1].l1i.rom[k] = 128'd0;
            dut17.cpu[0].l1i.rom[k] = 128'd0; dut17.cpu[1].l1i.rom[k] = 128'd0;
        end
        for (k = 0; k < 16384; k = k + 1) dut16.l2.mem[k] = 32'd0;
        for (k = 0; k < 32768; k = k + 1) dut17.l2.mem[k] = 32'd0;
        PR(0, ri(`ALU_ADD,6'd13,6'd0,9'hFF), ri(`ALU_ADD,6'd14,6'd0,9'd6), NOP, NOP);
        PR(1, ri(`ALU_SHL,6'd13,6'd13,9'd9), NOP, NOP, NOP);        // r13 = 0x1FE00
        PR(2, NOP, NOP, NOP, mst(`MEM_STW,6'd14,6'd13,9'h00));      // stw 6 -> 0x1FE00
        PR(3, NOP, NOP, NOP, ri(`MEM_LDW,6'd20,6'd13,9'h00));       // ldw   <- 0x1FE00
        PR(4, STOP, NOP, NOP, NOP);
        repeat (2) @(posedge clk);
        reset = 0; start = 1;
        k = 0;
        while (!(a0 === 1'b1 && b0 === 1'b1) && k < 4000) begin @(posedge clk); k = k + 1; end
        repeat (2) @(posedge clk);
        $display("============= Codex R21 PIU window with AW = 17 =============");
        chk("done", a0 && b0, 1);
        chk("AW16 alias: PIU EN cleared", dut16.cpu[0].core.u_piu.en, 0);
        chk("AW17 ldw 0x1FE00 == stored 6", dut17.cpu[0].core.gr[20], 6);
        chk("AW17 PIU EN still 1", dut17.cpu[0].core.u_piu.en, 1);
        $display("CODEX R21 PIUAW: %0d passed, %0d failed", pass, bad);
        if (bad == 0) $display("RESULT: CODEX R21 PIUAW PASSED");
        else $display("RESULT: CODEX R21 PIUAW FAILED (piu_hit decodes mem_ea[15:8], not [DMEM_AW-1:8])");
        $finish;
    end
endmodule
