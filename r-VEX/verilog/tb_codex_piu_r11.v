// Codex round-11 directed PIU-08/16/17 contract edges.
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_codex_piu_r11;
    reg clk = 0, reset = 1, start = 0;
    reg [127:0] i_rdata = 0;
    reg i_ready = 0, d_ready = 1;
    wire done, d_req, d_we, d_word, i_req;
    wire [31:0] cycles, d_addr, d_wdata;
    wire [11:0] i_addr;
    integer pass = 0, fail = 0;

    rvex_core #(.EXT_IMEM(1), .EXT_DMEM(1)) dut(
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata),
        .d_word(d_word), .d_rdata(32'hffff_ffff), .d_ready(d_ready),
        .stall_cycles(), .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata),
        .i_ready(i_ready), .istall_cycles());
    always #5 clk = ~clk;

    function [31:0] rr;
        input [6:0] op; input [5:0] d, a, b;
        begin rr = {op, 2'b00, d, a, b, 5'b0}; end
    endfunction
    function [31:0] mi;
        input [6:0] op; input [5:0] d, a; input [8:0] off;
        begin mi = {op, 2'b01, d, a, off, 2'b00}; end
    endfunction
    task chk;
        input [255:0] name; input [63:0] got, expected;
        begin
            if (got === expected) begin pass = pass + 1; $display("PASS %0s got=%0d", name, got); end
            else begin fail = fail + 1; $display("FAIL %0s got=%0d expected=%0d", name, got, expected); end
        end
    endtask

    initial begin
        repeat (2) @(posedge clk);
        reset = 0; start = 1; #1;

        // Build GR10 = 0x0000ff00 while the instruction side is stalled.
        // Even though i_rdata happens to contain a hw_sem load, issue/d_req are
        // suppressed, so an I-miss cannot simultaneously become SEMSTALL.
        dut.gr[10] = 32'h0000_ff00;
        i_rdata = {mi(`MEM_LDW, 6'd20, 6'd10, 9'd0), 96'd0};
        repeat (3) @(posedge clk); #1;
        chk("I-miss suppresses d_req", d_req, 0);
        chk("I-miss is not SEMSTALL", dut.u_piu.semstall, 0);

        // A sub-word hw_sem access completes immediately in rvex_system. Model
        // that ready response here: it commits and does not count as a stall.
        i_ready = 1; d_ready = 1;
        i_rdata = {mi(`MEM_LDBU, 6'd20, 6'd10, 9'd1), 96'd0};
        @(posedge clk); #1;
        chk("sub-word hw_sem no DSTALL", dut.u_piu.dstall, 0);
        chk("sub-word hw_sem no SEMSTALL", dut.u_piu.semstall, 0);

        // Two defined multiplies in one committed packet count as two MUL ops
        // but one MULPKT.
        i_rdata = {32'd0, rr(`MUL_MPYL, 6'd22, 6'd0, 6'd0),
                   rr(`MUL_MPYLL, 6'd21, 6'd0, 6'd0), 32'd0};
        @(posedge clk); #1;
        chk("dual MUL MUL16", dut.u_piu.mul16, 1);
        chk("dual MUL MUL32", dut.u_piu.mul32, 1);
        chk("dual MUL one MULPKT", dut.u_piu.mulpkt, 1);

        // Core is_mul is deliberately broad, but PIU-08 narrows 12..15 back
        // to ALU and PIU-16 must use that same narrowed definition.
        i_rdata = {64'd0, rr(7'd12, 6'd23, 6'd0, 6'd0), 32'd0};
        @(posedge clk); #1;
        chk("undefined opcode is ALU", dut.u_piu.alu, 1);
        chk("undefined opcode not MULPKT", dut.u_piu.mulpkt, 1);

        // A word hw_sem request held for two cycles is two SEMSTALL cycles.
        i_rdata = {mi(`MEM_LDW, 6'd24, 6'd10, 9'd0), 96'd0};
        d_ready = 0;
        repeat (2) @(posedge clk); #1;
        chk("held hw_sem DSTALL", dut.u_piu.dstall, 2);
        chk("held hw_sem SEMSTALL", dut.u_piu.semstall, 2);
        d_ready = 1; @(posedge clk); #1;
        chk("completed hw_sem no extra SEMSTALL", dut.u_piu.semstall, 2);

        $display("CODEX PIU R11: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: CODEX PIU R11 PASSED"); else $display("RESULT: CODEX PIU R11 FAILED");   // added by Claude (final verify F5)
        $finish;
    end
endmodule
