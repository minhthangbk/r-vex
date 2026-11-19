`timescale 1ns/1ps
`include "rvex_defs.vh"
`include "slot8.vh"

// tb_r27_fp_hold.v (round r27, Codex R54 F7; fpu.md FP-12, piu.md PIU-21): the one-cycle FP hold against the other
// hold paths of rvex_core with external memories.
//   P0  FADD in slot 1 + a word store in slot 3, the store D-stalled for 2 cycles: no memory request in the held
//       cycle, one store handshake, the hold is not repeated after the D stall (fp_ph kept)
//   P1  FMUL in slot 1, fetched with a 2-cycle I stall: no hold during the I stall, one hold after it
//   P2  FADD in slot 1 + a taken GOTO in slot 0: the PC moves only when the packet commits; P3 never runs
//   P4  FSUB in slot 1 + XNOP 2 in slot 0: one hold, then the XNOP bubbles
//   P5  STOP in slot 0 (FADD in slot 1): no hold
//   FPSTALL = 4 (P0, P1, P2, P4); after STOP the core drains: no I or D request stays up (rule RR-11), with a cycle cap.
module tb_r27_fp_hold;
    reg clk = 0, reset = 1, start = 0;
    reg [255:0] i_rdata;
    reg i_ready = 1, d_ready = 0;
    wire done, d_req, d_we, d_word, i_req;
    wire [31:0] cycles, d_addr, d_wdata;
    wire [11:0] i_addr;
    integer pass = 0, fail = 0, cyc = 0, hs = 0, req_held = 0, dwait = 0, iwait = 0, fpst_istall = 0;
    reg i_done = 0;
    reg [31:0] st_addr = 32'hx, st_data = 32'hx;

    rvex_core #(.EXT_IMEM(1), .EXT_DMEM(1)) dut(
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(d_word), .d_rdata(32'd0),
        .d_ready(d_ready), .stall_cycles(), .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata), .i_ready(i_ready),
        .istall_cycles());
    always #5 clk = ~clk;

    function [31:0] rr; input [6:0] op; input [5:0] d, a, b; begin rr = {op, `NO_IMM, d, a, b, 5'b0}; end endfunction
    function [31:0] mst; input [6:0] op; input [5:0] s, b; input [8:0] off; begin mst = {op, `NO_IMM, s, b, off, 2'b00}; end endfunction
    function [31:0] j12; input [6:0] op; input [11:0] n; begin j12 = {op, 8'b0, n, 5'b0}; end endfunction
    localparam [31:0] NOP = 0;
    task chk; input [511:0] name; input [63:0] got, expected; begin
        if (got === expected) begin pass = pass + 1; $display("PASS %0s got=%0h", name, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%0h expected=%0h", name, got, expected); end
    end endtask

    always @(*) begin
        case (i_addr)
          12'd0: i_rdata = pk8(mst(`MEM_STW, 6'd5, 6'd0, 9'd16), NOP, rr(`FPU_FADD, 6'd6, 6'd4, 6'd5), NOP);
          12'd1: i_rdata = pk8(NOP, NOP, rr(`FPU_FMUL, 6'd7, 6'd4, 6'd5), NOP);
          12'd2: i_rdata = pk8(NOP, NOP, rr(`FPU_FADD, 6'd9, 6'd4, 6'd4), j12(`CTRL_GOTO, 12'd4));
          12'd3: i_rdata = pk8(NOP, NOP, rr(`FPU_FADD, 6'd10, 6'd4, 6'd4), NOP);
          12'd4: i_rdata = pk8(NOP, NOP, rr(`FPU_FSUB, 6'd8, 6'd6, 6'd7), j12(`CTRL_XNOP, 12'd2));
          12'd5: i_rdata = pk8(NOP, NOP, rr(`FPU_FADD, 6'd11, 6'd4, 6'd4), {`OP_STOP, 25'd0});
          default: i_rdata = 256'd0;
        endcase
    end

    // memory models: the first D request waits 2 cycles; the first fetch of P1 waits 2 cycles
    always @(posedge clk) if (!reset && start && !done) begin
        cyc <= cyc + 1;
        if (dut.fp_wait && d_req) req_held <= req_held + 1;          // a request in the held cycle (must stay 0)
        if (d_req && d_ready) begin hs <= hs + 1; st_addr <= d_addr; st_data <= d_wdata; end
        if (d_req && !d_ready) dwait <= dwait + 1;
        if (!i_ready && dut.fp_wait) fpst_istall <= fpst_istall + 1;  // a hold during the I stall (must stay 0)
        if (i_req && i_addr == 12'd1 && !i_done) begin
            if (iwait < 2) iwait <= iwait + 1; else i_done <= 1;
        end
    end
    always @(*) begin
        d_ready = (dwait >= 2);
        i_ready = !(i_req && i_addr == 12'd1 && !i_done && iwait < 2);
    end

    initial begin
        repeat (2) @(posedge clk);
        reset = 0; #1;
        dut.gr[4] = 32'h3FC00000; dut.gr[5] = 32'h40100000;        // 1.5, 2.25
        dut.gr[10] = 32'h0000CAFE;
        start = 1;
        while (!done && cyc < 200) @(posedge clk);
        #1;
        chk("run ends by STOP within the cap", done, 1);
        chk("P0 fadd committed", dut.gr[6], 32'h40700000);           // 3.75
        chk("P0 one store handshake", hs, 1);
        chk("P0 store address", st_addr, 32'd16);
        chk("P0 store data", st_data, 32'h40100000);
        chk("P0 D stall cycles", dwait, 2);
        chk("no memory request in a held cycle", req_held, 0);
        chk("P1 fmul committed after the I stall", dut.gr[7], 32'h40580000);   // 3.375
        chk("P1 no hold during the I stall", fpst_istall, 0);
        chk("P1 I stall cycles", dut.u_piu.istall, 2);
        chk("P2 fadd with the taken goto", dut.gr[9], 32'h40400000); // 3.0
        chk("P3 skipped by the goto", dut.gr[10], 32'h0000CAFE);
        chk("P4 fsub with the XNOP", dut.gr[8], 32'h3EC00000);       // 0.375
        chk("PIU-21 FPSTALL = 4 held packets", dut.u_piu.fpstall, 4);
        chk("PIU XNOP cycles", dut.u_piu.xnop, 2);
        chk("PIU DSTALL cycles", dut.u_piu.dstall, 2);
        // drain (RR-11): nothing requested after STOP
        repeat (3) @(posedge clk); #1;
        chk("drain: no I request after STOP", i_req, 0);
        chk("drain: no D request after STOP", d_req, 0);
        if (fail == 0) $display("RESULT: R27 FP HOLD PASSED (%0d checks)", pass);
        else $display("RESULT: R27 FP HOLD FAILED (%0d of %0d)", fail, pass + fail);
        $finish;
    end
endmodule
