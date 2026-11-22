// tb_fpu_pipe.v (round r31 B4, reviews/contracts/fpu.md FP-13 / FP-14 / FP-15, piu.md PIU-21): the pipelined FPUs of
// slots 4-7 through rvex_core, packets driven one per cycle on the external I-port.
//   T1 FP-13: four FP ops in slots 4-7 commit in their first cycle (no hold, FPSTALL 0); their results land at the
//      next edge (not at the commit), the NOP packet after them is not held
//   T2 FP-14 RAW: an FP result read by the next packet's slot-0 add: one held cycle (FPSTALL + 1), the add reads the
//      new value
//   T3 FP-14 WAW: the next packet writes the FP destination: the later write wins
//   T4 FP-14 pair: the FP op writes r25, the next packet's dot4 reads the pair r24:r25: held, the dot4 reads it
//   T5 an independent packet after an FP op is not held
//   T6 FP-08: FPU codes in slots 1-2 write nothing; T7 FP-14: a store after an FP op is presented once with the result;
//   T8 FP-14: a reader that waits on the fetch is not an FP interlock
// Mutants (mutate.py): FP-M12 (T6), FP-M15 (T7), FP-M16 (T8), FP-M13 (stage register loads on the held cycle only: T1 results wrong), FP-M14 (no
// interlock: T2 / T4 read the old values), FP-M16 (pair halves not matched: T4), FP-M17 (FPSTALL not counted).
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_fpu_pipe;
    reg clk = 0, reset = 1, start = 0;
    reg [255:0] i_rdata = 0;
    wire done, d_req, d_we, d_word, i_req;
    wire [31:0] cycles, d_addr, d_wdata;
    wire [11:0] i_addr;
    integer pass = 0, fail = 0;
    reg [11:0] pc0;
    reg i_ready = 1;
    integer nst = 0;
    reg [31:0] st_data = 0;
    always @(posedge clk) if (d_req && d_we) begin nst = nst + 1; st_data = d_wdata; end   // T7: every store presented

    rvex_core #(.EXT_IMEM(1), .EXT_DMEM(1)) dut(
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(d_word), .d_rdata(32'd0),
        .d_ready(1'b1), .stall_cycles(), .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata), .i_ready(i_ready),
        .istall_cycles());
    always #5 clk = ~clk;

    function [31:0] rr; input [6:0] op; input [5:0] d, a, b; begin rr = {op, 2'b00, d, a, b, 5'b0}; end endfunction
    function [31:0] ri; input [6:0] op; input [5:0] d, a; input [8:0] im; begin ri = {op, 2'b01, d, a, im, 2'b00}; end endfunction
    function [255:0] pk; input [31:0] s7, s6, s5, s4, s3, s2, s1, s0; begin pk = {s7, s6, s5, s4, s3, s2, s1, s0}; end endfunction
    task chk; input [511:0] name; input [63:0] got, expected; begin
        if (got === expected) begin pass = pass + 1; $display("PASS %0s got=%08x", name, got); end
        else begin fail = fail + 1; $display("FAIL %0s got=%08x expected=%08x", name, got, expected); end
    end endtask
    task cyc; begin @(posedge clk); #1; end endtask

    initial begin
        repeat (2) @(posedge clk);
        reset = 0; start = 1; #1;
        i_rdata = 256'd0; cyc;
        dut.gr[4] = 32'h3FC00000; dut.gr[5] = 32'h40100000;                     // 1.5, 2.25
        dut.gr[24] = 32'h00010001; dut.gr[25] = 32'h0; dut.gr[26] = 32'h00010001; dut.gr[27] = 32'h00010001;
        // T1: fadd r10 = 3.75 (slot 4), fmul r11 = 3.375 (5), fsub r12 = 0.75 (6), fadd r13 = 3.0 (7)
        i_rdata = pk(rr(`FPU_FADD, 6'd13, 6'd4, 6'd4), rr(`FPU_FSUB, 6'd12, 6'd5, 6'd4), rr(`FPU_FMUL, 6'd11, 6'd4, 6'd5),
                     rr(`FPU_FADD, 6'd10, 6'd4, 6'd5), 32'd0, 32'd0, 32'd0, 32'd0);
        pc0 = i_addr; cyc;
        chk("FP-13 the FP packet commits in its first cycle (PC moves on)", i_addr, pc0 + 12'd1);
        chk("FP-13 no hold: FPSTALL 0", dut.u_piu.fpstall, 0);
        chk("FP-13 the result is not written at the commit", dut.gr[10], 32'h0);
        i_rdata = 256'd0; cyc;                                                    // NOP: fields 0, not held
        chk("FP-13 slot 4 fadd lands at the next edge", dut.gr[10], 32'h40700000);
        chk("FP-13 slot 5 fmul", dut.gr[11], 32'h40580000);
        chk("FP-13 slot 6 fsub", dut.gr[12], 32'h3F400000);
        chk("FP-13 slot 7 fadd", dut.gr[13], 32'h40400000);
        chk("FP-13 the NOP packet after it is not held", dut.u_piu.fpstall, 0);
        // T2 RAW: fadd r14 = 3.75 (slot 4), then add r15 = r14 + r0 (slot 0)
        i_rdata = pk(32'd0, 32'd0, 32'd0, rr(`FPU_FADD, 6'd14, 6'd4, 6'd5), 32'd0, 32'd0, 32'd0, 32'd0); cyc;
        i_rdata = pk(32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, rr(`ALU_ADD, 6'd15, 6'd14, 6'd0)); cyc;
        chk("FP-14 RAW: the reader is held one cycle (FPSTALL 1)", dut.u_piu.fpstall, 1);
        cyc;
        chk("FP-14 RAW: the reader sees the FP result", dut.gr[15], 32'h40700000);
        // T3 WAW: fmul r16 = 3.375 (slot 5), then add r16 = r0 + 5 (slot 0): the later write wins
        i_rdata = pk(32'd0, 32'd0, rr(`FPU_FMUL, 6'd16, 6'd4, 6'd5), 32'd0, 32'd0, 32'd0, 32'd0, 32'd0); cyc;
        i_rdata = pk(32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, ri(`ALU_ADD, 6'd16, 6'd0, 9'd5)); cyc; cyc;
        chk("FP-14 WAW: the later write of the FP destination wins", dut.gr[16], 32'd5);
        // T4 pair: fadd r25 = 3.75 (slot 6), then dot4 r30 = r24:r25 . r26:r27 (slot 4): 1 + 1 + 0x4070 + 0
        i_rdata = pk(32'd0, rr(`FPU_FADD, 6'd25, 6'd4, 6'd5), 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0); cyc;
        i_rdata = pk(32'd0, 32'd0, 32'd0, rr(`SQ_SIMD4, 6'd30, 6'd24, 6'd26), 32'd0, 32'd0, 32'd0, 32'd0); cyc; cyc;
        chk("FP-14 pair: the dot4 reads the FP result through the odd half", dut.gr[30], 32'd16498);
        // T5 independent: fsub r31 = 0.75 (slot 7), then add r32 = r4 + r0 (slot 0): no hold
        i_rdata = pk(rr(`FPU_FSUB, 6'd31, 6'd5, 6'd4), 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0); cyc;
        pc0 = dut.u_piu.fpstall;
        i_rdata = pk(32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, rr(`ALU_ADD, 6'd32, 6'd4, 6'd0)); cyc;
        chk("FP-14 an independent packet is not held", dut.u_piu.fpstall, pc0);
        chk("FP-14 the independent add", dut.gr[32], 32'h3FC00000);
        chk("FP-13 slot 7 fsub of T5", dut.gr[31], 32'h3F400000);
        chk("PIU-20 FLOP counts the FP ops of slots 4-7", dut.u_piu.flop, 8);
        chk("PIU-21 FPSTALL counts the two interlocked readers and the pair", dut.u_piu.fpstall, 3);
        // T6 FP-08: FPU codes in slots 1 and 2 (no FPU there) write nothing
        dut.gr[18] = 32'h12345678; dut.gr[19] = 32'h9ABCDEF0;                    // a write of 0 must show
        i_rdata = pk(32'd0, 32'd0, 32'd0, 32'd0, 32'd0, rr(`FPU_FMUL, 6'd19, 6'd4, 6'd5), rr(`FPU_FADD, 6'd18, 6'd4, 6'd5), 32'd0); cyc;
        i_rdata = 256'd0; cyc; cyc;
        chk("FP-08 an FPU code in slot 1 writes nothing", dut.gr[18], 32'h12345678);
        chk("FP-08 an FPU code in slot 2 writes nothing", dut.gr[19], 32'h9ABCDEF0);
        // T7 FP-14: fadd r17 = 3.75 (slot 5), then stw r17 -> [r0 + 0] (slot 3): the store is presented once, after
        // the interlock, with the FP result (no memory access in the held cycle)
        i_rdata = pk(32'd0, 32'd0, rr(`FPU_FADD, 6'd17, 6'd4, 6'd5), 32'd0, 32'd0, 32'd0, 32'd0, 32'd0); cyc;
        nst = 0;
        i_rdata = pk(32'd0, 32'd0, 32'd0, 32'd0, {`MEM_STW, `NO_IMM, 6'd17, 6'd0, 9'd0, 2'b0}, 32'd0, 32'd0, 32'd0); cyc; cyc;
        i_rdata = 256'd0; cyc;
        chk("FP-14 the store after an FP op is presented once", nst, 1);
        chk("FP-14 the store writes the FP result", st_data, 32'h40700000);
        // T8 FP-14: fsub r20 = 0.75 (slot 4); the next packet (add r21 = r20 + r0) is still being fetched (i_ready 0)
        // while r20 is pending: a fetch stall, not an FP interlock (FPSTALL unchanged), then it issues at once
        i_rdata = pk(32'd0, 32'd0, 32'd0, rr(`FPU_FSUB, 6'd20, 6'd5, 6'd4), 32'd0, 32'd0, 32'd0, 32'd0); cyc;
        pc0 = dut.u_piu.fpstall;
        i_rdata = pk(32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, rr(`ALU_ADD, 6'd21, 6'd20, 6'd0)); i_ready = 0; cyc;
        chk("FP-14 a fetch-stalled reader is not counted as an FP interlock", dut.u_piu.fpstall, pc0);
        i_ready = 1; cyc;
        i_rdata = 256'd0; cyc;
        chk("FP-14 the fetch-stalled reader then reads the FP result", dut.gr[21], 32'h3F400000);
        chk("FP-14 no FP interlock after the fetch stall", dut.u_piu.fpstall, pc0);
        i_rdata = {224'd0, `OP_STOP, 25'd0};
        repeat (4) begin if (!done) @(posedge clk); end
        #1;
        chk("drain: STOP ends the core", done, 1'b1);
        $display("FPU PIPE: %0d passed, %0d failed", pass, fail);
        $display("RESULT: FPU PIPE %s", fail == 0 ? "PASSED" : "FAILED");
        $finish;
    end
endmodule
