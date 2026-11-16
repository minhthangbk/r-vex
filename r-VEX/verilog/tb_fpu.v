// tb_fpu.v (round r27, fpu.md FP-01..06, FP-12): every vector of toolchain/sim/f32ref.py (op a b expected, hex) through
// fpu.v, combinational (PIPE = 0) and two-stage (PIPE = 1: load the stage register, then change every input: the result
// comes from the register only).
// Run: python3 toolchain/sim/f32ref.py vectors N SEED > fpu_vec.txt; iverilog -g2012 -I . -o tb_fpu.vvp tb_fpu.v fpu.v
`include "rvex_defs.vh"
module tb_fpu;
    reg  [6:0]  op;  reg [31:0] a, b, want; wire [31:0] r; wire v;
    integer fd, n, k, bad, nv, sel;
    reg clk = 1'b0, ld = 1'b0; reg [6:0] op_s; reg [31:0] a_s, b_s; wire [31:0] r2; wire v2;
    fpu dut(.clk(1'b0), .ld(1'b0), .fpuop(op), .src1(a), .src2(b), .result(r), .out_valid(v));
    fpu #(.PIPE(1)) dut2(.clk(clk), .ld(ld), .fpuop(op), .src1(a), .src2(b), .result(r2), .out_valid(v2));
    initial begin
        fd = $fopen("fpu_vec.txt", "r"); bad = 0; nv = 0;
        if (fd == 0) begin $display("FPU FAIL no fpu_vec.txt"); $finish; end
        while (!$feof(fd)) begin
            n = $fscanf(fd, "%d %h %h %h\n", sel, a, b, want);
            if (n == 4) begin
                op = sel == 0 ? `FPU_FADD : sel == 1 ? `FPU_FSUB : `FPU_FMUL;
                #1;
                nv = nv + 1;
                if (r !== want || !v) begin
                    bad = bad + 1;
                    if (bad <= 10) $display("FPU MISMATCH op %0d a %08h b %08h got %08h want %08h", sel, a, b, r, want);
                end
                // PIPE = 1: stage 1 into the register, then other inputs (and no load): the result must not move
                ld = 1'b1; #1 clk = 1'b1; #1 clk = 1'b0; ld = 1'b0;
                op_s = op; a_s = a; b_s = b;
                op = (sel == 2) ? `FPU_FADD : `FPU_FMUL; a = ~a; b = b ^ 32'h5A5A5A5A;
                #1 clk = 1'b1; #1 clk = 1'b0; #1;
                if (r2 !== want) begin
                    bad = bad + 1;
                    if (bad <= 10) $display("FPU PIPE MISMATCH op %0d a %08h b %08h got %08h want %08h", sel, a_s, b_s, r2, want);
                end
                op = op_s; a = a_s; b = b_s;
            end
        end
        // an opcode that is not an FPU op is not valid (the core keeps its old result path)
        op = `MUL_MPYLL; a = 32'h3F800000; b = 32'h3F800000; #1;
        if (v) begin bad = bad + 1; $display("FPU MISMATCH out_valid for MPYLL"); end
        $display("FPU %s vectors %0d mismatches %0d", bad == 0 ? "PASS" : "FAIL", nv, bad);
        $display("RESULT: FPU %s", (bad == 0 && nv > 0) ? "PASSED" : "FAILED");
        $finish;
    end
endmodule
