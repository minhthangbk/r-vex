// Codex round-10 PIU edge cases. Expectations come from reviews/contracts/piu.md.
`timescale 1ns/1ps

module tb_codex_piu_edges;
    reg clk = 0, reset = 1;
    reg active = 0, commit = 0, xnop_c = 0, istall_c = 0, dstall_c = 0;
    reg [6:0] op0 = 0, op1 = 0, op2 = 0, op3 = 0;
    reg ctrl0 = 0, taken0 = 0, mul1 = 0, mul2 = 0, ld3 = 0, st3 = 0;
    reg [5:0] addr = 0;
    reg we = 0;
    reg [31:0] wdata = 0;
    wire [31:0] rdata;
    integer pass = 0, fail = 0, a;

    rvex_piu dut(.clk(clk), .reset(reset), .active(active), .commit(commit),
        .xnop_c(xnop_c), .istall_c(istall_c), .dstall_c(dstall_c), .semstall_c(1'b0), .semfail3(1'b0), .misal3(1'b0),
        .op0(op0), .op1(op1), .op2(op2), .op3(op3), .ctrl0(ctrl0),
        .taken0(taken0), .mul1(mul1), .mul2(mul2), .ld3(ld3), .st3(st3),
        .addr(addr), .rdata(rdata), .we(we), .wdata(wdata));

    always #5 clk = ~clk;

    task chk;
        input [255:0] name;
        input [63:0] got, expected;
        begin
            if (got === expected) begin
                pass = pass + 1;
                $display("PASS %0s got=%0d", name, got);
            end else begin
                fail = fail + 1;
                $display("FAIL %0s got=%0d (0x%0h) expected=%0d (0x%0h)",
                         name, got, got, expected, expected);
            end
        end
    endtask

    initial begin
        repeat (2) @(posedge clk);
        reset = 0;

        // PIU-04: CLR and EN in one store clears all increments from this cycle,
        // while PIU-03 makes EN=1 for the following cycle.
        active = 1; commit = 1; op0 = 7'h41;
        we = 1; addr = 0; wdata = 3;
        @(posedge clk); #1;
        chk("CLR+EN en", dut.en, 1);
        chk("CLR+EN cycle clear wins", dut.cyc, 0);
        chk("CLR+EN packet clear wins", dut.pkt, 0);
        we = 0; commit = 0; op0 = 0;

        // PIU-06: carry from CYCLE to CYCLEH is a real 64-bit increment.
        dut.cyc = 64'h0000_0000_ffff_ffff;
        active = 1;
        @(posedge clk); #1;
        chk("CYCLE low carry", dut.cyc[31:0], 0);
        chk("CYCLEH carry", dut.cyc[63:32], 1);
        active = 0;

        // PIU-02: every unimplemented word offset, not just 0x54, reads zero.
        for (a = 6'h19; a <= 6'h3f; a = a + 1) begin   // 0x54 MULPKT, 0x58 SEMSTALL after R10 (PIU-16/17), 0x5C SEMFAIL r10 (PIU-18), 0x60 MISALIGN r21 (PIU-19)
            addr = a[5:0]; #1;
            if (rdata !== 0) begin
                fail = fail + 1;
                $display("FAIL reserved offset 0x%0h returned 0x%0h", a << 2, rdata);
            end else pass = pass + 1;
        end

        // PIU-08 names only opcodes 1..11 as multiplies. Undefined opcodes
        // 12..15 are non-NOP syllables, hence PIU-07/12 put them in ALU.
        reset = 1; @(posedge clk); #1; reset = 0;
        active = 1; commit = 1; op1 = 7'd12; mul1 = 1;
        @(posedge clk); #1;
        chk("opcode 12 OPS", dut.ops, 1);
        chk("opcode 12 MUL16", dut.mul16, 0);
        chk("opcode 12 MUL32", dut.mul32, 0);
        chk("opcode 12 ALU", dut.alu, 1);

        $display("CODEX PIU EDGES: %0d passed, %0d failed", pass, fail);
        if (fail == 0) $display("RESULT: CODEX PIU EDGES PASSED");
        else $display("RESULT: CODEX PIU EDGES FAILED");
        $finish;
    end
endmodule
