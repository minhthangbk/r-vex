// tb_fma.v (round r32, fpu.md FP-18 / FP-20): every vector of toolchain/sim/f32ref.py fmavectors (op a b c expected, hex)
// through the fused lane fma32.v (op 0 fmadd c + a*b, 1 fmsub c - a*b: product negated, 2 fmrsub a*b - c: c negated),
// and every fpu_vec.txt vector through the lane identities of the pair forms (fadd = fma(a, 1, b), fsub = fma(a, 1, -b),
// fmul = fma(a, b, -0)). Run: python3 f32ref.py fmavectors N SEED > fma_vec.txt (wc -l > fma_vec.n), with fpu_vec.txt.
module tb_fma;
    reg [31:0] a, b, c, want; reg np, nc; wire [31:0] r;
    integer fd, n, bad, nv, nl, sel, nexp;
    fma32 dut(.a(a), .b(b), .c(c), .np(np), .nc(nc), .r(r));
    initial begin
        bad = 0; nv = 0; nl = 0;
        fd = $fopen("fma_vec.txt", "r");
        if (fd == 0) begin $display("FMA FAIL no fma_vec.txt"); $finish; end
        while (!$feof(fd)) begin
            n = $fscanf(fd, "%d %h %h %h %h\n", sel, a, b, c, want);
            if (n == 5) begin
                np = (sel == 1); nc = (sel == 2); #1;
                nv = nv + 1;
                if (r !== want) begin
                    bad = bad + 1;
                    if (bad <= 10) $display("FMA MISMATCH op %0d a %08h b %08h c %08h got %08h want %08h", sel, a, b, c, r, want);
                end
            end
        end
        $fclose(fd);
        nexp = -1; fd = $fopen("fma_vec.n", "r");
        if (fd != 0) begin n = $fscanf(fd, "%d", nexp); $fclose(fd); end
        if (nv != nexp || nexp < 1000) begin bad = bad + 1; $display("FMA FAIL vectors read %0d, expected %0d", nv, nexp); end
        // FP-20 lane identities on the scalar vectors
        fd = $fopen("fpu_vec.txt", "r");
        if (fd == 0) begin $display("FMA FAIL no fpu_vec.txt"); $finish; end
        while (!$feof(fd)) begin
            n = $fscanf(fd, "%d %h %h %h\n", sel, a, b, want);
            if (n == 4) begin
                if (sel == 2) begin c = 32'h80000000; np = 0; nc = 0; end
                else begin c = b; b = 32'h3F800000; np = 0; nc = (sel == 1); end
                #1; nl = nl + 1;
                if (r !== want) begin
                    bad = bad + 1;
                    if (bad <= 10) $display("FMA LANE MISMATCH op %0d a %08h b/c %08h got %08h want %08h", sel, a, c, r, want);
                end
            end
        end
        $fclose(fd);
        if (nl < 1000) begin bad = bad + 1; $display("FMA FAIL lane vectors %0d", nl); end
        $display("FMA %s vectors %0d lane %0d mismatches %0d", bad == 0 ? "PASS" : "FAIL", nv, nl, bad);
        $display("RESULT: FMA %s", bad == 0 ? "PASSED" : "FAILED");
        $finish;
    end
endmodule
