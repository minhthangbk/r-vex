//======================================================================
// tb_codex_r36_fullsize (Codex round 36, task a; BUG-187, ppa.md PPA-14):
// directed test of l1d_cache x2 + coh_bus + l2_cache at the full-size
// corners (L1D 4096 sets / L2 4096 sets at AW 16 = tagless caches), plus
// the same scenario on geometries that force dirty evictions at the top
// of memory (L1S=2 with L2S=4096: L1 victims; L1S=4096 with L2S=2: L2
// victims). Every load is checked against a golden copy; at the end every
// touched word is checked through peek_m / l2.peek.
//   phase 1: core 0 writes the 4 lines 0xFFC0..0xFFF0 (M in L1D0)
//   phase 2: core 1 reads them (flush M->S), then overwrites 0xFFF0/0xFFF4 (UPGR)
//   phase 3: core 0 reads back (c2c), then writes lines that alias 0xFFF0 in
//            a 2-set L1 / L2 (0xFFD0, 0xFFB0, 0x0010, 0x0030) -> dirty victims
//   phase 4: both cores re-read everything
//   +HIGH: core 0 additionally loads 0x1FFF0 (EA bit 16 set; bits >= AW are
//          ignored by the part-select caches, the DMEM and the MMIO decode)
//          then core 1 stores to 0x1FFF4 and core 0 reads 0xFFF4,
//          with a 2000-cycle timeout: reports HANG if the access never completes.
// +TIMEOUT=n per access (default 2000)
//======================================================================
`timescale 1ns/1ps
module tb_codex_r36_fullsize;
    localparam AW = 16;
    parameter L1S = 4096, L2S = 4096;
    parameter L2LAT = 4, MLAT = 20;
    parameter DPF = 4, DPF_DEG = 1, DPF_HIST = 4;
    parameter FAST = 1, FWD = 1, WBB = 1, WBUF = 1;
    reg clk = 0, reset = 1;
    always #5 clk = ~clk;

    reg  [1:0]     c_req, c_we;
    reg  [31:0]    c_addr [0:1];
    reg  [31:0]    c_wdata [0:1];
    wire [31:0]    c_rdata [0:1];
    wire [1:0]     c_ready;
    wire [1:0]     b_req, b_pf, b_gnt, b_done, s_valid, s_hit_m;
    wire [3:0]     b_op;
    wire [2*AW-1:0] b_addr;
    wire [255:0]   b_wline, s_line;
    wire [127:0]   b_rline;
    wire [1:0]     s_op;
    wire [AW-1:0]  s_addr;
    wire           l2_req, l2_we, l2_done;
    wire [AW-1:0]  l2_addr;
    wire [127:0]   l2_wline, l2_rline;

    genvar g;
    generate for (g = 0; g < 2; g = g + 1) begin : c
        l1d_cache #(.AW(AW), .SETS(L1S), .DPF(DPF), .DPF_DEG(DPF_DEG), .DPF_HIST(DPF_HIST),
                    .FWD(FWD), .WBB(WBB)) l1d (
            .clk(clk), .reset(reset),
            .c_req(c_req[g]), .c_we(c_we[g]), .c_addr(c_addr[g]), .c_wdata(c_wdata[g]),
            .c_rdata(c_rdata[g]), .c_ready(c_ready[g]),
            .b_req(b_req[g]), .b_pf(b_pf[g]), .b_op(b_op[2*g +: 2]), .b_addr(b_addr[AW*g +: AW]),
            .b_wline(b_wline[128*g +: 128]), .b_gnt(b_gnt[g]), .b_done(b_done[g]),
            .b_rline(b_rline), .s_valid(s_valid[g]), .s_op(s_op), .s_addr(s_addr),
            .s_hit_m(s_hit_m[g]), .s_line(s_line[128*g +: 128]));
    end endgenerate
    coh_bus #(.N(2), .AW(AW), .FAST(FAST)) bus (
        .clk(clk), .reset(reset),
        .m_req(b_req), .m_pf(b_pf), .m_op(b_op), .m_addr(b_addr), .m_wline(b_wline),
        .m_gnt(b_gnt), .m_done(b_done), .m_rline(b_rline),
        .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr), .s_hit_m(s_hit_m), .s_line(s_line),
        .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wline(l2_wline),
        .l2_done(l2_done), .l2_rline(l2_rline));
    l2_cache #(.AW(AW), .SETS(L2S), .L2_LAT(L2LAT), .MEM_LAT(MLAT), .FAST(FAST), .WBUF(WBUF)) l2 (
        .clk(clk), .reset(reset), .req(l2_req), .we(l2_we), .addr(l2_addr),
        .wline(l2_wline), .done(l2_done), .rline(l2_rline));

    reg [31:0] golden [0:(1 << (AW - 2)) - 1];
    integer errors, k, tmo, nacc, hang;
    reg [32:0] pm0, pm1;
    reg [31:0] rd;

    // one access, held until c_ready (the core protocol), checked against golden
    task acc(input integer p, input integer we, input [31:0] a, input [31:0] d);
        integer t;
        begin
            @(negedge clk);
            c_req[p] = 1; c_we[p] = we; c_addr[p] = a; c_wdata[p] = d;
            t = 0;
            while (!c_ready[p] && t < tmo) begin @(negedge clk); t = t + 1; end
            if (!c_ready[p]) begin
                $display("HANG port=%0d we=%0d addr=%h after %0d cycles (b_req=%b busy=%b)", p, we, a, t, b_req, bus.busy);
                errors = errors + 1; hang = hang + 1;
            end else if (!we && c_rdata[p] !== golden[a[AW-1:2]]) begin
                $display("LOAD MISMATCH port=%0d addr=%h got=%h exp=%h", p, a, c_rdata[p], golden[a[AW-1:2]]);
                errors = errors + 1;
            end
            if (c_ready[p] && we) golden[a[AW-1:2]] = d;
            @(posedge clk); #1;
            c_req[p] = 0; c_we[p] = 0;
            nacc = nacc + 1;
        end
    endtask

    task final_check(input [31:0] a);
        begin
            pm0 = c[0].l1d.peek_m(a[AW-1:0]);
            pm1 = c[1].l1d.peek_m(a[AW-1:0]);
            if ((pm0[32] && pm0[31:0] !== golden[a[AW-1:2]]) || (pm1[32] && pm1[31:0] !== golden[a[AW-1:2]]) ||
                (pm0[32] && pm1[32]) ||
                (!pm0[32] && !pm1[32] && l2.peek(a[AW-1:0]) !== golden[a[AW-1:2]])) begin
                $display("FINAL MISMATCH addr=%h m0=%b/%h m1=%b/%h l2=%h exp=%h", a, pm0[32], pm0[31:0],
                         pm1[32], pm1[31:0], l2.peek(a[AW-1:0]), golden[a[AW-1:2]]);
                errors = errors + 1;
            end
        end
    endtask

    integer i, j;
    reg [31:0] al [0:7];
    initial begin
        if (!$value$plusargs("TIMEOUT=%d", tmo)) tmo = 2000;
        errors = 0; nacc = 0; hang = 0;
        for (k = 0; k < (1 << (AW - 2)); k = k + 1) begin
            golden[k] = 32'hA5000000 ^ (k * 32'h9E3779B1);
            l2.mem[k] = golden[k];
        end
        c_req = 0; c_we = 0;
        c_addr[0] = 0; c_addr[1] = 0; c_wdata[0] = 0; c_wdata[1] = 0;
        repeat (3) @(posedge clk);
        reset = 0;
        // lines at the top of memory and lines that alias 0xFFF0 in a 2-set cache (bit 4 = 1)
        al[0] = 32'hFFC0; al[1] = 32'hFFD0; al[2] = 32'hFFE0; al[3] = 32'hFFF0;
        al[4] = 32'hFFB0; al[5] = 32'h0010; al[6] = 32'h0030; al[7] = 32'h7FF0;
        // phase 1
        for (i = 0; i < 4; i = i + 1)
            for (j = 0; j < 4; j = j + 1) acc(0, 1, al[i] + 4 * j, 32'h10000000 + 16 * i + j);
        // phase 2
        for (i = 0; i < 4; i = i + 1)
            for (j = 0; j < 4; j = j + 1) acc(1, 0, al[i] + 4 * j, 0);
        acc(1, 1, 32'hFFF0, 32'h2222FFF0); acc(1, 1, 32'hFFF4, 32'h2222FFF4); acc(1, 1, 32'hFFFC, 32'h2222FFFC);
        // phase 3
        for (j = 0; j < 4; j = j + 1) acc(0, 0, 32'hFFF0 + 4 * j, 0);
        for (i = 4; i < 8; i = i + 1)
            for (j = 0; j < 4; j = j + 1) acc(0, 1, al[i] + 4 * j, 32'h30000000 + 16 * i + j);
        for (i = 0; i < 8; i = i + 1) acc(1, 1, al[i] + 12, 32'h4000000 + i);       // ping-pong ownership
        for (i = 0; i < 8; i = i + 1) acc(0, 1, al[(i + 3) % 8] + 8, 32'h5000000 + i);
        // phase 4
        for (i = 0; i < 8; i = i + 1)
            for (j = 0; j < 4; j = j + 1) begin acc(0, 0, al[i] + 4 * j, 0); acc(1, 0, al[i] + 4 * j, 0); end
        // drain
        k = 0;
        while ((bus.busy || |b_req) && k < 5000) begin @(posedge clk); k = k + 1; end
        #1;
        for (i = 0; i < 8; i = i + 1)
            for (j = 0; j < 4; j = j + 1) final_check(al[i] + 4 * j);
        if ($test$plusargs("HIGH")) begin
            // EA 0x1FFF0 aliases 0xFFF0 in every cache that takes the tag by part select (bits >= AW ignored)
            acc(0, 0, 32'h1FFF0, 0);
            acc(1, 1, 32'h1FFF4, 32'h6666FFF4);   // store with EA bit 16 set (golden updated at 0xFFF4)
            acc(0, 0, 32'h0FFF4, 0);
        end
        $display("FULLSIZE L1S=%0d L2S=%0d DPF=%0d FAST=%0d FWD=%0d WBB=%0d WBUF=%0d accesses=%0d hang=%0d l1miss=%0d/%0d wb=%0d/%0d l2hit=%0d l2miss=%0d l2wb=%0d errors=%0d",
                 L1S, L2S, DPF, FAST, FWD, WBB, WBUF, nacc, hang, c[0].l1d.n_miss, c[1].l1d.n_miss,
                 c[0].l1d.n_wb, c[1].l1d.n_wb, l2.n_hit, l2.n_miss, l2.n_wb, errors);
        if (errors == 0) $display("RESULT: FULLSIZE PASSED");
        else $display("RESULT: FULLSIZE FAILED");
        $finish;
    end
endmodule
