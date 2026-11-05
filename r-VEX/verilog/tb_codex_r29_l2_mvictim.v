`timescale 1ns/1ps
// Round 29: L2 may evict/post its stale dirty copy while a newer M copy survives in an L1D.
module tb_codex_r29_l2_mvictim;
    localparam AW = 16, A = 16'h1000, B = 16'h1020;
    reg clk = 0, reset = 1;
    always #5 clk = ~clk;

    reg [1:0] c_req, c_we;
    reg [31:0] c_addr [0:1];
    reg [31:0] c_wdata [0:1];
    wire [31:0] c_rdata [0:1];
    wire [1:0] c_ready;
    wire [1:0] b_req, b_pf, b_gnt, b_done, s_valid, s_hit_m;
    wire [3:0] b_op;
    wire [2*AW-1:0] b_addr;
    wire [255:0] b_wline, s_line;
    wire [127:0] b_rline;
    wire [1:0] s_op;
    wire [AW-1:0] s_addr;
    wire l2_req, l2_we, l2_done;
    wire [AW-1:0] l2_addr;
    wire [127:0] l2_wline, l2_rline;

    genvar g;
    generate for (g = 0; g < 2; g = g + 1) begin : c
        l1d_cache #(.AW(AW), .SETS(4), .DPF(0), .FWD(1), .WBB(1)) l1d (
            .clk(clk), .reset(reset), .c_req(c_req[g]), .c_we(c_we[g]),
            .c_addr(c_addr[g]), .c_wdata(c_wdata[g]), .c_rdata(c_rdata[g]), .c_ready(c_ready[g]),
            .b_req(b_req[g]), .b_pf(b_pf[g]), .b_op(b_op[2*g +: 2]),
            .b_addr(b_addr[AW*g +: AW]), .b_wline(b_wline[128*g +: 128]),
            .b_gnt(b_gnt[g]), .b_done(b_done[g]), .b_rline(b_rline),
            .s_valid(s_valid[g]), .s_op(s_op), .s_addr(s_addr),
            .s_hit_m(s_hit_m[g]), .s_line(s_line[128*g +: 128]));
    end endgenerate
    coh_bus #(.N(2), .AW(AW), .FAST(1)) bus (
        .clk(clk), .reset(reset), .m_req(b_req), .m_pf(b_pf), .m_op(b_op), .m_addr(b_addr),
        .m_wline(b_wline), .m_gnt(b_gnt), .m_done(b_done), .m_rline(b_rline),
        .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr), .s_hit_m(s_hit_m), .s_line(s_line),
        .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wline(l2_wline),
        .l2_done(l2_done), .l2_rline(l2_rline));
    l2_cache #(.AW(AW), .SETS(2), .L2_LAT(2), .MEM_LAT(12), .FAST(1), .WBUF(1)) l2 (
        .clk(clk), .reset(reset), .req(l2_req), .we(l2_we), .addr(l2_addr), .wline(l2_wline),
        .done(l2_done), .rline(l2_rline));

    integer guard, errors, k;
    reg [31:0] value;
    task access;
        input integer p;
        input wr;
        input [31:0] a, d;
        output [31:0] q;
        begin
            @(negedge clk); c_req[p] = 1; c_we[p] = wr; c_addr[p] = a; c_wdata[p] = d;
            guard = 0;
            while (!c_ready[p] && guard < 500) begin @(negedge clk); guard = guard + 1; end
            q = c_rdata[p];
            if (guard == 500) begin $display("FAIL access timeout p=%0d a=%h", p, a); errors = errors + 1; end
            @(posedge clk); #1; c_req[p] = 0;
        end
    endtask

    initial begin
        errors = 0; c_req = 0; c_we = 0;
        for (k = 0; k < 16384; k = k + 1) l2.mem[k] = 0;
        repeat (3) @(posedge clk); reset = 0;
        access(0, 1, A, 32'h1111_2901, value);       // A: cache 0 M, L2 clean/stale
        access(1, 0, A, 0, value);                   // flush A: both S, L2 dirty
        access(0, 1, A, 32'h2222_2902, value);       // UPGR: cache 0 M, L2 dirty/stale
        if (c[0].l1d.st[0] != 2 || !l2.drt[0]) begin
            $display("FAIL setup l1state=%0d l2dirty=%0d", c[0].l1d.st[0], l2.drt[0]); errors = errors + 1;
        end
        access(1, 0, B, 0, value);                   // evict/post stale dirty A while cache 0 owns newer A
        if (l2.n_wbuf == 0 || c[0].l1d.st[0] != 2) begin
            $display("FAIL victim path wbuf=%0d l1state=%0d", l2.n_wbuf, c[0].l1d.st[0]); errors = errors + 1;
        end
        access(1, 0, A, 0, value);                   // c2c must beat stale memory/write-behind
        if (value !== 32'h2222_2902 || bus.n_c2c < 2) begin
            $display("FAIL recovered value=%h c2c=%0d", value, bus.n_c2c); errors = errors + 1;
        end
        if (l2.peek(A) !== 32'h2222_2902) begin
            $display("FAIL L2 coherent value=%h", l2.peek(A)); errors = errors + 1;
        end
        $display("R29 L2_MVICTIM c2c=%0d l2wbuf=%0d mwait=%0d value=%h errors=%0d",
                 bus.n_c2c, l2.n_wbuf, l2.n_mwait, value, errors);
        $display("RESULT: CODEX R29 L2_MVICTIM %s", errors ? "FAILED" : "PASSED");
        $finish;
    end
endmodule
