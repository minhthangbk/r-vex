`timescale 1ns/1ps
// Round 29: a DPF read must obtain the newest line from the other L1D's M copy.
module tb_codex_r29_pf_m;
    localparam AW = 16, X = 16'h1040;
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
        l1d_cache #(.AW(AW), .SETS(4), .DPF(2), .DPF_DEG(1), .DPF_HIST(2),
                    .FWD(1), .WBB(1)) l1d (
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
    l2_cache #(.AW(AW), .SETS(8), .L2_LAT(2), .MEM_LAT(5), .FAST(1), .WBUF(1)) l2 (
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
            while (!c_ready[p] && guard < 300) begin @(negedge clk); guard = guard + 1; end
            q = c_rdata[p];
            if (guard == 300) begin $display("FAIL access timeout p=%0d a=%h", p, a); errors = errors + 1; end
            @(posedge clk); #1; c_req[p] = 0;
        end
    endtask

    initial begin
        errors = 0; c_req = 0; c_we = 0;
        for (k = 0; k < 16384; k = k + 1) l2.mem[k] = 0;
        repeat (3) @(posedge clk); reset = 0;
        access(0, 1, X, 32'hc0de_2901, value);       // X is M in cache 0
        access(1, 0, X - 16'h20, 0, value);          // miss L-2
        access(1, 0, X - 16'h10, 0, value);          // sequential miss allocates prefetch X
        guard = 0;
        while (!(c[1].l1d.pv[0] || c[1].l1d.pv[1]) && guard < 300) begin
            @(posedge clk); #1; guard = guard + 1;
        end
        if (guard == 300) begin $display("FAIL prefetch did not complete"); errors = errors + 1; end
        if (bus.n_c2c == 0 || c[0].l1d.n_flush == 0) begin
            $display("FAIL prefetch did not snoop M c2c=%0d flush=%0d", bus.n_c2c, c[0].l1d.n_flush);
            errors = errors + 1;
        end
        access(1, 0, X, 0, value);
        if (value !== 32'hc0de_2901 || c[1].l1d.n_pfhit == 0) begin
            $display("FAIL prefetched value=%h pfhit=%0d", value, c[1].l1d.n_pfhit); errors = errors + 1;
        end
        $display("R29 PF_M c2c=%0d pf=%0d pfhit=%0d flush=%0d errors=%0d",
                 bus.n_c2c, c[1].l1d.n_pf, c[1].l1d.n_pfhit, c[0].l1d.n_flush, errors);
        $display("RESULT: CODEX R29 PF_M %s", errors ? "FAILED" : "PASSED");
        $finish;
    end
endmodule
