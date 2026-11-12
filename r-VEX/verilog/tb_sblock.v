//======================================================================
// tb_sblock: directed test of the snoop hold (s_block) of l1d_cache (BUG-044, round 2026-11-11-r23;
// reviews/contracts/coherence items: a core access to a line being snooped in that cycle is held one cycle,
// so a store cannot slip in after the flushed copy). Core 0 owns line X in M; core 1 stores word 0 of X
// (RDX: snoop flush of core 0's copy); core 0 stores word 1 of X D cycles after core 1's request, for
// D = 0..24 (one of them lands in the snoop cycle). Codex R46 F4: the same sweep for an RD and an UPGR snoop, and
// each row must reach the snoop cycle at least once. Afterwards both stores must be visible: core 1 reads
// word 1 = core 0's value and core 0 reads word 0 = core 1's value. Mutant SB-M1 drops !s_block from c_ready.
//======================================================================
`timescale 1ns/1ps
module tb_sblock;
    localparam AW = 16, L1S = 16;
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
        l1d_cache #(.AW(AW), .SETS(L1S)) l1d (
            .clk(clk), .reset(reset),
            .c_req(c_req[g]), .c_we(c_we[g]), .c_addr(c_addr[g]), .c_wdata(c_wdata[g]),
            .c_rdata(c_rdata[g]), .c_ready(c_ready[g]),
            .b_req(b_req[g]), .b_pf(b_pf[g]), .b_op(b_op[2*g +: 2]), .b_addr(b_addr[AW*g +: AW]),
            .b_wline(b_wline[128*g +: 128]), .b_gnt(b_gnt[g]), .b_done(b_done[g]),
            .b_rline(b_rline), .s_valid(s_valid[g]), .s_op(s_op), .s_addr(s_addr),
            .s_hit_m(s_hit_m[g]), .s_line(s_line[128*g +: 128]));
    end endgenerate
    coh_bus #(.N(2), .AW(AW)) bus (
        .clk(clk), .reset(reset),
        .m_req(b_req), .m_pf(b_pf), .m_op(b_op), .m_addr(b_addr), .m_wline(b_wline),
        .m_gnt(b_gnt), .m_done(b_done), .m_rline(b_rline),
        .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr), .s_hit_m(s_hit_m), .s_line(s_line),
        .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wline(l2_wline),
        .l2_done(l2_done), .l2_rline(l2_rline));
    l2_cache #(.AW(AW), .SETS(64), .L2_LAT(3), .MEM_LAT(7)) l2 (
        .clk(clk), .reset(reset), .req(l2_req), .we(l2_we), .addr(l2_addr),
        .wline(l2_wline), .done(l2_done), .rline(l2_rline));

    integer errors, d, x, t;
    reg [31:0] r, r0, r1;
    // core p: one access, held until ready
    task automatic acc;
        input integer p, we; input [31:0] a, wd; output [31:0] rd;
        begin
            c_req[p] = 1; c_we[p] = we; c_addr[p] = a; c_wdata[p] = wd; #1;
            while (!c_ready[p]) begin @(posedge clk); #1; end
            rd = c_rdata[p]; @(posedge clk); #1 c_req[p] = 0;
        end
    endtask
    // Codex R46 F4: count, per row kind, the delays whose core-0 store request was presented in a cycle in which
    // core 0 snooped core 1's transaction of that kind on line X (the hold window was reached, not vacuous)
    integer kind, hitw, hits [0:2];
    reg     inrow;
    always @(posedge clk) if (!reset && inrow && c_req[0] && (kind == 2 ? !c_we[0] : c_we[0]) && s_valid[0] &&
                                 s_addr == x[15:0] && s_op == kind)
        hitw = 1;
    task row;   // kind 1 RDX (core 1 stores X), 0 RD (core 1 loads X), 2 UPGR (both S, core 1 stores X)
        input integer kd;
        begin
            kind = kd; hits[kd] = 0;
            for (d = 0; d < 25; d = d + 1) begin
                x = 32'h1000 + 16 * 32 * d + 16 * (kd + 1);       // a fresh line per row and delay
                if (kd == 2) begin acc(0, 0, x + 4, 0, r); acc(1, 0, x, 0, r); end   // both S
                else acc(0, 1, x + 4, 32'hAAAA_0000 + d, r);                        // core 0: X in M
                repeat (4) @(posedge clk); #1;
                hitw = 0; inrow = 1;
                fork
                    if (kd == 0) acc(1, 0, x + 4, 0, r1);                            // RD of X
                    else acc(1, 1, x, 32'hBBBB_0000 + d, r1);                       // RDX / UPGR of X
                    begin repeat (d) @(posedge clk); #1;
                          // UPGR: core 0 holds X in S, so its only hit is a load (of the word core 1 writes)
                          if (kd == 2) acc(0, 0, x, 0, r0); else acc(0, 1, x + 4, 32'hCCCC_0000 + d, r0); end
                join
                inrow = 0; hits[kd] = hits[kd] + hitw;
                repeat (4) @(posedge clk); #1;
                if (kd == 2 && r0 !== 32'd0 && r0 !== 32'hBBBB_0000 + d) begin
                    $display("FAIL kind 2 D=%0d core 0 load during the UPGR returned %h (neither old 0 nor new %h)", d, r0, 32'hBBBB_0000 + d);
                    errors = errors + 1; end
                acc(1, 0, x + 4, 0, r);
                if (kd != 2 && r !== 32'hCCCC_0000 + d) begin
                    $display("FAIL kind %0d D=%0d core 1 reads word 1 %h, expected %h (core 0 store lost)", kd, d, r, 32'hCCCC_0000 + d);
                    errors = errors + 1; end
                if (kd != 0) begin
                    acc(0, 0, x, 0, r);
                    if (r !== 32'hBBBB_0000 + d) begin
                        $display("FAIL kind %0d D=%0d core 0 reads word 0 %h, expected %h", kd, d, r, 32'hBBBB_0000 + d);
                        errors = errors + 1; end
                end
            end
            $display("     kind %0d (%s): %0d of 25 delays reached the snoop cycle", kd, kd == 0 ? "RD" : kd == 1 ? "RDX" : "UPGR", hits[kd]);
            if (hits[kd] == 0) begin $display("FAIL kind %0d: no delay reached the snoop cycle (vacuous row)", kd); errors = errors + 1; end
        end
    endtask
    integer k;
    initial begin
        errors = 0; c_req = 0; c_we = 0; inrow = 0; x = 0;
        for (k = 0; k < (1 << (AW - 2)); k = k + 1) l2.mem[k] = 32'd0;
        repeat (3) @(posedge clk); #1 reset = 0;
        row(1); row(0); row(2);
        if (errors == 0) $display("RESULT: SBLOCK PASSED (RDX, RD, UPGR x 25 delays)");
        else $display("RESULT: SBLOCK FAILED (%0d)", errors);
        $finish;
    end
endmodule
