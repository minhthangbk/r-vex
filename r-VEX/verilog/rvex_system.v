//======================================================================
// r-VEX | Two-core system: core + L1I + L1D (MSI) per core, shared L2
//----------------------------------------------------------------------
//   core0 -- l1i0 (rom0)          core1 -- l1i1 (rom1)
//     |                              |
//   l1d0 ------ coh_bus (atomic, snooping, RR) ------ l1d1
//                     |
//                  l2_cache + main memory (l2.mem)
//
// A single-core run starts only core 0 (start1 = 0); core 1 and its caches
// then stay idle and never request the bus. Testbenches load programs into
// cpu0.l1i.rom / cpu1.l1i.rom and the data image into l2.mem, and read
// results back through peek(), which returns the coherent value of a word
// (an M copy in an L1, else L2, else memory).
//======================================================================
module rvex_system #(
    parameter PC_W     = 12,
    parameter AW       = 16,
    parameter L1D_SETS = 256,      // 256 x 16 B = 4 KB per core
    parameter L1I_SETS = 64,       // 64 x 4 packets = 256 packets (4 KB) per core
    parameter L2_SETS  = 1024,     // 1024 x 16 B = 16 KB shared
    parameter L2_LAT   = 4,
    parameter MEM_LAT  = 20,
    parameter IFILL_LAT = 8
) (
    input  wire        clk,
    input  wire        reset,
    input  wire        start0,
    input  wire        start1,
    output wire        done0,
    output wire        done1,
    output wire [31:0] cycles0,
    output wire [31:0] cycles1
);
    localparam N = 2;
    wire [N-1:0]     b_req, b_gnt, b_done, s_valid, s_hit_m;
    wire [2*N-1:0]   b_op;
    wire [AW*N-1:0]  b_addr;
    wire [128*N-1:0] b_wline, s_line;
    wire [127:0]     b_rline;
    wire [1:0]       s_op;
    wire [AW-1:0]    s_addr;
    wire [N-1:0]     start = {start1, start0};
    wire [N-1:0]     done;
    wire [31:0]      cyc [0:N-1];
    assign done0 = done[0]; assign done1 = done[1];
    assign cycles0 = cyc[0]; assign cycles1 = cyc[1];

    genvar g;
    generate for (g = 0; g < N; g = g + 1) begin : cpu
        wire        d_req, d_we, d_ready, i_req, i_ready;
        wire [31:0] d_addr, d_wdata, d_rdata, stall_cycles, istall_cycles;
        wire [PC_W-1:0] i_addr;
        wire [127:0] i_rdata;
        rvex_core #(.PC_W(PC_W), .DMEM_AW(AW), .EXT_DMEM(1), .EXT_IMEM(1)) core (
            .clk(clk), .reset(reset), .start(start[g]), .done(done[g]), .cycles(cyc[g]),
            .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata),
            .d_rdata(d_rdata), .d_ready(d_ready), .stall_cycles(stall_cycles),
            .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata), .i_ready(i_ready),
            .istall_cycles(istall_cycles));
        l1i_cache #(.PC_W(PC_W), .SETS(L1I_SETS), .FILL_LAT(IFILL_LAT)) l1i (
            .clk(clk), .reset(reset), .i_req(i_req), .i_addr(i_addr),
            .i_rdata(i_rdata), .i_ready(i_ready));
        l1d_cache #(.AW(AW), .SETS(L1D_SETS)) l1d (
            .clk(clk), .reset(reset),
            .c_req(d_req), .c_we(d_we), .c_addr(d_addr), .c_wdata(d_wdata),
            .c_rdata(d_rdata), .c_ready(d_ready),
            .b_req(b_req[g]), .b_op(b_op[2*g +: 2]), .b_addr(b_addr[AW*g +: AW]),
            .b_wline(b_wline[128*g +: 128]), .b_gnt(b_gnt[g]), .b_done(b_done[g]),
            .b_rline(b_rline),
            .s_valid(s_valid[g]), .s_op(s_op), .s_addr(s_addr),
            .s_hit_m(s_hit_m[g]), .s_line(s_line[128*g +: 128]));
    end endgenerate

    wire          l2_req, l2_we, l2_done;
    wire [AW-1:0] l2_addr;
    wire [127:0]  l2_wline, l2_rline;
    coh_bus #(.N(N), .AW(AW)) bus (
        .clk(clk), .reset(reset),
        .m_req(b_req), .m_op(b_op), .m_addr(b_addr), .m_wline(b_wline),
        .m_gnt(b_gnt), .m_done(b_done), .m_rline(b_rline),
        .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr),
        .s_hit_m(s_hit_m), .s_line(s_line),
        .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wline(l2_wline),
        .l2_done(l2_done), .l2_rline(l2_rline));
    l2_cache #(.AW(AW), .SETS(L2_SETS), .L2_LAT(L2_LAT), .MEM_LAT(MEM_LAT)) l2 (
        .clk(clk), .reset(reset), .req(l2_req), .we(l2_we), .addr(l2_addr),
        .wline(l2_wline), .done(l2_done), .rline(l2_rline));

    function [31:0] peek;
        input [AW-1:0] a;
        reg [32:0] m0, m1;
        begin
            m0 = cpu[0].l1d.peek_m(a);
            m1 = cpu[1].l1d.peek_m(a);
            peek = m0[32] ? m0[31:0] : m1[32] ? m1[31:0] : l2.peek(a);
        end
    endfunction
endmodule
