//======================================================================
// r-VEX | Two-core system: core + L1I + L1D (MSI) per core, shared L2
//----------------------------------------------------------------------
//   core0 -- l1i0 (rom0)          core1 -- l1i1 (rom1)
//     |                              |
//   l1d0 ------ coh_bus (atomic, snooping, RR) ------ l1d1
//   (r12: fast completion, L1D write-back buffers, L2 posted write buffer)
//                     |
//                  l2_cache + main memory (l2.mem)
//
// Data addresses 0xFF00..0xFFFF (d_addr[AW-1:8] all ones) bypass the L1Ds
// and go to hw_sem: 64 hardware locks, ldw = test-and-set, stw = release.
// Sub-word accesses there have no effect (loads read 0xFFFFFFFF).
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
    parameter IFILL_LAT = 8,
    parameter L1I_WARM = 0,        // 1: L1I preloaded at reset (measurement mode, l1i_cache.v WARM)
    // prefetch (2026-11-04 r11, reviews/contracts/prefetch.md; defaults from the r11 A/B: cached -3.0 %,
    // cached-small -6.1 % of all suite cycles; L1I_IPF = 0 and L1D_DPF = 0 give the r10 system)
    parameter L1I_IPF  = 1,        // 1: L1I next-line prefetch buffer (VLIW fetch stream)
    parameter L1D_DPF  = 4,        // L1D stream prefetch buffer lines (0 = off)
    parameter L1D_STR  = 1,        // r23 (prefetch.md DPF-11..13, BUG-134): stride targets for the L1D prefetch (0 = r22)
    parameter L1D_AGE  = 1,        // r23 (prefetch.md DPF-14, BUG-240): oldest pending prefetch first (0 = r22)
    parameter L1D_DEG  = 1,        // L1D prefetch degree (lines ahead; 2 was slower: bus bandwidth)
    parameter L1D_HIST = 4,        // L1D demand-miss history (allocation filter)
    // bus latency and write buffers (2026-11-04 r12, reviews/contracts/bus.md; all 0 = the r11 system).
    // Defaults from the r12 A/B (717 suite cases): cached -14.8 %, cached-small -15.4 % of all cycles
    parameter BUS_FAST = 1,        // coh_bus + l2_cache FAST: combinational completion, no L2 IDLE bubble
    parameter L1D_FWD  = 1,        // L1D forwards the fill to the core in the completion cycle
    parameter L1D_WBB  = 1,        // L1D one-line write-back (victim) buffer
    parameter L2_WBUF  = 1         // L2 posted write buffer for dirty victims (read first)
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
    wire [N-1:0]     b_req, b_pf, b_gnt, b_done, s_valid, s_hit_m;
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

    wire [N-1:0]   sem_req, sem_we, sem_wbit, sem_ready, sem_rbit;
    wire [6*N-1:0] sem_idx;
    wire [31:0]    sem_acq, sem_spin;
    hw_sem #(.N(N)) sem (
        .clk(clk), .reset(reset), .req(sem_req), .we(sem_we), .idx(sem_idx),
        .wbit(sem_wbit), .ready(sem_ready), .rbit(sem_rbit),
        .n_acq(sem_acq), .n_spin(sem_spin));

    genvar g;
    generate for (g = 0; g < N; g = g + 1) begin : cpu
        wire        d_req, d_we, d_word, d_ready, i_req, i_ready, c_ready;
        wire [31:0] d_addr, d_wdata, d_rdata, c_rdata, stall_cycles, istall_cycles;
        wire        mmio = &d_addr[AW-1:8];
        // Only ldw/stw reach the semaphores. A sub-word access to the block
        // completes at once with no effect, and a sub-word load reads all
        // ones ("taken"), so a mistaken ldb/stb can neither acquire nor
        // release a lock (Codex round 6: d_addr is word aligned and d_wdata a
        // merged word, so the lane is lost).
        wire        mmio_sub = mmio && !d_word;
        assign sem_req[g]  = d_req && mmio && d_word;
        assign sem_we[g]   = d_we;
        assign sem_idx[6*g +: 6] = d_addr[7:2];
        assign sem_wbit[g] = d_wdata[0];
        assign d_ready = mmio_sub ? 1'b1 : mmio ? sem_ready[g] : c_ready;
        assign d_rdata = mmio_sub ? 32'hFFFF_FFFF : mmio ? {31'd0, sem_rbit[g]} : c_rdata;
        wire [PC_W-1:0] i_addr;
        wire [127:0] i_rdata;
        rvex_core #(.PC_W(PC_W), .DMEM_AW(AW), .EXT_DMEM(1), .EXT_IMEM(1)) core (
            .clk(clk), .reset(reset), .start(start[g]), .done(done[g]), .cycles(cyc[g]),
            .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_word(d_word),
            .d_rdata(d_rdata), .d_ready(d_ready), .stall_cycles(stall_cycles),
            .i_req(i_req), .i_addr(i_addr), .i_rdata(i_rdata), .i_ready(i_ready),
            .istall_cycles(istall_cycles));
        l1i_cache #(.PC_W(PC_W), .SETS(L1I_SETS), .FILL_LAT(IFILL_LAT), .WARM(L1I_WARM),
                    .IPF(L1I_IPF)) l1i (
            .clk(clk), .reset(reset), .i_req(i_req), .i_addr(i_addr),
            .i_rdata(i_rdata), .i_ready(i_ready));
        l1d_cache #(.AW(AW), .SETS(L1D_SETS), .DPF(L1D_DPF), .DPF_DEG(L1D_DEG),
                    .DPF_HIST(L1D_HIST), .DPF_STR(L1D_STR), .DPF_AGE(L1D_AGE), .FWD(L1D_FWD), .WBB(L1D_WBB)) l1d (
            .clk(clk), .reset(reset),
            .c_req(d_req && !mmio), .c_we(d_we), .c_addr(d_addr), .c_wdata(d_wdata),
            .c_rdata(c_rdata), .c_ready(c_ready),
            .b_req(b_req[g]), .b_pf(b_pf[g]), .b_op(b_op[2*g +: 2]), .b_addr(b_addr[AW*g +: AW]),
            .b_wline(b_wline[128*g +: 128]), .b_gnt(b_gnt[g]), .b_done(b_done[g]),
            .b_rline(b_rline),
            .s_valid(s_valid[g]), .s_op(s_op), .s_addr(s_addr),
            .s_hit_m(s_hit_m[g]), .s_line(s_line[128*g +: 128]));
    end endgenerate

    wire          l2_req, l2_we, l2_done;
    wire [AW-1:0] l2_addr;
    wire [127:0]  l2_wline, l2_rline;
    coh_bus #(.N(N), .AW(AW), .FAST(BUS_FAST)) bus (
        .clk(clk), .reset(reset),
        .m_req(b_req), .m_pf(b_pf), .m_op(b_op), .m_addr(b_addr), .m_wline(b_wline),
        .m_gnt(b_gnt), .m_done(b_done), .m_rline(b_rline),
        .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr),
        .s_hit_m(s_hit_m), .s_line(s_line),
        .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wline(l2_wline),
        .l2_done(l2_done), .l2_rline(l2_rline));
    l2_cache #(.AW(AW), .SETS(L2_SETS), .L2_LAT(L2_LAT), .MEM_LAT(MEM_LAT), .FAST(BUS_FAST),
               .WBUF(L2_WBUF)) l2 (
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
