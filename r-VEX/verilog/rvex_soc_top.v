//======================================================================
// r-VEX SoC phase 1 | Top-level integration (round 25)
//----------------------------------------------------------------------
// rvex_core_bus + a real SRAM macro (sram_sync) for IMEM (private TCM
// port) + a real SRAM macro behind an AHB-Lite slave (ahb_lite_sram_ctrl)
// for DMEM. This is the literal phase-1 SoC-roadmap deliverable ("SRAM
// macros replacing reg arrays + AXI4/AHB-Lite wrapper") -- see
// rvex_core_bus.v's header for the full design rationale and gate
// decisions, and reviews/rvex-round25-*.html for the NotebookLM grounding
// and Codex verification behind them.
//
// Testbenches load program/data images and check final architectural
// state via the same hierarchical-reference convention the pre-existing
// suite already uses against rvex_core.v (dut.imem[a]=..., dut.gr[N]),
// just one level deeper: dut.u_imem.mem[a], dut.u_dmem_ctrl.u_sram.mem[a],
// dut.u_core.gr[N]. See tb_soc_top.v.
//======================================================================
module rvex_soc_top (
    input  wire clk,
    input  wire reset,
    input  wire start,
    output wire done,
    output wire [31:0] cycles
);
    wire [7:0]   imem_addr;
    wire         imem_en;
    wire [127:0] imem_rdata;

    wire [7:0]  d_haddr;
    wire        d_hwrite;
    wire [1:0]  d_htrans;
    wire [31:0] d_hwdata;
    wire [31:0] d_hrdata;
    wire        d_hreadyout;

    // imem_stall tied low: this integration has no I-Cache (round 26 added
    // that port to rvex_core_bus.v for rvex_soc_top_cache.v's benefit) --
    // tying it low reproduces round 25's exact original behavior/timing,
    // reverified by rerunning this round's full regression unchanged.
    rvex_core_bus u_core (
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .imem_addr(imem_addr), .imem_en(imem_en), .imem_rdata(imem_rdata),
        .imem_stall(1'b0),
        .d_haddr(d_haddr), .d_hwrite(d_hwrite), .d_htrans(d_htrans),
        .d_hwdata(d_hwdata), .d_hrdata(d_hrdata), .d_hreadyout(d_hreadyout)
    );

    // IMEM: dedicated fixed-latency TCM SRAM macro (no bus protocol -- see
    // rvex_core_bus.v header for why). `en` MUST follow the core's
    // fetch_advance signal, not be tied high -- see that wire's comment in
    // rvex_core_bus.v for the bundle-dropping bug this fixed.
    sram_sync #(.WIDTH(128), .DEPTH(256), .ADDRW(8)) u_imem (
        .clk(clk), .en(imem_en), .we(1'b0), .addr(imem_addr), .din(128'd0), .dout(imem_rdata)
    );

    // DMEM: SRAM macro behind the real AHB-Lite slave.
    ahb_lite_sram_ctrl #(.DATA_WIDTH(32), .DEPTH(256), .ADDRW(8)) u_dmem_ctrl (
        .clk(clk), .reset(reset),
        .haddr(d_haddr), .hwrite(d_hwrite), .htrans(d_htrans), .hwdata(d_hwdata),
        .hrdata(d_hrdata), .hreadyout(d_hreadyout), .hresp()
    );
endmodule
