//======================================================================
// r-VEX SoC round 26 | Top-level integration: core_bus + I$/D$ + AXI4
//----------------------------------------------------------------------
// rvex_core_bus.v (UNCHANGED core logic from round 25, only gained the
// additive imem_stall port -- see that file's header) + icache.v +
// dcache.v + two axi4_sram_ctrl.v instances (instruction and data
// memory). This is the literal round-26 deliverable ("bus AXI, thêm
// I-Cache, D-Cache riêng") -- see icache.v/dcache.v headers for the full
// design rationale and gate decisions, and
// reviews/rvex-round26-axi-cache.html for the NotebookLM grounding and
// Codex verification behind them.
//
// Hierarchical-reference convention for testbenches, one level deeper
// again than rvex_soc_top.v: dut.u_icache.data_ram[a] / .tag_ram[a] /
// .valid_ram[a], dut.u_dcache.data_ram[a][w] / .tag_ram[a] / .valid_ram[a],
// dut.u_imem_ctrl.u_sram.mem[a], dut.u_dmem_ctrl.u_sram.mem[a],
// dut.u_core.gr[N]. See tb_soc_top_cache.v.
//======================================================================
module rvex_soc_top_cache (
    input  wire clk,
    input  wire reset,
    input  wire start,
    output wire done,
    output wire [31:0] cycles
);
    // ---- core <-> I-Cache ----
    wire [7:0]   imem_addr;
    wire         imem_en;
    wire [127:0] imem_rdata;
    wire         imem_stall;

    // ---- core <-> D-Cache (AHB-Lite-shaped) ----
    wire [7:0]  d_haddr;
    wire        d_hwrite;
    wire [1:0]  d_htrans;
    wire [31:0] d_hwdata;
    wire [31:0] d_hrdata;
    wire        d_hreadyout;

    // ---- I-Cache <-> AXI4 IMEM (128-bit, read-only) ----
    wire [7:0]   i_araddr;
    wire          i_arvalid, i_arready;
    wire [127:0] i_rdata;
    wire          i_rvalid;

    // ---- D-Cache <-> AXI4 DMEM (32-bit, read+write) ----
    wire [7:0]  d_awaddr;
    wire        d_awvalid, d_awready;
    wire [31:0] d_wdata;
    wire        d_wvalid, d_wready;
    wire        d_bvalid;
    wire [7:0]  d_araddr;
    wire        d_arvalid, d_arready;
    wire [31:0] d_rdata;
    wire        d_rvalid;

    // RESET_SQUASH=0, PC_E_FROM_PC_F=1, BRANCH_SQUASH=1: see
    // rvex_core_bus.v's parameter comments -- all three are consequences
    // of icache.v's fundamentally different (resolve-to-current-pc_f)
    // timing contract vs round 25's sram_sync (fixed 1-cycle-behind)
    // contract, which collapses the effective fetch pipeline from 2-deep
    // to 1-deep.
    rvex_core_bus #(.RESET_SQUASH(2'b00), .PC_E_FROM_PC_F(1), .BRANCH_SQUASH(2'b01)) u_core (
        .clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles),
        .imem_addr(imem_addr), .imem_en(imem_en), .imem_rdata(imem_rdata), .imem_stall(imem_stall),
        .d_haddr(d_haddr), .d_hwrite(d_hwrite), .d_htrans(d_htrans),
        .d_hwdata(d_hwdata), .d_hrdata(d_hrdata), .d_hreadyout(d_hreadyout)
    );

    icache #(.LINES(8), .IDXW(3)) u_icache (
        .clk(clk), .reset(reset),
        .addr(imem_addr), .en(imem_en), .rdata(imem_rdata), .stall(imem_stall),
        .araddr(i_araddr), .arvalid(i_arvalid), .arready(i_arready),
        .rdata_axi(i_rdata), .rvalid_axi(i_rvalid), .rready_axi()
    );

    axi4_sram_ctrl #(.DATA_WIDTH(128), .DEPTH(256), .ADDRW(8)) u_imem_ctrl (
        .clk(clk), .reset(reset),
        .awaddr(8'd0), .awvalid(1'b0), .awready(),
        .wdata(128'd0), .wvalid(1'b0), .wready(),
        .bresp(), .bvalid(), .bready(1'b1),
        .araddr(i_araddr), .arvalid(i_arvalid), .arready(i_arready),
        .rdata(i_rdata), .rresp(), .rlast(), .rvalid(i_rvalid), .rready(1'b1)
    );

    dcache #(.LINES(8), .IDXW(3), .OFFW(2)) u_dcache (
        .clk(clk), .reset(reset),
        .haddr(d_haddr), .hwrite(d_hwrite), .htrans(d_htrans), .hwdata(d_hwdata),
        .hrdata(d_hrdata), .hreadyout(d_hreadyout), .hresp(),
        .awaddr(d_awaddr), .awvalid(d_awvalid), .awready(d_awready),
        .wdata(d_wdata), .wvalid(d_wvalid), .wready(d_wready),
        .bvalid(d_bvalid), .bready(),
        .araddr(d_araddr), .arvalid(d_arvalid), .arready(d_arready),
        .rdata_axi(d_rdata), .rvalid_axi(d_rvalid), .rready_axi()
    );

    axi4_sram_ctrl #(.DATA_WIDTH(32), .DEPTH(256), .ADDRW(8)) u_dmem_ctrl (
        .clk(clk), .reset(reset),
        .awaddr(d_awaddr), .awvalid(d_awvalid), .awready(d_awready),
        .wdata(d_wdata), .wvalid(d_wvalid), .wready(d_wready),
        .bresp(), .bvalid(d_bvalid), .bready(1'b1),
        .araddr(d_araddr), .arvalid(d_arvalid), .arready(d_arready),
        .rdata(d_rdata), .rresp(), .rlast(), .rvalid(d_rvalid), .rready(1'b1)
    );
endmodule
