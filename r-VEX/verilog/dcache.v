//======================================================================
// r-VEX SoC round 26 | Direct-mapped, write-through D-Cache
//----------------------------------------------------------------------
// Drop-in replacement for ahb_lite_sram_ctrl.v from rvex_core_bus.v's
// point of view: SAME AHB-Lite-shaped slave port (haddr/hwrite/htrans/
// hwdata/hrdata/hreadyout/hresp). This is deliberate (round 26 gate
// decision, reviews/rvex-round26-*.html section 2): round 25's core
// already tolerates an ARBITRARILY long wait for hreadyout (bundle_stall
// freezes the whole core for however long that takes -- it was built
// for exactly the DMEM-bus-busy case). A cache miss "just" makes
// hreadyout arrive later than a hit would; NO core change was needed for
// the D-side (unlike the I-side -- see rvex_core_bus.v's imem_stall
// note). Downstream: an AXI4 master (read+write) to axi4_sram_ctrl.v.
//
// GATE DECISION: WRITE-THROUGH, write-around (no allocate-on-write-miss)
// -- a deliberate deviation from NotebookLM's real-silicon recommendation
// of write-back/write-allocate (justified there by avoiding constant bus
// traffic for DSP-style heavy local stores). Write-back needs dirty bits
// + an eviction eviction-buffer/write-back FSM on top of everything else
// new this round; write-through reuses the SAME direct AXI4-write path a
// plain STW already needs, at the cost of extra bus traffic. Documented
// tradeoff, not an oversight -- write-back is natural, well-scoped future
// work once this simpler version is verified. On a store: ALWAYS write
// through to memory; if the address is ALSO currently cached, update the
// cached copy too (avoids a stale-read-after-write-hit coherency bug --
// found by hand-tracing before writing any code, not by simulation).
//
// GATE DECISION: only 32-bit whole-word reads/writes are handled here.
// Sub-word STH/STB are NOT this module's concern: rvex_core_bus.v's own
// mem_unit.v-based read-modify-write logic already turns a byte/half
// store into a plain 32-bit READ (of the old word) followed by a plain
// 32-bit WRITE (of the merged word) at the AHB-Lite level (DM_WAIT1 ->
// DM_RMW_ISSUE -> DM_WAIT2, see that file) -- from THIS module's
// perspective those are just two ordinary word transactions, no special
// casing needed.
//
// Cache organization (round 26 gate decision, NotebookLM digital_design
// grounding, scaled down from its real-silicon recommendation for the
// same reason as icache.v -- see that file's header): 8 lines,
// direct-mapped, 4 words (128 bits) per line -- gives genuine spatial
// locality (unlike the I-Cache's 1-bundle line) and exercises AXI4
// multi-beat refill. Refill issues 4 SEQUENTIAL single-beat AXI4 reads
// rather than 1 length-4 burst (see axi4_sram_ctrl.v's header for why).
//======================================================================
module dcache #(
    parameter LINES = 8,
    parameter IDXW   = 3,   // log2(LINES)
    parameter OFFW   = 2    // log2(words per line)
) (
    input  wire clk,
    input  wire reset,

    // ---- upstream: AHB-Lite-shaped slave port (matches ahb_lite_sram_ctrl.v) ----
    input  wire [7:0]   haddr,
    input  wire         hwrite,
    input  wire [1:0]   htrans,
    input  wire [31:0]  hwdata,
    output wire [31:0]  hrdata,
    output wire         hreadyout,
    output wire         hresp,

    // ---- downstream: AXI4 master (read+write, DATA_WIDTH=32) ----
    output reg  [7:0]   awaddr,
    output reg           awvalid,
    input  wire          awready,
    output reg  [31:0]  wdata,
    output reg           wvalid,
    input  wire          wready,
    input  wire          bvalid,
    output wire          bready,

    output reg  [7:0]   araddr,
    output reg           arvalid,
    input  wire          arready,
    input  wire [31:0]  rdata_axi,
    input  wire          rvalid_axi,
    output wire          rready_axi
);
    localparam TAGW = 8 - IDXW - OFFW;
    localparam WORDS = 1 << OFFW;

    reg [31:0]     data_ram [0:LINES-1][0:WORDS-1];
    reg [TAGW-1:0] tag_ram  [0:LINES-1];
    reg            valid_ram[0:LINES-1];

    wire [OFFW-1:0] off = haddr[OFFW-1:0];
    wire [IDXW-1:0] idx = haddr[OFFW+IDXW-1:OFFW];
    wire [TAGW-1:0] tag = haddr[7:OFFW+IDXW];

    localparam DC_IDLE=3'd0, DC_HIT=3'd1, DC_REFILL=3'd2, DC_WR=3'd3;
    reg [2:0] state;

    wire addr_phase   = (state == DC_IDLE) && (htrans == 2'b10);
    wire tag_hit_now  = valid_ram[idx] && (tag_ram[idx] == tag);
    wire read_hit_now  = addr_phase && !hwrite && tag_hit_now;
    wire read_miss_now = addr_phase && !hwrite && !tag_hit_now;
    wire write_now      = addr_phase && hwrite;

    reg [IDXW-1:0] rd_idx_r;
    reg [OFFW-1:0] rd_off_r;

    reg [7:0]      miss_addr;
    reg [IDXW-1:0] miss_idx;
    reg [TAGW-1:0] miss_tag;
    reg [OFFW-1:0] miss_off;      // which word within the line the original request wanted
    reg [OFFW-1:0] refill_beat;

    assign hresp     = 1'b0;
    assign bready     = 1'b1;
    assign rready_axi = 1'b1;

    // Combinational hreadyout/hrdata (round 26, found by simulation --
    // NOT hand-derived first): an earlier draft registered these inside
    // the DC_HIT/DC_WR cases (`hreadyout <= 1'b1` etc.), which is EXACTLY
    // the round-25 ahb_lite_sram_ctrl.v off-by-one bug class (see that
    // file's header) -- it added a spurious extra cycle to every D-Cache
    // hit. Invisible on its own (bundle_stall tolerates any latency), but
    // once icache.v was fixed to give truly 1-cycle-latency hits (see
    // that file's TIMING note), the combined round-trip for the warmed-up
    // load-use-latency loop test (tb_soc_top_cache.v) grew by exactly
    // this 1 spurious cycle, pushing a load's GR write past the
    // distance-2 bundle -- caught because that test explicitly checks
    // BOTH directions, not just correctness of the final value.
    assign hreadyout = (state == DC_IDLE) ? !addr_phase :
                        (state == DC_HIT) ? 1'b1 :
                        (state == DC_WR)  ? bvalid :
                        1'b0; // DC_REFILL
    assign hrdata = data_ram[rd_idx_r][rd_off_r];

    integer k;
    always @(posedge clk) begin
        if (reset) begin
            state <= DC_IDLE;
            awvalid <= 1'b0; wvalid <= 1'b0; arvalid <= 1'b0;
            for (k = 0; k < LINES; k = k + 1) valid_ram[k] <= 1'b0;
        end else begin
            case (state)
                DC_IDLE: begin
                    if (read_hit_now) begin
                        rd_idx_r <= idx; rd_off_r <= off;
                        state <= DC_HIT;
                    end else if (read_miss_now) begin
                        miss_addr <= haddr; miss_idx <= idx; miss_tag <= tag; miss_off <= off;
                        refill_beat <= {OFFW{1'b0}};
                        araddr <= {tag, idx, {OFFW{1'b0}}}; arvalid <= 1'b1;
                        state <= DC_REFILL;
                    end else if (write_now) begin
                        awaddr <= haddr; awvalid <= 1'b1;
                        wdata  <= hwdata; wvalid <= 1'b1;
                        // write-through-with-update-on-hit (see header):
                        // if this address is cached, keep it consistent.
                        if (tag_hit_now) data_ram[idx][off] <= hwdata;
                        state <= DC_WR;
                    end
                end
                DC_HIT: begin
                    state <= DC_IDLE;
                end
                DC_REFILL: begin
                    if (arvalid && arready) arvalid <= 1'b0; // address phase accepted, drop request
                    if (rvalid_axi) begin
                        data_ram[miss_idx][refill_beat] <= rdata_axi;
                        if (refill_beat == {OFFW{1'b1}}) begin
                            valid_ram[miss_idx] <= 1'b1;
                            tag_ram[miss_idx]   <= miss_tag;
                            rd_idx_r <= miss_idx; rd_off_r <= miss_off; // bypass, see icache.v's identical trick
                            state <= DC_HIT;
                        end else begin
                            refill_beat <= refill_beat + 1'b1;
                            araddr <= {miss_tag, miss_idx, refill_beat + 1'b1};
                            arvalid <= 1'b1;
                        end
                    end
                end
                DC_WR: begin
                    if (awvalid && awready) awvalid <= 1'b0;
                    if (wvalid && wready)   wvalid  <= 1'b0;
                    if (bvalid) state <= DC_IDLE;
                end
                default: state <= DC_IDLE;
            endcase
        end
    end
endmodule
