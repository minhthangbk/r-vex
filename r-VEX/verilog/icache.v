//======================================================================
// r-VEX SoC round 26 | Direct-mapped I-Cache
//----------------------------------------------------------------------
// Sits between rvex_core_bus.v's IMEM TCM port (addr/en/rdata/stall --
// unchanged protocol, see that file's "ROUND 26 ADDITION" header note)
// and an AXI4 read-only master port to instruction memory
// (axi4_sram_ctrl.v, DATA_WIDTH=128 so one AXI4 beat == exactly one
// cache line).
//
// GATE DECISION (round 26, NotebookLM digital_design grounding --
// reviews/rvex-round26-*.html section 2): 8 lines, direct-mapped, line
// size = 1 VLIW bundle (128 bits). NotebookLM's generic real-silicon
// recommendation was much larger (32 KB, 2-way, 64B lines) -- doesn't
// fit this project's actual scale (IMEM is 256 packets = 4 KB total;
// a 32 KB cache would be larger than the memory it's caching, making
// hit/miss/eviction behavior untestable). Scaled down deliberately so
// the cache is a genuine, exercisable SUBSET of the backing store (8 of
// 256 lines = ~3%) -- proportionally comparable to a real 32 KB cache
// against a multi-MB memory. Line size = 1 bundle (not NotebookLM's
// recommended 2-4x) is a further deliberate simplification: avoids
// needing sub-line-offset indexing on top of everything else new this
// round; wider lines (for real spatial-locality benefit) are natural,
// well-scoped future work once this simpler version is verified.
//
// TIMING -- REVISED after a real bug found by simulation (round 26, see
// reviews/rvex-round26-*.html section 3): `rdata` is driven PURELY
// COMBINATIONALLY from `data_ram[idx]` (using the CURRENT address's
// index directly), not from a registered "1-cycle-behind" index as an
// earlier draft did. That earlier draft mirrored sram_sync.v's registered
// address-to-Q pattern to match round 25's exact fetch timing, but this
// created a real race: `stall` (hence whether rvex_core_bus.v may
// consume `rdata` THIS cycle) is computed from `tag_hit`, which is ALSO
// purely combinational -- so on any cycle a NEWLY-presented address
// (e.g. a taken branch's target, or a warm loop's back-edge) happens to
// ALREADY be cached, `stall` cleared immediately while the registered
// index register was still one cycle behind, still pointing at whatever
// address had been resolved last. The core would then capture and
// label an ENTIRELY DIFFERENT bundle's data under the new address's
// PC -- observed as: a taken branch's target bundle silently executing
// with a stale/wrong-path bundle's decode, and a loop that could never
// complete a second iteration. Driving `rdata` from `data_ram[idx]`
// directly removes the possibility of misalignment entirely (hit-
// readiness and hit-data now come from the exact same combinational
// source, indexed by the exact same wire) -- confirmed this ALSO applies
// to a plain sequential re-visit of a cached address, not just branch
// redirects, by hand-deriving both cases before trusting the fix (a
// registered version had passed several rounds of testing purely by
// coincidence: this project's straight-line kernels never revisit an
// address without a branch, so the sequential-revisit case was never
// exercised until the warmed-up load-use-latency loop test in
// tb_soc_top_cache.v). This does NOT reintroduce anything round 25
// warned against: instruction FETCH latency is not a value the compiler
// reasons about (rvexSchedule.td has no fetch-latency itinerary; only
// DMEM load-use timing is compiler-visible), so a hit resolving with
// zero added cycles here is a legitimate microarchitectural choice, not
// a hidden interlock masking a compiler bug the way an auto-stalling
// DMEM would be.
//
// On a MISS, `stall` is asserted (freezing the whole core, see
// rvex_core_bus.v) for exactly as long as the AXI4 refill takes; this is
// the LEQ-model "hardware slower than assumed" case NotebookLM confirmed
// is legitimate.
//
// `en` is NOT gated by `stall` here or upstream (see rvex_core_bus.v's
// comment on why that would be a real combinational loop) -- instead
// this module ignores repeated `en` pulses for an address it's already
// mid-refill for via the `miss_active` register (`lookup_now` below),
// which is what actually prevents the round-25 Bug-3 class of "re-latch
// a frozen-but-stale address" corruption here.
//======================================================================
module icache #(
    parameter LINES = 8,
    parameter IDXW   = 3   // log2(LINES)
) (
    input  wire        clk,
    input  wire        reset,

    // ---- upstream: rvex_core_bus.v's IMEM TCM port ----
    input  wire [7:0]   addr,
    input  wire         en,
    output wire [127:0] rdata,
    output wire          stall,

    // ---- downstream: AXI4 read-only master (DATA_WIDTH=128) ----
    output wire [7:0]   araddr,
    output wire          arvalid,
    input  wire          arready,
    input  wire [127:0] rdata_axi,
    input  wire          rvalid_axi,
    output wire          rready_axi
);
    localparam TAGW = 8 - IDXW;

    reg [127:0] data_ram [0:LINES-1];
    reg [TAGW-1:0] tag_ram [0:LINES-1];
    reg valid_ram [0:LINES-1];

    wire [IDXW-1:0] idx = addr[IDXW-1:0];
    wire [TAGW-1:0] tag = addr[7:IDXW];

    reg miss_active;
    reg [7:0]      miss_addr;
    reg [IDXW-1:0] miss_idx;
    reg [TAGW-1:0] miss_tag;
    localparam IC_IDLE = 1'b0, IC_WAIT = 1'b1;
    reg ic_state;

    wire lookup_now = en && !miss_active;
    wire tag_hit    = valid_ram[idx] && (tag_ram[idx] == tag);
    wire hit        = lookup_now && tag_hit;
    wire new_miss   = lookup_now && !tag_hit;

    assign stall = miss_active || new_miss;

    // Purely combinational -- see the TIMING note above for why this is
    // correct (and why a registered version was NOT, despite looking
    // like the more faithful SRAM-timing choice at first).
    assign rdata = data_ram[idx];

    assign araddr     = miss_addr;
    assign arvalid     = (ic_state == IC_WAIT);
    assign rready_axi = 1'b1;   // nowhere else to be; always ready to accept

    integer k;
    always @(posedge clk) begin
        if (reset) begin
            miss_active <= 1'b0; ic_state <= IC_IDLE;
            for (k = 0; k < LINES; k = k + 1) valid_ram[k] <= 1'b0;
        end else begin
            case (ic_state)
                IC_IDLE: begin
                    if (new_miss) begin
                        miss_addr <= addr; miss_idx <= idx; miss_tag <= tag;
                        miss_active <= 1'b1;
                        ic_state <= IC_WAIT;
                    end
                end
                IC_WAIT: begin
                    // Single combined AR+R wait: arvalid is asserted
                    // throughout IC_WAIT (see assign above); once BOTH
                    // arready (address accepted) and rvalid (data back)
                    // have been seen we're done. Since this project's
                    // AXI4 slave (axi4_sram_ctrl.v) always completes a
                    // read in exactly 2 cycles once arready fires, and
                    // arvalid stays asserted until the transaction
                    // completes, checking rvalid_axi alone here is
                    // sufficient (arready will already have fired).
                    if (rvalid_axi) begin
                        data_ram[miss_idx]  <= rdata_axi;
                        tag_ram[miss_idx]   <= miss_tag;
                        valid_ram[miss_idx] <= 1'b1;
                        miss_active         <= 1'b0;
                        ic_state            <= IC_IDLE;
                    end
                end
            endcase
        end
    end
endmodule
