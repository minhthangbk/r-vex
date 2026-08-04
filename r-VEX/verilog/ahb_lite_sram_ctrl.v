//======================================================================
// r-VEX SoC phase 1 | AHB-Lite slave wrapping a sram_sync macro
//----------------------------------------------------------------------
// Gate decision (round 25, grounded via NotebookLM digital_design -- see
// reviews/rvex-round25-*.html section 2): AHB-Lite chosen over AXI4(-Lite)
// for this phase-1 wrapper. Rationale: a single VLIW core, Harvard
// (separate I/D ports), no interconnect / no additional bus masters yet
// -- AHB-Lite's 2-phase address/data structure maps directly onto a
// synchronous SRAM's registered-address timing at a fraction of AXI4's
// area/verification cost (5 independent channels, ID tracking, none of
// which buys anything with a single master and no outstanding-transaction
// requirement). AXI4 becomes the right call once phase 3+ adds DMA / a
// real interconnect with multiple masters -- not before.
//
// SCOPE / DEVIATION from the full AMBA AHB-Lite spec (documented, not
// accidental): this slave supports exactly ONE outstanding transfer at a
// time (the master must hold HADDR/HWRITE/HTRANS steady across wait
// states and must not start a new address phase until HREADYOUT for the
// current one goes high). Real AHB-Lite allows the address phase of
// transfer N+1 to overlap the data phase of transfer N (pipelining) --
// unused here because rvex_core_bus.v never has more than one
// outstanding transfer per bus. HSIZE/HBURST/HPROT/HRESP-error are not
// implemented: DATA_WIDTH is fixed per instance (32 for DMEM, 128 for
// IMEM's whole-packet fetch) and every access is OKAY (no aborting slave
// exists downstream). This is an internal SoC-local bus, not a
// chip-boundary interface, so a non-standard 128-bit IMEM width is a
// deliberate project-scoped choice, not a spec violation.
//
// Timing (mirrors sram_sync's 1-cycle address-to-Q latency):
//   cycle N   (HTRANS=NONSEQ, HADDR/HWRITE/HWDATA driven, held stable):
//             address+control latched into the SRAM macro; HREADYOUT=0.
//   cycle N+1 (master still holds HADDR/HTRANS from cycle N):
//             SRAM data valid -> HRDATA driven, HREADYOUT=1 (transfer
//             completes this cycle; master samples HRDATA at this edge).
// A write completes with the same 2-cycle shape (HREADYOUT=1 in cycle
// N+1 confirms the write landed) even though there is no HRDATA to wait
// for -- kept symmetric so the master's wait-state logic does not need a
// read/write special case.
//======================================================================
module ahb_lite_sram_ctrl #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 256,
    parameter ADDRW      = 8
) (
    input  wire                   clk,
    input  wire                   reset,

    // AHB-Lite slave port
    input  wire [ADDRW-1:0]       haddr,
    input  wire                   hwrite,
    input  wire [1:0]             htrans,      // 2'b00=IDLE 2'b10=NONSEQ (only these two are used)
    input  wire [DATA_WIDTH-1:0]  hwdata,
    output wire [DATA_WIDTH-1:0]  hrdata,
    output wire                   hreadyout,
    output wire                   hresp        // tied OKAY (0): no error responses in phase 1
);
    localparam IDLE = 1'b0, WAIT = 1'b1;
    reg state;

    // Address phase is exactly the ONE cycle the master presents NONSEQ
    // while we're still IDLE; re-presenting NONSEQ during WAIT (as a
    // spec-compliant master holding the bus steady through a wait state
    // would) must NOT re-latch, hence the (state==IDLE) qualifier.
    wire sram_en = (state == IDLE) && (htrans == 2'b10);
    wire [DATA_WIDTH-1:0] sram_dout;

    sram_sync #(.WIDTH(DATA_WIDTH), .DEPTH(DEPTH), .ADDRW(ADDRW)) u_sram (
        .clk  (clk),
        .en   (sram_en),
        .we   (hwrite),
        .addr (haddr),
        .din  (hwdata),
        .dout (sram_dout)
    );

    assign hresp     = 1'b0;             // OKAY, always
    // Combinational HREADYOUT/HRDATA (NOT registered): the address is
    // latched into sram_sync at the edge ending the IDLE cycle, so by the
    // WAIT cycle sram_dout is already valid -- driving these off `state`
    // directly (rather than re-registering them) is what makes the
    // transfer exactly 2 cycles (address phase + data phase), matching
    // the module header's timing diagram and sram_sync's 1-cycle latency.
    // (An earlier draft of this module registered hrdata/hreadyout inside
    // the WAIT case, which added a spurious extra cycle -- caught before
    // this ever reached simulation by re-deriving the timing by hand.)
    assign hreadyout = (state == WAIT) ? 1'b1 : ~sram_en;
    assign hrdata    = sram_dout;

    always @(posedge clk) begin
        if (reset) begin
            state <= IDLE;
        end else begin
            case (state)
                IDLE: if (htrans == 2'b10) state <= WAIT;
                WAIT: state <= IDLE;
            endcase
        end
    end
endmodule
