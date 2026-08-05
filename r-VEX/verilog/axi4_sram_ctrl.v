//======================================================================
// r-VEX SoC round 26 | AXI4 slave wrapping a sram_sync macro
//----------------------------------------------------------------------
// Gate decision (round 26, grounded via NotebookLM digital_design --
// reviews/rvex-round26-*.html section 2): full AXI4 chosen over AXI4-Lite
// for the cache-to-memory refill path, because AXI4-Lite fixes burst
// length at 1 (AxLEN=0) -- fine for point transfers but NotebookLM's own
// answer flagged it as the wrong choice the moment a cache line needs
// more than one beat. This project's D-Cache line IS more than one beat
// (4 words -- see dcache.v).
//
// SCOPE / DEVIATION (documented, not accidental): this slave implements
// AXI4's channel STRUCTURE (separate AW/W/B and AR/R channels, unlike
// AHB-Lite's single shared channel) but NOT AXI4's burst feature --
// every transfer here is exactly 1 beat (RLAST always asserted,
// AxLEN/AxSIZE/AxBURST are not wired at all). GitHub research (see
// review section 4) found real small-SoC designs typically DO use a
// single INCR burst per cache-line refill; this project instead has
// each cache master issue N sequential single-beat AXI4 transfers for
// an N-word line. Deliberate simplification, not an oversight: this
// slave's timing pattern directly reuses ahb_lite_sram_ctrl.v's
// already-Codex-verified combinational-issue/registered-completion
// design (see that file's header for the off-by-one bug a REGISTERED
// version of this exact pattern produced in round 25) rather than
// adding untested beat-counting logic on top of an already-large round;
// burst support is flagged as natural, well-scoped future work once
// this simpler version is itself verified in silicon-verified regression.
//
// Single-outstanding: only one of a read or write transaction is ever in
// flight (matches this project's masters, which never issue two at
// once); if AR and AW+W are asserted the same cycle, read wins
// arbitrarily (never exercised by icache.v/dcache.v, which never do
// both simultaneously).
//
// Timing (identical shape to ahb_lite_sram_ctrl.v -- see that file for
// the full cycle-by-cycle derivation this one directly reuses):
//   cycle N   : ARVALID (or AWVALID+WVALID) held; address+data latched
//               into sram_sync this edge.
//   cycle N+1 : RVALID (or BVALID) combinationally high, RDATA valid;
//               transfer completes when the master asserts RREADY (or
//               BREADY) this same cycle.
//======================================================================
module axi4_sram_ctrl #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 256,
    parameter ADDRW      = 8
) (
    input  wire                   clk,
    input  wire                   reset,

    // AXI4 write address channel
    input  wire [ADDRW-1:0]       awaddr,
    input  wire                   awvalid,
    output wire                   awready,
    // AXI4 write data channel
    input  wire [DATA_WIDTH-1:0]  wdata,
    input  wire                   wvalid,
    output wire                   wready,
    // AXI4 write response channel
    output wire [1:0]             bresp,
    output wire                   bvalid,
    input  wire                   bready,

    // AXI4 read address channel
    input  wire [ADDRW-1:0]       araddr,
    input  wire                   arvalid,
    output wire                   arready,
    // AXI4 read data channel
    output wire [DATA_WIDTH-1:0]  rdata,
    output wire [1:0]             rresp,
    output wire                   rlast,   // always 1 -- see SCOPE note above
    output wire                   rvalid,
    input  wire                   rready
);
    localparam ST_IDLE = 2'd0, ST_RDATA = 2'd1, ST_BRESP = 2'd2;
    reg [1:0] state;

    wire do_read  = (state == ST_IDLE) && arvalid;
    wire do_write = (state == ST_IDLE) && !arvalid && awvalid && wvalid;

    wire                  sram_en   = do_read || do_write;
    wire                  sram_we   = do_write;
    wire [ADDRW-1:0]      sram_addr = do_read ? araddr : awaddr;
    wire [DATA_WIDTH-1:0] sram_dout;

    sram_sync #(.WIDTH(DATA_WIDTH), .DEPTH(DEPTH), .ADDRW(ADDRW)) u_sram (
        .clk  (clk),
        .en   (sram_en),
        .we   (sram_we),
        .addr (sram_addr),
        .din  (wdata),
        .dout (sram_dout)
    );

    assign arready = do_read;
    assign awready = do_write;
    assign wready  = do_write;

    assign rvalid = (state == ST_RDATA);
    assign rdata  = sram_dout;
    assign rresp  = 2'b00;   // OKAY, always
    assign rlast  = 1'b1;    // every transfer is exactly 1 beat -- see SCOPE note

    assign bvalid = (state == ST_BRESP);
    assign bresp  = 2'b00;   // OKAY, always

    always @(posedge clk) begin
        if (reset) begin
            state <= ST_IDLE;
        end else begin
            case (state)
                ST_IDLE:  if (do_read) state <= ST_RDATA;
                          else if (do_write) state <= ST_BRESP;
                ST_RDATA: if (rvalid && rready) state <= ST_IDLE;
                ST_BRESP: if (bvalid && bready) state <= ST_IDLE;
                default:  state <= ST_IDLE;
            endcase
        end
    end
endmodule
