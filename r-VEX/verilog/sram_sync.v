//======================================================================
// r-VEX SoC phase 1 | Synchronous single-port SRAM macro (behavioral)
//----------------------------------------------------------------------
// Models the timing a real foundry SRAM-compiler macro gives, NOT the
// idealized combinational `reg [W-1:0] mem [0:D-1]` arrays rvex_core.v
// used until round 25: address/CEN/WEN are latched on the rising edge,
// there is no combinational address->dout path, and read data is only
// valid on the cycle AFTER the address was presented (address-to-Q
// latency = 1 cycle, matching the standard "RAM inference" coding style
// -- registered address, data assignment inside always @(posedge clk)).
// Grounded via NotebookLM digital_design round-25 query (ModernSoC
// textbook Fig 8.27 RAM-inference coding style + DFT.pdf pipelined
// read/write latency discussion) -- see reviews/rvex-round25-*.html.
//
// Single read/write port, byte-write-enable-free (whole-word writes
// only -- matches rvex_core.v's existing dmem access granularity, where
// sub-word store masking already happens inside mem_unit.v before the
// word reaches memory). Write-first-hidden: a write and a read to the
// SAME address in the same cycle is undefined for real SRAM macros: we
// model "write wins, old data not observable" (read returns don't-care
// during a same-address write), since nothing in this project relies on
// read-during-write forwarding.
//======================================================================
module sram_sync #(
    parameter WIDTH = 32,
    parameter DEPTH = 256,
    parameter ADDRW = 8      // must satisfy 2**ADDRW >= DEPTH
) (
    input  wire             clk,
    input  wire             en,      // chip enable: latch addr/we this cycle
    input  wire             we,      // 1 = write, 0 = read
    input  wire [ADDRW-1:0] addr,
    input  wire [WIDTH-1:0] din,
    output wire [WIDTH-1:0] dout     // valid the cycle AFTER en was asserted
);
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDRW-1:0] addr_r;
    reg             valid_r;   // was a READ latched last cycle (vs write/idle)?

    always @(posedge clk) begin
        if (en) begin
            addr_r  <= addr;
            valid_r <= ~we;
            if (we) mem[addr] <= din;
        end else begin
            valid_r <= 1'b0;
        end
    end

    // Registered-address read: combinational from the LATCHED address, so
    // dout only reflects an access that was presented on a prior edge --
    // there is no same-cycle addr->dout path, unlike the old reg-array model.
    assign dout = mem[addr_r];

    // Preload hook for testbenches/hex images (mirrors dut.imem/dut.dmem
    // hierarchical access the existing suite already relies on).
    integer i;
    initial for (i = 0; i < DEPTH; i = i + 1) mem[i] = {WIDTH{1'b0}};
endmodule
