//======================================================================
// r-VEX | L1 instruction cache (one per core)
//----------------------------------------------------------------------
// Direct-mapped, 4 packets (4 x 128 bits) per line, read-only. A miss
// refills the line from this core's program memory `rom` after FILL_LAT
// cycles (the next level for instructions; code is never written, so it
// needs no coherence). Hits are combinational, so a hit keeps the core's
// one-packet-per-cycle fetch.
// WARM = 1 (measurement only, BUG-087, 2026-11-04 r10): reset loads set k
// with program line k (packets 4k..4k+3, tag 0), so a program of at most
// SETS lines runs with no cold I-miss; WARM = 0 (default) is a cold cache.
//======================================================================
module l1i_cache #(
    parameter PC_W     = 12,
    parameter SETS     = 16,
    parameter FILL_LAT = 8,
    parameter WARM     = 0
) (
    input  wire            clk,
    input  wire            reset,
    input  wire            i_req,
    input  wire [PC_W-1:0] i_addr,      // packet address (PC)
    output wire [127:0]    i_rdata,
    output wire            i_ready
);
    localparam IW = (SETS > 1) ? $clog2(SETS) : 1;
    initial if (FILL_LAT < 1) begin   // cnt <= LAT-1 would wrap to 65535
        $display("FATAL %m: FILL_LAT=%0d must be >= 1", FILL_LAT); $finish;
    end
    // SETS must be a power of two >= 2 (a 1-line cache has no index bits)
    initial if (SETS < 2 || (SETS & (SETS - 1)) != 0) begin
        $display("FATAL %m: SETS=%0d must be a power of two >= 2", SETS); $finish;
    end
    localparam TW = PC_W - 2 - IW;

    reg [127:0]  rom [0:(1<<PC_W)-1];
    reg          vld [0:SETS-1];
    reg [TW-1:0] tg  [0:SETS-1];
    reg [127:0]  dat [0:SETS*4-1];
    reg [31:0]   n_hit, n_miss;

    wire [IW-1:0] idx = i_addr[2+IW-1:2];
    wire [TW-1:0] tag = i_addr[PC_W-1:2+IW];
    wire          hit = vld[idx] && tg[idx] == tag;
    assign i_ready = i_req && hit;
    assign i_rdata = dat[idx*4 + i_addr[1:0]];

    reg        filling;
    reg [15:0] cnt;
    wire [PC_W-1:0] base = {i_addr[PC_W-1:2], 2'b00};
    integer k;
    always @(posedge clk) begin
        if (reset) begin
            for (k = 0; k < SETS; k = k + 1) begin
                vld[k] <= WARM != 0; tg[k] <= {TW{1'b0}};
            end
            if (WARM != 0) for (k = 0; k < SETS * 4; k = k + 1) dat[k] <= rom[k];
            filling <= 1'b0; n_hit <= 0; n_miss <= 0;
        end else if (filling) begin
            if (cnt != 0) cnt <= cnt - 1;
            else begin
                for (k = 0; k < 4; k = k + 1) dat[idx*4 + k] <= rom[base + k];
                vld[idx] <= 1'b1; tg[idx] <= tag; filling <= 1'b0;
            end
        end else if (i_req && !hit) begin
            filling <= 1'b1; cnt <= FILL_LAT - 1; n_miss <= n_miss + 1;
        end else if (i_ready) n_hit <= n_hit + 1;
    end
endmodule
