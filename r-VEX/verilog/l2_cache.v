//======================================================================
// r-VEX | Shared L2 cache + main memory
//----------------------------------------------------------------------
// Direct-mapped, 4-word lines, write-back + write-allocate, behind the
// coherence bus. One request at a time (the bus is atomic):
//   read  : line -> rline
//   write : full-line write (L1 write-back or flush of an M line)
// Timing: L2_LAT cycles for a hit; a miss adds MEM_LAT for writing back a
// dirty victim and MEM_LAT for fetching the line (not needed for a write,
// which overwrites the whole line). Main memory `mem` is word addressed and
// is what testbenches $readmemh the data image into.
//======================================================================
module l2_cache #(
    parameter AW      = 16,
    parameter SETS    = 256,
    parameter L2_LAT  = 4,
    parameter MEM_LAT = 20
) (
    input  wire          clk,
    input  wire          reset,
    input  wire          req,          // level; held until done
    input  wire          we,
    input  wire [AW-1:0] addr,         // line address
    input  wire [127:0]  wline,
    output reg           done,         // 1-cycle pulse
    output reg  [127:0]  rline
);
    localparam IW = $clog2(SETS);
    localparam TW = AW - 4 - IW;

    reg [31:0]   mem [0:(1<<(AW-2))-1];
    reg          vld [0:SETS-1];
    reg          drt [0:SETS-1];
    reg [TW-1:0] tg  [0:SETS-1];
    reg [127:0]  dat [0:SETS-1];

    reg [31:0] n_hit, n_miss, n_wb;

    localparam S_IDLE = 2'd0, S_LOOK = 2'd1, S_WBACK = 2'd2, S_FETCH = 2'd3;
    reg [1:0]  state;
    reg [15:0] cnt;

    wire [IW-1:0] idx = addr[4+IW-1:4];
    wire [TW-1:0] tag = addr[AW-1:4+IW];
    wire          hit = vld[idx] && tg[idx] == tag;
    wire [AW-3:0] wa  = addr[AW-1:2];                 // word address of the line
    wire [AW-3:0] va  = {tg[idx], idx, 2'b00};        // victim word address

    integer k;
    always @(posedge clk) begin
        done <= 1'b0;
        if (reset) begin
            for (k = 0; k < SETS; k = k + 1) begin vld[k] <= 1'b0; drt[k] <= 1'b0; end
            state <= S_IDLE; cnt <= 0; n_hit <= 0; n_miss <= 0; n_wb <= 0;
        end else case (state)
            S_IDLE: if (req && !done) begin state <= S_LOOK; cnt <= L2_LAT - 1; end
            S_LOOK: if (cnt != 0) cnt <= cnt - 1;
                    else if (hit) begin
                        n_hit <= n_hit + 1;
                        if (we) begin dat[idx] <= wline; drt[idx] <= 1'b1; end
                        else rline <= dat[idx];
                        done <= 1'b1; state <= S_IDLE;
                    end else begin
                        n_miss <= n_miss + 1;
                        if (vld[idx] && drt[idx]) begin
                            state <= S_WBACK; cnt <= MEM_LAT - 1; n_wb <= n_wb + 1;
                        end else begin
                            state <= S_FETCH; cnt <= we ? 0 : MEM_LAT - 1;
                        end
                    end
            S_WBACK: if (cnt != 0) cnt <= cnt - 1;
                     else begin
                         mem[va+0] <= dat[idx][31:0];   mem[va+1] <= dat[idx][63:32];
                         mem[va+2] <= dat[idx][95:64];  mem[va+3] <= dat[idx][127:96];
                         drt[idx] <= 1'b0;
                         state <= S_FETCH; cnt <= we ? 0 : MEM_LAT - 1;
                     end
            S_FETCH: if (cnt != 0) cnt <= cnt - 1;
                     else begin
                         vld[idx] <= 1'b1; tg[idx] <= tag;
                         if (we) begin dat[idx] <= wline; drt[idx] <= 1'b1; end
                         else begin
                             dat[idx] <= {mem[wa+3], mem[wa+2], mem[wa+1], mem[wa+0]};
                             rline    <= {mem[wa+3], mem[wa+2], mem[wa+1], mem[wa+0]};
                             drt[idx] <= 1'b0;
                         end
                         done <= 1'b1; state <= S_IDLE;
                     end
        endcase
    end

    // coherent peek (below the L1s): L2 copy if present, else memory
    function [31:0] peek;
        input [AW-1:0] a;
        reg [IW-1:0] i; reg [TW-1:0] t;
        begin
            i = a[4+IW-1:4]; t = a[AW-1:4+IW];
            if (vld[i] && tg[i] == t) peek = dat[i] >> (32 * a[3:2]);
            else peek = mem[a[AW-1:2]];
        end
    endfunction
endmodule
