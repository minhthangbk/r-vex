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
// IPF = 1 (2026-11-04 r11, contract prefetch.md IPF-01..06): next-line
// prefetch for the VLIW fetch stream. While the core fetches line L and no
// demand fill runs, line L+1 (if not cached) is read from the ROM into a
// one-line buffer after FILL_LAT cycles. A demand miss that finds its line
// in the buffer is served in the same cycle and installs it (no pollution
// before use); a miss on the line being prefetched waits for it; a miss on
// any other line aborts the prefetch (private ROM: no bus to release).
// IPF = 0 is the r10 cache, cycle for cycle.
//======================================================================
module l1i_cache #(
    parameter PC_W     = 12,
    parameter SETS     = 16,
    parameter FILL_LAT = 8,
    parameter WARM     = 0,
    parameter IPF      = 0
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
    reg          filling;
    reg [15:0]   cnt;

    wire [IW-1:0] idx = i_addr[2+IW-1:2];
    wire [TW-1:0] tag = i_addr[PC_W-1:2+IW];
    wire          hit = vld[idx] && tg[idx] == tag;

    // ---- next-line prefetch buffer (IPF) ----
    localparam LW = PC_W - 2;
    reg            pf_v, pf_busy;
    reg  [LW-1:0]  pf_ln;
    reg  [127:0]   pf_dat [0:3];
    reg  [15:0]    pf_cnt;
    reg  [31:0]    n_pf, n_pfhit, n_pfabort;
    wire [LW-1:0]  ln    = i_addr[PC_W-1:2];
    wire [LW-1:0]  nln   = ln + 1'b1;
    wire [IW-1:0]  nidx  = nln[IW-1:0];
    wire [TW-1:0]  ntag  = nln[LW-1:IW];
    wire           pfhit = (IPF != 0) && !hit && pf_v && pf_ln == ln;
    wire           pf_start = (IPF != 0) && i_req && !filling && !pf_busy && (hit || pfhit) &&
                              !(vld[nidx] && tg[nidx] == ntag) && !(pf_v && pf_ln == nln);
    wire           pf_late  = pf_busy && pf_ln == ln;    // the miss waits for this prefetch

    assign i_ready = i_req && (hit || pfhit);
    assign i_rdata = hit ? dat[idx*4 + i_addr[1:0]] : pf_dat[i_addr[1:0]];

    wire [PC_W-1:0] base = {i_addr[PC_W-1:2], 2'b00};
    integer k;
    always @(posedge clk) begin
        if (reset) begin
            for (k = 0; k < SETS; k = k + 1) begin
                vld[k] <= WARM != 0; tg[k] <= {TW{1'b0}};
            end
            if (WARM != 0) for (k = 0; k < SETS * 4; k = k + 1) dat[k] <= rom[k];
            filling <= 1'b0; n_hit <= 0; n_miss <= 0;
            pf_v <= 1'b0; pf_busy <= 1'b0; pf_ln <= {LW{1'b0}}; pf_cnt <= 0;
            n_pf <= 0; n_pfhit <= 0; n_pfabort <= 0;
        end else begin
          // prefetch engine (never runs while a demand fill does)
          if (pf_start) begin
              pf_busy <= 1'b1; pf_v <= 1'b0; pf_ln <= nln; pf_cnt <= FILL_LAT - 1; n_pf <= n_pf + 1;
          end else if (pf_busy) begin
              if (i_req && !hit && !pfhit && !filling && !pf_late) begin
                  pf_busy <= 1'b0; n_pfabort <= n_pfabort + 1;        // IPF-05: demand miss wins
              end else if (pf_cnt != 0) pf_cnt <= pf_cnt - 1;
              else begin
                  for (k = 0; k < 4; k = k + 1) pf_dat[k] <= rom[{pf_ln, 2'b00} + k];
                  pf_v <= 1'b1; pf_busy <= 1'b0;
              end
          end
          if (pfhit && i_req) begin                                   // IPF-04: install on use
              for (k = 0; k < 4; k = k + 1) dat[idx*4 + k] <= pf_dat[k];
              vld[idx] <= 1'b1; tg[idx] <= tag; n_pfhit <= n_pfhit + 1;
          end
          if (filling) begin
            if (cnt != 0) cnt <= cnt - 1;
            else begin
                for (k = 0; k < 4; k = k + 1) dat[idx*4 + k] <= rom[base + k];
                vld[idx] <= 1'b1; tg[idx] <= tag; filling <= 1'b0;
            end
          end else if (i_req && !hit && !pfhit && !pf_late) begin
            filling <= 1'b1; cnt <= FILL_LAT - 1; n_miss <= n_miss + 1;
          end else if (i_req && hit) n_hit <= n_hit + 1;
        end
    end
endmodule
