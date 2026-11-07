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
// FAST = 1 (2026-11-04 r12, reviews/contracts/bus.md BUS-04): the cycle the
// request arrives is the first lookup cycle, so a hit takes L2_LAT cycles
// from req (r11: L2_LAT + 1).
// WBUF = 1 (bus.md L2WB-01..03): posted write buffer for a dirty victim. The
// miss fetches first and completes; the victim is written behind it, keeping
// the single memory channel busy for MEM_LAT cycles. A memory access that
// arrives meanwhile waits (n_mwait); hits do not. The victim words are put in
// `mem` when it is posted, which is what read forwarding from the buffer
// would return, so only the timing is modelled by the busy counter.
//======================================================================
module l2_cache #(
    parameter AW      = 16,
    parameter SETS    = 256,
    parameter L2_LAT  = 4,
    parameter MEM_LAT = 20,
    parameter FAST    = 0,
    parameter WBUF    = 0
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
    localparam IW = (SETS > 1) ? $clog2(SETS) : 1;
    initial if (L2_LAT < 1) begin   // cnt <= LAT-1 would wrap to 65535
        $display("FATAL %m: L2_LAT=%0d must be >= 1", L2_LAT); $finish;
    end
    initial if (MEM_LAT < 1) begin   // cnt <= LAT-1 would wrap to 65535
        $display("FATAL %m: MEM_LAT=%0d must be >= 1", MEM_LAT); $finish;
    end
    // SETS must be a power of two >= 2 (a 1-line cache has no index bits)
    initial if (SETS < 2 || (SETS & (SETS - 1)) != 0) begin
        $display("FATAL %m: SETS=%0d must be a power of two >= 2", SETS); $finish;
    end
    // BUG-187 (r18): a cache as large as the address space has no tag bits; TW is kept >= 1 and every tag is
    // taken with a shift (always 0 then), so it elaborates and works; more sets than lines in memory is an error
    localparam TW = (AW - 4 - IW > 0) ? AW - 4 - IW : 1;
    initial if (AW - 4 - IW < 0) begin
        $display("FATAL %m: SETS=%0d x 16 B is larger than the %0d-byte address space", SETS, 1 << AW); $finish;
    end

    reg [31:0]   mem [0:(1<<(AW-2))-1];
    reg          vld [0:SETS-1];
    reg          drt [0:SETS-1];
    reg [TW-1:0] tg  [0:SETS-1];
    reg [127:0]  dat [0:SETS-1];

    reg [31:0] n_hit, n_miss, n_wb;
    reg [31:0] n_wbuf, n_mwait;    // r12: victims posted, cycles a memory access waited for the write behind

    localparam S_IDLE = 3'd0, S_LOOK = 3'd1, S_WBACK = 3'd2, S_FETCH = 3'd3, S_MWAIT = 3'd4;
    reg [2:0]  state;
    reg [15:0] cnt;
    reg        wb_pend;            // WBUF: a victim was posted, its write starts when the fetch ends
    reg        wb_busy;            // WBUF: memory busy with the write behind
    reg [15:0] wb_cnt;
    // BUG-145 (r13, bus.md L2WB-05): the channel is free from the next cycle on in the last write-behind
    // cycle (wb_cnt == 0), so a lookup then fetches at once and a waiting access leaves MWAIT in that cycle
    wire       mem_free = !wb_busy || wb_cnt == 0;
    // lookup decision this cycle: end of the LOOK count, or (FAST, L2_LAT = 1) the request cycle itself
    wire       look_now = (state == S_LOOK && cnt == 0) ||
                          (FAST && L2_LAT == 1 && state == S_IDLE && req && !done);

    wire [IW-1:0] idx = addr[4+IW-1:4];
    wire [TW-1:0] tag = addr >> (4 + IW);
    wire          hit = vld[idx] && tg[idx] == tag;
    wire [AW-3:0] wa  = addr[AW-1:2];                 // word address of the line
    wire [AW-3:0] va  = {tg[idx], idx, 2'b00};        // victim word address

    integer k;
    always @(posedge clk) begin
        done <= 1'b0;
        if (reset) begin
            for (k = 0; k < SETS; k = k + 1) begin vld[k] <= 1'b0; drt[k] <= 1'b0; end
            state <= S_IDLE; cnt <= 0; n_hit <= 0; n_miss <= 0; n_wb <= 0;
            n_wbuf <= 0; n_mwait <= 0; wb_pend <= 1'b0; wb_busy <= 1'b0; wb_cnt <= 0;
        end else begin
            // WBUF: the write behind occupies the memory channel for MEM_LAT cycles
            if (wb_busy) begin
                if (wb_cnt != 0) wb_cnt <= wb_cnt - 1; else wb_busy <= 1'b0;
            end
            if (look_now) begin
                if (hit) begin
                    n_hit <= n_hit + 1;
                    if (we) begin dat[idx] <= wline; drt[idx] <= 1'b1; end
                    else rline <= dat[idx];
                    done <= 1'b1; state <= S_IDLE;
                end else begin
                    n_miss <= n_miss + 1;
                    if (WBUF && ((vld[idx] && drt[idx]) || !we) && !mem_free) begin
                        state <= S_MWAIT;           // L2WB-02: one memory channel
                    end else if (vld[idx] && drt[idx]) begin
                        n_wb <= n_wb + 1;
                        if (WBUF) begin             // L2WB-01: post the victim, fetch first
                            mem[va+0] <= dat[idx][31:0];   mem[va+1] <= dat[idx][63:32];
                            mem[va+2] <= dat[idx][95:64];  mem[va+3] <= dat[idx][127:96];
                            drt[idx] <= 1'b0; vld[idx] <= 1'b0;
                            wb_pend <= 1'b1; n_wbuf <= n_wbuf + 1;
                            state <= S_FETCH; cnt <= we ? 0 : MEM_LAT - 1;
                        end else begin
                            state <= S_WBACK; cnt <= MEM_LAT - 1;
                        end
                    end else begin
                        state <= S_FETCH; cnt <= we ? 0 : MEM_LAT - 1;
                    end
                end
            end else case (state)
                S_IDLE: if (req && !done) begin
                            state <= S_LOOK;
                            cnt <= (FAST && L2_LAT > 1) ? L2_LAT - 2 : L2_LAT - 1;
                        end
                S_LOOK: cnt <= cnt - 1;           // (cnt == 0 is look_now)
                S_MWAIT: begin
                    n_mwait <= n_mwait + 1;
                    // the write behind ends this cycle: the miss goes on now (still a miss: one request at
                    // a time, the line cannot change while it waits); a dirty victim is posted (WBUF = 1 here)
                    if (mem_free) begin
                        if (vld[idx] && drt[idx]) begin
                            n_wb <= n_wb + 1;      // L2WB-05: post the victim after the wait
                            mem[va+0] <= dat[idx][31:0];   mem[va+1] <= dat[idx][63:32];
                            mem[va+2] <= dat[idx][95:64];  mem[va+3] <= dat[idx][127:96];
                            drt[idx] <= 1'b0; vld[idx] <= 1'b0;
                            wb_pend <= 1'b1; n_wbuf <= n_wbuf + 1;
                        end
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
                             if (wb_pend) begin wb_pend <= 1'b0; wb_busy <= 1'b1; wb_cnt <= MEM_LAT - 1; end
                             done <= 1'b1; state <= S_IDLE;
                         end
                default: state <= S_IDLE;
            endcase
        end
    end

    // coherent peek (below the L1s): L2 copy if present, else memory
    function [31:0] peek;
        input [AW-1:0] a;
        reg [IW-1:0] i; reg [TW-1:0] t;
        begin
            i = a[4+IW-1:4]; t = a >> (4 + IW);
            if (vld[i] && tg[i] == t) peek = dat[i] >> (32 * a[3:2]);
            else peek = mem[a[AW-1:2]];
        end
    endfunction
endmodule
