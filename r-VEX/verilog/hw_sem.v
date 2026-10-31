//======================================================================
// r-VEX | Hardware semaphores for the multi-core system (2026-10-31)
//----------------------------------------------------------------------
// NSEM one-bit locks shared by N cores, memory-mapped at the top 256 bytes
// of the data address space (rvex_system decodes d_addr[AW-1:8] == all ones;
// word k is at 0xFF00 + 4k). The accesses bypass the L1D caches.
//   ldw  : test-and-set. Returns the old value (0 = acquired) and sets the
//          lock to 1, atomically.
//   stw  : release. Writes wdata[0] into the lock (store 0 to free it).
// Same timing contract as an L1D hit: combinational rdata/ready, the side
// effect happens at the posedge of the cycle in which ready = 1. rvex_core
// raises d_req only for a packet that commits when d_ready is high, so a
// lock is never set by a packet that is then held.
// When several cores access the block in one cycle, one of them is served
// (round-robin among the requesters) and the others see ready = 0 and retry
// in the next cycle, so a spinning core cannot starve the others.
// Consistency: the core is in-order and blocking and the L1Ds commit a
// store only with the line in M, so every store before the release is
// globally visible when the release completes (sequential consistency
// without a fence).
//======================================================================
module hw_sem #(
    parameter N    = 2,
    parameter NSEM = 64
) (
    input  wire          clk,
    input  wire          reset,
    input  wire [N-1:0]  req,
    input  wire [N-1:0]  we,
    input  wire [6*N-1:0] idx,     // word index inside the block (addr[7:2])
    input  wire [N-1:0]  wbit,     // wdata[0] of a store
    output wire [N-1:0]  ready,
    output wire [N-1:0]  rbit,
    output reg  [31:0]   n_acq,    // statistics: successful test-and-sets
    output reg  [31:0]   n_spin    // test-and-sets that found the lock taken
);
    reg [NSEM-1:0] lock;
    reg [$clog2(N > 1 ? N : 2)-1:0] ptr;   // round-robin start
    integer k;
    reg [N-1:0] win;
    reg found;
    always @(*) begin
        win = {N{1'b0}}; found = 1'b0;
        for (k = 0; k < N; k = k + 1)
            if (!found && req[(ptr + k) % N]) begin win[(ptr + k) % N] = 1'b1; found = 1'b1; end
    end
    genvar g;
    generate for (g = 0; g < N; g = g + 1) begin : port
        assign ready[g] = win[g];
        assign rbit[g]  = lock[idx[6*g +: 6] % NSEM];
    end endgenerate

    always @(posedge clk) begin
        if (reset) begin
            lock <= {NSEM{1'b0}}; ptr <= 0; n_acq <= 0; n_spin <= 0;
        end else begin
            for (k = 0; k < N; k = k + 1)
                if (win[k]) begin
                    if (we[k]) lock[idx[6*k +: 6] % NSEM] <= wbit[k];
                    else begin
                        lock[idx[6*k +: 6] % NSEM] <= 1'b1;
                        if (lock[idx[6*k +: 6] % NSEM]) n_spin <= n_spin + 1;
                        else                            n_acq  <= n_acq + 1;
                    end
                    ptr <= (k + 1) % N;
                end
        end
    end
endmodule
