//======================================================================
// tb_hw_sem: constrained-random test of hw_sem.v (2026-10-31)
//----------------------------------------------------------------------
// N agents each loop: pick a lock k in 0..3, spin on test-and-set until it
// returns 0, hold it for a random time while doing a NON-atomic
// read-modify-write of counter[k] (read, wait, write+1), then release.
// Checks every cycle:
//   - mutual exclusion: at most one agent holds lock k;
//   - the hw lock bit is 1 whenever someone holds it;
//   - starvation: no agent spins longer than MAXWAIT cycles;
// and at the end counter[k] == number of critical sections on k.
// Requests that are not granted (ready = 0) must be retried unchanged.
//======================================================================
`timescale 1ns/1ps
module tb_hw_sem;
    localparam N = 2, NL = 4, MAXWAIT = 2000;
    reg clk = 0, reset = 1;
    always #5 clk = ~clk;
    reg  [N-1:0] req, we, wbit;
    reg  [6*N-1:0] idx;
    wire [N-1:0] ready, rbit;
    wire [31:0] n_acq, n_spin;
    hw_sem #(.N(N)) dut (.clk(clk), .reset(reset), .req(req), .we(we), .idx(idx), .wbit(wbit),
                         .ready(ready), .rbit(rbit), .n_acq(n_acq), .n_spin(n_spin));

    integer seed, seed0, cycles, i, k, fail, cs_total;
    integer st [0:N-1];          // 0 idle, 1 spin, 2 read, 3 hold, 4 write, 5 release
    integer lk [0:N-1], tmo [0:N-1], waitc [0:N-1], val [0:N-1];
    integer counter [0:NL-1], cs [0:NL-1], holders;

    initial begin
        if (!$value$plusargs("SEED=%d", seed)) seed = 1;
        seed0 = seed;
        if (!$value$plusargs("CYCLES=%d", cycles)) cycles = 50000;
        fail = 0; cs_total = 0;
        for (k = 0; k < NL; k = k + 1) begin counter[k] = 0; cs[k] = 0; end
        for (i = 0; i < N; i = i + 1) begin st[i] = 0; tmo[i] = 0; waitc[i] = 0; end
        req = 0; we = 0; wbit = 0; idx = 0;
        repeat (3) @(posedge clk);
        reset = 0;
        repeat (cycles) begin
            // drive requests for this cycle (state from the last edge)
            #1;
            for (i = 0; i < N; i = i + 1) begin
                req[i] = (st[i] == 1) || (st[i] == 5);
                we[i] = (st[i] == 5); wbit[i] = 1'b0;
                idx[6*i +: 6] = lk[i];
            end
            #1;
            // checks on the combinational outputs
            for (k = 0; k < NL; k = k + 1) begin
                holders = 0;
                for (i = 0; i < N; i = i + 1) if (st[i] >= 2 && st[i] <= 5 && lk[i] == k) holders = holders + 1;
                if (holders > 1) begin fail = fail + 1; $display("FAIL mutual exclusion on lock %0d", k); end
                if (holders == 1 && !dut.lock[k]) begin fail = fail + 1; $display("FAIL lock %0d held but bit 0", k); end
            end
            @(posedge clk);
            #0;
            for (i = 0; i < N; i = i + 1) begin
                case (st[i])
                    0: if (tmo[i] == 0) begin lk[i] = {$random(seed)} % NL; st[i] = 1; waitc[i] = 0; end
                       else tmo[i] = tmo[i] - 1;
                    1: if (ready[i]) begin
                           if (rbit[i] == 0) begin st[i] = 2; tmo[i] = {$random(seed)} % 4; end
                           else waitc[i] = waitc[i] + 1;
                       end else waitc[i] = waitc[i] + 1;
                    2: begin val[i] = counter[lk[i]]; st[i] = 3; end
                    3: if (tmo[i] == 0) st[i] = 4; else tmo[i] = tmo[i] - 1;
                    4: begin counter[lk[i]] = val[i] + 1; cs[lk[i]] = cs[lk[i]] + 1; st[i] = 5; end
                    5: if (ready[i]) begin st[i] = 0; tmo[i] = {$random(seed)} % 6; end
                endcase
                if (waitc[i] > MAXWAIT) begin
                    fail = fail + 1; $display("FAIL agent %0d starved on lock %0d", i, lk[i]); waitc[i] = 0;
                end
            end
        end
        for (k = 0; k < NL; k = k + 1) begin
            cs_total = cs_total + cs[k];
            if (counter[k] != cs[k]) begin fail = fail + 1; $display("FAIL counter[%0d]=%0d cs=%0d", k, counter[k], cs[k]); end
        end
        $display("HW_SEM seed=%0d cycles=%0d critical_sections=%0d acq=%0d spin=%0d", seed0, cycles, cs_total, n_acq, n_spin);
        if (fail == 0 && cs_total > 100) $display("RESULT: HW_SEM PASSED");
        else $display("RESULT: HW_SEM FAILED (%0d errors)", fail);
        $finish;
    end
endmodule
