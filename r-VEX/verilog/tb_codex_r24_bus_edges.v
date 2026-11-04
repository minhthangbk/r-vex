// Directed r12 bus/L2 edges not exercised explicitly by tb_bus_lat:
// FAST flush data, gap fairness, UPGR timing, L2_LAT=1/2, and a WBUF
// write miss with a dirty victim while an earlier write-behind is busy.
`timescale 1ns/1ps
module tb_codex_r24_bus_edges;
    localparam AW = 8;
    localparam OP_RD = 2'd0, OP_RDX = 2'd1, OP_UPGR = 2'd2, OP_WB = 2'd3;
    reg clk = 0, reset = 1;
    always #5 clk = ~clk;
    integer errors = 0;

    reg [1:0] m_req, m_pf;
    reg [3:0] m_op;
    reg [2*AW-1:0] m_addr;
    reg [255:0] m_wline;
    wire [1:0] m_gnt, m_done, s_valid;
    wire [127:0] m_rline;
    wire [1:0] s_op;
    wire [AW-1:0] s_addr;
    reg [1:0] s_hit_m;
    reg [255:0] s_line;
    wire l2_req, l2_we;
    wire [AW-1:0] l2_addr;
    wire [127:0] l2_wline;
    reg l2_done;
    reg [127:0] l2_rline;
    localparam [127:0] FLUSH = 128'h44444444_33333333_22222222_11111111;
    localparam [127:0] LOWER = 128'haaaaaaaa_bbbbbbbb_cccccccc_dddddddd;

    coh_bus #(.N(2), .AW(AW), .FAST(1)) bus (
        .clk(clk), .reset(reset), .m_req(m_req), .m_pf(m_pf), .m_op(m_op),
        .m_addr(m_addr), .m_wline(m_wline), .m_gnt(m_gnt), .m_done(m_done),
        .m_rline(m_rline), .s_valid(s_valid), .s_op(s_op), .s_addr(s_addr),
        .s_hit_m(s_hit_m), .s_line(s_line), .l2_req(l2_req), .l2_we(l2_we),
        .l2_addr(l2_addr), .l2_wline(l2_wline), .l2_done(l2_done), .l2_rline(l2_rline));

    reg q1, we1, q2, we2, qw, wew;
    reg [AW-1:0] a1, a2, aw;
    reg [127:0] wl1, wl2, wlw;
    wire d1, d2, dw;
    wire [127:0] rl1, rl2, rlw;
    l2_cache #(.AW(AW), .SETS(2), .L2_LAT(1), .MEM_LAT(4), .FAST(1), .WBUF(0)) l21
      (.clk(clk), .reset(reset), .req(q1), .we(we1), .addr(a1), .wline(wl1), .done(d1), .rline(rl1));
    l2_cache #(.AW(AW), .SETS(2), .L2_LAT(2), .MEM_LAT(4), .FAST(1), .WBUF(0)) l22
      (.clk(clk), .reset(reset), .req(q2), .we(we2), .addr(a2), .wline(wl2), .done(d2), .rline(rl2));
    l2_cache #(.AW(AW), .SETS(2), .L2_LAT(1), .MEM_LAT(4), .FAST(1), .WBUF(1)) l2w
      (.clk(clk), .reset(reset), .req(qw), .we(wew), .addr(aw), .wline(wlw), .done(dw), .rline(rlw));

    task check;
        input ok; input [8*56-1:0] what;
        begin
            if (!ok) begin $display("FAIL %0s", what); errors = errors + 1; end
            else $display("PASS %0s", what);
        end
    endtask

    initial begin
        m_req=0; m_pf=0; m_op=0; m_addr=0; m_wline=0; s_hit_m=0; s_line=0;
        l2_done=0; l2_rline=LOWER;
        q1=0; we1=0; a1=0; wl1=0; q2=0; we2=0; a2=0; wl2=0;
        qw=0; wew=0; aw=0; wlw=0;
        repeat (2) @(posedge clk); #1; reset=0;
        check(!bus.busy && !bus.done_d && !bus.upgr_wait && bus.last==1,
              "FAST bus registers reset");
        check(!l2w.wb_busy && !l2w.wb_pend && l2w.wb_cnt==0 && l2w.state==0,
              "WBUF registers reset");

        // Core 0 RD; core 1 supplies M. L2 returns deliberately different data.
        @(negedge clk); m_req=2'b01; m_op[1:0]=OP_RD; m_addr[AW-1:0]=8'h20;
        s_hit_m[1]=1; s_line[255:128]=FLUSH;
        @(posedge clk); #1; check(m_gnt==0 && bus.busy && l2_we && l2_wline==FLUSH,
                                  "snooped M flush is captured");
        m_req=0; s_hit_m=0;
        @(negedge clk); l2_done=1;
        #1; check(m_done==2'b01 && m_rline==FLUSH,
                  "FAST completion returns flush_line, not l2_rline");
        @(posedge clk); #1; l2_done=0;

        // Both ask in the gap. RR picks core 1, but it must wait one cycle.
        m_req=2'b11; m_op={OP_RD,OP_RD};
        #1; check(m_gnt==0, "other master blocked in FAST gap");
        @(posedge clk); #1; check(m_gnt==2'b10, "other master granted after gap");
        @(posedge clk); #1; m_req=0;
        l2_done=1; #1; check(m_done==2'b10, "second master completes");
        @(posedge clk); #1; l2_done=0;

        // UPGR completes combinationally in the cycle after its grant.
        @(posedge clk); #1;
        m_req=2'b01; m_op[1:0]=OP_UPGR;
        #1; check(m_gnt==2'b01, "UPGR granted");
        @(posedge clk); #1; m_req=0;
        check(m_done==2'b01 && !l2_req, "UPGR FAST completion timing");
        @(posedge clk); #1;

        // Preload hits and check request-cycle lookup at latency 1 versus latency 2.
        l21.vld[0]=1; l21.tg[0]=0; l21.dat[0]=FLUSH;
        l22.vld[0]=1; l22.tg[0]=0; l22.dat[0]=FLUSH;
        @(negedge clk); q1=1; q2=1; a1=0; a2=0;
        @(posedge clk); #1;
        check(d1 && rl1==FLUSH, "FAST L2_LAT=1 decides in request cycle");
        check(!d2, "FAST L2_LAT=2 does not finish too early");
        @(posedge clk); #1;
        check(d2 && rl2==FLUSH, "FAST L2_LAT=2 finishes second cycle");
        q1=0; q2=0;

        // Dirty victim at index 0, plus an older write behind. A whole-line
        // write miss needs no fetch, but must wait, then post this victim.
        @(negedge clk);
        l2w.vld[0]=1; l2w.drt[0]=1; l2w.tg[0]=0; l2w.dat[0]=FLUSH;
        l2w.wb_busy=1; l2w.wb_cnt=2;
        qw=1; wew=1; aw=8'h20; wlw=LOWER;
        @(posedge clk); #1; check(l2w.state==4 && !dw, "write miss enters MWAIT");
        // r13 BUG-145 / bus.md L2WB-05 (triage in the r13 review): the access leaves MWAIT in the last
        // write-behind cycle straight into FETCH; the r12 version of this check expected a LOOK retry
        @(posedge clk); #1; check(l2w.state==4 && l2w.wb_busy && l2w.wb_cnt==0, "still waits in the last write-behind cycle");
        @(posedge clk); #1;
        check(l2w.state==3 && l2w.wb_pend && !l2w.wb_busy && l2w.mem[0]==FLUSH[31:0],
              "L2WB-05 MWAIT to FETCH, victim posted");
        @(posedge clk); #1;
        check(dw && l2w.vld[0] && l2w.drt[0] && l2w.dat[0]==LOWER && l2w.wb_busy,
              "write miss installs and starts write behind");
        qw=0;

        if (errors==0) $display("RESULT: CODEX R24 BUS EDGES PASSED");
        else $display("RESULT: CODEX R24 BUS EDGES FAILED (%0d)", errors);
        $finish;
    end
    initial begin #10000; $display("RESULT: CODEX R24 BUS EDGES TIMEOUT"); $finish; end
endmodule
