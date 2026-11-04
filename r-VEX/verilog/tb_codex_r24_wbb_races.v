// Directed WBB races: a snoop of the dirty array victim must block parking,
// and a demand from the other core must consume a parked owner before its
// background drain.  Complements the random WBB-M8 coverage in tb_coherence.
`timescale 1ns/1ps
module tb_codex_r24_wbb_races;
    localparam AW=16, A=32'h1000, B=32'h1040, D=32'h1010, E=32'h1050;
    reg clk=0, reset=1; always #5 clk=~clk;
    reg [1:0] c_req, c_we;
    reg [31:0] c_addr[0:1], c_wdata[0:1];
    wire [31:0] c_rdata[0:1]; wire [1:0] c_ready;
    wire [1:0] b_req,b_pf,b_gnt,b_done,s_valid,s_hit_m;
    wire [3:0] b_op; wire [2*AW-1:0] b_addr; wire [255:0] b_wline,s_line;
    wire [127:0] b_rline; wire [1:0] s_op; wire [AW-1:0] s_addr;
    wire l2_req,l2_we,l2_done; wire [AW-1:0] l2_addr; wire [127:0] l2_wline,l2_rline;
    integer errors=0, k, snap;

    genvar g;
    generate for(g=0;g<2;g=g+1) begin:c
        l1d_cache #(.AW(AW),.SETS(4),.FWD(1),.WBB(1),.DPF(4),.DPF_DEG(1)) l1d(
          .clk(clk),.reset(reset),.c_req(c_req[g]),.c_we(c_we[g]),.c_addr(c_addr[g]),
          .c_wdata(c_wdata[g]),.c_rdata(c_rdata[g]),.c_ready(c_ready[g]),
          .b_req(b_req[g]),.b_pf(b_pf[g]),.b_op(b_op[2*g+:2]),.b_addr(b_addr[AW*g+:AW]),
          .b_wline(b_wline[128*g+:128]),.b_gnt(b_gnt[g]),.b_done(b_done[g]),.b_rline(b_rline),
          .s_valid(s_valid[g]),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m[g]),
          .s_line(s_line[128*g+:128]));
    end endgenerate
    coh_bus #(.N(2),.AW(AW),.FAST(1)) bus(
      .clk(clk),.reset(reset),.m_req(b_req),.m_pf(b_pf),.m_op(b_op),.m_addr(b_addr),
      .m_wline(b_wline),.m_gnt(b_gnt),.m_done(b_done),.m_rline(b_rline),
      .s_valid(s_valid),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m),.s_line(s_line),
      .l2_req(l2_req),.l2_we(l2_we),.l2_addr(l2_addr),.l2_wline(l2_wline),
      .l2_done(l2_done),.l2_rline(l2_rline));
    l2_cache #(.AW(AW),.SETS(8),.L2_LAT(2),.MEM_LAT(5),.FAST(1),.WBUF(1)) l2(
      .clk(clk),.reset(reset),.req(l2_req),.we(l2_we),.addr(l2_addr),.wline(l2_wline),
      .done(l2_done),.rline(l2_rline));

    task access;
      input integer p; input we; input [31:0] a,d; output [31:0] r;
      begin
        @(negedge clk); c_req[p]=1; c_we[p]=we; c_addr[p]=a; c_wdata[p]=d;
        #1; wait(c_ready[p]); #1; r=c_rdata[p];
        @(posedge clk); #1; c_req[p]=0;
      end
    endtask
    task check; input ok; input [8*58-1:0] what;
      begin if(ok) $display("PASS %0s",what); else begin $display("FAIL %0s",what); errors=errors+1; end end
    endtask
    reg [31:0] r;
    initial begin
      c_req=0;c_we=0;c_addr[0]=0;c_addr[1]=0;c_wdata[0]=0;c_wdata[1]=0;
      for(k=0;k<(1<<(AW-2));k=k+1) l2.mem[k]=32'h90000000+k;
      repeat(3) @(posedge clk); #1; reset=0;

      // Core 0 owns dirty A. Since its transaction was last, simultaneous
      // requests choose core 1 first: its RDX(A) snoops the exact victim that
      // core 0 would otherwise park while missing on same-index B.
      access(0,1,A,32'h11112222,r); snap=c[0].l1d.n_wbb;
      repeat(2) @(posedge clk); #1; // leave the completion gap; keep last=core 0
      @(negedge clk); c_req=2'b11; c_we=2'b10;
      c_addr[0]=B; c_addr[1]=A; c_wdata[1]=32'h33334444;
      #1; check(b_gnt==2'b10 && c[0].l1d.s_block && !c[0].l1d.park,
                "s_block prevents park when other core snoops victim");
      wait(c_ready[1]); @(posedge clk); #1; c_req[1]=0;
      wait(c_ready[0]); @(posedge clk); #1; c_req[0]=0;
      check(c[0].l1d.n_wbb==snap && !c[0].l1d.wv,
            "snooped victim was not duplicated in WBB");
      access(0,0,A,0,r); check(r==32'h33334444,"post-race load sees other core store");

      // Park D behind miss E. Present core 1's demand while E is still in
      // flight: demand priority keeps the background WB from winning, then
      // the RDX snoops and consumes the parked owner copy.
      access(0,1,D,32'h55556666,r); snap=c[0].l1d.n_wbbsnp;
      @(negedge clk); c_req[0]=1;c_we[0]=0;c_addr[0]=E;
      wait(c[0].l1d.wv); check(c[0].l1d.wa==D[AW-1:4],"dirty D parked");
      @(negedge clk); c_req[1]=1;c_we[1]=1;c_addr[1]=D;c_wdata[1]=32'h77778888;
      wait(c_ready[1]); @(posedge clk); #1;c_req[1]=0;
      wait(c_ready[0]); @(posedge clk); #1;c_req[0]=0;
      check(c[0].l1d.n_wbbsnp==snap+1 && !c[0].l1d.wv,
            "other demand consumes parked owner before background drain");
      access(0,0,D,0,r); check(r==32'h77778888,"consumed-buffer line is not stale");

      if(errors==0)$display("RESULT: CODEX R24 WBB RACES PASSED");
      else $display("RESULT: CODEX R24 WBB RACES FAILED (%0d)",errors);
      $finish;
    end
    initial begin #200000;$display("RESULT: CODEX R24 WBB RACES TIMEOUT");$finish;end
endmodule
