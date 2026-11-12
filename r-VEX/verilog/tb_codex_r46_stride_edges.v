// Round 46 review probe for prefetch.md DPF-11..13.
// Run with DEG=1..4.  Two L1Ds, coh_bus and L2 are instantiated exactly as
// in tb_l1d_pf; white-box checks make the detector boundary values explicit.
`timescale 1ns/1ps
module tb_codex_r46_stride_edges;
    localparam AW=16, L1S=16;
    parameter DEG=1;
    reg clk=0, reset=1; always #5 clk=~clk;
    reg [1:0] c_req=0, c_we=0;
    reg [31:0] c_addr[0:1]; reg [31:0] c_wdata[0:1];
    wire [31:0] c_rdata[0:1]; wire [1:0] c_ready;
    wire [1:0] b_req,b_pf,b_gnt,b_done,s_valid,s_hit_m;
    wire [3:0] b_op; wire [2*AW-1:0] b_addr;
    wire [255:0] b_wline,s_line; wire [127:0] b_rline;
    wire [1:0] s_op; wire [AW-1:0] s_addr;
    wire l2_req,l2_we,l2_done; wire [AW-1:0] l2_addr;
    wire [127:0] l2_wline,l2_rline;
    genvar g;
    generate for(g=0;g<2;g=g+1) begin:c
        l1d_cache #(.AW(AW),.SETS(L1S),.DPF(4),.DPF_DEG(DEG),
                    .DPF_HIST(4),.DPF_STR(1),.DPF_AGE(1)) l1d(
          .clk(clk),.reset(reset),.c_req(c_req[g]),.c_we(c_we[g]),
          .c_addr(c_addr[g]),.c_wdata(c_wdata[g]),.c_rdata(c_rdata[g]),.c_ready(c_ready[g]),
          .b_req(b_req[g]),.b_pf(b_pf[g]),.b_op(b_op[2*g+:2]),.b_addr(b_addr[AW*g+:AW]),
          .b_wline(b_wline[128*g+:128]),.b_gnt(b_gnt[g]),.b_done(b_done[g]),.b_rline(b_rline),
          .s_valid(s_valid[g]),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m[g]),.s_line(s_line[128*g+:128]));
    end endgenerate
    coh_bus #(.N(2),.AW(AW)) bus(.clk(clk),.reset(reset),.m_req(b_req),.m_pf(b_pf),
      .m_op(b_op),.m_addr(b_addr),.m_wline(b_wline),.m_gnt(b_gnt),.m_done(b_done),.m_rline(b_rline),
      .s_valid(s_valid),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m),.s_line(s_line),
      .l2_req(l2_req),.l2_we(l2_we),.l2_addr(l2_addr),.l2_wline(l2_wline),.l2_done(l2_done),.l2_rline(l2_rline));
    l2_cache #(.AW(AW),.SETS(64),.L2_LAT(3),.MEM_LAT(7)) l2(.clk(clk),.reset(reset),
      .req(l2_req),.we(l2_we),.addr(l2_addr),.wline(l2_wline),.done(l2_done),.rline(l2_rline));

    integer errors=0,k,i,m0,p0,h0;
    task check; input [8*42-1:0] n; input got,exp;
      begin if(got!==exp) begin $display("FAIL %0s got %0d expected %0d",n,got,exp); errors=errors+1; end
            else $display("ok   %0s",n); end endtask
    task setdet; input [15:0] last,delta,now; input conf;
      begin c[0].l1d.sd_a=last; c[0].l1d.sd_s=delta; c[0].l1d.sd_c=conf;
            c_addr[0]=now; #1; end endtask
    task store; input [15:0] a; input [31:0] d;
      begin c_addr[0]=a; c_wdata[0]=d; c_we[0]=1; c_req[0]=1; #1;
        while(!c_ready[0]) begin @(posedge clk); #1; end
        @(posedge clk); #1; c_req[0]=0; c_we[0]=0; repeat(35) @(posedge clk); #1;
      end endtask
    task access1; input we; input [15:0] a; input [31:0] d;
      begin c_addr[1]=a; c_wdata[1]=d; c_we[1]=we; c_req[1]=1; #1;
        while(!c_ready[1]) begin @(posedge clk); #1; end
        @(posedge clk); #1; c_req[1]=0; c_we[1]=0; repeat(8) @(posedge clk); #1;
      end endtask

    // DPF-04: one cycle after a killing snoop no matching pending/valid entry may remain.
    integer q; reg kill_d; reg [AW-5:0] kill_line;
    always @(posedge clk) begin
      if(kill_d) for(q=0;q<4;q=q+1)
        if((c[0].l1d.pv[q]||c[0].l1d.pp[q]) && c[0].l1d.pa[q]==kill_line) begin
          $display("FAIL DPF-04 buffered line survived RDX/UPGR"); errors=errors+1;
        end
      kill_d <= s_valid[0] && (s_op==1 || s_op==2); kill_line <= s_addr[AW-1:4];
    end

    initial begin
      if(DEG<1 || DEG>4) begin $display("FAIL DEG must be 1..4"); $finish; end
      for(k=0;k<(1<<(AW-2));k=k+1) l2.mem[k]=32'h10000000+k;
      repeat(3) @(posedge clk); #1; reset=0;

      // Exact lower/upper acceptor edges and the rejected neighbours.
      setdet(16'h1000,16,16'h1010,1); check("stride 16 accepted",c[0].l1d.sh,1);
      check("stride16 target",c[0].l1d.tgt(DEG)==((16'h1010+DEG*16)>>4),1);
      setdet(16'h1000,15,16'h100f,1); check("stride 15 rejected",c[0].l1d.sh,0);
      setdet(16'h1000,255,16'h10ff,1); check("stride 255 accepted",c[0].l1d.sh,1);
      setdet(16'h1000,256,16'h1100,1); check("stride 256 rejected",c[0].l1d.sh,0);

      // A descending 20-byte stream is the unsigned AW-bit delta 65516 and
      // therefore falls back to DPF-06.  Positive wrap at the top is modular.
      setdet(16'h1020,16'hffec,16'h100c,1); check("negative stride rejected",c[0].l1d.sh,0);
      setdet(16'hfff0,16,16'h0000,1); check("top-of-AW wrap accepted",c[0].l1d.sh,1);
      check("wrapped target line",c[0].l1d.tgt(DEG)==((DEG*16)>>4),1);

      // A stride target already resident in this L1 is recognized by have()
      // and therefore must not consume/reallocate a DPF entry.
      c[0].l1d.st[0]=1; c[0].l1d.tg[0]=8'h41;
      setdet(16'h40d8,20,16'h40ec,1);
      check("cached stride target suppressed",c[0].l1d.have(c[0].l1d.tgt(1)),1);
      c[0].l1d.st[0]=0;

      // Store-only stride-20 stream: RDX demand misses are stream evidence too.
      // Only warm-up/end entries may be unused; degree 1..4 is swept outside.
      c[0].l1d.sd_a=0; c[0].l1d.sd_s=0; c[0].l1d.sd_c=0;
      m0=c[0].l1d.n_miss; p0=c[0].l1d.n_pf; h0=c[0].l1d.n_pfhit;
      for(i=0;i<16;i=i+1) store(16'h2000+20*i,32'h50000000+i);
      $display("store-only DEG=%0d miss=%0d pf=%0d hit=%0d useless=%0d",DEG,
        c[0].l1d.n_miss-m0,c[0].l1d.n_pf-p0,c[0].l1d.n_pfhit-h0,
        (c[0].l1d.n_pf-p0)-(c[0].l1d.n_pfhit-h0));
      // The <=3 / <=DEG+1 performance bounds in DPF-11 are promised only by
      // the contract's DEG=1,2 stride5 rows.  For DEG=3,4 check the semantic
      // requirement: RDX evidence allocates prefetches and at least one is used.
      if(DEG<=2) begin
        check("store-only stream heals",c[0].l1d.n_miss-m0<=3,1);
        check("store-only useless bounded",(c[0].l1d.n_pf-p0)-(c[0].l1d.n_pfhit-h0)<=DEG+1,1);
      end else begin
        check("store-only issued prefetch",c[0].l1d.n_pf-p0>0,1);
        check("store-only used prefetch",c[0].l1d.n_pfhit-h0>0,1);
      end

      // Two cores touch the same strided lines; core 1 writes one line so its
      // RDX/UPGR exercises the DPF-04 monitor above.
      c_addr[1]=16'h3000; c_req[1]=1; c_we[1]=0; #1;
      while(!c_ready[1]) begin @(posedge clk); #1; end
      @(posedge clk); #1; c_req[1]=0;
      store(16'h3000,32'hdeadc046); repeat(20) @(posedge clk);

      // Exact pending side of DPF-04.  A demand wins over the injected
      // prefetch request, so its snoop reaches core 0 while pp[0] is pending.
      c[0].l1d.pp[0]=1; c[0].l1d.pv[0]=0; c[0].l1d.pa[0]=12'h310;
      access1(1,16'h3100,32'h46aa3100);
      check("RDX kills pending",c[0].l1d.pp[0],0);
      // A snooped RD is non-killing.  Keep core 0 busy so the injected entry
      // cannot issue before core 1's demand is granted.
      c[0].l1d.pp[0]=1; c[0].l1d.pv[0]=0; c[0].l1d.pa[0]=12'h320;
      force c[0].l1d.busy=1'b1; access1(0,16'h3200,0); release c[0].l1d.busy;
      check("RD keeps pending",c[0].l1d.pp[0],1); c[0].l1d.pp[0]=0;
      // Core 1 first gets S, so its following store requests UPGR rather than
      // RDX; UPGR must kill the other core's pending shared copy as well.
      access1(0,16'h3300,0);
      c[0].l1d.pp[0]=1; c[0].l1d.pv[0]=0; c[0].l1d.pa[0]=12'h330;
      access1(1,16'h3300,32'h46aa3300);
      check("UPGR kills pending",c[0].l1d.pp[0],0);

      if(errors==0) $display("RESULT: CODEX R46 STRIDE EDGES PASSED DEG=%0d",DEG);
      else $display("RESULT: CODEX R46 STRIDE EDGES FAILED DEG=%0d errors=%0d",DEG,errors);
      $finish;
    end
endmodule
