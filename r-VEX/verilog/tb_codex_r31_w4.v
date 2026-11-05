// Round 31 W4-exact: own WBB reload collides with peer RDX in one grant cycle.
`timescale 1ns/1ps
module tb_codex_r31_w4;
  localparam AW=16, A=32'h2000, B=32'h2040;
  reg clk=0, reset=1; always #5 clk=~clk;
  reg [1:0] c_req=0,c_we=0; reg [31:0] c_addr[0:1],c_wdata[0:1];
  wire [31:0] c_rdata[0:1]; wire [1:0] c_ready;
  wire [1:0] b_req,b_pf,b_gnt,b_done,s_valid,s_hit_m; wire [3:0] b_op;
  wire [2*AW-1:0] b_addr; wire [255:0] b_wline,s_line; wire [127:0] b_rline;
  wire [1:0] s_op; wire [AW-1:0] s_addr;
  wire l2_req,l2_we,l2_done; wire [AW-1:0] l2_addr; wire [127:0] l2_wline,l2_rline;
  integer errors=0,coverage=0,k,phase; reg [31:0] r;
  genvar g;
  generate for(g=0;g<2;g=g+1) begin:c
    l1d_cache #(.AW(AW),.SETS(4),.FWD(1),.WBB(1),.DPF(4),.DPF_DEG(1),.DPF_HIST(4)) l1d(
      .clk(clk),.reset(reset),.c_req(c_req[g]),.c_we(c_we[g]),.c_addr(c_addr[g]),.c_wdata(c_wdata[g]),
      .c_rdata(c_rdata[g]),.c_ready(c_ready[g]),.b_req(b_req[g]),.b_pf(b_pf[g]),
      .b_op(b_op[2*g+:2]),.b_addr(b_addr[AW*g+:AW]),.b_wline(b_wline[128*g+:128]),
      .b_gnt(b_gnt[g]),.b_done(b_done[g]),.b_rline(b_rline),.s_valid(s_valid[g]),
      .s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m[g]),.s_line(s_line[128*g+:128]));
  end endgenerate
  coh_bus #(.N(2),.AW(AW),.FAST(1)) bus(.clk(clk),.reset(reset),.m_req(b_req),.m_pf(b_pf),
    .m_op(b_op),.m_addr(b_addr),.m_wline(b_wline),.m_gnt(b_gnt),.m_done(b_done),.m_rline(b_rline),
    .s_valid(s_valid),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m),.s_line(s_line),
    .l2_req(l2_req),.l2_we(l2_we),.l2_addr(l2_addr),.l2_wline(l2_wline),.l2_done(l2_done),.l2_rline(l2_rline));
  l2_cache #(.AW(AW),.SETS(8),.L2_LAT(2),.MEM_LAT(5),.FAST(1),.WBUF(1)) l2(
    .clk(clk),.reset(reset),.req(l2_req),.we(l2_we),.addr(l2_addr),.wline(l2_wline),.done(l2_done),.rline(l2_rline));
  task ck; input ok; input [8*72-1:0] msg; begin if(!ok) begin $display("FAIL phase=%0d %0s",phase,msg); errors=errors+1; end end endtask
  task acc; input integer p; input we; input [31:0] a,d; output [31:0] q; begin
    @(negedge clk); c_req[p]=1;c_we[p]=we;c_addr[p]=a;c_wdata[p]=d; #1;
    while(!c_ready[p]) begin @(negedge clk);#1;end q=c_rdata[p]; @(posedge clk);#1;c_req[p]=0;
  end endtask
  task check_copy; input [31:0] a,v; integer i,w; reg [AW-5:0] la; begin
    la=a[AW-1:4];i=la[1:0];w=a[3:2];
    if(c[0].l1d.st[i]!=0 && c[0].l1d.tg[i]==la[AW-5:2]) ck(c[0].l1d.dat[i*4+w]===v,"core0 array value");
    if(c[1].l1d.st[i]!=0 && c[1].l1d.tg[i]==la[AW-5:2]) ck(c[1].l1d.dat[i*4+w]===v,"core1 array value");
    if(c[0].l1d.wv && c[0].l1d.wa==la) ck((c[0].l1d.wd>>(32*w))===v,"core0 WBB value");
    if(c[1].l1d.wv && c[1].l1d.wa==la) ck((c[1].l1d.wd>>(32*w))===v,"core1 WBB value");
    ck(l2.peek(a)===v,"L2/memory coherent value");
  end endtask
  always @(negedge clk) if(!reset) begin
    if(c[0].l1d.s_wbb && c[0].l1d.wsel && b_gnt==2'b10) coverage=coverage+1;
    if(c[0].l1d.wv && c[1].l1d.st[c[0].l1d.wa[1:0]]==2 && c[1].l1d.tg[c[0].l1d.wa[1:0]]==c[0].l1d.wa[AW-5:2]) begin
      $display("FAIL OWNER WBB vs peer M"); errors=errors+1;
    end
  end
  initial begin
    c_addr[0]=0;c_addr[1]=0;c_wdata[0]=0;c_wdata[1]=0;
    for(k=0;k<(1<<(AW-2));k=k+1) l2.mem[k]=32'h90000000+k;
    repeat(3)@(posedge clk);#1;reset=0;
    // Phase sweep changes when peer RDX is raised around the FAST completion gap.
    for(phase=0;phase<4;phase=phase+1) begin
      acc(0,1,A,32'h11110000+phase,r);
      // B miss parks A. Keep request asserted and switch it to reload A as B completes.
      @(negedge clk);c_req[0]=1;c_we[0]=1;c_addr[0]=B;c_wdata[0]=32'h22220000+phase;
      wait(c[0].l1d.wv); wait(b_done[0]); @(posedge clk);#1;
      c_we[0]=0;c_addr[0]=A;
      repeat(phase) @(negedge clk);
      c_req[1]=1;c_we[1]=1;c_addr[1]=A;c_wdata[1]=32'h33330000+phase;#1;
      while(!c_ready[1]) @(negedge clk); @(posedge clk);#1;c_req[1]=0;
      while(!c_ready[0]) @(negedge clk); r=c_rdata[0]; @(posedge clk);#1;c_req[0]=0;
      ck(r===32'h33330000+phase,"own reload returns peer store");
      acc(0,0,A,0,r);ck(r===32'h33330000+phase,"core0 final A");
      acc(1,0,A,0,r);ck(r===32'h33330000+phase,"core1 final A");
      repeat(20)@(posedge clk);
      check_copy(A,32'h33330000+phase); check_copy(B,32'h22220000+phase);
    end
    $display("R31 W4 coverage exact=%0d wb0=%0d wbbsnp0=%0d c2c=%0d",coverage,c[0].l1d.n_wb,c[0].l1d.n_wbbsnp,bus.n_c2c);
    if(coverage==0) begin $display("FAIL vacuous W4");errors=errors+1;end
    if(errors==0)$display("RESULT: CODEX R31 W4 PASSED");else $display("RESULT: CODEX R31 W4 FAILED (%0d)",errors);
    $finish;
  end
  initial begin #500000;$display("RESULT: CODEX R31 W4 FAILED TIMEOUT");$finish;end
endmodule
