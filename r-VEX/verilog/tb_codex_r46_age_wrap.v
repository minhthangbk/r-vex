// Round 46 review probe for prefetch.md DPF-14 stamp wrap.
// R46: the 8-bit stamps lost the order after 255 allocations (F2). r23 fix: an age matrix; the rows
// below mark entries the way l1d_cache.v does and must all pass.
`timescale 1ns/1ps
module tb_codex_r46_age_wrap;
  localparam AW=16; reg clk=0,reset=1; always #5 clk=~clk;
  reg [1:0] c_req=0,c_we=0; reg [31:0] c_addr[0:1],c_wdata[0:1];
  wire [31:0] c_rdata[0:1]; wire [1:0] c_ready,b_req,b_pf,b_gnt,b_done,s_valid,s_hit_m;
  wire [3:0] b_op; wire [31:0] b_addr; wire [255:0] b_wline,s_line; wire [127:0] b_rline;
  wire [1:0] s_op; wire [15:0] s_addr; wire l2_req,l2_we,l2_done; wire [15:0] l2_addr;
  wire [127:0] l2_wline,l2_rline;
  genvar g; generate for(g=0;g<2;g=g+1) begin:c
    l1d_cache #(.AW(AW),.SETS(16),.DPF(4),.DPF_DEG(4),.DPF_HIST(4),.DPF_STR(1),.DPF_AGE(1)) l1d(
      .clk(clk),.reset(reset),.c_req(c_req[g]),.c_we(c_we[g]),.c_addr(c_addr[g]),.c_wdata(c_wdata[g]),
      .c_rdata(c_rdata[g]),.c_ready(c_ready[g]),.b_req(b_req[g]),.b_pf(b_pf[g]),.b_op(b_op[2*g+:2]),
      .b_addr(b_addr[16*g+:16]),.b_wline(b_wline[128*g+:128]),.b_gnt(b_gnt[g]),.b_done(b_done[g]),
      .b_rline(b_rline),.s_valid(s_valid[g]),.s_op(s_op),.s_addr(s_addr),
      .s_hit_m(s_hit_m[g]),.s_line(s_line[128*g+:128]));
  end endgenerate
  coh_bus #(.N(2),.AW(AW)) bus(.clk(clk),.reset(reset),.m_req(b_req),.m_pf(b_pf),.m_op(b_op),
    .m_addr(b_addr),.m_wline(b_wline),.m_gnt(b_gnt),.m_done(b_done),.m_rline(b_rline),
    .s_valid(s_valid),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m),.s_line(s_line),
    .l2_req(l2_req),.l2_we(l2_we),.l2_addr(l2_addr),.l2_wline(l2_wline),.l2_done(l2_done),.l2_rline(l2_rline));
  l2_cache #(.AW(AW),.SETS(64),.L2_LAT(3),.MEM_LAT(7)) l2(.clk(clk),.reset(reset),.req(l2_req),
    .we(l2_we),.addr(l2_addr),.wline(l2_wline),.done(l2_done),.rline(l2_rline));
  integer errors=0,k;
  task want; input [8*40-1:0] n; input integer got,exp;
    begin @(posedge clk); #1; if(got!==exp) begin $display("FAIL %0s got %0d expected %0d",n,got,exp); errors=errors+1; end
      else $display("ok   %0s",n); end endtask
  task mark; input integer e; integer j; begin for(j=0;j<4;j=j+1) begin c[0].l1d.ob[e][j]=0; c[0].l1d.ob[j][e]=1; end end endtask
  task clearp; integer j; begin for(j=0;j<4;j=j+1) begin c[0].l1d.pp[j]=0; c[0].l1d.pv[j]=0; end end endtask
  initial begin
    for(k=0;k<(1<<(AW-2));k=k+1) l2.mem[k]=0;
    repeat(3) @(posedge clk); #1; reset=0; force c[0].l1d.busy=1'b1; // keep injected pending rows from issuing
    // r23 Claude after Codex R46 F2: the RTL keeps an age matrix ob[i][j] (i pending before j) instead of 8-bit
    // stamps; mark() makes an entry the youngest the way l1d_cache.v does, so any number of later allocations
    // keeps the order (the stamp rows of R46 no longer apply).
    clearp; mark(0); mark(1); c[0].l1d.pp[0]=1; c[0].l1d.pp[1]=1; #1 want("ordinary oldest",c[0].l1d.pqi,0);
    clearp; mark(1); mark(0); c[0].l1d.pp[0]=1; c[0].l1d.pp[1]=1; #1 want("order by marking, not index",c[0].l1d.pqi,1);
    // entry 0 pending while entries 1..3 become pending and are dropped 300 times
    clearp; mark(0); c[0].l1d.pp[0]=1;
    for(k=0;k<300;k=k+1) begin mark(1 + k % 3); end
    c[0].l1d.pp[1]=1; c[0].l1d.pp[2]=1; #1 want("long-pending (300 younger markings) oldest",c[0].l1d.pqi,0);
    clearp; mark(2); mark(1); mark(3); c[0].l1d.pp[1]=1; c[0].l1d.pp[2]=1; c[0].l1d.pp[3]=1;
    #1 want("three pending, oldest marked first",c[0].l1d.pqi,2);
    if(errors==0) $display("RESULT: CODEX R46 AGE WRAP PASSED");
    else $display("RESULT: CODEX R46 AGE WRAP FAILED errors=%0d",errors);
    $finish;
  end
endmodule
