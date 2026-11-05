// Round 31 P3': store to own in-flight prefetch, defaults FAST/FWD/WBB/WBUF.
`timescale 1ns/1ps
module tb_codex_r31_p3;
  localparam AW=16;
  reg clk=0,reset=1;always #5 clk=~clk;
  reg [1:0] c_req=0,c_we=0;reg [31:0] c_addr[0:1],c_wdata[0:1];wire [31:0] c_rdata[0:1];wire [1:0] c_ready;
  wire [1:0] b_req,b_pf,b_gnt,b_done,s_valid,s_hit_m;wire [3:0] b_op;wire [2*AW-1:0] b_addr;
  wire [255:0] b_wline,s_line;wire [127:0] b_rline;wire [1:0] s_op;wire [AW-1:0] s_addr;
  wire l2_req,l2_we,l2_done;wire [AW-1:0] l2_addr;wire [127:0] l2_wline,l2_rline;
  integer errors=0,d,k,cov_flight=0,cov_park=0,cov_install=0,cov_peer=0,cov_no_xwb=0;reg p3_active=0;
  reg [31:0] r,rp,rl,L0,L1,L2,X,V;
  genvar g;generate for(g=0;g<2;g=g+1)begin:c
    l1d_cache #(.AW(AW),.SETS(4),.FWD(1),.WBB(1),.DPF(4),.DPF_DEG(1),.DPF_HIST(4)) l1d(
      .clk(clk),.reset(reset),.c_req(c_req[g]),.c_we(c_we[g]),.c_addr(c_addr[g]),.c_wdata(c_wdata[g]),.c_rdata(c_rdata[g]),.c_ready(c_ready[g]),
      .b_req(b_req[g]),.b_pf(b_pf[g]),.b_op(b_op[2*g+:2]),.b_addr(b_addr[AW*g+:AW]),.b_wline(b_wline[128*g+:128]),
      .b_gnt(b_gnt[g]),.b_done(b_done[g]),.b_rline(b_rline),.s_valid(s_valid[g]),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m[g]),.s_line(s_line[128*g+:128]));
  end endgenerate
  coh_bus #(.N(2),.AW(AW),.FAST(1))bus(.clk(clk),.reset(reset),.m_req(b_req),.m_pf(b_pf),.m_op(b_op),.m_addr(b_addr),.m_wline(b_wline),
    .m_gnt(b_gnt),.m_done(b_done),.m_rline(b_rline),.s_valid(s_valid),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m),.s_line(s_line),
    .l2_req(l2_req),.l2_we(l2_we),.l2_addr(l2_addr),.l2_wline(l2_wline),.l2_done(l2_done),.l2_rline(l2_rline));
  l2_cache #(.AW(AW),.SETS(8),.L2_LAT(2),.MEM_LAT(5),.FAST(1),.WBUF(1))l2(.clk(clk),.reset(reset),.req(l2_req),.we(l2_we),.addr(l2_addr),.wline(l2_wline),.done(l2_done),.rline(l2_rline));
  task ck;input ok;input[8*64-1:0]m;begin if(!ok)begin $display("FAIL d=%0d %0s",d,m);errors=errors+1;end end endtask
  task automatic acc;input integer p;input we;input[31:0]a,v;output[31:0]q;begin
    @(negedge clk);c_req[p]=1;c_we[p]=we;c_addr[p]=a;c_wdata[p]=v;#1;while(!c_ready[p])begin @(negedge clk);#1;end q=c_rdata[p];@(posedge clk);#1;c_req[p]=0;
  end endtask
  always @(negedge clk)if(!reset)begin
    if(c[0].l1d.busy&&c[0].l1d.cur_pf&&c_req[0]&&c_we[0]&&c_addr[0][AW-1:4]==c[0].l1d.cur_addr[AW-1:4])cov_flight=cov_flight+1;
    if(p3_active&&b_gnt[0]&&b_op[1:0]==3&&b_addr[AW-1:0]==X[AW-1:0])begin $display("FAIL d=%0d WB(X) before prefetch install",d);errors=errors+1;end
    if(c[0].l1d.inst&&c[0].l1d.park)begin cov_install=cov_install+1;cov_park=cov_park+1;if(p3_active)begin cov_no_xwb=cov_no_xwb+1;p3_active=0;end end
    if(s_valid[0]&&s_addr==L2[AW-1:0])cov_peer=cov_peer+1;
  end
  initial begin
    c_addr[0]=0;c_addr[1]=0;c_wdata[0]=0;c_wdata[1]=0;for(k=0;k<(1<<(AW-2));k=k+1)l2.mem[k]=32'h90000000+k;
    repeat(3)@(posedge clk);#1;reset=0;
    for(d=0;d<=12;d=d+1)begin
      L0=32'h2000+d*32'h200;L1=L0+16;L2=L0+32;X=L2+64;V=32'h5a5a0000+d;
      acc(0,1,X+4,32'h4a4a0000+d,r);
      acc(0,0,L0,0,r);acc(0,0,L1,0,r);
      k=0;while(!(c[0].l1d.busy&&c[0].l1d.cur_pf)&&k<100)begin @(negedge clk);k=k+1;end
      ck(k<100,"prefetch reaches in-flight state");
      // "d cycles later" is measured from completion of the blocking store;
      // before that point an unsynchronised peer load may legally precede it.
      p3_active=1;acc(0,1,L2+4,V,rl);repeat(d)@(posedge clk);acc(1,0,L2+4,0,rp);
      ck(rp===V,"peer load sees completed store");
      ck(c[0].l1d.n_pflate>0,"late-prefetch counter nonzero");
      acc(0,0,L2+4,0,r);if(r!==V)$display("TRACE P3 d=%0d final-local=%h exp=%h st0=%0d st1=%0d",d,r,V,c[0].l1d.st[L2[5:4]],c[1].l1d.st[L2[5:4]]);ck(r===V,"local final L2 value");acc(1,0,L2+4,0,r);ck(r===V,"peer final L2 value");
      acc(1,0,X+4,0,r);ck(r===32'h4a4a0000+d,"dirty victim value preserved");
      repeat(30)@(posedge clk);
    end
    $display("R31 P3 coverage flight=%0d install=%0d park=%0d no-X-WB=%0d peer-snoop=%0d pflate=%0d pfhit=%0d wbb=%0d upgr=%0d c2c=%0d",
      cov_flight,cov_install,cov_park,cov_no_xwb,cov_peer,c[0].l1d.n_pflate,c[0].l1d.n_pfhit,c[0].l1d.n_wbb,c[0].l1d.n_upgr,bus.n_c2c);
    if(cov_flight==0||cov_install==0||cov_park==0||cov_no_xwb==0||cov_peer==0)begin $display("FAIL vacuous P3");errors=errors+1;end
    if(errors==0)$display("RESULT: CODEX R31 P3 PASSED");else $display("RESULT: CODEX R31 P3 FAILED (%0d)",errors);$finish;
  end
  initial begin #2000000;$display("RESULT: CODEX R31 P3 FAILED TIMEOUT");$finish;end
endmodule
