// Round 31 W3b: second dirty victim while WBB full and peer holds the bus.
`timescale 1ns/1ps
module tb_codex_r31_w3b;
 localparam AW=16;reg clk=0,reset=1;always #5 clk=~clk;
 reg[1:0]c_req=0,c_we=0;reg[31:0]c_addr[0:1],c_wdata[0:1];wire[31:0]c_rdata[0:1];wire[1:0]c_ready;
 wire[1:0]b_req,b_pf,b_gnt,b_done,s_valid,s_hit_m;wire[3:0]b_op;wire[2*AW-1:0]b_addr;wire[255:0]b_wline,s_line;
 wire[127:0]b_rline;wire[1:0]s_op;wire[AW-1:0]s_addr;wire l2_req,l2_we,l2_done;wire[AW-1:0]l2_addr;wire[127:0]l2_wline,l2_rline;
 integer errors=0,d,kind,k,s,cov_wbb_dem=0,cov_s_wbb=0,cov_s_victim=0,cov_busy=0,cov_owner=0;reg[31:0]A,B,C,H,r,va,vb,vc;
 genvar g;generate for(g=0;g<2;g=g+1)begin:c l1d_cache #(.AW(AW),.SETS(4),.FWD(1),.WBB(1),.DPF(4),.DPF_DEG(1),.DPF_HIST(4))l1d(
 .clk(clk),.reset(reset),.c_req(c_req[g]),.c_we(c_we[g]),.c_addr(c_addr[g]),.c_wdata(c_wdata[g]),.c_rdata(c_rdata[g]),.c_ready(c_ready[g]),
 .b_req(b_req[g]),.b_pf(b_pf[g]),.b_op(b_op[2*g+:2]),.b_addr(b_addr[AW*g+:AW]),.b_wline(b_wline[128*g+:128]),.b_gnt(b_gnt[g]),.b_done(b_done[g]),.b_rline(b_rline),
 .s_valid(s_valid[g]),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m[g]),.s_line(s_line[128*g+:128]));end endgenerate
 coh_bus #(.N(2),.AW(AW),.FAST(1))bus(.clk(clk),.reset(reset),.m_req(b_req),.m_pf(b_pf),.m_op(b_op),.m_addr(b_addr),.m_wline(b_wline),.m_gnt(b_gnt),.m_done(b_done),.m_rline(b_rline),.s_valid(s_valid),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m),.s_line(s_line),.l2_req(l2_req),.l2_we(l2_we),.l2_addr(l2_addr),.l2_wline(l2_wline),.l2_done(l2_done),.l2_rline(l2_rline));
 l2_cache #(.AW(AW),.SETS(8),.L2_LAT(2),.MEM_LAT(5),.FAST(1),.WBUF(1))l2(.clk(clk),.reset(reset),.req(l2_req),.we(l2_we),.addr(l2_addr),.wline(l2_wline),.done(l2_done),.rline(l2_rline));
 task ck;input ok;input[8*64-1:0]m;begin if(!ok)begin $display("FAIL kind=%0d d=%0d %0s",kind,d,m);errors=errors+1;end end endtask
 task automatic acc;input integer p;input we;input[31:0]a,v;output[31:0]q;begin @(negedge clk);c_req[p]=1;c_we[p]=we;c_addr[p]=a;c_wdata[p]=v;#1;while(!c_ready[p])begin @(negedge clk);#1;end q=c_rdata[p];@(posedge clk);#1;c_req[p]=0;end endtask
 task check_copy;input[31:0]a,v;integer i,w;reg[AW-5:0]la;begin
  la=a[AW-1:4];i=la[1:0];w=a[3:2];
  if(c[0].l1d.st[i]!=0&&c[0].l1d.tg[i]==la[AW-5:2])ck(c[0].l1d.dat[i*4+w]===v,"core0 array value");
  if(c[1].l1d.st[i]!=0&&c[1].l1d.tg[i]==la[AW-5:2])ck(c[1].l1d.dat[i*4+w]===v,"core1 array value");
  if(c[0].l1d.wv&&c[0].l1d.wa==la)ck((c[0].l1d.wd>>(32*w))===v,"core0 WBB value");
  if(c[1].l1d.wv&&c[1].l1d.wa==la)ck((c[1].l1d.wd>>(32*w))===v,"core1 WBB value");
  ck(l2.peek(a)===v,"L2/memory coherent value");
 end endtask
 always @(negedge clk)if(!reset)begin
  for(s=0;s<4;s=s+1)begin
   cov_owner=cov_owner+1;
   if(c[0].l1d.st[s]!=0&&c[1].l1d.st[s]!=0&&c[0].l1d.tg[s]==c[1].l1d.tg[s]&&(c[0].l1d.st[s]==2||c[1].l1d.st[s]==2))begin $display("FAIL OWNER arrays set=%0d",s);errors=errors+1;end
  end
  if(c[0].l1d.wv)begin
   if(c[0].l1d.st[c[0].l1d.wa[1:0]]!=0&&c[0].l1d.tg[c[0].l1d.wa[1:0]]==c[0].l1d.wa[AW-5:2])begin $display("FAIL OWNER core0 WBB/array");errors=errors+1;end
   if(c[1].l1d.st[c[0].l1d.wa[1:0]]!=0&&c[1].l1d.tg[c[0].l1d.wa[1:0]]==c[0].l1d.wa[AW-5:2])begin $display("FAIL OWNER core0 WBB/peer");errors=errors+1;end
  end
  if(c[1].l1d.wv)begin
   if(c[0].l1d.st[c[1].l1d.wa[1:0]]!=0&&c[0].l1d.tg[c[1].l1d.wa[1:0]]==c[1].l1d.wa[AW-5:2])begin $display("FAIL OWNER core1 WBB/peer");errors=errors+1;end
   if(c[1].l1d.st[c[1].l1d.wa[1:0]]!=0&&c[1].l1d.tg[c[1].l1d.wa[1:0]]==c[1].l1d.wa[AW-5:2])begin $display("FAIL OWNER core1 WBB/array");errors=errors+1;end
  end
  if(c[0].l1d.wv&&c[1].l1d.wv&&c[0].l1d.wa==c[1].l1d.wa)begin $display("FAIL OWNER duplicate WBB");errors=errors+1;end
  if(c[0].l1d.wv&&c[0].l1d.wdr_dem)cov_wbb_dem=cov_wbb_dem+1;
  if(c[0].l1d.s_wbb&&c[0].l1d.wdr_dem)cov_s_wbb=cov_s_wbb+1;
  if(c[0].l1d.s_match&&c[0].l1d.s_idx==c[0].l1d.c_idx&&c[0].l1d.wv)cov_s_victim=cov_s_victim+1;
  if(bus.busy&&c[0].l1d.wdr_dem)cov_busy=cov_busy+1;
 end
 initial begin c_addr[0]=0;c_addr[1]=0;c_wdata[0]=0;c_wdata[1]=0;for(k=0;k<(1<<(AW-2));k=k+1)l2.mem[k]=32'h90000000+k;
  repeat(3)@(posedge clk);#1;reset=0;
  for(kind=0;kind<3;kind=kind+1)for(d=0;d<=20;d=d+1)begin
   A=32'h1000+kind*32'h3000+d*32'h100;B=A+32'h40;C=A+32'h80;H=A+32'h30;
   va=32'ha0000000+kind*256+d;vb=32'hb0000000+kind*256+d;vc=32'hc0000000+kind*256+d;
   acc(0,1,A,va,r);acc(0,1,B,vb,r);ck(c[0].l1d.wv,"A parked before W3b");
   fork
    begin
     // Keep the peer port asserted across the H completion when d=0, so its
     // second demand is present in the FAST gap and can beat core 0's WB(A).
     @(negedge clk);c_req[1]=1;c_we[1]=0;c_addr[1]=H;c_wdata[1]=0;
     wait(b_done[1]);
     if(d!=0)begin @(posedge clk);#1;c_req[1]=0;repeat(d)@(posedge clk);@(negedge clk);c_req[1]=1;end
     if(kind==0)begin c_we[1]=0;c_addr[1]=A;end
     else if(kind==1)begin c_we[1]=1;c_addr[1]=B;c_wdata[1]=32'hbd000000+d;end
     else begin c_we[1]=0;c_addr[1]=C;end
     #1;while(!c_ready[1])begin @(negedge clk);#1;end
     @(posedge clk);#1;c_req[1]=0;
    end
    begin wait(b_gnt[1]); acc(0,1,C,vc,r);end
   join
   if(kind==1)vb=32'hbd000000+d;
   acc(1,0,A,0,r);ck(r===va,"final A");acc(1,0,B,0,r);ck(r===vb,"final B");acc(1,0,C,0,r);ck(r===vc,"final C");
   repeat(20)@(posedge clk);check_copy(A,va);check_copy(B,vb);check_copy(C,vc);
  end
  // Arbitration-alignment phase for the exact (s_wbb && wdr_dem) predicate.
  // H first holds the bus for core 1.  Core 0's clean miss D is already
  // pending, so it wins next without draining A and makes last=core 0.  At
  // D completion switch core 0 to the second dirty victim C and present the
  // peer RD(A); round-robin then grants the peer while wdr_dem is asserted.
  kind=3;d=0;A=32'he000;B=A+32'h40;C=A+32'h80;H=A+32'h30;
  va=32'ha3000000;vb=32'hb3000000;vc=32'hc3000000;
  acc(0,1,A,va,r);acc(0,1,B,vb,r);
  @(negedge clk);c_req[1]=1;c_we[1]=0;c_addr[1]=H;c_req[0]=1;c_we[0]=0;c_addr[0]=A+16;
  wait(b_done[0]); // this is D (core 0), after H completed
  c_we[0]=1;c_addr[0]=C;c_wdata[0]=vc;c_we[1]=0;c_addr[1]=A;
  #1;while(!c_ready[1])@(negedge clk);@(posedge clk);#1;c_req[1]=0;
  while(!c_ready[0])@(negedge clk);@(posedge clk);#1;c_req[0]=0;
  acc(1,0,A,0,r);ck(r===va,"aligned final A");acc(1,0,B,0,r);ck(r===vb,"aligned final B");acc(1,0,C,0,r);ck(r===vc,"aligned final C");
  repeat(20)@(posedge clk);check_copy(A,va);check_copy(B,vb);check_copy(C,vc);
  $display("R31 W3b coverage owner=%0d wdr_dem=%0d bus-held=%0d s_wbb+wdr=%0d snoop-victim=%0d wb=%0d wbbsnp=%0d",cov_owner,cov_wbb_dem,cov_busy,cov_s_wbb,cov_s_victim,c[0].l1d.n_wb,c[0].l1d.n_wbbsnp);
  ck(c[0].l1d.n_wb==88,"expected core0 write-back grant count");
  ck(c[0].l1d.n_wbbsnp==40,"expected core0 WBB snoop count");
  if(cov_owner==0||cov_wbb_dem==0||cov_busy==0||cov_s_wbb==0||cov_s_victim==0)begin $display("FAIL vacuous W3b");errors=errors+1;end
  if(errors==0)$display("RESULT: CODEX R31 W3B PASSED");else $display("RESULT: CODEX R31 W3B FAILED (%0d)",errors);$finish;
 end
 initial begin #4000000;$display("RESULT: CODEX R31 W3B FAILED TIMEOUT");$finish;end
endmodule
