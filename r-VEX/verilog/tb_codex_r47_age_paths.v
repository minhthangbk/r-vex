// Round 47: directed DPF-14 transition/order rows for the l1d_cache age matrix.
`timescale 1ns/1ps
module tb_codex_r47_age_paths;
  localparam AW=16, OP_RD=2'd0, OP_RDX=2'd1;
  reg clk=0, reset=1, c_req=0, c_we=0, b_gnt=0, b_done=0, s_valid=0;
  reg [31:0] c_addr=0, c_wdata=0; reg [127:0] b_rline=0;
  reg [1:0] s_op=0; reg [AW-1:0] s_addr=0;
  wire [31:0] c_rdata; wire c_ready,b_req,b_pf; wire [1:0] b_op;
  wire [AW-1:0] b_addr; wire [127:0] b_wline,s_line; wire s_hit_m;
  always #5 clk=~clk;
  l1d_cache #(.AW(AW),.SETS(16),.DPF(4),.DPF_DEG(4),.DPF_HIST(4),
               .DPF_STR(0),.DPF_AGE(1),.WBB(1)) dut(
    .clk(clk),.reset(reset),.c_req(c_req),.c_we(c_we),.c_addr(c_addr),.c_wdata(c_wdata),
    .c_rdata(c_rdata),.c_ready(c_ready),.b_req(b_req),.b_pf(b_pf),.b_op(b_op),
    .b_addr(b_addr),.b_wline(b_wline),.b_gnt(b_gnt),.b_done(b_done),.b_rline(b_rline),
    .s_valid(s_valid),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m),.s_line(s_line));
  integer errors=0,i,j;
  task tick; begin @(posedge clk); #1; end endtask
  task clear_entries; begin
    for(i=0;i<4;i=i+1) begin dut.pp[i]=0; dut.pv[i]=0; dut.pa[i]=0; dut.ob[i]=0; end
    dut.busy=0; dut.wv=0; dut.rp=0; c_req=0; b_gnt=0; s_valid=0;
  end endtask
  task mark; input integer e; begin
    for(j=0;j<4;j=j+1) begin dut.ob[e][j]=0; dut.ob[j][e]=1; end
  end endtask
  task want; input [8*64-1:0] n; input integer got,exp; begin
    #1; if(got!==exp) begin $display("FAIL %0s got %0d expected %0d",n,got,exp); errors=errors+1; end
    else $display("ok   %0s",n); end endtask
  initial begin
    repeat(3) tick; reset=0; tick;

    // Re-arm to an absent target: the re-armed entry is younger than an existing pending entry.
    clear_entries; mark(2); dut.pp[2]=1; dut.pa[2]=12'h020;
    dut.pv[0]=1; dut.pa[0]=12'h030; c_addr=16'h0300; force dut.inst=1; force dut.bhi=0;
    tick; release dut.inst; release dut.bhi; want("rearm have=false keeps older entry",dut.pqi,2);

    // Re-arm to a target already represented by another entry: it must not become pending.
    clear_entries; dut.pv[0]=1; dut.pa[0]=12'h040; dut.pp[1]=1; dut.pa[1]=12'h044;
    c_addr=16'h0400; force dut.inst=1; force dut.bhi=0; tick; release dut.inst; release dut.bhi;
    want("rearm have=true leaves entry clear",dut.pp[0],0); want("existing target remains oldest",dut.pqi,1);

    // DEG=4 allocation makes rp, rp+1, rp+2, rp+3 pending in that FIFO order.
    clear_entries; dut.rp=1; dut.hv[0]=1; dut.hist[0]=12'h04f; c_addr=16'h0500; c_req=1; b_gnt=1;
    tick; c_req=0; b_gnt=0; want("DEG allocation oldest is rp",dut.pqi,1);
    dut.pp[1]=0; #1; want("DEG allocation second",dut.pqi,2);
    dut.pp[2]=0; #1; want("DEG allocation third",dut.pqi,3);
    dut.pp[3]=0; #1; want("DEG allocation fourth",dut.pqi,0);

    // Allocation over a pending entry refreshes its age; an untouched older entry stays first.
    clear_entries; mark(3); mark(0); dut.pp[3]=1; dut.pa[3]=12'h070; dut.pp[0]=1; dut.pa[0]=12'h071;
    // Only target +1 is absent; +2..+4 are already represented, so allocation overwrites entry 0 only.
    dut.pv[1]=1; dut.pa[1]=12'h062; dut.pv[2]=1; dut.pa[2]=12'h063;
    dut.st[4]=2'd1; dut.tg[4]=8'h06;
    dut.rp=0; dut.hv[0]=1; dut.hist[0]=12'h05f; c_addr=16'h0600; c_req=1; b_gnt=1;
    tick; c_req=0; b_gnt=0; dut.st[4]=0; want("allocation over pending does not overtake older",dut.pqi,3);

    // Kill, cached-target drop, and WBB-target drop all remove pending state.
    clear_entries; dut.pp[1]=1; dut.pa[1]=12'h080; s_valid=1; s_op=OP_RDX; s_addr=16'h0800;
    tick; s_valid=0; want("kill clears pending",dut.pp[1],0);
    clear_entries; dut.pp[2]=1; dut.pa[2]=12'h090; dut.st[0]=2'd1; dut.tg[0]=8'h09;
    tick; want("p_cached clears pending",dut.pp[2],0); dut.st[0]=0;
    clear_entries; dut.pp[3]=1; dut.pa[3]=12'h0a0; dut.wv=1; dut.wa=12'h0a0;
    tick; want("p_wbb clears pending",dut.pp[3],0);

    // If re-arm and demand allocation coincide, the later allocation is younger; pre-existing pending stays oldest.
    clear_entries; mark(3); dut.pp[3]=1; dut.pa[3]=12'h0b0; dut.pv[0]=1; dut.pa[0]=12'h0c0;
    // Leave +1 absent for allocation; +2/+3 already buffered and +4 is the re-arm target represented here.
    dut.pv[1]=1; dut.pa[1]=12'h0c2; dut.pv[2]=1; dut.pa[2]=12'h0c3;
    dut.st[4]=2'd1; dut.tg[4]=8'h0c;
    dut.rp=1; dut.hv[0]=1; dut.hist[0]=12'h0bf; c_addr=16'h0c00; c_req=1; b_gnt=1;
    force dut.inst=1; force dut.bhi=0; tick; release dut.inst; release dut.bhi; c_req=0; b_gnt=0;
    dut.st[4]=0; want("simultaneous rearm/allocation preserves prior oldest",dut.pqi,3);

    if(errors==0) $display("RESULT: CODEX R47 AGE PATHS PASSED");
    else $display("RESULT: CODEX R47 AGE PATHS FAILED errors=%0d",errors);
    $finish;
  end
endmodule
