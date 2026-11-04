// IPF audit edges absent from tb_l1i_pf: short refill latencies, WARM+IPF,
// packet-address wrap, and cache-hit priority over a same-set buffer line.
`timescale 1ns/1ps
module tb_codex_r22_ipf_edges;
  localparam PW=12, SETS=16;
  reg clk=0, reset=1;
  always #5 clk=~clk;
  reg [PW-1:0] trace[0:7];
  integer p1=0,p2=0,p3=0,errors=0,k,watch;
  reg run1=0,run2=0,run3=0;
  wire [PW-1:0] a1=trace[p1],a2=trace[p2],a3=trace[p3];
  wire [127:0] d1,d2,d3,dw;
  wire r1,r2,r3,rw;
  reg wreq=0; reg [PW-1:0] wa=0;

  l1i_cache #(.PC_W(PW),.SETS(SETS),.FILL_LAT(1),.IPF(1)) c1
    (.clk(clk),.reset(reset),.i_req(run1),.i_addr(a1),.i_rdata(d1),.i_ready(r1));
  l1i_cache #(.PC_W(PW),.SETS(SETS),.FILL_LAT(2),.IPF(1)) c2
    (.clk(clk),.reset(reset),.i_req(run2),.i_addr(a2),.i_rdata(d2),.i_ready(r2));
  l1i_cache #(.PC_W(PW),.SETS(SETS),.FILL_LAT(3),.IPF(1)) c3
    (.clk(clk),.reset(reset),.i_req(run3),.i_addr(a3),.i_rdata(d3),.i_ready(r3));
  l1i_cache #(.PC_W(PW),.SETS(SETS),.FILL_LAT(2),.WARM(1),.IPF(1)) cw
    (.clk(clk),.reset(reset),.i_req(wreq),.i_addr(wa),.i_rdata(dw),.i_ready(rw));

  task ck; input ok; input [8*64-1:0] msg; begin
    if (!ok) begin $display("FAIL %0s",msg); errors=errors+1; end
    else $display("PASS %0s",msg);
  end endtask
  task tick; begin @(posedge clk); #1; end endtask
  task start_trace; begin
    p1=0;p2=0;p3=0;run1=1;run2=1;run3=1;watch=0;
    while ((run1||run2||run3) && watch<100) begin tick; watch=watch+1; end
    ck(!(run1||run2||run3),"FILL_LAT 1..3 trace completes");
  end endtask

  always @(posedge clk) if (!reset) begin
    if (run1 && r1) begin
      if (d1!==c1.rom[a1]) begin errors<=errors+1; $display("FAIL FL1 data"); end
      if (p1==7) run1<=0; else p1<=p1+1;
    end
    if (run2 && r2) begin
      if (d2!==c2.rom[a2]) begin errors<=errors+1; $display("FAIL FL2 data"); end
      if (p2==7) run2<=0; else p2<=p2+1;
    end
    if (run3 && r3) begin
      if (d3!==c3.rom[a3]) begin errors<=errors+1; $display("FAIL FL3 data"); end
      if (p3==7) run3<=0; else p3<=p3+1;
    end
  end

  initial begin
    for(k=0;k<(1<<PW);k=k+1) begin
      c1.rom[k]={4{32'h11000000+k}}; c2.rom[k]={4{32'h22000000+k}};
      c3.rom[k]={4{32'h33000000+k}}; cw.rom[k]={4{32'h44000000+k}};
    end
    repeat(2) tick; reset=0; tick;
    wreq=1;wa=0;#1;ck(rw && dw===cw.rom[0],"WARM+IPF returns warm line immediately");
    wreq=0;

    for(k=0;k<8;k=k+1) trace[k]=k;
    start_trace;

    reset=1;repeat(2) tick;reset=0;tick;
    for(k=0;k<4;k=k+1) begin trace[k]=12'hffc+k; trace[k+4]=k; end
    start_trace;
    ck(c1.n_pfhit>0 && c2.n_pfhit>0 && c3.n_pfhit>0,"PC line wrap prefetch is consumed");

    // A valid cache line wins over an unrelated buffer line mapping to its set.
    c1.vld[0]=1;c1.tg[0]=0;c1.dat[0]=128'h1;c1.pf_v=1;c1.pf_ln=10'h010;
    trace[0]=0;p1=0;run1=1;#1;
    ck(r1 && d1===128'h1,"cache hit has i_rdata priority over same-set buffer");
    run1=0;

    if(errors==0)$display("RESULT: CODEX R22 IPF EDGES PASSED");
    else $display("RESULT: CODEX R22 IPF EDGES FAILED (%0d)",errors);
    $finish;
  end
endmodule
