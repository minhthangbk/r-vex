// Directed DPF state-transition edges omitted by tb_l1d_pf.
// This is a cache-local protocol bench: bus grants/completions and snoops are
// driven explicitly so the relevant cycles are deterministic.
`timescale 1ns/1ps
module tb_codex_r22_dpf_edges;
  localparam AW=16, OP_RD=2'd0, OP_RDX=2'd1, OP_UPGR=2'd2;
  reg clk=0, reset=1;
  always #5 clk=~clk;
  reg c_req=0, c_we=0, b_gnt=0, b_done=0, s_valid=0;
  reg [31:0] c_addr=0;
  reg [AW-1:0] s_addr=0;
  reg [31:0] c_wdata=0;
  reg [1:0] s_op=OP_RD;
  reg [127:0] b_rline=0;
  wire [31:0] c_rdata;
  wire c_ready, b_req, b_pf, s_hit_m;
  wire [1:0] b_op;
  wire [AW-1:0] b_addr;
  wire [127:0] b_wline, s_line;
  reg [1:0] am_req=0, am_pf=0;
  reg [3:0] am_op=0;
  reg [2*AW-1:0] am_addr=0;
  wire [1:0] am_gnt, am_done, as_valid;
  wire [1:0] as_op; wire [AW-1:0] as_addr;
  wire al2_req, al2_we; wire [AW-1:0] al2_addr; wire [127:0] al2_wline;
  integer errors=0;

  l1d_cache #(.AW(AW),.SETS(16),.DPF(4),.DPF_DEG(1),.DPF_HIST(4)) dut(
    .clk(clk),.reset(reset),.c_req(c_req),.c_we(c_we),.c_addr(c_addr),.c_wdata(c_wdata),
    .c_rdata(c_rdata),.c_ready(c_ready),.b_req(b_req),.b_pf(b_pf),.b_op(b_op),
    .b_addr(b_addr),.b_wline(b_wline),.b_gnt(b_gnt),.b_done(b_done),.b_rline(b_rline),
    .s_valid(s_valid),.s_op(s_op),.s_addr(s_addr),.s_hit_m(s_hit_m),.s_line(s_line));
  coh_bus #(.N(2),.AW(AW)) arb(
    .clk(clk),.reset(reset),.m_req(am_req),.m_pf(am_pf),.m_op(am_op),.m_addr(am_addr),.m_wline(256'd0),
    .m_gnt(am_gnt),.m_done(am_done),.m_rline(),.s_valid(as_valid),.s_op(as_op),.s_addr(as_addr),
    .s_hit_m(2'b00),.s_line(256'd0),.l2_req(al2_req),.l2_we(al2_we),.l2_addr(al2_addr),
    .l2_wline(al2_wline),.l2_done(1'b0),.l2_rline(128'd0));

  task ck; input cond; input [8*80-1:0] msg; begin
    if (!cond) begin $display("FAIL %0s",msg); errors=errors+1; end
    else $display("PASS %0s",msg);
  end endtask
  task tick; begin @(posedge clk); #1; end endtask

  initial begin
    repeat(2) tick; reset=0; tick;

    // Exact DPF-09 edge: a prefetch RD and another master's demand RDX of
    // the same line are both presented before either is granted.
    am_req=2'b11; am_pf=2'b01; am_op={OP_RDX,OP_RD};
    am_addr={16'h1230,16'h1230}; #1;
    ck(am_gnt==2'b10,"same-line demand RDX wins over prefetch RD");
    tick; am_req=0; am_pf=0;

    // A buffer hit X re-arms X+1 on the same edge that another master is
    // granted RDX X+1. The new pending entry is safe to retain: the atomic bus
    // cannot grant its RD until the RDX owner has filled and performed its store.
    dut.pv[0]=1; dut.pp[0]=0; dut.pa[0]=12'h100;
    dut.pd[0]=128'h44444444_33333333_22222222_11111111;
    c_req=1; c_addr=16'h1000;
    s_valid=1; s_op=OP_RDX; s_addr=16'h1010;
    #1; ck(dut.inst,"buffer hit installs while snoop targets new line");
    tick; s_valid=0;
    ck(dut.pp[0] && dut.pa[0]==12'h101,"same-cycle rearm records new line");
    ck(c_ready && c_rdata==32'h11111111,"installed line returns buffer data");
    c_req=0; tick;

    // A demand for that pending line must replace the ungranted prefetch and
    // clear the pending entry when its demand RD is granted.
    c_req=1; c_addr=16'h1010; #1;
    ck(b_req && !b_pf && b_op==OP_RD,"demand supersedes pending prefetch");
    b_gnt=1; tick; b_gnt=0;
    ck(!dut.pp[0],"demand grant drops same-line pending entry");
    b_rline=128'ha4a4a4a4_a3a3a3a3_a2a2a2a2_a1a1a1a1; b_done=1;
    tick; b_done=0; tick;
    ck(c_ready && c_rdata==32'ha1a1a1a1,"demand fill installs correct line");
    c_req=0; tick;

    // A store arriving after its prefetch was granted waits for completion,
    // consumes the buffer in S, upgrades, and only then becomes ready.
    dut.pp[1]=1; dut.pv[1]=0; dut.pa[1]=12'h110; tick;
    ck(b_req && b_pf && b_addr==16'h1100,"pending entry presents prefetch RD");
    b_gnt=1; tick; b_gnt=0;
    c_req=1; c_we=1; c_addr=16'h1100; c_wdata=32'hfeed2211;
    ck(!c_ready,"store waits for in-flight prefetch");
    b_rline=128'hdddddddd_cccccccc_bbbbbbbb_aaaaaaaa; b_done=1;
    tick; b_done=0; tick;
    ck(!c_ready && dut.st[0]==1,"store consumes prefetched line as shared");
    ck(b_req && !b_pf && b_op==OP_UPGR,"store requests upgrade after install");
    b_gnt=1; tick; b_gnt=0; b_done=1; tick; b_done=0; tick;
    ck(c_ready,"store ready only after upgrade");
    tick; c_req=0; c_we=0;
    ck(dut.dat[0]==32'hfeed2211,"store data written after upgrade");

    // FIFO replacement of a valid entry is a lost prefetch, never wrong data:
    // allocation clears pv before changing the address and marks the slot pending.
    dut.busy=0; dut.rp=2; dut.pv[2]=1; dut.pp[2]=0; dut.pa[2]=12'h200;
    dut.hv[0]=1; dut.hist[0]=12'h120;
    c_req=1; c_we=0; c_addr=16'h1210; #1; b_gnt=1; tick; b_gnt=0;
    ck(!dut.pv[2] && dut.pp[2] && dut.pa[2]==12'h122,
       "FIFO overwrite discards old valid data and records only new pending line");
    c_req=0; dut.busy=0; tick;

    // Address arithmetic wraps in LA bits. A stream ending at the top line
    // allocates line zero; the FE00 window is likewise only an ordinary dead
    // memory prefetch when allocation originates at the preceding cacheable line.
    dut.rp=3; dut.hv[0]=1; dut.hist[0]=12'hffe;
    c_req=1; c_addr=16'hfff0; #1; b_gnt=1; tick; b_gnt=0;
    ck(dut.pp[3] && dut.pa[3]==12'h000,"top data line prefetch wraps to line zero");
    c_req=0; dut.busy=0; tick;
    dut.rp=3; dut.hv[0]=1; dut.hist[0]=12'hfde;
    c_req=1; c_addr=16'hfdf0; #1; b_gnt=1; tick; b_gnt=0;
    ck(dut.pp[3] && dut.pa[3]==12'hfe0,"preceding stream may allocate dead FE00 prefetch");

    if (errors==0) $display("RESULT: CODEX R22 DPF EDGES PASSED");
    else $display("RESULT: CODEX R22 DPF EDGES FAILED (%0d)",errors);
    $finish;
  end
endmodule
