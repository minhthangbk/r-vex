//======================================================================
// r-VEX SoC round 26 | axi4_sram_ctrl unit test
//----------------------------------------------------------------------
// Mirrors tb_ahb_lite_sram_ctrl.v's write-then-read-back structure
// (same timing family, same off-by-one risk class) plus an explicit
// check that AR and AW+W do not interfere when exercised back-to-back.
//======================================================================
`timescale 1ns/1ps

module tb_axi4_sram_ctrl;
    reg clk=0, reset=1;
    reg  [7:0]  awaddr, araddr; reg awvalid, wvalid, arvalid, bready, rready;
    reg  [31:0] wdata;
    wire awready, wready, bvalid, arready, rvalid;
    wire [1:0]  bresp, rresp;
    wire        rlast;
    wire [31:0] rdata;
    integer pass=0, fail=0;

    axi4_sram_ctrl #(.DATA_WIDTH(32), .DEPTH(256), .ADDRW(8)) dut (
        .clk(clk), .reset(reset),
        .awaddr(awaddr), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wvalid(wvalid), .wready(wready),
        .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .araddr(araddr), .arvalid(arvalid), .arready(arready),
        .rdata(rdata), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready)
    );
    always #5 clk = ~clk;

    task chk; input [127:0] nm; input got, exp; begin
        if (got===exp) begin pass=pass+1; $display("  PASS %0s : %b", nm, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got %b exp %b", nm, got, exp); end
    end endtask
    task chkd; input [127:0] nm; input [31:0] got, exp; begin
        if (got===exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h", nm, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", nm, got, exp); end
    end endtask

    initial begin
        awaddr=0; araddr=0; awvalid=0; wvalid=0; arvalid=0; wdata=0; bready=1; rready=1;
        reset=1; repeat(2) @(posedge clk); reset=0; @(posedge clk);

        // ---- WRITE 0xDEADBEEF to word 9 (AW+W concurrent, single beat) ----
        @(negedge clk);
        awaddr=8'd9; awvalid=1; wvalid=1; wdata=32'hDEADBEEF;
        #1;
        chk("write addr-phase: BVALID low this cycle", bvalid, 1'b0);
        @(posedge clk); @(negedge clk);
        awvalid=0; wvalid=0;
        chk("write resp-phase: BVALID high", bvalid, 1'b1);
        @(posedge clk); @(negedge clk);

        // ---- READ word 9 back ----
        araddr=8'd9; arvalid=1;
        #1;
        chk("read addr-phase: RVALID low this cycle", rvalid, 1'b0);
        @(posedge clk); @(negedge clk);
        arvalid=0;
        chk("read data-phase: RVALID high", rvalid, 1'b1);
        chk("read data-phase: RLAST high (single beat)", rlast, 1'b1);
        chkd("read data-phase: RDATA == 0xDEADBEEF", rdata, 32'hDEADBEEF);
        @(posedge clk); @(negedge clk);

        // ---- back-to-back: write word 10, immediately read word 9 again ----
        awaddr=8'd10; awvalid=1; wvalid=1; wdata=32'h11223344;
        @(posedge clk); @(negedge clk);
        awvalid=0; wvalid=0;
        @(posedge clk); @(negedge clk);   // consume BVALID
        araddr=8'd9; arvalid=1;
        @(posedge clk); @(negedge clk);
        arvalid=0;
        chkd("word 9 unaffected by word-10 write", rdata, 32'hDEADBEEF);
        @(posedge clk); @(negedge clk);
        araddr=8'd10; arvalid=1;
        @(posedge clk); @(negedge clk);
        arvalid=0;
        chkd("word 10 == 0x11223344", rdata, 32'h11223344);

        $display("=====================================================");
        $display("AXI4_SRAM_CTRL UNIT TEST: %0d passed, %0d failed", pass, fail);
        if (fail==0) $display("RESULT: AXI4 UNIT TEST PASSED"); else $display("RESULT: AXI4 UNIT TEST FAILED");
        $finish;
    end
endmodule
