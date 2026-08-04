//======================================================================
// r-VEX SoC phase 1 | ahb_lite_sram_ctrl unit test (round 25)
//----------------------------------------------------------------------
// Directly checks the 2-cycle transfer timing the module header
// documents (address phase cycle N: HREADYOUT=0; data phase cycle N+1:
// HREADYOUT=1, HRDATA valid) -- this exact off-by-one was caught by hand
// before it ever reached rvex_core_bus.v (an earlier draft registered
// HRDATA/HREADYOUT instead of driving them combinationally from `state`,
// which silently added a 3rd cycle). A write-then-read round trip plus
// an explicit wait-state-count check.
//======================================================================
`timescale 1ns/1ps

module tb_ahb_lite_sram_ctrl;
    reg clk=0, reset=1;
    reg  [7:0]  haddr; reg hwrite; reg [1:0] htrans; reg [31:0] hwdata;
    wire [31:0] hrdata; wire hreadyout, hresp;
    integer pass=0, fail=0;

    ahb_lite_sram_ctrl #(.DATA_WIDTH(32), .DEPTH(256), .ADDRW(8)) dut (
        .clk(clk), .reset(reset),
        .haddr(haddr), .hwrite(hwrite), .htrans(htrans), .hwdata(hwdata),
        .hrdata(hrdata), .hreadyout(hreadyout), .hresp(hresp)
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
        haddr=0; hwrite=0; htrans=2'b00; hwdata=0;
        reset=1; repeat(2) @(posedge clk); reset=0; @(posedge clk);

        // ---- WRITE 0xCAFEBABE to word 5 ----
        @(negedge clk);
        haddr=8'd5; hwrite=1'b1; htrans=2'b10; hwdata=32'hCAFEBABE;
        #1; // let delta-cycle combinational (assign) propagation settle before checking
        chk("write addr-phase: HREADYOUT low this cycle", hreadyout, 1'b0);
        @(posedge clk); @(negedge clk);           // now in data phase
        chk("write data-phase: HREADYOUT high", hreadyout, 1'b1);
        @(posedge clk); @(negedge clk);
        htrans = 2'b00;                            // end transfer

        // ---- READ word 5 back ----
        haddr=8'd5; hwrite=1'b0; htrans=2'b10;
        #1;
        chk("read addr-phase: HREADYOUT low this cycle", hreadyout, 1'b0);
        @(posedge clk); @(negedge clk);
        chk("read data-phase: HREADYOUT high", hreadyout, 1'b1);
        chkd("read data-phase: HRDATA == 0xCAFEBABE", hrdata, 32'hCAFEBABE);
        @(posedge clk); @(negedge clk);
        htrans = 2'b00;

        $display("=====================================================");
        $display("AHB_LITE_SRAM_CTRL UNIT TEST: %0d passed, %0d failed", pass, fail);
        if (fail==0) $display("RESULT: AHB UNIT TEST PASSED"); else $display("RESULT: AHB UNIT TEST FAILED");
        $finish;
    end
endmodule
