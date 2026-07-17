//======================================================================
// r-VEX | Control-unit directed testbench
//----------------------------------------------------------------------
// ctrl_unit.v had NO dedicated testbench before. This exercises every
// branch/link opcode and its side effects:
//   GOTO/IGOTO           - unconditional, target from offset vs lr
//   CALL/ICALL           - unconditional + link = pc+1
//   BR/BRF               - conditional on br, taken and fall-through
//   RETURN/RFI           - target from lr, link = sp+offset, is_rfi flag
//   XNOP                 - is_xnop + xnop_n cycle count, not taken
//   NOP                  - fall-through, still valid
//   invalid opcode       - out_valid = 0
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_ctrl;
    integer pass = 0, fail = 0;

    reg  [6:0]  op;
    reg  [7:0]  pc;
    reg  [31:0] lr, sp;
    reg  [11:0] offset;
    reg         br;
    wire [7:0]  pc_goto;
    wire [31:0] link_val;
    wire        taken, writes_link, is_xnop, is_rfi, out_valid;
    wire [11:0] xnop_n;

    ctrl_unit u_ctrl(.opcode(op),.pc(pc),.lr(lr),.sp(sp),.offset(offset),.br(br),
                     .pc_goto(pc_goto),.link_val(link_val),.taken(taken),
                     .writes_link(writes_link),.is_xnop(is_xnop),.xnop_n(xnop_n),
                     .is_rfi(is_rfi),.out_valid(out_valid));

    task chk8;  input [127:0] nm; input [7:0]  g,e; begin
        if (g===e) begin pass=pass+1; $display("  PASS %0s : 0x%02h",nm,g); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%02h exp 0x%02h",nm,g,e); end
    end endtask
    task chk32; input [127:0] nm; input [31:0] g,e; begin
        if (g===e) begin pass=pass+1; $display("  PASS %0s : 0x%08h",nm,g); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h",nm,g,e); end
    end endtask
    task chk12; input [127:0] nm; input [11:0] g,e; begin
        if (g===e) begin pass=pass+1; $display("  PASS %0s : %0d",nm,g); end
        else begin fail=fail+1; $display("  FAIL %0s : got %0d exp %0d",nm,g,e); end
    end endtask
    task chkb;  input [127:0] nm; input g,e; begin
        if (g===e) begin pass=pass+1; $display("  PASS %0s : %b",nm,g); end
        else begin fail=fail+1; $display("  FAIL %0s : got %b exp %b",nm,g,e); end
    end endtask

    task drive; input [6:0] o; input [7:0] p; input [31:0] l,s; input [11:0] off; input b; begin
        op=o; pc=p; lr=l; sp=s; offset=off; br=b; #1;
    end endtask

    initial begin
        $display("================ r-VEX CTRL unit tests ================");

        //---------------------------------------- unconditional jumps
        $display("[GOTO/IGOTO]");
        drive(`CTRL_GOTO ,8'h05,32'd0,32'd0,12'h020,0);
            chkb("GOTO taken",taken,1'b1); chk8("GOTO target",pc_goto,8'h20);
        drive(`CTRL_IGOTO,8'h05,32'h00000030,32'd0,12'd0,0);
            chkb("IGOTO taken",taken,1'b1); chk8("IGOTO target=lr",pc_goto,8'h30);

        //---------------------------------------- calls (link = pc+1)
        $display("[CALL/ICALL]");
        drive(`CTRL_CALL ,8'h05,32'd0,32'd0,12'h040,0);
            chkb("CALL taken",taken,1'b1); chk8("CALL target",pc_goto,8'h40);
            chkb("CALL writes_link",writes_link,1'b1); chk32("CALL link=pc+1",link_val,32'd6);
        drive(`CTRL_ICALL,8'h05,32'h00000050,32'd0,12'd0,0);
            chk8("ICALL target=lr",pc_goto,8'h50); chk32("ICALL link=pc+1",link_val,32'd6);

        //---------------------------------------- conditional branches
        $display("[BR/BRF]");
        drive(`CTRL_BR ,8'h07,32'd0,32'd0,12'h060,1);
            chkb("BR br=1 taken",taken,1'b1); chk8("BR target",pc_goto,8'h60);
        drive(`CTRL_BR ,8'h07,32'd0,32'd0,12'h060,0);
            chkb("BR br=0 fall",taken,1'b0);  chk8("BR fall=pc+1",pc_goto,8'h08);
        drive(`CTRL_BRF,8'h07,32'd0,32'd0,12'h070,0);
            chkb("BRF br=0 taken",taken,1'b1); chk8("BRF target",pc_goto,8'h70);
        drive(`CTRL_BRF,8'h07,32'd0,32'd0,12'h070,1);
            chkb("BRF br=1 fall",taken,1'b0);  chk8("BRF fall=pc+1",pc_goto,8'h08);

        //---------------------------------------- return / rfi
        $display("[RETURN/RFI]");
        drive(`CTRL_RETURN,8'h05,32'h00000022,32'h00000100,12'd4,0);
            chkb("RET taken",taken,1'b1); chk8("RET target=lr",pc_goto,8'h22);
            chk32("RET link=sp+off",link_val,32'h00000104); chkb("RET not rfi",is_rfi,1'b0);
        drive(`CTRL_RFI,8'h05,32'h00000024,32'h00000100,12'd4,0);
            chk8("RFI target=lr",pc_goto,8'h24); chk32("RFI link=sp+off",link_val,32'h00000104);
            chkb("RFI is_rfi",is_rfi,1'b1);

        //---------------------------------------- xnop
        $display("[XNOP]");
        drive(`CTRL_XNOP,8'h05,32'd0,32'd0,12'd5,0);
            chkb("XNOP is_xnop",is_xnop,1'b1); chk12("XNOP n",xnop_n,12'd5);
            chkb("XNOP not taken",taken,1'b0);

        //---------------------------------------- nop / invalid
        $display("[NOP/invalid]");
        drive(`OP_NOP,8'h09,32'd0,32'd0,12'd0,0);
            chkb("NOP not taken",taken,1'b0); chk8("NOP pc+1",pc_goto,8'h0A);
            chkb("NOP valid",out_valid,1'b1);
        drive(`OP_STOP,8'h09,32'd0,32'd0,12'd0,0);
            chkb("bad op invalid",out_valid,1'b0);

        $display("======================================================");
        $display("CTRL TESTS: %0d passed, %0d failed", pass, fail);
        if (fail==0) $display("RESULT: ALL CTRL TESTS PASSED");
        else         $display("RESULT: %0d CTRL TEST FAILURES", fail);
        $finish;
    end
endmodule
