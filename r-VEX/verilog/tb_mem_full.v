//======================================================================
// r-VEX | Exhaustive memory-format unit directed testbench
//----------------------------------------------------------------------
// Covers every mem_unit.v opcode and every byte/half position that
// tb_units.v skipped:
//   - LDW full word
//   - LDH/LDHU at pos 00/01/10  (sign vs zero extend, all field selects)
//   - LDB/LDBU at pos 00/01/10/11
//   - STW, STH at pos 00/01/10, STB at pos 00/01/10/11 (field insertion)
//   - PFT valid no-op, and invalid opcode -> out_valid=0
// pos->field mapping mirrors the comment block in mem_unit.v.
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_mem_full;
    integer pass = 0, fail = 0;

    reg  [6:0]  eop;
    reg  [31:0] emem, ereg;
    reg  [1:0]  epos;
    wire [31:0] eload, estore;
    wire        eisl, eiss, eval;

    mem_unit u_mem(.opcode(eop),.mem_val(emem),.reg_val(ereg),.pos(epos),
                   .load_data(eload),.store_data(estore),
                   .is_load(eisl),.is_store(eiss),.out_valid(eval));

    task chk; input [127:0] name; input [31:0] got, exp; begin
        if (got === exp) begin pass=pass+1; $display("  PASS %0s : 0x%08h", name, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got 0x%08h exp 0x%08h", name, got, exp); end
    end endtask
    task chkb; input [127:0] name; input got, exp; begin
        if (got === exp) begin pass=pass+1; $display("  PASS %0s : %b", name, got); end
        else begin fail=fail+1; $display("  FAIL %0s : got %b exp %b", name, got, exp); end
    end endtask

    task drive; input [6:0] op; input [31:0] mv,rv; input [1:0] p; begin
        eop=op; emem=mv; ereg=rv; epos=p; #1;
    end endtask

    initial begin
        $display("================ r-VEX MEM full tests ================");

        //---------------------------------------- word load
        $display("[LDW]");
        drive(`MEM_LDW,32'hDEADBEEF,0,2'b00); chk ("LDW word"    ,eload,32'hDEADBEEF);
        drive(`MEM_LDW,32'hDEADBEEF,0,2'b00); chkb("LDW is_load" ,eisl,1'b1);

        //---------------------------------------- halfword loads (sign/zero)
        $display("[LDH/LDHU]  half select: 00->[31:16] 01->[23:8] 10->[15:0]");
        drive(`MEM_LDH ,32'h80001234,0,2'b00); chk("LDH  pos00 sign" ,eload,32'hFFFF8000);
        drive(`MEM_LDH ,32'h12800034,0,2'b01); chk("LDH  pos01 sign" ,eload,32'hFFFF8000);
        drive(`MEM_LDH ,32'h00008000,0,2'b10); chk("LDH  pos10 sign" ,eload,32'hFFFF8000);
        drive(`MEM_LDHU,32'h00008000,0,2'b10); chk("LDHU pos10 zero" ,eload,32'h00008000);
        drive(`MEM_LDHU,32'h12340000,0,2'b00); chk("LDHU pos00 zero" ,eload,32'h00001234);

        //---------------------------------------- byte loads (sign/zero)
        $display("[LDB/LDBU]  byte select: 00->[31:24] 01->[23:16] 10->[15:8] 11->[7:0]");
        drive(`MEM_LDB ,32'h80000000,0,2'b00); chk("LDB  pos00 sign" ,eload,32'hFFFFFF80);
        drive(`MEM_LDB ,32'h00800000,0,2'b01); chk("LDB  pos01 sign" ,eload,32'hFFFFFF80);
        drive(`MEM_LDB ,32'h00008000,0,2'b10); chk("LDB  pos10 sign" ,eload,32'hFFFFFF80);
        drive(`MEM_LDB ,32'h00000080,0,2'b11); chk("LDB  pos11 sign" ,eload,32'hFFFFFF80);
        drive(`MEM_LDBU,32'h00000080,0,2'b11); chk("LDBU pos11 zero" ,eload,32'h00000080);
        drive(`MEM_LDBU,32'h12000000,0,2'b00); chk("LDBU pos00 zero" ,eload,32'h00000012);

        //---------------------------------------- stores
        $display("[STW]");
        drive(`MEM_STW,32'h00000000,32'hCAFEBABE,2'b00); chk ("STW word"   ,estore,32'hCAFEBABE);
        drive(`MEM_STW,32'h00000000,32'hCAFEBABE,2'b00); chkb("STW is_store",eiss,1'b1);

        $display("[STH]  insert reg[15:0] into half slot");
        drive(`MEM_STH,32'h0000FFFF,32'hAAAA1234,2'b00); chk("STH pos00",estore,32'h1234FFFF);
        drive(`MEM_STH,32'hAA0000BB,32'hAAAA1234,2'b01); chk("STH pos01",estore,32'hAA1234BB);
        drive(`MEM_STH,32'hFFFF0000,32'hAAAA1234,2'b10); chk("STH pos10",estore,32'hFFFF1234);

        $display("[STB]  insert reg[7:0] into byte slot");
        drive(`MEM_STB,32'h00ABCDEF,32'hAAAAAA78,2'b00); chk("STB pos00",estore,32'h78ABCDEF);
        drive(`MEM_STB,32'hAA00CDEF,32'hAAAAAA78,2'b01); chk("STB pos01",estore,32'hAA78CDEF);
        drive(`MEM_STB,32'hABCD00EF,32'hAAAAAA78,2'b10); chk("STB pos10",estore,32'hABCD78EF);
        drive(`MEM_STB,32'hABCDEF00,32'hAAAAAA78,2'b11); chk("STB pos11",estore,32'hABCDEF78);

        //---------------------------------------- PFT / invalid
        $display("[PFT/invalid]");
        drive(`MEM_PFT,32'h12345678,0,2'b00);
            chkb("PFT valid"    ,eval,1'b1);
            chkb("PFT no-load"  ,eisl,1'b0);
            chkb("PFT no-store" ,eiss,1'b0);
        drive(`OP_STOP,32'h0,0,2'b00); chkb("bad op invalid",eval,1'b0);

        $display("======================================================");
        $display("MEM FULL: %0d passed, %0d failed", pass, fail);
        if (fail==0) $display("RESULT: ALL MEM FULL TESTS PASSED");
        else         $display("RESULT: %0d MEM FULL TEST FAILURES", fail);
        $finish;
    end
endmodule
