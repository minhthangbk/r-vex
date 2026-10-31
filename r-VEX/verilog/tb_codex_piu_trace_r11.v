// Independent dynamic PIU cross-check, extended in round 11 with MULPKT.
`timescale 1ns/1ps

module tb_codex_piu_trace_r11;
    reg clk = 0, reset = 1, start = 0;
    wire done; wire [31:0] cycles;
    integer k, limit;
    integer ref_cycle = 0, ref_pkt = 0, ref_ops = 0, ref_ld = 0, ref_mulpkt = 0;
    integer first_bad = -1;
    reg [8*1024-1:0] imem_path, dmem_path;

    rvex_core dut(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles));
    always #5 clk = ~clk;

    initial begin
        if (!$value$plusargs("IMEM=%s", imem_path)) $finish;
        if (!$value$plusargs("DMEM=%s", dmem_path)) $finish;
        if (!$value$plusargs("LIMIT=%d", limit)) limit = 20000000;
        for (k = 0; k < (1 << dut.PC_W); k = k + 1) dut.imem[k] = 0;
        for (k = 0; k < (1 << (dut.DMEM_AW - 2)); k = k + 1) dut.dmem[k] = 0;
        $readmemh(imem_path, dut.imem);
        $readmemh(dmem_path, dut.dmem);
        repeat (2) @(posedge clk);
        reset = 0; start = 1; k = 0;
        while (!done && k < limit) begin @(posedge clk); k = k + 1; end
        #2;
        $display("REF done=%0d core_cycles=%0d cycle=%0d pkt=%0d ops=%0d ld=%0d mulpkt=%0d first_bad=%0d",
                 done, cycles, ref_cycle, ref_pkt, ref_ops, ref_ld, ref_mulpkt, first_bad);
        $display("PIU cycle=%0d pkt=%0d ops=%0d ld=%0d mulpkt=%0d", dut.u_piu.cyc,
                 dut.u_piu.pkt, dut.u_piu.ops, dut.u_piu.ld, dut.u_piu.mulpkt);
        $finish;
    end

    always @(posedge clk) begin : reference_monitor
        integer add_ops;
        reg ref_md1, ref_md2;
        if (reset || (dut.u_piu.we && dut.u_piu.addr == 0 && dut.u_piu.wdata[1])) begin
            ref_cycle = 0; ref_pkt = 0; ref_ops = 0; ref_ld = 0; ref_mulpkt = 0;
        end else if (dut.active && dut.u_piu.en) begin
            ref_cycle = ref_cycle + 1;
            if (dut.commit) begin
                add_ops = (dut.op0 != 0) + (dut.op1 != 0) + (dut.op2 != 0) + (dut.op3 != 0);
                ref_md1 = dut.op1 >= 1 && dut.op1 <= 11;
                ref_md2 = dut.op2 >= 1 && dut.op2 <= 11;
                ref_pkt = ref_pkt + 1;
                ref_ops = ref_ops + add_ops;
                if (dut.mem_acc && dut.mem_is_load) ref_ld = ref_ld + 1;
                if (ref_md1 || ref_md2) ref_mulpkt = ref_mulpkt + 1;
            end
        end
        #1;
        if (first_bad < 0 && (dut.u_piu.cyc !== ref_cycle || dut.u_piu.pkt !== ref_pkt ||
                             dut.u_piu.ops !== ref_ops || dut.u_piu.ld !== ref_ld ||
                             dut.u_piu.mulpkt !== ref_mulpkt)) begin
            first_bad = cycles;
            $display("FIRST_MISMATCH core_cycle=%0d pc=%0d ref=%0d/%0d/%0d/%0d/%0d piu=%0d/%0d/%0d/%0d/%0d",
                     cycles, dut.pc, ref_cycle, ref_pkt, ref_ops, ref_ld, ref_mulpkt,
                     dut.u_piu.cyc, dut.u_piu.pkt, dut.u_piu.ops, dut.u_piu.ld, dut.u_piu.mulpkt);
        end
    end
endmodule
