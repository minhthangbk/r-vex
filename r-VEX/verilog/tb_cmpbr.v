//======================================================================
// r-VEX | Compare to BR with an immediate (BRIMM form), bare core
//----------------------------------------------------------------------
// Contract: reviews/contracts/cmpbr.md. Expected values come from the
// contract's operation table (VEX compare/logic semantics) computed by the
// model below, not from the RTL.
// Program: NRND random packets of four BRIMM compares (one per slot, four
// distinct BRs, random op/source/imm, imm biased to 0/1/255/256/511),
// then directed packets: branch and slct on a BRIMM result in the next
// packet (CB-05), the old register-compare and GR-compare forms and
// imm type 2'b11 on a non-compare op (CB-04), STOP. The GR file is
// checked untouched by every BRIMM packet (CB-02).
//======================================================================
`timescale 1ns/1ps
`include "rvex_defs.vh"

module tb_cmpbr;
    reg clk = 0, reset = 1, start = 0;
    wire done; wire [31:0] cycles;
    integer pass = 0, fail = 0, k, p, s, s2, seed;

    rvex_core dut(.clk(clk), .reset(reset), .start(start), .done(done), .cycles(cycles));
    always #5 clk = ~clk;

    localparam NRND = 600;
    localparam [31:0] NOP  = 32'd0;
    localparam [31:0] STOP = {`OP_STOP,25'd0};

    // BRIMM syllable (CB-01/03): imm type 2'b11, BR in [19:17], GR A in [16:11], imm in [10:2]
    function [31:0] bi;  input [6:0] op; input [2:0] b; input [5:0] a; input [8:0] im; begin
        bi = {op,2'b11,3'b000,b,a,im,2'b00}; end endfunction
    function [31:0] rr;  input [6:0] op; input [5:0] d,a,b; begin
        rr = {op,`NO_IMM,d,a,b,3'b000,2'b00}; end endfunction
    function [31:0] rrb; input [6:0] op; input [5:0] a,b; input [2:0] dbr; begin
        rrb = {op,`NO_IMM,6'd0,a,b,dbr,2'b00}; end endfunction
    function [31:0] ri;  input [6:0] op; input [5:0] d,a; input [8:0] im; begin
        ri = {op,`SHORT_IMM,d,a,im,2'b00}; end endfunction
    function [31:0] jbr; input [6:0] op; input [11:0] off; input [2:0] bidx; begin
        jbr = {op,8'b0,off,bidx,2'b0}; end endfunction
    function [31:0] slct; input [2:0] b; input [5:0] d,a,c; begin
        slct = {`ALU_SLCT | {4'b0,b},`NO_IMM,d,a,c,3'b000,2'b00}; end endfunction

    // contract operation table (VEX semantics)
    function model; input [6:0] op; input [31:0] a, b; begin
        case (op)
            `ALU_CMPEQ : model = (a == b);
            `ALU_CMPGE : model = ($signed(a) >= $signed(b));
            `ALU_CMPGEU: model = (a >= b);
            `ALU_CMPGT : model = ($signed(a) >  $signed(b));
            `ALU_CMPGTU: model = (a >  b);
            `ALU_CMPLE : model = ($signed(a) <= $signed(b));
            `ALU_CMPLEU: model = (a <= b);
            `ALU_CMPLT : model = ($signed(a) <  $signed(b));
            `ALU_CMPLTU: model = (a <  b);
            `ALU_CMPNE : model = (a != b);
            `ALU_NANDL : model = !((a != 0) && (b != 0));
            `ALU_NORL  : model = !((a != 0) || (b != 0));
            `ALU_ORL   : model =  ((a != 0) || (b != 0));
            `ALU_ANDL  : model =  ((a != 0) && (b != 0));
            default    : model = 1'bx;
        endcase
    end endfunction

    reg [6:0] OPS [0:13];
    reg [31:0] grv [0:63];             // preloaded GR values (never written by BRIMM)
    reg [2:0]  ebi [0:NRND-1][0:3];    // expected BR index per packet/slot
    reg        ebv [0:NRND-1][0:3];    // expected BR value
    reg [8:0]  imms [0:9];
    reg [2:0] perm [0:3];
    reg [6:0] op; reg [5:0] a; reg [8:0] im; reg [2:0] t;
    reg [31:0] sy [0:3];
    integer opcnt [0:13];

    task chk; input [255:0] nm; input [31:0] got, exp; begin
        if (got === exp) pass = pass + 1;
        else begin fail = fail + 1; $display("  FAIL %0s : got %0d (0x%0h) exp %0d (0x%0h)", nm, got, got, exp, exp); end
    end endtask

    initial begin
        if (!$value$plusargs("SEED=%d", seed)) seed = 1;
        OPS[0]=`ALU_CMPEQ; OPS[1]=`ALU_CMPGE; OPS[2]=`ALU_CMPGEU; OPS[3]=`ALU_CMPGT; OPS[4]=`ALU_CMPGTU;
        OPS[5]=`ALU_CMPLE; OPS[6]=`ALU_CMPLEU; OPS[7]=`ALU_CMPLT; OPS[8]=`ALU_CMPLTU; OPS[9]=`ALU_CMPNE;
        OPS[10]=`ALU_NANDL; OPS[11]=`ALU_NORL; OPS[12]=`ALU_ORL; OPS[13]=`ALU_ANDL;
        imms[0]=0; imms[1]=1; imms[2]=255; imms[3]=256; imms[4]=511; imms[5]=2; imms[6]=44;
        imms[7]=510; imms[8]=128; imms[9]=9'h155;
        for (k = 0; k < 14; k = k + 1) opcnt[k] = 0;
        for (k = 0; k < 4096; k = k + 1) dut.imem[k] = 128'd0;
        // GR values: r0 = 0; r2..r40 around the immediates (v-1, v, v+1), zero, negatives, random
        grv[0] = 0;
        for (k = 1; k < 64; k = k + 1) begin
            case (k % 6)
                0: grv[k] = imms[$unsigned($random(seed)) % 10];
                1: grv[k] = imms[$unsigned($random(seed)) % 10] - 1;
                2: grv[k] = imms[$unsigned($random(seed)) % 10] + 1;
                3: grv[k] = 0;
                4: grv[k] = -($unsigned($random(seed)) % 600);
                default: grv[k] = $random(seed);
            endcase
        end
        // random BRIMM packets 0..NRND-1
        for (p = 0; p < NRND; p = p + 1) begin
            // 4 distinct BRs out of 8: start t, odd stride (a unit mod 8)
            t = $unsigned($random(seed)) % 8; im = 1 + 2 * ($unsigned($random(seed)) % 4);
            for (s = 0; s < 4; s = s + 1) perm[s] = (t + s * im) % 8;
            for (s = 0; s < 4; s = s + 1) begin
                k  = $unsigned($random(seed)) % 14; op = OPS[k]; opcnt[k] = opcnt[k] + 1;
                a  = 2 + $unsigned($random(seed)) % 39;
                if ($unsigned($random(seed)) % 2) im = imms[$unsigned($random(seed)) % 10];
                else im = $random(seed);
                sy[s] = bi(op, perm[s], a, im);
                ebi[p][s] = perm[s]; ebv[p][s] = model(op, grv[a], {23'b0, im});
            end
            dut.imem[p] = {sy[3], sy[2], sy[1], sy[0]};
        end
        p = NRND;
        // CB-05: r41 = 5 (preloaded). b5 = r41 ltu 6 (true); next packet br b5 -> p+3
        grv[41] = 5; grv[42] = 100; grv[43] = 200;
        for (k = 50; k <= 56; k = k + 1) grv[k] = 0;   // written by the directed packets
        dut.imem[p]   = {NOP, NOP, NOP, bi(`ALU_CMPLTU, 3'd5, 6'd41, 9'd6)};
        dut.imem[p+1] = {NOP, NOP, NOP, jbr(`CTRL_BR, p+3, 3'd5)};
        dut.imem[p+2] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd50, 6'd0, 9'd1)};          // skipped
        // p+3: b6 = r41 geu 6 (false) in slot 2; b2 = r41 eq 5 (true) in slot 3
        dut.imem[p+3] = {bi(`ALU_CMPEQ, 3'd2, 6'd41, 9'd5), bi(`ALU_CMPGEU, 3'd6, 6'd41, 9'd6), NOP, ri(`ALU_ADD, 6'd51, 6'd0, 9'd1)};
        // p+4: brf b6 -> p+6 (taken, b6 false) | r52 = slct b2 ? r42 : r43
        dut.imem[p+4] = {NOP, NOP, slct(3'd2, 6'd52, 6'd42, 6'd43), jbr(`CTRL_BRF, p+6, 3'd6)};
        dut.imem[p+5] = {NOP, NOP, NOP, ri(`ALU_ADD, 6'd53, 6'd0, 9'd1)};          // skipped
        // CB-04 p+6: register compare to BR syl[4:2] (b7 = r42 lt r43, true) | GR compare SHORT_IMM r54 = r41 eq 5
        //            | imm type 2'b11 on add: r55 = r42 + r43 (operand 2 is the GR)
        dut.imem[p+6] = {NOP, {`ALU_ADD,2'b11,6'd55,6'd42,6'd43,3'b111,2'b00},
                         ri(`ALU_CMPEQ, 6'd54, 6'd41, 9'd5), rrb(`ALU_CMPLT, 6'd42, 6'd43, 3'd7)};
        // CB-04 p+7: clz with imm type 2'b11 stays a GR op (is_brimm excludes CLZ): r56 = clz(r42 = 100) = 25
        dut.imem[p+7] = {NOP, NOP, NOP, {`ALU_CLZ,2'b11,6'd56,6'd42,6'd0,3'b011,2'b00}};
        dut.imem[p+8] = {NOP, NOP, NOP, STOP};

        repeat (2) @(posedge clk);
        reset = 0;
        @(posedge clk);
        for (k = 0; k < 64; k = k + 1) dut.gr[k] = grv[k];
        start = 1;
        k = 0;
        while (done !== 1'b1 && k < 5000) begin @(posedge clk); k = k + 1; end
        chk("STOP reached", done, 1);
        // CB-02: no BRIMM packet wrote a GR
        for (k = 0; k < 64; k = k + 1)
            if (k < 50 || k > 56) chk("CB-02 GR untouched", dut.gr[k], grv[k]);
        // CB-05 branch / slct in the next packet
        chk("CB-05 br taken (r50 skipped)", dut.gr[50], 0);
        chk("CB-05 target ran (r51)", dut.gr[51], 1);
        chk("CB-05 brf taken (r53 skipped)", dut.gr[53], 0);
        chk("CB-05 slct on BRIMM b2", dut.gr[52], 100);
        // CB-04 old forms
        chk("CB-04 reg compare -> BR syl[4:2]", dut.br[7], 1);
        chk("CB-04 GR compare SHORT_IMM", dut.gr[54], 1);
        chk("CB-04 imm type 11 on add uses GR", dut.gr[55], 300);
        chk("CB-04 clz with imm type 11 writes GR", dut.gr[56], 25);
        for (k = 0; k < 14; k = k + 1)
            if (opcnt[k] == 0) begin fail = fail + 1; $display("  FAIL op %0d never generated", k); end
        $display("=========================================================");
        $display("CMPBR TEST (seed %0d): %0d passed, %0d failed", seed, pass, fail);
        if (fail == 0) $display("RESULT: CMPBR PASSED"); else $display("RESULT: CMPBR FAILED");
        $finish;
    end

    // CB-01/03: after random packet p commits (pc == p + 1), BR[ebi] == ebv for its four slots
    always @(posedge clk) if (!reset && dut.running && !dut.done && dut.pc >= 1 && dut.pc <= NRND) begin
        for (s2 = 0; s2 < 4; s2 = s2 + 1)
            chk("CB-01/03 BR value", dut.br[ebi[dut.pc - 1][s2]], ebv[dut.pc - 1][s2]);
    end
endmodule
