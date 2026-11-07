//======================================================================
// r-VEX | Integrated execution core (equivalent Verilog port)
//----------------------------------------------------------------------
// SCOPE / DEVIATION (documented in the review):
//   The original VHDL uses a multi-FSM, clock-level handshake spread over
//   fetch/decode/execute/mem/writeback plus a ~700-line top. That handshake
//   is NOT cycle-reproduced here. Instead this is a clean, synchronous
//   "one VLIW packet per active issue cycle" core that preserves:
//     - the 4-slot resource model (ALU x4, MUL slots 1&2, CTRL slot 0, MEM slot 3)
//     - the syllable field layout of decode.vhd
//     - VEX packet semantics (all source reads happen before writes)
//   This is sufficient to drive every fixed/new instruction end-to-end and
//   observe GR / BR / data-memory / PC / XNOP-stall effects.
//
//   GR file is 64 deep (6-bit addresses) so $r0.63-style link regs are legal;
//   the VHDL constant GR_DEPTH=32 was a latent limitation. GR[0] is hardwired 0.
//
// MEMORY SYSTEM (2026-10-31):
//   PC_W     : PC width. 12 = the whole absolute branch field syl[16:5], so
//              imem holds 2**PC_W packets (was 8 bits / 256 packets).
//   DMEM_AW  : byte-address width of the internal data memory (was 10).
//   EXT_DMEM : 0 = internal single-cycle dmem (default, used by every
//              existing testbench). 1 = slot-3 loads/stores go out on the
//              d_* port (to an L1 D-cache). The port is combinational: on
//              d_ready the access completes in this cycle; without it the
//              WHOLE packet is held (no GR/BR/PC/memory update), i.e. the
//              core stalls on a cache miss and the compiler-visible
//              schedule is unchanged.
//   EXT_IMEM : 1 = packets come from the i_* port (an L1 I-cache); a fetch
//              miss holds the core the same way.
//
// PERFORMANCE INDICATOR UNIT (2026-10-31 r5, reviews/contracts/piu.md):
//   PIU      : 1 = slot-3 accesses with addr[15:8] == 8'hFE go to this core's
//              rvex_piu (event counters) instead of dmem / the D port. They
//              complete in one cycle and never stall.
//======================================================================
`include "rvex_defs.vh"
`include "rvex_piu.v"

module rvex_core #(
    parameter PC_W     = 12,
    parameter DMEM_AW  = 16,
    parameter EXT_DMEM = 0,
    parameter EXT_IMEM = 0,
    parameter PIU      = 1
) (
    input  wire clk,
    input  wire reset,
    input  wire start,
    output reg  done,
    output reg [31:0] cycles,
    // external data port (EXT_DMEM=1)
    output wire        d_req,
    output wire        d_we,
    output wire [31:0] d_addr,     // byte address, word aligned
    output wire [31:0] d_wdata,    // whole merged word (sub-word stores)
    output wire        d_word,     // the access is ldw/stw (hw_sem accepts only these)
    input  wire [31:0] d_rdata,
    input  wire        d_ready,
    output reg  [31:0] stall_cycles,     // cycles held by D-side misses
    // external instruction port (EXT_IMEM=1, to an L1 I-cache)
    output wire              i_req,
    output wire [PC_W-1:0]   i_addr,
    input  wire [127:0]      i_rdata,
    input  wire              i_ready,
    output reg  [31:0]       istall_cycles  // cycles held by I-side misses
);
    // ---- state ----
    reg [127:0] imem [0:(1<<PC_W)-1];      // instruction memory (4 syllables per packet)
    reg [31:0]  gr   [0:63];       // general registers ($r0.0 = 0)
    reg         br   [0:7];        // branch registers
    reg [31:0]  dmem [0:(1<<(DMEM_AW-2))-1]; // data memory (word addressed)
    reg [PC_W-1:0] pc;
    reg [11:0]  stall_cnt;
    reg         running;
    integer     i;

    // ---- current packet & syllables ----
    wire [127:0] packet = EXT_IMEM ? i_rdata : imem[pc];
    wire [31:0]  syl0 = packet[31:0];
    wire [31:0]  syl1 = packet[63:32];
    wire [31:0]  syl2 = packet[95:64];
    wire [31:0]  syl3 = packet[127:96];

    // ---- field extraction ----
    wire [6:0] op0=syl0[31:25], op1=syl1[31:25], op2=syl2[31:25], op3=syl3[31:25];
    wire [1:0] imt0=syl0[24:23], imt1=syl1[24:23], imt2=syl2[24:23], imt3=syl3[24:23];
    wire [5:0] dst0=syl0[22:17], dst1=syl1[22:17], dst2=syl2[22:17], dst3=syl3[22:17];
    wire [5:0] r1a0=syl0[16:11], r1a1=syl1[16:11], r1a2=syl2[16:11], r1a3=syl3[16:11];
    wire [5:0] r2a0=syl0[10:5],  r2a1=syl1[10:5],  r2a2=syl2[10:5];
    // Stores read the data GR from the dst field; every other slot-3 op
    // (ALU, load) reads src2 from [10:5] like the other slots.
    wire       st3 = (op3 == `MEM_STW) || (op3 == `MEM_STH) || (op3 == `MEM_STB);
    wire [5:0] r2a3 = st3 ? syl3[22:17] : syl3[10:5];
    wire [2:0] dbr0=syl0[4:2],   dbr1=syl1[4:2],   dbr2=syl2[4:2],   dbr3=syl3[4:2];
    wire [2:0] sbr0=syl0[27:25], sbr1=syl1[27:25], sbr2=syl2[27:25];
    wire [11:0] off0=syl0[16:5];

    // ---- helper: is a compare/logic op that may target BR (CMPEQ..ANDL) ----
    // CLZ (0x65) and MTB (0x67) sit inside the CMPEQ..ANDL opcode range but are not compares: CLZ writes
    // only a GR (clz.md), so `clz $r0.0 = x` must write nothing (BUG-181, pfuzz.md PZ-02, found by pfuzz r17:
    // it wrote BR[syl[4:2]] = clz(x)[0] and counted as a brn writer). MTB keeps its own writeback branch.
    function is_cmp; input [6:0] op; begin
        is_cmp = (op >= `ALU_CMPEQ) && (op <= `ALU_ANDL) && (op != `ALU_CLZ) && (op != `ALU_MTB); end
    endfunction
    function is_mul; input [6:0] op; begin
        is_mul = (op[6:4] == 3'b000) && (op != `OP_NOP); end
    endfunction
    function is_addcg_divs; input [6:0] op; begin
        is_addcg_divs = (op[6:3]==`ALU_ADDCG4) || (op[6:3]==`ALU_DIVS4); end
    endfunction

    // ---- operand read (combinational) ----
    wire [31:0] gr_r1_0 = gr[r1a0], gr_r2_0 = gr[r2a0];
    wire [31:0] gr_r1_1 = gr[r1a1], gr_r2_1 = gr[r2a1];
    wire [31:0] gr_r1_2 = gr[r1a2], gr_r2_2 = gr[r2a2];
    wire [31:0] gr_r1_3 = gr[r1a3], gr_r2_3 = gr[r2a3];

    // BRIMM (reviews/contracts/cmpbr.md): a compare/logic op with imm type
    // 2'b11 writes BR syl[19:17] = GR syl[16:11] <op> zext(syl[10:2]) and no
    // GR. A BR compare leaves the GR dst field free, so the BR index moves
    // there and syl[10:2] holds the immediate as for SHORT_IMM.
    function is_brimm; input [6:0] op; input [1:0] imt; begin
        is_brimm = (imt == `LONG_IMM) &&
                   (((op >= `ALU_CMPEQ) && (op <= `ALU_NORL)) || op == `ALU_ORL || op == `ALU_ANDL); end
    endfunction
    wire bi0 = is_brimm(op0, imt0), bi1 = is_brimm(op1, imt1),
         bi2 = is_brimm(op2, imt2), bi3 = is_brimm(op3, imt3);

    function [31:0] op2sel; input [1:0] imt; input bi; input [31:0] regv; input [31:0] syl; begin
        case (imt)
            `SHORT_IMM : op2sel = {23'b0, syl[10:2]};
            `BRANCH_IMM: op2sel = {20'b0, syl[16:5]};
            `LONG_IMM  : op2sel = bi ? {23'b0, syl[10:2]} : regv;
            default    : op2sel = regv;
        endcase
    end endfunction

    wire [31:0] a2_0 = op2sel(imt0, bi0, gr_r2_0, syl0);
    wire [31:0] a2_1 = op2sel(imt1, bi1, gr_r2_1, syl1);
    wire [31:0] a2_2 = op2sel(imt2, bi2, gr_r2_2, syl2);
    wire [31:0] a2_3 = op2sel(imt3, bi3, gr_r2_3, syl3);

    // BR operand for ALU (SLCT/SLCTF/ADDCG/DIVS use rb=syl[27:25])
    wire cin0 = br[sbr0], cin1 = br[sbr1], cin2 = br[sbr2];
    wire cin3 = br[syl3[27:25]];

    // ---- functional units ----
    wire [31:0] alu0_r,alu1_r,alu2_r,alu3_r;
    wire        alu0_c,alu1_c,alu2_c,alu3_c;
    wire        alu0_v,alu1_v,alu2_v,alu3_v;
    alu u_alu0(.aluop(op0),.src1(gr_r1_0),.src2(a2_0),.cin(cin0),.result(alu0_r),.cout(alu0_c),.out_valid(alu0_v));
    alu u_alu1(.aluop(op1),.src1(gr_r1_1),.src2(a2_1),.cin(cin1),.result(alu1_r),.cout(alu1_c),.out_valid(alu1_v));
    alu u_alu2(.aluop(op2),.src1(gr_r1_2),.src2(a2_2),.cin(cin2),.result(alu2_r),.cout(alu2_c),.out_valid(alu2_v));
    alu u_alu3(.aluop(op3),.src1(gr_r1_3),.src2(a2_3),.cin(cin3),.result(alu3_r),.cout(alu3_c),.out_valid(alu3_v));

    wire [31:0] mul1_r, mul2_r; wire mul1_v, mul2_v, mul1_o, mul2_o;
    mul u_mul1(.mulop(op1),.src1(gr_r1_1),.src2(a2_1),.result(mul1_r),.overflow(mul1_o),.out_valid(mul1_v));
    mul u_mul2(.mulop(op2),.src1(gr_r1_2),.src2(a2_2),.result(mul2_r),.overflow(mul2_o),.out_valid(mul2_v));

    // ---- memory unit (slot 3) ----
    wire [31:0] mem_ea        = gr_r1_3 + {23'b0, syl3[10:2]};
    wire [DMEM_AW-1:0] dmem_byte_addr = mem_ea[DMEM_AW-1:0];
    wire [DMEM_AW-3:0] dmem_word_addr = dmem_byte_addr[DMEM_AW-1:2];
    wire [1:0] dmem_pos       = mem_ea[1:0];
    // PIU window = 0xFE00..0xFEFF exactly (piu.md): bits [DMEM_AW-1:8] == 0..0FE, zero-extended
    // (R21-2: [15:8] alone also mapped 0x1FE00 to the PIU when DMEM_AW = 17)
    wire        piu_hit       = PIU && (op3[6:4] == 3'b001) && mem_ea[DMEM_AW-1:8] == 8'hFE;
    wire [31:0] piu_rdata;
    wire [31:0] dmem_word     = piu_hit ? piu_rdata : EXT_DMEM ? d_rdata : dmem[dmem_word_addr];
    wire [31:0] mem_load, mem_store; wire mem_is_load, mem_is_store, mem_v;
    mem_unit u_mem(.opcode(op3),.mem_val(dmem_word),.reg_val(gr_r2_3),.pos(dmem_pos),
                   .load_data(mem_load),.store_data(mem_store),
                   .is_load(mem_is_load),.is_store(mem_is_store),.out_valid(mem_v));

    // ---- control unit (slot 0) ----
    wire [PC_W-1:0] c_pc_goto; wire [31:0] c_link; wire c_taken, c_wlink, c_xnop, c_rfi, c_v;
    wire [11:0] c_xnop_n;
    // BR/BRF read rb = syl[4:2]. With syl[17] set (brn/brfn,
    // reviews/contracts/brnew.md) the branch takes the value that slot 1..3
    // of this packet writes to that BR (slot 3 last, as in the writeback).
    function brw_hit; input [6:0] op; input [1:0] imt; input [31:0] syl; input [2:0] b; begin
        if (op == `ALU_MTB || is_addcg_divs(op))       brw_hit = (syl[4:2] == b);
        else if (is_brimm(op, imt))                    brw_hit = (syl[19:17] == b);
        else if (is_cmp(op) && syl[22:17] == 6'd0)     brw_hit = (syl[4:2] == b);
        else                                           brw_hit = 1'b0;
    end endfunction
    function brw_val; input [6:0] op; input [31:0] r; input c; begin
        brw_val = (op == `ALU_MTB || is_addcg_divs(op)) ? c : r[0];
    end endfunction
    wire [2:0] c_bi = syl0[4:2];
    wire c_new = syl0[17] && (op0 == `CTRL_BR || op0 == `CTRL_BRF);
    wire c_br_in = !c_new ? br[c_bi] :
                   brw_hit(op3, imt3, syl3, c_bi) && op3[6:4] != 3'b001 ? brw_val(op3, alu3_r, alu3_c) :
                   brw_hit(op2, imt2, syl2, c_bi) ? brw_val(op2, alu2_r, alu2_c) :
                   brw_hit(op1, imt1, syl1, c_bi) ? brw_val(op1, alu1_r, alu1_c) : br[c_bi];
    // The link register is implicit ($l0.0 = GR63), as in VEX: syl[16:5] is
    // the offset of return/rfi (SP + offset), so it cannot also name a GR.
    ctrl_unit #(.PC_W(PC_W)) u_ctrl(.opcode(op0),.pc(pc),.lr(gr[63]),.sp(gr[1]),.offset(off0),.br(c_br_in),
                     .pc_goto(c_pc_goto),.link_val(c_link),.taken(c_taken),.writes_link(c_wlink),
                     .is_xnop(c_xnop),.xnop_n(c_xnop_n),.is_rfi(c_rfi),.out_valid(c_v));

    wire slot0_is_ctrl = (op0[6:4] == 3'b010);
    wire slot0_is_stop = (op0 == `OP_STOP);

    // ---- per-slot GR result select (ALU vs MUL) ----
    wire [31:0] res1 = is_mul(op1) ? mul1_r : alu1_r;
    wire [31:0] res2 = is_mul(op2) ? mul2_r : alu2_r;

    // ---- external data port / miss stall ----
    assign i_req   = EXT_IMEM && running && !done && stall_cnt == 0;
    assign i_addr  = pc;
    wire   fstall  = i_req && !i_ready;          // packet not fetched yet
    wire issue   = running && !done && stall_cnt == 0 && !fstall && !slot0_is_stop;
    wire mem_acc = (op3[6:4] == 3'b001) && (mem_is_load || mem_is_store);
    assign d_req   = EXT_DMEM && issue && mem_acc && !piu_hit;
    assign d_we    = mem_is_store;
    assign d_addr  = {mem_ea[31:2], 2'b00};
    assign d_wdata = mem_store;
    assign d_word  = (op3 == `MEM_LDW) || (op3 == `MEM_STW);
    wire   dstall  = d_req && !d_ready;

    // ---- performance indicator unit ----
    wire   active  = running && !done;
    wire   commit  = issue && !dstall;
    rvex_piu u_piu(.clk(clk), .reset(reset),
        .active(active), .commit(commit), .xnop_c(active && stall_cnt != 0),
        .istall_c(active && stall_cnt == 0 && fstall), .dstall_c(dstall), .semstall_c(&mem_ea[DMEM_AW-1:8]),
        .op0(op0), .op1(op1), .op2(op2), .op3(op3),
        .ctrl0(slot0_is_ctrl), .taken0(slot0_is_ctrl && c_taken && !c_xnop),
        .mul1(is_mul(op1)), .mul2(is_mul(op2)),
        .ld3(mem_acc && mem_is_load), .st3(mem_acc && mem_is_store),
        // PIU-18: a test-and-set ldw on hw_sem (system only: EXT_DMEM) that found the lock taken
        // window decoded like rvex_system (d_addr[AW-1:8] all ones; Codex R20 C-1: [15:8] was wrong for AW != 16)
        .semfail3(EXT_DMEM && op3 == `MEM_LDW && &mem_ea[DMEM_AW-1:8] && d_rdata[0]),
        .addr(mem_ea[7:2]), .rdata(piu_rdata),
        .we(commit && piu_hit && mem_is_store), .wdata(mem_store));

    // ================= sequential execution =================
    always @(posedge clk) begin
        if (reset) begin
            pc <= {PC_W{1'b0}}; stall_cnt <= 12'd0; running <= 1'b0; done <= 1'b0; cycles <= 32'd0;
            stall_cycles <= 32'd0; istall_cycles <= 32'd0;
            for (i=0;i<64;i=i+1) gr[i] <= 32'd0;
            for (i=0;i<8;i=i+1)  br[i] <= 1'b0;
        end else begin
            if (start) running <= 1'b1;
            if (running && !done) begin
                cycles <= cycles + 1;
                if (stall_cnt != 0) begin
                    stall_cnt <= stall_cnt - 1;         // XNOP bubble
                end else if (fstall) begin
                    istall_cycles <= istall_cycles + 1;  // I-cache miss: no packet yet
                end else if (dstall) begin
                    stall_cycles <= stall_cycles + 1;    // D-cache miss: hold the packet
                end else if (slot0_is_stop) begin
                    done <= 1'b1;                        // STOP
                end else begin
                    //--------------- writeback slot 0 ---------------
                    if (slot0_is_ctrl) begin
                        if (c_wlink && dst0 != 6'd0) gr[dst0] <= c_link;   // CALL/RETURN link
                    end else if (op0 == `ALU_MTB) begin
                        br[dbr0] <= alu0_c;
                    end else if (bi0) begin
                        br[syl0[19:17]] <= alu0_r[0];
                    end else if (is_addcg_divs(op0)) begin
                        if (dst0 != 6'd0) gr[dst0] <= alu0_r; br[dbr0] <= alu0_c;
                    end else if (is_cmp(op0) && dst0 == 6'd0) begin
                        br[dbr0] <= alu0_r[0];
                    end else if (op0 != `OP_NOP && alu0_v && dst0 != 6'd0) begin
                        gr[dst0] <= alu0_r;
                    end

                    //--------------- writeback slot 1 ---------------
                    if (op1 == `ALU_MTB) br[dbr1] <= alu1_c;
                    else if (bi1) br[syl1[19:17]] <= alu1_r[0];
                    else if (is_addcg_divs(op1)) begin
                        if (dst1 != 6'd0) gr[dst1] <= alu1_r; br[dbr1] <= alu1_c;
                    end else if (is_cmp(op1) && dst1 == 6'd0) br[dbr1] <= alu1_r[0];
                    else if (op1 != `OP_NOP && dst1 != 6'd0) gr[dst1] <= res1;

                    //--------------- writeback slot 2 ---------------
                    if (op2 == `ALU_MTB) br[dbr2] <= alu2_c;
                    else if (bi2) br[syl2[19:17]] <= alu2_r[0];
                    else if (is_addcg_divs(op2)) begin
                        if (dst2 != 6'd0) gr[dst2] <= alu2_r; br[dbr2] <= alu2_c;
                    end else if (is_cmp(op2) && dst2 == 6'd0) br[dbr2] <= alu2_r[0];
                    else if (op2 != `OP_NOP && dst2 != 6'd0) gr[dst2] <= res2;

                    //--------------- writeback slot 3 (ALU or MEM) ---------------
                    if (op3[6:4] == 3'b001) begin           // MEM range
                        if (mem_is_store && !EXT_DMEM && !piu_hit) dmem[dmem_word_addr] <= mem_store;
                        else if (mem_is_load && dst3 != 6'd0) gr[dst3] <= mem_load;
                        // PFT / SYL_FOLLOW -> no writeback
                    end else if (op3 == `ALU_MTB) br[dbr3] <= alu3_c;
                    else if (bi3) br[syl3[19:17]] <= alu3_r[0];
                    else if (is_addcg_divs(op3)) begin
                        if (dst3 != 6'd0) gr[dst3] <= alu3_r; br[dbr3] <= alu3_c;
                    end else if (is_cmp(op3) && dst3 == 6'd0) br[dbr3] <= alu3_r[0];
                    else if (op3 != `OP_NOP && dst3 != 6'd0) gr[dst3] <= alu3_r;

                    //--------------- PC / stall update ---------------
                    if (slot0_is_ctrl && c_xnop) begin
                        stall_cnt <= c_xnop_n;              // NEW: XNOP stall n cycles
                        pc <= pc + 1'b1;
                    end else if (slot0_is_ctrl && c_taken) begin
                        pc <= c_pc_goto;                    // branch / call / return
                    end else begin
                        pc <= pc + 1'b1;
                    end
                end
            end
            gr[0] <= 32'd0; // r0 hardwired
        end
    end
endmodule
