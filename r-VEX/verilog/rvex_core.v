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
//======================================================================
`include "rvex_defs.vh"

module rvex_core (
    input  wire clk,
    input  wire reset,
    input  wire start,
    output reg  done,
    output reg [31:0] cycles
);
    // ---- state ----
    reg [127:0] imem [0:255];      // instruction memory (4 syllables per packet)
    reg [31:0]  gr   [0:63];       // general registers ($r0.0 = 0)
    reg         br   [0:7];        // branch registers
    reg [31:0]  dmem [0:255];      // data memory (word addressed)
    reg [7:0]   pc;
    reg [11:0]  stall_cnt;
    reg         running;
    integer     i;

    // ---- current packet & syllables ----
    wire [127:0] packet = imem[pc];
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
    // FIX (F8): slot 3 hosts the memory unit, and a STORE takes the register
    // holding the data to write from the dst field -- that is why this used to
    // read syl3[22:17] unconditionally. But every other operation addresses its
    // second source with syl[10:5] like slots 0-2, so an ALU op placed in slot 3
    // used to read gr[dst] instead of gr[src2]. Select on the opcode instead.
    wire       st3  = (op3==`MEM_STW) || (op3==`MEM_STH) || (op3==`MEM_STB);
    wire [5:0] r2a3 = st3 ? syl3[22:17] : syl3[10:5];
    wire [2:0] dbr0=syl0[4:2],   dbr1=syl1[4:2],   dbr2=syl2[4:2],   dbr3=syl3[4:2];
    wire [2:0] sbr0=syl0[27:25], sbr1=syl1[27:25], sbr2=syl2[27:25];
    // Branch target for GOTO/CALL/BR/BRF: those carry no register source, so the
    // wide 12-bit field is free.  RETURN/RFI do read a register (the link) from
    // syl[16:11], which OVERLAPS that field -- with link=GR63 the top six offset
    // bits were forced to 111111 and "return $r0.1 = $r0.1, 0, $l0.0" computed
    // sp+4032.  FIX (F5): take the frame-pop offset from the standard 9-bit
    // immediate field syl[10:2] instead, which is disjoint from the link index.
    wire        isret0 = (op0==`CTRL_RETURN) || (op0==`CTRL_RFI);
    wire [11:0] off0   = isret0 ? {3'b0, syl0[10:2]} : syl0[16:5];

    // ---- helper: is a compare/logic op that may target BR (CMPEQ..ANDL) ----
    function is_cmp; input [6:0] op; begin
        is_cmp = (op >= `ALU_CMPEQ) && (op <= `ALU_ANDL); end
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

    // ---- immediate operand select ----
    // VEX defines three immediate widths (Fisher, App. A): branch offsets fit a
    // single syllable, SHORT immediates are "9-bit for all operations", and LONG
    // immediates are "32-bit for all operations" and "draw bits upon one adjacent
    // extension syllable in the same cluster and instruction".
    //
    // FIX (F7a): the 9-bit short immediate is SIGN-extended.  It used to be
    // zero-extended, which made the frame-allocating prologue the compiler emits
    // -- "add $r0.1 = $r0.1, (-0x20)", see the VEX manual's own example -- read
    // as a large positive number, so no function with a stack frame could run.
    //
    // FIX (F7b): LONG_IMM now decodes.  It was declared in rvex_defs.vh but had
    // no case here, so a constant outside the short range was unencodable.  The
    // extension syllable is the NEXT higher slot; it carries the top 23 bits and
    // is marked with OP_SYLFOLW so it issues nothing of its own.  Slot 3 has no
    // successor and therefore cannot carry a long immediate.
    // FIX (F10): a compare whose result goes to a BRANCH register carries no GR
    // destination, and its destination-BR index sits in syl[4:2] -- inside the
    // 9-bit immediate field.  The two cannot both be encoded, which used to
    // force the assembler to burn a scratch register materialising every
    // "cmplt $b0.0, $rA, 1".  Give that one form a narrower 6-bit signed
    // immediate from syl[10:5], which leaves syl[4:2] free for the BR index.
    function narrow_imm; input [6:0] op; input [5:0] dst; begin
        narrow_imm = is_cmp(op) && (dst == 6'd0); end
    endfunction

    function [31:0] op2sel;
        input [1:0] imt; input [31:0] regv; input [31:0] syl; input [31:0] ext;
        input narrow;
    begin
        case (imt)
            `SHORT_IMM : op2sel = narrow ? {{26{syl[10]}}, syl[10:5]}    // 6-bit
                                         : {{23{syl[10]}}, syl[10:2]};   // 9-bit
            `BRANCH_IMM: op2sel = {20'b0, syl[16:5]};
            `LONG_IMM  : op2sel = {ext[22:0],     syl[10:2]};   // 23 + 9 = 32
            default    : op2sel = regv;
        endcase
    end endfunction

    wire [31:0] a2_0 = op2sel(imt0, gr_r2_0, syl0, syl1,  narrow_imm(op0,dst0));
    wire [31:0] a2_1 = op2sel(imt1, gr_r2_1, syl1, syl2,  narrow_imm(op1,dst1));
    wire [31:0] a2_2 = op2sel(imt2, gr_r2_2, syl2, syl3,  narrow_imm(op2,dst2));
    wire [31:0] a2_3 = op2sel(imt3, gr_r2_3, syl3, 32'd0, narrow_imm(op3,dst3));

    // An extension syllable is data, not an operation: it must not write back.
    // Slot 0 already gates on alu0_v and slot 3 on the memory unit's decode, but
    // slots 1 and 2 write gr[dst] unconditionally, so gate all four explicitly.
    wire ext0 = (op0 == `OP_SYLFOLW), ext1 = (op1 == `OP_SYLFOLW);
    wire ext2 = (op2 == `OP_SYLFOLW), ext3 = (op3 == `OP_SYLFOLW);

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
    // The memory offset is the same 9-bit signed immediate field, so negative
    // displacements ("ldw $rD = -8[$rB]") address correctly (FIX F7a).
    wire [9:0] dmem_byte_addr = gr_r1_3[9:0] + {syl3[10], syl3[10:2]};
    wire [7:0] dmem_word_addr = dmem_byte_addr[9:2];
    wire [1:0] dmem_pos       = dmem_byte_addr[1:0];
    wire [31:0] dmem_word     = dmem[dmem_word_addr];
    wire [31:0] mem_load, mem_store; wire mem_is_load, mem_is_store, mem_v;
    mem_unit u_mem(.opcode(op3),.mem_val(dmem_word),.reg_val(gr_r2_3),.pos(dmem_pos),
                   .load_data(mem_load),.store_data(mem_store),
                   .is_load(mem_is_load),.is_store(mem_is_store),.out_valid(mem_v));

    // ---- control unit (slot 0) ----
    wire [7:0]  c_pc_goto; wire [31:0] c_link; wire c_taken, c_wlink, c_xnop, c_rfi, c_v;
    wire [11:0] c_xnop_n;
    wire c_br_in = br[syl0[4:2]]; // BR/BRF read rb = syl[4:2]
    ctrl_unit u_ctrl(.opcode(op0),.pc(pc),.lr(gr_r1_0),.sp(gr[1]),.offset(off0),.br(c_br_in),
                     .pc_goto(c_pc_goto),.link_val(c_link),.taken(c_taken),.writes_link(c_wlink),
                     .is_xnop(c_xnop),.xnop_n(c_xnop_n),.is_rfi(c_rfi),.out_valid(c_v));

    wire slot0_is_ctrl = (op0[6:4] == 3'b010);
    wire slot0_is_stop = (op0 == `OP_STOP);

    // ---- per-slot GR result select (ALU vs MUL) ----
    wire [31:0] res1 = is_mul(op1) ? mul1_r : alu1_r;
    wire [31:0] res2 = is_mul(op2) ? mul2_r : alu2_r;

    // ================= sequential execution =================
    always @(posedge clk) begin
        if (reset) begin
            pc <= 8'd0; stall_cnt <= 12'd0; running <= 1'b0; done <= 1'b0; cycles <= 32'd0;
            for (i=0;i<64;i=i+1) gr[i] <= 32'd0;
            for (i=0;i<8;i=i+1)  br[i] <= 1'b0;
        end else begin
            if (start) running <= 1'b1;
            if (running && !done) begin
                cycles <= cycles + 1;
                if (stall_cnt != 0) begin
                    stall_cnt <= stall_cnt - 1;         // XNOP bubble
                end else if (slot0_is_stop) begin
                    done <= 1'b1;                        // STOP
                end else begin
                    //--------------- writeback slot 0 ---------------
                    if (ext0) ;                              // immediate extension
                    else if (slot0_is_ctrl) begin
                        if (c_wlink && dst0 != 6'd0) gr[dst0] <= c_link;   // CALL/RETURN link
                    end else if (op0 == `ALU_MTB) begin
                        br[dbr0] <= alu0_c;
                    end else if (is_addcg_divs(op0)) begin
                        if (dst0 != 6'd0) gr[dst0] <= alu0_r; br[dbr0] <= alu0_c;
                    end else if (is_cmp(op0) && dst0 == 6'd0) begin
                        br[dbr0] <= alu0_r[0];
                    end else if (op0 != `OP_NOP && alu0_v && dst0 != 6'd0) begin
                        gr[dst0] <= alu0_r;
                    end

                    //--------------- writeback slot 1 ---------------
                    if (ext1) ;                              // immediate extension
                    else if (op1 == `ALU_MTB) br[dbr1] <= alu1_c;
                    else if (is_addcg_divs(op1)) begin
                        if (dst1 != 6'd0) gr[dst1] <= alu1_r; br[dbr1] <= alu1_c;
                    end else if (is_cmp(op1) && dst1 == 6'd0) br[dbr1] <= alu1_r[0];
                    else if (op1 != `OP_NOP && dst1 != 6'd0) gr[dst1] <= res1;

                    //--------------- writeback slot 2 ---------------
                    if (ext2) ;                              // immediate extension
                    else if (op2 == `ALU_MTB) br[dbr2] <= alu2_c;
                    else if (is_addcg_divs(op2)) begin
                        if (dst2 != 6'd0) gr[dst2] <= alu2_r; br[dbr2] <= alu2_c;
                    end else if (is_cmp(op2) && dst2 == 6'd0) br[dbr2] <= alu2_r[0];
                    else if (op2 != `OP_NOP && dst2 != 6'd0) gr[dst2] <= res2;

                    //--------------- writeback slot 3 (ALU or MEM) ---------------
                    if (ext3) ;                              // immediate extension
                    else if (op3[6:4] == 3'b001) begin      // MEM range
                        if (mem_is_store) dmem[dmem_word_addr] <= mem_store;
                        else if (mem_is_load && dst3 != 6'd0) gr[dst3] <= mem_load;
                        // PFT / SYL_FOLLOW -> no writeback
                    end else if (op3 == `ALU_MTB) br[dbr3] <= alu3_c;
                    else if (is_addcg_divs(op3)) begin
                        if (dst3 != 6'd0) gr[dst3] <= alu3_r; br[dbr3] <= alu3_c;
                    end else if (is_cmp(op3) && dst3 == 6'd0) br[dbr3] <= alu3_r[0];
                    else if (op3 != `OP_NOP && dst3 != 6'd0) gr[dst3] <= alu3_r;

                    //--------------- PC / stall update ---------------
                    if (slot0_is_ctrl && c_xnop) begin
                        stall_cnt <= c_xnop_n;              // NEW: XNOP stall n cycles
                        pc <= pc + 8'd1;
                    end else if (slot0_is_ctrl && c_taken) begin
                        pc <= c_pc_goto;                    // branch / call / return
                    end else begin
                        pc <= pc + 8'd1;
                    end
                end
            end
            gr[0] <= 32'd0; // r0 hardwired
        end
    end
endmodule
