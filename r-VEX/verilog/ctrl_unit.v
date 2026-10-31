//======================================================================
// r-VEX | Control unit (Verilog port of ctrl.vhd + ctrl_operations.vhd)
//----------------------------------------------------------------------
// Combinational branch-target / link computation for slot 0.
// NEW: XNOP is decoded to a cycle count (is_xnop + xnop_n) so the core
//      can stall the issue stage n cycles (VHDL only zeroed pc_goto).
// PARTIAL: RFI is mapped to RETURN semantics (no interrupt context save/
//          restore exists in this core) and flagged via is_rfi.
//======================================================================
`include "rvex_defs.vh"

module ctrl_unit #(parameter PC_W = 12) (
    input  wire [6:0]  opcode,
    input  wire [PC_W-1:0] pc,
    input  wire [31:0] lr,        // link register ($r0.63)
    input  wire [31:0] sp,        // stack pointer ($r0.1)
    input  wire [11:0] offset,    // branch offset immediate (or lr offset)
    input  wire        br,        // branch register value
    output reg  [PC_W-1:0] pc_goto, // next PC when taken
    output reg  [31:0] link_val,  // value written to lr / sp
    output reg         taken,     // '1' -> write PC with pc_goto
    output reg         writes_link,// '1' -> link_val is written back
    output reg         is_xnop,
    output reg  [11:0] xnop_n,
    output reg         is_rfi,
    output reg         out_valid
);
    always @(*) begin
        pc_goto     = pc + 1'b1;
        link_val    = 32'd0;
        taken       = 1'b0;
        writes_link = 1'b0;
        is_xnop     = 1'b0;
        xnop_n      = 12'd0;
        is_rfi      = 1'b0;
        out_valid   = 1'b1;

        case (opcode)
            `CTRL_GOTO : begin taken = 1'b1; pc_goto = offset[PC_W-1:0]; end
            `CTRL_IGOTO: begin taken = 1'b1; pc_goto = lr[PC_W-1:0]; end
            `CTRL_CALL : begin taken = 1'b1; pc_goto = offset[PC_W-1:0]; link_val = {{(32-PC_W){1'b0}}, pc} + 32'd1; writes_link = 1'b1; end
            `CTRL_ICALL: begin taken = 1'b1; pc_goto = lr[PC_W-1:0];    link_val = {{(32-PC_W){1'b0}}, pc} + 32'd1; writes_link = 1'b1; end
            `CTRL_BR   : begin taken = br;   pc_goto = br  ? offset[PC_W-1:0] : pc + 1'b1; end
            `CTRL_BRF  : begin taken = ~br;  pc_goto = ~br ? offset[PC_W-1:0] : pc + 1'b1; end
            `CTRL_RETURN: begin taken = 1'b1; pc_goto = lr[PC_W-1:0]; link_val = sp + {20'b0, offset}; writes_link = 1'b1; end
            `CTRL_RFI  : begin taken = 1'b1; pc_goto = lr[PC_W-1:0]; link_val = sp + {20'b0, offset}; writes_link = 1'b1; is_rfi = 1'b1; end
            `CTRL_XNOP : begin is_xnop = 1'b1; xnop_n = offset; end
            `OP_NOP    : begin /* no branch */ end
            default    : out_valid = 1'b0;
        endcase
    end
endmodule
