//======================================================================
// r-VEX | Memory formatting unit (Verilog port of mem.vhd + mem_operations)
//----------------------------------------------------------------------
// BUG FIXES vs the VHDL reference (see review):
//   * LDH : now SIGN-extends the 16-bit field ({{16{h[15]}},h})
//           (VHDL f_LDH wrongly zero-extended -> "same as LDHU")
//   * LDB : now SIGN-extends the 8-bit field  ({{24{b[7]}},b})
//           (VHDL f_LDB wrongly zero-extended -> "same as LDBU")
// NEW:
//   * PFT : prefetch is now decoded as an explicit valid no-op
//           (VHDL fell through "when others" -> silently ignored)
//
// pos selects the sub-field inside the 32-bit word, exactly as in the VHDL:
//   halfword: 00->[31:16] 01->[23:8] 10->[15:0]
//   byte    : 00->[31:24] 01->[23:16] 10->[15:8] 11->[7:0]
//======================================================================
`include "rvex_defs.vh"

module mem_unit (
    input  wire [6:0]  opcode,
    input  wire [31:0] mem_val,   // word currently in data memory at the address
    input  wire [31:0] reg_val,   // register data to store
    input  wire [1:0]  pos,       // byte/half offset in the word
    output reg  [31:0] load_data, // value to write into a GR (loads)
    output reg  [31:0] store_data, // value to write into data memory (stores)
    output reg         is_load,
    output reg         is_store,
    output reg         out_valid
);
    reg [15:0] h;   // extracted halfword
    reg [7:0]  b;   // extracted byte

    always @(*) begin
        load_data  = 32'd0;
        store_data = 32'd0;
        is_load    = 1'b0;
        is_store   = 1'b0;
        out_valid  = 1'b1;
        h = 16'd0;
        b = 8'd0;

        // halfword field select
        case (pos)
            2'b00: h = mem_val[31:16];
            2'b01: h = mem_val[23:8];
            2'b10: h = mem_val[15:0];
            default: h = 16'hFFFF;      // not allowed (mirrors VHDL)
        endcase
        // byte field select
        case (pos)
            2'b00: b = mem_val[31:24];
            2'b01: b = mem_val[23:16];
            2'b10: b = mem_val[15:8];
            default: b = mem_val[7:0];
        endcase

        case (opcode)
            `MEM_LDW : begin is_load = 1'b1; load_data = mem_val; end
            `MEM_LDH : begin is_load = 1'b1; load_data = {{16{h[15]}}, h}; end // FIX: sign-extend
            `MEM_LDHU: begin is_load = 1'b1; load_data = {16'b0,      h}; end
            `MEM_LDB : begin is_load = 1'b1; load_data = {{24{b[7]}},  b}; end // FIX: sign-extend
            `MEM_LDBU: begin is_load = 1'b1; load_data = {24'b0,       b}; end

            `MEM_STW : begin is_store = 1'b1; store_data = reg_val; end
            `MEM_STH : begin
                is_store = 1'b1;
                case (pos)
                    2'b00: store_data = {reg_val[15:0], mem_val[15:0]};
                    2'b01: store_data = {mem_val[31:24], reg_val[15:0], mem_val[7:0]};
                    2'b10: store_data = {mem_val[31:16], reg_val[15:0]};
                    default: store_data = 32'hFFFF_FFFF;
                endcase
            end
            `MEM_STB : begin
                is_store = 1'b1;
                case (pos)
                    2'b00: store_data = {reg_val[7:0],  mem_val[23:0]};
                    2'b01: store_data = {mem_val[31:24], reg_val[7:0], mem_val[15:0]};
                    2'b10: store_data = {mem_val[31:16], reg_val[7:0], mem_val[7:0]};
                    default: store_data = {mem_val[31:8], reg_val[7:0]};
                endcase
            end

            `MEM_PFT : begin /* NEW: prefetch = valid architectural no-op */ end

            default  : out_valid = 1'b0;
        endcase
    end
endmodule
