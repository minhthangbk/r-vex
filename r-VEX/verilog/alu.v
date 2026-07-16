//======================================================================
// r-VEX | ALU  (Verilog port of alu.vhd + alu_operations.vhd)
//----------------------------------------------------------------------
// Combinational execution unit. Faithful to the VHDL reference:
//   - SUB returns s1 - s2 (as in f_SUB)
//   - SHR is arithmetic (signed), SHRU is logical
//   - compares return 32'd1 / 32'd0
//   - MTB moves s1[0] onto cout (BR write)
//   - ADDCG / DIVS produce a carry-out on cout
//   - SLCT/SLCTF select on cin (branch operand b)
// out_valid = 0 for an unknown opcode (mirrors the VHDL fall-through).
//======================================================================
`include "rvex_defs.vh"

module alu (
    input  wire [6:0]  aluop,
    input  wire [31:0] src1,
    input  wire [31:0] src2,
    input  wire        cin,      // carry / branch operand
    output reg  [31:0] result,
    output reg         cout,
    output reg         out_valid
);
    wire [4:0]  sh = src2[4:0];
    reg  [32:0] addcg;
    // division-step temporaries (mirror f_DIVS)
    reg  [31:0] divs_shift;

    always @(*) begin
        result    = 32'd0;
        cout      = 1'b0;
        out_valid = 1'b1;
        addcg     = 33'd0;
        divs_shift= 32'd0;

        // ADDCG / DIVS / SLCT / SLCTF matched on the top nibble
        if (aluop[6:3] == `ALU_ADDCG4) begin
            addcg  = {1'b0, src1} + {1'b0, src2} + {32'b0, cin};
            result = addcg[31:0];
            cout   = addcg[32];
        end
        else if (aluop[6:3] == `ALU_DIVS4) begin
            divs_shift = {src1[30:0], 1'b0} + {31'b0, cin}; // (s1<<1)+ci
            if (src1[31]) result = divs_shift + src2;
            else          result = divs_shift - src2;
            cout   = src1[31];
        end
        else if (aluop[6:3] == `ALU_SLCT4)  result = cin ? src1 : src2;
        else if (aluop[6:3] == `ALU_SLCTF4) result = cin ? src2 : src1;
        else begin
            case (aluop)
                `ALU_ADD  : result = src1 + src2;
                `ALU_AND  : result = src1 & src2;
                `ALU_ANDC : result = (~src1) & src2;
                `ALU_OR   : result = src1 | src2;
                `ALU_ORC  : result = (~src1) | src2;
                `ALU_XOR  : result = src1 ^ src2;
                `ALU_MAX  : result = ($signed(src1)   >= $signed(src2))   ? src1 : src2;
                `ALU_MAXU : result = (src1            >= src2)            ? src1 : src2;
                `ALU_MIN  : result = ($signed(src1)   <= $signed(src2))   ? src1 : src2;
                `ALU_MINU : result = (src1            <= src2)            ? src1 : src2;
                `ALU_SH1ADD: result = (src1 << 1) + src2;
                `ALU_SH2ADD: result = (src1 << 2) + src2;
                `ALU_SH3ADD: result = (src1 << 3) + src2;
                `ALU_SH4ADD: result = (src1 << 4) + src2;
                `ALU_SHL  : result = src1 << sh;
                `ALU_SHR  : result = $signed(src1) >>> sh;   // arithmetic
                `ALU_SHRU : result = src1 >> sh;             // logical
                `ALU_SUB  : result = src1 - src2;
                `ALU_SXTB : result = {{24{src1[7]}},  src1[7:0]};
                `ALU_SXTH : result = {{16{src1[15]}}, src1[15:0]};
                `ALU_ZXTB : result = src1 & 32'h0000_00FF;
                `ALU_ZXTH : result = src1 & 32'h0000_FFFF;
                `ALU_MOV  : result = src1;
                `ALU_CMPEQ : result = (src1 == src2) ? 32'd1 : 32'd0;
                `ALU_CMPNE : result = (src1 != src2) ? 32'd1 : 32'd0;
                `ALU_CMPGE : result = ($signed(src1) >= $signed(src2)) ? 32'd1 : 32'd0;
                `ALU_CMPGEU: result = (src1 >= src2)                   ? 32'd1 : 32'd0;
                `ALU_CMPGT : result = ($signed(src1) >  $signed(src2)) ? 32'd1 : 32'd0;
                `ALU_CMPGTU: result = (src1 >  src2)                   ? 32'd1 : 32'd0;
                `ALU_CMPLE : result = ($signed(src1) <= $signed(src2)) ? 32'd1 : 32'd0;
                `ALU_CMPLEU: result = (src1 <= src2)                   ? 32'd1 : 32'd0;
                `ALU_CMPLT : result = ($signed(src1) <  $signed(src2)) ? 32'd1 : 32'd0;
                `ALU_CMPLTU: result = (src1 <  src2)                   ? 32'd1 : 32'd0;
                `ALU_NANDL : result = ((src1 == 0) || (src2 == 0)) ? 32'd1 : 32'd0;
                `ALU_NORL  : result = ((src1 == 0) && (src2 == 0)) ? 32'd1 : 32'd0;
                `ALU_ORL   : result = ((src1 == 0) && (src2 == 0)) ? 32'd0 : 32'd1;
                `ALU_ANDL  : result = ((src1 == 0) || (src2 == 0)) ? 32'd0 : 32'd1;
                `ALU_MTB   : begin result = 32'd0; cout = src1[0]; end
                default    : begin result = 32'd0; out_valid = 1'b0; end
            endcase
        end
    end
endmodule
