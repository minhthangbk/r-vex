//======================================================================
// r-VEX | Opcode & constant definitions (Verilog port of r-vex_pkg.vhd)
//----------------------------------------------------------------------
// Ported from src/r-vex_pkg.vhd. 7-bit opcode = syllable[31:25].
// Opcodes with trailing "---" in the VHDL (ADDCG/DIVS/SLCT/SLCTF) are
// matched on their high bits because the low 3 bits carry a BR index.
//======================================================================
`ifndef RVEX_DEFS_VH
`define RVEX_DEFS_VH

// ---- register / memory geometry ----
`define GR_DEPTH     32
`define BR_DEPTH     8
`define DMEM_DEPTH   256
`define DMEM_LOGDEP  8

// ---- immediate types (syllable[24:23]) ----
`define NO_IMM      2'b00
`define SHORT_IMM   2'b01
`define BRANCH_IMM  2'b10
`define LONG_IMM    2'b11

// ---- writeback targets ----
`define WRITE_NOP   3'b000
`define WRITE_G     3'b001
`define WRITE_B     3'b010
`define WRITE_G_B   3'b011
`define WRITE_P     3'b100
`define WRITE_P_G   3'b101
`define WRITE_M     3'b110
`define WRITE_MG    3'b111

// ---- misc opcodes ----
`define OP_STOP     7'b0011111
`define OP_NOP      7'b0000000
`define OP_SYLFOLW  7'b0011100
`define OP_SEND     7'b0101010
`define OP_RECV     7'b0101011

// ---- ALU opcodes ----
`define ALU_ADD     7'b1000001
`define ALU_AND     7'b1000011
`define ALU_ANDC    7'b1000100
`define ALU_MAX     7'b1000101
`define ALU_MAXU    7'b1000110
`define ALU_MIN     7'b1000111
`define ALU_MINU    7'b1001000
`define ALU_OR      7'b1001001
`define ALU_ORC     7'b1001010
`define ALU_SH1ADD  7'b1001011
`define ALU_SH2ADD  7'b1001100
`define ALU_SH3ADD  7'b1001101
`define ALU_SH4ADD  7'b1001110
`define ALU_SHL     7'b1001111
`define ALU_SHR     7'b1010000
`define ALU_SHRU    7'b1010001
`define ALU_SUB     7'b1010010
`define ALU_SXTB    7'b1010011
`define ALU_SXTH    7'b1010100
`define ALU_ZXTB    7'b1010101
`define ALU_ZXTH    7'b1010110
`define ALU_XOR     7'b1010111
`define ALU_MOV     7'b1011000
`define ALU_CMPEQ   7'b1011001
`define ALU_CMPGE   7'b1011010
`define ALU_CMPGEU  7'b1011011
`define ALU_CMPGT   7'b1011100
`define ALU_CMPGTU  7'b1011101
`define ALU_CMPLE   7'b1011110
`define ALU_CMPLEU  7'b1011111
`define ALU_CMPLT   7'b1100000
`define ALU_CMPLTU  7'b1100001
`define ALU_CMPNE   7'b1100010
`define ALU_NANDL   7'b1100011
`define ALU_NORL    7'b1100100
`define ALU_ORL     7'b1100110
`define ALU_MTB     7'b1100111
`define ALU_ANDL    7'b1101000
// high-nibble matched (low 3 bits = BR index)
`define ALU_ADDCG4  4'b1111
`define ALU_DIVS4   4'b1110
`define ALU_SLCT4   4'b0111
`define ALU_SLCTF4  4'b0110
// full 7-bit convenience encodings (low 3 bits = BR index, here 0)
`define ALU_ADDCG   7'b1111000
`define ALU_DIVS    7'b1110000
`define ALU_SLCT    7'b0111000
`define ALU_SLCTF   7'b0110000

// ---- MUL opcodes ----
`define MUL_MPYLL   7'b0000001
`define MUL_MPYLLU  7'b0000010
`define MUL_MPYLH   7'b0000011
`define MUL_MPYLHU  7'b0000100
`define MUL_MPYHH   7'b0000101
`define MUL_MPYHHU  7'b0000110
`define MUL_MPYL    7'b0000111
`define MUL_MPYLU   7'b0001000
`define MUL_MPYH    7'b0001001
`define MUL_MPYHU   7'b0001010
`define MUL_MPYHS   7'b0001011

// ---- MEM opcodes ----
`define MEM_LDW     7'b0010001
`define MEM_LDH     7'b0010010
`define MEM_LDHU    7'b0010011
`define MEM_LDB     7'b0010100
`define MEM_LDBU    7'b0010101
`define MEM_STW     7'b0010110
`define MEM_STH     7'b0010111
`define MEM_STB     7'b0011000
`define MEM_PFT     7'b0011001

// ---- CTRL opcodes ----
`define CTRL_GOTO   7'b0100001
`define CTRL_IGOTO  7'b0100010
`define CTRL_CALL   7'b0100011
`define CTRL_ICALL  7'b0100100
`define CTRL_BR     7'b0100101
`define CTRL_BRF    7'b0100110
`define CTRL_RETURN 7'b0100111
`define CTRL_RFI    7'b0101000
`define CTRL_XNOP   7'b0101001

`endif
