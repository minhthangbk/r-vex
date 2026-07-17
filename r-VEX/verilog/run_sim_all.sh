#!/usr/bin/env bash
# Build + run the FULL r-VEX Verilog test suite with Icarus Verilog.
# Requires iverilog/vvp on PATH (installed at F:\APPs\iverilog).
#
# Suite:
#   tb_units      original datapath sanity (bug-fix proofs)
#   tb_alu_full   exhaustive ALU opcode coverage
#   tb_mul_full   MUL incl. saturation / overflow flag
#   tb_mem_full   all load/store byte-half positions
#   tb_ctrl       ctrl_unit branch/link directed tests
#   tb_core       original integrated core program
#   tb_core_prog  2nd core program (branch/CALL/RETURN/SLCT/VLIW)
set -e
export PATH="/f/APPs/iverilog/bin:$PATH"
cd "$(dirname "$0")"

CORE="rvex_defs.vh alu.v mul.v mem_unit.v ctrl_unit.v rvex_core.v"
LOG="sim_results_all.log"
: > "$LOG"

run() {   # name  "<sources>"  <top.v>
    local name="$1"; local srcs="$2"; local tb="$3"
    echo "### Building $name ###" | tee -a "$LOG"
    iverilog -g2012 -I . -o "$name.vvp" $srcs "$tb"
    echo "### Running  $name ###" | tee -a "$LOG"
    vvp "$name.vvp" | tee -a "$LOG"
    echo | tee -a "$LOG"
}

run tb_units     "rvex_defs.vh alu.v mul.v mem_unit.v"        tb_units.v
run tb_alu_full  "rvex_defs.vh alu.v"                         tb_alu_full.v
run tb_mul_full  "rvex_defs.vh mul.v"                         tb_mul_full.v
run tb_mem_full  "rvex_defs.vh mem_unit.v"                    tb_mem_full.v
run tb_ctrl      "rvex_defs.vh ctrl_unit.v"                   tb_ctrl.v
run tb_core      "$CORE"                                      tb_core.v
run tb_core_prog "$CORE"                                      tb_core_prog.v

echo "================= SUITE SUMMARY =================" | tee -a "$LOG"
grep -E 'RESULT:' "$LOG" | tee -a "$LOG"
# non-zero exit if any suite reported a failure/mismatch
if grep -Eq 'FAIL|FAILED|FAILURES' "$LOG"; then
    echo "OVERALL: FAILURES PRESENT" | tee -a "$LOG"; exit 1
else
    echo "OVERALL: ALL SUITES PASSED" | tee -a "$LOG"
fi
