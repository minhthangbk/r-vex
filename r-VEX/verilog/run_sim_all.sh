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
#   tb_alu_rand   constrained-random ALU vs reference model
#   tb_mul_rand   constrained-random MUL vs reference model
#   tb_core       original integrated core program
#   tb_core_prog  2nd core program (branch/CALL/RETURN/SLCT/VLIW)
#   tb_core_carry 3rd core program (ADDCG carry-chain + STH/STB->load)
#   tb_codex_stall external-port hold contract (I/D miss, XNOP, carry, STOP)
#   tb_piu        performance indicator unit, bare core (reviews/contracts/piu.md)
#   tb_piu_sys    PIU in rvex_system: misses, hw_sem, per-core PIUs
#   tb_cmpbr      compare to BR with an immediate (reviews/contracts/cmpbr.md)
#   tb_brnew      brn/brfn: branch on the BR written in its own packet (reviews/contracts/brnew.md)
#   tb_coherence  constrained-random MSI L1D/bus/L2 vs golden memory (4 seeds; + 6 seeds with DPF = 4)
#   tb_l1i_pf / tb_l1d_pf  cache prefetch, directed (reviews/contracts/prefetch.md)
#   tb_hw_sem     constrained-random hardware semaphores (4 seeds)
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
run tb_ctrl       "rvex_defs.vh ctrl_unit.v"                  tb_ctrl.v
run tb_alu_rand   "rvex_defs.vh alu.v"                        tb_alu_rand.v
run tb_mul_rand   "rvex_defs.vh mul.v"                        tb_mul_rand.v
run tb_core       "$CORE"                                     tb_core.v
run tb_core_prog  "$CORE"                                     tb_core_prog.v
run tb_core_carry "$CORE"                                     tb_core_carry.v
run tb_codex_stall "$CORE"                                     tb_codex_stall.v
run tb_piu        "$CORE"                                     tb_piu.v
run tb_piu_sys    "$CORE l1i_cache.v l1d_cache.v coh_bus.v l2_cache.v hw_sem.v rvex_system.v" tb_piu_sys.v
run tb_codex_piu_edges   "rvex_piu.v"                          tb_codex_piu_edges.v
run tb_codex_piu_core    "$CORE"                               tb_codex_piu_core.v
run tb_codex_r21_piuaw  "$CORE l1i_cache.v l1d_cache.v coh_bus.v l2_cache.v hw_sem.v rvex_system.v" tb_codex_r21_piuaw.v
run tb_codex_semfail     "$CORE l1i_cache.v l1d_cache.v coh_bus.v l2_cache.v hw_sem.v rvex_system.v" tb_codex_semfail.v
run tb_codex_piu_dualctrl "$CORE l1i_cache.v l1d_cache.v coh_bus.v l2_cache.v hw_sem.v rvex_system.v" tb_codex_piu_dualctrl.v
run tb_codex_piu_r11     "$CORE"                               tb_codex_piu_r11.v
run tb_verify_piu_pkt    "$CORE"                               tb_verify_piu_pkt.v
run tb_codex_cmpbr_oldbr "$CORE"                             tb_codex_cmpbr_oldbr.v
run tb_verify_cmpbr_pkt  "$CORE"                             tb_verify_cmpbr_pkt.v
run tb_brnew           "$CORE"                             tb_brnew.v
run tb_codex_brnew_edges "$CORE"                           tb_codex_brnew_edges.v
# cache prefetch (reviews/contracts/prefetch.md, r11)
run tb_l1i_pf          "l1i_cache.v"                       tb_l1i_pf.v
run tb_l1d_pf          "l1d_cache.v coh_bus.v l2_cache.v"  tb_l1d_pf.v
# final verify r11: prefetch corners inside MSI, 2 seeds x 2 shapes
iverilog -g2012 -I . -o tb_verify_r11_dpf.vvp l1d_cache.v coh_bus.v l2_cache.v tb_verify_r11_dpf.v
iverilog -g2012 -I . -Ptb_verify_r11_dpf.L1S=4 -Ptb_verify_r11_dpf.DEG=2 -o tb_verify_r11_dpf4.vvp l1d_cache.v coh_bus.v l2_cache.v tb_verify_r11_dpf.v
for seed in 1 2; do
    echo "### Running  tb_verify_r11_dpf seed $seed ###" | tee -a "$LOG"
    vvp tb_verify_r11_dpf.vvp +SEED=$seed | tee -a "$LOG"
    vvp tb_verify_r11_dpf4.vvp +SEED=$seed | tee -a "$LOG"
done
run tb_codex_r22_ipf_edges "l1i_cache.v"                  tb_codex_r22_ipf_edges.v
run tb_codex_r22_dpf_edges "l1d_cache.v coh_bus.v l2_cache.v" tb_codex_r22_dpf_edges.v
iverilog -g2012 -I . -Ptb_l1d_pf.DEG=1 -o tb_l1d_pf1.vvp l1d_cache.v coh_bus.v l2_cache.v tb_l1d_pf.v
echo "### Running  tb_l1d_pf DEG=1 ###" | tee -a "$LOG"
vvp tb_l1d_pf1.vvp | tee -a "$LOG"
# compare to BR with an immediate (reviews/contracts/cmpbr.md), 3 seeds
iverilog -g2012 -I . -o tb_cmpbr.vvp $CORE tb_cmpbr.v
for seed in 1 2 3; do
    echo "### Running  tb_cmpbr seed $seed ###" | tee -a "$LOG"
    vvp tb_cmpbr.vvp +SEED=$seed | tee -a "$LOG"
done
# MSI coherence: 2 x L1D + atomic snooping bus + L2, random traffic, 4 seeds
iverilog -g2012 -I . -o tb_coherence.vvp l1d_cache.v coh_bus.v l2_cache.v tb_coherence.v
for seed in 1 2 3 4; do
    echo "### Running  tb_coherence seed $seed ###" | tee -a "$LOG"
    vvp tb_coherence.vvp +SEED=$seed +CYCLES=50000 | tee -a "$LOG"
done
# the same with the L1D stream prefetch buffer (prefetch.md DPF-03): 4 sparse seeds (idle gaps
# for prefetches) and 2 dense seeds
iverilog -g2012 -I . -Ptb_coherence.DPF=4 -o tb_coherence_pf.vvp l1d_cache.v coh_bus.v l2_cache.v tb_coherence.v
for seed in 1 2 3 4; do
    echo "### Running  tb_coherence_pf seed $seed sparse ###" | tee -a "$LOG"
    vvp tb_coherence_pf.vvp +SEED=$seed +CYCLES=50000 +SPARSE | tee -a "$LOG"
done
for seed in 5 6; do
    echo "### Running  tb_coherence_pf seed $seed ###" | tee -a "$LOG"
    vvp tb_coherence_pf.vvp +SEED=$seed +CYCLES=50000 | tee -a "$LOG"
done
iverilog -g2012 -I . -Ptb_coherence.DPF=4 -Ptb_coherence.DPF_DEG=1 -o tb_coherence_pf1.vvp l1d_cache.v coh_bus.v l2_cache.v tb_coherence.v
for seed in 3 5 7 8; do
    echo "### Running  tb_coherence_pf DEG=1 seed $seed sparse ###" | tee -a "$LOG"
    vvp tb_coherence_pf1.vvp +SEED=$seed +CYCLES=50000 +SPARSE | tee -a "$LOG"
done

# hardware semaphores (hw_sem.v): mutual exclusion + non-atomic RMW + starvation, 4 seeds
iverilog -g2012 -I . -o tb_hw_sem.vvp hw_sem.v tb_hw_sem.v
for seed in 1 2 3 4; do
    echo "### Running  tb_hw_sem seed $seed ###" | tee -a "$LOG"
    vvp tb_hw_sem.vvp +SEED=$seed +CYCLES=50000 | tee -a "$LOG"
done

echo "================= SUITE SUMMARY =================" | tee -a "$LOG"
grep -E 'RESULT:' "$LOG" | tee -a "$LOG"
# non-zero exit if any suite reported a failure/mismatch
# whole words (-w): PIU-18 tests print SEMFAIL (R21-1)
if grep -Eqw 'FAIL|FAILED|FAILURES' "$LOG"; then
    echo "OVERALL: FAILURES PRESENT" | tee -a "$LOG"; exit 1
else
    echo "OVERALL: ALL SUITES PASSED" | tee -a "$LOG"
fi
