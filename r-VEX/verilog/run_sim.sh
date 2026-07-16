#!/usr/bin/env bash
# Build + run the r-VEX Verilog simulation with Icarus Verilog.
# Requires iverilog/vvp on PATH (installed at F:\APPs\iverilog).
set -e
export PATH="/f/APPs/iverilog/bin:$PATH"
cd "$(dirname "$0")"

SRC="rvex_defs.vh alu.v mul.v mem_unit.v ctrl_unit.v rvex_core.v"

echo "### Building unit testbench ###"
iverilog -g2012 -I . -o tb_units.vvp rvex_defs.vh alu.v mul.v mem_unit.v tb_units.v
echo "### Running unit testbench ###"
vvp tb_units.vvp

echo
echo "### Building core testbench ###"
iverilog -g2012 -I . -o tb_core.vvp $SRC tb_core.v
echo "### Running core testbench ###"
vvp tb_core.vvp
