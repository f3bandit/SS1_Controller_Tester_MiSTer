#!/bin/sh
# Simulates the controller reader against two modelled controllers (needs Icarus Verilog).
cd "$(dirname "$0")/.." && iverilog -g2012 -o sim/tb_pad.vvp sim/tb_pad.sv rtl/sio_byte.sv rtl/card_sector.sv rtl/pad_ctrl.sv && stdbuf -o0 vvp -n sim/tb_pad.vvp
