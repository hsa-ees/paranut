#!/bin/bash
ghdl -a ../i2s_serializer.vhd i2s_serializer_tb.vhd
ghdl -r i2s_serializer_tb --wave=i2s_serializer_tb.ghw
gtkwave i2s_serializer_tb.ghw i2s_serializer_tb.gtkw
