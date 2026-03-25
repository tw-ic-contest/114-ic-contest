
analyze -format verilog top.v
elaborate top
link
current_design top

create_clock -period 50 [get_ports CLK] 
set_dont_touch_network [get_clocks CLK]

compile_ultra

write -format verilog -hierarchy -output top_syn.v
write_sdf -version 1.0 top_syn.sdf
write -format ddc -hierarchy -output top_syn.ddc

report_area > area.log
report_timing > timing.log
exit