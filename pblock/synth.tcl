set src /home/lr587/systolic4x4/systolic4x4.srcs/sources_1/new/systolic4x4.v
read_verilog $src
synth_design -top systolic4x4 -part xcv80-lsva4737-2MHP-e-S -mode out_of_context
create_clock -name clk -period 1.818 [get_ports clk]
write_checkpoint -force synth.dcp
report_utilization -file synth_util.rpt
report_utilization -hierarchical -file synth_util_hier.rpt
