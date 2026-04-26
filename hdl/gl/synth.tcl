yosys -import
read_verilog -lib pdk_file.v
read_verilog -DUSE_POWER_PINS bus_wrappers/CF_I2C_APB.pp.v axis_fifo.v i2c_master_wbs_16.v i2c_master.v
synth -top CF_I2C_APB
flatten
write_verilog -noattr CF_I2C_APB_synth.v