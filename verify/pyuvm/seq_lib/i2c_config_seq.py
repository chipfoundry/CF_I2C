"""I2C configuration sequence — sets up prescaler, clock gate, and interrupt mask."""

from pyuvm import uvm_sequence, ConfigDB

from cf_verify.bus_env.bus_seq_lib import write_reg_seq, reset_seq


class i2c_config_seq(uvm_sequence):
    def __init__(self, name="i2c_config_seq", prescaler=None, im=None):
        super().__init__(name)
        self.prescaler = prescaler
        self.im = im

    async def body(self):
        await reset_seq("rst").start(self.sequencer)
        regs = ConfigDB().get(None, "", "bus_regs")
        addr = regs.reg_name_to_address

        # Enable clock gate
        if "GCLK" in addr:
            await write_reg_seq("wr_gclk", addr["GCLK"], 1).start(self.sequencer)

        # Set prescaler: prescale = Fclk / (FI2Cclk * 4)
        pr = self.prescaler if self.prescaler is not None else 49
        await write_reg_seq("wr_pr", addr["PR"], pr).start(self.sequencer)

        # Set interrupt mask
        im_val = self.im if self.im is not None else 0x1FF
        if "IM" in addr:
            await write_reg_seq("wr_im", addr["IM"], im_val).start(self.sequencer)
