"""I2C write sequence — writes data bytes to an I2C slave (EEPROM)."""

import random

from cocotb.triggers import ClockCycles
from pyuvm import uvm_sequence, ConfigDB

from cf_verify.bus_env.bus_seq_lib import write_reg_seq, read_reg_seq, reset_seq
from seq_lib.i2c_config_seq import i2c_config_seq


SLAVE_ADDR = 0x50

STATUS_OFFSET = 0x0
CMD_OFFSET = 0x4
DATA_OFFSET = 0x8


class i2c_write_seq(uvm_sequence):
    def __init__(self, name="i2c_write_seq", slave_addr=SLAVE_ADDR,
                 mem_addr=0x00, data=None, prescaler=None):
        super().__init__(name)
        self.slave_addr = slave_addr
        self.mem_addr = mem_addr
        self.data = data
        self.prescaler = prescaler

    async def body(self):
        await reset_seq("rst").start(self.sequencer)
        regs = ConfigDB().get(None, "", "bus_regs")
        addr = regs.reg_name_to_address
        dut = ConfigDB().get(None, "", "DUT")

        config = i2c_config_seq("config", prescaler=self.prescaler)
        await config.start(self.sequencer)

        if self.data is None:
            self.data = [random.randint(0, 0xFF) for _ in range(random.randint(1, 4))]

        # Write EEPROM address high byte
        await write_reg_seq("wr_data", addr["Data"],
                            (self.mem_addr >> 8) & 0xFF).start(self.sequencer)
        # Write EEPROM address low byte with data_last=0
        await write_reg_seq("wr_data", addr["Data"],
                            self.mem_addr & 0xFF).start(self.sequencer)

        # Write data bytes
        for i, byte in enumerate(self.data):
            last = (1 << 9) if i == len(self.data) - 1 else 0
            await write_reg_seq("wr_data", addr["Data"],
                                byte | last).start(self.sequencer)

        # Issue I2C write command: start + write_multiple + stop
        cmd = (self.slave_addr & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
        await write_reg_seq("wr_cmd", addr["Command"], cmd).start(self.sequencer)

        # Wait for completion
        await self._wait_idle(dut, regs, addr)

    async def _wait_idle(self, dut, regs, addr):
        for _ in range(5000):
            await ClockCycles(dut.CLK, 10)
            await read_reg_seq("rd_status", addr["Status"]).start(self.sequencer)
            status = regs.read_reg_value("Status")
            if not (status & 0x1):
                return
