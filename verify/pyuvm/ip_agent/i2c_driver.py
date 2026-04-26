"""I2C IP driver — minimal driver since the M24AA64 EEPROM model handles slave-side.

The I2C master is driven via bus register writes; this driver exists to
satisfy the ip_agent framework requirement and can be extended for
injecting bus-level stimulus if needed.
"""

import cocotb
from cocotb.triggers import ClockCycles, FallingEdge, First
from pyuvm import uvm_driver, ConfigDB

from ip_item.i2c_item import i2c_item


class i2c_driver(uvm_driver):
    def build_phase(self):
        super().build_phase()
        self.dut = ConfigDB().get(self, "", "DUT")

    async def run_phase(self):
        while True:
            item = await self.seq_item_port.get_next_item()
            self.logger.info(f"I2C driver got item: {item.convert2string()}")
            await ClockCycles(self.dut.CLK, 1)
            self.seq_item_port.item_done()
