"""I2C write-then-read sequence — writes data to EEPROM, reads back, and verifies."""

import random

from cocotb.triggers import ClockCycles, Timer
from pyuvm import uvm_sequence, ConfigDB, uvm_root

from cf_verify.bus_env.bus_seq_lib import write_reg_seq, read_reg_seq, reset_seq
from seq_lib.i2c_config_seq import i2c_config_seq


SLAVE_ADDR = 0x50


class i2c_write_read_seq(uvm_sequence):
    def __init__(self, name="i2c_write_read_seq", prescaler=None):
        super().__init__(name)
        self.prescaler = prescaler

    async def body(self):
        await reset_seq("rst").start(self.sequencer)
        regs = ConfigDB().get(None, "", "bus_regs")
        addr = regs.reg_name_to_address
        dut = ConfigDB().get(None, "", "DUT")

        config = i2c_config_seq("config", prescaler=self.prescaler)
        await config.start(self.sequencer)

        mem_addr = random.randint(0, 0x1F00)
        write_data = [random.randint(0, 0xFF) for _ in range(random.randint(1, 4))]

        # WRITE PHASE: write data to EEPROM
        # Address bytes
        await write_reg_seq("wr_data", addr["Data"],
                            (mem_addr >> 8) & 0xFF).start(self.sequencer)
        await write_reg_seq("wr_data", addr["Data"],
                            mem_addr & 0xFF).start(self.sequencer)
        # Data bytes
        for i, byte in enumerate(write_data):
            last = (1 << 9) if i == len(write_data) - 1 else 0
            await write_reg_seq("wr_data", addr["Data"],
                                byte | last).start(self.sequencer)
        # Write command
        cmd_wr = (SLAVE_ADDR & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
        await write_reg_seq("wr_cmd", addr["Command"], cmd_wr).start(self.sequencer)

        await self._wait_idle(dut, regs, addr)

        # EEPROM write cycle time (~5ms real, but in sim we wait clock cycles)
        await ClockCycles(dut.CLK, 5000)

        # READ PHASE: set address then read back
        await write_reg_seq("wr_data", addr["Data"],
                            (mem_addr >> 8) & 0xFF).start(self.sequencer)
        await write_reg_seq("wr_data", addr["Data"],
                            (mem_addr & 0xFF) | (1 << 9)).start(self.sequencer)
        cmd_addr = (SLAVE_ADDR & 0x7F) | (1 << 8) | (1 << 11)
        await write_reg_seq("wr_cmd", addr["Command"], cmd_addr).start(self.sequencer)

        await self._wait_idle(dut, regs, addr)

        # Read commands: one per byte, start on first, stop on last
        for i in range(len(write_data)):
            start = (1 << 8) if i == 0 else 0
            stop = (1 << 12) if i == len(write_data) - 1 else 0
            cmd_rd = (SLAVE_ADDR & 0x7F) | start | (1 << 9) | stop
            await write_reg_seq("rd_cmd", addr["Command"], cmd_rd).start(self.sequencer)

        await self._wait_idle(dut, regs, addr)

        # Use read_reg_seq result (per-transfer bus data). Verilator can leave
        # the register mirror stale across consecutive reads of Data.
        read_data = []
        for i in range(len(write_data)):
            rd = read_reg_seq(f"rd_data_{i}", addr["Data"])
            await rd.start(self.sequencer)
            val = int(rd.result)
            assert val & (1 << 8), f"read Data not valid: 0x{val:04x}"
            read_data.append(val & 0xFF)

        assert read_data == write_data, (
            f"I2C write/read MISMATCH: addr=0x{mem_addr:04x} "
            f"wrote={[hex(d) for d in write_data]} "
            f"read={[hex(d) for d in read_data]}"
        )

    async def _wait_idle(self, dut, regs, addr):
        for _ in range(5000):
            await ClockCycles(dut.CLK, 10)
            await read_reg_seq("rd_status", addr["Status"]).start(self.sequencer)
            status = regs.read_reg_value("Status")
            if not (status & 0x1):
                return
