"""I2C read sequence — reads data bytes from an I2C slave (EEPROM)."""

from cocotb.triggers import ClockCycles
from pyuvm import uvm_sequence, ConfigDB

from cf_verify.bus_env.bus_seq_lib import write_reg_seq, read_reg_seq, reset_seq
from seq_lib.i2c_config_seq import i2c_config_seq


SLAVE_ADDR = 0x50

STATUS_OFFSET = 0x0
CMD_OFFSET = 0x4
DATA_OFFSET = 0x8


class i2c_read_seq(uvm_sequence):
    def __init__(self, name="i2c_read_seq", slave_addr=SLAVE_ADDR,
                 mem_addr=0x00, num_bytes=1, prescaler=None):
        super().__init__(name)
        self.slave_addr = slave_addr
        self.mem_addr = mem_addr
        self.num_bytes = num_bytes
        self.prescaler = prescaler
        self.read_data = []

    async def body(self):
        await reset_seq("rst").start(self.sequencer)
        regs = ConfigDB().get(None, "", "bus_regs")
        addr = regs.reg_name_to_address
        dut = ConfigDB().get(None, "", "DUT")

        config = i2c_config_seq("config", prescaler=self.prescaler)
        await config.start(self.sequencer)

        # First: write the memory address to the EEPROM (address phase)
        await write_reg_seq("wr_data", addr["Data"],
                            (self.mem_addr >> 8) & 0xFF).start(self.sequencer)
        await write_reg_seq("wr_data", addr["Data"],
                            (self.mem_addr & 0xFF) | (1 << 9)).start(self.sequencer)

        # Issue write command with start (no stop) to set address
        cmd_wr = (self.slave_addr & 0x7F) | (1 << 8) | (1 << 11)
        await write_reg_seq("wr_cmd", addr["Command"], cmd_wr).start(self.sequencer)

        await self._wait_idle(dut, regs, addr)

        # Now issue read command: start + read + stop
        cmd_rd = (self.slave_addr & 0x7F) | (1 << 8) | (1 << 9) | (1 << 12)
        await write_reg_seq("rd_cmd", addr["Command"], cmd_rd).start(self.sequencer)

        await self._wait_idle(dut, regs, addr)

        async def _wait_read_fifo():
            for _ in range(200_000):
                await read_reg_seq("rd_st_rx", addr["Status"]).start(self.sequencer)
                st = int(regs.read_reg_value("Status"))
                if not (st & (1 << 14)):
                    return
                await ClockCycles(dut.CLK, 3)
            assert False, "timeout waiting for I2C read data (rd_empty)"

        # Read data from FIFO (see i2c_write_read_seq for bus vs reg mirror)
        self.read_data = []
        for i in range(self.num_bytes):
            await _wait_read_fifo()
            rd = read_reg_seq(f"rd_data_{i}", addr["Data"])
            await rd.start(self.sequencer)
            v_bus = int(rd.result) if rd.result is not None else 0
            v_reg = int(regs.read_reg_value("Data"))
            if v_bus & (1 << 8):
                val = v_bus
            elif v_reg & (1 << 8):
                val = v_reg
            else:
                continue
            self.read_data.append(val & 0xFF)

    async def _wait_idle(self, dut, regs, addr):
        for _ in range(5000):
            await ClockCycles(dut.CLK, 10)
            await read_reg_seq("rd_status", addr["Status"]).start(self.sequencer)
            status = regs.read_reg_value("Status")
            if not (status & 0x1):
                return
