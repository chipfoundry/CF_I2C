"""I2C coverage closure — systematically hits all remaining coverage bins."""

from pyuvm import uvm_sequence, ConfigDB
from cocotb.triggers import ClockCycles

from cf_verify.bus_env.bus_seq_lib import write_reg_seq, read_reg_seq
from seq_lib.i2c_config_seq import i2c_config_seq


SLAVE_ADDR = 0x50


class i2c_coverage_closure_seq(uvm_sequence):
    async def body(self):
        regs = ConfigDB().get(None, "", "bus_regs")
        self.addr = regs.reg_name_to_address
        self.dut = ConfigDB().get(None, "", "DUT")

        if "GCLK" in self.addr:
            await self._w("gclk", "GCLK", 1)

        await self._prescaler_sweep()
        await self._data_value_sweep()
        await self._write_data_bins()
        await self._address_sweep()
        await self._command_bits()
        await self._status_and_irq_flags()

    async def _w(self, name, reg, val):
        await write_reg_seq(name, self.addr[reg], val).start(self.sequencer)

    async def _r(self, name, reg):
        await read_reg_seq(name, self.addr[reg]).start(self.sequencer)

    async def _wait_idle(self):
        for _ in range(5000):
            await ClockCycles(self.dut.CLK, 10)
            await self._r("status", "Status")
            regs = ConfigDB().get(None, "", "bus_regs")
            status = regs.read_reg_value("Status")
            if not (status & 0x1):
                return

    async def _do_write(self, slave_addr, mem_addr, data_bytes):
        """Write data to an I2C slave."""
        await self._w("data_hi", "Data", (mem_addr >> 8) & 0xFF)
        await self._w("data_lo", "Data", mem_addr & 0xFF)
        for i, b in enumerate(data_bytes):
            last = (1 << 9) if i == len(data_bytes) - 1 else 0
            await self._w("data_b", "Data", b | last)
        cmd = (slave_addr & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
        await self._w("cmd_wr", "Command", cmd)
        await self._r("cmd_rd", "Command")
        await self._wait_idle()
        await self._r("ris_wr", "RIS")

    async def _do_read(self, slave_addr, mem_addr, num_bytes):
        """Read data from an I2C slave (set address then read)."""
        await self._w("data_hi", "Data", (mem_addr >> 8) & 0xFF)
        await self._w("data_lo", "Data", (mem_addr & 0xFF) | (1 << 9))
        cmd_addr = (slave_addr & 0x7F) | (1 << 8) | (1 << 11)
        await self._w("cmd_addr", "Command", cmd_addr)
        await self._wait_idle()

        cmd_rd = (slave_addr & 0x7F) | (1 << 8) | (1 << 9) | (1 << 12)
        await self._w("cmd_rd", "Command", cmd_rd)
        await self._r("cmd_rd_back", "Command")
        await self._wait_idle()
        await self._r("ris_rd", "RIS")

        for _ in range(num_bytes):
            await self._r("data_rd", "Data")

    async def _prescaler_sweep(self):
        """Hit all 4 PR bins: (0-3), (4-15), (16-63), (64-255)."""
        for pr_val in [2, 10, 49, 99]:
            config = i2c_config_seq("config", prescaler=pr_val)
            await config.start(self.sequencer)
            await self._r("pr_rd", "PR")

            await self._do_write(SLAVE_ADDR, 0x0000, [0x42])
            await ClockCycles(self.dut.CLK, 5000)

    async def _data_value_sweep(self):
        """Hit all 8 WRITE.Data and READ.Data bins + reg.Data.data bins."""
        config = i2c_config_seq("config", prescaler=49)
        await config.start(self.sequencer)

        data_reps = [0x08, 0x28, 0x48, 0x68, 0x88, 0xA8, 0xC8, 0xE8]
        for i, d in enumerate(data_reps):
            mem_addr = 0x0100 + i * 0x10
            await self._do_write(SLAVE_ADDR, mem_addr, [d])
            await self._r("data_reg", "Data")
            await ClockCycles(self.dut.CLK, 5000)

        for i, d in enumerate(data_reps):
            mem_addr = 0x0100 + i * 0x10
            await self._do_read(SLAVE_ADDR, mem_addr, 1)

        # Write Data register directly with values in all 16 bins
        for i in range(16):
            val = i * (65536 // 16) + 1
            await self._w(f"data_bin{i}", "Data", val & 0x3FF)
            await self._r(f"data_rd{i}", "Data")

    async def _address_sweep(self):
        """Hit all 8 address bins and 16 cmd_address register bins.

        Only 0x50 has a real slave; the rest will NAK, which also covers
        the MISS_ACK flag.
        """
        config = i2c_config_seq("config", prescaler=49)
        await config.start(self.sequencer)

        # 8 address bins for i2c_item coverage: (0-15), (16-31), ..., (112-127)
        addr_reps = [0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78]
        for slave_a in addr_reps:
            await self._w("data_wr", "Data", 0x00 | (1 << 9))
            cmd = (slave_a & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
            await self._w("cmd", "Command", cmd)
            await self._r("cmd_rd", "Command")
            await self._wait_idle()
            await self._r("status_addr", "Status")
            await self._r("ris_addr", "RIS")
            await ClockCycles(self.dut.CLK, 500)

        # 16 cmd_address register bins for auto-coverage (7-bit field, 16 bins)
        # Each bin spans 8 addresses: (0-7), (8-15), ..., (120-127)
        for i in range(16):
            slave_a = i * 8 + 4  # mid-point of each bin
            if slave_a > 127:
                slave_a = 127
            await self._w("data_a", "Data", 0x00 | (1 << 9))
            cmd = (slave_a & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
            await self._w("cmd_a", "Command", cmd)
            await self._r("cmd_rd_a", "Command")
            await self._wait_idle()
            await self._r("ris_a", "RIS")
            await ClockCycles(self.dut.CLK, 500)

        # Read commands to different addresses
        for slave_a in addr_reps:
            cmd_rd = (slave_a & 0x7F) | (1 << 8) | (1 << 9) | (1 << 12)
            await self._w("cmd_rd_sweep", "Command", cmd_rd)
            await self._r("cmd_rd_sweep_rd", "Command")
            await self._wait_idle()
            await self._r("status_rdaddr", "Status")
            await self._r("ris_rdaddr", "RIS")
            await ClockCycles(self.dut.CLK, 500)

    async def _write_data_bins(self):
        """Hit all 8 WRITE.Data bins by using memory addresses with varied high bytes.

        The I2C monitor captures the first data byte after the address,
        which is the memory address high byte in EEPROM writes.
        """
        config = i2c_config_seq("config", prescaler=49)
        await config.start(self.sequencer)

        high_byte_reps = [0x08, 0x28, 0x48, 0x68, 0x88, 0xA8, 0xC8, 0xE8]
        for hb in high_byte_reps:
            mem_addr = (hb << 8) | 0x00
            await self._do_write(SLAVE_ADDR, mem_addr, [0x42])
            await ClockCycles(self.dut.CLK, 5000)

    async def _command_bits(self):
        """Exercise all Command register bits: start, read, write, write_multiple, stop."""
        config = i2c_config_seq("config", prescaler=49)
        await config.start(self.sequencer)

        # Write single byte with start+write+stop
        await self._w("data_single", "Data", 0x55 | (1 << 9))
        cmd_write = (SLAVE_ADDR & 0x7F) | (1 << 8) | (1 << 10) | (1 << 12)
        await self._w("cmd_write", "Command", cmd_write)
        await self._r("cmd_rd_write", "Command")
        await self._wait_idle()
        await ClockCycles(self.dut.CLK, 5000)

        # Read with start+read+stop
        cmd_read = (SLAVE_ADDR & 0x7F) | (1 << 8) | (1 << 9) | (1 << 12)
        await self._w("cmd_read", "Command", cmd_read)
        await self._r("cmd_rd_read", "Command")
        await self._wait_idle()
        await self._r("data_result", "Data")

        # Write_multiple with start+stop
        await self._w("data_m0", "Data", 0xAA)
        await self._w("data_m1", "Data", 0xBB | (1 << 9))
        cmd_wm = (SLAVE_ADDR & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
        await self._w("cmd_wm", "Command", cmd_wm)
        await self._r("cmd_rd_wm", "Command")
        await self._wait_idle()
        await ClockCycles(self.dut.CLK, 5000)

    async def _status_and_irq_flags(self):
        """Trigger and read all status and IRQ flags."""
        config = i2c_config_seq("config", prescaler=49, im=0x1FF)
        await config.start(self.sequencer)

        # Trigger busy/bus_active by starting a transaction and reading Status mid-flight
        await self._w("data_s", "Data", 0x00 | (1 << 9))
        cmd = (SLAVE_ADDR & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
        await self._w("cmd_s", "Command", cmd)
        await self._r("status_busy", "Status")
        await self._wait_idle()
        await self._r("status_idle", "Status")
        await self._r("ris_flags", "RIS")
        if "MIS" in self.addr:
            await self._r("mis_flags", "MIS")
        await ClockCycles(self.dut.CLK, 5000)

        # Trigger MISS_ACK by writing to non-existent slave
        await self._w("data_nack", "Data", 0x00 | (1 << 9))
        cmd_nack = (0x7E & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
        await self._w("cmd_nack", "Command", cmd_nack)
        await self._wait_idle()
        await self._r("status_nack", "Status")
        await self._r("ris_nack", "RIS")

        # Trigger WRF/WROVF by filling write FIFO beyond capacity
        for i in range(20):
            last = (1 << 9) if i == 19 else 0
            await self._w("data_fill", "Data", (i & 0xFF) | last)
        await self._r("ris_wrf", "RIS")
        await self._r("status_wrf", "Status")

        # Trigger CMDF/CMDOVF by filling command FIFO beyond capacity
        for _ in range(20):
            cmd_fill = (SLAVE_ADDR & 0x7F) | (1 << 8) | (1 << 11) | (1 << 12)
            await self._w("cmd_fill", "Command", cmd_fill)
        await self._r("ris_cmdf", "RIS")
        await self._r("status_cmdf", "Status")

        # Wait for transactions to drain
        for _ in range(20):
            await ClockCycles(self.dut.CLK, 5000)
            await self._r("status_drain", "Status")
            regs = ConfigDB().get(None, "", "bus_regs")
            status = regs.read_reg_value("Status")
            if not (status & 0x1):
                break

        await self._r("ris_mid", "RIS")

        # Trigger RDF by doing many reads that fill the read data FIFO
        for _ in range(5):
            await self._w("data_rd_addr", "Data", 0x00 | (1 << 9))
            cmd_rd = (SLAVE_ADDR & 0x7F) | (1 << 8) | (1 << 9) | (1 << 12)
            await self._w("cmd_rd_rdf", "Command", cmd_rd)
        # Wait for reads to complete and fill read FIFO
        for _ in range(10):
            await ClockCycles(self.dut.CLK, 5000)
            await self._r("status_rdf", "Status")
            regs = ConfigDB().get(None, "", "bus_regs")
            status = regs.read_reg_value("Status")
            if not (status & 0x1):
                break
        await self._r("ris_rdf", "RIS")
        await self._r("status_rdf2", "Status")

        await self._r("ris_final", "RIS")
        if "MIS" in self.addr:
            await self._r("mis_final", "MIS")

        # Read data FIFO to trigger RDE
        for _ in range(10):
            await self._r("data_drain", "Data")
        await self._r("ris_rde", "RIS")
