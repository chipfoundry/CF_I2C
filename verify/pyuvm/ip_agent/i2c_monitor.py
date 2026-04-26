"""I2C IP monitor — observes SCL and SDA lines, decodes I2C transactions."""

import cocotb
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, Timer, First
from pyuvm import uvm_monitor, uvm_analysis_port, ConfigDB

from ip_item.i2c_item import i2c_item


class i2c_monitor(uvm_monitor):
    def build_phase(self):
        super().build_phase()
        self.ap = uvm_analysis_port("ap", self)
        self.dut = ConfigDB().get(self, "", "DUT")

    async def run_phase(self):
        cocotb.start_soon(self._monitor_i2c())

    async def _monitor_i2c(self):
        """Watch the I2C bus for START conditions, then decode the transaction."""
        while True:
            await self._wait_start()
            tr = await self._decode_transaction()
            if tr is not None:
                self.ap.write(tr)

    async def _wait_start(self):
        """Detect I2C START: SDA falling while SCL is high."""
        while True:
            await FallingEdge(self.dut.SDA)
            await Timer(1, "ns")
            try:
                if self.dut.SCL.value == 1:
                    return
            except Exception:
                continue

    async def _decode_transaction(self):
        """Decode address byte + R/W bit + ACK after a START condition."""
        try:
            bits = []
            for _ in range(8):
                await RisingEdge(self.dut.SCL)
                await Timer(1, "ns")
                try:
                    bits.append(int(self.dut.SDA.value))
                except Exception:
                    return None

            address = 0
            for i in range(7):
                address = (address << 1) | bits[i]
            rw_bit = bits[7]

            # ACK bit
            await RisingEdge(self.dut.SCL)
            await Timer(1, "ns")
            try:
                ack = int(self.dut.SDA.value) == 0
            except Exception:
                ack = False

            tr = i2c_item("i2c_mon_tr")
            tr.address = address
            tr.direction = i2c_item.READ if rw_bit else i2c_item.WRITE
            tr.ack = ack

            # Decode data byte if present
            data_bits = []
            for _ in range(8):
                try:
                    await RisingEdge(self.dut.SCL)
                    await Timer(1, "ns")
                    data_bits.append(int(self.dut.SDA.value))
                except Exception:
                    break

            if len(data_bits) == 8:
                data = 0
                for b in data_bits:
                    data = (data << 1) | b
                tr.data = data

            self.logger.debug(f"I2C monitor: {tr.convert2string()}")
            return tr
        except Exception:
            return None
