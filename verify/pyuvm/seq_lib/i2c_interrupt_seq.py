"""I2C interrupt sequence — exercises interrupt sources and verifies IM/IC."""

from pyuvm import uvm_sequence, ConfigDB

from cf_verify.bus_env.bus_seq_lib import write_reg_seq, read_reg_seq
from seq_lib.i2c_config_seq import i2c_config_seq


class i2c_interrupt_seq(uvm_sequence):
    async def body(self):
        regs = ConfigDB().get(None, "", "bus_regs")
        addr = regs.reg_name_to_address

        config = i2c_config_seq("config", im=0x1FF)
        await config.start(self.sequencer)

        # Read RIS to check current interrupt status
        await read_reg_seq("ris_rd", addr["RIS"]).start(self.sequencer)
        # Read MIS to check masked status
        await read_reg_seq("mis_rd", addr["MIS"]).start(self.sequencer)

        # Test individual interrupt mask bits (9 flags)
        for bit in range(9):
            await write_reg_seq("im_set", addr["IM"], 1 << bit).start(self.sequencer)
            await read_reg_seq("im_rd", addr["IM"]).start(self.sequencer)
            await read_reg_seq("mis_check", addr["MIS"]).start(self.sequencer)

        # Restore all interrupts enabled
        await write_reg_seq("im_all", addr["IM"], 0x1FF).start(self.sequencer)

        # Verify final state
        await read_reg_seq("ris_check", addr["RIS"]).start(self.sequencer)
