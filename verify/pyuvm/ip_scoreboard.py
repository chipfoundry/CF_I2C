"""I2C scoreboard — compares I2C bus transactions from DUT against expected."""

from cf_verify.base.scoreboard import scoreboard
from ip_item.i2c_item import i2c_item


class i2c_scoreboard(scoreboard):
    async def _compare_ip(self):
        """Compare I2C transactions observed on the bus."""
        while True:
            dut_tr = await self.ip_dut_fifo.get()
            ref_tr = await self.ip_ref_fifo.get()
            self._check("IP", dut_tr, ref_tr)

    def check_phase(self):
        assert self.failed == 0, (
            f"I2C scoreboard mismatches: failed={self.failed}, passed={self.passed}"
        )
