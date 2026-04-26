"""I2C coverage component — samples both auto-generated and custom coverage."""

from pyuvm import ConfigDB

from cf_verify.ip_env.ip_coverage import ip_coverage
from ip_coverage.i2c_cov_groups import i2c_cov_groups
from ip_item.i2c_item import i2c_item


class i2c_coverage(ip_coverage):
    def build_phase(self):
        super().build_phase()
        regs = ConfigDB().get(None, "", "bus_regs")
        self.cov_groups = i2c_cov_groups("top.ip", regs)

    def sample(self, tr):
        if isinstance(tr, i2c_item):
            self.cov_groups.sample(tr)
        else:
            self.cov_groups.sample_bus(tr)
