"""I2C coverage groups — auto-generated + I2C-specific custom coverage."""

from cocotb_coverage.coverage import CoverPoint, CoverCross

from cf_verify.coverage.auto_coverage import generate_coverage_from_yaml
from cf_verify.bus_env.bus_item import bus_item
from ip_item.i2c_item import i2c_item

I2C_FIELD_BINS = {
    ("PR", None): [(0, 3), (4, 15), (16, 63), (64, 255)],
}

ADDRESS_BINS = [(i * 16, i * 16 + 15) for i in range(8)]
DATA_BINS = [(i * 32, i * 32 + 31) for i in range(8)]


class i2c_cov_groups:
    def __init__(self, hierarchy, regs):
        self.hierarchy = hierarchy
        self.regs = regs

        self.auto_points = generate_coverage_from_yaml(
            regs, hierarchy, field_bins_override=I2C_FIELD_BINS,
        )

        self.addr_cov = self._address_coverage()
        self.data_cov = self._data_coverage()
        self.status_cov = self._status_coverage()
        self.irq_cov = self._irq_coverage()

        self._init_sample(None)

    def _init_sample(self, tr):
        @self._apply_decorators(
            self.auto_points + self.addr_cov + self.data_cov
            + self.status_cov + self.irq_cov
        )
        def _cold(tr):
            pass

    def sample(self, tr):
        @self._apply_decorators(
            self.auto_points + self.addr_cov + self.data_cov
            + self.status_cov + self.irq_cov
        )
        def _s(tr):
            pass
        _s(tr)

    def sample_bus(self, tr):
        rname = self.regs._reg_address_to_name.get(tr.addr)
        if rname:
            self.regs._reg_values[rname.lower()] = tr.data

        @self._apply_decorators(
            self.auto_points + self.status_cov + self.irq_cov
        )
        def _bus(tr):
            pass
        _bus(tr)

    def _address_coverage(self):
        points = []
        for direction in [i2c_item.READ, i2c_item.WRITE]:
            d_str = "READ" if direction == i2c_item.READ else "WRITE"
            points.append(CoverPoint(
                f"{self.hierarchy}.{d_str}.Address",
                xf=lambda tr, d=direction: (
                    (tr.direction, tr.address)
                    if isinstance(tr, i2c_item) else (0, 0)
                ),
                bins=ADDRESS_BINS,
                bins_labels=[f"0x{lo:02x}-0x{hi:02x}" for lo, hi in ADDRESS_BINS],
                rel=lambda val, b, d=direction: (
                    val[0] == d and b[0] <= val[1] <= b[1]
                ),
            ))
        return points

    def _data_coverage(self):
        points = []
        for direction in [i2c_item.READ, i2c_item.WRITE]:
            d_str = "READ" if direction == i2c_item.READ else "WRITE"
            points.append(CoverPoint(
                f"{self.hierarchy}.{d_str}.Data",
                xf=lambda tr, d=direction: (
                    (tr.direction, tr.data)
                    if isinstance(tr, i2c_item) else (0, 0)
                ),
                bins=DATA_BINS,
                bins_labels=[f"0x{lo:02x}-0x{hi:02x}" for lo, hi in DATA_BINS],
                rel=lambda val, b, d=direction: (
                    val[0] == d and b[0] <= val[1] <= b[1]
                ),
            ))
        return points

    def _status_coverage(self):
        return [
            CoverPoint(
                f"{self.hierarchy}.Status.Busy",
                xf=lambda tr: self.regs.read_reg_value("Status") & 1,
                bins=[0, 1], bins_labels=["idle", "busy"], at_least=1,
            ),
            CoverPoint(
                f"{self.hierarchy}.Status.BusCont",
                xf=lambda tr: (self.regs.read_reg_value("Status") >> 1) & 1,
                bins=[0, 1], bins_labels=["no_control", "bus_control"], at_least=1,
            ),
            CoverPoint(
                f"{self.hierarchy}.Status.BusAct",
                xf=lambda tr: (self.regs.read_reg_value("Status") >> 2) & 1,
                bins=[0, 1], bins_labels=["inactive", "active"], at_least=1,
            ),
            CoverPoint(
                f"{self.hierarchy}.Status.MissAck",
                xf=lambda tr: (self.regs.read_reg_value("Status") >> 3) & 1,
                bins=[0, 1], bins_labels=["ack_ok", "miss_ack"], at_least=1,
            ),
        ]

    def _irq_coverage(self):
        flag_names = [
            "MISS_ACK", "CMDE", "CMDF", "CMDOVF",
            "WRE", "WRF", "WROVF", "RDE", "RDF",
        ]
        points = []
        for i, name in enumerate(flag_names):
            points.append(CoverPoint(
                f"{self.hierarchy}.IRQ.{name}",
                xf=lambda tr, bit=i: (self.regs.read_reg_value("RIS") >> bit) & 1,
                bins=[0, 1], bins_labels=["clear", "set"], at_least=1,
            ))
        return points

    @staticmethod
    def _apply_decorators(decorators):
        def wrapper(func):
            for dec in decorators:
                func = dec(func)
            return func
        return wrapper
