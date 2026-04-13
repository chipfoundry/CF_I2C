"""Test library for CF_I2C verification — 6 tests covering I2C master functionality."""

import os
from pathlib import Path

import cocotb
import pyuvm
from pyuvm import uvm_root, ConfigDB

from cocotb.triggers import ClockCycles
from cocotb_coverage.coverage import coverage_db

from cf_verify.base.base_test import base_test
from cf_verify.base.top_env import top_env
from cf_verify.bus_env.bus_regs import BusRegs
from cf_verify.bus_env.bus_seq_lib import write_read_regs_seq, reset_seq
from cf_verify.ip_env.ip_agent import ip_agent
from cf_verify.ip_env.ip_driver import ip_driver
from cf_verify.ip_env.ip_monitor import ip_monitor
from cf_verify.ip_env.ip_coverage import ip_coverage

from ip_agent.i2c_driver import i2c_driver
from ip_agent.i2c_monitor import i2c_monitor
from ip_coverage.i2c_coverage import i2c_coverage
from ip_scoreboard import i2c_scoreboard


class i2c_env(top_env):
    """I2C-specific top environment with proper component wiring."""

    def build_phase(self):
        from cf_verify.bus_env.bus_agent import bus_agent
        from cf_verify.ip_env.ip_logger import ip_logger
        from cf_verify.base.ref_model import ref_model

        self.bus_agent = bus_agent("bus_agent", self)
        self.ip_agent = i2c_ip_agent("ip_agent", self)
        self.ref_model = ref_model("ref_model", self)
        self.scoreboard = i2c_scoreboard("scoreboard", self)
        self.ip_coverage = i2c_coverage("ip_coverage", self)
        self.ip_logger = ip_logger("ip_logger", self)

    def connect_phase(self):
        super().connect_phase()
        self.bus_agent.monitor.ap.connect(self.ip_coverage.analysis_export)


class i2c_ip_agent(ip_agent):
    driver_cls = i2c_driver
    monitor_cls = i2c_monitor


class i2c_base_test(base_test):
    """Base test for CF_I2C — wires up the I2C environment."""

    def build_phase(self):
        import os
        import cocotb

        dut = cocotb.top
        bus_type = os.environ.get("BUS_TYPE", "APB")
        yaml_file = os.environ.get(
            "YAML_FILE",
            str(Path(__file__).resolve().parent.parent.parent / "CF_I2C.yaml"),
        )
        test_path = os.environ.get("TEST_PATH", "./sim")

        regs = BusRegs(yaml_file)

        ConfigDB().set(None, "*", "DUT", dut)
        ConfigDB().set(None, "*", "BUS_TYPE", bus_type)
        ConfigDB().set(None, "*", "bus_regs", regs)
        ConfigDB().set(None, "*", "irq_exist", regs.get_irq_exist())
        ConfigDB().set(None, "*", "collect_coverage", True)
        ConfigDB().set(None, "*", "disable_logger", False)
        ConfigDB().set(None, "*", "TEST_PATH", test_path)

        self.env = i2c_env("env", self)
        super().build_phase()


@pyuvm.test()
class WriteReadRegsTest(i2c_base_test):
    """Write/read all accessible registers."""

    async def run_phase(self):
        self.raise_objection()
        seq = write_read_regs_seq("write_read_regs")
        await seq.start(self.env.bus_agent.sequencer)
        self.drop_objection()


@pyuvm.test()
class I2CWriteReadTest(i2c_base_test):
    """Write data to EEPROM then read back and verify."""

    async def run_phase(self):
        self.raise_objection()
        from seq_lib.i2c_write_read_seq import i2c_write_read_seq
        seq = i2c_write_read_seq("wr_rd_test")
        await seq.start(self.env.bus_agent.sequencer)
        self.drop_objection()


@pyuvm.test()
class I2CWriteMultiTest(i2c_base_test):
    """Write multiple bytes to EEPROM via write_multiple command."""

    async def run_phase(self):
        self.raise_objection()
        from seq_lib.i2c_write_seq import i2c_write_seq
        seq = i2c_write_seq("write_multi", data=[0xAA, 0xBB, 0xCC, 0xDD])
        await seq.start(self.env.bus_agent.sequencer)
        self.drop_objection()


@pyuvm.test()
class PrescalerTest(i2c_base_test):
    """Test different I2C clock speeds by varying the prescaler."""

    async def run_phase(self):
        self.raise_objection()
        from seq_lib.i2c_write_seq import i2c_write_seq
        for pr in [10, 49, 99]:
            seq = i2c_write_seq(f"pr_{pr}", prescaler=pr, data=[0x42])
            await seq.start(self.env.bus_agent.sequencer)
        self.drop_objection()


@pyuvm.test()
class InterruptTest(i2c_base_test):
    """Test interrupt sources — mask, status, and clear."""

    async def run_phase(self):
        self.raise_objection()
        from seq_lib.i2c_interrupt_seq import i2c_interrupt_seq
        seq = i2c_interrupt_seq("irq_test")
        await seq.start(self.env.bus_agent.sequencer)
        self.drop_objection()


@pyuvm.test()
class StatusTest(i2c_base_test):
    """Verify status register flags during I2C operations."""

    async def run_phase(self):
        self.raise_objection()
        from seq_lib.i2c_write_seq import i2c_write_seq
        from cf_verify.bus_env.bus_seq_lib import read_reg_seq

        regs = ConfigDB().get(None, "", "bus_regs")
        addr = regs.reg_name_to_address
        dut = ConfigDB().get(self, "", "DUT")

        # Read status before any operation
        await read_reg_seq("rd_status_init", addr["Status"]).start(
            self.env.bus_agent.sequencer
        )

        # Start a write to trigger busy flag
        seq = i2c_write_seq("status_wr", data=[0x55])
        await seq.start(self.env.bus_agent.sequencer)

        # Read status after operation completes
        await read_reg_seq("rd_status_done", addr["Status"]).start(
            self.env.bus_agent.sequencer
        )

        self.drop_objection()
