"""I2C transaction item — carries address, data, direction, and ack status."""

import random

from pyuvm import uvm_sequence_item


class i2c_item(uvm_sequence_item):
    READ = 0
    WRITE = 1

    def __init__(self, name="i2c_item"):
        super().__init__(name)
        self.address = 0
        self.data = 0
        self.direction = i2c_item.WRITE
        self.ack = True

    def randomize(self, max_addr=0x7F, max_data=0xFF):
        self.address = random.randint(0, max_addr)
        self.data = random.randint(0, max_data)
        self.direction = random.choice([i2c_item.READ, i2c_item.WRITE])

    def convert2string(self):
        d = "READ" if self.direction == i2c_item.READ else "WRITE"
        ack_str = "ACK" if self.ack else "NACK"
        return (
            f"i2c addr=0x{self.address:02x} data=0x{self.data:02x} "
            f"direction={d} {ack_str}"
        )

    def do_compare(self, rhs):
        return (
            self.address == rhs.address
            and self.data == rhs.data
            and self.direction == rhs.direction
            and self.ack == rhs.ack
        )

    def do_copy(self, rhs):
        super().do_copy(rhs)
        self.address = rhs.address
        self.data = rhs.data
        self.direction = rhs.direction
        self.ack = rhs.ack

    def do_clone(self):
        new = i2c_item(self.get_name())
        new.do_copy(self)
        return new
