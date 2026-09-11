# mag.py
import math
import smbus2


CTRL_REG1 = 0x20
CTRL_REG2 = 0x21
CTRL_REG3 = 0x22
CTRL_REG4 = 0x23
OUT_X_L = 0x28


class Magnetometer:
    def __init__(
        self,
        address,
        offset_x,
        offset_y,
        scale_x,
        scale_y,
        avg_scale,
        bus_num=1,
    ):
        self.address = address
        self.bus = smbus2.SMBus(bus_num)

        self.offset_x = offset_x
        self.offset_y = offset_y
        self.scale_x = scale_x
        self.scale_y = scale_y
        self.avg_scale = avg_scale

    def read_i16(self, reg):
        """Read signed 16-bit little-endian value."""
        lo = self.bus.read_byte_data(self.address, reg)
        hi = self.bus.read_byte_data(self.address, reg + 1)

        value = (hi << 8) | lo

        if value & 0x8000:
            value -= 65536

        return value

    def init(self):
        """Initialize LIS3MDL magnetometer."""
        self.bus.write_byte_data(self.address, CTRL_REG1, 0b11110000)
        self.bus.write_byte_data(self.address, CTRL_REG2, 0b00000000)
        self.bus.write_byte_data(self.address, CTRL_REG3, 0b00000000)
        self.bus.write_byte_data(self.address, CTRL_REG4, 0b00001100)

    def get_heading(self):
        """Read magnetometer and return heading in degrees."""
        mx = self.read_i16(OUT_X_L)
        my = self.read_i16(OUT_X_L + 2)

        # Hard iron calibration
        mx -= self.offset_x
        my -= self.offset_y

        # Soft iron calibration
        mx *= self.avg_scale / self.scale_x
        my *= self.avg_scale / self.scale_y

        heading = math.degrees(math.atan2(my, mx))

        if heading < 0:
            heading += 360

        return heading