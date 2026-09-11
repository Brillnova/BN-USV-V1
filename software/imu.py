# imu.py
import smbus2
import math

IMU_ADDR = 0x68

PWR_MGMT_1 = 0x06
ACCEL_XOUT_H = 0x2D

class IMU:
    def __init__(self, bus_id=1, address=IMU_ADDR):
        self.bus = smbus2.SMBus(bus_id)
        self.addr = address

    def init(self):
        # wake up
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)

    def read_i16(self, reg):
        hi = self.bus.read_byte_data(self.addr, reg)
        lo = self.bus.read_byte_data(self.addr, reg + 1)
        val = (hi << 8) | lo
        if val & 0x8000:
            val -= 65536
        return val

    def read_accel(self):
        ax = self.read_i16(ACCEL_XOUT_H) / 16384.0
        ay = self.read_i16(ACCEL_XOUT_H + 2) / 16384.0
        az = self.read_i16(ACCEL_XOUT_H + 4) / 16384.0
        return ax, ay, az

    def read_gyro(self):
        gx = self.read_i16(ACCEL_XOUT_H + 6) / 131.0
        gy = self.read_i16(ACCEL_XOUT_H + 8) / 131.0
        gz = self.read_i16(ACCEL_XOUT_H + 10) / 131.0
        return gx, gy, gz