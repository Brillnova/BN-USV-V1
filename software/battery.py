# battery.py
import time
import smbus2

INA226_ADDR = 0x40

REG_BUS_VOLTAGE = 0x02
REG_CURRENT = 0x04
REG_CALIBRATION = 0x05


class BatteryMonitor:
    def __init__(
        self,
        bus_id=1,
        address=INA226_ADDR,
        shunt_ohm=0.002,
        current_lsb=0.001,
        capacity_mah=10000.0,
        cell_count=4,
        startup_samples=20,
        startup_sample_interval=0.1,
        idle_current_threshold_a=0.3,
        idle_time_required_s=5.0,
        voltage_correction_alpha=0.05,
    ):
        self.bus = smbus2.SMBus(bus_id)
        self.addr = address

        self.shunt_ohm = shunt_ohm
        self.current_lsb = current_lsb
        self.capacity_mah = capacity_mah
        self.cell_count = cell_count

        self.idle_current_threshold_a = idle_current_threshold_a
        self.idle_time_required_s = idle_time_required_s
        self.voltage_correction_alpha = voltage_correction_alpha

        self.idle_start_time = None
        self.used_mah = 0.0
        self.last_time = time.time()

        cal = int(0.00512 / (self.current_lsb * self.shunt_ohm))
        self.write_word(REG_CALIBRATION, cal)

        self.initial_voltage = self.measure_average_voltage(
            samples=startup_samples,
            interval_s=startup_sample_interval,
        )

        self.initial_soc_percent = self.estimate_lipo_soc_from_voltage(
            self.initial_voltage
        )

        self.remaining_mah = self.capacity_mah * (self.initial_soc_percent / 100.0)
        self.soc_percent = self.initial_soc_percent

    def write_word(self, reg, value):
        swapped = ((value & 0xFF) << 8) | (value >> 8)
        self.bus.write_word_data(self.addr, reg, swapped)

    def read_word(self, reg):
        value = self.bus.read_word_data(self.addr, reg)
        return ((value & 0xFF) << 8) | (value >> 8)

    def read_signed_word(self, reg):
        value = self.read_word(reg)
        if value > 32767:
            value -= 65536
        return value

    def get_voltage(self):
        raw = self.read_word(REG_BUS_VOLTAGE)
        return raw * 1.25e-3

    def get_current(self):
        raw = self.read_signed_word(REG_CURRENT)
        return raw * self.current_lsb

    def measure_average_voltage(self, samples=10, interval_s=0.1):
        values = []

        for _ in range(samples):
            values.append(self.get_voltage())
            time.sleep(interval_s)

        return sum(values) / len(values)

    def estimate_lipo_soc_from_voltage(self, voltage):
        cell_v = voltage / self.cell_count

        soc_table = [
            (4.20, 100),
            (4.10, 90),
            (4.00, 80),
            (3.92, 70),
            (3.85, 60),
            (3.79, 50),
            (3.75, 40),
            (3.70, 30),
            (3.60, 20),
            (3.50, 10),
            (3.30, 0),
        ]

        if cell_v >= soc_table[0][0]:
            return 100.0

        if cell_v <= soc_table[-1][0]:
            return 0.0

        for i in range(len(soc_table) - 1):
            v_high, soc_high = soc_table[i]
            v_low, soc_low = soc_table[i + 1]

            if v_low <= cell_v <= v_high:
                ratio = (cell_v - v_low) / (v_high - v_low)
                return soc_low + ratio * (soc_high - soc_low)

        return 0.0

    def update_coulomb_counter(self, current_a):
        now = time.time()
        dt_sec = now - self.last_time
        self.last_time = now

        if dt_sec <= 0:
            return

        used_mah_delta = current_a * 1000.0 * (dt_sec / 3600.0)

        # Positive current means battery discharge.
        if used_mah_delta > 0:
            self.used_mah += used_mah_delta
            self.remaining_mah -= used_mah_delta

        # Negative current could mean charging/regeneration.
        # Usually not expected in this USV, but this keeps the logic safe.
        elif used_mah_delta < 0:
            self.remaining_mah -= used_mah_delta
            self.used_mah = max(0.0, self.used_mah + used_mah_delta)

        self.remaining_mah = max(0.0, min(self.capacity_mah, self.remaining_mah))
        self.soc_percent = (self.remaining_mah / self.capacity_mah) * 100.0

    def update_idle_voltage_correction(self, voltage, current_a):
        now = time.time()

        is_idle = abs(current_a) <= self.idle_current_threshold_a

        if not is_idle:
            self.idle_start_time = None
            return False

        if self.idle_start_time is None:
            self.idle_start_time = now
            return False

        idle_duration = now - self.idle_start_time

        if idle_duration < self.idle_time_required_s:
            return False

        voltage_soc = self.estimate_lipo_soc_from_voltage(voltage)

        corrected_soc = (
            (1.0 - self.voltage_correction_alpha) * self.soc_percent
            + self.voltage_correction_alpha * voltage_soc
        )

        corrected_soc = max(0.0, min(100.0, corrected_soc))

        self.soc_percent = corrected_soc
        self.remaining_mah = self.capacity_mah * (corrected_soc / 100.0)
        self.used_mah = self.capacity_mah - self.remaining_mah

        return True

    def read(self):
        voltage = self.get_voltage()
        current = self.get_current()

        self.update_coulomb_counter(current)

        voltage_soc = self.estimate_lipo_soc_from_voltage(voltage)
        correction_applied = self.update_idle_voltage_correction(voltage, current)

        return {
            "voltage": voltage,
            "current": current,
            "voltage_soc": voltage_soc,
            "coulomb_soc": self.soc_percent,
            "used_mah": self.used_mah,
            "remaining_mah": self.remaining_mah,
            "initial_voltage": self.initial_voltage,
            "initial_soc": self.initial_soc_percent,
            "idle_correction_applied": correction_applied,
        }