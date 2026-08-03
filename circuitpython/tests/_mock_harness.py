"""
Shared mock-I2C harness for desktop (CPython) testing of the ICM42688 driver.

Stubs the CircuitPython-only modules (micropython, busio, digitalio,
adafruit_bus_device) and provides a FakeBus with a minimal ICM42688 register
map, so the REAL driver in adafruit_icm42688/__init__.py can run without
hardware.

Usage:
    from _mock_harness import reg, ICM42688, FakeBus, new_sensor
    icm = new_sensor()

This is desktop tooling only; it does NOT run on the board and is not deployed
to CIRCUITPY. It validates logic, not electrical/timing behaviour.
"""

import sys
import os
import types
import struct

# ---------------------------------------------------------------------------
# Stub CircuitPython-only modules BEFORE importing anything from the package
# (importing a submodule first runs the package __init__.py).
# ---------------------------------------------------------------------------

_mp = types.ModuleType("micropython")
_mp.const = lambda x: x
sys.modules["micropython"] = _mp

# busio / digitalio: the driver evaluates `Union[I2C, SPI]` annotations at
# class-definition time, so these names must exist.
_busio = types.ModuleType("busio")
class _I2C:  # noqa: N801
    pass
class _SPI:  # noqa: N801
    pass
_busio.I2C = _I2C
_busio.SPI = _SPI
sys.modules["busio"] = _busio

_digitalio = types.ModuleType("digitalio")
class _DigitalInOut:  # noqa: N801
    pass
_digitalio.DigitalInOut = _DigitalInOut
sys.modules["digitalio"] = _digitalio

# adafruit_bus_device package + submodules; I2CDevice/SPIDevice assigned below.
_abd = types.ModuleType("adafruit_bus_device")
_i2c_dev = types.ModuleType("adafruit_bus_device.i2c_device")
_spi_dev = types.ModuleType("adafruit_bus_device.spi_device")
_abd.i2c_device = _i2c_dev
_abd.spi_device = _spi_dev
sys.modules["adafruit_bus_device"] = _abd
sys.modules["adafruit_bus_device.i2c_device"] = _i2c_dev
sys.modules["adafruit_bus_device.spi_device"] = _spi_dev

# Make the package importable: parent of adafruit_icm42688
_LIB_PARENT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, _LIB_PARENT)

from adafruit_icm42688 import registers as reg  # noqa: E402


class FakeBus:
    """Fake busio.I2C-like object that also serves as the I2CDevice target."""

    def __init__(self):
        self.bank = 0
        self.regs = {}  # (bank, register) -> byte value
        self.regs[(0, reg.REG_WHO_AM_I)] = reg.ICM42688_CHIP_ID
        self._tick = 0

    def writeto(self, *args, **kwargs):
        pass  # only needed for hasattr(..., "writeto") detection

    def write(self, buf, **kwargs):
        b = list(buf)
        register = b[0]
        if register == reg.REG_BANK_SEL:
            self.bank = b[1]
            return
        for i, val in enumerate(b[1:]):
            self.regs[(self.bank, register + i)] = val

    def write_then_readinto(self, out, inbuf, out_end=None, in_end=None, **kwargs):
        register = out[0]
        n = in_end if in_end is not None else len(inbuf)

        # Burst sensor-data read (TEMP + ACCEL + GYRO = 14 bytes).
        # Emit a realistic ~1 g on Z for whatever accel range is configured, so
        # the converted m/s^2 value stays ~9.8 across ranges (lets the harness
        # validate scale-factor correctness, not just the cache state machine).
        if register == reg.REG_TEMP_DATA1 and n >= 14:
            self._tick += 1
            accel_cfg = self.regs.get((0, reg.REG_ACCEL_CONFIG0), 0)
            accel_range_code = (accel_cfg >> 5) & 0x07
            sens = reg.ACCEL_SENSITIVITY.get(accel_range_code, 2048.0)
            temp_raw = 1325                       # ~35 C
            ax = self._tick % 3                   # tiny jitter between reads
            ay = -3
            az = int(round(sens))                 # ~1 g for the configured range
            gx, gy, gz = 1, -1, 0
            packed = struct.pack(">7h", temp_raw, ax, ay, az, gx, gy, gz)
            for i in range(14):
                inbuf[i] = packed[i]
            return

        for i in range(n):
            inbuf[i] = self.regs.get((self.bank, register + i), 0)


class FakeI2CDevice:
    def __init__(self, bus, address):
        self._bus = bus

    def __enter__(self):
        return self._bus

    def __exit__(self, *exc):
        return False


class FakeSPIDevice:  # present so the import succeeds; unused
    def __init__(self, *args, **kwargs):
        raise RuntimeError("SPI not used in mock harness")


_i2c_dev.I2CDevice = FakeI2CDevice
_spi_dev.SPIDevice = FakeSPIDevice

from adafruit_icm42688 import ICM42688  # noqa: E402


def new_sensor():
    """Return a fresh ICM42688 backed by a fresh FakeBus."""
    return ICM42688(FakeBus())
