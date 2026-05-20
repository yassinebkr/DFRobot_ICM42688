"""
Mock-I2C logic test (runs on a desktop, NO hardware required).

This harness stubs out CircuitPython-only modules (micropython, busio,
adafruit_bus_device) and a fake ICM42688 register map, then drives the REAL
library code in adafruit_icm42688/__init__.py.

It validates the cache state-machine logic (invalid -> refresh -> valid ->
invalidate on every configuration trigger) and confirms the register-code
constants used by the hardware test suite are accepted by the setters.

It does NOT validate real I2C timing, electrical behaviour, or actual sensor
values -- those still require running the hardware test suite on the board.

Run:  python3 circuitpython/tests/mock_logic_test.py
"""

import sys
import os
import types
import struct

# ---------------------------------------------------------------------------
# 1. Stub CircuitPython-only modules so the library imports under CPython
#    (must be installed BEFORE importing anything from adafruit_icm42688,
#     because importing a submodule first runs the package __init__.py)
# ---------------------------------------------------------------------------

# micropython.const
_mp = types.ModuleType("micropython")
_mp.const = lambda x: x
sys.modules["micropython"] = _mp

# busio / digitalio: the driver evaluates `Union[I2C, SPI]` annotations at
# class-definition time, so these names must exist (on real hardware they come
# from CircuitPython's busio/digitalio).
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

# adafruit_bus_device package + submodules. I2CDevice/SPIDevice are assigned
# the fake implementations further down (after the classes are defined); they
# are only *accessed* when ICM42688() is instantiated.
_abd = types.ModuleType("adafruit_bus_device")
_i2c_dev = types.ModuleType("adafruit_bus_device.i2c_device")
_spi_dev = types.ModuleType("adafruit_bus_device.spi_device")
_abd.i2c_device = _i2c_dev
_abd.spi_device = _spi_dev
sys.modules["adafruit_bus_device"] = _abd
sys.modules["adafruit_bus_device.i2c_device"] = _i2c_dev
sys.modules["adafruit_bus_device.spi_device"] = _spi_dev

# Make the package importable: add the parent of adafruit_icm42688 to path
_LIB_PARENT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")
)
sys.path.insert(0, _LIB_PARENT)

# Real register definitions (importing this runs the package __init__, which
# needs the adafruit_bus_device stub above to already exist).
from adafruit_icm42688 import registers as reg


class FakeBus:
    """
    Minimal fake of a busio.I2C-like object AND the I2CDevice context target.

    The library does:  hasattr(bus, "writeto")  -> detect I2C
    then I2CDevice(bus, addr); `with self._i2c as i2c: i2c.write(...)`.
    We mock I2CDevice to just yield this same object, so this class implements
    write() and write_then_readinto().
    """

    def __init__(self):
        self.bank = 0
        self.regs = {}  # (bank, register) -> byte value
        self.regs[(0, reg.REG_WHO_AM_I)] = reg.ICM42688_CHIP_ID
        self._tick = 0

    # only needed for hasattr(..., "writeto") detection
    def writeto(self, *args, **kwargs):
        pass

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

        # Burst sensor-data read (TEMP + ACCEL + GYRO = 14 bytes)
        if register == reg.REG_TEMP_DATA1 and n >= 14:
            self._tick += 1
            temp_raw = 1325                 # ~35 C
            ax = 5 + (self._tick % 3)       # tiny variation between reads
            ay = -3
            az = 2048                       # ~1 g at +/-16g (2048 LSB/g)
            gx, gy, gz = 1, -1, 0
            packed = struct.pack(">7h", temp_raw, ax, ay, az, gx, gy, gz)
            for i in range(14):
                inbuf[i] = packed[i]
            return

        # Default: return stored register bytes (0 if unset)
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
        raise RuntimeError("SPI not used in mock test")


# Wire the fake bus-device classes into the stub modules created at the top.
_i2c_dev.I2CDevice = FakeI2CDevice
_spi_dev.SPIDevice = FakeSPIDevice

# Now import the real driver
from adafruit_icm42688 import ICM42688


# ---------------------------------------------------------------------------
# 2. Tiny test framework
# ---------------------------------------------------------------------------

class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.fails = []

    def ok(self, name):
        self.passed += 1
        print(f"  PASS  {name}")

    def bad(self, name, reason):
        self.failed += 1
        self.fails.append((name, reason))
        print(f"  FAIL  {name}: {reason}")

    def done(self):
        total = self.passed + self.failed
        print("\n" + "=" * 60)
        print(f"Summary: {self.passed}/{total} passed")
        for name, reason in self.fails:
            print(f"  - {name}: {reason}")
        print("=" * 60)
        return self.failed == 0


def raises_runtime(fn):
    try:
        fn()
        return False
    except RuntimeError:
        return True


# ---------------------------------------------------------------------------
# 3. The actual logic tests (mirror the hardware suite, minus electrical bits)
# ---------------------------------------------------------------------------

def main():
    print("Mock-I2C cache logic test (no hardware)\n")
    icm = ICM42688(FakeBus())
    r = Results()

    # Basic lifecycle
    print("\n[Basic cache lifecycle]")
    if raises_runtime(lambda: icm.accel_x):
        r.ok("cache invalid before first refresh()")
    else:
        r.bad("cache invalid before first refresh()", "no RuntimeError")

    icm.refresh()
    try:
        _ = (icm.accel_x, icm.accel_y, icm.accel_z,
             icm.gyro_x, icm.gyro_y, icm.gyro_z, icm.temp_cached)
        r.ok("all cached props readable after refresh()")
    except Exception as e:
        r.bad("all cached props readable after refresh()", str(e))

    icm.refresh()
    if icm.accel_x == icm.accel_x:
        r.ok("repeated cached read returns same value")
    else:
        r.bad("repeated cached read", "values differ")

    # Each invalidation trigger
    triggers = [
        ("accelerometer_range",
         lambda: setattr(icm, "accelerometer_range",
                         reg.ACCEL_RANGE_8G if icm.accelerometer_range != reg.ACCEL_RANGE_8G
                         else reg.ACCEL_RANGE_4G)),
        ("gyro_range",
         lambda: setattr(icm, "gyro_range",
                         reg.GYRO_RANGE_500_DPS if icm.gyro_range != reg.GYRO_RANGE_500_DPS
                         else reg.GYRO_RANGE_250_DPS)),
        ("accelerometer_data_rate",
         lambda: setattr(icm, "accelerometer_data_rate",
                         reg.ODR_100HZ if icm.accelerometer_data_rate != reg.ODR_100HZ
                         else reg.ODR_200HZ)),
        ("gyro_data_rate",
         lambda: setattr(icm, "gyro_data_rate",
                         reg.ODR_100HZ if icm.gyro_data_rate != reg.ODR_100HZ
                         else reg.ODR_200HZ)),
        ("set_power_mode",
         lambda: icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LP,
                                    gyro_mode=reg.GYRO_MODE_LN)),
        ("enable_fifo",
         lambda: icm.enable_fifo(accel=True, gyro=True)),
        ("disable_fifo",
         lambda: icm.disable_fifo()),
        ("reset",
         lambda: icm.reset()),
    ]

    print("\n[Invalidation triggers]")
    for name, trigger in triggers:
        icm.refresh()
        try:
            _ = icm.accel_x  # valid now
        except Exception as e:
            r.bad(f"{name}: pre-trigger refresh", str(e))
            continue
        try:
            trigger()
        except Exception as e:
            r.bad(f"{name}: trigger call", str(e))
            continue
        if raises_runtime(lambda: icm.accel_x):
            r.ok(f"{name} invalidates cache")
        else:
            r.bad(f"{name} invalidates cache", "cache still valid after trigger")
        # confirm recovery
        try:
            icm.refresh()
            _ = icm.accel_x
        except Exception as e:
            r.bad(f"{name}: refresh recovery", str(e))

    # Filter setters (the design-gap question): do they invalidate?
    print("\n[Filter setters - currently expected NOT to invalidate]")
    for name, trigger in [
        ("set_ui_filter", lambda: icm.set_ui_filter("both", filter_order=2, bandwidth_index=3)),
        ("set_aaf_filter", lambda: icm.set_aaf_filter("both", enabled=True, bandwidth_index=15)),
        ("set_gyro_notch_filter", lambda: icm.set_gyro_notch_filter(frequency_hz=120.0, axis="all")),
    ]:
        icm.refresh()
        trigger()
        still_valid = not raises_runtime(lambda: icm.accel_x)
        print(f"  INFO  after {name}: cache still valid = {still_valid}")

    # Non-cached APIs independent of cache
    print("\n[Non-cached APIs ignore cache state]")
    icm.invalidate_cache()
    try:
        a = icm.acceleration
        g = icm.gyro
        t, a2, g2 = icm.all_data
        if (isinstance(a, tuple) and len(a) == 3 and
                isinstance(g, tuple) and len(g) == 3):
            r.ok("acceleration/gyro/all_data work while cache invalid")
        else:
            r.bad("non-cached APIs", "unexpected return shapes")
    except Exception as e:
        r.bad("non-cached APIs while cache invalid", str(e))

    # Non-cached read must not silently revalidate the cache
    icm.invalidate_cache()
    _ = icm.acceleration
    if raises_runtime(lambda: icm.accel_x):
        r.ok("non-cached read does NOT revalidate cache")
    else:
        r.bad("non-cached read does NOT revalidate cache", "cache became valid")

    ok = r.done()
    print("RESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
