"""
Mock-I2C logic test (runs on a desktop, NO hardware required).

Drives the REAL library code through the shared FakeBus harness to validate the
cache state-machine logic (invalid -> refresh -> valid -> invalidate on every
configuration trigger) and confirm the register-code constants used by the
hardware suite are accepted by the setters.

It does NOT validate real I2C timing, electrical behaviour, or actual sensor
values -- those still require running the hardware test suite on the board.

Run:  python3 circuitpython/tests/mock_logic_test.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _mock_harness import reg, new_sensor


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


def main():
    print("Mock-I2C cache logic test (no hardware)\n")
    icm = new_sensor()
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
        try:
            icm.refresh()
            _ = icm.accel_x
        except Exception as e:
            r.bad(f"{name}: refresh recovery", str(e))

    # Filter setters should also invalidate for consistency
    print("\n[Filter setter invalidation]")
    for name, trigger in [
        ("set_ui_filter", lambda: icm.set_ui_filter("both", filter_order=2, bandwidth_index=3)),
        ("set_aaf_filter", lambda: icm.set_aaf_filter("both", enabled=True, bandwidth_index=15)),
        ("set_gyro_notch_filter", lambda: icm.set_gyro_notch_filter(frequency_hz=120.0, axis="all")),
    ]:
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
        try:
            icm.refresh()
            _ = icm.accel_x
        except Exception as e:
            r.bad(f"{name}: refresh recovery", str(e))

    # Scale-factor correctness: converted gravity must stay ~9.8 m/s^2 across
    # ranges (proves the setter updates the scale factor before the next read).
    print("\n[Scale-factor correctness across ranges]")
    GRAVITY = 9.80665
    for range_code in (reg.ACCEL_RANGE_2G, reg.ACCEL_RANGE_4G,
                       reg.ACCEL_RANGE_8G, reg.ACCEL_RANGE_16G):
        icm.accelerometer_range = range_code
        icm.refresh()
        ax, ay, az = icm.accel_x, icm.accel_y, icm.accel_z
        mag = (ax * ax + ay * ay + az * az) ** 0.5
        if abs(mag - GRAVITY) < 0.5:
            r.ok(f"range code {range_code}: |accel|={mag:.3f} m/s^2 (~1 g)")
        else:
            r.bad(f"range code {range_code}: scale correctness",
                  f"|accel|={mag:.3f} m/s^2, expected ~{GRAVITY:.2f}")
    icm.accelerometer_range = reg.ACCEL_RANGE_16G  # restore default

    # Snapshot semantics: cached value is a frozen snapshot, NOT live data.
    print("\n[Snapshot semantics]")
    icm.refresh()
    ax_snapshot = icm.accel_x
    for _ in range(5):
        _ = icm.acceleration  # live reads advance the bus tick / change data
    if icm.accel_x == ax_snapshot:
        r.ok("cached accel_x is a frozen snapshot across live reads")
    else:
        r.bad("snapshot semantics", "cached value drifted with live reads")

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
