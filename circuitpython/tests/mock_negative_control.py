"""
Negative control (mutation test) -- proves the cache tests actually have teeth.

A test suite that passes is only meaningful if it would FAIL on broken code.
This script re-introduces the original "cache poisoning" bug and shows that the
exact assertion the hardware/mock suites use (after a config change, a cached
getter must raise RuntimeError) flips from PASS to FAIL.

How the bug is re-introduced WITHOUT duplicating any setter logic:
  BugInjector overrides the `_cache_valid` flag so that invalidation requests
  (setting it False) are ignored once the cache has been validated. That is
  exactly the pre-fix behaviour: refresh() makes the cache valid and nothing
  ever marks it stale again -- so every config setter's `self._cache_valid =
  False` becomes a no-op, mirroring the original code that simply lacked those
  lines.

Run:  python3 circuitpython/tests/mock_negative_control.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _mock_harness import reg, ICM42688, FakeBus


class BugInjector(ICM42688):
    """ICM42688 with the cache-invalidation fix neutralised (simulates pre-fix)."""

    @property
    def _cache_valid(self):
        return self.__dict__.get("_cv_real", False)

    @_cache_valid.setter
    def _cache_valid(self, value):
        # PRE-FIX BUG: ignore invalidation (False); only allow validation (True).
        if value:
            self.__dict__["_cv_real"] = True
        # setting False is silently dropped -> cache is never invalidated


def raises_runtime(fn):
    try:
        fn()
        return False
    except RuntimeError:
        return True


# The triggers exactly mirror the hardware/mock suites.
def trigger_list(icm):
    return [
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
         lambda: icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LP, gyro_mode=reg.GYRO_MODE_LN)),
        ("enable_fifo", lambda: icm.enable_fifo(accel=True, gyro=True)),
        ("disable_fifo", lambda: icm.disable_fifo()),
        ("reset", lambda: icm.reset()),
        ("set_ui_filter", lambda: icm.set_ui_filter("both", filter_order=2, bandwidth_index=3)),
        ("set_aaf_filter", lambda: icm.set_aaf_filter("both", enabled=True, bandwidth_index=15)),
        ("set_gyro_notch_filter", lambda: icm.set_gyro_notch_filter(frequency_hz=120.0, axis="all")),
    ]


def assertion_after_trigger(icm, trigger):
    """
    Reproduce the suite's assertion: refresh, read (works), apply config change,
    then the cached getter MUST raise. Returns True if the assertion holds.
    """
    icm.refresh()
    _ = icm.accel_x          # valid now
    trigger()                # config change
    return raises_runtime(lambda: icm.accel_x)


def main():
    print("Negative control: do the cache tests actually catch the bug?\n")

    fixed = ICM42688(FakeBus())
    buggy = BugInjector(FakeBus())

    # Sanity: confirm the mutation actually reproduces stale-cache behaviour.
    buggy.refresh()
    snapshot = buggy.accel_x
    buggy.refresh()  # re-read; FakeBus varies ax slightly each burst
    # In buggy mode the cache is "valid" but we just proved getters still work;
    # the meaningful check is per-trigger below.

    print(f"{'TRIGGER':<26} {'FIXED (want raise)':<20} {'BUGGY (want NO raise)':<22} VERDICT")
    print("-" * 88)

    all_good = True
    for (name, _), (_, _) in zip(trigger_list(fixed), trigger_list(buggy)):
        # Re-create lambdas bound to the right instance each pass
        fixed_trig = dict(trigger_list(fixed))[name]
        buggy_trig = dict(trigger_list(buggy))[name]

        fixed_raises = assertion_after_trigger(fixed, fixed_trig)
        buggy_raises = assertion_after_trigger(buggy, buggy_trig)

        # We want: fixed code RAISES (assertion holds, test passes)
        #          buggy code does NOT raise (assertion fails, test catches bug)
        has_teeth = fixed_raises and not buggy_raises
        all_good = all_good and has_teeth

        verdict = "TEST HAS TEETH" if has_teeth else "WEAK / NO SIGNAL"
        print(f"{name:<26} {str(fixed_raises):<20} {str(buggy_raises):<22} {verdict}")

    print("-" * 88)
    if all_good:
        print("\nRESULT: PASS")
        print("For every trigger, the assertion PASSES on fixed code and FAILS on")
        print("buggy code. The cache tests genuinely detect the regression -- they")
        print("are not rigged to pass unconditionally.")
        sys.exit(0)
    else:
        print("\nRESULT: FAIL")
        print("At least one trigger did not distinguish fixed from buggy code.")
        sys.exit(1)


if __name__ == "__main__":
    main()
