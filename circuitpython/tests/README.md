# CircuitPython ICM42688 Test Suite

Comprehensive test suite for the CircuitPython ICM42688 library, specifically testing the cache invalidation fix in branch `claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9`.

## Hardware Requirements

- **Board**: RP2040 (e.g., Adafruit Feather RP2040)
- **Sensor**: ICM42688 (TDK InvenSense ICM-42688-P)
- **Connection**: I2C (400 kHz recommended)
- **CircuitPython**: Version 8.0.0 or later

## Wiring

See [../examples/FEATHER_RP2040_WIRING_GUIDE.md](../examples/FEATHER_RP2040_WIRING_GUIDE.md) for detailed wiring instructions.

Quick reference:
```
ICM42688    →    RP2040 (Feather)
─────────────────────────────────
VCC (3.3V)  →    3.3V
GND         →    GND
SCL         →    SCL (GPIO 3)
SDA         →    SDA (GPIO 2)
```

## Test Scripts

### 1. `hardware_check.py` — Hardware Connection Verification

**Purpose**: Verify sensor is properly connected before running tests

**Usage**:
```bash
# Deploy to board and run
./deploy_and_test.sh tests/hardware_check.py

# Monitor serial output
screen /dev/ttyACM0 115200
```

**What it checks**:
- I2C bus initialization
- I2C device scanning
- ICM42688 detection at correct address (0x68 or 0x69)
- Library import and initialization
- Sensor communication (reads temperature, accel, gyro)
- Data sanity checks (values in reasonable ranges)

**Expected output**:
```
✅ All checks passed!
```

### 2. `manual_cache_test.py` — Interactive Manual Testing

**Purpose**: Quick interactive verification of cache invalidation behavior

**Usage**:
```bash
./deploy_and_test.sh tests/manual_cache_test.py
screen /dev/ttyACM0 115200
# Press Enter to step through each test
```

**What it tests**:
1. Cache invalid initially (RuntimeError before refresh)
2. refresh() populates cache
3. accelerometer_range change invalidates cache
4. FIFO enable/disable invalidates cache
5. Non-cached APIs work without refresh()
6. Real-world continuous reading pattern

**Duration**: ~2 minutes (with user input)

### 3. `test_cache_invalidation.py` — Comprehensive Automated Testing

**Purpose**: Full automated test suite covering all cache invalidation scenarios

**Usage**:
```bash
./deploy_and_test.sh tests/test_cache_invalidation.py
screen /dev/ttyACM0 115200
# Tests run automatically
```

**What it tests**:

| Test Group | Tests | Focus |
|------------|-------|-------|
| **Test 1: Basic Cache API** | 4 tests | Cache lifecycle, refresh(), multiple reads |
| **Test 2: Accel Range** | 2 tests | Invalidation on accelerometer_range change |
| **Test 3: Gyro Range** | 2 tests | Invalidation on gyro_range change |
| **Test 4: Accel ODR** | 2 tests | Invalidation on accelerometer_data_rate change |
| **Test 5: Gyro ODR** | 2 tests | Invalidation on gyro_data_rate change |
| **Test 6: Power Mode** | 2 tests | Invalidation on set_power_mode() |
| **Test 7: Reset** | 2 tests | Invalidation on reset() |
| **Test 8: FIFO** | 4 tests | Invalidation on enable_fifo()/disable_fifo() |
| **Test 9: all_data** | 2 tests | Non-cached API behavior |
| **Test 10: acceleration/gyro** | 4 tests | Non-cached tuple properties |
| **Test 11: Edge Cases** | 4 tests | Multiple refresh, interleaved reads, manual invalidation |
| **Test 12: Real-World** | 3 tests | Common usage patterns |

**Total**: 33 automated tests

**Duration**: ~30-45 seconds (fully automated)

**Expected output**:
```
Test Summary: 33/33 passed
✅ ALL TESTS PASSED!
```

## Running Tests

### Quick Start (Recommended Order)

1. **Hardware Check First**:
   ```bash
   cd circuitpython
   ./deploy_and_test.sh tests/hardware_check.py
   ```
   Wait for "✅ All checks passed!"

2. **Manual Interactive Test**:
   ```bash
   ./deploy_and_test.sh tests/manual_cache_test.py
   ```
   Step through each test with Enter key

3. **Full Automated Test Suite**:
   ```bash
   ./deploy_and_test.sh tests/test_cache_invalidation.py
   ```
   Wait for "✅ ALL TESTS PASSED!"

### Manual Deployment (Alternative)

If you prefer manual deployment:

1. **Mount the board**:
   ```bash
   # Board should auto-mount as /media/CIRCUITPY
   # Or manually:
   sudo mkdir -p /media/CIRCUITPY
   sudo mount /dev/sdX1 /media/CIRCUITPY  # Replace sdX1
   ```

2. **Copy library**:
   ```bash
   cp -r adafruit_icm42688 /media/CIRCUITPY/lib/
   ```

3. **Copy test as code.py**:
   ```bash
   cp tests/hardware_check.py /media/CIRCUITPY/code.py
   ```

4. **Sync and monitor**:
   ```bash
   sync
   screen /dev/ttyACM0 115200
   # Board will auto-reboot and run code.py
   ```

## What the Fix Addresses

### Original Bug (Cache Poisoning)

In the original implementation:
```python
# After one refresh(), cache stayed valid FOREVER
icm.refresh()
x = icm.accel_x  # Correct value

icm.accelerometer_range = 8  # Changes scale factor!
x = icm.accel_x  # Still returns OLD cached value with WRONG scale
                  # Data is INCORRECT but no error raised
```

### Fixed Behavior

With the cache invalidation fix:
```python
icm.refresh()
x = icm.accel_x  # Correct value

icm.accelerometer_range = 8  # Changes scale factor
# Cache automatically invalidated!

x = icm.accel_x  # RuntimeError: "Cache invalid, call refresh() first"
                  # Forces user to refresh with new scale

icm.refresh()    # Read with new scale
x = icm.accel_x  # Correct value with new scale
```

### All 7 Auto-Invalidation Triggers

The cache is automatically invalidated when:

1. `accelerometer_range` is changed (scale factor changes)
2. `gyro_range` is changed (scale factor changes)
3. `accelerometer_data_rate` is changed (data path changes)
4. `gyro_data_rate` is changed (data path changes)
5. `set_power_mode()` is called (data path changes)
6. `reset()` is called (everything changes)
7. `enable_fifo()` or `disable_fifo()` is called (data path changes)

This ensures cached values are never stale after configuration changes.

## Test Results Interpretation

### All Tests Pass
```
Test Summary: 33/33 passed
✅ ALL TESTS PASSED!
```
**Action**: Cache invalidation fix is working correctly. Ready to merge.

### Some Tests Fail
```
Test Summary: 28/33 passed
❌ SOME TESTS FAILED

Failed tests:
  • 2.1: accelerometer_range invalidation: Expected RuntimeError after range change
```

**Action**: Review failures:
- If cache invalidation triggers are missing → bug in fix
- If test expects wrong behavior → bug in test
- If hardware issues → check connections, re-run hardware_check.py

### Fatal Errors
```
Failed to initialize sensor: OSError: [Errno 19] No such device
```

**Action**: Run `hardware_check.py` to diagnose:
- I2C bus issues
- Sensor not detected
- Wiring problems
- Power issues

## Troubleshooting

### Board Not Detected
```bash
ls /dev/ttyACM*        # Should show /dev/ttyACM0
ls /media/CIRCUITPY/   # Should show lib/ directory
```

If not found:
- Reconnect USB cable
- Press reset button on board
- Check `dmesg` for USB events
- Verify CircuitPython is installed (not Arduino bootloader)

### Library Not Found
```
ImportError: no module named 'adafruit_icm42688'
```

Check:
```bash
ls /media/CIRCUITPY/lib/
# Should show: adafruit_icm42688/
```

Re-deploy if missing:
```bash
cp -r adafruit_icm42688 /media/CIRCUITPY/lib/
sync
```

### Sensor Not Responding

Run `hardware_check.py` first. Common issues:
- **Wrong address**: Check AD0 pin (GND = 0x68, VCC = 0x69)
- **No pull-ups**: I2C needs 4.7kΩ pull-ups on SCL/SDA (usually built-in)
- **5V sensor on 3.3V I2C**: Use level shifter or 3.3V sensor
- **Loose connections**: Check all 4 wires (VCC, GND, SCL, SDA)

### Tests Hang or Timeout

- Press Ctrl+C to stop
- Press board reset button
- Check serial output for error messages
- Run `hardware_check.py` to verify sensor communication

## Success Criteria

Before merging the branch, all three test scripts must pass:

✅ `hardware_check.py` — All checks passed  
✅ `manual_cache_test.py` — All manual tests passed  
✅ `test_cache_invalidation.py` — 33/33 tests passed  

Once confirmed, the branch is ready to merge to main.

## Additional Testing

After automated tests pass, also test existing examples for regressions:

```bash
./deploy_and_test.sh examples/aerospace_filters_demo.py
./deploy_and_test.sh examples/manual_refresh_mode.py
./deploy_and_test.sh examples/fifo_bulk_read_demo.py
```

All examples should run without errors.

## Support

For issues or questions:
- Check [../examples/FEATHER_RP2040_WIRING_GUIDE.md](../examples/FEATHER_RP2040_WIRING_GUIDE.md)
- Review [../../docs/SAFETY_CHECK_PROCEDURE.md](../../docs/SAFETY_CHECK_PROCEDURE.md)
- See [../../docs/FEATHER_RP2040_QUICKSTART.md](../../docs/FEATHER_RP2040_QUICKSTART.md)

## Test Development

To add new tests:

1. Add test function to `test_cache_invalidation.py`:
   ```python
   def test_my_new_feature(icm, results):
       """Test description"""
       print_header("Test X: My New Feature")
       try:
           # Test code here
           results.record_pass("X.1: Sub-test description")
       except Exception as e:
           results.record_fail("X.1: Sub-test description", str(e))
   ```

2. Call from `run_all_tests()`:
   ```python
   test_my_new_feature(icm, results)
   ```

3. Deploy and run to verify.

---

**Branch**: `claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9`  
**Date**: 2026-05-20  
**Author**: Cache invalidation fix test suite
