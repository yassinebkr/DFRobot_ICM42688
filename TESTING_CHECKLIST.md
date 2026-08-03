# CircuitPython Cache Fix Testing Checklist

**Branch**: `claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9`  
**Date**: 2026-05-20  
**Hardware**: RP2040 + ICM42688 (I2C)

## Pre-Testing Setup

- [ ] Branch checked out locally
      ```bash
      git checkout claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9
      ```

- [ ] RP2040 board connected via USB

- [ ] CIRCUITPY drive mounted at `/media/CIRCUITPY`

- [ ] ICM42688 sensor wired to I2C (3.3V, GND, SCL, SDA)

- [ ] Serial monitor ready
      ```bash
      screen /dev/ttyACM0 115200
      # Or: python3 -m serial.tools.miniterm /dev/ttyACM0 115200
      ```

## Test Execution (In Order)

### 1. Hardware Connection Verification

- [ ] Deploy and run hardware check
      ```bash
      cd circuitpython
      ./deploy_and_test.sh tests/hardware_check.py
      ```

- [ ] Verify output shows:
      - ✅ I2C bus initialized
      - ✅ ICM42688 detected at 0x68 or 0x69
      - ✅ Library initialized
      - ✅ Sensor responding
      - ✅ Data sanity checks passed

- [ ] **Result**: ⬜ PASS / ⬜ FAIL

**If FAIL**: Stop here, fix wiring/connections, retry


### 2. Manual Interactive Testing

- [ ] Deploy and run manual test
      ```bash
      ./deploy_and_test.sh tests/manual_cache_test.py
      ```

- [ ] Step through each test with Enter key

- [ ] Verify each test shows ✅ PASS:
      - [ ] Test 1: Initial cache state (RuntimeError expected)
      - [ ] Test 2: refresh() populates cache
      - [ ] Test 3: accelerometer_range invalidation
      - [ ] Test 4: FIFO enable/disable invalidation
      - [ ] Test 5: Non-cached APIs work
      - [ ] Test 6: Real-world usage pattern

- [ ] **Result**: ⬜ PASS / ⬜ FAIL

**If FAIL**: Note which test failed, check error message


### 3. Comprehensive Automated Testing

- [ ] Deploy and run full test suite
      ```bash
      ./deploy_and_test.sh tests/test_cache_invalidation.py
      ```

- [ ] Wait for completion (~30-45 seconds)

- [ ] Verify final output:
      ```
      Test Summary: 33/33 passed
      ✅ ALL TESTS PASSED!
      ```

- [ ] **Result**: ⬜ PASS / ⬜ FAIL
      - Tests passed: ___/33
      - Tests failed: ___/33

**If FAIL**: Review failed test names and reasons in output


### 4. Regression Testing (Existing Examples)

Test that existing examples still work with the fix:

- [ ] aerospace_filters_demo.py
      ```bash
      ./deploy_and_test.sh examples/aerospace_filters_demo.py
      ```
      **Result**: ⬜ PASS / ⬜ FAIL

- [ ] manual_refresh_mode.py
      ```bash
      ./deploy_and_test.sh examples/manual_refresh_mode.py
      ```
      **Result**: ⬜ PASS / ⬜ FAIL

- [ ] fifo_bulk_read_demo.py
      ```bash
      ./deploy_and_test.sh examples/fifo_bulk_read_demo.py
      ```
      **Result**: ⬜ PASS / ⬜ FAIL

- [ ] icm42688_simpletest.py
      ```bash
      ./deploy_and_test.sh examples/icm42688_simpletest.py
      ```
      **Result**: ⬜ PASS / ⬜ FAIL


### 5. Specific Cache Invalidation Scenarios

Manually verify critical scenarios:

- [ ] **Scenario A: Range change during operation**
      1. Run continuous read loop
      2. Change accelerometer_range while running
      3. Verify RuntimeError raised
      4. Call refresh()
      5. Verify reading resumes with correct scale

- [ ] **Scenario B: FIFO mode switching**
      1. Start with FIFO disabled
      2. Enable FIFO mid-operation
      3. Verify cache invalidated
      4. Disable FIFO
      5. Verify cache invalidated again

- [ ] **Scenario C: Power mode transitions**
      1. Start in low_noise mode
      2. Switch to low_power mode
      3. Verify cache invalidated
      4. Switch back to low_noise
      5. Verify cache invalidated

- [ ] **Scenario D: Reset during operation**
      1. Run continuous read loop
      2. Call reset()
      3. Verify cache invalidated
      4. Re-initialize and verify recovery


## Edge Case Testing

- [ ] **Multiple rapid range changes**
      - Change accelerometer_range 10 times quickly
      - Verify each change invalidates cache
      - **Result**: ⬜ PASS / ⬜ FAIL

- [ ] **Interleaved cached and non-cached reads**
      - Call refresh()
      - Read accel_x (cached)
      - Read acceleration (non-cached)
      - Read accel_x again (should still be cached)
      - **Result**: ⬜ PASS / ⬜ FAIL

- [ ] **Cache after configuration sequence**
      - Change multiple settings in sequence
      - Verify single refresh() populates cache correctly
      - **Result**: ⬜ PASS / ⬜ FAIL


## Performance Verification

- [ ] **Cached read performance**
      - Time 1000 iterations of: refresh() + 6 cached reads
      - Expected: 150-200 Hz (5-6.5ms per iteration)
      - Actual: _____ Hz (_____ ms per iteration)
      - **Result**: ⬜ PASS / ⬜ FAIL

- [ ] **Non-cached read performance**
      - Time 1000 iterations of: all_data property
      - Expected: ~150 Hz (6-7ms per iteration)
      - Actual: _____ Hz (_____ ms per iteration)
      - **Result**: ⬜ PASS / ⬜ FAIL


## Final Verification

- [ ] All automated tests passed (33/33)

- [ ] All manual tests passed

- [ ] All existing examples work without regression

- [ ] All edge cases behave correctly

- [ ] Performance is acceptable

- [ ] No unexpected errors or warnings in serial output

- [ ] Code review: changes match intended fix
      - `invalidate_cache()` method added
      - 7 setters call `self._cache_valid = False`
      - Docstrings updated appropriately


## Sign-Off

**Tested by**: ___________________  
**Date**: ___________________  
**Hardware**: RP2040 + ICM42688  
**Branch**: claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9  

**Overall Result**: ⬜ APPROVED FOR MERGE / ⬜ NEEDS FIXES

**Notes**:
___________________________________________________________________________
___________________________________________________________________________
___________________________________________________________________________


## Merge Criteria

Branch is ready to merge when:

✅ Hardware check passes  
✅ Manual test passes (all 6 tests)  
✅ Automated test passes (33/33)  
✅ All example programs run without errors  
✅ Edge cases verified  
✅ Performance acceptable  
✅ Code review complete  

## After Merge

Once merged to main, this fix should be incorporated into the Arduino branch:
- Arduino branch: `claude/add-aerospace-filters-01W3ZLWLe4RqG3if7ugvoZA9`
- Arduino needs similar cache invalidation in cached API methods
- Test Arduino implementation before final merge

---

**Next Steps After This Branch**:
1. ✅ Merge CircuitPython fix to main
2. ⏭ Test Arduino cache fix branch
3. ⏭ Merge Arduino enhancements to main
4. ⏭ Port features to MicroPython ESP32
5. ⏭ Evaluate Python/Raspberry Pi needs
