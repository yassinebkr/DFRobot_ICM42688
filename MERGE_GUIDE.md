# Cache Fix Testing & Merge Guide

Branch: `claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9`
PR: https://github.com/yassinebkr/DFRobot_ICM42688/pull/10

## What Changed

### CircuitPython Library
- **Fixed cache poisoning bug**: Cache now invalidates on every config change
- **New API**: `invalidate_cache()` method
- **Auto-invalidation** on: range changes, ODR changes, power mode, FIFO, reset, filters
- **Comprehensive test suite**: 38 tests covering all invalidation triggers

### Arduino Library  
- **New cached read API**: `refreshSensorData()` + `getCached*()` methods
- **Backward compatible**: Original getters unchanged
- **Fixed bank-selection** issues in bulk read methods
- **New examples**: `cachedRead.ino`, `bulkFIFORead.ino`

---

## Testing Steps

### 1. CircuitPython Testing (Desktop - No Hardware)

Run all three mock tests to validate the cache logic:

```bash
cd circuitpython/tests

# Main logic test (21 checks)
python3 mock_logic_test.py

# Full test suite (38 tests)
python3 -m pytest test_cache_invalidation.py -v

# Negative control (proves tests catch bugs)
python3 mock_negative_control.py
```

**Expected**: All tests PASS

### 2. CircuitPython Testing (On Hardware - Optional)

If you have an ESP32 + ICM42688:

```bash
cd circuitpython/tests

# Deploy and run hardware test
./deploy_and_test.sh

# Or manually copy test_cache_invalidation.py to CIRCUITPY and check serial output
```

**Expected**: 38/38 tests pass on the device

### 3. Arduino Testing (On Hardware - Recommended)

**Option A: Cached Read Pattern**
1. Open `arduino/examples/cachedRead/cachedRead.ino` in Arduino IDE
2. Upload to your board (ESP32/Arduino)
3. Open Serial Monitor (115200 baud)
4. Verify:
   - Single I2C transaction per refresh
   - Cached values match bulk values
   - Conditional axis reads work without extra I2C traffic

**Option B: Bulk FIFO Read**
1. Open `arduino/examples/bulkFIFORead/bulkFIFORead.ino`
2. Upload and monitor
3. Verify FIFO packet parsing works correctly

**Option C: Quick Compile Check** (if no hardware)
```bash
# Install arduino-cli if needed
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Compile all examples
cd arduino/examples
for example in */; do
  arduino-cli compile --fqbn esp32:esp32:esp32 "$example" || echo "FAIL: $example"
done
```

---

## Merge to Main

Once testing passes:

### Option 1: GitHub UI (Recommended)
1. Go to https://github.com/yassinebkr/DFRobot_ICM42688/pull/10
2. Review the changes
3. Click "Merge pull request"
4. Confirm merge
5. Delete the branch (optional cleanup)

### Option 2: Command Line
```bash
# Switch to main and update
git checkout main
git pull origin main

# Merge the fix branch
git merge claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9

# Push to main
git push origin main

# Clean up (optional)
git branch -d claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9
git push origin --delete claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9
```

---

## Quick Validation Checklist

- [ ] Desktop mock tests pass (CircuitPython)
- [ ] Arduino examples compile without errors
- [ ] (Optional) Hardware tests pass on real device
- [ ] PR #10 shows all commits
- [ ] No merge conflicts with main
- [ ] Ready to merge

---

## Rollback Plan (If Issues Found)

If problems are discovered after merge:

```bash
# Find the merge commit
git log --oneline -5

# Revert the merge (replace MERGE_SHA with actual hash)
git revert -m 1 MERGE_SHA

# Push the revert
git push origin main
```

Then investigate and re-apply fixes on a new branch.
