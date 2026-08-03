#!/bin/bash
# Deploy CircuitPython library and run tests on RP2040
#
# Usage: ./deploy_and_test.sh [test_file]
#
# If no test_file specified, deploys library only.
# If test_file specified, deploys library and runs the test.

set -e

CIRCUITPY="/media/CIRCUITPY"
LIB_SRC="./adafruit_icm42688"
TEST_FILE="$1"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================="
echo "  CircuitPython Deployment Script"
echo "========================================="
echo ""

# Check if CIRCUITPY is mounted
if [ ! -d "$CIRCUITPY" ]; then
    echo -e "${RED}Error: CIRCUITPY drive not found at $CIRCUITPY${NC}"
    echo ""
    echo "Please:"
    echo "  1. Connect your RP2040 board via USB"
    echo "  2. Ensure CircuitPython is installed"
    echo "  3. The board should appear as /media/CIRCUITPY"
    echo ""
    echo "To manually mount:"
    echo "  sudo mkdir -p /media/CIRCUITPY"
    echo "  sudo mount /dev/sdX1 /media/CIRCUITPY  # Replace sdX1 with your device"
    echo ""
    exit 1
fi

echo -e "${GREEN}✓${NC} CIRCUITPY drive found"

# Create lib directory if it doesn't exist
if [ ! -d "$CIRCUITPY/lib" ]; then
    echo "Creating lib directory..."
    mkdir -p "$CIRCUITPY/lib"
fi

# Deploy library
echo ""
echo "Deploying library..."
echo "  Source: $LIB_SRC"
echo "  Target: $CIRCUITPY/lib/adafruit_icm42688"

# Remove old version
if [ -d "$CIRCUITPY/lib/adafruit_icm42688" ]; then
    rm -rf "$CIRCUITPY/lib/adafruit_icm42688"
fi

# Copy new version
cp -r "$LIB_SRC" "$CIRCUITPY/lib/"
echo -e "${GREEN}✓${NC} Library deployed"

# If test file specified, deploy and run it
if [ -n "$TEST_FILE" ]; then
    echo ""
    echo "Deploying test script..."
    echo "  Source: $TEST_FILE"
    echo "  Target: $CIRCUITPY/code.py"

    # Copy test as code.py (auto-runs on boot)
    cp "$TEST_FILE" "$CIRCUITPY/code.py"
    echo -e "${GREEN}✓${NC} Test script deployed as code.py"

    echo ""
    echo "Syncing filesystem..."
    sync
    sleep 1

    echo ""
    echo -e "${YELLOW}Board will auto-reboot and run the test.${NC}"
    echo "Monitor serial output to see results:"
    echo "  screen /dev/ttyACM0 115200"
    echo "  OR"
    echo "  python3 -m serial.tools.miniterm /dev/ttyACM0 115200"
    echo ""
else
    echo ""
    echo "Syncing filesystem..."
    sync

    echo ""
    echo -e "${GREEN}✓${NC} Deployment complete"
    echo ""
    echo "Library is now available on the board."
    echo "To run a test, use:"
    echo "  ./deploy_and_test.sh tests/test_cache_invalidation.py"
    echo ""
fi
