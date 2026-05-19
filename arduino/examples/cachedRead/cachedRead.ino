/*!
 * @file cachedRead.ino
 * @brief Demonstrates efficient cached reading for per-axis access
 *
 * This example shows two patterns:
 * 1. refreshSensorData() + per-axis getters (efficient per-axis access)
 * 2. getAllSensorData() (single read for all axes)
 *
 * Both patterns avoid redundant sensor reads when accessing multiple axes.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author Yassine Benkhira
 * @version  V1.1
 * @date  2025-01-20
 * @url https://github.com/yassinebkr/DFRobot_ICM42688
 */

#include <DFRobot_ICM42688.h>

// Use I2C communication
DFRobot_ICM42688_I2C icm(&Wire, DFRobot_ICM42688_I2C_H_ADDR);

// Or use SPI communication
// DFRobot_ICM42688_SPI icm(/*csPin=*/5);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ICM42688 Cached Read Example");
  Serial.println("=============================");

  // Initialize sensor
  while (icm.begin() != 0) {
    Serial.println("Failed to initialize sensor! Please check connection.");
    delay(1000);
  }
  Serial.println("✓ Sensor initialized");

  // Configure sensor: 1kHz ODR, ±16g accel, ±2000dps gyro, Low-Noise mode
  icm.setODRAndFSR(ALL, ODR_1KHZ, FSR_0);
  icm.startAccelMeasure(LN_MODE);
  icm.startGyroMeasure(LN_MODE);

  Serial.println("\nDemonstrating two efficient reading patterns:");
  Serial.println("1. refreshSensorData() + per-axis getters");
  Serial.println("2. getAllSensorData() (all-in-one)\n");

  delay(1000);
}

void loop() {
  // =========================================================================
  // Pattern 1: Cached Read (efficient per-axis access)
  // =========================================================================
  Serial.println("═══ Pattern 1: Cached Read ═══");

  // Call refreshSensorData() ONCE
  icm.refreshSensorData();

  // Now you can access any axes without additional I2C/SPI reads
  // This is efficient when you don't need all axes every time
  float ax = icm.getAccelDataX();  // No I2C read (uses cache)
  float ay = icm.getAccelDataY();  // No I2C read (uses cache)
  float az = icm.getAccelDataZ();  // No I2C read (uses cache)

  Serial.print("Accel: X=");
  Serial.print(ax, 1);
  Serial.print(" mg, Y=");
  Serial.print(ay, 1);
  Serial.print(" mg, Z=");
  Serial.print(az, 1);
  Serial.println(" mg");

  // You can also conditionally access axes
  if (abs(ax) > 100) {  // Only read gyro if X accel exceeds threshold
    float gx = icm.getGyroDataX();  // No I2C read (uses cache)
    Serial.print("  → X motion detected! Gyro X = ");
    Serial.print(gx, 2);
    Serial.println(" dps");
  }

  delay(500);

  // =========================================================================
  // Pattern 2: Single All-Data Read (most efficient when you need all axes)
  // =========================================================================
  Serial.println("\n═══ Pattern 2: All-Data Read ═══");

  float ax2, ay2, az2, gx2, gy2, gz2, temp;

  // Single I2C/SPI read gets everything (14 bytes)
  icm.getAllSensorData(ax2, ay2, az2, gx2, gy2, gz2, temp);

  Serial.print("Accel: (");
  Serial.print(ax2, 1);
  Serial.print(", ");
  Serial.print(ay2, 1);
  Serial.print(", ");
  Serial.print(az2, 1);
  Serial.println(") mg");

  Serial.print("Gyro:  (");
  Serial.print(gx2, 2);
  Serial.print(", ");
  Serial.print(gy2, 2);
  Serial.print(", ");
  Serial.print(gz2, 2);
  Serial.println(") dps");

  Serial.print("Temp:  ");
  Serial.print(temp, 1);
  Serial.println(" °C");

  delay(500);

  // =========================================================================
  // Performance Comparison (informational)
  // =========================================================================
  Serial.println("\n═══ Performance Notes ═══");
  Serial.println("✓ Pattern 1: Flexible per-axis access, single read");
  Serial.println("✓ Pattern 2: All axes at once, cleanest code");
  Serial.println("✗ OLD WAY (don't do this):");
  Serial.println("  ax = getAccelDataX(); // Read 1");
  Serial.println("  ay = getAccelDataY(); // Read 2");
  Serial.println("  az = getAccelDataZ(); // Read 3");
  Serial.println("  → 3x slower! (3 separate I2C transactions)\n");

  delay(2000);
}
