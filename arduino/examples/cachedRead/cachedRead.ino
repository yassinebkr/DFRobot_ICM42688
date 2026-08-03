/*!
 * @file cachedRead.ino
 * @brief Demonstrates efficient cached reading for per-axis access.
 *
 * Two patterns are shown:
 * 1. refreshSensorData() + getCachedAccelX/Y/Z + getCachedGyroX/Y/Z
 *    Single transaction, then access any axes from the cache.
 *
 * 2. getAllSensorData() - single transaction, all axes returned by reference.
 *
 * Both avoid the redundant per-axis I2C/SPI traffic of calling six separate
 * getAccelDataX/Y/Z / getGyroDataX/Y/Z methods in a row.
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

  while (icm.begin() != 0) {
    Serial.println("Failed to initialize sensor! Please check connection.");
    delay(1000);
  }
  Serial.println("Sensor initialized");

  icm.setODRAndFSR(ALL, ODR_1KHZ, FSR_0);
  icm.startAccelMeasure(LN_MODE);
  icm.startGyroMeasure(LN_MODE);

  Serial.println("\nTwo efficient reading patterns:");
  Serial.println("  1. refreshSensorData() + getCached*()");
  Serial.println("  2. getAllSensorData()");
  Serial.println();
  delay(1000);
}

void loop() {
  // ----- Pattern 1: cached read --------------------------------------------
  // Refresh the cache once per loop iteration, then read whichever axes you
  // need. Each getCached*() call returns the value from this single refresh.
  icm.refreshSensorData();

  float ax = icm.getCachedAccelX();
  float ay = icm.getCachedAccelY();
  float az = icm.getCachedAccelZ();

  Serial.print("[cached] Accel: X=");
  Serial.print(ax, 1);
  Serial.print(" Y=");
  Serial.print(ay, 1);
  Serial.print(" Z=");
  Serial.print(az, 1);
  Serial.println(" mg");

  // Conditional axis access reuses the same cache - no extra I2C traffic.
  if (abs(ax) > 100) {
    float gx = icm.getCachedGyroX();
    Serial.print("  -> motion on X, gyro X = ");
    Serial.print(gx, 2);
    Serial.println(" dps");
  }

  delay(500);

  // ----- Pattern 2: getAllSensorData ---------------------------------------
  // Cleanest option when you always need every axis: one call returns all
  // seven values via output references (single 14-byte transaction).
  float ax2, ay2, az2, gx2, gy2, gz2, temp;
  icm.getAllSensorData(ax2, ay2, az2, gx2, gy2, gz2, temp);

  Serial.print("[bulk]   Accel=(");
  Serial.print(ax2, 1); Serial.print(", ");
  Serial.print(ay2, 1); Serial.print(", ");
  Serial.print(az2, 1); Serial.print(") mg  Gyro=(");
  Serial.print(gx2, 2); Serial.print(", ");
  Serial.print(gy2, 2); Serial.print(", ");
  Serial.print(gz2, 2); Serial.print(") dps  Temp=");
  Serial.print(temp, 1); Serial.println(" C");

  Serial.println();

  // Note: getAccelDataX/Y/Z and getGyroDataX/Y/Z still work exactly as
  // before - each call performs its own 2-byte read. Use them when you only
  // need one axis at a time. For multiple axes per sample, prefer the
  // patterns above.

  delay(1500);
}
