/*!
 * @file bulkFIFORead.ino
 * @brief Demonstrates high-performance bulk FIFO reading for data logging
 *
 * This example shows how to use getFIFODataBulk() for 10-100x faster data logging
 * compared to reading one packet at a time with getFIFOData().
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

  Serial.println("ICM42688 Bulk FIFO Read Example");
  Serial.println("================================");

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

  // Enable FIFO
  icm.startFIFOMode();
  Serial.println("✓ FIFO enabled (1kHz ODR)");

  Serial.println("\nCollecting data for 5 seconds...");
  Serial.println("This will demonstrate bulk reading for data logging.\n");

  delay(1000);
}

void loop() {
  // Wait for data to accumulate (100ms = ~100 packets at 1kHz)
  delay(100);

  // Check how much data is available
  uint16_t fifoBytes = icm.getFIFOCount();
  uint16_t availablePackets = fifoBytes / 16;

  if (availablePackets == 0) {
    return;  // No data yet
  }

  // Allocate buffer for bulk read (max 128 packets = 2KB FIFO)
  const uint16_t MAX_PACKETS = 128;
  sFIFOPacket_t packets[MAX_PACKETS];

  // Bulk read all packets at once (10-100x faster than loop!)
  unsigned long startTime = micros();
  uint16_t numRead = icm.getFIFODataBulk(packets, MAX_PACKETS);
  unsigned long elapsedTime = micros() - startTime;

  // Display results
  Serial.print("📦 Read ");
  Serial.print(numRead);
  Serial.print(" packets in ");
  Serial.print(elapsedTime);
  Serial.print(" µs (");
  Serial.print((float)elapsedTime / numRead, 2);
  Serial.println(" µs/packet)");

  // Show first and last packet
  if (numRead > 0) {
    Serial.print("   First: Accel(");
    Serial.print(packets[0].accelX, 1);
    Serial.print(", ");
    Serial.print(packets[0].accelY, 1);
    Serial.print(", ");
    Serial.print(packets[0].accelZ, 1);
    Serial.println(") mg");

    Serial.print("   Last:  Accel(");
    Serial.print(packets[numRead-1].accelX, 1);
    Serial.print(", ");
    Serial.print(packets[numRead-1].accelY, 1);
    Serial.print(", ");
    Serial.print(packets[numRead-1].accelZ, 1);
    Serial.println(") mg");

    // Calculate average accelerometer magnitude
    float avgMag = 0;
    for (uint16_t i = 0; i < numRead; i++) {
      float ax = packets[i].accelX;
      float ay = packets[i].accelY;
      float az = packets[i].accelZ;
      avgMag += sqrt(ax*ax + ay*ay + az*az);
    }
    avgMag /= numRead;

    Serial.print("   Avg magnitude: ");
    Serial.print(avgMag, 1);
    Serial.println(" mg (expect ~1000 mg = 1g when stationary)");
  }

  Serial.println();

  // In a real application, you would:
  // 1. Write packets[] to SD card in bulk
  // 2. Send via WiFi/BLE in batches
  // 3. Process with signal analysis
  // 4. Log for ML training data collection
}
