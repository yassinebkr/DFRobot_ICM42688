# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Register definitions and constants for the ICM42688 6-axis IMU.

This module contains all register addresses, bit masks, and configuration
constants for the TDK InvenSense ICM42688 IMU sensor.

* Author(s): Yassine Bekkari (based on DFRobot_ICM42688)
"""

# I2C Addresses
ICM42688_I2C_ADDR_LOW = 0x68  # SDO pin pulled low
ICM42688_I2C_ADDR_HIGH = 0x69  # SDO pin pulled high (default)

# Chip ID
ICM42688_CHIP_ID = 0x47  # Expected value from WHO_AM_I register

# ========================================================================
# BANK 0 REGISTERS (Primary bank - default after reset)
# ========================================================================

# Device configuration and control
REG_DEVICE_CONFIG = 0x11  # Device configuration register
REG_DRIVE_CONFIG = 0x13  # Drive strength configuration
REG_SIGNAL_PATH_RESET = 0x4B  # Signal path reset
REG_PWR_MGMT0 = 0x4E  # Power management 0

# Interrupt registers
REG_INT_CONFIG = 0x14  # Interrupt pin configuration
REG_INT_STATUS = 0x2D  # Interrupt status (read to clear)
REG_INT_STATUS2 = 0x37  # Interrupt status 2 (SMD, WOM)
REG_INT_STATUS3 = 0x38  # Interrupt status 3 (TAP, TILT, etc.)
REG_INT_CONFIG0 = 0x63  # Interrupt configuration 0
REG_INT_CONFIG1 = 0x64  # Interrupt configuration 1

# Interrupt source routing registers
REG_INT_SOURCE0 = 0x65  # INT source for UI functions
REG_INT_SOURCE1 = 0x66  # INT1 source for wake events
REG_INT_SOURCE3 = 0x68  # INT2 source for UI functions
REG_INT_SOURCE4 = 0x69  # INT2 source for wake events
REG_INT_SOURCE6 = 0x4D  # INT1 source for APEX
REG_INT_SOURCE7 = 0x4E  # INT2 source for APEX

# Temperature data registers (big-endian, 16-bit signed)
REG_TEMP_DATA1 = 0x1D  # Temperature data MSB
REG_TEMP_DATA0 = 0x1E  # Temperature data LSB

# Accelerometer data registers (big-endian, 16-bit signed)
REG_ACCEL_DATA_X1 = 0x1F  # X-axis MSB
REG_ACCEL_DATA_X0 = 0x20  # X-axis LSB
REG_ACCEL_DATA_Y1 = 0x21  # Y-axis MSB
REG_ACCEL_DATA_Y0 = 0x22  # Y-axis LSB
REG_ACCEL_DATA_Z1 = 0x23  # Z-axis MSB
REG_ACCEL_DATA_Z0 = 0x24  # Z-axis LSB

# Gyroscope data registers (big-endian, 16-bit signed)
REG_GYRO_DATA_X1 = 0x25  # X-axis MSB
REG_GYRO_DATA_X0 = 0x26  # X-axis LSB
REG_GYRO_DATA_Y1 = 0x27  # Y-axis MSB
REG_GYRO_DATA_Y0 = 0x28  # Y-axis LSB
REG_GYRO_DATA_Z1 = 0x29  # Z-axis MSB
REG_GYRO_DATA_Z0 = 0x2A  # Z-axis LSB

# Timestamp registers
REG_TMST_FSYNCH = 0x43  # Timestamp FSYNC high byte
REG_TMST_FSYNCL = 0x44  # Timestamp FSYNC low byte

# Sensor configuration registers
REG_GYRO_CONFIG0 = 0x4F  # Gyro ODR and full-scale range
REG_ACCEL_CONFIG0 = 0x50  # Accel ODR and full-scale range
REG_GYRO_CONFIG1 = 0x51  # Gyro filter configuration
REG_GYRO_ACCEL_CONFIG0 = 0x52  # Combined filter config
REG_ACCEL_CONFIG1 = 0x53  # Accel filter configuration
REG_TMST_CONFIG = 0x54  # Timestamp configuration

# FIFO registers
REG_FIFO_CONFIG = 0x16  # FIFO mode configuration
REG_FIFO_COUNTH = 0x2E  # FIFO count high byte
REG_FIFO_COUNTL = 0x2F  # FIFO count low byte
REG_FIFO_DATA = 0x30  # FIFO data read port
REG_FIFO_CONFIG1 = 0x5F  # FIFO config 1 (packet format)
REG_FIFO_CONFIG2 = 0x60  # FIFO config 2 (watermark)
REG_FIFO_CONFIG3 = 0x61  # FIFO config 3 (watermark)
REG_FIFO_LOST_PKT0 = 0x6C  # FIFO packet loss count LSB
REG_FIFO_LOST_PKT1 = 0x6D  # FIFO packet loss count MSB

# Motion detection and APEX features
REG_SMD_CONFIG = 0x57  # Significant motion detection config
REG_APEX_CONFIG0 = 0x56  # APEX config 0 (DMP features)
REG_APEX_DATA0 = 0x31  # APEX data 0
REG_APEX_DATA1 = 0x32  # APEX data 1
REG_APEX_DATA2 = 0x33  # APEX data 2
REG_APEX_DATA3 = 0x34  # APEX data 3
REG_APEX_DATA4 = 0x35  # APEX data 4 (tap info)
REG_APEX_DATA5 = 0x36  # APEX data 5

# Interface configuration
REG_INTF_CONFIG0 = 0x4C  # Interface config 0 (endianness, FIFO)
REG_INTF_CONFIG1 = 0x4D  # Interface config 1 (clock select)
REG_INTF_CONFIG4 = 0x7A  # Interface config 4
REG_INTF_CONFIG5 = 0x7B  # Interface config 5
REG_INTF_CONFIG6 = 0x7C  # Interface config 6

# Identification and bank selection
REG_WHO_AM_I = 0x75  # Chip ID register (reads 0x47)
REG_BANK_SEL = 0x76  # Register bank selection

# Sensor configuration (used for certain features)
REG_SENSOR_CONFIG0 = 0x03  # Sensor configuration 0

# ========================================================================
# BANK 1 REGISTERS (Gyroscope advanced configuration)
# ========================================================================

REG_GYRO_CONFIG_STATIC2 = 0x0B  # Gyro AAF and NF disable
REG_GYRO_CONFIG_STATIC3 = 0x0C  # Gyro AAF delta
REG_GYRO_CONFIG_STATIC4 = 0x0D  # Gyro AAF delta squared LSB
REG_GYRO_CONFIG_STATIC5 = 0x0E  # Gyro AAF delta squared MSB + bitshift
REG_GYRO_CONFIG_STATIC6 = 0x0F  # Gyro X notch filter coswz
REG_GYRO_CONFIG_STATIC7 = 0x10  # Gyro Y notch filter coswz
REG_GYRO_CONFIG_STATIC8 = 0x11  # Gyro Z notch filter coswz
REG_GYRO_CONFIG_STATIC9 = 0x12  # Gyro notch filter config
REG_GYRO_CONFIG_STATIC10 = 0x13  # Gyro notch filter bandwidth

# Gyro self-test data (Bank 1)
REG_XG_ST_DATA = 0x5F  # X-axis gyro self-test data
REG_YG_ST_DATA = 0x60  # Y-axis gyro self-test data
REG_ZG_ST_DATA = 0x61  # Z-axis gyro self-test data

# Timestamp values (Bank 1)
REG_TMSTVAL0 = 0x62  # Timestamp value byte 0
REG_TMSTVAL1 = 0x63  # Timestamp value byte 1
REG_TMSTVAL2 = 0x64  # Timestamp value byte 2

# ========================================================================
# BANK 2 REGISTERS (Accelerometer advanced configuration)
# ========================================================================

REG_ACCEL_CONFIG_STATIC2 = 0x03  # Accel AAF config
REG_ACCEL_CONFIG_STATIC3 = 0x04  # Accel AAF delta squared LSB
REG_ACCEL_CONFIG_STATIC4 = 0x05  # Accel AAF delta squared MSB + bitshift

# Accelerometer self-test data (Bank 2)
REG_XA_ST_DATA = 0x3B  # X-axis accel self-test data
REG_YA_ST_DATA = 0x3C  # Y-axis accel self-test data
REG_ZA_ST_DATA = 0x3D  # Z-axis accel self-test data

# ========================================================================
# BANK 4 REGISTERS (APEX and advanced features)
# ========================================================================

REG_APEX_CONFIG1 = 0x40  # APEX config 1
REG_APEX_CONFIG2 = 0x41  # APEX config 2
REG_APEX_CONFIG3 = 0x42  # APEX config 3
REG_APEX_CONFIG4 = 0x43  # APEX config 4 (tilt, sleep timing)
REG_APEX_CONFIG5 = 0x44  # APEX config 5
REG_APEX_CONFIG6 = 0x45  # APEX config 6
REG_APEX_CONFIG7 = 0x46  # APEX config 7 (tap detection)
REG_APEX_CONFIG8 = 0x47  # APEX config 8 (tap timing)
REG_APEX_CONFIG9 = 0x48  # APEX config 9

# Wake-on-motion threshold registers (Bank 4)
REG_ACCEL_WOM_X_THR = 0x4A  # X-axis WOM threshold
REG_ACCEL_WOM_Y_THR = 0x4B  # Y-axis WOM threshold
REG_ACCEL_WOM_Z_THR = 0x4C  # Z-axis WOM threshold

# Offset user registers (Bank 4)
REG_OFFSET_USER0 = 0x77  # User offset 0
REG_OFFSET_USER1 = 0x78  # User offset 1
REG_OFFSET_USER2 = 0x79  # User offset 2
REG_OFFSET_USER3 = 0x7A  # User offset 3
REG_OFFSET_USER4 = 0x7B  # User offset 4
REG_OFFSET_USER5 = 0x7C  # User offset 5
REG_OFFSET_USER6 = 0x7D  # User offset 6
REG_OFFSET_USER7 = 0x7E  # User offset 7
REG_OFFSET_USER8 = 0x7F  # User offset 8

# ========================================================================
# BIT MASKS AND FIELD DEFINITIONS
# ========================================================================

# PWR_MGMT0 bit fields (0x4E)
ACCEL_MODE_OFF = 0b00  # Accelerometer off
ACCEL_MODE_LP = 0b10  # Accelerometer low-power mode
ACCEL_MODE_LN = 0b11  # Accelerometer low-noise mode

GYRO_MODE_OFF = 0b00  # Gyroscope off
GYRO_MODE_STANDBY = 0b01  # Gyroscope standby
GYRO_MODE_LN = 0b11  # Gyroscope low-noise mode

TEMP_DIS_BIT = 0x20  # Temperature sensor disable bit
IDLE_BIT = 0x10  # IDLE mode bit

# FIFO_CONFIG bit fields (0x16)
FIFO_MODE_BYPASS = 0x00  # FIFO bypass mode
FIFO_MODE_STREAM = 0x40  # FIFO stream mode (1<<6)
FIFO_MODE_STOP_ON_FULL = 0x80  # FIFO stop when full (1<<7)

# FIFO_CONFIG1 bit fields (0x5F)
FIFO_ACCEL_EN = 0x01  # Enable accel in FIFO (bit 0)
FIFO_GYRO_EN = 0x02  # Enable gyro in FIFO (bit 1)
FIFO_TEMP_EN = 0x04  # Enable temp in FIFO (bit 2)
FIFO_TMST_FSYNC_EN = 0x08  # Enable timestamp in FIFO (bit 3)
FIFO_HIRES_EN = 0x10  # Enable high-resolution mode (bit 4)

# INT_CONFIG bit fields (0x14)
INT1_POLARITY_LOW = 0x00  # INT1 active low
INT1_POLARITY_HIGH = 0x01  # INT1 active high (bit 0)
INT1_DRIVE_OD = 0x00  # INT1 open-drain
INT1_DRIVE_PP = 0x02  # INT1 push-pull (bit 1)
INT1_MODE_PULSE = 0x00  # INT1 pulse mode
INT1_MODE_LATCH = 0x04  # INT1 latched mode (bit 2)

INT2_POLARITY_LOW = 0x00  # INT2 active low
INT2_POLARITY_HIGH = 0x08  # INT2 active high (bit 3)
INT2_DRIVE_OD = 0x00  # INT2 open-drain
INT2_DRIVE_PP = 0x10  # INT2 push-pull (bit 4)
INT2_MODE_PULSE = 0x00  # INT2 pulse mode
INT2_MODE_LATCH = 0x20  # INT2 latched mode (bit 5)

# Interrupt status bits (INT_STATUS2 - 0x37)
INT_STATUS_WOM_X = 0x01  # Wake-on-motion X (bit 0)
INT_STATUS_WOM_Y = 0x02  # Wake-on-motion Y (bit 1)
INT_STATUS_WOM_Z = 0x04  # Wake-on-motion Z (bit 2)
INT_STATUS_SMD = 0x08  # Significant motion detection (bit 3)

# Interrupt status bits (INT_STATUS3 - 0x38)
INT_STATUS_TAP = 0x01  # Tap detected (bit 0)
INT_STATUS_SLEEP = 0x02  # Sleep detected (bit 1)
INT_STATUS_WAKE = 0x04  # Wake detected (bit 2)
INT_STATUS_TILT = 0x08  # Tilt detected (bit 3)
INT_STATUS_STEP_CNT_OVF = 0x10  # Step counter overflow (bit 4)
INT_STATUS_STEP_DET = 0x20  # Step detected (bit 5)

# SMD_CONFIG bit fields (0x57)
SMD_MODE_OFF = 0x00  # SMD disabled
SMD_MODE_SHORT = 0x02  # SMD short (1 sec wait)
SMD_MODE_LONG = 0x03  # SMD long (3 sec wait)
WOM_MODE_CMP_INITIAL = 0x00  # Compare to initial sample
WOM_MODE_CMP_PREV = 0x04  # Compare to previous sample (bit 2)
WOM_INT_MODE_OR = 0x00  # WOM interrupt on OR of axes
WOM_INT_MODE_AND = 0x08  # WOM interrupt on AND of axes (bit 3)

# APEX_CONFIG0 bit fields (0x56) - Bank 0
DMP_POWER_SAVE_EN = 0x80  # DMP power save enable (bit 7)
TAP_ENABLE = 0x40  # Tap detection enable (bit 6)
PED_ENABLE = 0x20  # Pedometer enable (bit 5)
TILT_ENABLE = 0x10  # Tilt detection enable (bit 4)
R2W_ENABLE = 0x08  # Raise-to-wake enable (bit 3)

# APEX tap detection values (APEX_DATA4 - 0x35)
TAP_NUM_MASK = 0x18  # Tap number mask (bits 3-4)
TAP_SINGLE = 0x08  # Single tap (bit 3)
TAP_DOUBLE = 0x10  # Double tap (bit 4)
TAP_AXIS_MASK = 0x06  # Tap axis mask (bits 1-2)
TAP_AXIS_X = 0x00  # X-axis tap
TAP_AXIS_Y = 0x02  # Y-axis tap (bit 1)
TAP_AXIS_Z = 0x04  # Z-axis tap (bit 2)
TAP_DIR_MASK = 0x01  # Tap direction (bit 0)

# ========================================================================
# CONFIGURATION CONSTANTS
# ========================================================================

# Accelerometer full-scale range options (FSR)
# Maps to bits 5-7 of ACCEL_CONFIG0 (0x50)
ACCEL_RANGE_16G = 0  # ±16g (default)
ACCEL_RANGE_8G = 1  # ±8g
ACCEL_RANGE_4G = 2  # ±4g
ACCEL_RANGE_2G = 3  # ±2g

# Gyroscope full-scale range options (FSR)
# Maps to bits 5-7 of GYRO_CONFIG0 (0x4F)
GYRO_RANGE_2000_DPS = 0  # ±2000 dps (default)
GYRO_RANGE_1000_DPS = 1  # ±1000 dps
GYRO_RANGE_500_DPS = 2  # ±500 dps
GYRO_RANGE_250_DPS = 3  # ±250 dps
GYRO_RANGE_125_DPS = 4  # ±125 dps
GYRO_RANGE_62_5_DPS = 5  # ±62.5 dps
GYRO_RANGE_31_25_DPS = 6  # ±31.25 dps
GYRO_RANGE_15_625_DPS = 7  # ±15.625 dps

# Output Data Rate (ODR) options
# Maps to bits 0-3 of ACCEL_CONFIG0 and GYRO_CONFIG0
ODR_32KHZ = 1  # 32 kHz (LN mode only)
ODR_16KHZ = 2  # 16 kHz (LN mode only)
ODR_8KHZ = 3  # 8 kHz (LN mode only)
ODR_4KHZ = 4  # 4 kHz (LN mode only)
ODR_2KHZ = 5  # 2 kHz (LN mode only)
ODR_1KHZ = 6  # 1 kHz (LN mode, default)
ODR_200HZ = 7  # 200 Hz (LP or LN mode)
ODR_100HZ = 8  # 100 Hz (LP or LN mode)
ODR_50HZ = 9  # 50 Hz (LP or LN mode)
ODR_25HZ = 10  # 25 Hz (LP or LN mode)
ODR_12_5HZ = 11  # 12.5 Hz (LP or LN mode)
ODR_6_25HZ = 12  # 6.25 Hz (LP mode only - accel)
ODR_3_125HZ = 13  # 3.125 Hz (LP mode only - accel)
ODR_1_5625HZ = 14  # 1.5625 Hz (LP mode only - accel)
ODR_500HZ = 15  # 500 Hz (LP or LN mode - accel)

# Filter configuration constants
FILTER_ORDER_1ST = 0  # 1st order filter
FILTER_ORDER_2ND = 1  # 2nd order filter
FILTER_ORDER_3RD = 2  # 3rd order filter

# UI Filter bandwidth indices (0-15)
# See datasheet Table 6.4 and 6.7 for detailed bandwidth calculations
FILTER_BW_ODR_DIV_2 = 0  # BW = ODR/2
FILTER_BW_MAX_400_ODR_DIV_4 = 1  # BW = max(400Hz, ODR)/4 (default)
FILTER_BW_MAX_400_ODR_DIV_5 = 2  # BW = max(400Hz, ODR)/5
FILTER_BW_MAX_400_ODR_DIV_8 = 3  # BW = max(400Hz, ODR)/8
FILTER_BW_MAX_400_ODR_DIV_10 = 4  # BW = max(400Hz, ODR)/10
FILTER_BW_MAX_400_ODR_DIV_16 = 5  # BW = max(400Hz, ODR)/16
FILTER_BW_MAX_400_ODR_DIV_20 = 6  # BW = max(400Hz, ODR)/20
FILTER_BW_MAX_400_ODR_DIV_40 = 7  # BW = max(400Hz, ODR)/40
FILTER_BW_LOW_LATENCY = 14  # Low latency option
FILTER_BW_LOW_LATENCY_2 = 15  # Low latency option 2

# ========================================================================
# SCALING FACTORS FOR RAW DATA CONVERSION
# ========================================================================

# Temperature scaling
# Formula: Temperature (°C) = (RAW / 132.48) + 25.0
TEMP_SENSITIVITY = 132.48  # LSB per °C
TEMP_OFFSET = 25.0  # °C at 0 LSB

# Accelerometer sensitivity (LSB per g) for each range
# Used to convert raw 16-bit values to m/s²
ACCEL_SENSITIVITY = {
    ACCEL_RANGE_16G: 2048.0,  # ±16g range
    ACCEL_RANGE_8G: 4096.0,  # ±8g range
    ACCEL_RANGE_4G: 8192.0,  # ±4g range
    ACCEL_RANGE_2G: 16384.0,  # ±2g range
}

# Gyroscope sensitivity (LSB per dps) for each range
# Used to convert raw 16-bit values to rad/s
GYRO_SENSITIVITY = {
    GYRO_RANGE_2000_DPS: 16.4,  # ±2000 dps range
    GYRO_RANGE_1000_DPS: 32.8,  # ±1000 dps range
    GYRO_RANGE_500_DPS: 65.5,  # ±500 dps range
    GYRO_RANGE_250_DPS: 131.0,  # ±250 dps range
    GYRO_RANGE_125_DPS: 262.0,  # ±125 dps range
    GYRO_RANGE_62_5_DPS: 524.3,  # ±62.5 dps range
    GYRO_RANGE_31_25_DPS: 1048.6,  # ±31.25 dps range
    GYRO_RANGE_15_625_DPS: 2097.2,  # ±15.625 dps range
}

# Unit conversion constants
GRAVITY_EARTH = 9.80665  # m/s² (standard gravity)
DEG_TO_RAD = 0.017453292519943295  # π/180 for degrees to radians

# ========================================================================
# TIMING CONSTANTS (in seconds)
# ========================================================================

RESET_DELAY = 0.002  # 2ms delay after soft reset
MODE_CHANGE_DELAY = 0.0002  # 200µs delay after power mode change
GYRO_STARTUP_TIME = 0.045  # 45ms minimum gyro startup time
CONFIG_CHANGE_DELAY = 0.001  # 1ms delay after configuration changes

# ========================================================================
# FIFO CONSTANTS
# ========================================================================

FIFO_PACKET_SIZE_BASIC = 16  # 16 bytes per packet (header + 3-axis data)
FIFO_PACKET_SIZE_HIRES = 20  # 20 bytes per packet (high-resolution mode)
FIFO_MAX_SIZE = 2048  # Maximum FIFO size in bytes
