/**
 * Master Hardware Configuration. The master configuration is
 * held in EEProm as follows
 * 
 * Byte  Contents
 * ----  -----------------------------------------------------------
 *    0  Hardware major revision numnber, 0 ==> prerelease
 *    1  Hardware minor revision number
 *    2  Hardware change number.
 *    3  Reserved, must be 0
 *    4  Serial Number, LSB
 *    5  Serial Number, MSB
 *    6  Slave count on bank 0 in range [0 .. 8]
 *    7  Slave count in bank 1 in range [0 .. 8]
 */
#ifndef __MASTER_HARDWARE_CONFIGURATION
#define __MASTER_HARDWARE_CONFIGURATION

// Pin Assignments
#define HALT_AND_CATCH_FIRE A1  // Lower to close the file and wait for reset.
#define MEASUREMENT_CONTROL A0  // Lower to start count
#define SPI_CLOCK           13  // SPI clock signal
#define MASTER_IN_SLAVE_OUT 12  // SPI Master In Slave Out
#define MASTER_OUT_SLAVE_IN 11  // SPI Master Out Slave In
#define SDCARD_SELECT       10  // Lower to select the SD Card
#define SENSOR_SELECT_4      9  // 4 bit of sensor select value
#define SENSOR_SELECT_2      8  // 2 bit of sensor select value
#define SENSOR_SELECT_1      7  // 1 bit of sensor select value
#define BANK_1_SELECT        6  // Lower to select sensor bank one
#define BANK_0_SELECT        5  // Lower to select sensor bank zero
#define SLAVE_RESET          4  // Lower to reset all slaves
#define INDICATOR_LIGHT      3  // Pin 3 is available
#define ACTIVE_SLAVE_READY   2  // Active slave lowers when ready

// EEPROM Address Allocations
#define MAJOR_VERSION_ADDRESS 0
#define MINOR_VERSION_ADDRESS 1
#define HARDWARE_CHANGE_ADDRESS 2
#define RESERVED1_ADDRESS 3
#define SERIAL_LSB_ADDRESS 4
#define SERIAL_MSB_ADDRESS 5
#define BANK_0_COUNT_ADDRESS 6
#define BANK_1_COUNT_ADDRESS 7


// EEProm configuration
#define EEPROM_HW_MAJOR_VERSION       0
#define EEPROM_HW_MINOR_VERSION       1
#define EEPROM_HW_CHANGE_NUMBER       2
#define EEPROM_HW_RESERVED            3
#define EEPROM_HW_SERIAL_LSB          4
#define EEPROM_HW_SERIAL_MSB          5
#define EEPROM_BANK_ZERO_SLAVE_COUNT  6
#define EEPROM_BANK_ONE_SLAVE_COUNT   7

// Panic codes
#define PANIC_SD_CARD_SELECT_FAILUED   1
#define PANIC_SD_CARD_DRIVER_AWOL      2
#define PANIC_SD_CARD_ROOT_AWOL        3
#define PANIC_SD_CARD_DATA_FILE_AWOL   4
#define PANIC_OUTPUT_QUEUE_OVERRUN     5
#define PANIC_HALT_AND_CATCH_FIRE      6

// TSL230XX sensitivity attentuations
#define SENSITIVITY_POWER_DOWN 0x0
#define SENSITIVITY_UNITY      0x1
#define SENSITIVITY_TEN        0x2
#define SENSITIVITY_HUNDRED    0x3

// TSL230XX frequency precaling
#define SCALE_UNITY            0x0
#define SCALE_TWO              0x4
#define SCALE_TEN              0x8
#define SCALE_HUNDRED          0xC

// Exposure times in microseconds
#define EXPOSURE_125   0x00
#define EXPOSURE_250   0x10
#define EXPOSURE_500   0x20
#define EXPOSURE_1000  0x30

// Light measurement range. The system will try to keep the measurement between these two values;

#define MEASUREMENT_UPPER_LIMIT 0xFFFFFFFF
#define MEASUREMENT_LOWER_LIMIT 0xFFF

#endif  // __MASTER_HARDWARE_CONFIGURATION


