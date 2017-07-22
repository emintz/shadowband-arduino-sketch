// Settings for the Eclipse Shadow Band Detector
#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#define BAUD 115200
#define GPS_DEFAULT_BAUD 9600

// EEProm configuration
#define EEPROM_HW_MAJOR_VERSION       0
#define EEPROM_HW_MINOR_VERSION       1
#define EEPROM_HW_CHANGE_NUMBER       2
#define EEPROM_HW_RESERVED            3
#define EEPROM_HW_SERIAL_LSB          4
#define EEPROM_HW_SERIAL_MSB          5
#define EEPROM_NUMBER_OF_SENSORS      6

// Status (i.e. indicator light flash) codes:
#define STATUS_OK       3
#define STATUS_POWER_ON 5

// Panic codes
#define PANIC_SD_CARD_SELECT_FAILED    1
#define PANIC_SD_CARD_DRIVER_AWOL      2
#define PANIC_SD_CARD_ROOT_AWOL        3
#define PANIC_SD_CARD_DATA_FILE_AWOL   4
#define PANIC_OUTPUT_QUEUE_OVERRUN     5
#define PANIC_HALT_AND_CATCH_FIRE      6

// Individual pin definitions. Note that the sketch uses
// low-level port manipulation for most I/O.
#define IN_GPS_SECOND_START_SIGNAL 2
#define OUT_SECOND_TURNOVER 5
#define OUT_RCLK 6
#define OUT_SENSOR_OUTPUT_ENABLE_NOT 7
#define OUT_COUNTER_CLEAR_NOT 8
#define IN_SD_CARD_INSERTED_NOT 9
#define IN_PANIC_BUTTON_NOT 10
#define OUT_INDICATOR_LED 11
#define OUT_GPS_ENABLE 12
#define OUT_HAVE_GPS_SIGNAL 13

// SPI Pin Definitions
#define OUT_SPI_SLAVE_SELECT 53
#define OUT_SDCARD_SELECT_NOT OUT_SPI_SLAVE_SELECT
#define OUT_SPI_CLOCK 52
#define OUT_SPI_MASTER_OUT_SLAVE_IN 51
#define IN_SPI_MASTER_IN_SLAVE_OUT 50


// Disk record types. Note that disk data is encoded as hexadecimal.
// TODO(emintz): OBSOLETE: REVISE
#define RECORD_HARDWARE_CONFIGURATION 0  // Hardware serial number (RESERVED)
#define RECORD_OBSERVATION            1  // One light measurement

#endif // #ifdef __SETTINGS_H__
// End of file
