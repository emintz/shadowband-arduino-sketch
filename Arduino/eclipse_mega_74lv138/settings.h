// Settings for the Eclipse Shadow Band Detector
// TODO(emintz); Reconcile with physical wiring!

#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#define BAUD 115200
#define GPS_DEFAULT_BAUD 9600

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

#define MEASUREMENT_UPPER_LIMIT 0xFFFFFFF
#define MEASUREMENT_LOWER_LIMIT 0xFFFF

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
#define OUT_GPS_ENABLE 5
#define IN_SD_CARD_INSERTED_NOT 6  // use INPUT_PULLUP
#define OUT_SECOND_TURNOVER 7
#define OUT_INDICATOR_LED 8
#define IN_PANIC_BUTTON_NOT 9
#define OUT_EXPOSURE_START_SIGNAL 10     // Connected and not used yet.
#define OUT_EXPOSURE_END_SIGNAL 11  // Connected and not used yet.
#define OUT_READ_START_SIGNAL 12  // Connected and not used yet. 
#define OUT_HAVE_GPS_SIGNAL 13

#define OUT_COUNTER_CLEAR_NOT 43
#define OUT_RCLK 44
#define OUT_SENSOR_OUTPUT_ENABLE_NOT 45

#define IN_DEBUG_1 A0
#define IN_DEBUG_2 A1

// SPI Pin Definitions
#define OUT_SPI_SLAVE_SELECT 53
#define OUT_SDCARD_SELECT_NOT OUT_SPI_SLAVE_SELECT
#define OUT_SPI_CLOCK 52
#define OUT_SPI_MASTER_OUT_SLAVE_IN 51
#define IN_SPI_MASTER_IN_SLAVE_OUT 50

#define SD_CARD_SPI_SURRENDER_MICROSECONDS 10

// Status indications
#define STATUS_OK        3
#define STATUS_LAMP_TEST 4
#define STATUS_POWER_ON  5

// GPS Time Acquisition States

#define TIME_STATE_READY 0
#define TIME_STATE_SET 1
#define TIME_STATE_STAGED 2
#define TIME_STATE_SENDING 3
 
// Disk record types. Note that disk data is encoded as hexadecimal.
// TODO(emintz): OBSOLETE: REVISE
#define RECORD_HARDWARE_CONFIGURATION 0  // Hardware serial number (RESERVED)
#define RECORD_OBSERVATION            1  // One light measurement

/** Counter selection settings for port A
 *  
 *  The low order nyble enable and selects the byte that a pair of 
 *  counters emits on the data bus, and enables or disables data output
 *  for the entire system.
 *  
 *  The high order nybble selects the pair of counters that is currently
 *  under control and controls a reserved status line.
 */
#define OUT_COUNTER_SELECTION_ONE         0b00000001
#define OUT_COUNTER_SELECTION_TWO         0b00000010
#define OUT_COUNTER_SELECTION_FOUR        0b00000100

#define OUT_GLOBAL_BYTE_OUTPUT_ENABLE_NOT 0b00001000

#define OUT_BYTE_SELECTION_ONE            0b00010000
#define OUT_BYTE_SELECTION_TWO            0b00100000
#define OUT_BYTE_SELECTION_FOUR           0b01000000

#define OUT_CONTROL_STATUS                0b10000000

// Byte display control masks

#define CLEAR_BYTE_SELECTION_MASK    (~(OUT_BYTE_SELECTION_ONE | OUT_BYTE_SELECTION_TWO | OUT_BYTE_SELECTION_FOUR))
#define CLEAR_COUNTER_SELECTION_MASK (~(OUT_COUNTER_SELECTION_ONE | OUT_COUNTER_SELECTION_TWO | OUT_COUNTER_SELECTION_FOUR))

/** Sensor and counter control settings for port L
 *
 * Port L (Counter and Sensor Management) Bit Breakout. Note that bit 0
 * is the least significant
 *
 * Bit  Signal
 * ---  ----------------------------------------------------
 *   0  Sensor S0 pin
 *   1  Sensor S1 pin
 *   2  Sensor S2 pin
 *   3  Sensor S3 pin
 *   4  Sensor /Output Enable pin (LOW ==> Enabled)
 *   5  Counter RCLK (low to high latches count)
 *   6  Counter /CLEAR (low ==> clears the count latches)
 *   7  unused, not connected
 */

#define SENSOR_POWER_DOWN            0b00000000
#define SENSOR_TIMES_ONE             0b00000001
#define SENSOR_TIMES_TEN             0b00000010
#define SENSOR_TIMES_ONE_HUNDRED     0b00000011
#define SENSOR_DIVIDE_BY_TWO         0b00000101
#define SENSOR_DIVIDE_BY_TEN         0b00001001
#define SENSOR_DIVIDE_BY_ONE_HUNDRED 0b00001101
#define SENSOR_OUTPUT_ENABLE         0b00010000
#define COUNTER_LATCH_MEASUREMENT    0b00100000
#define COUNTER_CLEAR_NOT            0b01000000

// NO-OP instruction, which can be used to create short delays.
#define NO_OP __asm__ __volatile__ ("nop\n\t");

#endif // #ifdef __SETTINGS_H__
// End of file

