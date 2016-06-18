/*
 * Sketch that collects shadow band data. It directs two banks of sensors, each of which contains
 * up to 8 light sensors. 
 *
 * Copyright (C) 2016, The Winer Observatory, www.winer.org
 *
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SdFat.h>
#include <Wire.h>

#include "MasterHardwareConfiguration.h"


// i2c slave address of the DS3231 chip
#define DS3231_I2C_ADDR             0x68

// Objects for SD card access.
SdFat sd_card;
File file;

// Serial communications baud rate
#define BAUD 115200

/*
 * The Gray code 0 -- 7 is noted below, along with the pin changes
 * required to drive the code through its cycle. Note that pin SENSOR_SELECT_1
 * represents the 1 column, pin SENSOR_SELECT_2 represents the 2 value, and
 * pin SENSOR_SELECT_4 represents the 4 value
 * 
 * 4 2 1   Pin              Signal
 * - - -   ---------------  ------
 * 0 0 0   SENSOR_SELECT_4  LOW
 * 0 0 1   SENSOR_SELECT_1  HIGH
 * 0 1 1   SENSOR_SELECT_2  HIGH
 * 0 1 0   SENSOR_SELECT_1  LOW
 * 1 1 0   SENSOR_SELECT_4  HIGH
 * 1 1 1   SENSOR_SELECT_1  HIGH
 * 1 0 1   SENSOR_SELECT_2  LOW
 * 1 0 0   SENSOR_SELECT_1  LOW
 *
 * Since the Arduino Toolkit already defines HIGH and LOW, we will
 * define SET_HIGH and SET_LOW instead.
 */

#define SET_LOW 0x00
#define SET_HIGH 0x80

const uint8_t GREY_TRANSITIONS[] PROGMEM = {
  SENSOR_SELECT_4 | SET_LOW,    // 100 --> 000
  SENSOR_SELECT_1 | SET_HIGH,   // 000 --> 001
  SENSOR_SELECT_2 | SET_HIGH,   // 001 --> 011
  SENSOR_SELECT_1 | SET_LOW,    // 011 --> 010
  SENSOR_SELECT_4 | SET_HIGH,   // 010 --> 110
  SENSOR_SELECT_1 | SET_HIGH,   // 110 --> 111
  SENSOR_SELECT_2 | SET_LOW,    // 111 --> 101
  SENSOR_SELECT_1 | SET_LOW,    // 101 --> 100
};

/*
 * Values required to drive the slave and its light sensor
 */

// TSL230XX configuration bits
#define SENSITIVITY_LOW   0x1
#define SENSITIVITY_HIGH  0x2
#define SCALE_LOW         0x4
#define SCALE_HIGH        0x8

// Status definitions
#define RUNNING_COMMAND  0x00
#define DATA_AVAILABLE   0x20
#define READY_IDLE       0x40
#define MEASURING        0x80
#define INITIALIZING     0xFF

#define CMD_NOOP 0
#define CMD_RESERVED 1
#define CMD_READ_SENSOR 2
#define CMD_CONFIGURE_SENSOR 3
#define CMD_SET_INDICATOR 4
#define CMD_CLEAR_STATUS 5
#define CMD_STAGE_SOFTWARE_VERSION 6
#define CMD_STAGE_EEPROM 7
#define CMD_CLEAR_BUFFER 0x0E
#define CMD_DUMP_BUFFER 0xF

#define SPI_NOOP 0
#define SPI_LOAD_SERIAL_NUMBER 1
#define SPI_RUN_COMMAND 2
#define SPI_SEND_BYTE 3
#define SPI_RECEIVE_BYTES 4
#define SPI_PING_AND_INCREMENT 5
#define SPI_LOAD_STATUS 6

// Disk record types. Note that disk data is encoded as hexadecimal.
#define RECORD_HARDWARE_CONFIGURATION 0  // Hardware serial number
#define RECORD_SOFTWARE_SERIAL_NUMBER 1  // Software serial number
#define RECORD_CURRENT_TIME           2  // Current time (only)
#define RECORD_OBSERVATION            3  // One light measurement
#define RECORD_HALT_AND_CATCH_FIRE    4  // Close file and wait for reset.

#define OVERRUN_DELAY_MICROSECONDS 10
#define SD_CARD_SPI_SURRENDER_MICROSECONDS 20
#define SLAVE_RESET_DELAY_MICROSECONDS     4000
/*
 * The signature of slave command functions. These functions are invoked
 * within a loop that selects the active slave before invoking the command
 * function.
 * 
 * Parameter  Contents
 * ---------  --------
 *      bank  Slave bank number: 0 for slaves 0 .. 7, 1 for
 *            slaves 8 .. 15.
 *      line  Slave data line number, 0 .. 7
 *
 * Note: the slave number is 8 * bank + line.
 */

typedef void (*SlaveCommand)(const uint8_t bank, const uint8_t line);

/**
 * Time management software. The time is kept by a module that is bult around a
 * DS3231 temperature compensated time keeper. The mudule is set to ZULU (a.k.a. UTC)
 * the terrestrial time at the Prime Meridian, without daylight saving or summer time.
 */

const uint8_t DAYS_IN_MONTH[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/*
 * Allowable exposure times in microseconds. The sensitivity configurations reference
 * these values.
 */
const uint16_t EXPOSURE_TIME_MICROSECONDS[] PROGMEM = {125, 250, 500, 1000};

/*
 * Sensor and exposure time configurations, ordered from least sensentive to most
 * sensitive.
 */

const uint8_t SENSOR_CONFIGURATIONS[] PROGMEM = {
  SCALE_HUNDRED | SENSITIVITY_HUNDRED | EXPOSURE_125,
  SCALE_HUNDRED | SENSITIVITY_HUNDRED | EXPOSURE_250,
  SCALE_HUNDRED | SENSITIVITY_HUNDRED | EXPOSURE_500,
  SCALE_HUNDRED | SENSITIVITY_HUNDRED | EXPOSURE_1000,
  
  SCALE_TEN | SENSITIVITY_HUNDRED | EXPOSURE_125,
  SCALE_TEN | SENSITIVITY_HUNDRED | EXPOSURE_250,
  SCALE_TEN | SENSITIVITY_HUNDRED | EXPOSURE_500,

  SCALE_TWO | SENSITIVITY_HUNDRED | EXPOSURE_125,

  SCALE_TWO | SENSITIVITY_HUNDRED | EXPOSURE_1000,
  
  SCALE_TWO | SENSITIVITY_HUNDRED | EXPOSURE_250,
  SCALE_TWO | SENSITIVITY_HUNDRED | EXPOSURE_500,
  SCALE_TWO | SENSITIVITY_HUNDRED | EXPOSURE_1000,

  SCALE_UNITY | SENSITIVITY_TEN | EXPOSURE_125,
  SCALE_UNITY | SENSITIVITY_TEN | EXPOSURE_250,
  SCALE_UNITY | SENSITIVITY_TEN | EXPOSURE_500,
  SCALE_UNITY | SENSITIVITY_TEN | EXPOSURE_1000,
 
  SCALE_UNITY | SENSITIVITY_UNITY | EXPOSURE_125,
  SCALE_UNITY | SENSITIVITY_UNITY | EXPOSURE_250,
  SCALE_UNITY | SENSITIVITY_UNITY | EXPOSURE_500,
  SCALE_UNITY | SENSITIVITY_UNITY | EXPOSURE_1000,
};

static uint8_t light_sensitivity = 0;

void increase_light_sensitivity() {
  if (light_sensitivity < sizeof(SENSOR_CONFIGURATIONS) - 1) {
    ++light_sensitivity;
  }
}

void decrease_light_sensitivity() {
  if (0 < light_sensitivity) {
    --light_sensitivity;
  }
}

uint8_t raw_sensor_configuration() {
  return pgm_read_byte(SENSOR_CONFIGURATIONS + light_sensitivity);
}

uint8_t sensor_configuration() {
  return raw_sensor_configuration() & 0xF;
}

uint16_t exposure_time() {
  uint8_t exposure_offset = (raw_sensor_configuration() >> 4) & 0xF;
  return pgm_read_word(EXPOSURE_TIME_MICROSECONDS + exposure_offset); 
}

/**
   Converts the BCD representation of a value to its binary equivalent. BCD format
   is: bits 0 .. 3, most significant deciman digit, bits 4 .. 7, least significant
   decimal digit.So the decimal value 23 would be represented as 0x23 in BCD.

   Parameter       Contents
   --------------  --------
   bcd             Value in [0 .. 99] in BCD format

   Returns: the same value in binary format.
*/
uint8_t bcd2bin(uint8_t bcd) {
  return bcd - 6 * bcd >> 4;
}

/**
   Converts a raw time value into the number of seconds since January 1, 2000 in the
   same timezone. Note that this function returns value resuts only for dates occurring
   on or after January 1, 2000. All values must be non-negative, and are rounded DOWN.

   Parameters  Contents
   ----------  --------
   days        Number of days that have elapsed since January 1, 2000
   hours       Number of hours elapsed from mignight in the current day
   minutes     Number of minutes that have elapsed in the current hour
   seconds     Number of seconds that have elapsed in the current minute
*/
uint32_t time_to_seconds_since_y2k(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds) {
  return ((((uint32_t)days * 24 + hours) * 60) + minutes) * 60 + seconds;
}

/**
 * Calculates the number of days that have elapsed since 2000/01/01 on the prevailing
 * timezone.
 *
 * Parameter  Contents
 * ---------  --------
 * year       Year with reference to 2000, in the range [0 .. 99]
 * month      Month, in the range [1 .. 12] (conventional numbering)
 * day        Day of the month, in the range [1 .. 31], depending on
 *            the value of month
 */
int16_t date_to_days_since_y2k(uint8_t year, uint8_t month, uint8_t day) {
  uint16_t days = day - 1;
  for (uint8_t i = 1; i < (month & 0x7F); ++i)
  {
    days += pgm_read_byte(DAYS_IN_MONTH + i - 1);
  }

  if (month > 2 && year % 4 == 0) {
    ++days;
  }
  return days + 365 * year + (year + 3) / 4;
}

/**
   Read the time from the DS3231 and return the current time as seconds since
   Jaunary 1, 2000, UTC.
*/
uint32_t ds3231_current_time() {

  // Set the DS3231 register pointer to 0. We will read bytes
  // 0 - 6.
  Wire.beginTransmission(DS3231_I2C_ADDR);
  Wire.write(0);
  Wire.endTransmission();

  // Read the address 0 - 6 (inclusive) contents
  Wire.requestFrom(DS3231_I2C_ADDR, 7);  // [0 .. 6]
  uint8_t raw_seconds = Wire.read();
  uint8_t raw_minutes = Wire.read();
  uint8_t raw_hour = Wire.read();
  uint8_t raw_day_of_week = Wire.read();
  uint8_t raw_day_of_month = Wire.read();
  uint8_t raw_month_of_year = Wire.read();
  uint8_t raw_year_of_century = Wire.read();

  uint16_t days_since_2000 = date_to_days_since_y2k(
                               raw_year_of_century, raw_month_of_year, raw_day_of_month);
  uint32_t seconds_since_2000 = time_to_seconds_since_y2k(
                              days_since_2000, raw_hour, raw_minutes, raw_seconds);

  return seconds_since_2000;;
}

/*
 * Master/Slave communication support functions. The master exchages data with the
 * active slave via SPI. The slave lowers and raises the voltage on the "slave ready"
 * pin to indicate that it is ready.
 */

volatile uint8_t slave_busy = 0;   // 0 --> ready; 1 --> busy.
volatile uint8_t slave_count = 0;  // Incremented by 1 when the slave becomes ready.

/*
 * Slave Ready interrupt handler. When the "SLAVE_BUSY" pin transitioins from
 * LOW to HIGH, this method sets the slave_busy flag to 0. The byte output will
 * then return its results. This interrupt handler must be installed in setup().
 */
void interrupt_zero_low_to_high() {
  slave_busy = 0;
  ++slave_count;
}

/*
 * Main SPI output routine. This method exchanges one byte of information with
 * the active slave.
 * 
 * Parameter  Contents
 * ---------  --------
 * to_send    unsigned byte to be sent to the slave
 * 
 * Returns: the byte sent from the slave to this master.
 */
uint8_t spi_transfer(const uint8_t to_send) {
  noInterrupts();
  slave_busy = 1;
  interrupts();
  uint8_t received = SPI.transfer(to_send);
  while (slave_busy) {}  // Wait for slave to process the byte we sent.
  delayMicroseconds(OVERRUN_DELAY_MICROSECONDS);
  return received;
}

/*
 * Sends an SPI-level NO-OP command to the current slave. This is a cheap and
 * chearful way to read the contents of the Slave's SPDR register.
 *
 * Returns: the contents of the slave's SPDR (SPI Transfer) register
 */
uint8_t spi_noop() {
  return spi_transfer(SPI_NOOP);
}

/*
 * Exchanges a byte with a slave, then retrieves and returns the next byte that
 * the slave loads into its SPI register.
 *
 * Parameter  Contents
 * ---------  --------
 * to_send    unsigned byte to be sent to the slave
 * 
 * Returns: the byte that the slave loads into its SPI register *after* it processes
 * the sent byte..
 */
uint8_t spi_handshake(const uint8_t to_send) {
  spi_transfer(to_send);
  return spi_noop();
}

uint8_t spi_tagged_noop(const uint8_t tag) {
  return spi_handshake((tag << 4) & 0xF0);
}


/**
   Retrieves the serial number of the current slave

   Returns: the slave's serial number, in the range [0 .. 511]
*/
uint8_t spi_serial_number() {
  return spi_handshake(SPI_LOAD_SERIAL_NUMBER);
}

/**
   Sends the specified command to the current slave.

   Paramters:

   Name       Type            Contents
   ---------  ---------       ----------------------------------
   command    unsigned byte   Command, in range [0 .. 15]

   Returns: the slave's status after the commend has been set

*/
uint8_t spi_run_command(const uint8_t command) {
  return spi_handshake((command << 4) | SPI_RUN_COMMAND);
}

/**
   Orders the slave to sende the specified byte from the slave's I/O buffer

   Paramters:

   Name       Type            Contents
   ---------  ---------       ----------------------------------
   byte_index unsigned byte   Character index. Undefined if out of range
                              [0 .. 3]

*/
uint8_t spi_send_byte(const uint8_t byte_index) {
  return spi_handshake((byte_index << 4) | SPI_SEND_BYTE);
}

/*
 * Orders the slave to accept (i.e. receive) the specified number of bytes.
 *
 * Paramters:
 *
 * Name       Type            Contents
 * ---------  ---------       ----------------------------------
 * byte_count unsigned byte   Numberof bytes to receive, in range
 *                           [0 .. 15]. Note that the slave will treat
 *                           the next byte_count bytes as incoming data,
 *                           no matter what they contain.
 *
 * Returns: the contents of the SPI register.
 */
uint8_t spi_accept_bytes(const uint8_t byte_count) {
  return spi_transfer(((byte_count << 4) & 0xF0) | SPI_RECEIVE_BYTES);
}

/**
   Ping the current slave and post-increment the ping count. Note
   that the ping count wraps around at 511. The slave sets its ping
   count to 0 on startup/reset.

   Returns: the current (i.e. unincremented) ping count.
*/
uint8_t spi_ping_slave() {
  return spi_handshake(SPI_PING_AND_INCREMENT);
}

// Slave control code.

/*
 * Retrieves the status of the current slave
 *
 * Returns: the current slave's status
 */
uint8_t spi_get_slave_status() {
  return spi_handshake(SPI_LOAD_STATUS);
}

void reset_all_slaves() {
  digitalWrite(SLAVE_RESET, LOW);
  delay(2);
  digitalWrite(SLAVE_RESET, HIGH);
  delay(SLAVE_RESET_DELAY_MICROSECONDS);
}

uint8_t *retrieve_bytes(uint8_t count, uint8_t *output_buffer) {
  for (uint8_t i = 0; i < count; ++i) {
    output_buffer[i] = spi_send_byte(i);
  }
  return output_buffer;
}

uint8_t *read_eeprom(uint16_t address, uint8_t count, uint8_t *output_buffer) {
  spi_accept_bytes(2);
  uint8_t low = address & 0xFF;
  uint8_t high = (address >> 8) & 0xFF;
  spi_transfer(low);
  spi_transfer(high);
  spi_run_command(CMD_STAGE_EEPROM);
  return retrieve_bytes(count, output_buffer);
}

uint8_t * read_software_version(uint8_t * output_buffer) {
  spi_run_command(CMD_STAGE_SOFTWARE_VERSION);
  return retrieve_bytes(4, output_buffer);
}

/*
 * Panic loop that pulses the indicator light until the master is reset.
 *
 * Paramters:
 *
 * Name       Type            Contents
 * ---------  ---------       ----------------------------------
 * byte_count unsigned byte   Numberof bytes to receive, in range
 *                           [0 .. 15]. Note that the slave will treat
 *                           the next byte_count bytes as incoming data,
 *                           no matter what they contain.
 */
void panic_loop(uint8_t code) {
  for(;;){
    for (uint8_t i; i < code; ++i) {
      digitalWrite(INDICATOR_LIGHT, HIGH);
      delay(250);
      digitalWrite(INDICATOR_LIGHT, LOW);
      delay(250);
    }
    delay(1000);
  }
}

/*
 * Runs the command that changes the slave selection from the current
 * one, which must be stepNumber - 1 to stepNumber.
 * 
 * Preconditions:
 *   The slave selection must be configured to stepNumber - 1, or
 *   stepNumber 7 if stepNumber is 0. The stepNumber parameter
 *   must be set to [0 .. 7] inclusive.
 * 
 * Postconditions:
 *   The slave selection will be configured to stepNumber.
 *   
 *
 * Name        Type            Contents
 * ---------   ---------       ----------------------------------
 * step_number byte            The desired step number.
 */
void set_slave_selection(uint8_t stepNumber) {
  uint8_t operation = pgm_read_byte(GREY_TRANSITIONS + stepNumber);
  digitalWrite(operation & 0x7F, (operation & 0x80) ? HIGH : LOW);
}

/*
 * Slave bank management methods. The hardware provides two banks, each of which can\
 * manage 8 slaves. The following methods enable and disable the slave banks. Note that
 * all disable operations are always performed before any enable operations.
 */
void disable_all_slaves() {
  digitalWrite(BANK_1_SELECT, HIGH);
  digitalWrite(BANK_0_SELECT, HIGH);
  digitalWrite(SENSOR_SELECT_4, LOW);
  digitalWrite(SENSOR_SELECT_2, LOW);
  digitalWrite(SENSOR_SELECT_1, LOW);
}

void enable_bank_zero() {
  digitalWrite(BANK_1_SELECT, HIGH);
  digitalWrite(BANK_0_SELECT, LOW);
}

void enable_bank_one() {
  digitalWrite(BANK_0_SELECT, HIGH);
  digitalWrite(BANK_1_SELECT, LOW);
}

static const uint8_t HEX_DIGITS[] PROGMEM = 
    {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void wait_for_sdcard_bus_surrender() {
  digitalWrite(SDCARD_SELECT, HIGH);
  delayMicroseconds(SD_CARD_SPI_SURRENDER_MICROSECONDS);
}

/*
 * Writes a new-line character to the file.
 */
void write_newline() {
  file.write('\n');
}

/*
 * Ends a record by writing a new-line character and flushing the buffer
 * to the SD card.
 */
void write_record_end() {
  write_newline();
  file.flush();
  wait_for_sdcard_bus_surrender();
}

/*
 * Encodes one byte of data as in hexadecimal and writes the resulting
 * two digits to the disk file
 *
 * Name        Type            Contents
 * ---------   ---------       ----------------------------------
 * value       byte            The value to write.
 */
void write_byte(uint8_t value) {
  file.write(pgm_read_byte(HEX_DIGITS + ((value >> 4) & 0xF)));
  file.write(pgm_read_byte(HEX_DIGITS + (value & 0xF)));
}

void write_uint16(uint16_t value) {
  write_byte(highByte(value));
  write_byte(lowByte(value)); 
}

/*
 * Writes the unsigned 32-bit value to the SD card
 * 
 * Preconditions: The write count must have been initialized and at least one byte
 *                must be free in the output queue. Neither is checked.
 *
 * Postconditions: the first remaining queue entry is set to the specified value and
 *                 is removed from the free space pool.
 * Paramters:
 *
 * Name         Type            Contents
 * ---------    ---------       ----------------------------------
 * value        uint32_t        The byte value to enqueue for output to the SD card
 */
void write_uint32(uint32_t value) {
  uint8_t bytes[4];
  uint8_t *pbytes = bytes + 4;
  while(pbytes > bytes) {
    *(--pbytes) = value & 0xFF;
    value >>= 8;
  }

  while (pbytes < bytes + 4) {
    write_byte(*pbytes++);
  }
}

/*
 * Extract one byte from EEPROM and write it to disk.
 * 
 * Name        Type            Contents
 * ---------   ---------       ----------------------------------
 * address     uint16_t        The EEPROM address containing the byte to be 
 *                             written.
 */
void write_eeprom(uint16_t address) {
  write_byte(EEPROM.read(address));
}

/*
 * Writes one data byte from PROGMEM to disk. Note that all bytes are encoded in
 * hexadecimal.
 *
 * Name        Type            Contents
 * ---------   ---------       ----------------------------------
 * address     address_short   The flash address containing the byte to be
 *                             written.
 */
void write_pgm_mem(const uint8_t PROGMEM * address) {
  write_byte(pgm_read_byte(address));  
}

/*
 * Writes the hardware configuration record. The record contains the
 * following information. Note that all information is encoded in
 * hexadecimal, per standards.
 * 
 * Byte No.  Contents
 * ------- ----------------------------------------------------
 *   0 - 1 Record type indication, must be 0.
 *       2 Most significant byte of the master serial number
 *       3 Least significant byte of the master serial number
 *       4 Hardware major revision number
 *       5 Hardware minor revision number
 *       6 Hardware revision number
 *       7 Reserved, must be "00:
 *       8 Number of slaves on bank 0
 *       9 Number of slaves on bank 1
 *
 *         The following record is repeated for each slave.
 *      10 Slave serial number major revision
 *      11 Slave serial number minor revision
 *      12 Slave revision number
 *      13 Reserved, must be "00"
 */
void write_hardware_serial_number() {
  write_byte(RECORD_HARDWARE_CONFIGURATION);
  write_eeprom(EEPROM_HW_MAJOR_VERSION);
  write_eeprom(EEPROM_HW_MINOR_VERSION);
  write_eeprom(EEPROM_HW_CHANGE_NUMBER);
  write_eeprom(EEPROM_HW_RESERVED);
  write_eeprom(EEPROM_HW_SERIAL_MSB);
  write_eeprom(EEPROM_HW_SERIAL_LSB);
  write_eeprom(EEPROM_BANK_ZERO_SLAVE_COUNT);
  write_eeprom(EEPROM_BANK_ONE_SLAVE_COUNT);
}

static const uint8_t SOFTWARE_SERIAL_NUMBER[] PROGMEM = {0, 9, 1, 0};

void write_software_serial_number() {
  write_byte(RECORD_SOFTWARE_SERIAL_NUMBER);
  for (uint8_t index = 0; index < sizeof(SOFTWARE_SERIAL_NUMBER); ++index) {
    write_pgm_mem(SOFTWARE_SERIAL_NUMBER + index);
  }
}

/*
 * Writes the current time, in seconds since January 1, 2000, to the SD card.
 */

void write_current_time() {
  write_uint32(ds3231_current_time());
  write_uint32(millis());
}

void write_record_type_and_time(uint8_t record_type) {
  write_byte(record_type);
  write_current_time();
}

void write_time_record() {
  write_record_type_and_time(RECORD_CURRENT_TIME);
  write_record_end();
}

void write_master_software_serial_number() {
  write_byte(RECORD_SOFTWARE_SERIAL_NUMBER);
  write_software_serial_number();
}

// Slave command output buffer and assocatiated functions.
static uint8_t write_queue[32];
static uint8_t write_count;

void panic_on_buffer_overflow() {
  if (sizeof(write_queue) <= write_count) {
    panic_loop(PANIC_OUTPUT_QUEUE_OVERRUN);
  }  
}

/*
 * Enqueues the byte value for output to the SD card
 * 
 * Preconditions: The write count must have been initialized and at least one byte
 *                must be free in the output queue. Neither is checked.
 *
 * Postconditions: the first remaining queue entry is set to the specified value and
 *                 is removed from the free space pool.
 * Paramters:
 *
 * Name         Type            Contents
 * ---------    ---------       ----------------------------------
 * value        uint8_t         The byte value to enqueue for output to the SD card
 */
void enqueue_for_write(uint8_t value) {
  panic_on_buffer_overflow();
  write_queue[write_count++] = value;
}

/**
 * Applies the specified command to the currently selected slave bank and
 * writes any enqueued output to the output file.
 * 
 * Preconditions: the selected slave bank must be active.
 * 
 * Postconditions: the command is executed for each slave in the active
 *                 bank. The precise postconditions depend on the command
 *                 performed. All slaves will be disabled. If the invoked
 *                 command enqueues any output, that output will be written
 *                 to the SD card. The output buffer will NOT be flushed.
 *
 * Note: slave command functions MUST NOT perform file writes. Instead, they must
 *       enqueue output to the file output queue. Each command invocation MAY
 *       enqueue 0 .. 4 bytes, and MUST NOT enqueue any more than 4. 
 *
 * Paramters:
 *
 * Name         Type            Contents
 * ---------    ---------       ----------------------------------
 * bank_no      uint8_t         Current bank selected, 0 or 1.
 * slave_count  uint8_t         Number of slaves available in the bank, 0 .. 7
 * command      function ptr    Pointer to the slave command function
 * 
 * Returns: the number of bytes written to the file, 0 ==> none written.
 */
uint8_t apply_to_selected_slave_bank(
    const uint8_t bank_no, 
    const uint8_t slave_count, SlaveCommand command) {
  write_count = 0;
  SPI.beginTransaction(SPISettings());
  for (uint8_t line_no = 0; line_no < slave_count; ++line_no) {
    set_slave_selection(line_no);
    command(bank_no, line_no);
  }
  SPI.endTransaction();
  disable_all_slaves();
  uint8_t *pdata = write_queue;
  for (uint8_t i = 0; i < write_count; ++i) {
    write_byte(*(pdata++));
  }
  return write_count;
}

/*
 * Invokes the provided slave command function against all
 * available slaves.
 *
 * Note: slave command functions MUST NOT perform file writes. Instead, they must
 *       enqueue output to the file output queue. Each command invocation MAY
 *       enqueue 0 .. 4 bytes, and MUST NOT push any more than 4. 
 * 
 * Paramters:
 *
 * Name         Type            Contents
 * ---------    ---------       ----------------------------------
 * command      function ptr    Pointer to the slave command function
 * 
 * Returns: the total number of bytes written to the SD card, with 0 ==> none.
 */
uint8_t apply_to_slaves(SlaveCommand command) {
  enable_bank_zero();
  uint8_t bytes_written = apply_to_selected_slave_bank(
        0, 
        EEPROM.read(EEPROM_BANK_ZERO_SLAVE_COUNT),
        command);
  enable_bank_one();
  bytes_written += apply_to_selected_slave_bank(
        1, 
        EEPROM.read(EEPROM_BANK_ONE_SLAVE_COUNT),
        command);
  return bytes_written;;
}

/*
 * Invokes the provided slave command function against all available
 * slaves. If the command writes anything to the SD card, a newline
 * is added and the buffer is flushed.
 * 
 * Paramters:
 *
 * Name         Type            Contents
 * ---------    ---------       ----------------------------------
 * command      function ptr    Pointer to the slave command function
 */
void apply_to_slaves_and_write_record_end(SlaveCommand command) {
  if(apply_to_slaves(command)) {
    write_record_end();
  }
}

void short_delay(uint8_t bank, uint8_t line) {
  delay(200);
}

void flash_twice(const uint8_t bank, const uint8_t line) {
  delay(500);
  for (uint8_t i = 0; i < 2; ++i) {
    digitalWrite(INDICATOR_LIGHT, HIGH);
    delay(10);
    digitalWrite(INDICATOR_LIGHT, LOW);
    delay(90);
  }
  delay(2000);
}

/*
 * Reads the active slave's serial number and writes it to the SD Card
 *
 * Preconditions: slave selected.
 * 
 * Postconditions: selected slave's serial number written to the SD card.
 * 
 * Paramters:
 *
 * Name         Type            Contents
 * ---------    ---------       ----------------------------------
 * bank         uint8_t         Slave bank number, 0 or 1
 * line         uint8_t         Slave line number, 0 .. 7
 */
void write_slave_serial_number(const uint8_t bank, const uint8_t line) {
  uint8_t value = spi_serial_number();
  enqueue_for_write(value);
  flash_twice(0, 0);
}

void enqueue_slave_software_serial_number(const uint8_t bakn, const uint8_t line) {
  panic_on_buffer_overflow();
  spi_run_command(CMD_STAGE_SOFTWARE_VERSION);
  retrieve_bytes(4, write_queue + write_count);
  write_count += 4;
}

void perform_tagged_noop(const uint8_t bank, const uint8_t line) {
  spi_tagged_noop(0xB);
}

void perform_tagged2_noop(const uint8_t bank, const uint8_t line) {
  spi_tagged_noop(0xC);
}

void perform_tagged3_noop(const uint8_t bank, const uint8_t line) {
  spi_tagged_noop(0xD);
}

void perform_tagged_one_noop(const uint8_t bank, const uint8_t line) {
  spi_tagged_noop(1);  
}


void write_software_serial_numbers() {
  write_master_software_serial_number();
  apply_to_slaves(enqueue_slave_software_serial_number);
  write_record_end();
}

// Measurement functions.

static uint32_t minimum_measurement;
static uint32_t maximum_measurement;

void configure_current_slave_light_sensor(uint8_t bank, uint8_t line) {
  spi_accept_bytes(1);
  spi_transfer(sensor_configuration());
  spi_run_command(CMD_CONFIGURE_SENSOR);
}

void take_light_exposure() {
  digitalWrite(MEASUREMENT_CONTROL, LOW);
  delayMicroseconds(exposure_time());
  digitalWrite(MEASUREMENT_CONTROL, HIGH);
}

void persist_sensor_readings(uint8_t bank, uint8_t line) {
  uint8_t backwards_sensor_reading[4];
  uint32_t measurement = 0;
  spi_run_command(CMD_READ_SENSOR);
  uint8_t *pbackwards_value = retrieve_bytes(
    sizeof(backwards_sensor_reading), backwards_sensor_reading)
    + sizeof(backwards_sensor_reading);
  while(backwards_sensor_reading < pbackwards_value) {
    uint8_t byte_value = *(--pbackwards_value);
    measurement = (measurement << 8) | byte_value;
    enqueue_for_write(byte_value);
  }
  minimum_measurement = min(minimum_measurement, measurement);
  maximum_measurement = max(maximum_measurement, measurement);
}

void calibrate_sensitivity() {
  if (MEASUREMENT_UPPER_LIMIT < maximum_measurement) {
    decrease_light_sensitivity();
  } else if (minimum_measurement < MEASUREMENT_LOWER_LIMIT) {
    increase_light_sensitivity();
  }
}

void halt_and_catch_fire() {
  disable_all_slaves();   // Just in case.
  write_record_type_and_time(RECORD_HALT_AND_CATCH_FIRE); 
  write_record_end();
  panic_loop(PANIC_HALT_AND_CATCH_FIRE);
  
}

void flash_indicator_light_quickly() {
  for (uint8_t i = 0; i < 5; ++i) {
    digitalWrite(INDICATOR_LIGHT, HIGH);
    delay(50);
    digitalWrite(INDICATOR_LIGHT, LOW);
    delay(150);
    digitalWrite(INDICATOR_LIGHT, HIGH);
    delay(150);
    digitalWrite(INDICATOR_LIGHT, LOW);
    delay(50);
  }
}

void flash_indicator_light() {
  digitalWrite(INDICATOR_LIGHT, HIGH);
  delay(500);
  digitalWrite(INDICATOR_LIGHT, LOW);
}


void setup() {
  // Set the initial state of all pins:
  pinMode(HALT_AND_CATCH_FIRE, INPUT_PULLUP);
  pinMode(ACTIVE_SLAVE_READY, INPUT_PULLUP);
  pinMode(INDICATOR_LIGHT, OUTPUT);
  pinMode(SLAVE_RESET, OUTPUT);
  pinMode(BANK_0_SELECT, OUTPUT);
  pinMode(BANK_1_SELECT, OUTPUT);
  pinMode(SENSOR_SELECT_1, OUTPUT);
  pinMode(SENSOR_SELECT_2, OUTPUT);
  pinMode(SENSOR_SELECT_4, OUTPUT);
  pinMode(SDCARD_SELECT, OUTPUT);
  pinMode(MASTER_OUT_SLAVE_IN, OUTPUT);
  pinMode(MASTER_IN_SLAVE_OUT, INPUT);
  pinMode(SPI_CLOCK, OUTPUT);
  pinMode(MEASUREMENT_CONTROL, OUTPUT);

  // Disable the SD card.
  digitalWrite(SDCARD_SELECT, HIGH);

  // Known state for SPI Master out pin
  digitalWrite(MASTER_OUT_SLAVE_IN, HIGH);

  // Disable light measurement.
  digitalWrite(MEASUREMENT_CONTROL, HIGH);

  reset_all_slaves();

  disable_all_slaves();
  flash_indicator_light();

  // This responds to "I am done" pulses.
  attachInterrupt(
    digitalPinToInterrupt(ACTIVE_SLAVE_READY),
    interrupt_zero_low_to_high,
    RISING);

  // Start communications.

  Wire.begin();
  SPDR = 0;
  SPI.begin();

  flash_twice(0, 0);

  for (int i = 0; i < 5; ++i) {
    apply_to_slaves(perform_tagged_one_noop);
  }

  if (!sd_card.begin(SDCARD_SELECT, 2)) {
    panic_loop(PANIC_SD_CARD_SELECT_FAILUED);
  }
  if (!sd_card.card()) {
    panic_loop(PANIC_SD_CARD_DRIVER_AWOL);
  }

  if (!sd_card.chdir("/")) {
    panic_loop(PANIC_SD_CARD_ROOT_AWOL);
  }

  file = sd_card.open("solar.dat", FILE_WRITE);
  if (!file) {
    panic_loop(PANIC_SD_CARD_DATA_FILE_AWOL);
  }
  write_hardware_serial_number();
  file.flush();
  wait_for_sdcard_bus_surrender();
  
  for (uint8_t i = 0; i < 5; ++i) {
    apply_to_slaves(perform_tagged_noop);
    apply_to_slaves(perform_tagged2_noop);
    apply_to_slaves(perform_tagged3_noop);
  }
  
  apply_to_slaves(write_slave_serial_number);
  write_record_end();
  write_software_serial_numbers();
  write_time_record();

  for (uint8_t i = 0; i < 5; ++i) {
    apply_to_slaves(perform_tagged_noop);
    apply_to_slaves(perform_tagged2_noop);
    apply_to_slaves(perform_tagged3_noop);
  }
  
  flash_indicator_light();
}

void loop() {
  if (digitalRead(HALT_AND_CATCH_FIRE) == LOW) {
    halt_and_catch_fire(); // Does not return.
  }
  maximum_measurement = 0;
  minimum_measurement = 0xFFFFFFFF;
  apply_to_slaves(configure_current_slave_light_sensor);
  take_light_exposure();
  write_record_type_and_time(RECORD_OBSERVATION);
  write_byte(light_sensitivity);
  write_byte(sensor_configuration());
  write_uint16(exposure_time());
  wait_for_sdcard_bus_surrender();
  apply_to_slaves_and_write_record_end (persist_sensor_readings);
  calibrate_sensitivity();
}
