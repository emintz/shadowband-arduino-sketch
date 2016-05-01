/* SPI Test Program. Sends commands to
*/

// Pin assignments
#define SPI_CLOCK 13
#define MASTER_IN_SLAVE_OUT 12
#define MASTER_OUT_SLAVE_IN 11
#define SLAVE_SELECT 10
#define MEASUREMENT_CONTROL 9

#define RED_LIGHT 6
#define GREEN_LIGHT 5
#define SLAVE_RESET 4

#define SLAVE_READY 2


// TSL230XX configuration bits
#define SENSITIVITY_LOW   0x1
#define SENSITIVITY_HIGH  0x2
#define SCALE_LOW         0x4
#define SCALE_HIGH        0x8

// TSL230XX Sensitivities
#define SENSITIVITY_POWER_DOWN 0x0
#define SENSITIVITY_UNITY      0x1
#define SENSITIVITY_TEN        0x2
#define SENSITIVITY_HUNDRED    0x3

// TSL230XX Scaling
#define SCALE_UNITY            0x0
#define SCALE_TWO              0x4
#define SCALE_TEN              0x8
#define SCALE_HUNDRED          0xC

// Status definitions
#define RUNNING_COMMAND  0x00
#define DATA_AVAILABLE   0x20
#define READY_IDLE       0x40
#define MEASURING        0x80
#define INITIALIZING     0xFF

#define SPI_TURNAROUND_MICROS 15

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

#define SPI_STATUS_WAIT_LIMIT 400
#define SPI_STATUS_PAUSE_MICROS 5

// i2c slave address of the DS3231 chip
#define DS3231_I2C_ADDR             0x68

#define BAUD 115200

#include <Wire.h>
#include <SPI.h>

const uint8_t DAYS_IN_MONTH[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

uint8_t lamp_setting = 0;

volatile uint8_t slave_busy = 0;
volatile uint8_t slave_count = 0;

/*
   Slave Ready interrupt handler. When the "SLAVE_BUSY" pin transitioins from
   LOW to HIGH, this method sets the slave_busy flag to 0. The byte output will
   then return its results.
*/
void interrupt_zero_low_to_high() {
  slave_busy = 0;
  ++slave_count;
}

/**
   Read the time from the DS3231 and return the current time as seconds since
   Jaunary 1, 2000, UTC.
*/
long ds3231_current_time() {

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
  long seconds_since_2000 = time_to_seconds_since_y2k(
                              days_since_2000, raw_hour, raw_minutes, raw_seconds);

  return seconds_since_2000;;
}

/**
   Calculates the number of days that have elapsed since 2000/01/01 on the prevailing
   timezone.

   Parameter  Contents
   ---------  --------
   year       Year with reference to 2000, in the range [0 .. 99]
   month      Month, in the range [1 .. 12] (conventional numbering)
   day        Day of the month, in the range [1 .. 31], depending on
              the value of month
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
long time_to_seconds_since_y2k(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds) {
  return ((((long)days * 24 + hours) * 60) + minutes) * 60 + seconds;
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

void sent_and_received(uint8_t sent, uint8_t received) {
  Serial.print("Just sent: ");
  Serial.print(sent, HEX);
  Serial.print(" and received: ");
  Serial.print(received, HEX);
  Serial.println(".");
}

uint8_t spi_transfer(uint8_t to_send) {
  noInterrupts();
  slave_busy = 1;
  interrupts();
  uint8_t received = SPI.transfer(to_send);
//  delayMicroseconds(SPI_TURNAROUND_MICROS);
//  delayMicroseconds(SPI_TURNAROUND_MICROS);
//  delayMicroseconds(SPI_TURNAROUND_MICROS);
//  delayMicroseconds(SPI_TURNAROUND_MICROS);
//  delayMicroseconds(SPI_TURNAROUND_MICROS);
//  if (slave_busy) {
//    Serial.println("Interrupt not working");
//  }

  while (slave_busy) {}
  return received;
}
/**
   Sends an SPI-level NO-OP command to the current slave. This is a cheap and
   chearful way to read the contents of the Slave's SPDR register.

   Returns: the contents of the slave's SPDR (SPI Transfer) register
*/
uint8_t spi_noop() {
  return spi_transfer(SPI_NOOP);
}

uint8_t spi_handshake(uint8_t to_send) {
  spi_transfer(to_send);
  return spi_noop();
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
uint8_t spi_run_command(uint8_t command) {
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
uint8_t spi_send_byte(uint8_t byte_index) {
  delayMicroseconds(SPI_TURNAROUND_MICROS);
  return spi_handshake((byte_index << 4) | SPI_SEND_BYTE);
}

/**
   Orders the slave to accept (i.e. receive) the specified number of bytes.

   Paramters:

   Name       Type            Contents
   ---------  ---------       ----------------------------------
   byte_count unsigned byte   Numberof bytes to receive, in range
                             [0 .. 15]. Note that the slave will treat
                             the next byte_count bytes as incoming data,
                             no matter what they contain.

   Returns: the contents of the SPI register.
*/
uint8_t spi_accept_bytes(uint8_t byte_count) {
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

/**
   Retrieves the status of the current slave

   Returns: the current slave's status
*/
uint8_t spi_get_slave_status() {
  return spi_handshake(SPI_LOAD_STATUS);
}

uint8_t spi_wait_for_status(uint8_t desired_status) {
  uint8_t iterations = 0;
  uint8_t status_to_return;
  do {
    status_to_return = spi_get_slave_status();
    ++ iterations;
  } while (status_to_return != desired_status && iterations < SPI_STATUS_WAIT_LIMIT);
  return status_to_return;
}

uint8_t spi_wait_for_command_to_complete() {
  uint8_t i = 0;
  uint8_t spi_value = 0;
  while (!spi_value && i < SPI_STATUS_WAIT_LIMIT) {
    delayMicroseconds(SPI_STATUS_PAUSE_MICROS);
    spi_value = spi_noop();
    ++i;
  }
  return spi_value;
}

uint8_t *retrieve_bytes(uint8_t count, uint8_t *output_buffer) {
  for (uint8_t i = 0; i < count; ++i) {
    output_buffer[i] = spi_send_byte(i);
    delayMicroseconds(SPI_STATUS_PAUSE_MICROS);
  }
  return output_buffer;
}

uint8_t *read_eeprom(uint16_t address, uint8_t count, uint8_t *output_buffer) {
  Serial.print("Reading from address: ");
  Serial.print(address, HEX);
  Serial.print(", count: ");
  Serial.print(count);
  Serial.println('.');
  spi_accept_bytes(2);
  uint8_t low = address & 0xFF;
  uint8_t high = (address >> 8) & 0xFF;
  Serial.print("Address split into: ");
  Serial.print(low, HEX);
  Serial.print(", high ");
  Serial.println(high);
  spi_transfer(low);
  spi_transfer(high);
  spi_run_command(CMD_STAGE_EEPROM);
  spi_wait_for_command_to_complete();
  return retrieve_bytes(count, output_buffer);
}

uint8_t * read_software_version(uint8_t * output_buffer) {
  spi_run_command(CMD_STAGE_SOFTWARE_VERSION);
  spi_wait_for_command_to_complete();
  return retrieve_bytes(4, output_buffer);
}

// Debug Debug Debug Debug Debug Debug Debug Debug
void dbg_display_value(const char *header, uint8_t value) {
  Serial.print(header);
  Serial.println(value, HEX);
}

void dbg_dump_and_clear_slave_SPDR() {
  Serial.print("Dump and clear slave SPDR: ");
  Serial.println(SPI.transfer(SPI_NOOP), HEX);
}

void dbg_slave_serial_number() {
  Serial.print("Serial no: ");
  Serial.println(spi_serial_number(), HEX);
}

void dbg_run_command(uint8_t command) {
  uint8_t command_status = spi_run_command(command);
  Serial.print("Status for command ");
  Serial.print(command);
  Serial.print(" is ");
  Serial.println(command_status, HEX);
}

void dbg_ping_slave() {
  Serial.print("Ping: ");
  Serial.println(spi_ping_slave());
}

void dbg_retrieve_byte(uint8_t byte_index) {
  Serial.print("Slave byte ");
  Serial.print(byte_index);
  Serial.print(" is: ");
  Serial.println(spi_send_byte(byte_index), HEX);
}

void dbg_retrieve_slave_status() {
  Serial.print("Current slave status is: ");
  Serial.println(spi_get_slave_status(), HEX);
}

void dbg_receive_n_bytes(uint8_t byte_count) {
  char * separator = "";
  Serial.print("Receiving ");
  Serial.print(byte_count);
  Serial.print(" bytes: ");
  for (uint8_t i = 0; i < byte_count; ++i) {
    Serial.print(separator);
    Serial.print(spi_send_byte(i), HEX);
    separator = ", ";
  }
  Serial.println("");
}

void dbg_accept_bytes(uint8_t byte_count) {
  Serial.print("Accepting bytes, SPI contained: ");
  Serial.println(spi_accept_bytes(byte_count));
}

void dbg_wait_for_status(uint8_t desired_status) {
  uint8_t actual_status = spi_wait_for_status(desired_status);
  Serial.print("Wanted status: ");
  Serial.print(desired_status, HEX);
  Serial.print(" and got status: ");
  Serial.println(actual_status, HEX);
}

void dbg_snapshot_slave() {
  dbg_run_command(CMD_DUMP_BUFFER);
  delay(5);
}

void setup() {
  pinMode(GREEN_LIGHT, OUTPUT);
  pinMode(RED_LIGHT, OUTPUT);
  pinMode(SLAVE_RESET, OUTPUT);
  digitalWrite(SLAVE_RESET, LOW);
  delay(20);
  digitalWrite(SLAVE_RESET, HIGH);

  pinMode(SPI_CLOCK, INPUT);
  pinMode(MASTER_IN_SLAVE_OUT, INPUT);
  pinMode(MASTER_OUT_SLAVE_IN, OUTPUT);
  pinMode(SLAVE_SELECT, OUTPUT);

  pinMode(MEASUREMENT_CONTROL, OUTPUT);
  digitalWrite(MEASUREMENT_CONTROL, HIGH);

  Wire.begin();
  SPI.begin();

  attachInterrupt(
    digitalPinToInterrupt(SLAVE_READY),
    interrupt_zero_low_to_high,
    FALLING);

  analogReference(DEFAULT);
  Serial.begin(BAUD);
  while (!Serial) {};
  Serial.println("Serial I/O ready.");
  delay(480);

  for (uint8_t i = 0; i < 16; ++i) {
    digitalWrite(GREEN_LIGHT, HIGH);
    digitalWrite(RED_LIGHT, LOW);
    delay(250);
    digitalWrite(GREEN_LIGHT, LOW);
    digitalWrite(RED_LIGHT, HIGH);
    delay(100);
  }
  digitalWrite(RED_LIGHT, LOW);
  while (!Serial) {}
  digitalWrite(RED_LIGHT, HIGH);

  dbg_dump_and_clear_slave_SPDR();
  dbg_receive_n_bytes(4);
  Serial.println("Setup completed.----------------------------");
}

void loop() {
  uint8_t byte_to_send;
  uint8_t received_byte;
  uint8_t received_buffer[4];
  uint8_t retrieved_status;
  uint8_t sensor_config = SENSITIVITY_HUNDRED | SCALE_UNITY;

  digitalWrite(GREEN_LIGHT, HIGH);
  digitalWrite(RED_LIGHT, LOW);

  digitalWrite(SLAVE_SELECT, LOW);

  dbg_ping_slave();  // Get ping count
  dbg_slave_serial_number();  // Get device serial number

  read_eeprom(0, 4, received_buffer);
  Serial.println("Hardware serial number: ");
  for (int i = 0; i < 4; ++i) {
    Serial.print(" ");
    Serial.print(received_buffer[i], HEX);
  }
  Serial.println('.');
  dbg_receive_n_bytes(4);

  read_software_version(received_buffer);
  Serial.println("Software version: ");
  for (int i = 0; i < 4; ++i) {
    Serial.print(" ");
    Serial.print(received_buffer[i], HEX);
  }
  Serial.println('.');
  dbg_receive_n_bytes(4);

  dbg_retrieve_slave_status();// Load slave status

  dbg_accept_bytes(4);

  received_buffer[0] = spi_transfer(0x1A); // First byte
  received_buffer[1] = spi_transfer(0x2B); // Second byte
  received_buffer[2] = spi_transfer(0x3C); // Third Byte
  received_buffer[3] = spi_transfer(0x4D); // Fourth byte

  Serial.println("From sending:");
  for (int i = 0; i < 4; ++i) {
    Serial.print(" ");
    Serial.print(received_buffer[i], HEX);
  }
  Serial.println(".");
  //  dbg_snapshot_slave();

  dbg_receive_n_bytes(4);
  dbg_retrieve_slave_status();     // Load slave status

  Serial.print("Setting sensor config to ");
  Serial.println(sensor_config, HEX);
  dbg_accept_bytes(1);
  spi_transfer(sensor_config);
  dbg_run_command(CMD_CONFIGURE_SENSOR);

  Serial.println("Reading the time and enabling measurement control.");
  long seconds_since_2000 = ds3231_current_time();
  digitalWrite(MEASUREMENT_CONTROL, LOW);
  retrieved_status = spi_noop();
  delay(100);
  digitalWrite(MEASUREMENT_CONTROL, HIGH);
  Serial.println("Disabling measurement control.");
  dbg_display_value("Status while counting: ", retrieved_status);
  delayMicroseconds(SPI_STATUS_PAUSE_MICROS);
  dbg_display_value("Status after counting: ", spi_noop());

  Serial.print("Measurement started at ");
  Serial.println(seconds_since_2000);
  dbg_run_command(CMD_READ_SENSOR); // Send command 2: read sensor
  dbg_display_value("Status after command 2: ", spi_wait_for_command_to_complete());

  dbg_receive_n_bytes(4);
  dbg_retrieve_slave_status();

  Serial.print("Interrupt count: ");
  Serial.println(slave_count);
  Serial.println("Loop Completed ------------------------------");
  delay(1000);
}

