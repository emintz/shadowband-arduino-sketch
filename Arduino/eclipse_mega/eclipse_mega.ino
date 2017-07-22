 /**
 * Read the sensor values from Port C.
 * 
 * Register assignments
 * 
 * Register Use
 * -------- -----------------------------------------------
 *        A Output: sensor and counter byte control
 *        C Input: read register value byte
 *        F Output: select counters 0 - 7
 *        K Output: select counters 8 - 15
 * 
 * All other registers are used pinwise
 * 
 * 
 * Pin assignments
 * 
 *    Pin(s) Signal
 * --------- ------------------------------------------------
 *        37 PC0 (A8) Input: Bit 0 (LSB) of counter byte
 *        36 PC1 (A9) Input: Bit 1 of the counter byte
 *        35 PC2 (A10) Input: Bit 2 of the counter byte
 *        34 PC3 (A11) Input: Bit 3 of the counter byte
 *        33 PC4 (A12) Input: Bit 4 of the counter byte
 *        32 PC5 (A13) Input: Bit 5 of the counter byte
 *        31 PC6 (A14) Input: Bit 6 of the counter byte
 *        30 PC7 (A15) Input: Bit 7 (MSB) of the counter byte
 *
 *        29 PA7 (AD7) Output: Counter /GBU
 *        28 PA6 (AD6) Output: Counter /GBL
 *        27 PA5 (AD5) Output: Counter /GAU
 *        26 PA4 (AD4) Output: Counter /GAL
 *
 *        25 PA3 (AD3) Output: Sensor configuration S3
 *        24 PA2 (AD2) Output: Sensor configuration S2
 *        23 PA1 (AD1) Output: Sensor configuration S1
 *        22 PA0 (AD0) Output: Sensor configuration S0
 *        
 *       A15 PK7 (Analog pin 15), /Select counter 15
 *       A14 PK6 (Analog pin 14), /Select counter 14
 *       A13 PK5 (Analog pin 13), /Select counter 13
 *       A12 PK4 (Analog pin 12), /Select counter 12
 *       A11 PK3 (Analog pin 11), /Select counter 11
 *       A10 PK2 (Analog pin 10), /Select counter 10
 *        A9 PK1 (Analog pin 9), /Select counter 9
 *        A8 PK0 (Analog pin 8), /Select counter 8
 *        
 *        A7 PF7 (Analog pin 7), /Select counter 7
 *        A6 PF6 (Analog pin 6), /Select counter 6
 *        A5 PF5 (Analog pin 5), /Select counter 5
 *        A4 PF4 (Analog pin 4), /Select counter 4
 *        A3 PF3 (Analog pin 3), /Select counter 3
 *        A2 PF2 (Analog pin 2), /Select counter 2
 *        A1 PF1 (Analog pin 1), /Select counter 1
 *        A0 PF0 (Analog pin 0), /Select counter 0
 *
 *        D2 GPS Second Start (interrupt 1)
 *        D6 Digital pin 6, Counter RCLK
 *        D7 Digital pin 7, /Sensor Output Enable
 *
 *       D15 RX3 data sent from the GPS to the Mega
 *       D14 TX3 data sent from the Mega to the GPS.
 *       D13 not used
 *       D12 enable GPS.
 *
 * Libraries:
 * 
 *     Name Purpose and surce
 * -------- -----------------------------------------------------------------------
 *   Wire.h I2C protocol support. This is a standard package in the Arduino SDK
 * config.h [TK[
 * ds3231.h DS3231 real time clock management library. Requires Wire.h. Download
 *          from https://github.com/rodan/ds3231
 *
 * Peripheral Devices:
 * 
 *   Device  Description                         Data Sheet
 * --------  ----------------------------------  ----------------------------------------------------
 * 74HC125   Quadruple Bus Buffer Gates With     http://www.ti.com/lit/ds/symlink/sn74hc125.pdf
 *           3-State Outputs
 * 74LV8154N Dual 16-bit binary counters with    http://www.ti.com/lit/ds/symlink/sn74lv8154.pdf
 *           3-state output registers
 * TSL230RD  Programmable Light-toFrequency      http://www.mouser.com/ds/2/588/TSL230RDTSL230ARDTSL230BRD-P-519226.pdf
 *           Converters
 * Adafriut  SPI-compatible Micro-SD Card        https://www.adafruit.com/product/254
 *           adapter                             https://forum.arduino.cc/index.php?action=dlattach;topic=360325.0;attach=182293
 * DS3231    Real time clock                     https://www.maximintegrated.com/en/products/digital/real-time-clocks/DS3231.html
 *
 * Note: in the Arduino documentation, bit 0 is least significant. We follow that convention.
 *
 * Data Record Format. Note that byte 0 is the first byte written to the record.  All digits
 * are in hex, and all times are UTC. All records end with a new line ('\n') character.
 * 
 * The following record types are written.  Note that type numbers are hex. Values MAY be 
 * prefixed with '0x' in the documentation, but prefixes WILL NOT occur in the files them selves
 * 
 *   Type Contents
 *   ---- -------------------------------------------------
 *     00 Reserved.
 *     01 Data -- one set of readings from the sensor array
 * 
 * 
 * Data Record format. Note that byte numbers are in decimal.
 * 
 *     Bytes Contents
 *   ------- ----------------------------------------------------------
 *     00-01 Record type. 01, always 
 *     02-03 Number of counters (will be 0x10 in production systems)
 *        04 Current  GPS time index, in [0 .. 7]. This is initialized to 7 and becomes
 *           0 when GPS time becomes available.
 *        05 S3, S2, S1, S0 settings for the sensor. TODO(emintz): verify bit order.
 *     06-09 Exposure time in milliseconds
 *     10-17 Seconds since Midnight, January 1, 2000, UTC, from the DS3231 clock.
 *     18-21 Year, from GPS time 0 ==> GPS not synched.
 *     22-23 Month, in range [0x0 .. 0xC], 0 ==> GPS not synched
 *     24-25 Day of month, 0 ==> GPS not synched.
 *     26-27 Hour of day using 24 hour clock. Values will be [0x00 .. 0x17]
 *     28-29 Minute of hour, in the range [0x00 .. 0x3B]
 *     30-31 Second of minute, in the range [0x00 .. 0x3B]
 *     32-39 Time of most recently recorded second turnover, in milliseconds
 *           since program start.[1]
 *     47-47 Time exposure started, in milliseconds since program start[1]
 *     48-55 Counter 1 value
 *     56-63 Counter 2 value
 *       ... repeated till all counter values are recorded, then followed by a
 *           new line character ('\n').
 *     
 */

#include <EEPROM.h>   // EE PROM access library
#include <SPI.h>      // Serial Peripheral Interface library (for SD card)
#include <Wire.h>     // I2C Bus Library
#include <TinyGPS++.h>
#include "settings.h" // Application-specific settings
#include <config.h>   // [TK]
#include <ds3231.h>   // Real Time Clock library
#include <eclipse_time.h>
#include <SdFat.h>    // FAT File System library

static const uint8_t PROGMEM COUNTER_BYTE_SELECT[] = {
  0b01110000,
  0b10110000,
  0b11010000,
  0b11100000,
};

static const uint8_t HEX_DIGITS[] PROGMEM = 
    {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

static const uint8_t SOFTWARE_SERIAL_NUMBER[] PROGMEM = {0, 1, 0, 0};

static char formatted_date_buffer[32];

#define TIME_STATE_READY 0
#define TIME_STATE_SET 1
#define TIME_STATE_STAGED 2
#define TIME_STATE_SENDING 3

static volatile uint32_t second_base_in_millis = 0;
static uint32_t exposure_time_base_in_millis = 0;
static long secs_since_y2k = 0;

static TinyGPSPlus gps;

static eclipse_time timestamps[8];

// TODO(emintz): autorange exposure and sensitivity
static uint16_t exposure_time_micros = 2000;
static uint8_t counter_configuration = 0x3; // Max sensitivity.

static uint8_t newest_time_index = 7;
static volatile uint8_t index_of_time_to_send = 0;
static volatile uint8_t time_state = TIME_STATE_READY;
static volatile uint8_t send_index = 0;
static volatile uint8_t second_turnover = 1;

// Objects for SD card access.
static SdFat sd_card;
static File file;

void setup() {
  pinMode(OUT_INDICATOR_LED, OUTPUT);
  digitalWrite(OUT_RCLK, LOW);
  flash_indicator(STATUS_POWER_ON);

  Serial.begin(BAUD);
  while(!Serial) {}
  Serial3.begin(GPS_DEFAULT_BAUD);
  while(!Serial3) {}
  Wire.begin();
  Serial.println("");
  Serial.println("I2C initialized.");

  SPI.begin();
  Serial.println("SPI initialized.");

  // We use port A to configure the sensor and select the
  // displayed counter byte. Configure it for output.
  DDRA = 0xFF;
  PORTA = 0xF0;
  
  // Set all bits in port B to input so we can read a counter
  // value in a single instruction
  DDRC = 0; 

  // Set registers F and K to output. F selects counters 0 - 7, and
  // K selects counters 8 - 15
  DDRF = 0xFF;
  DDRK = 0xFF;
  deselect_all_counters();

  pinMode(IN_GPS_SECOND_START_SIGNAL, INPUT_PULLUP);
  pinMode(OUT_SECOND_TURNOVER, OUTPUT);
  pinMode(OUT_RCLK, OUTPUT);
  pinMode(OUT_SENSOR_OUTPUT_ENABLE_NOT, OUTPUT);
  pinMode(OUT_COUNTER_CLEAR_NOT, OUTPUT);

  pinMode(OUT_SPI_SLAVE_SELECT, OUTPUT);
  pinMode(OUT_SPI_CLOCK, OUTPUT);
  pinMode(OUT_SPI_MASTER_OUT_SLAVE_IN, OUTPUT);
  pinMode(IN_SPI_MASTER_IN_SLAVE_OUT, INPUT);
  pinMode(IN_SD_CARD_INSERTED_NOT, INPUT);
  pinMode(IN_PANIC_BUTTON_NOT, INPUT);
  pinMode(OUT_GPS_ENABLE, OUTPUT);
  pinMode(OUT_HAVE_GPS_SIGNAL, OUTPUT);

  digitalWrite(OUT_SECOND_TURNOVER, LOW);
  digitalWrite(OUT_SENSOR_OUTPUT_ENABLE_NOT, HIGH);
  digitalWrite(OUT_COUNTER_CLEAR_NOT, HIGH);
  digitalWrite(OUT_INDICATOR_LED, LOW);

  digitalWrite(OUT_SDCARD_SELECT_NOT, HIGH);
  digitalWrite(OUT_SPI_CLOCK, LOW);
  digitalWrite(OUT_SPI_MASTER_OUT_SLAVE_IN, LOW);
  digitalWrite(OUT_INDICATOR_LED, LOW);
  digitalWrite(OUT_GPS_ENABLE, LOW);
  digitalWrite(OUT_HAVE_GPS_SIGNAL, LOW);

  clear_all_counters();

  if (digitalRead(IN_SD_CARD_INSERTED_NOT) == HIGH) {
    Serial.println("SDCard present.");
  } else {
    Serial.println("SDCard missing.");
  }

  if (!sd_card.begin(OUT_SDCARD_SELECT_NOT, 2)) {
    Serial.println("SD Card begin fail");
    panic_loop(PANIC_SD_CARD_SELECT_FAILED);
  }

  if (!sd_card.card()) {
    panic_loop(PANIC_SD_CARD_DRIVER_AWOL);
  }

  if (!sd_card.chdir("/")) {
    panic_loop(PANIC_SD_CARD_ROOT_AWOL);
  }

  file = sd_card.open("solar.dat", FILE_WRITE);
  if (!file) {
    Serial.println("SD Card solar.dat fail");
    panic_loop(PANIC_SD_CARD_DATA_FILE_AWOL);
  }
  file.flush();

  digitalWrite(OUT_GPS_ENABLE, HIGH);
  attachInterrupt(digitalPinToInterrupt(IN_GPS_SECOND_START_SIGNAL), 
      record_second_turnover, RISING);

  flash_indicator(STATUS_OK);
  print_timestamp();
  Serial.println(". initialization complete.");
}

void print_dash() {
  Serial.print('-');
}

void print_time_sep() {
  Serial.print(':');
}

void print_two_digit_dec(uint8_t value) {
  if (value < 10) {
    Serial.print('0');
  }
  Serial.print(value, DEC);
}

void process_new_time() {
  if (gps.date.isValid() && gps.time.isValid()) {
    noInterrupts();
    uint32_t current_base_millis = second_base_in_millis;
    interrupts();
    uint8_t index_of_time_to_set = newest_time_index;
    index_of_time_to_set = (++index_of_time_to_set) & 0x7;
    timestamps[index_of_time_to_set].set(gps);
    timestamps[index_of_time_to_set].set_base_millis(current_base_millis);
    newest_time_index = index_of_time_to_set;
 
//    Serial.print('[');
//    Serial.print(index_of_time_to_set);
//    Serial.print(']');
//    Serial.print(timestamps[index_of_time_to_set].get_year(), DEC);
//    print_dash();
//    print_two_digit_dec(timestamps[index_of_time_to_set].get_month());
//    print_dash();
//    print_two_digit_dec(timestamps[index_of_time_to_set].get_day());
//    Serial.print(' ');
// 
//    print_two_digit_dec(timestamps[index_of_time_to_set].get_hour());
//    print_time_sep();
//    print_two_digit_dec(timestamps[index_of_time_to_set].get_minute());
//    print_time_sep();
//    print_two_digit_dec(timestamps[index_of_time_to_set].get_second());
//    Serial.print("--|");
//    Serial.print(timestamps[index_of_time_to_set].get_base_millis());
//    Serial.println("");
//  } else {
//    Serial.println("Waiting for valid date/time.");
  }
}

void loop() {
  if (digitalRead(IN_PANIC_BUTTON_NOT) == LOW) {
    halt_and_catch_fire();
    // There is no return. The only way to restart is to reset.
  }
  
//  format_date(formatted_date_buffer);
  secs_since_y2k = ds3231_current_time();
  PORTA = 0xF3;  // Counter bytes disabled, max counter sensitivity.
  exposure_time_base_in_millis = millis();
  take_exposure(exposure_time_micros);  // 1/500 second exposure.
  timestamps[newest_time_index].set_timestamp_millis(exposure_time_base_in_millis);
  PORTA = 0xF0;  // Sensors off. All counter bytes disabled.
  signal_read_start();
  uint32_t counter_value_0 = select_and_read_counter(0);
  uint32_t counter_value_1 = select_and_read_counter(1);
  const uint32_t values[]= {counter_value_0, counter_value_1};
  write_data_record(2, values);

//  Serial.print("At ");
//  Serial.print(formatted_date_buffer);
//  Serial.print(" newest time index: ");
//  Serial.print(get_newest_time_index());
//  Serial.print(", ");
//  Serial.println(secs_since_y2k);
//  Serial.print("Counter 0: ");
//  Serial.println(counter_value_0, HEX);
//  Serial.print("Counter 1: ");
//  Serial.println(counter_value_1, HEX);

  while(Serial3.available()) {
    if(gps.encode(Serial3.read()))
      digitalWrite(OUT_HAVE_GPS_SIGNAL, HIGH);
      if (gps.time.isUpdated()) {
        process_new_time();
    }
  }
  digitalWrite(OUT_HAVE_GPS_SIGNAL, LOW);
}

void select_counter(uint8_t counter_number) {
  uint8_t select_not = 0xFF;
  bitClear(select_not, counter_number & 0x7);
  if (counter_number > 7) {
    PORTF = 0xFF;
    PORTK = select_not;
  } else {
    PORTF = select_not;
    PORTK = 0xFF;
  }
}

void rising_pulse(uint8_t pin) {
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
}

void deselect_all_counters() {
  PORTF = 0xFF;
  PORTK = 0xFF;
}

void store_current_counter_values() {
  digitalWrite(OUT_RCLK, HIGH);
  digitalWrite(OUT_RCLK, LOW);
  delayMicroseconds(10);
}

void clear_all_counters() {
  digitalWrite(OUT_COUNTER_CLEAR_NOT, LOW);
  digitalWrite(OUT_COUNTER_CLEAR_NOT, HIGH);
  store_current_counter_values();  
}

void take_exposure(uint16_t exposure_time_micros) {
  signal_exposure_start();
  clear_all_counters();
  digitalWrite(OUT_SENSOR_OUTPUT_ENABLE_NOT, LOW);
  delayMicroseconds(exposure_time_micros);
  digitalWrite(OUT_SENSOR_OUTPUT_ENABLE_NOT, HIGH);
  store_current_counter_values();
}

void signal_read_start() {
}

void signal_exposure_start() {
}

uint32_t read_selected_counter_value() {
  uint32_t value = 0;
  for (uint8_t byte_index = 0; byte_index < 4; ++byte_index) {
    uint8_t porta_setting = pgm_read_byte(COUNTER_BYTE_SELECT + byte_index);
    PORTA = porta_setting;
    delayMicroseconds(100);
    value <<= 8;
    value |= PINC;
  }
  PORTA = 0xF0;
  return value;
}

uint32_t select_and_read_counter(uint8_t counter_number) {
  select_counter(counter_number);
  delayMicroseconds(25);
  uint32_t value = read_selected_counter_value();
  deselect_all_counters();
  return value;
}

// Real Time Clock Management

void print_timestamp() {
  format_date(formatted_date_buffer);
  Serial.print("Timestamp: ");
  Serial.print(formatted_date_buffer);
  Serial.print(", next valid time: ");
  uint8_t next_valid_time_to_print = get_newest_time_index();
  Serial.print(next_valid_time_to_print);
}

void format_date(char * date_buffer) {
  ts time_struct;
  DS3231_get(&time_struct);
   date_buffer = put_two_digits(date_buffer, time_struct.year_s);
   *date_buffer++ = '-';
   date_buffer = put_two_digits(date_buffer, time_struct.mon);
   *date_buffer++ = '-';
   date_buffer = put_two_digits(date_buffer, time_struct.mday);
   *date_buffer++ = ' ';
   date_buffer = put_two_digits(date_buffer, time_struct.hour);
   *date_buffer++ = ':';
   date_buffer = put_two_digits(date_buffer, time_struct.min);
   *date_buffer++ = ':';
   date_buffer = put_two_digits(date_buffer, time_struct.sec);
   *date_buffer++ = '\0';
}

 char * put_two_digits(char * date_buffer, uint8_t value) {
  *date_buffer++ = '0' + value / 10;
  *date_buffer++ = '0' + value % 10;
  return date_buffer;
 }

/**
 * Read the time from the DS3231 and return the current time as seconds since
 * Jaunary 1, 2000, UTC.
 */
 long ds3231_current_time() {
   long current_time = 0;

  ts time_struct;
  DS3231_get(&time_struct);

   uint16_t days_since_2000 = date_to_days_since_y2k(time_struct.year_s, time_struct.yday);
   return time_to_seconds_since_y2k(days_since_2000, time_struct.hour, time_struct.min, time_struct.sec);
 }

/**
 * Calculates the number of days that have elapsed since 2000/01/01 in the prevailing
 * timezone. By convention, the prevailing timezone is UTC
 * 
 * Parameter   Contents
 * ----------- --------
 * year        Year with reference to 2000, in the range [0 .. 99]
 * day_of_year Day of the year, in the range [0 .. 364] for normal years,
 *             and [0 .. 365] for leap years.
 */
int16_t date_to_days_since_y2k(uint8_t year, uint8_t day_of_year) {
  return day_of_year + 365 * year + (year + 3) / 4;  
}

/**
 * Converts a raw time value into the number of seconds since January 1, 2000 in the
 * same timezone. Note that this function returns value resuts only for dates occurring
 * on or after January 1, 2000. All values must be non-negative, and are rounded DOWN.
 * 
 * Parameters  Contents
 * ----------  --------
 * days        Number of days that have elapsed since January 1, 2000
 * hours       Number of hours elapsed from mignight in the current day
 * minutes     Number of minutes that have elapsed in the current hour
 * seconds     Number of seconds that have elapsed in the current minute
 */
long time_to_seconds_since_y2k(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds) {
  return ((((long)days * 24 + hours) * 60) + minutes) * 60 + seconds;
}

uint8_t get_newest_time_index() {
  return newest_time_index;
}

// File I/O functions.

// Write field values

/**
 * Fetch the current time in seconds since January 1, 2000, UTC, convert it
 * to Hex, and write the resulting 8 characters to disk.
 */
void write_current_time_field() {
    write_uint32(ds3231_current_time());
}

// File management (SD Card)

void wait_for_sdcard_bus_surrender() {
  digitalWrite(OUT_SDCARD_SELECT_NOT, HIGH);
  // TODO(emintz): any delay required here?
}

/*
 * Writes a new-line character to the file.
 */
void write_newline() {
  file.write('\n');
}

/*
 * Ends a record by writing a new-line character and flushing the buffer
 * to the SD card, and deactivating the SD card.
 */
void write_record_end() {
  write_newline();
  file.flush();
  wait_for_sdcard_bus_surrender();
}

/*
 * Encodes lease significant nybble of the specifed data byte in hexadecimal and writes the
 * resulting digit to the disk file
 *
 * Name        Type            Contents
 * ---------   ---------       ----------------------------------
 * value       byte            The value to write.
 */
void write_hex_digit(uint8_t value) {
  file.write(pgm_read_byte(HEX_DIGITS + (value & 0xF)));
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

/**
 * Writes a 16-bit unsigned integer value to the disk file. The value is
 * written as 4 hex digits.
 *
 * Name        Type            Contents
 * ---------   ---------       ----------------------------------
 * value       uint16_t        The value to write.
 */
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

void write_data_record(uint8_t value_count, const uint32_t* values) {
  write_byte(RECORD_OBSERVATION);
  write_byte(value_count);
  write_hex_digit(newest_time_index);
  write_hex_digit(counter_configuration);
  write_uint32(exposure_time_micros);
  write_uint32(secs_since_y2k);
  write_current_time_field();
  const eclipse_time& exposure_start_time = timestamps[newest_time_index];
  write_uint16(exposure_start_time.get_year());
  write_byte(exposure_start_time.get_month());
  write_byte(exposure_start_time.get_day());
  write_byte(exposure_start_time.get_hour());
  write_byte(exposure_start_time.get_minute());
  write_byte(exposure_start_time.get_second());
  write_uint32(exposure_start_time.get_base_millis());
  write_uint32(exposure_start_time.get_timestamp_millis());
  for (uint8_t i; i < value_count; ++i) {
    write_uint32(*(values + i));
  }
  write_record_end();
}


// Indicator LED control methods.

/*
 * Panic loop that pulses the indicator light until the master is reset.
 *
 * Paramters:
 *
 * Name       Type            Contents
 * ---------  ---------       ----------------------------------
 * code       unsigned byte   Number of flashes per signal. There will be
 *                            1 second wait between signals.
 */
void panic_loop(uint8_t code) {
  for(;;){
    flash_indicator(code);
    delay(1000);
  }
}

void halt_and_catch_fire() {
  file.close();
  panic_loop(PANIC_HALT_AND_CATCH_FIRE);
}

/*
 * Function pulses the indicator light once.
 *
 * Paramters:
 *
 * Name       Type            Contents
 * ---------  ---------       ----------------------------------
 * code       unsigned byte   Number of flashes per signal. A flash
 *                            consists of 250 ms ON followed by
 *                            250 ms OFF.
 */
void flash_indicator(uint8_t code) {
  for (uint8_t i = 0; i < code; ++i) {
    digitalWrite(OUT_INDICATOR_LED, HIGH);
    delay(250);
    digitalWrite(OUT_INDICATOR_LED, LOW);
    delay(250);
  }
}

// Interrupt routines

/**
 * Invoked when the GPS detects when a second turns over (i.e., when
 * the time changes from Nth second of the day to the N=1st second of
 * the day, e.g.). The function records the milliseconds since the
 * program starts.
 */
void record_second_turnover() {
  second_base_in_millis = millis();
  second_turnover ^= 1;
  digitalWrite(OUT_SECOND_TURNOVER, second_turnover ? HIGH : LOW);
}

