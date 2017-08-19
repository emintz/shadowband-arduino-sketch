/*
 * Control sketch for the 74LV138-based solar eclipse shadow band detector.
 * 
 * I/O Port Assignments
 * 
 * Port Use
 * ---- -----------------------------------------------
 *    A Output: Counter read-out control: pair enablement
 *              and byte output selection
 *    C Input: read register value byte
 *    L Output: sensor configuration and counter clear, enable, and latch
 * 
 * All other registers are used pinwise
 * 
 * 
 * Pin assignments
 * 
 *    Pin(s) Signal
 * --------- ------------------------------------------------
 *        53 PB0 Output: SPI Slave Select
 *        52 PB1 Output: SPI Clock
 *        51 PB2 Output: SPI Master Out Slave In (MOSI)
 *        50 PB3 Output: SPI Master In Slave Out (MISO)
 *        
 *        49 PL0 Output: Sensor S0
 *        48 PL1 Output: Sensor S1
 *        47 PL2 Output: Sensor S2
 *        46 PL3 Output: Sensor S3
 *        45 PL4 Output: Sensor /Output Enable
 *        44 PL5 Output: Counter RCLK (latch the current count)
 *        43 PL6 Output: Counter /CLEAR (reset count)
 *        42 PL7 Output: not connected
 * 
 *        37 PC0 (A8)  Input: Bit 0 (LSB) of counter byte
 *        36 PC1 (A9)  Input: Bit 1 of the counter byte
 *        35 PC2 (A10) Input: Bit 2 of the counter byte
 *        34 PC3 (A11) Input: Bit 3 of the counter byte
 *        33 PC4 (A12) Input: Bit 4 of the counter byte
 *        32 PC5 (A13) Input: Bit 5 of the counter byte
 *        31 PC6 (A14) Input: Bit 6 of the counter byte
 *        30 PC7 (A15) Input: Bit 7 (MSB) of the counter byte
 *
 *        29 PA7 (AD7) Output: Control indicator LED (reserved)
 *        28 PA6 (AD6) Output: Counter Byte Select C (MSB)
 *        27 PA5 (AD5) Output: Counter Byte Select B
 *        26 PA4 (AD4) Output: Counter Byte Select A (LSB)
 *
 *        25 PA3 (AD3) Output: /Byte Output Enable (LOW enables all output)
 *
 *        24 PA2 (AD2) Output: Counter Pair Enable C (MSB)
 *        23 PA1 (AD1) Output: Counter Pair Enable B
 *        22 PA0 (AD0) Output: Counter Pair Enable A (LSB)
 *     
 *        21 PD0  Output: I2C SCL (White)
 *        20 PD1  Mixed:  I2C SDA (Gray)
 *
 *        15 RX3 Input: to GPS TX (Blue)
 *        14 TX3 Ooutput: to GPS RX (Green)
 *        13 PB7 Output: Have GPS Signal Indicator
 *        12 PB6 Output, Start counter read signal (before each counter is read)
 *        11 PB5 Output, Exposure end signal
 *        10 PB4 Output, Exposure start signal (used for scope trigger)                                                                                                               
 *         9 PH6 Input: Panic Button Not (low --> panic)
 *         8 PH5 Output: Indicator LED (status signal)
 *         7 PH4 Output: Second Turnover Signal
 *         6 PH3 Input: SD Card Inserted Not
 *         5 PE3 Output: GPS Enable (Yellow)
 *         4 PG5
 *         3 PE5 
 *         2 PE4 (INT5) Input: Rise on Second Start (GPS White)
 *         1 PE1 TX
 *         0 PE0 RX
 *
 * Peripheral Devices:
 * 
 *   Device  Description                         Data Sheet
 * --------  ----------------------------------  ----------------------------------------------------
 * 74LV138   3-Line To 8-Line Decoders/          http://www.ti.com/lit/ds/symlink/sn74hc138.pdf
 *           Demultiplexers
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
 * Data Record format. Note that byte numbers are in decimal.
 * 
 *     Bytes Contents TODO(emintz): not accurate. Fix
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

/*
 * Sensor configuration data.
 * 
 * Allowable exposure times in microseconds. The sensitivity configurations reference
 * these values.
 */
const uint16_t EXPOSURE_TIME_MICROSECONDS[] PROGMEM = {125, 250, 500, 1000};

/*
 * Sensor and exposure time configurations, ordered from least sensentive to most
 * sensitive. The bit layout follows, where 0 denotes LSB:
 * 
 * Bit(s)   Contents
 * ------   -----------------------------------------------------
 * 0-3      Sensor configuration, where 0 == S0, 1 = S1, 2 = S2, 3 = S3
 * 4-7      Exposure index in EXPOSURE_TIME_MICROSECONDS above
 * 
 * Relative sensitivity is given in the comments, units are sensor-sensitivity-seconds,
 * The numbers merely indicate relative nominal exposure, and are not guaranteed to be
 * linear. Note that constants of the form nnEmm, where nn and mm are decimal digits, 
 * indicate nn x 10^mm, per FORTRAN and (I think) Java specifications.
 */

const uint8_t SENSOR_CONFIGURATIONS[] PROGMEM = {
  SENSOR_DIVIDE_BY_ONE_HUNDRED | EXPOSURE_125,    // 1E-2 * 1.25E-4 = 1.25E-6
  SENSOR_DIVIDE_BY_ONE_HUNDRED | EXPOSURE_250,    // 1E-2 * 2.5E-4  = 2.5E-6
  SENSOR_DIVIDE_BY_ONE_HUNDRED | EXPOSURE_500,    // 1E-2 * 5E-4    = 5E-6
  SENSOR_DIVIDE_BY_ONE_HUNDRED | EXPOSURE_1000,   // 1E-2 * 1E-3    = 1E-5
  
  SENSOR_DIVIDE_BY_TEN         | EXPOSURE_125,    // 1E-1 * 1.25E-4 = 1.25E-5
  SENSOR_DIVIDE_BY_TEN         | EXPOSURE_250,    // 1E-1 * 2.5E-4  = 2.5E-5
  SENSOR_DIVIDE_BY_TEN         | EXPOSURE_500,    // 1E-1 * 5E-4    = 5E-5

  SENSOR_DIVIDE_BY_TWO         | EXPOSURE_125,    // 5E-1 * 1.25E-4 = 6.25E-5

  SENSOR_DIVIDE_BY_TEN         | EXPOSURE_1000,   // 1E-1 * 1E-3    = 1E-4
  
  SENSOR_TIMES_ONE             | EXPOSURE_125,    // 1E0  * 1.25E-4 = 1.25E-4   
  SENSOR_TIMES_ONE             | EXPOSURE_250,    // 1E0  * 2.5E-4  = 2.5E-4
  SENSOR_TIMES_ONE             | EXPOSURE_500,    // 1E0  * 5E-4    = 5E-4
  SENSOR_TIMES_ONE             | EXPOSURE_1000,   // 1E0  * 1E-3    = 1E-3
 
  SENSOR_TIMES_TEN             | EXPOSURE_125,    // 1E1  * 1.25E-4 = 1,25E-3
  SENSOR_TIMES_TEN             | EXPOSURE_250,    // 1E1  * 2.5E-4  = 2.5E-3
  SENSOR_TIMES_TEN             | EXPOSURE_500,    // 1E1  * 5E-4    = 5E-3
  SENSOR_TIMES_TEN             | EXPOSURE_1000,   // 1E1  * 1E-3    = 1E-2

  SENSOR_TIMES_ONE_HUNDRED     | EXPOSURE_125,    // 1E2  * 1.25E-4 = 1,25E-2
  SENSOR_TIMES_ONE_HUNDRED     | EXPOSURE_250,    // 1E2  * 2.5E-4  = 2.5E-2
  SENSOR_TIMES_ONE_HUNDRED     | EXPOSURE_500,    // 1E2  * 5E-4    = 5E-2
  SENSOR_TIMES_ONE_HUNDRED     | EXPOSURE_1000,   // 1E2  * 1E-3    = 1E-1  
};

#define SENSOR_CONFIGURATION_COUNT ((sizeof(SENSOR_CONFIGURATIONS))/(sizeof(SENSOR_CONFIGURATIONS[0])))
#define MAX_SENSOR_CONFIGURATION_INDEX (SENSOR_CONFIGURATION_COUNT - 1)

static const uint8_t HEX_DIGITS[] PROGMEM = 
    {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

static const uint8_t SOFTWARE_SERIAL_NUMBER[] PROGMEM = {0, 4, 0, 0};


static char formatted_date_buffer[32];

#define TIME_STATE_READY 0
#define TIME_STATE_SET 1
#define TIME_STATE_STAGED 2
#define TIME_STATE_SENDING 3

static volatile uint32_t second_base_in_millis = 0;

static TinyGPSPlus gps;

static eclipse_time timestamps[8];

static uint32_t counter_values[16];
static uint32_t lowest_count = 0xFFFFFFFF;
static uint32_t highest_count = 0;
static uint16_t exposure_time_micros = 0;

static uint8_t number_of_counters = 16; // [0 .. 16]
static uint8_t configuration_index = 0;
static uint8_t enable_debug_output = 0; // Nonzero send counter data --> console. 
static uint8_t sensor_sensitivity_setting = 0;  // Sensor pin settings. See PORTL.

static volatile uint8_t newest_time_index = 7;
static volatile uint8_t index_of_time_to_send = 0;
static volatile uint8_t time_state = TIME_STATE_READY;
static volatile uint8_t send_index = 0;
static volatile uint8_t second_turnover = 1;

// Objects for SD card access.
static SdFat sd_card;
static File file;

void setup() {
  pinMode(OUT_INDICATOR_LED, OUTPUT);
  digitalWrite(OUT_INDICATOR_LED, LOW);
  digitalWrite(OUT_RCLK, LOW);
  // Signal startup. 
  flash_indicator(STATUS_POWER_ON);

  Serial.begin(BAUD);
  while(!Serial) {}
  Serial.print("Starting. Sensor configuration count is: ");
  Serial.print(SENSOR_CONFIGURATION_COUNT, DEC);
  Serial.println(", configurations:");
  for (int i = 0; i < SENSOR_CONFIGURATION_COUNT; ++i) {
    Serial.print('[');
    Serial.print(i);
    Serial.print("]: ");
    uint8_t val = pgm_read_byte_near(SENSOR_CONFIGURATIONS + i);
    uint8_t hi = (val >> 4) & 0x0F;
    uint8_t lo = val & 0x0F;
    Serial.print(hi, HEX);
    Serial.print(lo, HEX);
    Serial.print('(');
    Serial.print(val, HEX);
    Serial.print(") ");
  }
  Serial3.begin(GPS_DEFAULT_BAUD);
  while(!Serial3) {}
  Serial.println("Serial port 3 initialized");
  Wire.begin();
  Serial.println("");
  Serial.println("I2C initialized.");

  SPI.begin();
  Serial.println("SPI initialized.");

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

  file = sd_card.open("solar.txt", FILE_WRITE);
  if (!file) {
    Serial.println("SD Card solar.txt fail");
    panic_loop(PANIC_SD_CARD_DATA_FILE_AWOL);
  }
  file.flush();

  // We use port L to configure the sensor and select the
  // displayed counter byte. Configure it for output.
  DDRL = 0xFF;
  // Sensor and counter initialization. TODO(emintz): Start at minimum
  // sensitivity and autorange.
  PORTL = SENSOR_DIVIDE_BY_ONE_HUNDRED;

  // Register A is used for counter selection. All bits are output
  DDRA = 0xFF;
  deselect_all_counters();
  
  // Set all bits in port B to input so we can read a counter
  // value in a single instruction
  DDRC = 0; 

  // Set registers F and K to output. F selects counters 0 - 7, and
  // K selects counters 8 - 15
  DDRF = 0xFF;
  DDRK = 0xFF;

  pinMode(IN_GPS_SECOND_START_SIGNAL, INPUT);
  pinMode(OUT_SECOND_TURNOVER, OUTPUT);
  pinMode(OUT_RCLK, OUTPUT);
  pinMode(OUT_SENSOR_OUTPUT_ENABLE_NOT, OUTPUT);
  pinMode(OUT_COUNTER_CLEAR_NOT, OUTPUT);

  pinMode(OUT_SPI_SLAVE_SELECT, OUTPUT);
  pinMode(OUT_SPI_CLOCK, OUTPUT);
  pinMode(OUT_SPI_MASTER_OUT_SLAVE_IN, OUTPUT);
  pinMode(IN_SPI_MASTER_IN_SLAVE_OUT, INPUT);
  pinMode(IN_SD_CARD_INSERTED_NOT, INPUT_PULLUP);
  pinMode(IN_PANIC_BUTTON_NOT, INPUT_PULLUP);
  pinMode(OUT_EXPOSURE_START_SIGNAL, OUTPUT);
  pinMode(OUT_EXPOSURE_END_SIGNAL, OUTPUT);
  pinMode(OUT_READ_START_SIGNAL, OUTPUT);
  pinMode(OUT_GPS_ENABLE, OUTPUT);
  pinMode(OUT_HAVE_GPS_SIGNAL, OUTPUT);
  pinMode(IN_DEBUG_1, INPUT_PULLUP);
  pinMode(IN_DEBUG_2, INPUT_PULLUP);

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
  digitalWrite(OUT_EXPOSURE_START_SIGNAL, LOW);
  digitalWrite(OUT_EXPOSURE_END_SIGNAL, LOW);
  digitalWrite(OUT_READ_START_SIGNAL, LOW);

  digitalWrite(OUT_GPS_ENABLE, HIGH);

  if (digitalRead(IN_SD_CARD_INSERTED_NOT) == LOW) {
    Serial.println("SD Card inserted. Recording is good to go!");
  } else {
    Serial.println("SD card is UNAVAILABLE!");
  }

  attachInterrupt(digitalPinToInterrupt(IN_GPS_SECOND_START_SIGNAL), 
      record_second_turnover, RISING);

  clear_all_counters();

  flash_pin(OUT_HAVE_GPS_SIGNAL, STATUS_LAMP_TEST);
  flash_pin(OUT_SECOND_TURNOVER, STATUS_LAMP_TEST);
  flash_indicator(STATUS_OK);
}

void loop() {
  lowest_count = 0xFFFFFFFF;
  highest_count = 0;
  check_for_console_debug();
  // TODO(emintz): rig IN_ALT_PANIC_NOT
  if (digitalRead(IN_ALT_PANIC_NOT) == LOW) {
    Serial.println("Panic button pushed!");
    halt_and_catch_fire();
    // There is no return. The only way to restart is to reset.
  }

  // Process pending GPS input
  while(Serial3.available()) {
    digitalWrite(OUT_HAVE_GPS_SIGNAL, HIGH);
    if(gps.encode(Serial3.read()))
      if (gps.time.isUpdated()) {
        process_new_time();
    }
  }
  digitalWrite(OUT_HAVE_GPS_SIGNAL, LOW);
  take_exposure();
  read_all_counters();
}

// GPS Time Management

void process_new_time() {
  if (gps.date.isValid() && gps.time.isValid()) {
    noInterrupts();
    uint32_t current_base_millis = second_base_in_millis;
    uint8_t current_second_turnover = second_turnover;
    interrupts();
    uint8_t index_of_time_to_set = newest_time_index;
    index_of_time_to_set = (++index_of_time_to_set) & 0x7;
    timestamps[index_of_time_to_set].set(gps);
    timestamps[index_of_time_to_set].set_base_millis(current_base_millis);
    newest_time_index = index_of_time_to_set;
//    if (digitalRead(IN_DEBUG_1) == LOW) {
//      Serial.print('[');
//      Serial.print(index_of_time_to_set);
//      print_dash();
//      Serial.print(current_second_turnover, DEC);
//      Serial.print(']');
//      Serial.print(timestamps[index_of_time_to_set].get_year(), DEC);
//      print_dash();
//      print_two_digit_dec(timestamps[index_of_time_to_set].get_month());
//      print_dash();
//      print_two_digit_dec(timestamps[index_of_time_to_set].get_day());
//      Serial.print(' ');
//   
//      print_two_digit_dec(timestamps[index_of_time_to_set].get_hour());
//      print_time_sep();
//      print_two_digit_dec(timestamps[index_of_time_to_set].get_minute());
//      print_time_sep();
//      print_two_digit_dec(timestamps[index_of_time_to_set].get_second());
//      Serial.print("--|");
//      Serial.print(timestamps[index_of_time_to_set].get_base_millis());
//      Serial.println("");
//    }
  }
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
// Debug management

/**
 * Sets enable_debug_management to TRUE if and only if the IN_DEBUG_2 pin is LOW.
 */
void check_for_console_debug() {
  enable_debug_output = (digitalRead(IN_DEBUG_2) == LOW);
}

// Counter read methods

void read_all_counters() {
  for (uint8_t counter_no = 0; counter_no < number_of_counters; ++counter_no) {
    uint8_t pair_number = counter_no >> 1;  // Fast divide by two
    uint8_t offset = (counter_no & 1) << 2; // Fast mod 2 then * 4.
    select_counter_pair_for_output(pair_number);
    uint32_t the_counter_value = read_selected_counter_value(offset);
    counter_values[counter_no] = the_counter_value;
    if (the_counter_value < lowest_count) {
      lowest_count = the_counter_value;
    }
    if (highest_count < the_counter_value) {
      highest_count = the_counter_value;
    }
  }

  if (enable_debug_output) {
    for (uint8_t index = 0; index < number_of_counters; ++index) {
      Serial.print(" [");
      Serial.print(index);
      Serial.print("], ");
      Serial.print(counter_values[index], HEX);
      Serial.println("");
    }
  }
}

/**
 * Reads the value from the currently selected counter
 * 
 * Preconditions: Counter pair must be selected and count
 *                must be latched.
 * 
 * Postconditions: Count read and assembled.
 * 
 * Invariants: Counter pair selection and count unchanged.
 *
 * Paramters:
 *
 * Name       Type            Contents
 * ---------  ---------       ----------------------------------
 * offset     unsigned byte   Offset of the byte selection within the
 *                            currently selected cunter pair. Valid
 *                            values are 0, for the low-order counter, and
 *                            4 for the high-order counter. Within each 
 *                            counter bytes are labeled [offset ... offset + 3]
 *                            with offset denoting the MOST significant byte.
 */
uint32_t read_selected_counter_value(uint8_t offset) {
  signal_read_start();
  uint32_t counter_value = 0;
  for (uint8_t byte_index = 0; byte_index < 4; ++byte_index) {
    select_counter_byte_to_output(offset + byte_index);
    enable_counter_output();
    NO_OP
    NO_OP
    uint8_t pinc_byte = PINC;
    counter_value <<= 8;
    counter_value |= pinc_byte;
    disable_counter_output();
  }
  return counter_value;
}


// Counter control methods

void deselect_all_counters() {
  PORTA = OUT_GLOBAL_BYTE_OUTPUT_ENABLE_NOT; // Outputs disabled
}

void disable_counter_output() {
  uint8_t porta = PORTA;
  PORTA = porta | OUT_GLOBAL_BYTE_OUTPUT_ENABLE_NOT; 
}

void enable_counter_output() {
  uint8_t porta = PORTA;
  PORTA = porta & (~OUT_GLOBAL_BYTE_OUTPUT_ENABLE_NOT); 
}

void select_counter_pair_for_output(uint8_t counter_pair_number) {
  uint8_t porta = PORTA;
  PORTA = (porta & CLEAR_COUNTER_SELECTION_MASK) | (counter_pair_number & 0x07);
}

void select_counter_byte_to_output(uint8_t byte_number) {
  uint8_t porta = PORTA;
  PORTA = (porta & CLEAR_BYTE_SELECTION_MASK) | ((byte_number << 4) & 0x70);
}

void store_current_counter_values() {
  uint8_t portl = PORTL;
  portl &= (~COUNTER_LATCH_MEASUREMENT);
  PORTL = portl | COUNTER_LATCH_MEASUREMENT;
  NO_OP
  NO_OP
  NO_OP
  NO_OP
  NO_OP
  PORTL = portl;
}

void clear_all_counters() {
  uint8_t portl = PORTL;
  PORTL = portl & (~COUNTER_CLEAR_NOT);
  NO_OP
  NO_OP
  NO_OP
  NO_OP
  NO_OP
  PORTL = portl | COUNTER_CLEAR_NOT;
}

/**
 * Configures and enables all light sensors. Note that the sensors are configured via port L.
 * 
 * Returns: the desired exposure time in microsesonds. The longest exposure time is 1000
 * microseconds == 1/1000 second.
 */
uint16_t configure_and_enable_sensors() {
  uint8_t portl = PORTL;
  uint8_t exposure_configuration = pgm_read_byte(SENSOR_CONFIGURATIONS + configuration_index);
  uint8_t exposure_index = (exposure_configuration >> 4) & 0x0F;
  if (enable_debug_output) {
    Serial.print("Before configuration PORTL: ");
    Serial.print(portl, HEX);
    Serial.println(", config index: ");
    Serial.print(configuration_index, DEC);
    Serial.print(", exp config: ");
    Serial.print(exposure_configuration, HEX);
    Serial.print(", index: ");
    Serial.print(exposure_index);
  }
  portl &= 0xF0;  // Clear the sensitivity bits
  sensor_sensitivity_setting = exposure_configuration & 0x0F; 
  portl |= (sensor_sensitivity_setting | SENSOR_OUTPUT_ENABLE);
  uint16_t exposure_time = pgm_read_word_near(EXPOSURE_TIME_MICROSECONDS + exposure_index);
  if (enable_debug_output) {
    Serial.print(", exposure time: ");
    Serial.print(exposure_time, DEC);
    Serial.print(", PORTL: ");
    Serial.println(portl, HEX);
  }
  PORTL = portl;
  return exposure_time;
}

/**
 * Disable all light sensors
 */
void disable_sensors() {
  uint8_t portl = PORTL;
  PORTL = portl & (~SENSOR_OUTPUT_ENABLE);
}

/**
 * Takes an exposure, which requires the following steps:
 * 
 * 1.  Select the timestamp that will represent the exposure time. This is the
 *     most recently set timestamp. Note that multiple timestamps may be set
 *     during a single second.
 * 2.  Send a sync pulse to signal exposure start to trigger a 'scope sweep, if needed.
 * 3.  Clear all counters.
 * 4.  Set the time base, the time at which the exposure started. 
 * 5.  Configure and enable the light sensors. This starts the actual exposure. Note that
 *     the configuration function returns the desired exposure time in microseconds.
 * 6.  Wait for the exposure to complete.
 * 7.  Disable the light sensors, which ends the expposure. No further pulses will be 
 *     counted during this exposure cycle.
 * 8,  Signal the exposure end, to trigger a 'scope sweep if needed.
 * 9.  Command the counters to transfer the current count values to their output
 *     buffers, whence they can be retrieved.
 */
void take_exposure() {
  index_of_time_to_send = newest_time_index;
  signal_exposure_start();
  clear_all_counters();
  timestamps[index_of_time_to_send].set_timestamp_millis(millis());
  exposure_time_micros = configure_and_enable_sensors();
  delayMicroseconds(exposure_time_micros);
  disable_sensors();
  signal_exposure_end();
  store_current_counter_values();
  read_all_counters();
  write_data_record();
  adjust_exposure_configuration();
}

void signal_read_start() {
  rising_pulse(OUT_READ_START_SIGNAL);
}

void signal_exposure_start() {
  rising_pulse(OUT_EXPOSURE_START_SIGNAL);
}

void signal_exposure_end() {
  rising_pulse(OUT_EXPOSURE_END_SIGNAL);
}

// Indicator LED control methods.

/**
 * Enters a non-terminating panic loop.
 */
void halt_and_catch_fire() {
  file.close();
  panic_loop(PANIC_HALT_AND_CATCH_FIRE);
}

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
  Serial.flush();
  for(;;){
    flash_indicator(code);
    delay(1000);
  }
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
  flash_pin(OUT_INDICATOR_LED, code);
}

/*
 * Function pulses the light connected to a specified pin.
 *
 * Paramters:
 *
 * Name       Type            Contents
 * ---------  ---------       ----------------------------------
 * pin        unsighed byte   The pin to which the light is connected. The pin
 *                            must be configured as OUTPUT and set LOW.
 * code       unsigned byte   Number of flashes per signal. A flash
 *                            consists of 250 ms ON followed by
 *                            250 ms OFF.
 */
 void flash_pin(uint8_t pin, uint8_t code) {
  for (uint8_t i = 0; i < code; ++i) {
    digitalWrite(pin, HIGH);
    delay(250);
    digitalWrite(pin, LOW);
    delay(250);
  }
}

// Low level pin control

/**
 * Sends the shortest possible rising pulse on the specified pin.
 * 
 * Paramters:
 *
 * Name       Type            Contents
 * ---------  ---------       ----------------------------------
 * pin        uint8_t         Digital output pin number 
 * 
 * Preconditions: The specified pin must be configured for output, and must
 *                be set low.
 *
 * Postconditions: Pulse emitted (low --> high --> low), and pin left low.
 */
void rising_pulse(uint8_t pin) {
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
}

// Write SD card records

/*
 * Writes the hardware configuration record. The record contains the
 * following information. Note that all information is encoded in
 * hexadecimal, per standards. Also write the current time.
 */
void write_hardware_serial_number() {
  write_byte(RECORD_HARDWARE_CONFIGURATION);
  write_byte(CONFIG_HW_MAJOR_VERSION);
  write_byte(CONFIG_HW_MINOR_VERSION);
  write_byte(CONFIG_HW_CHANGE_NUMBER);
  write_byte(CONFIG_HW_RESERVED);
  write_byte(CONFIG_HW_SERIAL_MSB);
  write_byte(CONFIG_HW_SERIAL_LSB);
  write_byte(CONFIG_NUMBER_OF_SENSORS);

  write_elapsed_time();
  write_current_time_field();
  write_record_end();
}

/**
 * Adjusts the configuration. This is the autoranging.
 */
void adjust_exposure_configuration() {
  if (enable_debug_output) {
    Serial.print("Configuration index: ");
    Serial.print(configuration_index, DEC);
    Serial.print(" lowest: ");
    Serial.print(lowest_count, DEC);
    Serial.print(", highest: ");
    Serial.print(highest_count, DEC);
  }
  
  if (MEASUREMENT_UPPER_LIMIT < highest_count) {
    if (0 < configuration_index) {
      -- configuration_index;
    }
  } else if (lowest_count < MEASUREMENT_DESIRED_LOWER_LIMIT) {
    if (configuration_index < MAX_SENSOR_CONFIGURATION_INDEX) {
      ++configuration_index;
    }
  }

  if (enable_debug_output) {
    Serial.print(", configuration index set to: ");
    Serial.println(configuration_index, DEC);
  }
}

/**
 * Writes a data record.
 */
void write_data_record() {
  write_data_record_type();
  write_number_of_counters();
  write_gps_time_index();
  write_sensor_config();
  write_elapsed_time();
  write_current_time_field();
  write_light_values(counter_values, number_of_counters);
  write_record_end();
}

/**
 * Writes the data record header. 
 */
void write_data_record_type() {
  write_byte(1);
}

/**
 * Write the number of counters
 */
void write_number_of_counters() {
  write_byte(number_of_counters);
}

void write_gps_time_index() {
  write_byte(index_of_time_to_send);
}

void write_sensor_config() {
  write_byte(sensor_sensitivity_setting);
}

/**
 * Writes the elapsed time since startup
 */
void write_elapsed_time() {
  write_uint32(micros());
}

/*
 * Reads all counters and writes their valus to the SD card.
 * 
 * Preconditions:
 *   Light observations must be available in the values[]  array. 
 */
void write_light_values(const  uint32_t * values, uint8_t sensor_count) {
  for (uint8_t sensor_number = 0; sensor_number < sensor_count; ++sensor_number) {
    uint32_t count = values[sensor_number];
    write_uint32(count);
  }
}

// Write field values

/**
 * Write the exposure start time.
 */
void write_current_time_field() {
  eclipse_time * ptime = timestamps + index_of_time_to_send;
  write_uint16(ptime->get_year());
  write_byte(ptime->get_month());
  write_byte(ptime->get_day());
  write_byte(ptime->get_hour());
  write_byte(ptime->get_minute());
  write_byte(ptime->get_second());
  write_uint32(ptime->get_base_millis());
  write_uint32(ptime->get_timestamp_millis());
}

// File management (SD Card)

void wait_for_sdcard_bus_surrender() {
  digitalWrite(OUT_SDCARD_SELECT_NOT, HIGH);
  delayMicroseconds(SD_CARD_SPI_SURRENDER_MICROSECONDS);
}

/*
 * Writes a new-line character to the file.
 */
void write_newline() {
  output_raw_byte('\n');
}

/*
 * Ends a record by writing a new-line character and flushing the buffer
 * to the output, and, if the output is going to the SD card, flushing the
 * output buffer.
 */
void write_record_end() {
  if (!enable_debug_output) {
    write_newline();
    file.flush();
    wait_for_sdcard_bus_surrender();
  } else {
    Serial.println("<<<");
    Serial.flush();
  }
}

/**
 * Outputs one raw byte to the desired destination. If enable_output_debug is TRUE
 * (i.e. non-zero), the byte gets written to Serial (the terminal). Otherwise, it
 * gets written to the SD card.
 */
void output_raw_byte(uint8_t value) {
  if (enable_debug_output) {
    Serial.write(value);
  } else {
    file.write(value);
  }
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
  output_raw_byte(pgm_read_byte(HEX_DIGITS + ((value >> 4) & 0xF)));
  output_raw_byte(pgm_read_byte(HEX_DIGITS + (value & 0xF)));
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

