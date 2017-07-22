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
 *    L Output: sensor and counter clear, enable, and latchl
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

static const uint8_t HEX_DIGITS[] PROGMEM = 
    {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

static const uint8_t SOFTWARE_SERIAL_NUMBER[] PROGMEM = {0, 3, 0, 0};


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
static uint32_t lowest_count = 0xFFFFFFFF;
static uint32_t highest_count = 0;
static uint16_t exposure_time_micros = 2000;
static uint8_t counter_configuration = 0x3; // Max sensitivity.
static uint8_t number_of_counters = 8; // [0 .. 16]

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
  digitalWrite(OUT_INDICATOR_LED, LOW);
  digitalWrite(OUT_RCLK, LOW);
  flash_indicator(STATUS_POWER_ON);

  Serial.begin(BAUD);
  while(!Serial) {}
  Serial.println("Starting.");
  Serial3.begin(GPS_DEFAULT_BAUD);
  while(!Serial3) {}
  Wire.begin();
  Serial.println("");
  Serial.println("I2C initialized.");

  SPI.begin();
  Serial.println("SPI initialized.");

  // We use port L to configure the sensor and select the
  // displayed counter byte. Configure it for output.
  DDRL = 0xFF;
  // Sensor and counter initialization. TODO(emintz): Start at minimum
  // sensitivity and autorange.
  PORTL = SENSOR_TIMES_ONE_HUNDRED;

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

  pinMode(IN_GPS_SECOND_START_SIGNAL, INPUT_PULLUP);
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
  if (digitalRead(IN_PANIC_BUTTON_NOT) == LOW) {
    halt_and_catch_fire();
    // There is no return. The only way to restart is to reset.
  }

  // Process pending GPS input
  while(Serial3.available()) {
    if(gps.encode(Serial3.read()))
      digitalWrite(OUT_HAVE_GPS_SIGNAL, HIGH);
      if (gps.time.isUpdated()) {
        process_new_time();
    }
  }
  digitalWrite(OUT_HAVE_GPS_SIGNAL, LOW);
  take_exposure(1000000);
  read_all_counters();
}

// GPS Time Management

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
    if (digitalRead(IN_DEBUG_1) == LOW) {
      Serial.print('[');
      Serial.print(index_of_time_to_set);
      Serial.print(']');
      Serial.print(timestamps[index_of_time_to_set].get_year(), DEC);
      print_dash();
      print_two_digit_dec(timestamps[index_of_time_to_set].get_month());
      print_dash();
      print_two_digit_dec(timestamps[index_of_time_to_set].get_day());
      Serial.print(' ');
   
      print_two_digit_dec(timestamps[index_of_time_to_set].get_hour());
      print_time_sep();
      print_two_digit_dec(timestamps[index_of_time_to_set].get_minute());
      print_time_sep();
      print_two_digit_dec(timestamps[index_of_time_to_set].get_second());
      Serial.print("--|");
      Serial.print(timestamps[index_of_time_to_set].get_base_millis());
      Serial.println("");
    }
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

// Counter read methods

void read_all_counters() {
  uint32_t counter_values[16];
  for (uint8_t counter_no = 0; counter_no < number_of_counters; ++counter_no) {
    uint8_t pair_number = counter_no >> 1;  // Fast divide by two
    uint8_t offset = (counter_no & 1) << 2; // Fast mod 2 then * 4.
    select_counter_pair_for_output(pair_number);
    counter_values[counter_no] = read_selected_counter_value(offset);
  }

  if (digitalRead(IN_DEBUG_2) == LOW) {
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

void configure_counters(uint8_t configuration_nybble) {
  // TODO(emintz): validate configuration
  uint8_t portl = PORTL;
  PORTL = (portl & 0xF0) | (configuration_nybble & 0x0F);
}

void enable_sensors() {
  uint8_t portl = PORTL;
  PORTL = portl | SENSOR_OUTPUT_ENABLE;
}

void disable_sensors() {
  uint8_t portl = PORTL;
  PORTL = portl & (~SENSOR_OUTPUT_ENABLE);
}

void take_exposure(uint16_t exposure_time_micros) {
  signal_exposure_start();
  clear_all_counters();
  enable_sensors();
  delayMicroseconds(exposure_time_micros);
  disable_sensors();
  signal_exposure_end();
  store_current_counter_values();
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

