/**
 * GPS Sketch for Eclipse experiment
 */

#include <Arduino.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include "gps_config.h"
#include <eclipse_time.h>

#define TIME_STATE_READY 0
#define TIME_STATE_SET 1
#define TIME_STATE_STAGED 2
#define TIME_STATE_SENDING 3

volatile uint32_t second_base_in_millis = 0;

TinyGPSPlus gps;

static eclipse_time timestamps[8];

static volatile uint8_t serialzed_timestamp[sizeof(eclipse_time)];

volatile uint8_t newest_time_index = 7;
volatile uint8_t index_of_time_to_send = 0;
volatile uint8_t time_state = TIME_STATE_READY;
volatile uint8_t send_index = 0;

void setup() {
  pinMode(IN_SLAVE_SELECT_SS, INPUT);
  pinMode(IN_SPI_CLOCK_SCK, INPUT);
  pinMode(IN_MASTER_OUT_SLAVE_IN_MOSI, INPUT);
  pinMode(OUT_MASTER_IN_SLAVE_OUT_MISO, OUTPUT);

  
  // turn on SPI in slave mode
  SPCR |= bit (SPE);
  
  pinMode(IN_SECOND_START_SIGNAL, INPUT_PULLUP);
  pinMode(IN_EXPOSURE_START_SIGNAL, INPUT_PULLUP);
  pinMode(OUT_EXPOSURE_TIME_READY_TO_SEND, OUTPUT);
  pinMode(OUT_LED_EXPOSURE_TIME_STAGED, OUTPUT);
  pinMode(OUT_LED_EXPOSURE_TIME_SENDING, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(OUT_EXPOSURE_TIME_READY_TO_SEND, LOW);
  digitalWrite(OUT_LED_EXPOSURE_TIME_STAGED, LOW);
  digitalWrite(OUT_LED_EXPOSURE_TIME_SENDING, LOW);
  digitalWrite(13, HIGH);
  Serial.begin(HIGH_SPEED_BAUD);
  Serial1.begin(9600);
  while (!Serial){};
  Serial.println("Serial port 0 initialized");
  while (!Serial1) {}
  attachInterrupt(digitalPinToInterrupt(IN_SECOND_START_SIGNAL), record_second_turnover, RISING);
  attachInterrupt(digitalPinToInterrupt(IN_EXPOSURE_START_SIGNAL), record_exposure_start, FALLING);
}

void loop() {

  switch(time_state) {
    case TIME_STATE_READY:
      digitalWrite(OUT_LED_EXPOSURE_TIME_STAGED, LOW);
      digitalWrite(OUT_LED_EXPOSURE_TIME_SENDING, LOW);
      break;

    case TIME_STATE_SET:
      serialize_timestamp();
      break;

    case TIME_STATE_STAGED:
      digitalWrite(OUT_LED_EXPOSURE_TIME_STAGED, HIGH);
      break;

    case TIME_STATE_SENDING:
      digitalWrite(OUT_LED_EXPOSURE_TIME_SENDING, HIGH);
      break;
  }

  while (Serial1.available()) {
    if (gps.encode(Serial1.read()) && gps.time.isUpdated()) {
      process_new_data();
    }
  }
}

void serialize_timestamp() {
  send_index = 0;
  memcpy(serialzed_timestamp, timestamps + index_of_time_to_send, sizeof(eclipse_time));
  time_state = TIME_STATE_STAGED;
  digitalWrite(OUT_EXPOSURE_TIME_READY_TO_SEND, HIGH);
  digitalWrite(OUT_EXPOSURE_TIME_READY_TO_SEND, LOW);
  digitalWrite(OUT_LED_EXPOSURE_TIME_STAGED, HIGH);
}

void process_new_data() {
  if (gps.date.isValid() && gps.time.isValid()) {
    noInterrupts();
    uint32_t current_base_millis = second_base_in_millis;
    uint8_t index_of_time_to_set = newest_time_index;
    interrupts();
    index_of_time_to_set = (++index_of_time_to_set) & 0x7;
    timestamps[index_of_time_to_set].set(gps);
    timestamps[index_of_time_to_set].set_base_millis(current_base_millis);
    noInterrupts();
    newest_time_index = index_of_time_to_set;
    interrupts();
 
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
  } else {
    Serial.println("Waiting for valid date/time.");
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


// Interrupt routines

/**
 * Invoked when the GPS detects when a second turns over (i.e., when
 * the time changes from Nth second of the day to the N=1st second of
 * the day, e.g.). The function records the milliseconds since the
 * program starts.
 */
void record_second_turnover() {
  second_base_in_millis = millis(); 
}

/**
 * Records the time at which an exposure started.
 */
void record_exposure_start() {
  index_of_time_to_send = newest_time_index;
  timestamps[index_of_time_to_send].set_timestamp_millis(millis());
  time_state = TIME_STATE_SET;
}

// SPI interrupt routine
ISR (SPI_STC_vect) {
  if (send_index < sizeof(eclipse_time)) {
     SPDR = serialzed_timestamp[send_index++];
     if (send_index == sizeof(eclipse_time)) {
       time_state = TIME_STATE_READY;
     }
  } else {
     SPDR = 0xFF;
  }
}

