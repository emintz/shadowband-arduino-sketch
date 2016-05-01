/* SPI Slave light measurement. Takes messurements when ordered to do so.
 * Always sends its serial number when master sends a command. Serial
 * Serial numbers range from 0 - 511. Even if we make 16 complete
 * instruments, serial numbers will range from 0 - 255.
 */

#include <EEPROM.h>
#include <SPI.h>

// Pin assignments
#define SPI_CLOCK 13
#define MASTER_IN_SLAVE_OUT 12
#define MASTER_OUT_SLAVE_IN 11
#define SLAVE_SELECT 10
#define INPUT_ACKNOWLEDGE 4
#define MEASUREMENT_CONTROL 3
#define SENSOR_IN 2
#define SENSITIVITY_0 A0
#define SENSITIVITY_1 A1
#define SCALE_2 A2
#define SCALE_3 A3

// TLS230XX configuration bits
#define SENSITIVITY_LOW   0x1
#define SENSITIVITY_HIGH  0x2
#define SCALE_LOW         0x4
#define SCALE_HIGH        0x8

#define LIGHT_1 7
#define LIGHT_2 8
#define LIGHT_3 9

#define BAUD 115200

// Status definitions
#define RUNNING_COMMAND  0x00
#define DATA_AVAILABLE   0x20
#define READY_IDLE       0x40
#define MEASURING        0x80
#define INITIALIZING     0xF0

volatile union {
  long for_initializion_only;
  uint8_t bytes[sizeof(long)];
} output_buffer;

volatile uint32_t pulse_count;
volatile uint8_t i_o_status;
volatile int buffer_index;
volatile int buffer_limit;
volatile uint8_t received_byte;
volatile uint8_t buffer_state;
volatile uint8_t command;
volatile uint8_t ping_count;
volatile uint8_t command_has_been_run;

const uint8_t serial_number = 0xED;

/*
 * Software version number: 4 bytes as follows:
 * 
 * Byte  Contents
 * ----  --------------------------------------------
 *    0  Major version, 0 .. 511
 *    1  Minor version, 0 .. 511
 *    2  Build version, 0 .. 511
 *    3  Unused, must be 0
 */
const uint8_t SOFTWARE_VERSION[] PROGMEM = {0, 9, 5, 0};

// Count an incoming rising edge from the sensor.
void count_rising_pulse() {
  ++pulse_count; 
}

/*
 * Interuppt routine bound to pin D3. The routine binds the counter
 * counter ISR to pin D2 when the signle goes LOW, and disconnets it 
 * when the counter goes HIGH. This "active low" convention allows the
 * signal to do double duty as the not OE signal for the DLS230 XX on
 * the chip's pin 3.
 */
void start_and_stop_count() {
  if (digitalRead(MEASUREMENT_CONTROL) == LOW) {
    pulse_count = 0;
    i_o_status = SPDR = MEASURING;
    attachInterrupt(digitalPinToInterrupt(SENSOR_IN), count_rising_pulse, RISING);    
  } else {
    detachInterrupt(digitalPinToInterrupt(SENSOR_IN));
    i_o_status = SPDR = READY_IDLE;
  }
}

// SPI Interrupt routine
ISR (SPI_STC_vect) {
  digitalWrite(INPUT_ACKNOWLEDGE, HIGH);
  uint8_t inputAckStatus = LOW;
  received_byte = SPDR;

  if (buffer_index < buffer_limit) {
    output_buffer.bytes[buffer_index++] = received_byte;
  } else {
    uint8_t value = (received_byte >> 4) & 0xF;
    switch(received_byte & 0xF) {
      case 0:  // No-Op; byte exchange only
        SPDR = i_o_status;
        digitalWrite(LIGHT_1, HIGH);
        digitalWrite(LIGHT_2, LOW); 
        break;
  
      case 1:  // Load instrument serial number.
        SPDR = serial_number;
        break;
  
      case 2:  // Command
        // inputAckStatus = HIGH;  // Leave INPUT_ACKNOWLEDGE active till command runs
        command_has_been_run = 0;
        command = value;
        digitalWrite(LIGHT_1, HIGH);
        digitalWrite(LIGHT_2, HIGH); 
        i_o_status = SPDR = RUNNING_COMMAND;
        break;
  
      case 3:  // Send Nth byte, where N == value. Precondition: have value to send.
        SPDR = output_buffer.bytes[value];
        break;

      case 4:  // Receive N bytes, where N == value.
        buffer_index = 0;
        SPDR = buffer_limit = value;
        break;

      case 5:  // Load and increment ping count (testing/handshake)
        SPDR = ping_count++;
        digitalWrite(LIGHT_1, LOW);
        digitalWrite(LIGHT_2, HIGH); 
        break;

      case 6:  // Load status
        SPDR = i_o_status;
        break;
    }
  }
  digitalWrite(INPUT_ACKNOWLEDGE, inputAckStatus);
}

void set_pin(uint8_t pin, uint8_t flag) {
  digitalWrite(pin, flag ? HIGH : LOW);
}

/*
 * Configures the sensor by setting its analog sentivity and scaling.
 * 
 * Preconditions
 * -------------
 * 
 * Byte 0 of the output buffer must contain the configuration byte.
 * With bit 0 representing the least signficant bit, the bits are
 * used as follows:
 * 
 * Bit  Pin Constant      Contents
 * ---  --- ------------- ---------------------------
 *   0   S0 SENSITIVITY_0 Least significant sensitivity bit
 *   1   S1 SENSITIVITY_1 Most significant sensitivity bit
 *   2   S2 SCALE_0       Least significant scale bit
 *   3   S3 SCALE_1       Most significant scale bit.
 */
void set_sensor_configuration() {
  uint8_t configuration = output_buffer.bytes[0];
  set_pin(SENSITIVITY_0, SENSITIVITY_LOW & configuration);
  set_pin(SENSITIVITY_1, SENSITIVITY_HIGH & configuration);

  set_pin(SCALE_2, SCALE_LOW & configuration);
  set_pin(SCALE_3, SCALE_HIGH & configuration);
}

void setup() {
  Serial.begin(BAUD);

  pulse_count = 0;
  i_o_status = INITIALIZING;
  buffer_index = 0;
  buffer_limit = 0;
  received_byte = 0;
  buffer_state = 0;
  command = 0;
  ping_count = 0;
  command_has_been_run = 1;

  // LED Setup
  pinMode(LIGHT_1, OUTPUT);
  pinMode(LIGHT_2, OUTPUT);
  pinMode(LIGHT_3, OUTPUT); 
  digitalWrite(LIGHT_1, LOW);
  digitalWrite(LIGHT_2, LOW);
  digitalWrite(LIGHT_3, LOW);

  // Set up the sensor control pins.
  pinMode(SENSITIVITY_0, OUTPUT);
  pinMode(SENSITIVITY_1, OUTPUT);
  pinMode(SCALE_2, OUTPUT);
  pinMode(SCALE_3, OUTPUT);

  // We need Master In Slave Out to be set to OOUTPUT, and
  // Master Out Slave In to INPUT
  pinMode(MASTER_IN_SLAVE_OUT, OUTPUT);

  // Set up the response line. The line will transition HIGH --> LOW
  // when the slave receives a byte via SPI, and will transition
  // LOW --> HIGH when the slave is ready to receive the next byte.
  pinMode(INPUT_ACKNOWLEDGE, OUTPUT);
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(INPUT_ACKNOWLEDGE, HIGH);
    delay(250);
    digitalWrite(INPUT_ACKNOWLEDGE, LOW);
    delay(250);
  }
  digitalWrite(INPUT_ACKNOWLEDGE, LOW);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  SPI.attachInterrupt();
  SPDR = 0xEE;

  output_buffer.bytes[0] = 0xFE;
  output_buffer.bytes[1] = 0xDC;
  output_buffer.bytes[2] = 0xBA;
  output_buffer.bytes[3] = 0x98;
  

  digitalWrite(LIGHT_1, HIGH);
  digitalWrite(LIGHT_2, HIGH);
  digitalWrite(LIGHT_3, HIGH);
  delay(500);
  digitalWrite(LIGHT_1, LOW);
  digitalWrite(LIGHT_2, LOW);
  digitalWrite(LIGHT_3, LOW);

  attachInterrupt(digitalPinToInterrupt(MEASUREMENT_CONTROL), start_and_stop_count, CHANGE);

  i_o_status = READY_IDLE;
  Serial.println("Initialization complete.");
}

/*
 * Main processing loop. Note that INPUT_ACKNOWLEDGE must be HIGH if and only 
 * if a command is to be run, and that the loop sets INPUT_ACKNOWLEDGE HIGH on
 * completion.
 */
void loop() {
  // Prevent commands from being run twice.
  uint8_t command_to_run = 0;
  noInterrupts();
  if (!command_has_been_run) {
    command_to_run = command;
    command_has_been_run = 1;
  }
  interrupts();

  switch (command_to_run) {
    case 0x0: // There is nothing to do. Don't change status.
      break;

    case 0x1:  // RESERVED
      noInterrupts();
      i_o_status = SPDR = READY_IDLE;
      interrupts();
 break;

    case 0x2:  // Read the sensor
      {
        uint32_t local_sensor_value = pulse_count;
        for (int i = 0; i < 4; ++i) {
          output_buffer.bytes[i] = local_sensor_value & 0xFF;
          local_sensor_value >>= 8;
        }
        noInterrupts();
        i_o_status = SPDR = DATA_AVAILABLE | 0x4;
        interrupts();
      }
      break;

    case 0x3:  // Configure the sensor
        set_sensor_configuration();
        noInterrupts();
        i_o_status = SPDR = READY_IDLE;
        interrupts();
        break;

    case 0x4:  // Set indicator
      digitalWrite(LIGHT_1, output_buffer.bytes[0] & 0x01 ? HIGH : LOW);
      digitalWrite(LIGHT_2, output_buffer.bytes[0] & 0x02 ? HIGH : LOW);
      digitalWrite(LIGHT_3, output_buffer.bytes[0] & 0x04 ? HIGH : LOW);
      noInterrupts();
      i_o_status = SPDR = READY_IDLE;
      interrupts();
      break;

    case 0x5:  // Clear status
      noInterrupts();
      i_o_status = SPDR = READY_IDLE;
      interrupts();
      break;

    case 0x6:  // Stage serial number
      for (uint8_t i = 0; i < 4; ++i) {
        output_buffer.bytes[i] = pgm_read_byte(SOFTWARE_VERSION + i);
      }
      noInterrupts();
      i_o_status = SPDR = READY_IDLE;
      interrupts();
      break;

    case 0x7:  // Stage 4 bytes of EEPROM, starting with the address in
      {        // bytes 0 and 1 of the buffer
        uint16_t address = word(output_buffer.bytes[1], output_buffer.bytes[0]);
        for (uint8_t i = 0; i < 4; ++i) {
          output_buffer.bytes[i] = EEPROM.read(address++);          
        }
      }
      noInterrupts();
      i_o_status = SPDR = READY_IDLE;
      interrupts();
      break;

    case 0x0E:
      for (uint8_t pidx = 0; pidx < 4; ++pidx) {
        output_buffer.bytes[pidx] = 0;
      }
      noInterrupts();
      i_o_status = SPDR = READY_IDLE;
      interrupts();
      break;

    case 0x0F:
      Serial.print("Ping Count: ");
      Serial.print(ping_count, HEX);
      Serial.print(", pulse count: ");
      Serial.print(pulse_count, HEX);
      Serial.print(", buffer index = ");
      Serial.print(buffer_index, HEX);
      Serial.print(", buffer limit: ");
      Serial.print(buffer_limit, HEX);
      Serial.print(", buffer contents:");
      for (int pidx = 0; pidx < 4; ++pidx) {
        Serial.print(' ');
        Serial.print(output_buffer.bytes[pidx], HEX);
      }
      Serial.println("");
      noInterrupts();
      i_o_status = SPDR = READY_IDLE;
      interrupts();
      break;      
  }
  digitalWrite(INPUT_ACKNOWLEDGE, LOW);
}

