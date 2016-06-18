/* SPI Slave for light measurement during the 2017 solar eclipse. Takes 
 * messurements when ordered to do so, and performs other functions on
 * request.
 *  
 * Copyright (C) 2016, The Winer Observatory, www.winer.org
 *
 * it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
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

#include <EEPROM.h>
#include <SPI.h>

#define _DEBUG 1 // Uncomment to enable debug output.

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

#define ACK_PULSE_MICROS 8

volatile uint32_t pulse_count;
volatile uint8_t i_o_status;
volatile int buffer_index;
volatile int buffer_limit;
volatile uint8_t received_byte;
volatile uint8_t buffer_state;
volatile uint8_t command;
volatile uint8_t ping_count;
volatile uint8_t command_has_been_run;
volatile uint8_t serial_toggle = 0;
volatile uint8_t data_buffer[4] = {0xFE, 0xDC, 0xBA, 0x98};

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
const uint8_t SOFTWARE_VERSION[] PROGMEM = {0, 9, 7, 0};

// ISR that counts an incoming rising edge from the sensor.
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

// SPI Interrupt routine and data

volatile uint8_t received_from_spdr = 0;
volatile uint8_t spdi_input_available = 0;

ISR (SPI_STC_vect) {
  received_from_spdr = SPDR;
  spdi_input_available = 1;
}

// Light sensor management functions


/*
 * Sets an output pin from a flag value.
 * 
 * Preconditions
 * -------------
 * The target pin must be configured for output.
 * 
 * 
 * Parameter  Contents
 * ---------  --------
 * pin        The pin to alter
 * flag       A boolean flag. The pin will be set HIGH if and
 *            only of flag is not 0.
 */
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
  uint8_t configuration = data_buffer[0];
  set_pin(SENSITIVITY_0, SENSITIVITY_LOW & configuration);
  set_pin(SENSITIVITY_1, SENSITIVITY_HIGH & configuration);

  set_pin(SCALE_2, SCALE_LOW & configuration);
  set_pin(SCALE_3, SCALE_HIGH & configuration);
}

// SPI Management functions.

/*
 * Acknowledges that a byte received via SPI has been handled.
 * 
 * Preconditions
 * -------------
 * The master must have sent a byte that has not been acknowledged
 * 
 * Postconditions:
 * --------------
 * A "got it" pulse is sent to the master. The acknowledgement pin is left in a
 * high impedence state with pullup resistor connected. This lets us wire all
 * response pins together without intervening gates.
 */

void ack_spi_send() {
  pinMode(INPUT_ACKNOWLEDGE, OUTPUT);
  digitalWrite(INPUT_ACKNOWLEDGE,HIGH);
  delayMicroseconds(ACK_PULSE_MICROS);
  digitalWrite(INPUT_ACKNOWLEDGE, LOW);
  delayMicroseconds(ACK_PULSE_MICROS);
  digitalWrite(INPUT_ACKNOWLEDGE, HIGH);
  delayMicroseconds(ACK_PULSE_MICROS);
  pinMode(INPUT_ACKNOWLEDGE, INPUT_PULLUP);
}

uint8_t perform_operation(uint8_t opcode, uint8_t command) {
  uint8_t result = command;

  switch (opcode) {
    case 0x0:  // No-op, do nothing.
      break;

    case 0x1:  // Reset status, RESERVED
      result = i_o_status = READY_IDLE;
      break;

    case 0x2:  // Read the sensor value
       {
        uint32_t local_sensor_value = pulse_count;
        for (int i = 0; i < 4; ++i) {
          data_buffer[i] = local_sensor_value & 0xFF;
          local_sensor_value >>= 8;
        }
        result = i_o_status = DATA_AVAILABLE | 0x4;
      }
    break;

    case 0x3:
      set_sensor_configuration();
      result = READY_IDLE;
      break;

    case 0x4:  // Set debug indicator
      digitalWrite(LIGHT_1, data_buffer[0] & 0x01 ? HIGH : LOW);
      digitalWrite(LIGHT_2, data_buffer[0] & 0x02 ? HIGH : LOW);
      digitalWrite(LIGHT_3, data_buffer[0] & 0x04 ? HIGH : LOW);
      result = i_o_status = READY_IDLE;
      break;

    case 0x5:  // Clear status
      result = i_o_status = READY_IDLE;
      break;

     case 0x6:  // Stage software serial number
      for (uint8_t i = 0; i < 4; ++i) {
        data_buffer[i] = pgm_read_byte(SOFTWARE_VERSION + i);
      }
      result = i_o_status = READY_IDLE;
      break;

    case 0x7:  // Stage 4 bytes of EEPROM, starting with the address in
      {        // bytes 0 and 1 of the buffer
        uint16_t address = word(data_buffer[1], data_buffer[0]);
        for (uint8_t i = 0; i < 4; ++i) {
          data_buffer[i] = EEPROM.read(address++);          
        }
      }
      result = i_o_status = READY_IDLE;
      break;

    case 0x8:  // Unused, ignore
    case 0x9:
    case 0xA:
    case 0xB:
    case 0xC:
    case 0xD:
      break;
  
   case 0x0E:  // Clears the I/O buffer.
      for (uint8_t pidx = 0; pidx < 4; ++pidx) {
        data_buffer[pidx] = 0;
      }
      result = i_o_status  = READY_IDLE;
      break;

    case 0x0F:  // Reserved for debug output.
      break;      
   }

  return result;
}

uint8_t process_command(uint8_t command) {
  uint8_t result = command;  // Default is to return the sent value;

  // If we are filling the input buffer, the input byte goes there. Otherwise
  // the byte represents a command to be run.
  if (buffer_index < buffer_limit) {
    data_buffer[buffer_index++] = received_byte;
  } else {
    // The most significant nybble contains the command value; the least significant
    // nybble contains the op-code.
    uint8_t value = (command >> 4) & 0xF;
    switch (command & 0xF) {
      case 0:  // No-op; byte exchange only
        break;

      case 1:  // Load instrument serial number. TODO(emintz) take from EEPROM
        result = serial_number;
        break;

      case 2:  // Perform high-level operation
        result = perform_operation(value, command);
        break;

      case 3:  // Return the specified byte in the I/O buffer.
        result = data_buffer[value];
        break;

      case 4:  // Set up to receive N bytes, where N == value;
        buffer_index = 0;
        result = buffer_limit = value;
        break;

      case 5:  // Return and post increment the ping count. 
        result = ping_count++;
        break;

      case 6:  // Return the current I/O status
        result = i_o_status;
        break;
    }
  }
  return result;
}

void setup() {
  // turn on SPI in slave mode, attach interrupt handler, and
  // load the "slave setting up" status value into the SPI register.
  // The master will exchange this value with the slave until the
  // slave clears the SPI register to indicate that it is ready.
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(4);
  noInterrupts();
  SPCR |= _BV(SPE);
  SPDR = 0xE0;  // Indicates slave is setting up.
  interrupts();

  // LED Setup. Do this early so we can blinkenlights.
  pinMode(LIGHT_1, OUTPUT);
  pinMode(LIGHT_2, OUTPUT);
  pinMode(LIGHT_3, OUTPUT); 
  digitalWrite(LIGHT_1, LOW);
  digitalWrite(LIGHT_2, LOW);
  digitalWrite(LIGHT_3, LOW);

#ifdef _DEBUG
  digitalWrite(LIGHT_1, HIGH);
  delay(100);
  digitalWrite(LIGHT_1, LOW);
  delay(200);
  Serial.begin(BAUD);
  while (!Serial) {}
  Serial.println("Hello, World!");
#endif

  pulse_count = 0;
  i_o_status = INITIALIZING;
  buffer_index = 0;
  buffer_limit = 0;
  received_byte = 0;
  buffer_state = 0;
  command = 0xFF;
  ping_count = 0;
  command_has_been_run = 1;

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
  // The acknowledgement function switches INPUT_ACKNOWLEDGE
  // to OUTPUT when D10 goes LOW, and to INPUT_PULLUP when D10 goes HIGH.

  pinMode(INPUT_ACKNOWLEDGE, INPUT_PULLUP);

#ifdef _DEBUG
  digitalWrite(LIGHT_1, HIGH);
  digitalWrite(LIGHT_2, HIGH);
  digitalWrite(LIGHT_3, HIGH);
  delay(500);
  digitalWrite(LIGHT_1, LOW);
  digitalWrite(LIGHT_2, LOW);
  digitalWrite(LIGHT_3, LOW);
#endif

  attachInterrupt(digitalPinToInterrupt(MEASUREMENT_CONTROL), start_and_stop_count, CHANGE);

  noInterrupts();
  SPDR = 0;
  i_o_status = READY_IDLE;
  interrupts();
  delayMicroseconds(100);
  SPI.attachInterrupt();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  delay(100);
}

void loop() {
  uint8_t raw_command;
  uint8_t have_command_to_run;
  noInterrupts();
  raw_command = received_from_spdr;
  have_command_to_run = spdi_input_available;
  spdi_input_available = 0;
  interrupts();

  if (have_command_to_run) {
    uint8_t result;

    result = process_command(raw_command);
    SPDR = result;
#ifdef _DEBUG
    Serial.print("Raw command: ");
    Serial.print(raw_command, HEX);
    Serial.print(", command result: ");
    Serial.println(result, HEX);
#endif
    ack_spi_send();   
  }
}
