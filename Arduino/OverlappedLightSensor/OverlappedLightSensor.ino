
#include <avr/io.h>
#include "Arduino.h"
#include "wiring_private.h"
#include "pins_arduino.h"

#define ANALOG_PIN_COUNT 8

#define BAUD 115200

#define GREEN_LIGHT 2
#define RED_LIGHT 3
#define DATA_SIZE 16
#define BUFFER_SIZE DATA_SIZE + 2

#define ACK 0xF0
#define NAK 0xF1

#define SEND (unsigned char) 'S'

/**
 * States used to control iteration through the light sensors.
 */
enum State {
  WAITING_FOR_START,   // Waiting for the start command from the data collector
  ACQUIRE_DATA,        // Need to acquire data from the currently nominated pin
  A_TO_D_IN_PROGRESS,  // Analog to digital conversion in progress
  SEND_DATA,           // Send data from the current pin and advance to next
  SEND_CHECKSUM,       // Data acquired, checksum needs to be sent
  WAIT_FOR_FINISH,     // Packet is completely sent
};

/*
 * A linear buffer that holds characters for transmission out the serial
 * USB port. It is designed to accept characters at its end while sending
 * them off the beginning. The buffer is laid out as follows:
 * 
 *    Byte No    Contents
 * ---------     -------------------------------------------------------------
 *           0   Data capacity, 0 - 255 bytes
 *  1 - last-1   Data, 16-bit integers in LITTLE ENDIAN format
 *        last   XOR checksum
 */
class DataBuffer {
  private:
    unsigned char * _byte_array;
    unsigned char * _checksum;
    uint8_t _capacity;
    uint8_t _buffer_size;
    uint8_t _put_index;
    uint8_t _take_index; 

    void put_byte(uint8_t value);

  public:
    DataBuffer(unsigned char * byte_array, uint8_t capacity);
    boolean init_packet();
    boolean send_value(uint16_t value);
    boolean send_complete();
    boolean can_take();
    boolean can_put();
    uint8_t take();
    boolean is_full();
    void send_checksum();
};

class SerialIO {
  private:
    DataBuffer * _data_buffer;
    
  public:
    SerialIO(DataBuffer * data_buffer);
    boolean put_value(uint16_t value);
    void send_checksum();
    boolean reset();
    void update();
    boolean is_full();
    boolean send_complete();
};

uint8_t analog_reference = DEFAULT;

unsigned char raw_data_buffer[BUFFER_SIZE];
DataBuffer data_buffer = DataBuffer(raw_data_buffer, DATA_SIZE);
SerialIO serial_io = SerialIO(&data_buffer);
State state = WAIT_FOR_FINISH;
uint8_t current_pin = 0;

void setup() {
  selectAnalogReference(analog_reference);
  Serial.begin(BAUD);
  pinMode(GREEN_LIGHT, OUTPUT);
  pinMode(RED_LIGHT, OUTPUT);
  digitalWrite(GREEN_LIGHT, HIGH);
  digitalWrite(RED_LIGHT, HIGH);

  delay(500);
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(GREEN_LIGHT, HIGH);
    digitalWrite(RED_LIGHT, LOW);
    delay(250);
    digitalWrite(GREEN_LIGHT, LOW);
    digitalWrite(RED_LIGHT, HIGH);
    delay(100);
  }
  digitalWrite(RED_LIGHT, LOW);
  while(!Serial) {}
  digitalWrite(RED_LIGHT, HIGH);
}

void loop() {
  serial_io.update();

  switch (state) {
    case WAITING_FOR_START:
      if (acquire_data()) {
        digitalWrite(RED_LIGHT, LOW);
        digitalWrite(GREEN_LIGHT, HIGH);
        current_pin = 0;
        serial_io.reset();
        state = ACQUIRE_DATA;
      }
      break;

    case ACQUIRE_DATA:
      selectAnalogPin(current_pin);
      state = A_TO_D_IN_PROGRESS;
      break;

    case A_TO_D_IN_PROGRESS:
      if (analogValueReady()) {
        state = SEND_DATA;
      }
      break;

    case SEND_DATA:
      serial_io.put_value(readAnalogValue());
      ++ current_pin;
      state = current_pin < ANALOG_PIN_COUNT ? ACQUIRE_DATA : SEND_CHECKSUM;
      break;

    case SEND_CHECKSUM:
      serial_io.send_checksum();
      state = WAIT_FOR_FINISH;
      break;

    case WAIT_FOR_FINISH:
      if (serial_io.send_complete()) {
        state = WAITING_FOR_START;
        digitalWrite(RED_LIGHT, HIGH);
        digitalWrite(GREEN_LIGHT, LOW);
      }
      break;    
  }
}

boolean acquire_data() {
  boolean result = false;
  if (0 < Serial.available()) {
    result = Serial.read() == SEND;   
  }

  return result;
}

void selectAnalogReference(uint8_t mode)
{
        // can't actually set the register here because the default setting
        // will connect AVCC and the AREF pin, which would cause a short if
        // there's something connected to AREF.
        analog_reference = mode;
}

void selectAnalogPin(uint8_t pin)
{
        uint8_t low, high;

#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
        if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#endif
        pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
        if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
        if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
        if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
        // the MUX5 bit of ADCSRB selects whether we're reading from channels
        // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
        ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
  
        // set the analog reference (high two bits of ADMUX) and select the
        // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
        // to 0 (the default).
        ADMUX = (analog_reference << 6) | (pin & 0x07);


        // start the conversion
        sbi(ADCSRA, ADSC);
}

boolean analogValueReady() {

        // ADSC is cleared when the conversion finishes
        return (bit_is_set(ADCSRA, ADSC));
}

uint16_t readAnalogValue() {
        // we have to read ADCL first; doing so locks both ADCL
        // and ADCH until ADCH is read.  reading ADCL second would
        // cause the results of each conversion to be discarded,
        // as ADCL and ADCH would be locked when it completed.
        uint8_t low  = ADCL;
        uint8_t high = ADCH;


        // combine the two bytes
        return (high << 8) | low;
}

SerialIO::SerialIO (DataBuffer * data_buffer) :
  _data_buffer(data_buffer) {
}

boolean SerialIO::put_value(uint16_t value) {
  return _data_buffer->send_value(value);
}

void SerialIO::send_checksum() {
  if (_data_buffer-> is_full()) {
    _data_buffer->send_checksum();    
  }
}

boolean SerialIO::reset() {
  return _data_buffer->init_packet();
}

void SerialIO::update() {
  if (0 < Serial.availableForWrite() && _data_buffer->can_take()) {
    Serial.write(_data_buffer->take());
  }
}

boolean SerialIO::is_full() {
  return _data_buffer->is_full();
}

boolean SerialIO::send_complete() {
  return _data_buffer->send_complete();
}

void DataBuffer::put_byte(uint8_t value) {
  _byte_array[_put_index++] = value;
  *_checksum ^= value;
}

DataBuffer::DataBuffer(unsigned char * byte_array, uint8_t capacity) :
  _byte_array(byte_array),
  _capacity(capacity) {
    _buffer_size = _capacity + 2;
    _put_index = _buffer_size;
    _take_index = _buffer_size;
    _checksum = _byte_array + _buffer_size - 1;
}

boolean DataBuffer::init_packet() {
  boolean result = _put_index == _buffer_size && _take_index == _buffer_size;
  if (result) {
    *_byte_array = _capacity;
    *_checksum = 0;
    _take_index = 0;
    _put_index = 1;
  }
}

boolean DataBuffer::send_value(uint16_t value) {
  boolean result = can_put();
  if (result) {
    put_byte(lowByte(value));
    put_byte(highByte(value));

    if (_byte_array + _put_index == _checksum) {
      ++_put_index; 
    }
  }
}

boolean DataBuffer::can_put() {
  return _put_index < _buffer_size;
}

boolean DataBuffer::send_complete() {
  return _take_index >= _buffer_size;
}

boolean DataBuffer::can_take() {
  return _take_index < _put_index;
}

uint8_t DataBuffer::take() {
  return _byte_array[_take_index++];
}

boolean DataBuffer::is_full() {
  return _put_index > _capacity;
}

void DataBuffer::send_checksum() {
  ++_put_index;
}

