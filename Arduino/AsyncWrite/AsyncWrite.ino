// TODO: Copyright, Gnu V3 License. Do not check in without.

static const char* message = "Hello, world!\n";
static const uint8_t message_length = 14;

class AsyncSerialWrite {
  /*
   * States that the writer can assume.
   */
public:
  enum State {
    CREATED,    // Created but not initialized
    IDLE,       // Not busy and ready to write.
    SENDING,    // Sending a string.
    DONE,       // Done sending
  };

  private:
    State state;
    const char * pbuffer;
    uint8_t length;
    uint8_t sent;

  public:
    AsyncSerialWrite() {
      state = CREATED;
      pbuffer = 0;
      length = 0;
      sent = 0;
    }

    State getState() const {
      return state; 
    }

    void init(long baud) {
      Serial.begin(baud);
      while(!Serial) {}
      state = IDLE;
    }

    bool send(uint8_t user_length, const char *user_pbuffer) {
      if (state == DONE || state == IDLE) {
         pbuffer = user_pbuffer;
         length = user_length;
         sent = 0;
         state = SENDING;
         return true; 
      }
      return false;
    }

    bool process() {
      switch (state) {
        case CREATED:
          return false;

        case IDLE:
          return true;
          
        case SENDING:
          if (sent == length) {
            pbuffer = 0;
            length = 0;
            sent = 0;
            state = DONE;
          } else {
            if (0 < Serial.availableForWrite()) {
              Serial.write(pbuffer[sent]);
              ++sent;
            }
          }
          return false;

        case DONE:
          return true;
       }
    }
};

static AsyncSerialWrite asyncWriter;

void setup() {
  // put your setup code here, to run once:
  asyncWriter.init(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (asyncWriter.process()) {
    asyncWriter.send(message_length, message);
  }
}
