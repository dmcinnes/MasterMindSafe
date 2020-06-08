/*

  LED7Segment/MAX7219/RawDrive
  Driving a series of 7-segment LED displays with the MAX7219 and raw SPI commands.

  For info and circuit diagrams see https://github.com/tardate/LittleArduinoProjects/tree/master/playground/LED7Segment/MAX7219/RawDrive

 */
#include <RotaryEncoder.h>

const uint8_t CS_PIN        = 10;
const uint8_t DIN_PIN       = 11;
const uint8_t CLK_PIN       = 13;
const uint8_t ENCODER_PIN_A = 2;
const uint8_t ENCODER_PIN_B = 3;
const uint8_t BUTTON_PIN    = 4;


// register addresses
const uint8_t MAX7219_noop        = 0x00;
const uint8_t MAX7219_digit0      = 0x01;
const uint8_t MAX7219_digit1      = 0x02;
const uint8_t MAX7219_digit2      = 0x03;
const uint8_t MAX7219_digit3      = 0x04;
const uint8_t MAX7219_digit4      = 0x05;
const uint8_t MAX7219_digit5      = 0x06;
const uint8_t MAX7219_digit6      = 0x07;
const uint8_t MAX7219_digit7      = 0x08;
const uint8_t MAX7219_seg_a       = 0x40;
const uint8_t MAX7219_seg_b       = 0x20;
const uint8_t MAX7219_seg_c       = 0x10;
const uint8_t MAX7219_seg_d       = 0x08;
const uint8_t MAX7219_seg_e       = 0x04;
const uint8_t MAX7219_seg_f       = 0x02;
const uint8_t MAX7219_seg_g       = 0x01;
const uint8_t MAX7219_seg_dp      = 0x80;
const uint8_t MAX7219_seg_blank   = 0x00;
const uint8_t MAX7219_decodeMode  = 0x09;
const uint8_t MAX7219_intensity   = 0x0a;
const uint8_t MAX7219_scanLimit   = 0x0b;
const uint8_t MAX7219_shutdown    = 0x0c;
const uint8_t MAX7219_displayTest = 0x0f;

const uint8_t MAX7219_DECODE_NONE = 0x00;
const uint8_t MAX7219_DECODE_0    = 0x01;
const uint8_t MAX7219_DECODE_LOW  = 0x0f;
const uint8_t MAX7219_DECODE_ALL  = 0xff;

const uint8_t seekPatternSize = 18;
const uint8_t seekPattern[][2] = {
  { MAX7219_digit3, MAX7219_seg_e },
  { MAX7219_digit3, MAX7219_seg_f },
  { MAX7219_digit3, MAX7219_seg_a },
  { MAX7219_digit3, MAX7219_seg_blank },
  { MAX7219_digit2, MAX7219_seg_a },
  { MAX7219_digit2, MAX7219_seg_blank },
  { MAX7219_digit1, MAX7219_seg_a },
  { MAX7219_digit1, MAX7219_seg_blank },
  { MAX7219_digit0, MAX7219_seg_a },
  { MAX7219_digit0, MAX7219_seg_b },
  { MAX7219_digit0, MAX7219_seg_c },
  { MAX7219_digit0, MAX7219_seg_d },
  { MAX7219_digit0, MAX7219_seg_blank },
  { MAX7219_digit1, MAX7219_seg_d },
  { MAX7219_digit1, MAX7219_seg_blank },
  { MAX7219_digit2, MAX7219_seg_d },
  { MAX7219_digit2, MAX7219_seg_blank },
  { MAX7219_digit3, MAX7219_seg_d }
};

RotaryEncoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

unsigned long currentMillis = 0;
unsigned long nextUpdate = 0;

// digits[0] is the left most side
volatile uint8_t digits[4];
volatile uint8_t currentDigit = 0;

/*
  Send a 16-bit command packet to the device,
  comprising the +reg+ register selection and +data+ bytes.
  Data is latched on the rising edge of CS_PIN
 */
void maxWrite(byte reg, byte data) {
  digitalWrite(CS_PIN, LOW);
  shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, reg);
  shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, data);
  digitalWrite(CS_PIN, HIGH);
}

/*
On initial power-up, all control registers are reset, the
display is blanked, and the MAX7219/MAX7221 enter
shutdown mode. Program the display driver prior to
display use. Otherwise, it will initially be set to scan one
digit, it will not decode data in the data registers, and
the intensity register will be set to its minimum value.
*/
void initialiseDisplay() {
  maxWrite(MAX7219_scanLimit,   0x03); // only scan digits 0, 1
  maxWrite(MAX7219_decodeMode,  MAX7219_DECODE_ALL);
  maxWrite(MAX7219_displayTest, 0x00); // normal operation
  maxWrite(MAX7219_shutdown,    0x00); // display off
}

/*
 Turn on the display with LED +intensity+ from 0x0 (min) to 0xF (max).
 */
void startupDisplay(byte intensity) {
  maxWrite(MAX7219_shutdown,    0x01); // display on
  maxWrite(MAX7219_intensity,   intensity);
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  initialiseDisplay();
  startupDisplay(0x03);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

  PCICR |= (1 << PCIE2); // turn on pin change interrupt for D pins (0-7)
  PCMSK2 |= (1 << PD4);  // turn PC interrupt mask on for just pin 4
  sei();
}

void loop() {
  currentMillis = millis();
  if (nextUpdate <= currentMillis) {
    nextUpdate = currentMillis + 25;
    digits[currentDigit] = encoderPosition();
    updateDigits();
  }
}

void updateDigits() {
  maxWrite(MAX7219_digit3, digits[0]);
  maxWrite(MAX7219_digit2, digits[1]);
  maxWrite(MAX7219_digit1, digits[2]);
  maxWrite(MAX7219_digit0, digits[3]);
}

const uint8_t timing = 50;
void pattern() {
  uint8_t prev;
  for (uint8_t i = 0; i < seekPatternSize; i++) {
    prev = i - 1;
    if (i < 0) {
      i += seekPatternSize;
    }
    if (seekPattern[prev][0] != seekPattern[i][0]) {
      maxWrite(seekPattern[prev][0], MAX7219_seg_blank);
    }
    maxWrite(seekPattern[i][0], seekPattern[i][1]);
    delay(timing);
  }
}

uint8_t encoderPosition() {
  long newPosition = encoder.getPosition() % 10;
  if (newPosition < 0) {
    newPosition += 10;
  }
  return uint8_t(newPosition);
}

void updateEncoder() {
  encoder.tick();
}

ISR(PCINT2_vect) {
  if (digitalRead(BUTTON_PIN) == LOW) {
    currentDigit = (currentDigit + 1) % 4;
  }
}
