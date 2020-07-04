/*

  LED7Segment/MAX7219/RawDrive
  Driving a series of 7-segment LED displays with the MAX7219 and raw SPI commands.

  For info and circuit diagrams see https://github.com/tardate/LittleArduinoProjects/tree/master/playground/LED7Segment/MAX7219/RawDrive

 */
#include <RotaryEncoder.h>
#include <Servo.h>

const uint8_t CS_PIN        = 11;
const uint8_t DIN_PIN       = 10;
const uint8_t CLK_PIN       = 13;
const uint8_t ENCODER_PIN_A = 3;
const uint8_t ENCODER_PIN_B = 2;
const uint8_t BUTTON_PIN    = 8;
const uint8_t SERVO_PIN     = 9;

const int correctNumLEDs[4]   = {4, 5, 6, 7};
const int correctPlaceLEDs[4] = {A0, A1, A2, A3};

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
const uint8_t MAX7219_digit_blank = 0x0F;
const uint8_t MAX7219_decodeMode  = 0x09;
const uint8_t MAX7219_intensity   = 0x0a;
const uint8_t MAX7219_scanLimit   = 0x0b;
const uint8_t MAX7219_shutdown    = 0x0c;
const uint8_t MAX7219_displayTest = 0x0f;

const uint8_t MAX7219_DECODE_NONE = 0x00;
const uint8_t MAX7219_DECODE_0    = 0x01;
const uint8_t MAX7219_DECODE_LOW  = 0x0f;
const uint8_t MAX7219_DECODE_ALL  = 0xff;

const uint8_t digitAddresses[4] {
  MAX7219_digit0,
  MAX7219_digit1,
  MAX7219_digit2,
  MAX7219_digit3
};

const uint8_t seekPatternSize = 18;
const uint8_t seekPattern[][2] = {
  { MAX7219_digit0, MAX7219_seg_e },
  { MAX7219_digit0, MAX7219_seg_f },
  { MAX7219_digit0, MAX7219_seg_a },
  { MAX7219_digit0, MAX7219_seg_blank },
  { MAX7219_digit1, MAX7219_seg_a },
  { MAX7219_digit1, MAX7219_seg_blank },
  { MAX7219_digit2, MAX7219_seg_a },
  { MAX7219_digit2, MAX7219_seg_blank },
  { MAX7219_digit3, MAX7219_seg_a },
  { MAX7219_digit3, MAX7219_seg_b },
  { MAX7219_digit3, MAX7219_seg_c },
  { MAX7219_digit3, MAX7219_seg_d },
  { MAX7219_digit3, MAX7219_seg_blank },
  { MAX7219_digit2, MAX7219_seg_d },
  { MAX7219_digit2, MAX7219_seg_blank },
  { MAX7219_digit1, MAX7219_seg_d },
  { MAX7219_digit1, MAX7219_seg_blank },
  { MAX7219_digit0, MAX7219_seg_d }
};
unsigned long nextPatternTick = 0;

const uint8_t waitLEDPatternSize = 12;
const uint8_t waitLEDPattern[waitLEDPatternSize] = {
  0b00000001,
  0b00010000,
  0b00001000,
  0b10000000,
  0b00000100,
  0b01000000,
  0b00000010,
  0b00100000,
  0b00000100,
  0b01000000,
  0b00001000,
  0b10000000,
};

const uint8_t A      = 0x77;
const uint8_t UpperC = 0x4E;
const uint8_t LowerC = 0x0D;
const uint8_t D      = 0x3D;
const uint8_t E      = 0x4F;
const uint8_t F      = 0x47;
const uint8_t UpperH = 0x37;
const uint8_t LowerH = 0x17;
const uint8_t UpperI = 0x06;
const uint8_t LowerI = 0x04;
const uint8_t K      = 0x57;
const uint8_t L      = 0x0E;
const uint8_t N      = 0x15;
const uint8_t UpperO = 0x77;
const uint8_t LowerO = 0x1D;
const uint8_t P      = 0x67;
const uint8_t R      = 0x05;
const uint8_t S      = 0x5B;
const uint8_t LowerT = 0x0F;
const uint8_t UpperT = 0x70;
const uint8_t LowerU = 0x1C;
const uint8_t UpperU = 0x3E;
const uint8_t Space  = 0x00;

const uint8_t IntroCrawlSize = 14;
const uint8_t IntroCrawl[IntroCrawlSize] =
{ UpperC, R, A, LowerC, K, Space, LowerT, LowerH, E, Space, UpperC, LowerO, D, E };

const uint8_t PressToLockCrawlSize = 13;
const uint8_t PressToLockCrawl[PressToLockCrawlSize] =
{ P, R, E, S, S, Space, LowerT, LowerO, Space, L, LowerO, LowerC, K};

const uint8_t LockedCrawlSize = 6;
const uint8_t LockedCrawl[LockedCrawlSize] =
{ L, LowerO, LowerC, K, E, D };

const uint8_t UnlockedCrawlSize = 8;
const uint8_t UnlockedCrawl[UnlockedCrawlSize] =
{ UpperU, N, L, LowerO, LowerC, K, E, D };

const uint8_t TriesCrawlSize = 11;
const uint8_t TriesCrawl[TriesCrawlSize] =
{ UpperT, LowerO, LowerT, A, L, Space, UpperT, R, LowerI, E, S };

unsigned long nextCrawlUpdate = 0;
uint8_t crawlOffset = 0;

unsigned long nextLEDUpdate = 0;
uint8_t LEDOffset = 0;

RotaryEncoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

Servo lockServo;

unsigned long currentMillis = 0;
unsigned long nextUpdate = 0;
unsigned long nextButtonCheck = 0;
unsigned long nextSelectedDigitBlink = 0;

// index 0 is the left most side
uint8_t secretCode[4];
uint8_t codeGuess[4];
uint8_t currentDigit = 0;

uint8_t attempts = 0;

bool digitBlinkOn = true;

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

const uint8_t stateCount = 12;
void (*stateFunctions[stateCount]) () =
  { stateIntro, stateWaitForLock, stateLock, stateLockMessage, stateReset, stateGuess, stateResult, stateSuccess, stateUnlock, stateUnlockMessage, stateTries, stateTriesCount };
enum States { Intro, WaitForLock, Lock, LockMessage, Reset, Guess, Result, Success, Unlock, UnlockMessage, Tries, TriesCount };
volatile uint8_t currentState = Intro;

/*
On initial power-up, all control registers are reset, the
display is blanked, and the MAX7219/MAX7221 enter
shutdown mode. Program the display driver prior to
display use. Otherwise, it will initially be set to scan one
digit, it will not decode data in the data registers, and
the intensity register will be set to its minimum value.
*/
void initialiseDisplay() {
  maxWrite(MAX7219_scanLimit,   0x03); // only scan digits 0-3
  maxWrite(MAX7219_decodeMode,  MAX7219_DECODE_NONE);
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

void lockDoor() {
  lockServo.write(5);
}

void unlockDoor() {
  lockServo.write(180);
}

void setup() {
  lockServo.attach(SERVO_PIN);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(correctNumLEDs[i], OUTPUT);
    pinMode(correctPlaceLEDs[i], OUTPUT);
  }
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  initialiseDisplay();
  startupDisplay(0x03);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

  randomSeed(analogRead(A5));

  updateLEDs(0, 0); // set them all off

  unlockDoor();
}

void clearSegmentDisplay() {
  maxWrite(digitAddresses[0], Space);
  maxWrite(digitAddresses[1], Space);
  maxWrite(digitAddresses[2], Space);
  maxWrite(digitAddresses[3], Space);
}

void clearDigitDisplay() {
  maxWrite(digitAddresses[0], MAX7219_digit_blank);
  maxWrite(digitAddresses[1], MAX7219_digit_blank);
  maxWrite(digitAddresses[2], MAX7219_digit_blank);
  maxWrite(digitAddresses[3], MAX7219_digit_blank);
}

void updateDigits() {
  for (uint8_t i = 0; i < 4; i++) {
    if (i == currentDigit && !digitBlinkOn) {
      maxWrite(digitAddresses[i], MAX7219_digit_blank);
    } else {
      maxWrite(digitAddresses[i], codeGuess[i]);
    }
  }
}

bool patternTick(const uint8_t pattern[][2], uint8_t size) {
  static int patternOffset = 0;
  if (patternOffset >= size) {
    patternOffset = 0;
  }
  maxWrite(pattern[patternOffset][0], pattern[patternOffset][1]);
  patternOffset++;
  return (patternOffset == size);
}

bool LEDPatternTick(const uint8_t pattern[], uint8_t size) {
  static int patternOffset = 0;
  if (patternOffset >= size) {
    patternOffset = 0;
  }
  uint8_t current = pattern[patternOffset];
  for (uint8_t i = 0; i < 4 ; i++) {
    digitalWrite(correctNumLEDs[i], (current >> i) & 0x1);
    digitalWrite(correctPlaceLEDs[i], (current >> (i + 4)) & 0x1);
  }
  patternOffset++;
  return (patternOffset == size);
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

bool buttonState() {
  // debounce state
  static uint16_t state = 0;
  state = (state << 1) | digitalRead(BUTTON_PIN) | 0xe000;
  return (state == 0xf000);
}

bool scrollTextTick(const uint8_t crawl[], const uint8_t size, long delta) {
  bool done = false;
  if (nextCrawlUpdate <= currentMillis) {
    nextCrawlUpdate = currentMillis + delta;
    crawlOffset += 1;
    if (crawlOffset > size + 4) {
      crawlOffset = 0;
      done = true;
    }
    for (uint8_t i = 0; i < 4; i++) {
      int column = crawlOffset + i - 4;
      if (column < 0 || column > (size - 1)) {
        maxWrite(digitAddresses[i], Space);
      } else {
        maxWrite(digitAddresses[i], crawl[column]);
      }
    }
  }
  return done;
}

void resetTextTick() {
  crawlOffset = 0;
}

void generateNewCode() {
  for (uint8_t i = 0; i < 4; i++) {
    secretCode[i] = random(0, 9);
  }
}

void clearGuess() {
  for (uint8_t i = 0; i < 4; i++) {
    codeGuess[i] = 0;
  }
}

bool checkCodeGuess() {
  uint8_t correctNum = 0;
  uint8_t correctPlace = 0;
  bool usedDigits[4] = {0, 0, 0, 0};
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      if (codeGuess[i] == secretCode[j] && !usedDigits[j]) {
        correctNum++;
        usedDigits[j] = true;
        break;
      }
    }
  }
  for (uint8_t i=0; i < 4; i++) {
    if (codeGuess[i] == secretCode[i]) {
      correctPlace++;
    }
  }
  updateLEDs(correctNum, correctPlace);
  return (correctPlace == 4);
}

void updateLEDs(uint8_t corNum, uint8_t corPla) {
  for (uint8_t i = 0; i < 4 ; i++) {
    digitalWrite(correctNumLEDs[i], LOW);
    digitalWrite(correctPlaceLEDs[i], LOW);
  }
  for (uint8_t j = 0; j < corNum; j++) {
    digitalWrite(correctNumLEDs[j], HIGH);
  }
  for (uint8_t k = 0; k < corPla; k++) {
    digitalWrite(correctPlaceLEDs[k], HIGH);
  }
}

void loop() {
  currentMillis = millis();
  stateFunctions[currentState]();
}

/* State Functions */

void stateIntro() {
  maxWrite(MAX7219_digit0, F);
  maxWrite(MAX7219_digit1, LowerI);
  maxWrite(MAX7219_digit2, N);
  maxWrite(MAX7219_digit3, D);
  updateLEDs(4, 0);
  delay(1500);
  maxWrite(MAX7219_digit0, LowerT);
  maxWrite(MAX7219_digit1, LowerH);
  maxWrite(MAX7219_digit2, E);
  maxWrite(MAX7219_digit3, MAX7219_seg_blank);
  updateLEDs(0, 4);
  delay(1500);
  maxWrite(MAX7219_digit0, UpperC);
  maxWrite(MAX7219_digit1, LowerO);
  maxWrite(MAX7219_digit2, D);
  maxWrite(MAX7219_digit3, E);
  updateLEDs(4, 4);
  delay(1500);
  updateLEDs(0, 0);
  currentState = WaitForLock;
}

void stateWaitForLock() {
  scrollTextTick(PressToLockCrawl, PressToLockCrawlSize, 200);
  if (nextButtonCheck <= currentMillis) {
    nextButtonCheck = currentMillis + 5;
    if (buttonState()) {
      clearSegmentDisplay();
      resetTextTick();
      attempts = 0;
      updateLEDs(0, 0);
      currentState = Lock;
    }
  }
  if (nextLEDUpdate <= currentMillis) {
    nextLEDUpdate = currentMillis + 100;
    LEDPatternTick(waitLEDPattern, waitLEDPatternSize);
  }
}

void stateLock() {
  lockDoor();
  if (nextPatternTick <= currentMillis) {
    nextPatternTick = currentMillis + 50;
    if (patternTick(seekPattern, seekPatternSize)) {
      currentState = LockMessage;
    }
  }
}

void stateLockMessage() {
  if (scrollTextTick(LockedCrawl, LockedCrawlSize, 200)) {
    currentState = Reset;
  }
}

void stateReset() {
  generateNewCode();
  clearGuess();
  encoder.setPosition(0);
  attempts = 1;
  currentDigit = 0;
  updateLEDs(0, 0);
  // stateGuess needs to decode the digits
  maxWrite(MAX7219_decodeMode, MAX7219_DECODE_ALL);
  currentState = Guess;
}

void stateGuess() {
  if (nextSelectedDigitBlink <= currentMillis) {
    digitBlinkOn = !digitBlinkOn;
    if (digitBlinkOn) {
      nextSelectedDigitBlink = currentMillis + 400;
    } else {
      nextSelectedDigitBlink = currentMillis + 50;
    }
  }

  if (nextUpdate <= currentMillis) {
    nextUpdate = currentMillis + 25;
    codeGuess[currentDigit] = encoderPosition();
    updateDigits();
  }

  if (nextButtonCheck <= currentMillis) {
    nextButtonCheck = currentMillis + 5;
    if (buttonState()) {
      currentDigit++;
      if (currentDigit == 4) {
        currentState = Result;
      } else {
        encoder.setPosition(codeGuess[currentDigit]);
      }
    }
  }
}

void stateResult() {
  if (checkCodeGuess()) {
    currentState = Success;
  } else {
    attempts++;
    currentDigit = 0;
    encoder.setPosition(codeGuess[currentDigit]);
    currentState = Guess;
  }
}

void stateSuccess() {
  digitBlinkOn = true;
  maxWrite(MAX7219_decodeMode, MAX7219_DECODE_ALL);

  updateLEDs(0, 4);
  clearDigitDisplay();
  delay(100);
  updateDigits();
  delay(200);

  updateLEDs(4, 0);
  clearDigitDisplay();
  delay(100);
  updateDigits();
  delay(200);

  updateLEDs(0, 4);
  clearDigitDisplay();
  delay(100);
  updateDigits();
  delay(200);

  updateLEDs(4, 0);
  clearDigitDisplay();
  delay(100);
  updateDigits();
  delay(200);

  updateLEDs(0, 4);
  clearDigitDisplay();
  delay(100);
  updateDigits();
  delay(200);

  updateLEDs(0, 0);

  // Unlock needs segment mode
  maxWrite(MAX7219_decodeMode, MAX7219_DECODE_NONE);
  clearSegmentDisplay();
  currentState = Unlock;
}

void stateUnlock() {
  unlockDoor();
  if (nextPatternTick <= currentMillis) {
    nextPatternTick = currentMillis + 50;
    if (patternTick(seekPattern, seekPatternSize)) {
      currentState = UnlockMessage;
    }
  }
}

void stateUnlockMessage() {
  if (scrollTextTick(UnlockedCrawl, UnlockedCrawlSize, 200)) {
    currentState = Tries;
  }
}

void stateTries() {
  if (scrollTextTick(TriesCrawl, TriesCrawlSize, 200)) {
    maxWrite(MAX7219_decodeMode, MAX7219_DECODE_ALL);
    currentState = TriesCount;
  }
}

void stateTriesCount() {
  if (attempts > 999) {
    maxWrite(digitAddresses[0], (attempts % 10000) / 1000);
  } else {
    maxWrite(digitAddresses[0], MAX7219_digit_blank);
  }
  if (attempts > 99) {
    maxWrite(digitAddresses[1], (attempts % 1000) / 100);
  } else {
    maxWrite(digitAddresses[1], MAX7219_digit_blank);
  }
  if (attempts > 9) {
    maxWrite(digitAddresses[2], (attempts % 100) / 10);
  } else {
    maxWrite(digitAddresses[2], MAX7219_digit_blank);
  }
  maxWrite(digitAddresses[3], attempts % 10);
  delay(4000);
  maxWrite(MAX7219_decodeMode, MAX7219_DECODE_NONE);
  currentState = WaitForLock;
}
