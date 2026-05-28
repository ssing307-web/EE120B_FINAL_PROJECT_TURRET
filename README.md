# EE120B_FINAL_PROJECT_TURRET
code workspace for turret final project


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timerISR-Fixed.h"

// -----------------------------
// Task scheduler struct
// -----------------------------
typedef struct task {
  signed char state;
  unsigned long period;
  unsigned long elapsedTime;
  int (*TickFct)(int);
} task;

// -----------------------------
// Arduino Mega pin mappings
// -----------------------------

// LEDs:
// Mega pin 25 = PA3
// Mega pin 27 = PA5
// Mega pin 29 = PA7
#define LED_LEFT    PA3
#define LED_CENTER  PA5
#define LED_RIGHT   PA7

// Laser:
// Mega pin 33 = PC4
#define LASER_PIN   PC4

// Buzzer:
// Mega pin 53 = PB0
#define BUZZER_PIN  PB0

// Servos:
// Mega pin 2 = PE4 = OC3B
// Mega pin 3 = PE5 = OC3C
#define Y_SERVO_PIN PE4
#define X_SERVO_PIN PE5

// Joystick and DIP pins:
// A0 = PF0 = joystick X
// A1 = PF1 = joystick Y
// A2 = PF2 = joystick switch
// A3 = PF3 = DIP power
// A4 = PF4 = DIP manual
// A5 = PF5 = DIP sentry
#define JOY_X_CH    0
#define JOY_Y_CH    1

#define JOY_SW_PIN  PF2
#define DIP_POWER   PF3
#define DIP_MANUAL  PF4
#define DIP_SENTRY  PF5

// OLED:
// Mega pin 20 = SDA
// Mega pin 21 = SCL/SCK
#define OLED_ADDR 0x3C

// -----------------------------
// Shared variables
// -----------------------------

enum Mode {
  MODE_POWER_OFF,
  MODE_STANDBY,
  MODE_MANUAL,
  MODE_SENTRY,
  MODE_ERROR
};

volatile enum Mode currentMode = MODE_POWER_OFF;

volatile unsigned int joyXValue = 512;
volatile unsigned int joyYValue = 512;

volatile unsigned char joyPressed = 0;

volatile unsigned char joyXLeft = 0;
volatile unsigned char joyXRight = 0;
volatile unsigned char joyXCenter = 1;

volatile unsigned char joyYUp = 0;
volatile unsigned char joyYDown = 0;
volatile unsigned char joyYCenter = 1;

volatile int xAngle = 90;
volatile int yAngle = 90;

volatile unsigned char flashToggle = 0;
volatile unsigned char oledAwake = 1;

// -----------------------------
// Tuning values
// -----------------------------

const int JOY_CENTER = 512;
const int DEADZONE = 130;

const int LEFT_THRESHOLD = 400;
const int RIGHT_THRESHOLD = 600;

const int X_SERVO_MIN = 20;
const int X_SERVO_MAX = 160;

// Smaller Y angle = more UP
// Bigger Y angle = more DOWN
const int Y_SERVO_MIN = 20;
const int Y_SERVO_MAX = 110;

const int SERVO_STEP = 7;

// -----------------------------
// Helper functions
// -----------------------------

int constrain_int(int value, int minVal, int maxVal) {
  if (value < minVal) {
    return minVal;
  }
  if (value > maxVal) {
    return maxVal;
  }
  return value;
}

void allOutputsOff() {
  PORTA &= ~((1 << LED_LEFT) | (1 << LED_CENTER) | (1 << LED_RIGHT));
  PORTC &= ~(1 << LASER_PIN);
  PORTB &= ~(1 << BUZZER_PIN);
}

void LEDsOff() {
  PORTA &= ~((1 << LED_LEFT) | (1 << LED_CENTER) | (1 << LED_RIGHT));
}

void LEDsAllOn() {
  PORTA |= (1 << LED_LEFT) | (1 << LED_CENTER) | (1 << LED_RIGHT);
}

void setLeftLED() {
  LEDsOff();
  PORTA |= (1 << LED_LEFT);
}

void setCenterLED() {
  LEDsOff();
  PORTA |= (1 << LED_CENTER);
}

void setRightLED() {
  LEDsOff();
  PORTA |= (1 << LED_RIGHT);
}

// -----------------------------
// ADC
// -----------------------------

void ADC_init() {
  ADMUX = (1 << REFS0);

  ADCSRA = (1 << ADEN) |
           (1 << ADPS2) |
           (1 << ADPS1) |
           (1 << ADPS0);

  ADCSRB = 0x00;
}

unsigned int ADC_read(unsigned char chnl) {
  ADMUX = (ADMUX & 0xF0) | (chnl & 0x0F);

  ADCSRA |= (1 << ADSC);

  while (ADCSRA & (1 << ADSC));

  return ADC;
}

// -----------------------------
// Servo PWM using Timer3
// Mega pin 2 = OC3B = PE4
// Mega pin 3 = OC3C = PE5
// -----------------------------

void ServoTimer3_init() {
  DDRE |= (1 << Y_SERVO_PIN) | (1 << X_SERVO_PIN);

  // Fast PWM, TOP = ICR3
  // Non-inverting PWM on OC3B and OC3C
  TCCR3A = (1 << COM3B1) | (1 << COM3C1) | (1 << WGM31);
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);

  // 20 ms period
  ICR3 = 39999;

  // Center-ish
  OCR3B = 3000;
  OCR3C = 3000;
}

unsigned int angleToCounts(int angle) {
  angle = constrain_int(angle, 0, 180);

  // 0-180 degrees mapped to 1000-2000 us
  // timer counts: 2000-4000
  return 2000 + ((long)angle * 2000L) / 180L;
}

void writeXServo(int angle) {
  OCR3C = angleToCounts(angle);
}

void writeYServo(int angle) {
  OCR3B = angleToCounts(angle);
}

// -----------------------------
// TWI / I2C for OLED
// Mega SDA = pin 20
// Mega SCL = pin 21
// -----------------------------

void TWI_init() {
  TWSR = 0x00;
  TWBR = 72;              // about 100 kHz
  TWCR = (1 << TWEN);
}

void TWI_start() {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

void TWI_stop() {
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void TWI_write(unsigned char data) {
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

// -----------------------------
// SH1106 OLED functions
// -----------------------------

void OLED_command(unsigned char cmd) {
  TWI_start();
  TWI_write(OLED_ADDR << 1);
  TWI_write(0x00);
  TWI_write(cmd);
  TWI_stop();
}

void OLED_setCursor(unsigned char page, unsigned char col) {
  col += 2;  // SH1106 offset

  OLED_command(0xB0 + page);
  OLED_command(0x00 + (col & 0x0F));
  OLED_command(0x10 + ((col >> 4) & 0x0F));
}

void OLED_writeByte(unsigned char data) {
  TWI_start();
  TWI_write(OLED_ADDR << 1);
  TWI_write(0x40);
  TWI_write(data);
  TWI_stop();
}

void OLED_init() {
  OLED_command(0xAE);
  OLED_command(0xD5);
  OLED_command(0x80);
  OLED_command(0xA8);
  OLED_command(0x3F);
  OLED_command(0xD3);
  OLED_command(0x00);
  OLED_command(0x40);
  OLED_command(0xAD);
  OLED_command(0x8B);
  OLED_command(0xA1);
  OLED_command(0xC8);
  OLED_command(0xDA);
  OLED_command(0x12);
  OLED_command(0x81);
  OLED_command(0xCF);
  OLED_command(0xD9);
  OLED_command(0x1F);
  OLED_command(0xDB);
  OLED_command(0x40);
  OLED_command(0xA4);
  OLED_command(0xA6);
  OLED_command(0xAF);
}

void OLED_clear() {
  for (unsigned char page = 0; page < 8; page++) {
    OLED_setCursor(page, 0);

    TWI_start();
    TWI_write(OLED_ADDR << 1);
    TWI_write(0x40);

    for (unsigned char col = 0; col < 128; col++) {
      TWI_write(0x00);
    }

    TWI_stop();
  }
}

void OLED_clearRow(unsigned char page) {
  OLED_setCursor(page, 0);

  TWI_start();
  TWI_write(OLED_ADDR << 1);
  TWI_write(0x40);

  for (unsigned char col = 0; col < 128; col++) {
    TWI_write(0x00);
  }

  TWI_stop();
}

void OLED_sleep() {
  OLED_clear();
  OLED_command(0xAE);
  oledAwake = 0;
}

void OLED_wake() {
  OLED_command(0xAF);
  oledAwake = 1;
}

// -----------------------------
// Simple 5x7 font
// -----------------------------

void OLED_drawPattern(const unsigned char p[5]) {
  for (unsigned char i = 0; i < 5; i++) {
    OLED_writeByte(p[i]);
  }

  OLED_writeByte(0x00);
}

void OLED_char(char c) {
  static const unsigned char SPACE[5] = {0x00,0x00,0x00,0x00,0x00};
  static const unsigned char COLON[5] = {0x00,0x36,0x36,0x00,0x00};

  static const unsigned char A[5] = {0x7E,0x11,0x11,0x11,0x7E};
  static const unsigned char B[5] = {0x7F,0x49,0x49,0x49,0x36};
  static const unsigned char D[5] = {0x7F,0x41,0x41,0x22,0x1C};
  static const unsigned char E[5] = {0x7F,0x49,0x49,0x49,0x41};
  static const unsigned char F[5] = {0x7F,0x09,0x09,0x09,0x01};
  static const unsigned char G[5] = {0x3E,0x41,0x49,0x49,0x7A};
  static const unsigned char I[5] = {0x00,0x41,0x7F,0x41,0x00};
  static const unsigned char L[5] = {0x7F,0x40,0x40,0x40,0x40};
  static const unsigned char M[5] = {0x7F,0x02,0x0C,0x02,0x7F};
  static const unsigned char N[5] = {0x7F,0x04,0x08,0x10,0x7F};
  static const unsigned char O[5] = {0x3E,0x41,0x41,0x41,0x3E};
  static const unsigned char P[5] = {0x7F,0x09,0x09,0x09,0x06};
  static const unsigned char R[5] = {0x7F,0x09,0x19,0x29,0x46};
  static const unsigned char S[5] = {0x46,0x49,0x49,0x49,0x31};
  static const unsigned char T[5] = {0x01,0x01,0x7F,0x01,0x01};
  static const unsigned char U[5] = {0x3F,0x40,0x40,0x40,0x3F};
  static const unsigned char W[5] = {0x7F,0x20,0x18,0x20,0x7F};
  static const unsigned char X[5] = {0x63,0x14,0x08,0x14,0x63};
  static const unsigned char Y[5] = {0x07,0x08,0x70,0x08,0x07};

  static const unsigned char ZERO[5] = {0x3E,0x51,0x49,0x45,0x3E};
  static const unsigned char ONE[5]  = {0x00,0x42,0x7F,0x40,0x00};
  static const unsigned char TWO[5]  = {0x42,0x61,0x51,0x49,0x46};
  static const unsigned char THREE[5]= {0x21,0x41,0x45,0x4B,0x31};
  static const unsigned char FOUR[5] = {0x18,0x14,0x12,0x7F,0x10};
  static const unsigned char FIVE[5] = {0x27,0x45,0x45,0x45,0x39};
  static const unsigned char SIX[5]  = {0x3C,0x4A,0x49,0x49,0x30};
  static const unsigned char SEVEN[5]= {0x01,0x71,0x09,0x05,0x03};
  static const unsigned char EIGHT[5]= {0x36,0x49,0x49,0x49,0x36};
  static const unsigned char NINE[5] = {0x06,0x49,0x49,0x29,0x1E};

  switch (c) {
    case ' ': OLED_drawPattern(SPACE); break;
    case ':': OLED_drawPattern(COLON); break;

    case 'A': OLED_drawPattern(A); break;
    case 'B': OLED_drawPattern(B); break;
    case 'D': OLED_drawPattern(D); break;
    case 'E': OLED_drawPattern(E); break;
    case 'F': OLED_drawPattern(F); break;
    case 'G': OLED_drawPattern(G); break;
    case 'I': OLED_drawPattern(I); break;
    case 'L': OLED_drawPattern(L); break;
    case 'M': OLED_drawPattern(M); break;
    case 'N': OLED_drawPattern(N); break;
    case 'O': OLED_drawPattern(O); break;
    case 'P': OLED_drawPattern(P); break;
    case 'R': OLED_drawPattern(R); break;
    case 'S': OLED_drawPattern(S); break;
    case 'T': OLED_drawPattern(T); break;
    case 'U': OLED_drawPattern(U); break;
    case 'W': OLED_drawPattern(W); break;
    case 'X': OLED_drawPattern(X); break;
    case 'Y': OLED_drawPattern(Y); break;

    case '0': OLED_drawPattern(ZERO); break;
    case '1': OLED_drawPattern(ONE); break;
    case '2': OLED_drawPattern(TWO); break;
    case '3': OLED_drawPattern(THREE); break;
    case '4': OLED_drawPattern(FOUR); break;
    case '5': OLED_drawPattern(FIVE); break;
    case '6': OLED_drawPattern(SIX); break;
    case '7': OLED_drawPattern(SEVEN); break;
    case '8': OLED_drawPattern(EIGHT); break;
    case '9': OLED_drawPattern(NINE); break;

    default: OLED_drawPattern(SPACE); break;
  }
}

void OLED_print(const char* str) {
  while (*str) {
    OLED_char(*str);
    str++;
  }
}

void OLED_printNum(int value) {
  if (value < 0) {
    value = 0;
  }

  if (value >= 100) {
    OLED_char('0' + (value / 100));
    value = value % 100;
    OLED_char('0' + (value / 10));
    OLED_char('0' + (value % 10));
  }
  else if (value >= 10) {
    OLED_char('0' + (value / 10));
    OLED_char('0' + (value % 10));
  }
  else {
    OLED_char('0' + value);
  }
}

void OLED_printMode() {
  if (currentMode == MODE_MANUAL) {
    OLED_print("MANUAL");
  }
  else if (currentMode == MODE_STANDBY) {
    OLED_print("STANDBY");
  }
  else if (currentMode == MODE_SENTRY) {
    OLED_print("SENTRY");
  }
  else if (currentMode == MODE_ERROR) {
    OLED_print("ERROR");
  }
  else {
    OLED_print("OFF");
  }
}

void OLED_updateXYRow() {
  if (currentMode == MODE_POWER_OFF || !oledAwake) {
    return;
  }

  OLED_clearRow(4);

  OLED_setCursor(4, 0);
  OLED_print("X:");
  OLED_printNum(xAngle);

  OLED_setCursor(4, 54);
  OLED_print("Y:");
  OLED_printNum(yAngle);
}

void OLED_updateTrigRow() {
  if (currentMode == MODE_POWER_OFF || !oledAwake) {
    return;
  }

  OLED_clearRow(6);

  OLED_setCursor(6, 0);
  OLED_print("TRIG:");

  if (joyPressed && currentMode == MODE_MANUAL) {
    OLED_print("ON");
  }
  else {
    OLED_print("OFF");
  }
}

void OLED_renderStatus() {
  if (currentMode == MODE_POWER_OFF) {
    if (oledAwake) {
      OLED_sleep();
    }
    return;
  }

  if (!oledAwake) {
    OLED_wake();
  }

  OLED_clear();

  OLED_setCursor(0, 0);
  OLED_print("TURRET SYSTEM");

  OLED_setCursor(2, 0);
  OLED_print("MODE:");
  OLED_printMode();

  OLED_updateXYRow();
  OLED_updateTrigRow();
}

// -----------------------------
// I/O initialization
// -----------------------------

void IO_init() {
  DDRA |= (1 << LED_LEFT) | (1 << LED_CENTER) | (1 << LED_RIGHT);

  DDRC |= (1 << LASER_PIN);

  DDRB |= (1 << BUZZER_PIN);

  // Joystick switch input with pull-up
  // Pressed = LOW
  DDRF &= ~(1 << JOY_SW_PIN);
  PORTF |= (1 << JOY_SW_PIN);

  // DIP switch inputs
  // External 10k pulldown:
  // OFF = LOW
  // ON = HIGH
  DDRF &= ~((1 << DIP_POWER) | (1 << DIP_MANUAL) | (1 << DIP_SENTRY));

  // Internal pullups off for DIP pins
  PORTF &= ~((1 << DIP_POWER) | (1 << DIP_MANUAL) | (1 << DIP_SENTRY));

  allOutputsOff();
}

// -----------------------------
// Task 1: ModeSM
// -----------------------------

enum ModeSM_States {
  MODE_SM_START,
  MODE_SM_POWER_OFF,
  MODE_SM_STANDBY,
  MODE_SM_MANUAL,
  MODE_SM_SENTRY,
  MODE_SM_ERROR
};

int TickFct_ModeSM(int state) {
  unsigned char systemPower = ((PINF & (1 << DIP_POWER)) != 0);
  unsigned char manualMode = ((PINF & (1 << DIP_MANUAL)) != 0);
  unsigned char sentryMode = ((PINF & (1 << DIP_SENTRY)) != 0);

  switch (state) {
    case MODE_SM_START:
      state = MODE_SM_POWER_OFF;
      break;

    case MODE_SM_POWER_OFF:
      if (systemPower) {
        state = MODE_SM_STANDBY;
      }
      break;

    case MODE_SM_STANDBY:
      if (!systemPower) {
        state = MODE_SM_POWER_OFF;
      }
      else if (manualMode && sentryMode) {
        state = MODE_SM_ERROR;
      }
      else if (manualMode) {
        state = MODE_SM_MANUAL;
      }
      else if (sentryMode) {
        state = MODE_SM_SENTRY;
      }
      break;

    case MODE_SM_MANUAL:
      if (!systemPower) {
        state = MODE_SM_POWER_OFF;
      }
      else if (manualMode && sentryMode) {
        state = MODE_SM_ERROR;
      }
      else if (!manualMode && !sentryMode) {
        state = MODE_SM_STANDBY;
      }
      else if (!manualMode && sentryMode) {
        state = MODE_SM_SENTRY;
      }
      break;

    case MODE_SM_SENTRY:
      if (!systemPower) {
        state = MODE_SM_POWER_OFF;
      }
      else if (manualMode && sentryMode) {
        state = MODE_SM_ERROR;
      }
      else if (!manualMode && !sentryMode) {
        state = MODE_SM_STANDBY;
      }
      else if (manualMode && !sentryMode) {
        state = MODE_SM_MANUAL;
      }
      break;

    case MODE_SM_ERROR:
      if (!systemPower) {
        state = MODE_SM_POWER_OFF;
      }
      else if (!manualMode && !sentryMode) {
        state = MODE_SM_STANDBY;
      }
      else if (manualMode && !sentryMode) {
        state = MODE_SM_MANUAL;
      }
      else if (!manualMode && sentryMode) {
        state = MODE_SM_SENTRY;
      }
      break;

    default:
      state = MODE_SM_START;
      break;
  }

  switch (state) {
    case MODE_SM_POWER_OFF:
      currentMode = MODE_POWER_OFF;
      break;

    case MODE_SM_STANDBY:
      currentMode = MODE_STANDBY;
      break;

    case MODE_SM_MANUAL:
      currentMode = MODE_MANUAL;
      break;

    case MODE_SM_SENTRY:
      currentMode = MODE_SENTRY;
      break;

    case MODE_SM_ERROR:
      currentMode = MODE_ERROR;
      break;

    default:
      currentMode = MODE_POWER_OFF;
      break;
  }

  return state;
}

// -----------------------------
// Task 2: JoystickSM
// -----------------------------

enum JoystickSM_States {
  JOY_SM_START,
  JOY_SM_READ
};

int TickFct_JoystickSM(int state) {
  switch (state) {
    case JOY_SM_START:
      state = JOY_SM_READ;
      break;

    case JOY_SM_READ:
      state = JOY_SM_READ;
      break;

    default:
      state = JOY_SM_START;
      break;
  }

  if (state == JOY_SM_READ) {
    joyXValue = ADC_read(JOY_X_CH);
    joyYValue = ADC_read(JOY_Y_CH);

    joyPressed = ((PINF & (1 << JOY_SW_PIN)) == 0);

    joyXLeft = 0;
    joyXRight = 0;
    joyXCenter = 0;

    if (joyXValue < JOY_CENTER - DEADZONE) {
      joyXLeft = 1;
    }
    else if (joyXValue > JOY_CENTER + DEADZONE) {
      joyXRight = 1;
    }
    else {
      joyXCenter = 1;
    }

    joyYUp = 0;
    joyYDown = 0;
    joyYCenter = 0;

    if (joyYValue < JOY_CENTER - DEADZONE) {
      joyYUp = 1;
    }
    else if (joyYValue > JOY_CENTER + DEADZONE) {
      joyYDown = 1;
    }
    else {
      joyYCenter = 1;
    }
  }

  return state;
}

// -----------------------------
// Task 3: ServoSM
// -----------------------------

enum ServoSM_States {
  SERVO_SM_START,
  SERVO_SM_HOLD,
  SERVO_SM_MANUAL
};

int TickFct_ServoSM(int state) {
  switch (state) {
    case SERVO_SM_START:
      state = SERVO_SM_HOLD;
      break;

    case SERVO_SM_HOLD:
      if (currentMode == MODE_MANUAL) {
        state = SERVO_SM_MANUAL;
      }
      break;

    case SERVO_SM_MANUAL:
      if (currentMode != MODE_MANUAL) {
        state = SERVO_SM_HOLD;
      }
      break;

    default:
      state = SERVO_SM_START;
      break;
  }

  if (state == SERVO_SM_MANUAL) {
    if (joyXLeft) {
      xAngle -= SERVO_STEP;
    }
    else if (joyXRight) {
      xAngle += SERVO_STEP;
    }

    if (joyYUp) {
      yAngle -= SERVO_STEP;
    }
    else if (joyYDown) {
      yAngle += SERVO_STEP;
    }

    xAngle = constrain_int(xAngle, X_SERVO_MIN, X_SERVO_MAX);
    yAngle = constrain_int(yAngle, Y_SERVO_MIN, Y_SERVO_MAX);

    writeXServo(xAngle);
    writeYServo(yAngle);
  }

  return state;
}

// -----------------------------
// Task 4: LedSM
// -----------------------------

enum LedSM_States {
  LED_SM_START,
  LED_SM_OFF,
  LED_SM_MANUAL,
  LED_SM_FLASH,
  LED_SM_ERROR
};

int TickFct_LedSM(int state) {
  switch (state) {
    case LED_SM_START:
      state = LED_SM_OFF;
      break;

    case LED_SM_OFF:
      if (currentMode == MODE_MANUAL && joyPressed) {
        state = LED_SM_FLASH;
      }
      else if (currentMode == MODE_MANUAL) {
        state = LED_SM_MANUAL;
      }
      else if (currentMode == MODE_ERROR) {
        state = LED_SM_ERROR;
      }
      break;

    case LED_SM_MANUAL:
      if (currentMode != MODE_MANUAL) {
        if (currentMode == MODE_ERROR) {
          state = LED_SM_ERROR;
        }
        else {
          state = LED_SM_OFF;
        }
      }
      else if (joyPressed) {
        state = LED_SM_FLASH;
      }
      break;

    case LED_SM_FLASH:
      if (currentMode != MODE_MANUAL) {
        if (currentMode == MODE_ERROR) {
          state = LED_SM_ERROR;
        }
        else {
          state = LED_SM_OFF;
        }
      }
      else if (!joyPressed) {
        state = LED_SM_MANUAL;
      }
      break;

    case LED_SM_ERROR:
      if (currentMode != MODE_ERROR) {
        state = LED_SM_OFF;
      }
      break;

    default:
      state = LED_SM_START;
      break;
  }

  switch (state) {
    case LED_SM_OFF:
      LEDsOff();
      break;

    case LED_SM_MANUAL:
      if (joyXValue < LEFT_THRESHOLD) {
        setLeftLED();
      }
      else if (joyXValue > RIGHT_THRESHOLD) {
        setRightLED();
      }
      else {
        setCenterLED();
      }
      break;

    case LED_SM_FLASH:
    case LED_SM_ERROR:
      flashToggle = !flashToggle;

      if (flashToggle) {
        LEDsAllOn();
      }
      else {
        LEDsOff();
      }
      break;

    default:
      break;
  }

  return state;
}

// -----------------------------
// Task 5: BuzzerLaserSM
// -----------------------------

enum BuzzerLaserSM_States {
  BL_SM_START,
  BL_SM_OFF,
  BL_SM_ON
};

int TickFct_BuzzerLaserSM(int state) {
  switch (state) {
    case BL_SM_START:
      state = BL_SM_OFF;
      break;

    case BL_SM_OFF:
      if (currentMode == MODE_MANUAL && joyPressed) {
        state = BL_SM_ON;
      }
      break;

    case BL_SM_ON:
      if (!(currentMode == MODE_MANUAL && joyPressed)) {
        state = BL_SM_OFF;
      }
      break;

    default:
      state = BL_SM_START;
      break;
  }

  switch (state) {
    case BL_SM_OFF:
      PORTB &= ~(1 << BUZZER_PIN);
      PORTC &= ~(1 << LASER_PIN);
      break;

    case BL_SM_ON:
      PORTB |= (1 << BUZZER_PIN);
      PORTC |= (1 << LASER_PIN);
      break;

    default:
      break;
  }

  return state;
}

// -----------------------------
// Scheduler
// -----------------------------

const unsigned short tasksNum = 5;
task tasks[5];

void TimerISR() {
  TimerFlag = 1;
}

void runTasks() {
  for (unsigned int i = 0; i < tasksNum; i++) {
    if (tasks[i].elapsedTime >= tasks[i].period) {
      tasks[i].state = tasks[i].TickFct(tasks[i].state);
      tasks[i].elapsedTime = 0;
    }

    tasks[i].elapsedTime += 10;
  }
}

// -----------------------------
// Main
// -----------------------------

int main(void) {
  IO_init();
  ADC_init();
  ServoTimer3_init();

  TWI_init();
  OLED_init();
  OLED_clear();

  xAngle = constrain_int(xAngle, X_SERVO_MIN, X_SERVO_MAX);
  yAngle = constrain_int(yAngle, Y_SERVO_MIN, Y_SERVO_MAX);

  writeXServo(xAngle);
  writeYServo(yAngle);

  unsigned char i = 0;

  tasks[i].state = MODE_SM_START;
  tasks[i].period = 50;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &TickFct_ModeSM;
  i++;

  tasks[i].state = JOY_SM_START;
  tasks[i].period = 50;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &TickFct_JoystickSM;
  i++;

  tasks[i].state = SERVO_SM_START;
  tasks[i].period = 40;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &TickFct_ServoSM;
  i++;

  tasks[i].state = LED_SM_START;
  tasks[i].period = 100;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &TickFct_LedSM;
  i++;

  tasks[i].state = BL_SM_START;
  tasks[i].period = 50;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &TickFct_BuzzerLaserSM;
  i++;

  TimerSet(10);
  TimerOn();

  enum Mode lastDisplayMode = MODE_POWER_OFF;
  unsigned char lastJoyPressed = 0;
  int lastXDisplay = -1;
  int lastYDisplay = -1;

  OLED_renderStatus();

  while (1) {
    while (!TimerFlag);

    TimerFlag = 0;

    runTasks();

    // Full OLED redraw only when mode changes
    if (currentMode != lastDisplayMode) {
      lastDisplayMode = currentMode;
      OLED_renderStatus();

      lastJoyPressed = joyPressed;
      lastXDisplay = xAngle;
      lastYDisplay = yAngle;
    }

    // Small OLED update only when trigger changes
    if (joyPressed != lastJoyPressed) {
      lastJoyPressed = joyPressed;
      OLED_updateTrigRow();
    }

    // Small OLED update only when X/Y changes by 3 degrees
    if (currentMode == MODE_MANUAL) {
      if ((xAngle >= lastXDisplay + 3) || (xAngle <= lastXDisplay - 3) ||
          (yAngle >= lastYDisplay + 3) || (yAngle <= lastYDisplay - 3)) {

        lastXDisplay = xAngle;
        lastYDisplay = yAngle;

        OLED_updateXYRow();
      }
    }
  }

  return 0;
}
