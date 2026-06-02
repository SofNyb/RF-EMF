#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// NeoPixel
#define NEO_PIN    D6
#define NEO_COUNT  7
Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

// RF
const int PIN_X = A0;
const int PIN_Y = A1;
const int PIN_Z = A2;

// Hardware offset calibration
const float OFFSET_X =  2.0f;
const float OFFSET_Y = -2.0f;
const float OFFSET_Z =  0.0f;

// dBm = (INTERCEPT - volts) / SLOPE
const float SLOPE     = 0.0228f;
const float INTERCEPT = 0.4f;

// Exponential Moving Average
const float EMA_ALPHA = 0.2f;

// Slow rolling EMA around 100 samples
const float SLOW_EMA_ALPHA =  0.01f;
float slowEma =               -55.0f;
bool slowEmaInitialized =     false;
int slowEmaSampleCount =      0;

// Buzzer
const int BUZZER_PIN    = D9;
const float BUZZ_THRESH = -40.0f;
unsigned long lastBuzz  = 0;

// Fast EMA values per axis
float emaX = -55.0f, emaY = -55.0f, emaZ = -55.0f;

// Previous EMA values per axis
float lastX = -55.0f, lastY = -55.0f, lastZ = -55.0f;

// Exposure limits in dBm
const float EXP_LOW    = -50.0f;
const float EXP_HIGH   = -30.0f;
const float EXP_THRESH = -55.0f;

// Buttons
const int BTN_SCREEN   = PC13;
const int SWITCH_PIN   = D5;
int screen             = 0;
bool powered           = true;

// Detection of a button press
bool lastScreenState = HIGH;

// Forward declarations
void updateNeoPixel(float exposure);
void updateBuzzer(float exposure);
void printBar(float exposure);
void setBaseline();
float readAxis(int pin, float &ema, float &last, float offset);
float combineAxes(float dX, float dY, float dZ);

float readAxis(int pin, float &ema, float &last, float offset) {
  int numSamples = (ema < -45.0f) ? 128 : 64;
  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }

  float raw             = sum / (float)numSamples;
  float volts           = raw * 3.3f / 4095.0f;
  float dBm             = (INTERCEPT - volts) / SLOPE;
  if (dBm < -70.0f) dBm = -70.0f;

  // Fast EMA filter
  ema = EMA_ALPHA * dBm + (1.0f - EMA_ALPHA) * ema;

  //Rejection if the new EMA differs by more than 20 dBm from the last accepted value, return previous
  if (abs(ema - last) > 20.0f) {
    return last + offset;
  }

  last = ema;
  return ema + offset;
}

// Combine readings from active axes into a single value in dBm
float combineAxes(float dX, float dY, float dZ) {
  float linX = pow(10.0f, dX / 10.0f);
  float linY = pow(10.0f, dY / 10.0f);
  float linZ = pow(10.0f, dZ / 10.0f);

  return 10.0f * log10(linX + linY + linZ);
}

// Update slow rolling EMA
void updateSlowEma(float exposure) {
  if (!slowEmaInitialized) {
    slowEma = exposure;
    slowEmaInitialized = true;
  } else {
    slowEma = SLOW_EMA_ALPHA * exposure + (1.0f - SLOW_EMA_ALPHA) * slowEma;
  }
  if (slowEmaSampleCount < 100) slowEmaSampleCount++;
}

// Power off -- resetting the slow EMA
void powerOff() {
  powered = false;
  slowEmaInitialized = false;
  slowEmaSampleCount = 0;

  strip.clear();
  strip.show();

  lcd.clear();
  lcd.noBacklight();

  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

// Warms up the EMA filters by running 10 samples with 100ms spacing (~1 second total)
void setBaseline() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Warming up...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");

  for (int i = 0; i < 10; i++) {
    if (digitalRead(SWITCH_PIN) == HIGH) {
      powerOff();
      return;
    }
    readAxis(PIN_X, emaX, lastX, OFFSET_X);
    readAxis(PIN_Y, emaY, lastY, OFFSET_Y);
    readAxis(PIN_Z, emaZ, lastZ, OFFSET_Z);
    delay(100);
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready!");

  for (int i = 0; i < 15; i++) {
    if (digitalRead(SWITCH_PIN) == HIGH) {
      powerOff();
      return;
    }
    delay(100);
  }
  lcd.clear();
}

// NeoPixel
// Low exposure = green LEDs lit from the right
// high exposure = More of the strip is lighting up
void updateNeoPixel(float exposure) {
  float t = (exposure - EXP_LOW) / (EXP_HIGH - EXP_LOW);
  t = max(0.0f, min(1.0f, t));

  // Number of LEDs
  int lit = (int)(t * NEO_COUNT);

  for (int i = 0; i < NEO_COUNT; i++) {
    if (i < (NEO_COUNT - lit)) {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    } else {
      float pos = (float)(NEO_COUNT - 1 - i) / (NEO_COUNT - 1);
      uint8_t r = 0, g = 0, b = 0;
      if (pos < 0.5f) {
        r = (uint8_t)(pos * 2.0f * 255);
        g = 255;
      } else {
        r = 255;
        g = (uint8_t)((1.0f - (pos - 0.5f) * 2.0f) * 255);
      }
      strip.setPixelColor(i, strip.Color(r, g, b));
    }
  }
  strip.show();
}

// Buzzer
void updateBuzzer(float exposure) {
  if (exposure < BUZZ_THRESH) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  float t = (exposure - BUZZ_THRESH) / (EXP_HIGH - BUZZ_THRESH);
  t = max(0.0f, min(1.0f, t));

  unsigned long interval = (unsigned long)(800 - t * 700);

  if (millis() - lastBuzz > interval) {
    lastBuzz = millis();
    tone(BUZZER_PIN, 1000, 50); // 1kHz tone for 50ms
  }
}

// Initializes serial, pins, I2C, LCD, NeoPixel
void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Use 12-bit ADC resolution (0-4095)

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BTN_SCREEN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // I2C pins for LCD (STM32 alternate pin mapping)
  Wire.setSDA(D14);
  Wire.setSCL(D15);
  Wire.begin();

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("RF Exposure");

  strip.begin();
  strip.setBrightness(30);
  strip.show();

  delay(1000);

  if (digitalRead(SWITCH_PIN) == LOW) {
    setBaseline();
  } else {
    powered = false;
    lcd.noBacklight();
    lcd.clear();
  }
}

// Handles power toggle, screen switching, reading sensors and updating all outputs
void loop() {
  bool switchState = digitalRead(SWITCH_PIN);

  // Power on: switch flipped to ON while device was off - reset state and recalibrate
  if (switchState == LOW && !powered) {
    powered = true;
    screen = 0;
    emaX = -55.0f; emaY = -55.0f; emaZ = -55.0f;
    lastX = -55.0f; lastY = -55.0f; lastZ = -55.0f;
    slowEma = -55.0f;
    slowEmaInitialized = false;
    slowEmaSampleCount = 0;
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("RF Exposure");
    delay(1000);
    setBaseline();
  }

  // Power off: switch flipped to OFF while device was on
  if (switchState == HIGH && powered) {
    powerOff();
  }

  // Do nothing while powered off
  if (!powered) {
    delay(100);
    return;
  }

  // Change screen
  bool screenState = digitalRead(BTN_SCREEN);
  if (screenState == LOW && lastScreenState == HIGH) {
    screen = (screen + 1) % 3;
    lcd.clear();
  }
  lastScreenState = screenState;

  // Read active axes with the offset applied
  float dX = readAxis(PIN_X, emaX, lastX, OFFSET_X);
  float dY = readAxis(PIN_Y, emaY, lastY, OFFSET_Y);
  float dZ = readAxis(PIN_Z, emaZ, lastZ, OFFSET_Z);

  float exposure = combineAxes(dX, dY, dZ);

  // Update slow rolling EMA
  updateSlowEma(exposure);

  updateNeoPixel(exposure);
  updateBuzzer(exposure);

  // Screen 0: Live combined exposure 
  if (screen == 0) {
      lcd.setCursor(0, 0);
      lcd.print("RF Exposure:");
      lcd.setCursor(0, 1);
      lcd.print(exposure, 1);
      lcd.print(" dBm   ");

  // Screen 1: Slow rolling EMA (100 samples) with warmup
  } else if (screen == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Avg(100):");
    if (slowEmaSampleCount < 100) {
      lcd.print(slowEmaSampleCount);
      lcd.print("/100  ");
    } else {
      lcd.print("ready ");
    }
    lcd.setCursor(0, 1);
    lcd.print(slowEma, 1);
    lcd.print(" dBm   ");

  // Screen 2: Individual axis readings
  } else {
    lcd.setCursor(0, 0);
    lcd.print("X:"); lcd.print(dX, 1);
    lcd.print(" Y:"); lcd.print(dY, 1);
    lcd.print("  ");
    lcd.setCursor(0, 1);
    lcd.print("Z:"); lcd.print(dZ, 1);
    lcd.print(" dBm  ");
} 
  // Serial output for loggin and debugging via USB
  Serial.print("X: ");      Serial.print(dX, 2);       Serial.print(" dBm");
  Serial.print(" Y: ");     Serial.print(dY, 2);       Serial.print(" dBm");
  Serial.print(" Z: ");     Serial.print(dZ, 2);       Serial.print(" dBm");
  Serial.print(" | Exp: "); Serial.print(exposure, 2); Serial.print(" dBm");
  Serial.print(" | Avg: "); Serial.print(slowEma, 2);  Serial.println(" dBm");
  
  delay(200); // 4-5 updates per second
}