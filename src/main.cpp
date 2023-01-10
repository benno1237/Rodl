#include <Arduino.h>
#include <WS2812FX.h>
#include <EasyButton.h>

// Pindefinitions +++++++++++++++++
#define buttonPin 2  // main control button
#define mosfetDriverLow 5  // mosfet driver low beam
#define mosfetDriverHigh 6  // mosfet driver high beam
#define batteryIndicator 3  // battery indicator pin
#define potentiometerPin A7 // potentiometer pin
#define VoltageDividerPin A2 // Voltage divider pin

// LED Strip settings +++++++++++++
#define ledPin 4
#define ledNum 85
#define ledSpeed 200

uint32_t currentColor = 65535;  // default LED color: pink-ish
uint32_t currentColorHue = 1000;  // default LED color: pink-ish
uint8_t currentBrightness = 150;  // default LED brightness
uint8_t currentBrightnessOld = 150;  // default LED brightness
uint8_t currentEffect = 0;
const uint8_t effects[] = {0, 1, 2, 3, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 27, 33, 39, 42};

// Constants ++++++++++++++++++++++
#define LONG_PRESS_DURATION 3000  // 3000 milliseconds (3 secs)
#define SEMI_LONG_PRESS_DURATION 1000  // 1000 milliseconds (1 sec)
// #define DOUBLE_PRESS_TIMEOUT 300  // 300 milliseconds (0.3 secs)

// Variables +++++++++++++++++++++
volatile bool state = false;
volatile unsigned long disableBatteryIndicatorTimer = 0;

const uint8_t potThreshhold = 50; // Threshold to register potentiometer change
const uint8_t deadBand = 15; // deadband for potentiometer (effective deadband is 2*deadBand)
const uint8_t numReadings = 10;
int* readings = (int*)calloc(numReadings, sizeof(int));   // array with potentiometer readings to average and initialized to 0
uint8_t readIndex = 0;
int total = 0;
int average = 0;
int previousPotValue = 0;
int minPotValue = 0, maxPotValue = 0;
bool potentiometerValueChanged = false, changeB = false;
long hue;
unsigned long previousTime = millis();
int16_t potOffset = 0, potOffsetInit = 0;   // offset from max/min value of potentiometer and initial potvalue used for offset calculation
bool brightnessInit = false, colorInit = false;

EasyButton button(buttonPin);
WS2812FX ws2812fx = WS2812FX(ledNum, ledPin, NEO_RGB + NEO_KHZ800);

void initializePeripherals();
void resetPeripherals();
void buttonLongPressed();
// void buttonDoublePressed();
void buttonPressed();
void changeColor();
void disableBatteryIndicator();
void changeEffect();
int readPotentiometer();
int readVoltageDivider();

void setup() {
  Serial.begin(9600);

  initializePeripherals();
  resetPeripherals();
}

void initializePeripherals() {
  button.begin();
  button.onPressed(buttonPressed);
  // button.onSequence(2, DOUBLE_PRESS_TIMEOUT, buttonDoublePressed);
  button.onPressedFor(LONG_PRESS_DURATION, buttonLongPressed);

  ws2812fx.init();
  ws2812fx.setBrightness(currentBrightness);
  ws2812fx.setSpeed(ledSpeed);
  ws2812fx.start();
  ws2812fx.setColor(currentColor);

  pinMode(mosfetDriverLow, OUTPUT);
  pinMode(mosfetDriverHigh, OUTPUT);
  pinMode(batteryIndicator, OUTPUT);
  pinMode(potentiometerPin, INPUT);
  pinMode(VoltageDividerPin, INPUT);
}

void resetPeripherals() {
  ws2812fx.setBrightness(0);

  digitalWrite(mosfetDriverLow, LOW);
  digitalWrite(mosfetDriverHigh, LOW);
  
  disableBatteryIndicatorTimer = millis();
  potentiometerValueChanged = false;
}

void buttonPressed() {
  if (state == true && potentiometerValueChanged == false) {
    changeEffect();
    Serial.println("Pressed");
  }
}

// void buttonDoublePressed() {
// }

void buttonLongPressed() {
  // if turned off, turn on and exit function
  if (state == false) {
    state = true;
    ws2812fx.setColor(currentColor);
    digitalWrite(batteryIndicator, HIGH);
    return;
  }
  // if turned on and potentiometer value not changed, turn off
  if (potentiometerValueChanged == false) {
    state = false;
    resetPeripherals();
  }
  Serial.println("Long Pressed");
}

void changeEffect() {
  currentEffect++;
  // apply modulo to currentEffect to loop through effects
  if (currentEffect == sizeof(effects) / sizeof(int)) {
    currentEffect = 0;
  }
  ws2812fx.setMode(currentEffect);
}

void changeColor() {
  if (colorInit == false) {
    colorInit = true;
    potOffsetInit = readPotentiometer();    // initial potvalue used for offset calculation
  }
  hue = map(potOffsetInit - readPotentiometer(), 0, 1024, 0, 65600) + currentColorHue;  // hue is oldhue - newhue (hue difference) + the already set hue
  // "modulo" for hue
  if (hue > 65500) {
    hue -= 65500;
  }
  else if (hue < 0) {
    hue += 65500;
  }

  currentColorHue = hue;
  currentColor = ws2812fx.ColorHSV(currentColorHue);
  ws2812fx.setColor(currentColor);
  potOffsetInit = readPotentiometer();
}

void changeBrightness() {
  if (brightnessInit == false) {
    brightnessInit = true;
    potOffsetInit = readPotentiometer();    // initial potvalue used for offset calculation
    potOffset = (1024 - potOffsetInit)%512;   // offset from max/min value
  }
  // currentBrightness = map(readPotentiometer(), potOffsetInit - potOffset, potOffsetInit + potOffset, 0, 255);
  // ws2812fx.setBrightness(currentBrightness);
  // potOffsetInit = readPotentiometer();

  hue = map(potOffsetInit - readPotentiometer(), 0, 1024, 0, 255) + currentBrightness;  // hue is oldhue - newhue (hue difference) + the already set hue
  // "modulo" for hue
  if (hue > 255) {
    hue -= 255;
  }
  else if (hue < 0) {
    hue += 255;
  }

  currentBrightness = hue;
  ws2812fx.setBrightness(currentBrightness);
  potOffsetInit = readPotentiometer();
}

void disableBatteryIndicator() {
  if (millis() - disableBatteryIndicatorTimer < 80) {
    digitalWrite(batteryIndicator, LOW);
  }
  else if (millis() - disableBatteryIndicatorTimer < 160) {
    digitalWrite(batteryIndicator, HIGH);
  }
  else {
    digitalWrite(batteryIndicator, LOW);
    disableBatteryIndicatorTimer = 0;
  }
}

int readVoltageDivider() {
  return analogRead(VoltageDividerPin);
}

int readPotentiometer() {
  total -= readings[readIndex];
  readings[readIndex] = analogRead(potentiometerPin);
  total += readings[readIndex];
  readIndex++;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  average = total / numReadings;

  if ((average > previousPotValue + deadBand) || (average < previousPotValue - deadBand)) {
    previousPotValue = average;
  }

  if (previousPotValue < minPotValue) {
    minPotValue = previousPotValue;
  }
  else if (previousPotValue > maxPotValue) {
    maxPotValue = previousPotValue;
  }
  
  if (maxPotValue - minPotValue > potThreshhold) {
    potentiometerValueChanged = true;
  }

  return previousPotValue;
}

void changeLight(int potValue) {
  // 0-450, 450-540, 540-1024 (potentiometer reading)
  uint8_t pwm;
  if (potValue < 450) {
    pwm = map(potValue, 0, 450, 255, 0);
    analogWrite(mosfetDriverLow, pwm);
  }
  else if (potValue < 540) {
    digitalWrite(mosfetDriverLow, 0);
    digitalWrite(mosfetDriverHigh, 0);
  }
  else {
    pwm = map(potValue, 540, 1024, 0, 255);
    analogWrite(mosfetDriverHigh, pwm);
  }
}

void loop() {
  button.read();
  ws2812fx.service();

  if (disableBatteryIndicatorTimer != 0) {
    disableBatteryIndicator();
  }

  if (button.isPressed() == true && state == true) {
    if (button.wasPressed() == true) {
      minPotValue = readPotentiometer();
      maxPotValue = minPotValue;
      potentiometerValueChanged = false;
      previousTime = millis();
    }
    // if SEMI_LONG_PRESS_DURATION passed and potentiometer value not changed, switch to brightness mode
    if (millis() - previousTime > SEMI_LONG_PRESS_DURATION && changeB == false && potentiometerValueChanged == false) {
      changeB = true;
    }
    
    if (changeB == true) {
      changeBrightness();
    }
    else {
      changeColor();
    }
  }
  else if(button.isPressed() == false && state == true) {
    changeB = false;
    brightnessInit = false;
    colorInit = false;
    // Serial.println("Released");
    changeLight(readPotentiometer());
  }
  // Serial.println(readPotentiometer());
}