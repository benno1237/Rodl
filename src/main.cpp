#include <Arduino.h>
#include <WS2812FX.h>
#include <EasyButton.h>

// Pindefinitions +++++++++++++++++
#define buttonPin 2  // main control button
#define mosfetDriverLow 5  // mosfet driver low beam
#define mosfetDriverHigh 6  // mosfet driver high beam
#define batteryIndicator 3  // battery indicator pin
#define potentiometerPin A7 // potentiometer pin

// LED Strip settings +++++++++++++
#define ledPin 4
#define ledNum 85
#define ledBrightness 150
#define ledSpeed 200

uint32_t currentColor = 65535;  // default LED color: pink-ish
int currentEffect = 0;
const int effects[] = {0, 1, 2, 3, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 27, 33, 39, 42};

// Constants ++++++++++++++++++++++
#define LONG_PRESS_DURATION 3000  // 3000 milliseconds (3 secs)
// #define DOUBLE_PRESS_TIMEOUT 300  // 300 milliseconds (0.3 secs)

// Variables +++++++++++++++++++++
volatile bool state = false;
volatile unsigned long disableBatteryIndicatorTimer = 0;

const uint8_t potThreshhold = 50; // Threshold to register potentiometer change
const uint8_t deadBand = 5; // deadband for potentiometer (effective deadband is 2*deadBand)
const uint8_t numReadings = 10;
int readings[numReadings];
uint8_t readIndex = 0;
int total = 0;
int average = 0;
int previousPotValue = 0;
int minPotValue = 0, maxPotValue = 0;
bool potentiometerValueChanged = false;

EasyButton button(buttonPin);
WS2812FX ws2812fx = WS2812FX(ledNum, ledPin, NEO_RGB + NEO_KHZ800);

void initializePeripherals();
void resetPeripherals();
void buttonLongPressed();
// void buttonDoublePressed();
void buttonPressed();
void changeEffect();
int readPotentiometer();

void setup() {
  Serial.begin(9600);

  initializePeripherals();
  resetPeripherals();

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void initializePeripherals() {
  button.begin();
  button.onPressed(buttonPressed);
  // button.onSequence(2, DOUBLE_PRESS_TIMEOUT, buttonDoublePressed);
  button.onPressedFor(LONG_PRESS_DURATION, buttonLongPressed);

  ws2812fx.init();
  ws2812fx.setBrightness(ledBrightness);
  ws2812fx.setSpeed(ledSpeed);
  ws2812fx.start();
  ws2812fx.setColor(currentColor);

  pinMode(mosfetDriverLow, OUTPUT);
  pinMode(mosfetDriverHigh, OUTPUT);
  pinMode(batteryIndicator, OUTPUT);
  pinMode(potentiometerPin, INPUT);
}

void resetPeripherals() {
  ws2812fx.setColor(0);

  digitalWrite(mosfetDriverLow, LOW);
  digitalWrite(mosfetDriverHigh, LOW);
  
  disableBatteryIndicatorTimer = millis();
  potentiometerValueChanged = false;
}

void buttonPressed() {
  if (state == true) {
    changeEffect();
  }
}

// void buttonDoublePressed() {
// }

void buttonLongPressed() {
  if (state == false) {
    state = true;
    ws2812fx.setColor(65500);
    digitalWrite(batteryIndicator, HIGH);
    return;
  }

  if (potentiometerValueChanged == false) {
    state = false;
    resetPeripherals();
  }
}

void changeEffect() {
  currentEffect++;
  if (currentEffect == sizeof(effects) / sizeof(int)) {
    currentEffect = 0;
  }
  ws2812fx.setMode(currentEffect);
}

void changeColor() {
  double hue = map(readPotentiometer(), 0, 1024, 0, 65600);
  currentColor = ws2812fx.ColorHSV(hue);
  ws2812fx.setColor(currentColor);
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

int readPotentiometer() {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(potentiometerPin);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

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
    }

    changeColor();
  }
}