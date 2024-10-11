#include <Arduino.h>
#include <WS2812FX.h>
#include <EasyButton.h>
// #include <BluetoothSerial.h>
// #include "HTU21D.h"

// BluetoothSerial SerialBT;

// HTU21D myHumidity;
// const int Interval = 15000;   // Interval for readouts (in ms)
// unsigned long prevTime = -Interval;

// float humd, temp;

// Pindefinitions +++++++++++++++++
#define buttonPin 16  // main control button
#define mosfetDriverLow 12  // mosfet driver low beam (analogWrite)
#define mosfetDriverHigh 11  // mosfet driver high beam (analogWrite)
// #define batteryIndicator 6  // battery indicator pin 
#define potentiometerPin 15 // potentiometer pin
#define VoltageDividerPin 6 // Voltage divider pin
#define controlBoxPower 7
#define LedPower 13

// LED Strip settings +++++++++++++
#define ledPin 10
#define ledNum 88 
#define ledSpeed 500

uint32_t currentColor = 65535 ;  // default LED color: pink-ish
uint32_t currentColorHue = 1000;  // default LED color: pink-ish
uint8_t currentBrightness = 150;  // default LED brightness
uint8_t currentEffect = 0;
const uint8_t effects[] = {12, 11, 2, 7, 0, 1, 3, 5, 8, 13, 17, 18, 20, 27, 33, 39};
// const uint16_t effectSpeed[] = {};
// 17 slower, 27 color and speed = speed dependent

// Constants ++++++++++++++++++++++
#define LONG_PRESS_DURATION 1500  // 3000 milliseconds (3 secs)
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
bool colorInit = false;

EasyButton button(buttonPin);
WS2812FX ws2812fx = WS2812FX(ledNum, ledPin, NEO_RGB + NEO_KHZ800);

void initializePeripherals();
void resetPeripherals();
void buttonLongPressed();
// void buttonDoublePressed();
void buttonPressed();
void changeColor();
// void disableBatteryIndicator();
void changeEffect();
int readPotentiometer();
int readVoltageDivider();

void setup() {
  Serial.begin(115200);

  // set adc resolution to 10 bit
  analogReadResolution(12);

  initializePeripherals();
  resetPeripherals();
  // myHumidity.begin();
}

void initializePeripherals() {
  button.begin();
  button.onPressed(buttonPressed);
  // button.onSequence(2, DOUBLE_PRESS_TIMEOUT, buttonDoublePressed);
  button.onPressedFor(LONG_PRESS_DURATION, buttonLongPressed);

  ws2812fx.init();
  ws2812fx.setBrightness(currentBrightness);
  ws2812fx.setSpeed(ledSpeed);
  ws2812fx.setColor(currentColor);
  ws2812fx.setMode(effects[currentEffect]);
  ws2812fx.start();

  pinMode(mosfetDriverLow, OUTPUT);
  pinMode(mosfetDriverHigh, OUTPUT);
  // pinMode(batteryIndicator, OUTPUT);
  pinMode(potentiometerPin, INPUT);
  pinMode(VoltageDividerPin, INPUT);
}

void resetPeripherals() {
  ws2812fx.setBrightness(0);

  analogWrite(mosfetDriverHigh, 0);
  analogWrite(mosfetDriverLow, 0);
  // analogWrite(controlBoxPower, 0);
  
  disableBatteryIndicatorTimer = millis();
  potentiometerValueChanged = false;
}

void buttonPressed() {
//   if (state == true && potentiometerValueChanged == false) {
  if (state) {
    changeEffect();
    Serial.println("Pressed");
  }
}

// void buttonDoublePressed() {
// }

void buttonLongPressed() {
  Serial.println("Long Pressed");
  // if turned off, turn on and exit function
  if (state == false) {
    state = true;
    ws2812fx.setBrightness(currentBrightness);
    ws2812fx.setColor(currentColor);
    // digitalWrite(batteryIndicator, HIGH);
    // return;
  }
  // if turned on
  else {
    state = false;
    resetPeripherals();
  }
}

void changeEffect() {
  currentEffect++;
  // apply modulo to currentEffect to loop through effects
  if (currentEffect == sizeof(effects) / sizeof(uint8_t)) {
    currentEffect = 0;
  }
  ws2812fx.setMode(effects[currentEffect]);
  Serial.println(effects[currentEffect]);
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

// void disableBatteryIndicator() {
//   if (millis() - disableBatteryIndicatorTimer < 80) {
//     digitalWrite(batteryIndicator, LOW);
//   }
//   else if (millis() - disableBatteryIndicatorTimer < 160) {
//     digitalWrite(batteryIndicator, HIGH);
//   }
//   else {
//     digitalWrite(batteryIndicator, LOW);
//     disableBatteryIndicatorTimer = 0;
//   }
// }

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
  // 0-1800, 450-2160, 540-4096 (potentiometer reading)
  int multiplier = 4;
  uint8_t pwm;
  if (potValue < 420 * multiplier) {
    pwm = map(potValue, 0, 420 * multiplier, 255, 0);
    analogWrite(mosfetDriverLow, pwm);
  }
  else if (potValue < 520 * multiplier) {
    analogWrite(mosfetDriverLow, 0);
    analogWrite(mosfetDriverHigh, 0);
  }
  else {
    pwm = map(potValue, 520 * multiplier, 1024 * multiplier, 0, 255);
    analogWrite(mosfetDriverHigh, pwm);
  }
  // Serial.println(potValue);
}

void loop() {
  button.read();
  ws2812fx.service();

  // if (disableBatteryIndicatorTimer != 0) {
  //   disableBatteryIndicator();
  // }

  // if (button.isPressed() == true && state == true) {
  //   if (button.wasPressed() == true) {
  //     minPotValue = readPotentiometer();
  //     maxPotValue = minPotValue;
  //     potentiometerValueChanged = false;
  //     previousTime = millis();
  //   }
  //   // if SEMI_LONG_PRESS_DURATION passed and potentiometer value not changed, switch to brightness mode
  //   if (millis() - previousTime > SEMI_LONG_PRESS_DURATION && potentiometerValueChanged == false) {
  //     changeB = true;
  //   }
    
  //   // if (changeB == true) {
  //   //   changeBrightness();
  //   // }
  //   else {
  //   changeColor();
  //   }
  // }
  if(state == true) {
  //   // changeB = false;
  //   // brightnessInit = false;
  //   // colorInit = false;
  //   // Serial.println("Released");
    changeLight(readPotentiometer());
  }
  // Serial.println(readPotentiometer());
  // if (millis() - prevTime > Interval){
  //   prevTime = millis();
  //   if (humd < myHumidity.readHumidity()) {
  //     humd = myHumidity.readHumidity();
  //   }
  //   if (temp < myHumidity.readTemperature()) {
  //     temp = myHumidity.readTemperature();
  //   }
  //   Serial.println(temp, 1);
  //   Serial.println(humd, 1);
  // }
}