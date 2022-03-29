/*
*/

#include <Arduino.h>
#include <ezButton.h>
#include <WS2812FX.h>

// Pindefinitions +++++++++++++++++

#define Button 2  // Button
#define POT_PIN A7  // Potentiometer
#define Mos1 5   // Mosfet1
#define Mos2 6   // Mosfet29

#define ledPin 4
#define numLed 60

ezButton button(Button);
WS2812FX ws2812fx = WS2812FX(numLed, ledPin, NEO_RGB + NEO_KHZ800);

const uint8_t LONG_PRESS_TIME = 3000; // 3000 milliseconds (3 secs)
bool released = true, state = false;
unsigned long lastPressed = millis();
uint8_t potValue, prevPotValue, pwm;

int currentEffect;
const int effects[] = {0, 1, 2, 3, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 27, 33, 39, 42};
uint8_t red, green, blue;

void changeEffect();
void changeColor();
void hsvToHex(double hue, uint8_t& red, uint8_t& green, uint8_t& blue);

void setup() {
  Serial.begin(9600);
  Serial.println();

  button.setDebounceTime(50);

  ws2812fx.init();
  ws2812fx.setBrightness(150);
  ws2812fx.setSpeed(200);
  ws2812fx.setColor(0, 0, 0);
  ws2812fx.start();

  digitalWrite(Mos2, 0);        // start Headlight in off state
  digitalWrite(Mos1, 0);
}


void loop() {
  button.loop();
  ws2812fx.service();

  if (button.isPressed()) {
    prevPotValue = analogRead(POT_PIN);
    lastPressed = millis();
  }

  // turn System on and off
  if (button.getState() == LOW && released && millis() - lastPressed > LONG_PRESS_TIME) {
    released = false;   
    state = !state; 

    if (state) {
      changeColor();
    }
    else {
      // ws2812fx.strip_off();
      ws2812fx.setColor(0, 0, 0);
      ws2812fx.setMode(0);
    }
  }

  if (button.isReleased()) {
    released = true;
    
    if (millis() - lastPressed < LONG_PRESS_TIME && state) {
      // Short press
      changeEffect();
    }
  }

  // 0-450, 450-540, 540-1024 (potentiometer reading)
  if (state) {
    potValue = analogRead(POT_PIN);

    if (button.isPressed()) {
      if (!(prevPotValue - 20 < potValue < prevPotValue + 20) || !released) {
        changeColor();
        released = false;
      }
    }

    else {      
      if (potValue < 450) {
        pwm = map(potValue, 0, 450, 255, 0);
        analogWrite(Mos1, pwm);
      }
      else if (potValue < 540) {
        digitalWrite(Mos1, 0);
        digitalWrite(Mos2, 0);
      }
      else {
        pwm = map(potValue, 540, 1024, 255, 0);
        analogWrite(Mos2, pwm);
      }
    }
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
  potValue = analogRead(POT_PIN);
  double hue = map(potValue, 0, 1024, 0, 1000) / 1000.0;
  hsvToHex(hue, red, green, blue);
  ws2812fx.setColor(red, green, blue);
}

void hsvToHex(double hue, uint8_t& red, uint8_t& green, uint8_t& blue) {
  double s = 1;
  double v = 1;
  double r, g, b;

  auto i = static_cast<int>(hue * 6);
  auto f = hue * 6 - i;
  auto p = v * (1 - s);
  auto q = v * (1 - f * s);
  auto t = v * (1 - (1 - f) * s);

  switch (i % 6) {
    case 0:
      r = v, g = t, b = p;
      break;
    case 1:
      r = q, g = v, b = p;
      break;
    case 2:
      r = p, g = v, b = t;
      break;
    case 3:
      r = p, g = q, b = v;
      break;
    case 4:
      r = t, g = p, b = v;
      break;
    case 5:
      r = v, g = p, b = q;
      break;
  }
  red = static_cast<uint8_t>(r * 255);
  green = static_cast<uint8_t>(g * 255);
  blue = static_cast<uint8_t>(b * 255);

  // Serial.print(red); Serial.print(" "); Serial.print(green); Serial.print(" "); Serial.println(blue);
}