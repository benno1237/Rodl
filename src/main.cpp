/*
*/

#include <Arduino.h>
#include <ezButton.h>
#include <WS2812FX.h>


// Pindefinitions +++++++++++++++++

#define Button 2  // Button
#define Potentio A7  // Potentiometer
#define Mos1 5   // Mosfet1
#define Mos2 6   // Mosfet29

#define ledPin 4
#define numLed 60

ezButton button(Button);
WS2812FX ws2812fx = WS2812FX(numLed, ledPin, NEO_RGB + NEO_KHZ800);

const int LONG_PRESS_TIME = 3000; // 3000 milliseconds (3 secs)
bool released = true, state = false;
unsigned long lastPressed = millis();
int potValue, pwm;
double volt, shift;

void setup() {
  Serial.begin(9600);
  Serial.println();

  button.setDebounceTime(50);

  ws2812fx.init();
  ws2812fx.setBrightness(150);
  ws2812fx.setSpeed(200);
  ws2812fx.start();

  digitalWrite(Mos2, 0);        // start Headlight in off state
  digitalWrite(Mos1, 0);
}


void loop() {
  button.loop();
  ws2812fx.service();

  if (button.isPressed()) {
    lastPressed = millis();
  }

  // turn System on and off
  if (button.getState() == LOW && released && millis() - lastPressed > LONG_PRESS_TIME) {   
    released = false;
    state = !state; 
  }

  if (button.isReleased()) {
    released = true;
    if (millis() - lastPressed < LONG_PRESS_TIME && state) {
      // change lighting effect
      Serial.println("short");
    }
  }
  // 0-450, 450-540, 540-1024 (potentiometer reading)
  if (state) {
    // volt = analogRead(A6);
    // shift = 52224 * 3.55 / volt;
    Serial.println(analogRead(Potentio)); 
    potValue = analogRead(Potentio);        // Potentiometer Reading (0 - 1024)
    // pwm = map(potValue, 0, 1024, 0, 255);  	// Potentiometer Reading mapped to pwm
    // analogWrite(A6, potValue);
    // Serial.println(analogRead(5));
    if (potValue < 450){  
      // pwm = map(potValue, 450, 0, shift * 9, shift * 12);
      pwm = map(potValue, 0, 450, 255, 0);
      analogWrite(Mos1, pwm);
    }
    else if(potValue < 540){    // off
      digitalWrite(Mos1, 0);
      digitalWrite(Mos2, 0);
    }
    else{
      pwm = map(potValue, 540, 1024, 0, 255);
      analogWrite(Mos2, pwm);
    }

  }

  // Serial.println(digitalRead(2));
}

void hsvToHex(double hue, uint8_t& red, uint8_t& green, uint8_t& blue) {
  double r, g, b;

  auto i = static_cast<int>(hue * 6);
  auto f = hue * 6 - i;
  auto p = 255 * (1 - 255);
  auto q = 255 * (1 - f * 255);
  auto t = 255 * (1 - (1 - f) * 255);

  switch (i % 6) {
    case 0:
      r = 255, g = t, b = p;
      break;
    case 1:
      r = q, g = 255, b = p;
      break;
    case 2:
      r = p, g = 255, b = t;
      break;
    case 3:
      r = p, g = q, b = 255;
      break;
    case 4:
      r = t, g = p, b = 255;
      break;
    case 5:
      r = 255, g = p, b = q;
      break;
  }
  red = static_cast<byte>(r * 255);
  green = static_cast<byte>(g * 255);
  blue = static_cast<byte>(b * 255);

  long RGB = ((long)red << 16L) | ((long)green << 8L) | (long)blue;
}