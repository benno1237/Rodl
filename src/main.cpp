/*
*/

#include <Arduino.h>
#include <ezButton.h>
#include <WS2812FX.h>
#include <math.h>

// Pindefinitions +++++++++++++++++

#define Button 2  // Button
#define POT_PIN A7  // Potentiometer
#define Mos1 5   // Mosfet1
#define Mos2 6   // Mosfet29

#define ledPin 4
#define numLed 85

ezButton button(Button);
WS2812FX ws2812fx = WS2812FX(numLed, ledPin, NEO_RGB + NEO_KHZ800);

const long LONG_PRESS_TIME = 3000; // 3000 milliseconds (3 secs)
bool released = true, state = true;
unsigned long lastPressed = millis();
uint16_t potValue, prevPotValue, mem;
uint8_t pwm;
uint32_t currentColor = 65535; // default color is pinkish

int currentEffect;
const int effects[] = {0, 1, 2, 3, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 27, 33, 39, 42};

void changeEffect();
void changeColor();
uint32_t ColorHSV(uint16_t hue, uint8_t sat, uint8_t val);

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

  mem = analogRead(POT_PIN);
}


void loop() {
  delay(20);
  button.loop();
  ws2812fx.service();

  if (button.isPressed() && released) {
    prevPotValue = analogRead(POT_PIN);
    lastPressed = millis();
    released = false;
    Serial.print("pressed");
    Serial.println(state);
  }

  // turn System on and off
  if (!released && millis() - lastPressed > LONG_PRESS_TIME) {
    if (state) {   // if turned on, turn light off
      ws2812fx.setColor(0, 0, 0);
      ws2812fx.setMode(0);
    }
    else {
      ws2812fx.setColor(currentColor);
      ws2812fx.setMode(currentEffect);
    }
    Serial.println(millis() - lastPressed);
    state = !state; 
    lastPressed = millis();
    prevPotValue = analogRead(POT_PIN);
  }

  if (button.isReleased()) {
    released = true;
    
    if (millis() - lastPressed < LONG_PRESS_TIME && state) {
      // Short press
      changeEffect();
      ws2812fx.setColor(currentColor);    // set color to current color (just to make sure)
      Serial.println("change");
    }
  }

  // 0-450, 450-540, 540-1024 (potentiometer reading)
  if (state) {
    potValue = analogRead(POT_PIN);
    if (!released) {
      if (abs(potValue - prevPotValue) > 40) {
        Serial.println(abs(potValue - prevPotValue));
        changeColor();
        lastPressed = millis();
        // released = false;
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
  mem = potValue;
  }
  Serial.println(analogRead(POT_PIN));
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
  double hue = map(potValue, 0, 1024, 0, 65600);

  currentColor = ws2812fx.ColorHSV(hue);
  // Serial.println(hue);
  // Serial.println(currentColor);
  ws2812fx.setColor(currentColor);
}

uint32_t ColorHSV(uint16_t hue, uint8_t sat, uint8_t val) {

  uint8_t r, g, b;

  // Remap 0-65535 to 0-1529. Pure red is CENTERED on the 64K rollover;
  // 0 is not the start of pure red, but the midpoint...a few values above
  // zero and a few below 65536 all yield pure red (similarly, 32768 is the
  // midpoint, not start, of pure cyan). The 8-bit RGB hexcone (256 values
  // each for red, green, blue) really only allows for 1530 distinct hues
  // (not 1536, more on that below), but the full unsigned 16-bit type was
  // chosen for hue so that one's code can easily handle a contiguous color
  // wheel by allowing hue to roll over in either direction.
  hue = (hue * 1530L + 32768) / 65536;
  // Because red is centered on the rollover point (the +32768 above,
  // essentially a fixed-point +0.5), the above actually yields 0 to 1530,
  // where 0 and 1530 would yield the same thing. Rather than apply a
  // costly modulo operator, 1530 is handled as a special case below.

  // So you'd think that the color "hexcone" (the thing that ramps from
  // pure red, to pure yellow, to pure green and so forth back to red,
  // yielding six slices), and with each color component having 256
  // possible values (0-255), might have 1536 possible items (6*256),
  // but in reality there's 1530. This is because the last element in
  // each 256-element slice is equal to the first element of the next
  // slice, and keeping those in there this would create small
  // discontinuities in the color wheel. So the last element of each
  // slice is dropped...we regard only elements 0-254, with item 255
  // being picked up as element 0 of the next slice. Like this:
  // Red to not-quite-pure-yellow is:        255,   0, 0 to 255, 254,   0
  // Pure yellow to not-quite-pure-green is: 255, 255, 0 to   1, 255,   0
  // Pure green to not-quite-pure-cyan is:     0, 255, 0 to   0, 255, 254
  // and so forth. Hence, 1530 distinct hues (0 to 1529), and hence why
  // the constants below are not the multiples of 256 you might expect.

  // Convert hue to R,G,B (nested ifs faster than divide+mod+switch):
  if (hue < 510) { // Red to Green-1
    b = 0;
    if (hue < 255) { //   Red to Yellow-1
      r = 255;
      g = hue;       //     g = 0 to 254
    } else {         //   Yellow to Green-1
      r = 510 - hue; //     r = 255 to 1
      g = 255;
    }
  } else if (hue < 1020) { // Green to Blue-1
    r = 0;
    if (hue < 765) { //   Green to Cyan-1
      g = 255;
      b = hue - 510;  //     b = 0 to 254
    } else {          //   Cyan to Blue-1
      g = 1020 - hue; //     g = 255 to 1
      b = 255;
    }
  } else if (hue < 1530) { // Blue to Red-1
    g = 0;
    if (hue < 1275) { //   Blue to Magenta-1
      r = hue - 1020; //     r = 0 to 254
      b = 255;
    } else { //   Magenta to Red-1
      r = 255;
      b = 1530 - hue; //     b = 255 to 1
    }
  } else { // Last 0.5 Red (quicker than % operator)
    r = 255;
    g = b = 0;
  }

  // Apply saturation and value to R,G,B, pack into 32-bit result:
  uint32_t v1 = 1 + val;  // 1 to 256; allows >>8 instead of /255
  uint16_t s1 = 1 + sat;  // 1 to 256; same reason
  uint8_t s2 = 255 - sat; // 255 to 0
  return ((((((r * s1) >> 8) + s2) * v1) & 0xff00) << 8) |
         (((((g * s1) >> 8) + s2) * v1) & 0xff00) |
         (((((b * s1) >> 8) + s2) * v1) >> 8);
}