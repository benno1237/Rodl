#include <Arduino.h>
#include <ezButton.h>


// Pindefinitions +++++++++++++++++

#define Button 2  // Button
#define Potentio A7  // Potentiometer
#define Mos1 5   // Mosfet1
#define Mos2 6   // Mosfet29

ezButton button(Button);

const int LONG_PRESS_TIME = 3000; // 3000 milliseconds (3 secs)
bool released = true, state = false;
unsigned long lastPressed = millis();
int potValue, pwm;
double volt, shift;

void setup() {
  Serial.begin(9600);
  Serial.println();
  button.setDebounceTime(50);
  digitalWrite(Mos2, 0);
  digitalWrite(Mos1, 0);
}


void loop() {
  button.loop();

  if (button.isPressed()) {
    lastPressed = millis();
  }

  if (button.getState() == LOW && released && millis() - lastPressed > LONG_PRESS_TIME) {
    released = false;
    state = !state;
  }

  if (button.isReleased()) {
    released = true;
    if (millis() - lastPressed < LONG_PRESS_TIME && state) {
      Serial.println("short");
    }
  }
// 3-450, 450-540, 540-1023 
  if (state) {
    // volt = analogRead(A6);
    // shift = 52224 * 3.55 / volt;
    // Serial.println(analogRead(Potentio)); 
    potValue = analogRead(Potentio);
    // pwm = map(potValue, 0, 1024, 0, 255);
    // analogWrite(A6, potValue);
    // Serial.println(analogRead(5));
    if (potValue < 450){
      // pwm = map(potValue, 450, 0, shift * 9, shift * 12);
      pwm = map(potValue, 0, 450, 255, 0);
      analogWrite(Mos1, pwm);
    }
    else if(potValue < 540){
      analogWrite(Mos1, 0);
      analogWrite(Mos2, 0);
    }
    else{
      pwm = map(potValue, 540, 1024, 150, 255);
      analogWrite(Mos2, pwm);
    }

  }

  
  // bool newState = digitalRead(Button);
  // Serial.println(state);

  // Serial.println(digitalRead(2));
}
