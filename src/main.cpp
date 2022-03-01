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
  digitalWrite(Mos2, 0);        // start Headlight in off state
  digitalWrite(Mos1, 0);
}


void loop() {
  button.loop();

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
      analogWrite(Mos1, 0);
      analogWrite(Mos2, 0);
    }
    else{
      pwm = map(potValue, 540, 1024, 150, 255);
      analogWrite(Mos2, pwm);
    }

  }

  // Serial.println(digitalRead(2));
}
