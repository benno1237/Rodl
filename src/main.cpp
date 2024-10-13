#include <Arduino.h>
#include <WS2812FX.h>
#include <EasyButton.h>
#include <LIS3DHTR.h>  // Include the LIS3DHTR library

// Pin Definitions
#define BUTTON_PIN          16  // Main control button
#define MOSFET_DRIVER_LOW   12  // MOSFET driver for low beam
#define MOSFET_DRIVER_HIGH  11  // MOSFET driver for high beam
#define POTENTIOMETER_PIN   15  // Potentiometer pin

// LED Strip Settings
#define LED_PIN    10
#define LED_NUM    88
#define LED_SPEED  500

// Default Settings
uint32_t currentColor     = 65535;  // Default LED color (pink-ish)
uint32_t currentColorHue  = 1000;   // Default LED hue
uint8_t  currentBrightness = 150;   // Default LED brightness
uint8_t  currentEffect     = 0;
const uint8_t effects[] = {12, 11, 2, 7, 0, 1, 3, 5, 8, 13, 17, 18, 20, 27, 33, 39};

// Constants
#define LONG_PRESS_DURATION       1500  // Duration for long press in milliseconds
#define SIDEWAYS_THRESHOLD_TIME   5000  // Time in milliseconds to detect sideways orientation
#define SIDEWAYS_ACCEL_THRESHOLD  0.7   // Acceleration threshold for sideways detection (in g)

// PWM Settings
const int PWM_FREQ         = 5000; // PWM frequency in Hz
const int PWM_RESOLUTION   = 8;    // PWM resolution in bits
const int PWM_CHANNEL_LOW  = 0;    // PWM channel for low beam
const int PWM_CHANNEL_HIGH = 1;    // PWM channel for high beam

// Variables
volatile bool state = false;  // System state: true = on, false = off
const uint8_t numReadings = 10;
int readings[numReadings] = {0};  // Potentiometer readings for averaging
uint8_t readIndex = 0;
int total = 0;
int previousPotValue = 0;

// Orientation Detection Variables
LIS3DHTR<TwoWire> lis;  // Accelerometer object
unsigned long sidewaysStartTime = 0;
bool isSideways = false;
bool orientationDisabled = false;

// Objects
EasyButton button(BUTTON_PIN);
WS2812FX ws2812fx = WS2812FX(LED_NUM, LED_PIN, NEO_RGB + NEO_KHZ800);

// Function Prototypes
void initializePeripherals();
void resetPeripherals();
void restorePeripherals();
void buttonLongPressed();
void buttonPressed();
void changeEffect();
int readPotentiometer();
void changeLight(int potValue);
void checkOrientation();

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // Set ADC resolution to 12 bits

  initializePeripherals();
  resetPeripherals();
}

void initializePeripherals() {
  button.begin();
  button.onPressed(buttonPressed);
  button.onPressedFor(LONG_PRESS_DURATION, buttonLongPressed);

  ws2812fx.init();
  ws2812fx.setBrightness(currentBrightness);
  ws2812fx.setSpeed(LED_SPEED);
  ws2812fx.setColor(currentColor);
  ws2812fx.setMode(effects[currentEffect]);
  ws2812fx.start();

  // Set up PWM for MOSFET drivers
  ledcSetup(PWM_CHANNEL_LOW, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOSFET_DRIVER_LOW, PWM_CHANNEL_LOW);

  ledcSetup(PWM_CHANNEL_HIGH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOSFET_DRIVER_HIGH, PWM_CHANNEL_HIGH);

  pinMode(POTENTIOMETER_PIN, INPUT);

  // Initialize accelerometer
  Wire.begin();
  if (!lis.begin(Wire, 0x18)) {  // 0x18 is the default I2C address
    Serial.println("Failed to initialize LIS3DHTR.");
    while (1);
  }
  lis.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  lis.setHighSolution(true);
}

void resetPeripherals() {
  ws2812fx.stop();           // Stop the LED effects
  ws2812fx.setBrightness(0); // Turn off LED strip
  ws2812fx.strip_off();      // Ensure LEDs are off
  ledcWrite(PWM_CHANNEL_HIGH, 0);
  ledcWrite(PWM_CHANNEL_LOW, 0);
}

void restorePeripherals() {
  ws2812fx.setBrightness(currentBrightness);
  ws2812fx.setColor(currentColor);
  ws2812fx.setMode(effects[currentEffect]);
  ws2812fx.start(); // Start the LED effects

  // Restore PWM outputs according to potentiometer
  changeLight(readPotentiometer());
}

void buttonPressed() {
  if (state && !orientationDisabled) {
    changeEffect();
  }
}

void buttonLongPressed() {
  Serial.println("Long Pressed");
  if (!state) {
    state = true;
    if (!orientationDisabled) {
      restorePeripherals();
    }
  } else {
    state = false;
    resetPeripherals();
  }
}

void changeEffect() {
  currentEffect = (currentEffect + 1) % (sizeof(effects) / sizeof(effects[0]));
  ws2812fx.setMode(effects[currentEffect]);
  Serial.println(effects[currentEffect]);
}

int readPotentiometer() {
  total -= readings[readIndex];
  readings[readIndex] = analogRead(POTENTIOMETER_PIN);
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  int average = total / numReadings;
  previousPotValue = average;
  return previousPotValue;
}

void changeLight(int potValue) {
  uint8_t pwm;
  if (potValue < 1680) {  // Low beam range
    pwm = map(potValue, 0, 1680, 255, 0);
    ledcWrite(PWM_CHANNEL_LOW, pwm);
    ledcWrite(PWM_CHANNEL_HIGH, 0);
  } else if (potValue < 2080) {  // Dead zone
    ledcWrite(PWM_CHANNEL_LOW, 0);
    ledcWrite(PWM_CHANNEL_HIGH, 0);
  } else {  // High beam range
    pwm = map(potValue, 2080, 4095, 0, 255);
    ledcWrite(PWM_CHANNEL_LOW, 0);
    ledcWrite(PWM_CHANNEL_HIGH, pwm);
  }
}

void checkOrientation() {
  // Read accelerometer data
  float ax = lis.getAccelerationX();
  float ay = lis.getAccelerationY();
  float az = lis.getAccelerationZ();

  // Determine if the board is sideways
  bool sideways = false;
  if (abs(az) < SIDEWAYS_ACCEL_THRESHOLD) {
    if (abs(ax) > SIDEWAYS_ACCEL_THRESHOLD || abs(ay) > SIDEWAYS_ACCEL_THRESHOLD) {
      sideways = true;
    }
  }

  // Handle sideways detection
  if (sideways) {
    if (!isSideways) {
      isSideways = true;
      sidewaysStartTime = millis();
    } else {
      if (millis() - sidewaysStartTime >= SIDEWAYS_THRESHOLD_TIME) {
        if (!orientationDisabled) {
          orientationDisabled = true;
          resetPeripherals();
          Serial.println("Board is sideways. Shutting down lights.");
        }
      }
    }
  } else {
    isSideways = false;
    sidewaysStartTime = millis(); // Reset the sideways timer

    if (orientationDisabled) {
      orientationDisabled = false;
      if (state) {
        restorePeripherals();
        Serial.println("Board returned to normal orientation. Restoring lights.");
      }
    }
  }
}

void loop() {
  button.read();
  ws2812fx.service();

  if (state && !orientationDisabled) {
    changeLight(readPotentiometer());
  }

  checkOrientation();
}
