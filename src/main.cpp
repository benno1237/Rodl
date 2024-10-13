#include <Arduino.h>
#include <WS2812FX.h>
#include <EasyButton.h>
#include <LIS3DHTR.h>          // Include the LIS3DHTR library
#include <BluetoothSerial.h>   // Include BluetoothSerial library
#include <Preferences.h>       // Include Preferences library for NVS storage

// Pin Definitions
constexpr uint8_t BUTTON_PIN          = 16;  // Main control button
constexpr uint8_t MOSFET_DRIVER_LOW   = 12;  // MOSFET driver for low beam
constexpr uint8_t MOSFET_DRIVER_HIGH  = 11;  // MOSFET driver for high beam
constexpr uint8_t POTENTIOMETER_PIN   = 15;  // Potentiometer pin

// LED Strip Settings
constexpr uint8_t   LED_PIN     = 10;  // WS2815 LED control pin
constexpr uint8_t   LED_NUM     = 88;  // LED amount
constexpr uint16_t  LED_SPEED   = 500; // LED update speed

// Constants
constexpr uint8_t   numReadings              = 10;    // Amount of pot values to average
const uint8_t effects[] = {12, 11, 2, 7, 0, 1, 3, 5, 8, 13, 17, 18, 20, 27, 33, 39};  // LED effects to use

// PWM Settings
constexpr int PWM_FREQ         = 5000; // PWM frequency in Hz
constexpr int PWM_RESOLUTION   = 8;    // PWM resolution in bits
constexpr int PWM_CHANNEL_LOW  = 0;    // PWM channel for low beam
constexpr int PWM_CHANNEL_HIGH = 1;    // PWM channel for high beam

// Default Settings (will be used if no stored values are found)
constexpr uint32_t DEFAULT_COLOR      = 65535;  // Default LED color (pink-ish)
constexpr uint8_t  DEFAULT_BRIGHTNESS = 150;    // Default LED brightness
constexpr uint8_t  DEFAULT_EFFECT     = 0;      // Default LED effect

// Advanced Settings (will be used if no stored values are found)
constexpr uint16_t  DEFAULT_LONG_PRESS_DURATION      = 1500;  // Duration for long press in milliseconds
constexpr uint16_t  DEFAULT_SIDEWAYS_THRESHOLD_TIME  = 5000;  // Time in milliseconds to detect sideways orientation
constexpr float     DEFAULT_SIDEWAYS_ACCEL_THRESHOLD = 0.7;   // Acceleration threshold for sideways detection (in g)

// Variables (will be loaded from storage or defaulted)
uint32_t  currentColor;
uint8_t   currentBrightness;
uint8_t   currentEffect;
uint16_t  LONG_PRESS_DURATION;
uint16_t  SIDEWAYS_THRESHOLD_TIME;
float     SIDEWAYS_ACCEL_THRESHOLD;
bool     ACCEL_AUTO_SHUTDOWN;  // Shut down automatically if the sledge is upwards
bool     ACCEL_AUTO_STARTUP;   // Start back up if the sledge is on the ground again

// Variables
volatile bool state = false;  // System state: true = on, false = off
int readings[numReadings] = {0};  // Potentiometer readings for averaging
uint8_t readIndex = 0;
int total = 0;
int previousPotValue = 0;

// Orientation Detection Variables
LIS3DHTR<TwoWire> lis;  // Accelerometer object
unsigned long sidewaysStartTime = 0;
bool isSideways = false;
bool orientationDisabled = false;

// Bluetooth and Preferences
BluetoothSerial SerialBT;
Preferences preferences;

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
void handleBluetooth();
void loadConfiguration();
void saveConfiguration();

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // Set ADC resolution to 12 bits

  initializePeripherals();
  resetPeripherals();
}

void initializePeripherals() {
  // Load configuration from NVS
  loadConfiguration();

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

  // Initialize Bluetooth
  SerialBT.begin("ESP32_Lights"); // Bluetooth device name
  Serial.println("Bluetooth started. Waiting for connections...");
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
  saveConfiguration(); // Save the new effect to NVS
  Serial.println("Effect changed to: " + String(currentEffect));
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

void handleBluetooth() {
  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');
    input.trim();

    // Parse the input command
    if (input.startsWith("SET_COLOR ")) {
      uint32_t color = strtoul(input.substring(10).c_str(), NULL, 16);
      currentColor = color;
      ws2812fx.setColor(currentColor);
      saveConfiguration();
      SerialBT.println("Color set to: " + String(currentColor, HEX));
    } else if (input.startsWith("SET_BRIGHTNESS ")) {
      int brightness = input.substring(15).toInt();
      currentBrightness = constrain(brightness, 0, 255);
      ws2812fx.setBrightness(currentBrightness);
      saveConfiguration();
      SerialBT.println("Brightness set to: " + String(currentBrightness));
    } else if (input.startsWith("SET_EFFECT ")) {
      int effectIndex = input.substring(11).toInt();
      if (effectIndex >= 0 && effectIndex < (sizeof(effects) / sizeof(effects[0]))) {
        currentEffect = effectIndex;
        ws2812fx.setMode(effects[currentEffect]);
        saveConfiguration();
        SerialBT.println("Effect set to: " + String(currentEffect));
      } else {
        SerialBT.println("Invalid effect index.");
      }
    } else if (input.startsWith("SET_ACCEL_AUTO_SHUTDOWN ")) {
      int value = input.substring(24).toInt();
      ACCEL_AUTO_SHUTDOWN = (value != 0);
      saveConfiguration();
      SerialBT.println("ACCEL_AUTO_SHUTDOWN set to: " + String(ACCEL_AUTO_SHUTDOWN));
    } else if (input.startsWith("SET_ACCEL_AUTO_STARTUP ")) {
      int value = input.substring(23).toInt();
      ACCEL_AUTO_STARTUP = (value != 0);
      saveConfiguration();
      SerialBT.println("ACCEL_AUTO_STARTUP set to: " + String(ACCEL_AUTO_STARTUP));
    } else if (input.equals("GET_ACCEL_AUTO_SHUTDOWN")) {
      SerialBT.println(String(ACCEL_AUTO_SHUTDOWN));
    } else if (input.equals("GET_ACCEL_AUTO_STARTUP")) {
      SerialBT.println(String(ACCEL_AUTO_STARTUP));
    } else if (input.startsWith("SET_LONG_PRESS_DURATION ")) {
      int duration = input.substring(24).toInt();
      LONG_PRESS_DURATION = duration;
      button.onPressedFor(LONG_PRESS_DURATION, buttonLongPressed); // Update the button long press duration
      saveConfiguration();
      SerialBT.println("Long press duration set to: " + String(LONG_PRESS_DURATION));
    } else if (input.startsWith("SET_SIDEWAYS_THRESHOLD_TIME ")) {
      int time = input.substring(27).toInt();
      SIDEWAYS_THRESHOLD_TIME = time;
      saveConfiguration();
      SerialBT.println("Sideways threshold time set to: " + String(SIDEWAYS_THRESHOLD_TIME));
    } else if (input.startsWith("SET_SIDEWAYS_ACCEL_THRESHOLD ")) {
      float threshold = input.substring(29).toFloat();
      SIDEWAYS_ACCEL_THRESHOLD = threshold;
      saveConfiguration();
      SerialBT.println("Sideways acceleration threshold set to: " + String(SIDEWAYS_ACCEL_THRESHOLD));
    } else if (input.equals("GET_COLOR")) {
      SerialBT.println(String(currentColor, HEX));
    } else if (input.equals("GET_BRIGHTNESS")) {
      SerialBT.println(String(currentBrightness));
    } else if (input.equals("GET_EFFECT")) {
      SerialBT.println(String(currentEffect));
    } else if (input.equals("GET_LONG_PRESS_DURATION")) {
      SerialBT.println(String(LONG_PRESS_DURATION));
    } else if (input.equals("GET_SIDEWAYS_THRESHOLD_TIME")) {
      SerialBT.println(String(SIDEWAYS_THRESHOLD_TIME));
    } else if (input.equals("GET_SIDEWAYS_ACCEL_THRESHOLD")) {
      SerialBT.println(String(SIDEWAYS_ACCEL_THRESHOLD));
    } else if (input.equals("GET_CONFIG")) {
      // Send current configuration values without labels
      SerialBT.println(String(currentColor, HEX));
      SerialBT.println(String(currentBrightness));
      SerialBT.println(String(currentEffect));
      SerialBT.println(String(LONG_PRESS_DURATION));
      SerialBT.println(String(SIDEWAYS_THRESHOLD_TIME));
      SerialBT.println(String(SIDEWAYS_ACCEL_THRESHOLD));
    } else {
      SerialBT.println("Unknown command.");
    }
  }
}


void loadConfiguration() {
  preferences.begin("config", false);
  currentColor = preferences.getUInt("ledColor", DEFAULT_COLOR);
  currentBrightness = preferences.getUChar("ledBrightness", DEFAULT_BRIGHTNESS);
  currentEffect = preferences.getUChar("ledEffect", DEFAULT_EFFECT);
  LONG_PRESS_DURATION = preferences.getUShort("LONG_PRESS_DURATION", DEFAULT_LONG_PRESS_DURATION);
  SIDEWAYS_THRESHOLD_TIME = preferences.getUShort("SIDEWAYS_THRESHOLD_TIME", DEFAULT_SIDEWAYS_THRESHOLD_TIME);
  SIDEWAYS_ACCEL_THRESHOLD = preferences.getFloat("SIDEWAYS_ACCEL_THRESHOLD", DEFAULT_SIDEWAYS_ACCEL_THRESHOLD);
  ACCEL_AUTO_SHUTDOWN = preferences.getBool("ACCEL_AUTO_SHUTDOWN", false);  // this surely defaults to false
  ACCEL_AUTO_STARTUP = preferences.getBool("ACCEL_AUTO_STARTUP", false);    // same here
  preferences.end();

  Serial.println("Configuration loaded:");
  Serial.println("Color: " + String(currentColor, HEX));
  Serial.println("Brightness: " + String(currentBrightness));
  Serial.println("Effect: " + String(currentEffect));
  Serial.println();
  Serial.println("Advanced configuration:");
  Serial.println("LONG_PRESS_DURATION: " + String(LONG_PRESS_DURATION));
  Serial.println("SIDEWAYS_THRESHOLD_TIME: " + String(SIDEWAYS_THRESHOLD_TIME));
  Serial.println("SIDEWAYS_ACCEL_THRESHOLD: " + String(SIDEWAYS_ACCEL_THRESHOLD));
  Serial.println("ACCEL_AUTO_SHUTDOWN: " + String(ACCEL_AUTO_SHUTDOWN));
  Serial.println("ACCEL_AUTO_STARTUP: " + String(ACCEL_AUTO_STARTUP));
}


void saveConfiguration() {
  preferences.begin("config", false);
  preferences.putUInt("ledColor", currentColor);
  preferences.putUChar("ledBrightness", currentBrightness);
  preferences.putUChar("ledEffect", currentEffect);
  preferences.putUShort("LONG_PRESS_DURATION", LONG_PRESS_DURATION);
  preferences.putUShort("SIDEWAYS_THRESHOLD_TIME", SIDEWAYS_THRESHOLD_TIME);
  preferences.putFloat("SIDEWAYS_ACCEL_THRESHOLD", SIDEWAYS_ACCEL_THRESHOLD);
  preferences.putBool("ACCEL_AUTO_SHUTDOWN", ACCEL_AUTO_SHUTDOWN);
  preferences.putBool("ACCEL_AUTO_STARTUP", ACCEL_AUTO_STARTUP);
  preferences.end();

  Serial.println("Configuration saved.");
}


void loop() {
  button.read();
  ws2812fx.service();
  handleBluetooth();

  if (state && !orientationDisabled) {
    changeLight(readPotentiometer());
  }

  checkOrientation();
}
