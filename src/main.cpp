#include <Arduino.h>
#include <EasyButton.h>
#include <Preferences.h> // Include Preferences library for NVS storage

#define DEBUG
#define ACC
// #define BLE

#ifdef ACC
#include <Wire.h>    // Include the Wire library
#include "LIS3DHTR.h" // Include the LIS3DHTR library
#endif

#ifdef BLE
#include <BLEDevice.h> // Core BLE functionality
#include <BLEServer.h> // BLE Server setup
#include <BLEUtils.h>  // Utility functions
#include <BLE2902.h>   // For BLE2902 descriptor (notifications)

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#endif

#define __MINMAX_H__ // Fix for conflicting min/max definitions
#include <WS2812FX.h>

// Pin Definitions
constexpr uint8_t BUTTON_PIN = 7;          // Main control button
constexpr uint8_t MOSFET_DRIVER_LOW = 19;  // MOSFET driver for low beam
constexpr uint8_t MOSFET_DRIVER_HIGH = 20; // MOSFET driver for high beam
constexpr uint8_t POT_PIN = 5;             // Potentiometer pin
constexpr uint8_t POT_ENABLE = 15;         // Pot enable pin
constexpr uint8_t ACC_CS_PIN = 10;         // Accelerometer chip select pin
constexpr uint8_t ACC_SCL_PIN = 12;        // Accelerometer clock pin
constexpr uint8_t ACC_SDA_PIN = 11;        // Accelerometer data pin

// LED Strip Settings
constexpr uint8_t LED_PIN = 8;         // WS2815 LED control pin
constexpr uint8_t LED_BACKUP_PIN = 16; // Backup LED control pin
constexpr uint8_t LED_NUM = 10;        // LED amount
constexpr uint16_t LED_SPEED = 500;    // LED update speed

// Constants
constexpr uint8_t numReadings = 10;                                                  // Amount of pot values to average
const uint8_t effects[] = {12, 11, 2, 7, 0, 1, 3, 5, 8, 13, 17, 18, 20, 27, 33, 39}; // LED effects to use

// PWM Settings
constexpr int PWM_FREQ = 5000;      // PWM frequency in Hz
constexpr int PWM_RESOLUTION = 8;   // PWM resolution in bits
constexpr int PWM_CHANNEL_LOW = 0;  // PWM channel for low beam
constexpr int PWM_CHANNEL_HIGH = 1; // PWM channel for high beam

// Default Settings (will be used if no stored values are found)
constexpr uint32_t LED_DEFAULT_COLOR = 30000;   // Default LED color (pink-ish)
constexpr uint8_t LED_DEFAULT_BRIGHTNESS = 150; // Default LED brightness
constexpr uint8_t LED_DEFAULT_EFFECT = 0;       // Default LED effect

// Advanced Settings (will be used if no stored values are found)
constexpr uint16_t DEFAULT_LONG_PRESS_DURATION = 1500;     // Duration for long press in milliseconds
constexpr uint16_t DEFAULT_SIDEWAYS_THRESHOLD_TIME = 1000; // Time in milliseconds to detect sideways orientation
constexpr float DEFAULT_SIDEWAYS_ACCEL_THRESHOLD = 0.7;    // Acceleration threshold for sideways detection (in g)
constexpr bool DEFAULT_ACCEL_AUTO_SHUTDOWN = true;        // Automatically shut down if the sledge is sideways
constexpr bool DEFAULT_ACCEL_AUTO_STARTUP = false;         // Automatically start back up if the sledge is on the ground again

// Variables (will be loaded from storage or defaulted)
uint32_t currentColor;
uint8_t currentBrightness;
uint8_t currentEffect;
uint16_t LONG_PRESS_DURATION;
uint16_t SIDEWAYS_THRESHOLD_TIME;
float SIDEWAYS_ACCEL_THRESHOLD;
bool ACCEL_AUTO_SHUTDOWN; // Shut down automatically if the sledge is upwards
bool ACCEL_AUTO_STARTUP;  // Start back up if the sledge is on the ground again

// Variables
volatile bool state = false;     // System state: true = on, false = off
int readings[numReadings] = {0}; // Potentiometer readings for averaging
uint8_t readIndex = 0;
int total = 0;
int previousPotValue = 0;

// Orientation Detection Variables
#ifdef ACC
#define WIRE Wire
LIS3DHTR<TwoWire> lis; // Accelerometer object
#endif
unsigned long sidewaysStartTime = 0;
bool isSideways = false;
bool orientationDisabled = false;

// Bluetooth and Preferences
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
void loadConfiguration();
void saveConfiguration();
#ifdef ACC
void checkOrientation();
#endif
#ifdef BLE
BLECharacteristic *pCharacteristic;
String processCommand(String input);

class CommandCallbacks : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string cmd = pCharacteristic->getValue();
    cmd.erase(std::remove(cmd.begin(), cmd.end(), '\n'), cmd.end()); // Remove newline characters
    cmd.erase(std::remove(cmd.begin(), cmd.end(), '\r'), cmd.end()); // Remove carriage returns

    Serial.print("Received command: ");
    Serial.println(cmd.c_str());

    // Process the command similar to your handleBluetooth() function
    String input = String(cmd.c_str());
    input.trim();

    String response = processCommand(input);

    // Send the response back via notification
    if (response.length() > 0)
    {
      pCharacteristic->setValue(response.c_str());
      pCharacteristic->notify();
      Serial.print("Sent response: ");
      Serial.println(response);
    }
  }
};

String processCommand(String input)
{
  if (input.startsWith("SET_COLOR "))
  {
    uint32_t color = strtoul(input.substring(10).c_str(), NULL, 16);
    currentColor = color;
    ws2812fx.setColor(currentColor);
    saveConfiguration();
    return "Color set to: " + String(currentColor, HEX);
  }
  else if (input.startsWith("SET_BRIGHTNESS "))
  {
    int brightness = input.substring(15).toInt();
    currentBrightness = constrain(brightness, 0, 255);
    ws2812fx.setBrightness(currentBrightness);
    saveConfiguration();
    return "Brightness set to: " + String(currentBrightness);
  }
  else if (input.startsWith("SET_EFFECT "))
  {
    int effectIndex = input.substring(11).toInt();
    if (effectIndex >= 0 && effectIndex < (sizeof(effects) / sizeof(effects[0])))
    {
      currentEffect = effectIndex;
      ws2812fx.setMode(effects[currentEffect]);
      saveConfiguration();
      return "Effect set to: " + String(currentEffect);
    }
    else
    {
      return "Invalid effect index.";
    }
  }
  else if (input.startsWith("SET_ACCEL_AUTO_SHUTDOWN "))
  {
    int value = input.substring(24).toInt();
    ACCEL_AUTO_SHUTDOWN = (value != 0);
    saveConfiguration();
    return "ACCEL_AUTO_SHUTDOWN set to: " + String(ACCEL_AUTO_SHUTDOWN);
  }
  else if (input.startsWith("SET_ACCEL_AUTO_STARTUP "))
  {
    int value = input.substring(23).toInt();
    ACCEL_AUTO_STARTUP = (value != 0);
    saveConfiguration();
    return "ACCEL_AUTO_STARTUP set to: " + String(ACCEL_AUTO_STARTUP);
  }
  else if (input.equals("GET_ACCEL_AUTO_SHUTDOWN"))
  {
    return String(ACCEL_AUTO_SHUTDOWN);
  }
  else if (input.equals("GET_ACCEL_AUTO_STARTUP"))
  {
    return String(ACCEL_AUTO_STARTUP);
  }
  else if (input.startsWith("SET_LONG_PRESS_DURATION "))
  {
    int duration = input.substring(24).toInt();
    LONG_PRESS_DURATION = duration;
    button.onPressedFor(LONG_PRESS_DURATION, buttonLongPressed); // Update the button long press duration
    saveConfiguration();
    return "Long press duration set to: " + String(LONG_PRESS_DURATION);
  }
  else if (input.startsWith("SET_SIDEWAYS_THRESHOLD_TIME "))
  {
    int time = input.substring(27).toInt();
    SIDEWAYS_THRESHOLD_TIME = time;
    saveConfiguration();
    return "Sideways threshold time set to: " + String(SIDEWAYS_THRESHOLD_TIME);
  }
  else if (input.startsWith("SET_SIDEWAYS_ACCEL_THRESHOLD "))
  {
    float threshold = input.substring(29).toFloat();
    SIDEWAYS_ACCEL_THRESHOLD = threshold;
    saveConfiguration();
    return "Sideways acceleration threshold set to: " + String(SIDEWAYS_ACCEL_THRESHOLD);
  }
  else if (input.equals("GET_COLOR"))
  {
    return String(currentColor, HEX);
  }
  else if (input.equals("GET_BRIGHTNESS"))
  {
    return String(currentBrightness);
  }
  else if (input.equals("GET_EFFECT"))
  {
    return String(currentEffect);
  }
  else if (input.equals("GET_LONG_PRESS_DURATION"))
  {
    return String(LONG_PRESS_DURATION);
  }
  else if (input.equals("GET_SIDEWAYS_THRESHOLD_TIME"))
  {
    return String(SIDEWAYS_THRESHOLD_TIME);
  }
  else if (input.equals("GET_SIDEWAYS_ACCEL_THRESHOLD"))
  {
    return String(SIDEWAYS_ACCEL_THRESHOLD);
  }
  else if (input.equals("GET_CONFIG"))
  {
    // Send current configuration values without labels
    String config = "";
    config += String(currentColor, HEX) + "\n";
    config += String(currentBrightness) + "\n";
    config += String(currentEffect) + "\n";
    config += String(LONG_PRESS_DURATION) + "\n";
    config += String(SIDEWAYS_THRESHOLD_TIME) + "\n";
    config += String(SIDEWAYS_ACCEL_THRESHOLD);
    return config;
  }
  else
  {
    return "Unknown command.";
  }
}
#endif

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif
  analogReadResolution(12); // Set ADC resolution to 12 bits

  initializePeripherals();
  resetPeripherals();

#ifdef BLE
  // Initialize BLE
  BLEDevice::init("ESP32_BLE_Server");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  // Add descriptor for notifications
  pCharacteristic->addDescriptor(new BLE2902());

  // Set callbacks
  pCharacteristic->setCallbacks(new CommandCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Helps with iPhone connection issues
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

#ifdef DEBUG
  Serial.println("BLE Server is now advertising...");
#endif
#endif
}

void initializePeripherals()
{
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
  // ledcAttachChannel(MOSFET_DRIVER_LOW, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_LOW);

  ledcSetup(PWM_CHANNEL_HIGH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOSFET_DRIVER_HIGH, PWM_CHANNEL_HIGH);
  // ledcAttachChannel(MOSFET_DRIVER_HIGH, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_HIGH);

  pinMode(POT_PIN, INPUT);
  pinMode(POT_ENABLE, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BACKUP_PIN, OUTPUT);

  // Initialize accelerometer
#ifdef ACC
  pinMode(ACC_CS_PIN, OUTPUT);
  digitalWrite(ACC_CS_PIN, HIGH); // Enable the accelerometer
  Wire.setPins(ACC_SDA_PIN, ACC_SCL_PIN);
  lis.begin(Wire, 0x18); // Initialize the LIS3DHTR sensor
  if (!lis.isConnection())
  { // 0x18 is the default I2C address
#ifdef DEBUG
    Serial.println("Failed to initialize LIS3DHTR.");
#endif
    while (1);
  }
  lis.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  lis.setHighSolution(true);
#endif

  // Initialize Bluetooth
#ifdef DEBUG && BLE
  Serial.println("Bluetooth started. Waiting for connections...");
#endif
}

void resetPeripherals()
{
  digitalWrite(LED_BACKUP_PIN, LOW);
  digitalWrite(POT_ENABLE, HIGH); // Reversed, turn off pot
  ws2812fx.stop();                // Stop the LED effects
  ws2812fx.setBrightness(0);      // Turn off LED strip
  ws2812fx.strip_off();           // Ensure LEDs are off
  ledcWrite(PWM_CHANNEL_HIGH, 0);
  ledcWrite(PWM_CHANNEL_LOW, 0);
}

void restorePeripherals()
{
  digitalWrite(POT_ENABLE, LOW); // Reversed, turn on pot
  ws2812fx.setBrightness(currentBrightness);
  ws2812fx.setColor(currentColor);
  ws2812fx.setMode(effects[currentEffect]);
  ws2812fx.start(); // Start the LED effects

  // Restore PWM outputs according to potentiometer
  changeLight(readPotentiometer());
}

void buttonPressed()
{
  if (state && !orientationDisabled)
  {
    changeEffect();
  }
}

void buttonLongPressed()
{
#ifdef DEBUG
  Serial.println("Long Pressed");
#endif
  if (!state)
  {
    state = true;
    if (!orientationDisabled)
    {
      restorePeripherals();
    }
  }
  else
  {
    state = false;
    resetPeripherals();
  }
}

void changeEffect()
{
  currentEffect = (currentEffect + 1) % (sizeof(effects) / sizeof(effects[0]));
  ws2812fx.setMode(effects[currentEffect]);
  saveConfiguration(); // Save the new effect to NVS
#ifdef DEBUG
  Serial.println("Effect changed to: " + String(currentEffect));
#endif
}

int readPotentiometer()
{
  total -= readings[readIndex];
  readings[readIndex] = analogRead(POT_PIN);
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  int average = total / numReadings;
  previousPotValue = average;
  return previousPotValue;
}

void changeLight(int potValue)
{
  uint8_t pwm;
  if (potValue < 1900)
  { // Low beam range
    pwm = map(potValue, 0, 1900, 255, 0);
    ledcWrite(PWM_CHANNEL_LOW, pwm);
    ledcWrite(PWM_CHANNEL_HIGH, 0);
  }
  else if (potValue < 2500)
  { // Dead zone
    ledcWrite(PWM_CHANNEL_LOW, 0);
    ledcWrite(PWM_CHANNEL_HIGH, 0);
  }
  else
  { // High beam range
    pwm = map(potValue, 2500, 4095, 0, 255);
    ledcWrite(PWM_CHANNEL_LOW, 0);
    ledcWrite(PWM_CHANNEL_HIGH, pwm);
  }
}

#ifdef ACC
void checkOrientation()
{
  // Read accelerometer data
  float ax = lis.getAccelerationX();
  float ay = lis.getAccelerationY();
  float az = lis.getAccelerationZ();

  // Determine if the board is sideways
  bool sideways = false;
  if (abs(az) < SIDEWAYS_ACCEL_THRESHOLD)
  {
    if (abs(ax) > SIDEWAYS_ACCEL_THRESHOLD || abs(ay) > SIDEWAYS_ACCEL_THRESHOLD)
    {
      sideways = true;
    }
  }

  // Handle sideways detection
  if (sideways)
  {
    if (!isSideways)
    {
      isSideways = true;
      sidewaysStartTime = millis();
    }
    else
    {
      if (millis() - sidewaysStartTime >= SIDEWAYS_THRESHOLD_TIME)
      {
        if (!orientationDisabled && ACCEL_AUTO_SHUTDOWN) // shut down if the sledge is sideways
        {
          orientationDisabled = true;
          state = false;
          resetPeripherals();
#ifdef DEBUG
          Serial.println("Board is sideways. Shutting down lights.");
#endif
        }
      }
    }
  }
  else
  {
    isSideways = false;
    sidewaysStartTime = millis(); // Reset the sideways timer

    if (orientationDisabled)
    {
      orientationDisabled = false;
      if (state == false && ACCEL_AUTO_STARTUP)  // start back up if the sledge is on the ground again
      {
        state = true;
        restorePeripherals();
#ifdef DEBUG
        Serial.println("Board returned to normal orientation. Restoring lights.");
#endif
      }
    }
  }
}
#endif

void loadConfiguration()
{
  preferences.begin("config", false);
  currentColor = preferences.getUInt("LC", LED_DEFAULT_COLOR);
  currentBrightness = preferences.getUChar("LB", LED_DEFAULT_BRIGHTNESS);
  currentEffect = preferences.getUChar("LE", LED_DEFAULT_EFFECT);
  LONG_PRESS_DURATION = preferences.getUShort("LPD", DEFAULT_LONG_PRESS_DURATION);
  SIDEWAYS_THRESHOLD_TIME = preferences.getUShort("STT", DEFAULT_SIDEWAYS_THRESHOLD_TIME);
  SIDEWAYS_ACCEL_THRESHOLD = preferences.getFloat("SAT", DEFAULT_SIDEWAYS_ACCEL_THRESHOLD);
  ACCEL_AUTO_SHUTDOWN = preferences.getBool("AASD", DEFAULT_ACCEL_AUTO_SHUTDOWN);
  ACCEL_AUTO_STARTUP = preferences.getBool("AASU", DEFAULT_ACCEL_AUTO_STARTUP); 
  preferences.end();

  // for now, force accel_auto_startup to true
  ACCEL_AUTO_STARTUP = true;

#ifdef DEBUG
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
#endif
}

void saveConfiguration()
{
  preferences.begin("config", false);
  preferences.putUInt("LC", currentColor);
  preferences.putUChar("LB", currentBrightness);
  preferences.putUChar("LE", currentEffect);
  preferences.putUShort("LPD", LONG_PRESS_DURATION);
  preferences.putUShort("STT", SIDEWAYS_THRESHOLD_TIME);
  preferences.putFloat("SAT", SIDEWAYS_ACCEL_THRESHOLD);
  preferences.putBool("AASD", ACCEL_AUTO_SHUTDOWN);
  preferences.putBool("AASU", ACCEL_AUTO_STARTUP);
  preferences.end();

#ifdef DEBUG
  Serial.println("Configuration saved.");
#endif
}

void loop()
{
  button.read();
  ws2812fx.service();

  if (state && !orientationDisabled)
  {
    // Serial.println("Pot value: " + String(readPotentiometer()));
    changeLight(readPotentiometer());
  }

#ifdef ACC
  if (ACCEL_AUTO_SHUTDOWN) {  // Only check orientation if auto shutdown is enabled
    checkOrientation();
  }
#endif
}
