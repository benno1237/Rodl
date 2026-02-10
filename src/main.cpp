#include <Arduino.h>
#include <EasyButton.h>
#include <Preferences.h> // Include Preferences library for NVS storage
#include <TinyGPS++.h>   // GPS parsing library
#include <HardwareSerial.h>
#include <LittleFS.h>
#include <FS.h>

#define DEBUG
#define ACC
#define GPS

#ifdef ACC
#include <Wire.h>    // Include the Wire library
#include "LIS3DHTR.h" // Include the LIS3DHTR library
#endif

#define __MINMAX_H__ // Fix for conflicting min/max definitions
#include <WS2812FX.h>

// Pin Definitions
constexpr uint8_t BUTTON_PIN = 4;        // Main control button
constexpr uint8_t LED_BUTTON_PIN = 2;       // Potentiometer enable pin
constexpr uint8_t LED_LOW_PIN = 13;      // Low beam LED driver
constexpr uint8_t LED_HIGH_PIN = 14;     // High beam LED driver
constexpr uint8_t POT_PIN = 5;           // Potentiometer pin
constexpr uint8_t POT_ENABLE = 6;       // Pot enable pin
constexpr uint8_t ACC_SCL_PIN = 9;      // Accelerometer clock pin
constexpr uint8_t ACC_SDA_PIN = 10;      // Accelerometer data pin
constexpr uint8_t ACC_INT_PIN = 11;      // Accelerometer interrupt pin

// GPS Pin Definitions
constexpr uint8_t GPS_RX_PIN = 17;       // GPS RX (ESP32 TX)
constexpr uint8_t GPS_TX_PIN = 18;       // GPS TX (ESP32 RX)

// LED Strip Settings
constexpr uint8_t LED_PIN = 12;          // WS2815 LED control pin
constexpr uint8_t LED_NUM = 10;          // LED amount
constexpr uint16_t LED_SPEED = 500;    // LED update speed

// GPS Settings
constexpr uint32_t GPS_BAUDRATE = 115200;
constexpr uint16_t GPS_UPDATE_INTERVAL_MS = 100;  // 10Hz update rate
constexpr uint16_t MAX_RIDES = 5;
constexpr uint16_t POINTS_PER_RIDE = 6000;  // 10Hz × 600 seconds = 10 minutes (max)

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

// GPS Data Structure
struct GPSPoint {
  unsigned long timestamp;  // ms since start
  double lat;               // degrees
  double lon;               // degrees
  float speed_kmh;          // km/h
  float alt_m;              // meters
  uint8_t sats;             // number of satellites
  float hdop;               // horizontal dilution of precision
  uint16_t age;             // sentence age in ms
};

// GPS Variables
#ifdef GPS
GPSPoint rideBuffer[MAX_RIDES][POINTS_PER_RIDE];
uint16_t rideCounts[MAX_RIDES] = {0};
uint8_t currentRideIndex = 0;
bool isGPSLogging = false;
unsigned long lastGPSLogTime = 0;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // UART2 for GPS
#endif

// Preferences
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
#ifdef GPS
void initGPS();
void startGPSLogging();
void stopGPSLogging();
bool saveRideToFS(uint8_t rideIndex);
uint8_t getOldestRideIndex();
void processGPSData();
#endif

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif
  analogReadResolution(12); // Set ADC resolution to 12 bits

  initializePeripherals();
  resetPeripherals();
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
  ledcAttachPin(LED_LOW_PIN, PWM_CHANNEL_LOW);
  // ledcAttachChannel(LED_LOW_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_LOW);

  ledcSetup(PWM_CHANNEL_HIGH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LED_HIGH_PIN, PWM_CHANNEL_HIGH);
  // ledcAttachChannel(LED_HIGH_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_HIGH);

  pinMode(POT_PIN, INPUT);
  pinMode(POT_ENABLE, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize accelerometer
#ifdef ACC
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

  // Initialize GPS
#ifdef GPS
  initGPS();
#endif

}

void resetPeripherals()
{
  digitalWrite(POT_ENABLE, HIGH); // Reversed, turn off pot
  ws2812fx.stop();                // Stop the LED effects
  ws2812fx.setBrightness(0);      // Turn off LED strip
  ws2812fx.strip_off();           // Ensure LEDs are off
  ledcWrite(PWM_CHANNEL_HIGH, 0);
  ledcWrite(PWM_CHANNEL_LOW, 0);
}

#ifdef GPS
void initGPS()
{
#ifdef DEBUG
  Serial.println("Initializing GPS...");
#endif

  // Initialize LittleFS (without auto-format to preserve existing rides)
  if (!LittleFS.begin(false)) {
    // Try formatting only if mount fails
    if (!LittleFS.begin(true)) {
#ifdef DEBUG
      Serial.println("LittleFS format failed");
#endif
      return;
    }
#ifdef DEBUG
    Serial.println("LittleFS mounted successfully after format");
#endif
  } else {
#ifdef DEBUG
    Serial.println("LittleFS mounted successfully");
#endif
  }

  // Initialize GPS serial
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Configure GPS for 10Hz update rate
  // ATGM336H-6N-6N configuration commands via UBX protocol
  // Set update rate to 100ms (10Hz)
  const uint8_t cfgRate10Hz[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00,
    0x01, 0x00, 0x01, 0x00, 0x00, 0x00
  };
  gpsSerial.write(cfgRate10Hz, sizeof(cfgRate10Hz));
  gpsSerial.flush();
  delay(100);

#ifdef DEBUG
  Serial.println("GPS initialized at 10Hz");
#endif
}

void startGPSLogging()
{
  if (isGPSLogging) {
#ifdef DEBUG
    Serial.println("GPS logging already running");
#endif
    return;
  }

  // Get the oldest ride index (circular buffer)
  currentRideIndex = getOldestRideIndex();

  // Reset buffer for this ride
  rideCounts[currentRideIndex] = 0;
  isGPSLogging = true;
  lastGPSLogTime = millis();

#ifdef DEBUG
  Serial.print("Starting GPS logging on ride ");
  Serial.println(currentRideIndex);
#endif
}

void stopGPSLogging()
{
  if (!isGPSLogging) {
#ifdef DEBUG
    Serial.println("GPS logging not running");
#endif
    return;
  }

  isGPSLogging = false;

  // Save the ride to filesystem
  if (saveRideToFS(currentRideIndex)) {
#ifdef DEBUG
    Serial.print("Ride ");
    Serial.print(currentRideIndex);
    Serial.println(" saved successfully");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Failed to save ride");
#endif
  }
}

uint8_t getOldestRideIndex()
{
  // Read index from file to determine oldest ride
  File file = LittleFS.open("/rides/index.txt", "r");
  if (file && file.available()) {
    String indexStr = file.readString();
    file.close();
    // Validate: index should be a single digit 0-9
    if (indexStr.length() >= 1) {
      char c = indexStr.charAt(0);
      if (c >= '0' && c <= '9') {
        return c - '0';
      }
    }
  }
  // Default to 0 if no index file or invalid content
  return 0;
}

bool saveRideToFS(uint8_t rideIndex)
{
  // Create rides directory if it doesn't exist
  if (!LittleFS.exists("/rides")) {
    LittleFS.mkdir("/rides");
  }

  // Build filename
  char filename[32];
  snprintf(filename, sizeof(filename), "/rides/ride_%03d.csv", rideIndex);

  File file = LittleFS.open(filename, "w");
  if (!file) {
#ifdef DEBUG
    Serial.print("Failed to open ");
    Serial.println(filename);
#endif
    return false;
  }

  // Write CSV header
  file.println("timestamp,lat,lon,speed_kmh,alt_m,sats,hdop,age");

  // Write data points
  for (uint16_t i = 0; i < rideCounts[rideIndex]; i++) {
    GPSPoint p = rideBuffer[rideIndex][i];
    file.print(p.timestamp);
    file.print(",");
    file.print(p.lat, 7);
    file.print(",");
    file.print(p.lon, 7);
    file.print(",");
    file.print(p.speed_kmh, 1);
    file.print(",");
    file.print(p.alt_m, 1);
    file.print(",");
    file.print(p.sats);
    file.print(",");
    file.print(p.hdop, 1);
    file.print(",");
    file.println(p.age);
  }

  file.close();

  // Update index file with next ride index
  File indexFile = LittleFS.open("/rides/index.txt", "w");
  if (indexFile) {
    uint8_t nextRide = (rideIndex + 1) % MAX_RIDES;
    indexFile.print(nextRide);
    indexFile.close();
  }

#ifdef DEBUG
  Serial.print("Saved ");
  Serial.print(rideCounts[rideIndex]);
  Serial.println(" points to ");
  Serial.println(filename);
#endif

  return true;
}

void processGPSData()
{
  if (!isGPSLogging) return;

  // Read all available GPS data
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // Check if we have a valid fix and it's time to log
  // Only log if location is updated AND we have valid data for key fields
  if (gps.location.isUpdated() &&
      gps.location.isValid() &&
      (millis() - lastGPSLogTime >= GPS_UPDATE_INTERVAL_MS)) {
    uint16_t pos = rideCounts[currentRideIndex];

    if (pos < POINTS_PER_RIDE) {
      GPSPoint &p = rideBuffer[currentRideIndex][pos];

      p.timestamp = millis();
      p.lat = gps.location.lat();
      p.lon = gps.location.lng();
      // Only use speed if valid, otherwise 0
      p.speed_kmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
      // Only use altitude if valid, otherwise 0
      p.alt_m = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
      // Only use satellites if valid
      p.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
      // Only use HDOP if valid
      p.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;
      p.age = gps.age();

      rideCounts[currentRideIndex]++;
      lastGPSLogTime = millis();
    } else {
      // Buffer full - stop logging silently
      // User must manually stop via long press to save current data
      isGPSLogging = false;
#ifdef DEBUG
      Serial.println("GPS buffer full, logging stopped");
#endif
    }
  }
}
#endif
    }
  }
}
#endif

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

#ifdef GPS
  if (isGPSLogging)
  {
    stopGPSLogging();
    state = false;
    resetPeripherals();
    return;
  }
#endif

  if (!state)
  {
    state = true;
#ifdef GPS
    startGPSLogging();
#endif
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

#ifdef GPS
  processGPSData();
#endif
}
