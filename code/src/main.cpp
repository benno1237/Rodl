#include <Arduino.h>
#include <EasyButton.h>
#include <Preferences.h> // Include Preferences library for NVS storage
#include <TinyGPS++.h>   // GPS parsing library
#include <HardwareSerial.h>


#define DEBUG
// Uncomment to trace every GPS notification ([BLE tx] gps #...) on serial.
//#define DEBUG_GPS_STREAM
#define ACC
#define GPS
#define BLE

#ifdef ACC
#include <Wire.h>    // Include the Wire library
#include "LIS3DHTR.h" // Include the LIS3DHTR library
#endif

#ifdef BLE
#include <NimBLEDevice.h>
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
constexpr uint8_t GPS_RX_PIN = 18;       // GPS RX (ESP32 TX)
constexpr uint8_t GPS_TX_PIN = 17;       // GPS TX (ESP32 RX)

// LED Strip Settings
constexpr uint8_t LED_PIN = 12;          // WS2815 LED control pin
constexpr uint8_t LED_NUM = 10;          // LED amount
uint16_t LED_SPEED = 500;    // LED update speed

// GPS Settings
constexpr uint32_t GPS_BAUDRATE = 115200;
constexpr uint16_t GPS_UPDATE_INTERVAL_MS = 100;  // 10Hz update rate
constexpr uint16_t GPS_STATUS_INTERVAL_MS = 1000; // idle heartbeat when connected

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
  float accel_x;            // accelerometer X
  float accel_y;            // accelerometer Y
  float accel_z;            // accelerometer Z
};

// GPS Variables
#ifdef GPS
bool isGPSLogging = false;
uint8_t currentRideIndex = 0;
unsigned long lastGPSLogTime = 0;
unsigned long lastGPSStatsPrint = 0;
constexpr uint16_t GPS_STATS_PRINT_INTERVAL_MS = 500;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // UART2 for GPS
#endif

// BLE Variables
#ifdef BLE
#define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define BLE_SETTINGS_UUID "12345678-1234-1234-1234-123456789abd"
#define BLE_GPS_DATA_UUID "12345678-1234-1234-1234-123456789abe"
#define BLE_COMMAND_UUID "12345678-1234-1234-1234-123456789abf"
#define BLE_NAME_UUID "12345678-1234-1234-1234-1234567890bf"
// Short 16-bit marker service uuid carried in the advertisement payload
// (the full 128-bit service uuid does not fit alongside name + flags).
// Service data under this uuid is a single flag byte: 0x01 = in use.
#define RODL_MARKER_UUID 0x524C
constexpr size_t BLE_MAX_NAME_LEN = 20;

// Command bytes written by the app to the command characteristic
enum BleCommand : uint8_t {
  CMD_START_LOG = 0x01, // power on + start GPS logging
  CMD_STOP_LOG = 0x02,  // stop GPS logging (lights stay on)
  CMD_POWER_ON = 0x03,  // power on lights, no logging
  CMD_POWER_OFF = 0x04, // power off lights + stop logging
  CMD_TIME_SYNC = 0x05, // respond with a status packet (contains millis())
  CMD_GET_STATUS = 0x06 // respond with a status packet
};

NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr;
NimBLECharacteristic* pSettingsCharacteristic = nullptr;
NimBLECharacteristic* pGPSDataCharacteristic = nullptr;
NimBLECharacteristic* pCommandCharacteristic = nullptr;
NimBLECharacteristic* pNameCharacteristic = nullptr;
bool bleConnected = false;
// Display name == advertised Bluetooth name, persisted in NVS ("NAME").
// Empty until loadConfiguration() runs.
String bleDeviceName;
// Set when a rename should reboot the board (millis deadline, 0 = none).
// Written from the NimBLE task, consumed in loop() so the GATT write
// response reaches the app before we restart.
volatile unsigned long renameRebootAt = 0;
// Debounced settings persist: slider drags arrive as a burst of writes, and
// committing NVS inside the NimBLE callback would stall the BLE task (and
// wear flash). The values are applied immediately; the save happens once the
// stream has been quiet for SETTINGS_SAVE_DELAY_MS.
constexpr uint32_t SETTINGS_SAVE_DELAY_MS = 2000;
volatile bool settingsSavePending = false;
volatile unsigned long settingsSaveAt = 0;

// Ring buffer of GPS fixes waiting for BLE delivery. Fixes generated while no
// phone is connected are kept here (512 * 10Hz = ~51 s) and backfilled in
// order after a reconnect; oldest entries are dropped when it is full.
// Cleared when a new ride starts so stale points cannot leak into it.
struct GPSRingEntry {
  GPSPoint point;
  uint8_t rideIndex;
};
constexpr uint16_t GPS_RING_SIZE = 512;
constexpr uint32_t GPS_RING_SEND_INTERVAL_MS = 10; // paced backfill (~40 points/s)
constexpr uint32_t GPS_RING_RESUME_DELAY_MS = 400; // let the phone finish CCCD subscribe
GPSRingEntry gpsRing[GPS_RING_SIZE];
uint16_t gpsRingHead = 0; // next slot to write
uint16_t gpsRingTail = 0;  // next slot to read
volatile bool gpsRingClearReq = false; // set on new ride, consumed in loop()
uint32_t lastRingSendTime = 0;
uint32_t bleConnectTime = 0;
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
void powerOn(bool withLogging);
void powerOff();
#ifdef ACC
void checkOrientation();
#endif
#ifdef GPS
void initGPS();
void startGPSLogging();
void stopGPSLogging();
void processGPSData();
#endif
#ifdef BLE
void initBLE();
void startBLE();
void stopBLE();
void updateAdvertising(bool inUse);
void applySettings();
void sendSettingsResponse();
void sendStatus();
void sendGPSPoint(const GPSPoint& point, uint8_t rideIndex);
void enqueueGPSPoint(const GPSPoint& point, uint8_t rideIndex);
bool dequeueGPSPoint(GPSRingEntry& out);
void drainGPSRing();
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
  // BLE is always available so the app can reach a powered-down sled
  // (remote power on/off, start/stop logging).
  initBLE();
  startBLE();
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
    // Logging never stopped (the previous link died before STOP_LOG could be
    // sent): treat this as a fresh ride so undelivered points from the old
    // one are cleared instead of leaking into the new recording.
    currentRideIndex++;
#ifdef BLE
    gpsRingClearReq = true;
#endif
#ifdef DEBUG
    Serial.print("Restarting GPS logging with fresh ride ");
    Serial.println(currentRideIndex);
#endif
    return;
  }

  currentRideIndex++;
  isGPSLogging = true;
  lastGPSLogTime = millis();
#ifdef BLE
  gpsRingClearReq = true; // new ride: drop any stale undelivered points
#endif

#ifdef DEBUG
  Serial.print("Starting GPS logging, ride ");
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
}

#ifdef BLE
// Append a fix to the delivery ring. Called from loop() only; when the ring
// is full the oldest point is dropped.
void enqueueGPSPoint(const GPSPoint& point, uint8_t rideIndex)
{
  uint16_t next = (uint16_t)((gpsRingHead + 1) % GPS_RING_SIZE);
  if (next == gpsRingTail) {
    gpsRingTail = (uint16_t)((gpsRingTail + 1) % GPS_RING_SIZE); // full
#ifdef DEBUG
    static uint32_t dropCount = 0;
    dropCount++;
    if (dropCount == 1 || dropCount % 100 == 0) {
      Serial.printf("[BLE ring] full - dropping oldest (total %lu)\n",
                    (unsigned long)dropCount);
    }
#endif
  }
  gpsRing[gpsRingHead].point = point;
  gpsRing[gpsRingHead].rideIndex = rideIndex;
  gpsRingHead = next;
}

bool dequeueGPSPoint(GPSRingEntry& out)
{
  if (gpsRingTail == gpsRingHead) return false;
  out = gpsRing[gpsRingTail];
  gpsRingTail = (uint16_t)((gpsRingTail + 1) % GPS_RING_SIZE);
  return true;
}
#endif

void processGPSData()
{
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

#ifdef BLE
  // While logging we record every fix. While idle but connected we still
  // push a slow heartbeat so the app can show fix status ("No GPS fix")
  // and gate "Start Recording" before a ride starts.
  const bool heartbeat = !isGPSLogging && bleConnected;
#else
  const bool heartbeat = false;
#endif
  if (!isGPSLogging && !heartbeat) return;

  const uint32_t minInterval =
      isGPSLogging ? GPS_UPDATE_INTERVAL_MS : GPS_STATUS_INTERVAL_MS;

  if (gps.location.isUpdated() &&
      gps.location.isValid() &&
      (millis() - lastGPSLogTime >= minInterval)) {
    
    GPSPoint p;
    p.timestamp = millis();
    p.lat = gps.location.lat();
    p.lon = gps.location.lng();
    p.speed_kmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
    p.alt_m = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    p.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    p.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;
    p.age = gps.location.age();

#ifdef ACC
    p.accel_x = lis.getAccelerationX();
    p.accel_y = lis.getAccelerationY();
    p.accel_z = lis.getAccelerationZ();
#else
    p.accel_x = 0;
    p.accel_y = 0;
    p.accel_z = 0;
#endif

    lastGPSLogTime = millis();

#ifdef BLE
    if (isGPSLogging) {
      // Always go through the ring: delivery (live or backfilled after a
      // reconnect) happens paced from loop(), preserving order.
      enqueueGPSPoint(p, currentRideIndex);
    } else {
      // Idle heartbeat: live delivery only, never part of a ride.
      sendGPSPoint(p, currentRideIndex);
    }
#endif
  }
}

void printGPSStats()
{
#ifdef DEBUG
  if (state && (millis() - lastGPSStatsPrint >= GPS_STATS_PRINT_INTERVAL_MS)) {
    lastGPSStatsPrint = millis();
    Serial.print("GPS: ");
    if (gps.satellites.isValid()) {
      Serial.print("Sats: ");
      Serial.print(gps.satellites.value());
    } else {
      Serial.print("Sats: --");
    }
    Serial.print(" | ");
    if (gps.location.isValid()) {
      Serial.print("Lat: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Lon: ");
      Serial.print(gps.location.lng(), 6);
    } else {
      Serial.print("No fix");
    }
    Serial.print(" | Speed: ");
    if (gps.speed.isValid()) {
      Serial.print(gps.speed.kmph(), 1);
    } else {
      Serial.print("--");
    }
    Serial.print(" km/h | Alt: ");
    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters(), 1);
    } else {
      Serial.print("--");
    }
    Serial.print("m | HDOP: ");
    if (gps.hdop.isValid()) {
      Serial.print(gps.hdop.hdop(), 1);
    } else {
      Serial.print("--");
    }
    Serial.print(" | Logging: ");
    Serial.println(isGPSLogging ? "YES" : "NO");
  }
#endif
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

  // Long press toggles the ride: on -> logging + lights, off -> everything off
  bool logging = false;
#ifdef GPS
  logging = isGPSLogging;
#endif
  if (state || logging)
  {
    powerOff();
  }
  else
  {
    powerOn(true);
  }
}

// Power on the sled. When [withLogging] is true GPS logging is started too
// (same behaviour as the physical long press).
void powerOn(bool withLogging)
{
  state = true;
#ifdef GPS
  if (withLogging)
  {
    startGPSLogging();
  }
#endif
  if (!orientationDisabled)
  {
    restorePeripherals();
  }
}

// Power off the sled; GPS logging is stopped as well.
void powerOff()
{
#ifdef GPS
  if (isGPSLogging)
  {
    stopGPSLogging();
  }
#endif
  state = false;
  resetPeripherals();
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
#ifdef BLE
  // Display name == advertised Bluetooth name. Falls back to the default
  // "RODL-XXXX" (last two MAC bytes) which marks the board as unclaimed.
  bleDeviceName = preferences.getString("NAME", "");
#endif
  preferences.end();

#ifdef BLE
  if (bleDeviceName.isEmpty()) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macStr[5];
    snprintf(macStr, sizeof(macStr), "%02X%02X", mac[4], mac[5]);
    bleDeviceName = "RODL-";
    bleDeviceName += macStr;
  }
#endif

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
#ifdef BLE
  preferences.putString("NAME", bleDeviceName);
#endif
  preferences.end();

#ifdef DEBUG
  Serial.println("Configuration saved.");
#endif
}

#ifdef BLE
class SettingsCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 0) return;

#ifdef DEBUG
    if (value.length() == 1 && value[0] == 0x01) {
      Serial.println("[BLE rx] settings request (01)");
    } else {
      Serial.printf("[BLE rx] settings (%u B):", (unsigned)value.length());
      for (size_t i = 0; i < value.length(); i++) {
        Serial.printf(" %02X", (uint8_t)value[i]);
      }
      Serial.println();
    }
#endif

    // Single byte 0x01 = "send me your current settings".
    // Anything else must be a full 15 byte settings payload (a payload that
    // merely starts with 0x01, e.g. a color, must not be mistaken for it).
    if (value.length() == 1 && value[0] == 0x01) {
      sendSettingsResponse();
      return;
    }

    if (value.length() < 15) {
#ifdef DEBUG
      Serial.println("Invalid settings data length");
#endif
      return;
    }

    currentColor = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | value[3];
    currentBrightness = value[4];
    currentEffect = value[5];
    LED_SPEED = (value[6] << 8) | value[7];
    LONG_PRESS_DURATION = (value[8] << 8) | value[9];
    SIDEWAYS_THRESHOLD_TIME = (value[10] << 8) | value[11];
    SIDEWAYS_ACCEL_THRESHOLD = value[12] / 10.0;
    ACCEL_AUTO_SHUTDOWN = value[13] == 1;
    ACCEL_AUTO_STARTUP = value[14] == 1;

    // Apply now (LEDs follow the slider live); persist after the stream has
    // been quiet for a moment - see SETTINGS_SAVE_DELAY_MS.
    settingsSaveAt = millis() + SETTINGS_SAVE_DELAY_MS;
    settingsSavePending = true;
    applySettings();

#ifdef DEBUG
    Serial.println("Settings received via BLE");
#endif
  }
};

class ServerCallback : public NimBLEServerCallbacks {
  // NOTE: NimBLE invokes both the 1-arg and 2-arg overloads; all logic lives
  // here in the 2-arg variant only.
  void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
    if (bleConnected) {
      // Single-connection lock: kick any additional central while the sled
      // is in use by someone else.
      pServer->disconnect(desc->conn_handle);
#ifdef DEBUG
      Serial.println("BLE kicked second central");
#endif
      return;
    }
    bleConnected = true;
    bleConnectTime = millis();
    // Keep advertising while connected so other users can see the in-use flag.
    updateAdvertising(true);
#ifdef DEBUG
    Serial.println("BLE client connected");
#endif
  }

  void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
    (void)pServer;
    (void)desc;
    bleConnected = false;
    // Rebuild the advertisement (clears in-use, picks up a renamed device)
    // and restart it; advertiseOnDisconnect(true) is a safety net.
    updateAdvertising(false);
#ifdef DEBUG
    Serial.println("BLE client disconnected");
#endif
  }
};

class CommandCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 0) return;

#ifdef DEBUG
    Serial.printf("[BLE rx] command: 0x%02X", (uint8_t)value[0]);
    if (value.length() > 1) {
      Serial.printf(" (+%u B)", (unsigned)(value.length() - 1));
    }
    Serial.println();
#endif

    switch ((uint8_t)value[0]) {
      case CMD_START_LOG:
        powerOn(true);
#ifdef DEBUG
        Serial.println("BLE command: start logging");
#endif
        break;
      case CMD_STOP_LOG:
#ifdef GPS
        stopGPSLogging();
#endif
#ifdef DEBUG
        Serial.println("BLE command: stop logging");
#endif
        break;
      case CMD_POWER_ON:
        powerOn(false);
#ifdef DEBUG
        Serial.println("BLE command: power on");
#endif
        break;
      case CMD_POWER_OFF:
        powerOff();
#ifdef DEBUG
        Serial.println("BLE command: power off");
#endif
        break;
      case CMD_TIME_SYNC:
      case CMD_GET_STATUS:
        sendStatus();
        break;
      default:
#ifdef DEBUG
        Serial.println("BLE command: unknown");
#endif
        break;
    }
  }
};

// Rename characteristic: app writes a new display name, board persists it to
// NVS and restarts advertising under the new name.
class NameCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    String newName;
    for (size_t i = 0; i < value.length() && newName.length() < BLE_MAX_NAME_LEN; i++) {
      char c = value[i];
      if (c >= 32 && c <= 126) newName += c; // printable ASCII only
    }
    newName.trim();
    if (newName.length() == 0) return;

#ifdef DEBUG
    Serial.printf("[BLE rx] name (%u B): \"%s\"\n", (unsigned)value.length(),
                  newName.c_str());
#endif

    bleDeviceName = newName;
    NimBLEDevice::setDeviceName(bleDeviceName.c_str());
    saveConfiguration();
    if (pNameCharacteristic != nullptr) {
      pNameCharacteristic->setValue(bleDeviceName.c_str());
      pNameCharacteristic->notify();
    }
    // Rebuild the live advertisement so the new name (and in-use flag) show
    // up immediately.
    updateAdvertising(bleConnected);
    // Reboot shortly after: the fresh boot + advertisement force every
    // cached name (board and phone) to pick up the new one, and the app
    // reconnects on its own. The 500 ms delay lets the write response and
    // name cache update land first. Skipped while logging so a ride is
    // never lost.
    if (!isGPSLogging) {
      renameRebootAt = millis() + 500;
#ifdef DEBUG
      Serial.println("[BLE] rename applied - reboot in 500 ms, fresh advert under new name");
#endif
    } else {
#ifdef DEBUG
      Serial.println("[BLE] rename applied - reboot deferred (logging a ride)");
#endif
    }
#ifdef DEBUG
    Serial.println("BLE name set to: " + bleDeviceName);
#endif
  }
};

// Status packet: [flags, rideIndex, millis0..millis3]
// flags: bit0 = power state, bit1 = GPS logging active
void sendStatus() {
  if (pCommandCharacteristic == nullptr) return;

  uint8_t flags = 0;
  if (state) flags |= 0x01;
#ifdef GPS
  if (isGPSLogging) flags |= 0x02;
#endif

  uint32_t ms = (uint32_t)millis();
  uint8_t data[6];
  data[0] = flags;
#ifdef GPS
  data[1] = currentRideIndex;
#else
  data[1] = 0;
#endif
  data[2] = (ms >> 24) & 0xFF;
  data[3] = (ms >> 16) & 0xFF;
  data[4] = (ms >> 8) & 0xFF;
  data[5] = ms & 0xFF;

  pCommandCharacteristic->setValue(data, sizeof(data));
  pCommandCharacteristic->notify();

#ifdef DEBUG
  Serial.printf("[BLE tx] status: %02X %02X %02X %02X %02X %02X"
                " (power=%s, logging=%s, ride=%u, millis=%lu)\n",
                data[0], data[1], data[2], data[3], data[4], data[5],
                (flags & 0x01) ? "on" : "off", (flags & 0x02) ? "on" : "off",
                (unsigned)data[1], (unsigned long)ms);
#endif
}

void initBLE() {
  // bleDeviceName was loaded (or defaulted) by loadConfiguration().
  NimBLEDevice::init(bleDeviceName.c_str());
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(512); // GPS packets are 48 bytes
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallback());
  pServer->advertiseOnDisconnect(true);
  
  pService = pServer->createService(BLE_SERVICE_UUID);
  pSettingsCharacteristic = pService->createCharacteristic(
    BLE_SETTINGS_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );
  pSettingsCharacteristic->setCallbacks(new SettingsCallback());
  
  pGPSDataCharacteristic = pService->createCharacteristic(
    BLE_GPS_DATA_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  pCommandCharacteristic = pService->createCharacteristic(
    BLE_COMMAND_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );
  pCommandCharacteristic->setCallbacks(new CommandCallback());

  pNameCharacteristic = pService->createCharacteristic(
    BLE_NAME_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );
  pNameCharacteristic->setCallbacks(new NameCallback());
  pNameCharacteristic->setValue(bleDeviceName.c_str());
  
  if (!pService->start()) {
#ifdef DEBUG
    // A failed start leaves the GATT table without our service: the phone
    // can connect but will not find any Rodl characteristics.
    Serial.println("BLE ERROR: service start failed");
#endif
  }
}

void startBLE() {
  if (pServer == nullptr) {
    initBLE();
  }
  updateAdvertising(false);
#ifdef DEBUG
  Serial.println("BLE started advertising as \"" + bleDeviceName + "\"");
#endif
}

// Rebuilds the advertisement payload and (re)starts advertising.
//
// adv packet:      flags + 16-bit marker uuid (0x524C) + service data
//                  (1 byte: in-use flag)
// scan response:   complete name (display name, <= BLE_MAX_NAME_LEN)
void updateAdvertising(bool inUse) {
  if (pServer == nullptr) return;
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();

  if (pAdvertising->isAdvertising()) {
    pAdvertising->stop();
  }

  NimBLEUUID marker((uint16_t)RODL_MARKER_UUID);

  NimBLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setCompleteServices(marker);
  advData.setServiceData(marker, std::string(1, inUse ? 0x01 : 0x00));
  pAdvertising->setAdvertisementData(advData);

  NimBLEAdvertisementData scanData;
  scanData.setName(bleDeviceName.c_str());
  pAdvertising->setScanResponseData(scanData);
  pAdvertising->setScanResponse(true);

  if (!pAdvertising->start()) {
#ifdef DEBUG
    // Not fatal: if the controller refuses advertising while a connection is
    // open the sled is simply invisible to others until it disconnects - the
    // single-connection kick still enforces the lock.
    Serial.println("BLE advertising start failed (in-use=" + String(inUse ? 1 : 0) + ")");
#endif
  }
}

void stopBLE() {
  NimBLEDevice::stopAdvertising();
  if (pServer) {
    pServer->disconnect(0);
  }
#ifdef DEBUG
  Serial.println("BLE stopped");
#endif
}

void applySettings() {
  ws2812fx.setBrightness(currentBrightness);
  ws2812fx.setColor(currentColor);
  ws2812fx.setSpeed(LED_SPEED);
  ws2812fx.setMode(effects[currentEffect]);
  ws2812fx.start();
}

void sendSettingsResponse() {
  if (pSettingsCharacteristic == nullptr) return;

  uint8_t data[15];
  data[0] = (currentColor >> 24) & 0xFF;
  data[1] = (currentColor >> 16) & 0xFF;
  data[2] = (currentColor >> 8) & 0xFF;
  data[3] = currentColor & 0xFF;
  data[4] = currentBrightness;
  data[5] = currentEffect;
  data[6] = (LED_SPEED >> 8) & 0xFF;
  data[7] = LED_SPEED & 0xFF;
  data[8] = (LONG_PRESS_DURATION >> 8) & 0xFF;
  data[9] = LONG_PRESS_DURATION & 0xFF;
  data[10] = (SIDEWAYS_THRESHOLD_TIME >> 8) & 0xFF;
  data[11] = SIDEWAYS_THRESHOLD_TIME & 0xFF;
  data[12] = (uint8_t)(SIDEWAYS_ACCEL_THRESHOLD * 10);
  data[13] = ACCEL_AUTO_SHUTDOWN ? 1 : 0;
  data[14] = ACCEL_AUTO_STARTUP ? 1 : 0;

  pSettingsCharacteristic->setValue(data, 15);
  pSettingsCharacteristic->notify();

#ifdef DEBUG
  Serial.printf("[BLE tx] settings (%u B):", (unsigned)sizeof(data));
  for (size_t i = 0; i < sizeof(data); i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
#endif
}

void sendGPSPoint(const GPSPoint& point, uint8_t rideIndex) {
  if (pGPSDataCharacteristic == nullptr || !bleConnected) return;

  uint8_t data[48];
  data[0] = rideIndex;
  data[1] = (point.timestamp >> 24) & 0xFF;
  data[2] = (point.timestamp >> 16) & 0xFF;
  data[3] = (point.timestamp >> 8) & 0xFF;
  data[4] = point.timestamp & 0xFF;
  
  memcpy(&data[5], &point.lat, sizeof(double));
  memcpy(&data[13], &point.lon, sizeof(double));
  memcpy(&data[21], &point.speed_kmh, sizeof(float));
  memcpy(&data[25], &point.alt_m, sizeof(float));
  data[29] = point.sats;
  memcpy(&data[30], &point.hdop, sizeof(float));
  data[34] = (point.age >> 8) & 0xFF;
  data[35] = point.age & 0xFF;
  memcpy(&data[36], &point.accel_x, sizeof(float));
  memcpy(&data[40], &point.accel_y, sizeof(float));
  memcpy(&data[44], &point.accel_z, sizeof(float));

  pGPSDataCharacteristic->setValue(data, 48);
  pGPSDataCharacteristic->notify();

#ifdef DEBUG_GPS_STREAM
  static uint32_t gpsTxSeq = 0;
  gpsTxSeq++;
  Serial.printf("[BLE tx] gps #%lu (%s): ride=%u ts=%lu lat=%.6f lon=%.6f "
                "spd=%.1f alt=%.0f sats=%u hdop=%.1f\n",
                (unsigned long)gpsTxSeq, isGPSLogging ? "log" : "hb",
                (unsigned)rideIndex, (unsigned long)point.timestamp, point.lat,
                point.lon, point.speed_kmh, point.alt_m, (unsigned)point.sats,
                point.hdop);
#endif
}

// Deliver waiting fixes while connected, paced so the TX queue cannot be
// flooded during a backfill. Consumes any pending fresh-ride clear first.
// Called from loop().
void drainGPSRing()
{
  if (gpsRingClearReq) {
    gpsRingClearReq = false;
    gpsRingTail = gpsRingHead; // a new ride started: drop stale points
#ifdef DEBUG
    Serial.println("[BLE ring] cleared for fresh ride");
#endif
    return;
  }
  if (!bleConnected) return;
  // Notifications are useless until the phone has written the CCCD; give it
  // a moment after (re)connect before backfilling.
  if ((uint32_t)(millis() - bleConnectTime) < GPS_RING_RESUME_DELAY_MS) return;
#ifdef DEBUG
  // One line per (re)connect announcing the queued backfill.
  static uint32_t announcedFor = 0;
  if (announcedFor != bleConnectTime && gpsRingTail != gpsRingHead) {
    announcedFor = bleConnectTime;
    uint16_t pending = (uint16_t)((gpsRingHead + GPS_RING_SIZE - gpsRingTail) %
                                  GPS_RING_SIZE);
    Serial.printf("[BLE ring] backfill: %u points queued from the gap\n",
                  (unsigned)pending);
  }
#endif
  if ((uint32_t)(millis() - lastRingSendTime) < GPS_RING_SEND_INTERVAL_MS) return;
  GPSRingEntry entry;
  if (!dequeueGPSPoint(entry)) return;
  lastRingSendTime = millis();
  sendGPSPoint(entry.point, entry.rideIndex);
}
#endif

void loop()
{
  // Pending rename reboot (see NameCallback::onWrite).
  if (renameRebootAt != 0 && (long)(millis() - renameRebootAt) >= 0) {
    renameRebootAt = 0;
    if (settingsSavePending) { // do not lose a debounced save to the reboot
      settingsSavePending = false;
      saveConfiguration();
    }
    ESP.restart();
  }

  // Debounced settings persist (see SettingsCallback::onWrite).
  if (settingsSavePending && (long)(millis() - settingsSaveAt) >= 0) {
    settingsSavePending = false;
    saveConfiguration();
  }

  button.read();
  ws2812fx.service();

  if (state && !orientationDisabled)
  {
    // Serial.println("Pot value: " + String(readPotentiometer()));
    changeLight(readPotentiometer());
  }

#ifdef ACC
  if (ACCEL_AUTO_SHUTDOWN) {
    checkOrientation();
  }
#endif

#ifdef BLE
  drainGPSRing(); // before processGPSData so a fresh-ride clear cannot eat new fixes
#endif
#ifdef GPS
  processGPSData();
  printGPSStats();
#endif
}

// #include <Arduino.h>

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Hello, World!");
//   pinMode(4, OUTPUT);
//   pinMode(6, OUTPUT); 
//   digitalWrite(6, HIGH); // Turn on the potentiometer (reversed logic)
// }

// void loop() {
//   // Example: Blink an LED connected to pin 4
//   digitalWrite(6, HIGH);
//   delay(2000);
//   digitalWrite(6, LOW);
//   delay(2000);
// }
