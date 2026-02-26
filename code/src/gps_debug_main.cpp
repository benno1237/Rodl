#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

constexpr uint8_t GPS_RX_PIN = 18;
constexpr uint8_t GPS_TX_PIN = 17;
constexpr uint32_t GPS_BAUDRATE = 115200;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

unsigned long lastStatusPrint = 0;
constexpr uint32_t STATUS_PRINT_INTERVAL = 2000;

struct NmeaCounters {
  uint32_t gga = 0;
  uint32_t gsa = 0;
  uint32_t gsv = 0;
  uint32_t rmc = 0;
  uint32_t other = 0;
} nmea;

char lineBuf[128];
size_t lineLen = 0;

void countSentence(const char* line) {
  // line looks like: $GNGGA,... or $GPGGA,...
  if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) nmea.gga++;
  else if (strncmp(line, "$GNGSA", 6) == 0 || strncmp(line, "$GPGSA", 6) == 0) nmea.gsa++;
  else if (strncmp(line, "$GNGSV", 6) == 0 || strncmp(line, "$GPGSV", 6) == 0) nmea.gsv++;
  else if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) nmea.rmc++;
  else nmea.other++;
}

void printFixHealth() {
  Serial.println("\n=== Fix / Time Health ===");

  Serial.printf("Location valid: %s, updated: %s, age: %lums\n",
    gps.location.isValid() ? "yes" : "no",
    gps.location.isUpdated() ? "yes" : "no",
    gps.location.age());

  Serial.printf("Time valid: %s, updated: %s\n",
    gps.time.isValid() ? "yes" : "no",
    gps.time.isUpdated() ? "yes" : "no");

  Serial.printf("Date valid: %s, updated: %s\n",
    gps.date.isValid() ? "yes" : "no",
    gps.date.isUpdated() ? "yes" : "no");

  if (gps.satellites.isValid()) {
    Serial.printf("Satellites (in view/used-ish): %d\n", gps.satellites.value());
  } else {
    Serial.println("Satellites: invalid");
  }

  if (gps.hdop.isValid()) {
    Serial.printf("HDOP: %.1f (raw=%lu)\n", gps.hdop.hdop(), gps.hdop.value());
  } else {
    Serial.println("HDOP: invalid");
  }
}

void printGPSStatus() {
  Serial.println("\n=== GPS Location ===");
  if (gps.location.isValid()) {
    Serial.printf("Lat: %.6f\n", gps.location.lat());
    Serial.printf("Lon: %.6f\n", gps.location.lng());
    Serial.printf("Alt: %.1f m\n", gps.altitude.meters());
    Serial.printf("Speed: %.1f km/h\n", gps.speed.kmph());
    Serial.printf("Heading: %.1f deg\n", gps.course.deg());
  } else {
    Serial.println("No fix");
  }
}

void printSentenceStats() {
  Serial.println("\n=== NMEA Sentence Counters (since boot) ===");
  Serial.printf("GGA: %lu  GSA: %lu  GSV: %lu  RMC: %lu  Other: %lu\n",
    nmea.gga, nmea.gsa, nmea.gsv, nmea.rmc, nmea.other);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n\n========================================");
  Serial.println("GPS Debug Tool");
  Serial.println("========================================");

  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("GPS Serial initialized at %lu baud\n", GPS_BAUDRATE);
  Serial.printf("RX Pin: %d, TX Pin: %d\n", GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("========================================\n");
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();

    // Raw passthrough
    Serial.write(c);

    // Feed parser
    gps.encode(c);

    // Capture full NMEA lines for counting
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 6 && lineBuf[0] == '$') countSentence(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0; // overflow, reset
    }
  }

  if (millis() - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
    lastStatusPrint = millis();
    printFixHealth();
    printGPSStatus();
    printSentenceStats();
    Serial.println("\n---");
  }
}