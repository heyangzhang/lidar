#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// -------------------- USER SETTINGS --------------------

// Pick a HW serial port that you wired to the lidar.
// GIGA has multiple UARTs; change this if you wired a different one.
#define LIDAR_SERIAL Serial1

// WS2812B strip/ring
static const uint8_t  LED_PIN   = 10;     // <-- change to your chosen data pin
static const uint16_t NUM_LEDS  = 36;    // <-- set to your strip length

// Distance-to-color mapping
static const uint16_t NEAR_DIST_MM = 50;     // C1 blind range ~50mm :contentReference[oaicite:3]{index=3}
static const uint16_t FAR_DIST_MM  = 3000;   // choose what "far" means for your effect

// How often to print debug (ms). Set to 0 to disable.
static const uint32_t DEBUG_EVERY_MS = 250;

// -------------------- PROTOCOL CONSTANTS --------------------
// SLAMTEC SC-series protocol:
// Start scan request: A5 20
// Response descriptor starts with A5 5A, then 5 bytes descriptor.
// Standard scan node is 5 bytes. :contentReference[oaicite:4]{index=4}
static const uint8_t  RPLIDAR_REQ_START_FLAG = 0xA5;
static const uint8_t  RPLIDAR_CMD_SCAN       = 0x20;
static const uint8_t  RPLIDAR_CMD_STOP       = 0x25;

static const uint32_t C1_BAUD = 460800;  // C1 typical UART baud :contentReference[oaicite:5]{index=5}

// -------------------- LED OBJECT --------------------
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Per-revolution bins: store closest distance for each LED sector
static uint16_t minDist[NUM_LEDS];

// Track whether we’ve started receiving scan nodes yet
static bool scanning = false;

// For debug throttling
static uint32_t lastDebugMs = 0;

// -------------------- UTIL --------------------
static inline void resetBins() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) minDist[i] = 0xFFFF;
}

static inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Map distance to smooth red->green gradient (near=red, far=green)
static void distanceToColor(uint16_t d, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (d == 0 || d == 0xFFFF) { r = g = b = 0; return; }

  d = clampU16(d, NEAR_DIST_MM, FAR_DIST_MM);
  float t = (float)(d - NEAR_DIST_MM) / (float)(FAR_DIST_MM - NEAR_DIST_MM);
  if (t < 0) t = 0;
  if (t > 1) t = 1;

  r = (uint8_t)((1.0f - t) * 255.0f);
  g = (uint8_t)(t * 255.0f);
  b = 0;
}

static void renderBinsToLEDs() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    uint8_t r, g, b;
    distanceToColor(minDist[i], r, g, b);
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

// Read exactly N bytes with a timeout (ms). Returns true if successful.
static bool readBytesTimeout(HardwareSerial &ser, uint8_t *buf, size_t n, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t got = 0;
  while (got < n && (millis() - start) < timeoutMs) {
    while (ser.available() && got < n) {
      buf[got++] = (uint8_t)ser.read();
    }
    yield();
  }
  return (got == n);
}

// Send STOP
static void rplidarStop() {
  uint8_t cmd[2] = { RPLIDAR_REQ_START_FLAG, RPLIDAR_CMD_STOP };
  LIDAR_SERIAL.write(cmd, 2);
  delay(10);
  while (LIDAR_SERIAL.available()) LIDAR_SERIAL.read(); // flush
}

// Send SCAN and consume response descriptor (7 bytes total: A5 5A + 5 bytes)
static bool rplidarStartScan() {
  rplidarStop();

  uint8_t cmd[2] = { RPLIDAR_REQ_START_FLAG, RPLIDAR_CMD_SCAN };
  LIDAR_SERIAL.write(cmd, 2);

  // Find descriptor header A5 5A
  uint8_t b = 0;
  uint32_t start = millis();
  bool gotA5 = false;

  while ((millis() - start) < 1000) {
    if (!LIDAR_SERIAL.available()) { yield(); continue; }
    b = (uint8_t)LIDAR_SERIAL.read();
    if (!gotA5) {
      if (b == 0xA5) gotA5 = true;
    } else {
      if (b == 0x5A) {
        // read remaining 5 bytes of descriptor
        uint8_t desc[5];
        if (!readBytesTimeout(LIDAR_SERIAL, desc, 5, 250)) return false;
        return true;
      }
      gotA5 = (b == 0xA5);
    }
  }
  return false;
}

// Decode a single 5-byte measurement node (standard scan). :contentReference[oaicite:6]{index=6}
// Returns true if valid and outputs angle (deg) and dist (mm).
static bool decodeNode(const uint8_t n[5], float &angleDeg, uint16_t &distMm, uint8_t &quality, bool &startOfScan) {
  // Byte0: [quality:6 bits][Sbar][S]
  uint8_t b0 = n[0];
  bool s  = (b0 & 0x01) != 0;
  bool sb = (b0 & 0x02) != 0;  // inverted start flag
  // Valid node requires s XOR sb == 1
  if (((uint8_t)s ^ (uint8_t)sb) != 1) return false;

  quality = (b0 >> 2);
  startOfScan = s;

  // Byte1: [angle_q6 low 7 bits][C]
  uint8_t b1 = n[1];
  bool c = (b1 & 0x01) != 0;
  if (!c) return false; // check bit should be 1 in standard nodes

  uint16_t angle_q6 = ((uint16_t)n[2] << 7) | (uint16_t)(b1 >> 1);
  angleDeg = (float)angle_q6 / 64.0f;

  // distance_q2 little-endian in bytes 3..4 (fixed point with 2 fractional bits)
  uint16_t dist_q2 = (uint16_t)n[3] | ((uint16_t)n[4] << 8);
  distMm = dist_q2 / 4; // convert to mm

  return true;
}

// Map angle (0..360) to LED index
static inline uint16_t angleToLed(float angleDeg) {
  while (angleDeg < 0) angleDeg += 360.0f;
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  float t = angleDeg / 360.0f;
  uint16_t idx = (uint16_t)(t * NUM_LEDS);
  if (idx >= NUM_LEDS) idx = NUM_LEDS - 1;
  return idx;
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  strip.begin();
  strip.show(); // all off

  resetBins();

  LIDAR_SERIAL.begin(C1_BAUD);

  Serial.println("Starting RPLIDAR C1 (standard SCAN)...");
  if (!rplidarStartScan()) {
    Serial.println("ERROR: Failed to enter SCAN mode (no descriptor). Check wiring/baud/power.");
  } else {
    Serial.println("SCAN started.");
    scanning = true;
  }
}

void loop() {
  if (!scanning) {
    delay(200);
    if (rplidarStartScan()) {
      Serial.println("SCAN started.");
      scanning = true;
    }
    return;
  }

  // Read nodes as they arrive; update bins; on start-of-scan, render previous rev.
  while (LIDAR_SERIAL.available() >= 5) {
    uint8_t node[5];
    for (int i = 0; i < 5; i++) node[i] = (uint8_t)LIDAR_SERIAL.read();

    float angleDeg;
    uint16_t distMm;
    uint8_t quality;
    bool startOfScan;

    if (!decodeNode(node, angleDeg, distMm, quality, startOfScan)) {
      continue; // drop invalid nodes
    }

    // When a new scan starts, render what we collected from the previous scan
    if (startOfScan) {
      renderBinsToLEDs();
      resetBins();
    }

    // Bin this point into one LED sector and keep closest distance
    uint16_t led = angleToLed(angleDeg);

    if (distMm > 0 && distMm <= FAR_DIST_MM) {
      if (distMm < minDist[led]) minDist[led] = distMm;
    }

    // Optional debug
    if (DEBUG_EVERY_MS > 0 && (millis() - lastDebugMs) >= DEBUG_EVERY_MS) {
      lastDebugMs = millis();
      Serial.print("angle=");
      Serial.print(angleDeg, 2);
      Serial.print(" deg, dist=");
      Serial.print(distMm);
      Serial.print(" mm, q=");
      Serial.print(quality);
      Serial.print(", led=");
      Serial.println(led);
    }
  }
}
