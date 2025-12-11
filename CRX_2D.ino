/*
  WIRING:

  LIDAR TX   -> CPX Rx
  LIDAR GND  -> CPX GND
  LIDAR 5V   -> CPX VOUT
*/

#include <Adafruit_CircuitPlayground.h>

#define LIDAR_SERIAL Serial1

const uint8_t HEADER          = 0x54;
const uint8_t VERLEN_EXPECTED = 0x2C;
const uint8_t POINTS_PER_PACK = 12;
const uint8_t FRAME_SIZE      = 47;

uint8_t frameBuf[FRAME_SIZE];
uint8_t frameIndex = 0;

// ---- CRC table from LD19 docs ----
const uint8_t CrcTable[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
  0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
  0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
  0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
  0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
  0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
  0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
  0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
  0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
  0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
  0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
  0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
  0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
  0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
  0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
  0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
  0x7f, 0x32, 0xe5, 0xa8
};

uint8_t calcCRC8(const uint8_t *p, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc = CrcTable[(crc ^ *p++) & 0xFF];
  }
  return crc;
}

uint16_t readLE16(const uint8_t *buf) {
  return buf[0] | (buf[1] << 8);
}


const uint16_t NEAR_DIST_MM = 5;
const uint16_t FAR_DIST_MM  = 1000; 

// Per-frame closest distance per pixel (temp)
uint16_t newDist[10];

uint8_t curR[10];
uint8_t curG[10];
uint8_t curB[10];

int angleToPixelIndex(float angleDeg) {
  while (angleDeg < 0)   angleDeg += 360.0f;
  while (angleDeg >= 360) angleDeg -= 360.0f;
  int idx = (int)(angleDeg / 36.0f);   // 360 / 10 pixels
  if (idx < 0) idx = 0;
  if (idx > 9) idx = 9;
  idx = 9 - idx;
  return idx;
}

// Map distance to smooth RGB gradient from red (near) to green (far)
void distanceToColor(uint16_t d, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (d == 0 || d >= FAR_DIST_MM) {
    r = g = b = 0;
    return;
  }

  // Clamp distance to [NEAR_DIST_MM, FAR_DIST_MM]
  if (d < NEAR_DIST_MM) d = NEAR_DIST_MM;
  if (d > FAR_DIST_MM)  d = FAR_DIST_MM;

  // t = 0 (near)  -> red (255,0,0)
  // t = 1 (far)   -> green (0,255,0)
  float t = (float)(d - NEAR_DIST_MM) / (float)(FAR_DIST_MM - NEAR_DIST_MM);
  if (t < 0) t = 0;
  if (t > 1) t = 1;

  r = (uint8_t)((1.0f - t) * 255.0f);  // decreases with distance
  g = (uint8_t)(t * 255.0f);          // increases with distance
  b = 0;
}

void updateNeoPixelsFromDistances() {
  for (int i = 0; i < 10; i++) {
    if (newDist[i] == 0xFFFF) continue;

    uint8_t r, g, b;
    distanceToColor(newDist[i], r, g, b);

    // Only write to the LED if the color actually changed
    if (r != curR[i] || g != curG[i] || b != curB[i]) {
      curR[i] = r;
      curG[i] = g;
      curB[i] = b;
      CircuitPlayground.setPixelColor(i, r, g, b);

      Serial.print("Pixel ");
      Serial.print(i);
      Serial.print(" color updated to R=");
      Serial.print(r);
      Serial.print(" G=");
      Serial.print(g);
      Serial.print(" B=");
      Serial.print(b);
      Serial.print(" (dist=");
      Serial.print(newDist[i]);
      Serial.println(" mm)");
    }
  }
}

void parseFrame(const uint8_t *buf) {
  Serial.println("\n--- NEW FRAME ---");

  for (int i = 0; i < 10; i++) {
    newDist[i] = 0xFFFF;
  }

  uint16_t startAngleRaw = readLE16(&buf[4]);
  uint16_t endAngleRaw   = readLE16(&buf[42]);

  int16_t diffRaw = (int16_t)(endAngleRaw - startAngleRaw);
  if (diffRaw < -18000) diffRaw += 36000;
  if (diffRaw >  18000) diffRaw -= 36000;

  float stepRaw = diffRaw / (POINTS_PER_PACK - 1);

  for (uint8_t i = 0; i < POINTS_PER_PACK; i++) {
    uint8_t idx = 6 + i * 3;

    uint16_t dist = readLE16(&buf[idx]);    // mm
    uint8_t  intensity = buf[idx + 2];      // available if needed

    float angleRaw = startAngleRaw + stepRaw * i;
    float angleDeg = angleRaw / 100.0f;

    int pixelIndex = angleToPixelIndex(angleDeg);

    Serial.print("Pt ");
    Serial.print(i);
    Serial.print(": angle=");
    Serial.print(angleDeg, 2);
    Serial.print(" deg, dist=");
    Serial.print(dist);
    Serial.print(" mm, pixel=");
    Serial.print(pixelIndex);
    Serial.print(", intensity=");
    Serial.println(intensity);

    // Only consider valid distances within our range
    if (dist > 0 && dist < FAR_DIST_MM) {
      if (dist < newDist[pixelIndex]) {
        newDist[pixelIndex] = dist;
      }
    }
  }
/*
  Serial.println("Per-pixel closest distances this frame (only where changed):");
  for (int i = 0; i < 10; i++) {
    Serial.print("Pixel ");
    Serial.print(i);
    Serial.print(": ");
    if (newDist[i] == 0xFFFF) {
      Serial.println("no new data");
    } else {
      Serial.print(newDist[i]);
      Serial.println(" mm (candidate)");
    }
  }
*/
  updateNeoPixelsFromDistances();
}

// ---- SETUP & LOOP ----

void setup() {
  //Serial.begin(115200);
  //while (!Serial) {
    // wait for Serial Monitor
  //}

  CircuitPlayground.begin();
  CircuitPlayground.clearPixels();

  // Initialize stored colors to off
  for (int i = 0; i < 10; i++) {
    curR[i] = curG[i] = curB[i] = 0;
  }

  LIDAR_SERIAL.begin(230400);
}

void loop() {
  while (LIDAR_SERIAL.available()) {
    uint8_t b = LIDAR_SERIAL.read();

    if (frameIndex == 0) {
      if (b != HEADER) continue;
      frameBuf[frameIndex++] = b;

    } else if (frameIndex == 1) {
      if (b != VERLEN_EXPECTED) {
        frameIndex = 0;
        continue;
      }
      frameBuf[frameIndex++] = b;

    } else {
      frameBuf[frameIndex++] = b;

      if (frameIndex == FRAME_SIZE) {
        uint8_t crcCalc = calcCRC8(frameBuf, FRAME_SIZE - 1);
        uint8_t crcRecv = frameBuf[FRAME_SIZE - 1];

        if (crcCalc == crcRecv) {
          parseFrame(frameBuf);
        } else {
          Serial.println("CRC error (frame dropped)");
        }

        frameIndex = 0;
      }
    }
  }
}
