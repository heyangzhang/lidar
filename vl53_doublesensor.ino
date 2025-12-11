#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// hd44780 library includes
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class

// LCD object (auto-detects I2C address & pin mapping)
hd44780_I2Cexp lcd;

// Two VL53L0X sensors
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// XSHUT pins for the two sensors
const int XSHUT1_PIN = 2;
const int XSHUT2_PIN = 3;

// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// I2C addresses to assign
const uint8_t VL53_ADDR_1 = 0x29; // default
const uint8_t VL53_ADDR_2 = 0x30; // new address

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Nano Every: A4 = SDA, A5 = SCL

  // Initialize LCD
  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (status) {
    Serial.print("LCD init error: ");
    Serial.println(status);
  }

  lcd.clear();
  lcd.print("Starting VL53L0X");

  // Set XSHUT pins
  pinMode(XSHUT1_PIN, OUTPUT);
  pinMode(XSHUT2_PIN, OUTPUT);

  // Hold both sensors in reset
  digitalWrite(XSHUT1_PIN, LOW);
  digitalWrite(XSHUT2_PIN, LOW);
  delay(10);

  // ---- Initialize first sensor ----
  digitalWrite(XSHUT1_PIN, HIGH);  // bring sensor 1 out of reset
  delay(10);
  if (!lox1.begin(VL53_ADDR_1)) {  // default address
    Serial.println("Failed to boot VL53L0X #1");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("VL53 #1 ERROR");
    while (1);
  }

  // Change first sensor's address so second can use default
  lox1.setAddress(VL53_ADDR_2);   // now sensor 1 is at 0x30

  // ---- Initialize second sensor ----
  digitalWrite(XSHUT2_PIN, HIGH); // bring sensor 2 out of reset
  delay(10);
  if (!lox2.begin(VL53_ADDR_1)) { // second sensor still at default 0x29
    Serial.println("Failed to boot VL53L0X #2");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("VL53 #2 ERROR");
    while (1);
  }

  Serial.println("VL53L0X #1 & #2 ready.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("2x VL53 Ready");

  // Start continuous ranging on both
  lox1.startRangeContinuous();
  lox2.startRangeContinuous();
}

void loop() {
  // Wait until *both* have a measurement ready
  if (lox1.isRangeComplete() && lox2.isRangeComplete()) {
    int dist1 = lox1.readRange();
    int dist2 = lox2.readRange();
    int sumDist = dist1 + dist2;

    // Print to Serial
    Serial.print("Sensor1 (mm): ");
    Serial.print(dist1);
    Serial.print("  Sensor2 (mm): ");
    Serial.print(dist2);
    Serial.print("  Sum (mm): ");
    Serial.println(sumDist);

    // Print to LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    // Row 0: "1:123 2:456"
    lcd.print("1:");
    lcd.print(dist1);
    lcd.print(" 2:");
    lcd.print(dist2);

    lcd.setCursor(0, 1);
    // Row 1: "Sum: 579mm"
    lcd.print("Sum: ");
    lcd.print(sumDist);
    lcd.print("mm");

    delay(150); // small delay to reduce flicker
  }
}
