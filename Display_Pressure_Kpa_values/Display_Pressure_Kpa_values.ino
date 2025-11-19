#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <HX711_ADC.h>

// --------------------
// Pin definitions
// --------------------
#define OLED_SDA 5
#define OLED_SCL 6
#define HX_DOUT 7
#define HX_SCK 8

// --------------------
// OLED setup
// --------------------
U8G2_SSD1306_72X40_ER_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// --------------------
// HX710B setup
// --------------------
HX711_ADC hxSensor(HX_DOUT, HX_SCK);

// --------------------
// Variables
// --------------------
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // update every 1s
long rawZero = 0;                     // baseline at 235 m
const float P0 = 101.3;               // sea-level reference in kPa
const float currentAltitudeSLM = 235; // meters

void setup() {
  Serial.begin(115200);
  Wire.begin(OLED_SDA, OLED_SCL);
  oled.begin();
  oled.clearBuffer();

  // Initialize HX710B
  Serial.println("Initializing HX710B...");
  hxSensor.begin();
  hxSensor.start(2000); // auto-zero for 2 sec
  hxSensor.setCalFactor(1.0);

  // Read baseline raw value
  delay(2000);
  hxSensor.update();
  rawZero = hxSensor.getData();
  Serial.print("Baseline raw value (284 m): ");
  Serial.println(rawZero);
  Serial.println("HX710B ready!");
}

void loop() {
  unsigned long currentMillis = millis();
  hxSensor.update();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Get relative pressure
    long raw = hxSensor.getData();
    long delta = raw - rawZero;

    // Convert to kPa
    const float SCALE = 40.0 / 8388607.0; // ΔkPa per count
    float pressure_kPa = P0 + delta * SCALE;

    // Compute altitude relative to baseline
    float altitude = 44330.0 * (1.0 - pow(pressure_kPa / P0, 0.1903));
    altitude += currentAltitudeSLM; // offset for your elevation

    // Serial output
    Serial.print("Altitude: ");
    Serial.print(altitude, 1);
    Serial.print(" m | Pressure: ");
    Serial.print(pressure_kPa, 2);
    Serial.println(" kPa");

    // Display on OLED
    oled.clearBuffer();

    // --------------------
    // Line 1: altitude value + "m" (right-aligned)
    // --------------------

    // Altitude number (big font)
    char bufAlt[10];
    sprintf(bufAlt, "%d", (int)altitude);

    oled.setFont(u8g2_font_fub20_tr);        // big font
    int wNum = oled.getStrWidth(bufAlt);     // measure width in the same font

    // Unit "m" (small font)
    oled.setFont(u8g2_font_7x14_tr);         // small font
    int wUnit = oled.getStrWidth(" m");      // measure width

    // X position so number + "m" fit entirely
    int xpos1 = oled.getDisplayWidth() - (wNum + wUnit);
    if (xpos1 < 0) xpos1 = 0;               // safety

    // Print altitude number
    oled.setFont(u8g2_font_fub20_tr);
    oled.setCursor(xpos1, 20);               // Y baseline
    oled.print(bufAlt);

    // Print small unit "m" immediately after
    oled.setFont(u8g2_font_7x14_tr);
    oled.setCursor(xpos1 + wNum, 20);
    oled.print(" m");

    // --------------------
    // Line 2: pressure value (right-aligned)
    // --------------------

    char bufP[16];
    sprintf(bufP, "%.1f kPa", pressure_kPa);

    // Set font before measuring
    oled.setFont(u8g2_font_7x14_tr);
    int wP = oled.getStrWidth(bufP);         // width in same font

    // X position for right alignment
    int xpos2 = oled.getDisplayWidth() - wP;
    if (xpos2 < 0) xpos2 = 0;               // safety

    oled.setCursor(xpos2, 40);               // Y baseline
    oled.print(bufP);


    oled.sendBuffer();
  }
}
