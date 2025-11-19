#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// Pin Definitions
#define SDA_PIN 5
#define SCL_PIN 6

// Display Configuration
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();
}

void loop() {
  unsigned long currentMillis = millis();

  // Update random "pressure" every second
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    int randomPressure = random(0, 40001); // 0–40 kPa * 1000 for precision
    float pressureValue = randomPressure / 1000.0; // convert to float kPa
    drawPressure(pressureValue);
  }
}

void drawPressure(float value) {
  u8g2.clearBuffer();

  // Choose bigger font
  u8g2.setFont(u8g2_font_fub20_tr); // 25px height font, fits small screen width-wise

  // Prepare string with 1 decimal to fit screen
  char buf[16];
  sprintf(buf, "%.1f", value);

  // Compute string width to right-align
  int w = u8g2.getStrWidth(buf);
  int xpos = u8g2.getDisplayWidth() - w;
  int ypos = 30; // vertical position, adjust for font size

  u8g2.setCursor(xpos, ypos);
  u8g2.print(buf);
  u8g2.sendBuffer();
}
