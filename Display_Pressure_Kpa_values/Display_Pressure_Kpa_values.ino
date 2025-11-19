#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <HX711_ADC.h>

// WiFi credentials
const char* ssid = "<YOUR_SSID>";
const char* password = "<YOUR_WIFI_PASSWORD>";

// HiveMQ Cluster info
const char* mqtt_server = "<YOUR_MQTT_BROKER_URL>";
const char* mqtt_user = "<YOUR_MQTT_USER>";
const char* mqtt_pass = "<YOUR_MQTT_PASS>";
const int reconnect_delay = 5000;
const int status_delay = 60000;

// NTP - Timezone stuff
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;  // Europe/Berlin is UTC+1
const int daylightOffset_sec = 3600;

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
// Sensors
const char* sensorKeys[] = {"pressure", "altitude"};
const size_t sensorCount = sizeof(sensorKeys) / sizeof(sensorKeys[0]);
HX711_ADC hxSensor(HX_DOUT, HX_SCK);

// --------------------
// Variables
// --------------------
unsigned long previousMillis = 0;
const unsigned long interval = 5000; // update every 5s
long rawZero = 0;                     // baseline at 235 m
const float P0 = 101.3;               // sea-level reference in kPa
const float currentAltitudeSLM = 235; // meters

// Topics
char registerTopic[64];
String statusTopic;

// CA certificate chain
const char* ca_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFGTCCBAGgAwIBAgISBtYpojzbu6YILpLtnYRbYbC/MA0GCSqGSIb3DQEBCwUA\n" \
"MDMxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQwwCgYDVQQD\n" \
"EwNSMTIwHhcNMjUxMDAzMDQ0NzMyWhcNMjYwMTAxMDQ0NzMxWjAfMR0wGwYDVQQD\n" \
"DBQqLnMyLmV1LmhpdmVtcS5jbG91ZDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCC\n" \
"AQoCggEBAMSjskkjT3lienCFm5ptT7HN1Zi/2CviuTWY9hpTiSWjCqKW+stAPFe8\n" \
"MmDNJGoiHuHxn3S8UisKBa0l3vILJmjpIHVO9+DsHjhgj2lN1tGYyKmc0c4aU/BW\n" \
"Pc4/DZaWaVf0Bm25mAjsumHwIsaWddC/S9YO+AjvTOhrn+9Og34PRNTuDeSMFkK9\n" \
"YmL9Yb3a9bUx5W5Y9A3BVbvp2GidESD7Ih7d5X5XaYVxrsNaSBKbr+tQ9dtX7q2a\n" \
"kA96J1sNaEN3lISWFCP5yqqzUsq2A872HJHczp54rf5QriagM5hXWLxJsh75lsEu\n" \
"xxjhGCojmI7VpK3yurIhLNqapP8zjWkCAwEAAaOCAjkwggI1MA4GA1UdDwEB/wQE\n" \
"AwIFoDAdBgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDAYDVR0TAQH/BAIw\n" \
"ADAdBgNVHQ4EFgQUlL6goqpL++sQlIGTQXv8bP6Ck94wHwYDVR0jBBgwFoAUALUp\n" \
"8i2ObzHom0yteD763OkM0dIwMwYIKwYBBQUHAQEEJzAlMCMGCCsGAQUFBzAChhdo\n" \
"dHRwOi8vcjEyLmkubGVuY3Iub3JnLzAzBgNVHREELDAqghQqLnMyLmV1LmhpdmVt\n" \
"cS5jbG91ZIISczIuZXUuaGl2ZW1xLmNsb3VkMBMGA1UdIAQMMAowCAYGZ4EMAQIB\n" \
"MC4GA1UdHwQnMCUwI6AhoB+GHWh0dHA6Ly9yMTIuYy5sZW5jci5vcmcvNjQuY3Js\n" \
"MIIBBQYKKwYBBAHWeQIEAgSB9gSB8wDxAHcA7TxL1ugGwqSiAFfbyyTiOAHfUS/t\n" \
"xIbFcA8g3bc+P+AAAAGZqJseDwAABAMASDBGAiEAg50XZlV6D3C3FdsuQI8FohyU\n" \
"PQC3evkoSldjh9d5ZekCIQCRgIKVL7YcO7ZBYKGX2QZiNqKnLYqWTyjSU/nw3W2r\n" \
"sgB2AJaXZL9VWJet90OHaDcIQnfp8DrV9qTzNm5GpD8PyqnGAAABmaibHjwAAAQD\n" \
"AEcwRQIgVXSzlK9rN5heWgdUi9mGGy9dtfajc/rcLIvzzuPpbhYCIQDoO2nKM4pz\n" \
"ftKfqY1LwdNrwYbwUMVNbdLezn8GVJOR6jANBgkqhkiG9w0BAQsFAAOCAQEAmcgt\n" \
"6VWtVR32eHscRuDHmUiXuAho0RCsGbxD0j9/l3bYBna/qLYtr3ArbHGrvkQcCurZ\n" \
"0jS2RHANhVgnm5PjfAR4ovKkzIJy1PY7Y71MLqtCwSUAtLuKuhpYtnwhu4iI2F7H\n" \
"0IPQAR0MLfEUrjmV+MEBEpK6yT9CUjtuSVEXJQr6Kzyz+Lyfmjyv01K8HkIGj3mk\n" \
"ArZnsN0cfjSC5K2GiIQu6pFFrBXhcpvBxaipQ5DI2AUZVsI9CGx34Igpj/8WjE8D\n" \
"Rt2FVBvNXMPw13Gcrf9AayJdkhjVBGMUciFAiCh4J6qvbrQ3qd1G65Hqsb6l32f6\n" \
"g2AWh3WXA9kSs4jUXg==\n" \
"-----END CERTIFICATE-----\n" \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFBjCCAu6gAwIBAgIRAMISMktwqbSRcdxA9+KFJjwwDQYJKoZIhvcNAQELBQAw\n" \
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjQwMzEzMDAwMDAw\n" \
"WhcNMjcwMzEyMjM1OTU5WjAzMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg\n" \
"RW5jcnlwdDEMMAoGA1UEAxMDUjEyMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB\n" \
"CgKCAQEA2pgodK2+lP474B7i5Ut1qywSf+2nAzJ+Npfs6DGPpRONC5kuHs0BUT1M\n" \
"5ShuCVUxqqUiXXL0LQfCTUA83wEjuXg39RplMjTmhnGdBO+ECFu9AhqZ66YBAJpz\n" \
"kG2Pogeg0JfT2kVhgTU9FPnEwF9q3AuWGrCf4yrqvSrWmMebcas7dA8827JgvlpL\n" \
"Thjp2ypzXIlhZZ7+7Tymy05v5J75AEaz/xlNKmOzjmbGGIVwx1Blbzt05UiDDwhY\n" \
"XS0jnV6j/ujbAKHS9OMZTfLuevYnnuXNnC2i8n+cF63vEzc50bTILEHWhsDp7CH4\n" \
"WRt/uTp8n1wBnWIEwii9Cq08yhDsGwIDAQABo4H4MIH1MA4GA1UdDwEB/wQEAwIB\n" \
"hjAdBgNVHSUEFjAUBggrBgEFBQcDAgYIKwYBBQUHAwEwEgYDVR0TAQH/BAgwBgEB\n" \
"/wIBADAdBgNVHQ4EFgQUALUp8i2ObzHom0yteD763OkM0dIwHwYDVR0jBBgwFoAU\n" \
"ebRZ5nu25eQBc4AIiMgaWPbpm24wMgYIKwYBBQUHAQEEJjAkMCIGCCsGAQUFBzAC\n" \
"hhZodHRwOi8veDEuaS5sZW5jci5vcmcvMBMGA1UdIAQMMAowCAYGZ4EMAQIBMCcG\n" \
"A1UdHwQgMB4wHKAaoBiGFmh0dHA6Ly94MS5jLmxlbmNyLm9yZy8wDQYJKoZIhvcN\n" \
"AQELBQADggIBAI910AnPanZIZTKS3rVEyIV29BWEjAK/duuz8eL5boSoVpHhkkv3\n" \
"4eoAeEiPdZLj5EZ7G2ArIK+gzhTlRQ1q4FKGpPPaFBSpqV/xbUb5UlAXQOnkHn3m\n" \
"FVj+qYv87/WeY+Bm4sN3Ox8BhyaU7UAQ3LeZ7N1X01xxQe4wIAAE3JVLUCiHmZL+\n" \
"qoCUtgYIFPgcg350QMUIWgxPXNGEncT921ne7nluI02V8pLUmClqXOsCwULw+PVO\n" \
"ZCB7qOMxxMBoCUeL2Ll4oMpOSr5pJCpLN3tRA2s6P1KLs9TSrVhOk+7LX28NMUlI\n" \
"usQ/nxLJID0RhAeFtPjyOCOscQBA53+NRjSCak7P4A5jX7ppmkcJECL+S0i3kXVU\n" \
"y5Me5BbrU8973jZNv/ax6+ZK6TM8jWmimL6of6OrX7ZU6E2WqazzsFrLG3o2kySb\n" \
"zlhSgJ81Cl4tv3SbYiYXnJExKQvzf83DYotox3f0fwv7xln1A2ZLplCb0O+l/AK0\n" \
"YE0DS2FPxSAHi0iwMfW2nNHJrXcY3LLHD77gRgje4Eveubi2xxa+Nmk/hmhLdIET\n" \
"iVDFanoCrMVIpQ59XWHkzdFmoHXHBV7oibVjGSO7ULSQ7MJ1Nz51phuDJSgAIU7A\n" \
"0zrLnOrAj/dfrlEWRhCvAgbuwLZX1A2sjNjXoPOHbsPiy+lO1KF8/XY7\n" \
"-----END CERTIFICATE-----\n";

// SSL stuff
WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE 500
char msg[MSG_BUFFER_SIZE];
int value = 0;

String deviceId;

// Returns a 64-bit device ID based on the Wi-Fi MAC address
String getChipId() {
  // Ensure Wi-Fi MAC is initialized
  if (WiFi.getMode() == WIFI_MODE_NULL) {
    WiFi.mode(WIFI_MODE_STA);
    delay(10);
  }

  String mac = WiFi.macAddress(); // "20:6E:F1:6A:C6:64"
  uint64_t id = 0;
  int index = 0;

  for (int i = 0; i < 6; i++) {
    String byteStr = mac.substring(index, index + 2);
    id = (id << 8) | strtoul(byteStr.c_str(), nullptr, 16);
    index += 3; // skip colon
  }

  // Convert to lowercase hex string
  char buf[17]; // 16 hex digits + null
  sprintf(buf, "%016llx", id);

  return String("esp32-") + buf;
}

void publishStatus() {
  // Create alive JSON status
  StaticJsonDocument<500> status;
  status["id"] = deviceId;
  status["status"] = "up";
  status["ip"] = WiFi.localIP().toString();
  status["uptime"] = getAliveSeconds();       // seconds since boot
  status["timestamp"] = currentTimestamp();

  JsonObject sensorsData = status.createNestedObject("sensors_data");
  float pressure = getPressure();
  float altitude = getAltitude(pressure);

  for (size_t i = 0; i < sensorCount; i++) {
    const char* key = sensorKeys[i];

    if (strcmp(key, "pressure") == 0) {
      sensorsData[key] = isnan(pressure) ? 0 : pressure;
    } 
    else if (strcmp(key, "altitude") == 0) {
      sensorsData[key] = isnan(altitude) ? 0 : altitude;
    }  
  }

  char buf[512];
  serializeJson(status, buf);

  // Publish to device-specific status topic
  if (client.publish(statusTopic.c_str(), buf, false)) { // retained
    Serial.printf("[%s] Up status sent\n", deviceId.c_str());
  } else {
    Serial.printf("[%s] Failed to send up status\n", deviceId.c_str());
  }

  printOnDisplay(pressure, altitude);
}

// --------------------
// Function to get pressure in kPa
// --------------------
float getPressure() {
    long raw = hxSensor.getData();
    long delta = raw - rawZero;

    const float SCALE = 40.0 / 8388607.0; // ΔkPa per count
    float pressure_kPa = P0 + delta * SCALE;
    return pressure_kPa;
}

// --------------------
// Function to get altitude in meters
// --------------------
float getAltitude(float pressure) {
    float altitude = 44330.0 * (1.0 - pow(pressure / P0, 0.1903));
    altitude += currentAltitudeSLM; // adjust for your elevation
    return altitude;
}

unsigned long getAliveSeconds() {
  return millis() / 1000; // millis() returns ms since boot
}

void printOnDisplay(float pressure, float altitude) {
        // Serial output
        Serial.print("Altitude: ");
        Serial.print(altitude, 1);
        Serial.print(" m | Pressure: ");
        Serial.print(pressure, 2);
        Serial.println(" kPa");

        // Display on OLED (same as your code, using altitude and pressure)
        oled.clearBuffer();

        // Altitude line
        char bufAlt[10];
        sprintf(bufAlt, "%d", (int)altitude);
        oled.setFont(u8g2_font_fub20_tr);
        int wNum = oled.getStrWidth(bufAlt);
        oled.setFont(u8g2_font_7x14_tr);
        int wUnit = oled.getStrWidth(" m");
        int xpos1 = oled.getDisplayWidth() - (wNum + wUnit);
        if (xpos1 < 0) xpos1 = 0;
        oled.setFont(u8g2_font_fub20_tr);
        oled.setCursor(xpos1, 20);
        oled.print(bufAlt);
        oled.setFont(u8g2_font_7x14_tr);
        oled.setCursor(xpos1 + wNum, 20);
        oled.print(" m");

        // Pressure line
        char bufP[16];
        sprintf(bufP, "%.2f kPa", pressure);
        oled.setFont(u8g2_font_7x14_tr);
        int wP = oled.getStrWidth(bufP);
        int xpos2 = oled.getDisplayWidth() - wP;
        if (xpos2 < 0) xpos2 = 0;
        oled.setCursor(xpos2, 40);
        oled.print(bufP);

        oled.sendBuffer();
}

String currentTimestamp() {
    struct tm timeinfo;

    // getLocalTime returns false if time not set yet
    if (!getLocalTime(&timeinfo)) {
        return "Time not set";
    }

    char timestamp[26];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);

    return String(timestamp);
}

// Publish device registration
void publishRegistration() {
  StaticJsonDocument<200> reg;
  reg["model"] = "ESP32 C3 OLED";
  reg["id"]    = deviceId;
  reg["ip"]    = WiFi.localIP().toString();

  // Nested JSON array from global array
  JsonArray sensors = reg.createNestedArray("sensors");
  for (size_t i = 0; i < sensorCount; i++) {
    sensors.add(sensorKeys[i]);
  }

  char buf[256];
  serializeJson(reg, buf);

  snprintf(registerTopic, sizeof(registerTopic), "devices/%s/register", deviceId.c_str());

  if (client.publish(registerTopic, buf, true)) {
    Serial.printf("[%s] Registration sent successfully\n", deviceId.c_str());
  } else {
    Serial.printf("[%s] Failed to send registration\n", deviceId.c_str());
  }
}

// ====== SETUP ======
void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  espClient.setCACert(ca_cert);
  
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to String
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();  // remove whitespace/newlines

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  publishStatus();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "Wemos-" + deviceId;
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");

      // Publish registration after connecting
      publishRegistration();

      // Publish first status after connecting
      publishStatus();

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.printf(" try again in %s seconds", (reconnect_delay / 1000));
      delay(reconnect_delay);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C3 Serial OK"); 

  deviceId = getChipId();
  Serial.print("Device ID: ");
  Serial.println(deviceId);

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
  Serial.printf("Baseline raw value (%d m): ", (int)currentAltitudeSLM);
  Serial.println(rawZero);
  Serial.println("HX710B ready!");

  statusTopic = "devices/" + deviceId + "/status";

  setup_wifi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  client.setServer(mqtt_server, 8883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

    static unsigned long lastAlive = 0;
    unsigned long now = millis();
    if (now - lastMsg > status_delay) {  // every <status_delay> seconds
    lastMsg = now;

    publishStatus();
  }
}