#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// INA219 instances
Adafruit_INA219 ina0(0x40), ina1(0x41), ina2(0x44),
                 ina3(0x45), ina4(0x46), ina5(0x4F);
Adafruit_INA219* sensors[] = {&ina0,&ina1,&ina2,&ina3,&ina4,&ina5};
const char* labels[]  = {"0x40","0x41","0x44","0x45","0x46","0x4F"};
const uint8_t NUM_SENS = sizeof(sensors)/sizeof(sensors[0]);

// Wi-Fi settings
const char* ssid     = "Drieshotspot";
const char* password = "kaaskaas";
// Your laptop’s IP on the hotspot
const char* serverIP   = "192.168.164.111";
const uint16_t serverPort = 5000;

// ACS711 Viout → any ADC1 pin; here GPIO32 (ADC1_CH4)
const int ACS_PIN = 32;

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin();
  pinMode(ACS_PIN, INPUT);
  // optional: full-range attenuation for 0–3.3 V
  analogSetPinAttenuation(ACS_PIN, ADC_11db);

  // initialize all INA219 sensors
  for (uint8_t i = 0; i < NUM_SENS; i++) {
    if (!sensors[i]->begin()) {
      Serial.printf("No INA219 @ %s\n", labels[i]);
    } else {
      sensors[i]->setCalibration_32V_2A();
    }
  }
  Serial.println("INA219 ready.");

  // connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Joining Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected, IP = %s\n", WiFi.localIP().toString().c_str());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    // build JSON with ArduinoJson
    StaticJsonDocument<512> doc;
    doc["timestamp"] = millis();

    // sensor array
    JsonArray arr = doc.createNestedArray("readings");
    for (uint8_t i = 0; i < NUM_SENS; i++) {
      float busV    = sensors[i]->getBusVoltage_V();
      float shuntmV = sensors[i]->getShuntVoltage_mV();
      float current = sensors[i]->getCurrent_mA();
      float power   = sensors[i]->getPower_mW();
      JsonObject obj = arr.createNestedObject();
      obj["addr"] = labels[i];
      obj["V"]    = busV;
      obj["mV"]   = shuntmV;
      obj["mA"]   = current;
      obj["mW"]   = power;
    }

    // read ACS711 output & compute
    int   raw     = analogRead(ACS_PIN);
    float Vadc    = raw * (3.3f / 4095.0f);         // raw voltage at pin
    float Vbus5   = sensors[3]->getBusVoltage_V();  // INA @0x45 = 5 V rail
    float baseline= Vbus5 / 2.0f;                   // half-supply
    float I_system = (Vadc - baseline) / 0.045f;    // A per 45 mV/A

    // add both raw voltage and computed current
    doc["system_voltage_V"]   = Vadc;
    doc["system_current_A"]   = I_system;

    // serialize & POST
    String json;
    serializeJson(doc, json);
    HTTPClient http;
    http.begin(String("http://") + serverIP + ":" + serverPort + "/data");
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(json);
    Serial.printf("POST %d\n", code);
    http.end();
  } else {
    WiFi.reconnect();
  }

  delay(500);
}
