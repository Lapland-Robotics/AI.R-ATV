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

// *** Wi-Fi settings ***
const char* ssid     = "Drieshotspot";
const char* password = "kaaskaas";
// IP of your laptop on the hotspot network
const char* serverIP = "192.168.245.111";  
const uint16_t serverPort = 5000;
const int ACS_PIN = 32;

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin();
  pinMode(ACS_PIN, INPUT);
  // init INA219s
  for (uint8_t i=0; i<NUM_SENS; i++) {
    if (!sensors[i]->begin()) {
      Serial.printf("No INA219 @ %s\n", labels[i]);
    } else {
      sensors[i]->setCalibration_32V_2A();
    }
  }
  Serial.println("INA219 ready.");
  // connect Wi-Fi
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
    // 1) Read all INA219s into JSON as before…
    StaticJsonDocument<512> doc;
    doc["timestamp"] = millis();
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

    // 2) Read ACS711 → system current
    int raw = analogRead(ACS_PIN);
    float Vadc     = raw * (3.3f / 4095.0f);
    float Vbus5    = sensors[3]->getBusVoltage_V();   // sensor @0x45  
    float baseline = Vbus5 / 2.0f;                   
    float I_system = (Vadc - baseline) / 0.045f;      // A

    doc["system_current_A"] = I_system;

    // 3) Serialize + send HTTP POST
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
  delay(1000);
}

