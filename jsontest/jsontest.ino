#include <ArduinoJson.h>
char json[128];
// Allocate the JSON document
//
// Inside the brackets, 200 is the capacity of the memory pool in bytes.
// Don't forget to change this value to match your JSON document.
// Use arduinojson.org/v6/assistant to compute the capacity.
StaticJsonDocument<200> doc;

void setup() {
  // Initialize serial port
  Serial.begin(9600);
  Serial.setTimeout(50);
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial) continue;

}

void loop() {
  if (Serial.available()) {
    Serial.readBytesUntil('\n', json, 128);
    // Deserialize the JSON document
    deserializeJson(doc, json);

    if (doc["led"]) {
      // Turn on LED
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      // Turn off LED
      digitalWrite(LED_BUILTIN, LOW);
    }
  
    serializeJson(doc, Serial);
    Serial.println();
  }
/*
  // Fetch values.
  //
  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do doc["time"].as<long>();
  const char* sensor = doc["sensor"];
  long time = doc["time"];
  double latitude = doc["data"][0];
  double longitude = doc["data"][1];

  // Print values.
  Serial.println(sensor);
  Serial.println(time);
  Serial.println(latitude, 6);
  Serial.println(longitude, 6);
  */
}
