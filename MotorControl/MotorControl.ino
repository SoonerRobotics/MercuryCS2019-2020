#include <RobotLib.h>
#include <ArduinoJson.h>
char json[128];
// Allocate the JSON document
//
// Inside the brackets, 200 is the capacity of the memory pool in bytes.
// Don't forget to change this value to match your JSON document.
// Use arduinojson.org/v6/assistant to compute the capacity.
StaticJsonDocument<200> doc;
Motor motorLeft;
Motor motorRight;

void setup() {
  // Initialize serial port
  Serial.begin(38400);
  Serial.setTimeout(50);
  motorLeft.begin(2,3,9);
  motorRight.begin(4,5,10).reverse();
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
    else if (doc["led"] == false) {
      // Turn off LED
      digitalWrite(LED_BUILTIN, LOW);
    }
    motorLeft.output(doc["leftStick"]);
    motorRight.output(doc["rightStick"]);
    
    serializeJson(doc, Serial);
    Serial.println();
  }
}
