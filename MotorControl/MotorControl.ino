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
  // Set up motors on specified pins
  motorLeft.begin(8,9,5).reverse();
  motorRight.begin(10,12,6).reverse();
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial) continue;
}

void loop() {
  if (Serial.available()) {
    // Read the JSON over serial
    Serial.readBytesUntil('\n', json, 128);
    // Deserialize the JSON document so we can easily access its data
    deserializeJson(doc, json);
    
    // Turn on/off built in LED based led status received
    if (doc["led"] == true) {
      // Turn on LED
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (doc["led"] == false) {
      // Turn off LED
      digitalWrite(LED_BUILTIN, LOW);
    }
    
    // Set left and right motor speed based on controller stick values received
    motorLeft.output(doc["leftStick"]);
    motorRight.output(doc["rightStick"]);

    // Send the JSON back over serial
    serializeJson(doc, Serial);
    Serial.println();
  }
}
