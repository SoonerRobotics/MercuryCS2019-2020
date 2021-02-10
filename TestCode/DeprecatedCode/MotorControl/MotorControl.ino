#include <RobotLib.h>
#include <ArduinoJson.h>
// If we haven't received data for this many ms, we've probably lost connection
const unsigned long CONNECTION_TIMEOUT = 200;
char json[128];
// Allocate the JSON document
//
// Inside the brackets, 200 is the capacity of the memory pool in bytes.
// Don't forget to change this value to match your JSON document.
// Use arduinojson.org/v6/assistant to compute the capacity.
StaticJsonDocument<200> doc;
Motor motorLeft;
Motor motorRight;
unsigned long lastReceiveTime = 0;


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
  // If there is Serial data to be read
  if (Serial.available()) {
    lastReceiveTime = millis();
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
    motorLeft.output(doc["leftMotor"]);
    motorRight.output(doc["rightMotor"]);

    // Send the JSON back over serial
    serializeJson(doc, Serial);
    Serial.println();
  }
  // If there is no Serial data to be read
  else {
    // If we have lost connection
    if (millis() - lastReceiveTime > CONNECTION_TIMEOUT) {
      // Stop motors
      motorLeft.output(0);
      motorRight.output(0);
    }
  }
}
