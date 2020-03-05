#include <ArduinoJson.h>

// Enough space for 1 object with 6 members
StaticJsonDocument<JSON_OBJECT_SIZE(7)> jsonOut; // TODO: determine what all is being sent back

void setup() {
  // Initialize serial port
  Serial.begin(38400); // TODO: decide on Serial rate

  // TODO: set up ultrasonic sensors


  // TODO: set up magnetometer IMU
  
}

void loop() {
  // Indicate that this Arduino is the sensor Arduino
  jsonOut["sensor"] = 1;
  
  // Add data from ultrasonic sensors to JSON
  // TODO: read data from ultrasonic sensors
  jsonOut["us_front"] = -1;
  jsonOut["us_left"] = -1;
  jsonOut["us_right"] = -1;

  // Add data from magnetometer to JSON
  // TODO: read data from magnetometer
  jsonOut["mag_x"] = -1;
  jsonOut["mag_y"] = -1;
  jsonOut["mag_z"] = -1;
  
  // Send JSON over serial
  serializeJson(jsonOut, Serial);
  Serial.println();
}
