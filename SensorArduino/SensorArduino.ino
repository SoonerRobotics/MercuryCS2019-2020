#include <ArduinoJson.h>
#include <hcsr04.h> //Bifrost Library for HC SR04 sensors

// Enough space for 1 object with 7 members
StaticJsonDocument<JSON_OBJECT_SIZE(7)> jsonOut; // TODO: determine what all is being sent back

/* Declaring constants that will not be changing throughout the program */
// First sensor, polling one that goes on the front of the bot
const int S1_TRIG = 9;
const int S1_ECHO = 3;
// Second sensor, polling one that goes on the right side of the bot
const int S2_TRIG = 5;
const int S2_ECHO = 4;
// Third sensor, polling one that goes on the left side of the bot
const int S3_TRIG = 6;
const int S3_ECHO = 7;

/* Declaring the side sensors */
// HCSR04 name(trigger pin, echo pin, minimum distance in mm, maximum distance in mm)
HCSR04 SensorFront(S1_TRIG, S1_ECHO, 25, 4000);
HCSR04 SensorRight(S2_TRIG, S2_ECHO, 25, 4000);
HCSR04 SensorLeft(S3_TRIG, S3_ECHO, 25, 4000);

void setup() {
  // Initialize serial port
  Serial.begin(38400); // TODO: decide on Serial rate

  // TODO: set up magnetometer IMU
  
}

void loop() {
  // Indicate that this Arduino is the sensor Arduino
  jsonOut["sensor"] = 1;

  // Read and add data from ultrasonic sensors to JSON
  jsonOut["us_front"] = SensorFront.distanceInMillimeters();
  jsonOut["us_left"] = SensorLeft.distanceInMillimeters();
  jsonOut["us_right"] = SensorRight.distanceInMillimeters();

  // Add data from magnetometer to JSON
  // TODO: read data from magnetometer
  jsonOut["mag_x"] = -1;
  jsonOut["mag_y"] = -1;
  jsonOut["mag_z"] = -1;
  
  // Send JSON over serial
  serializeJson(jsonOut, Serial);
  Serial.println();
}
