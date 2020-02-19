#include <hcsr04.h> //Bifrost Library for HC SR04 sensors

// First sensor, the nice one that goes on the front of the bot, analog because of the sensor
const int S1_PIN = A0;
const double NUM_SAMPLES = 20.0;
// Second sensor, polling one that goes on the right side of the bot
const int S2_TRIG = 11;
const int S2_ECHO = 10;
// Third sensor, polling one that goes on the left side of the bot
const int S3_TRIG = 9;
const int S3_ECHO = 8;
bool moving = false; //OMG, the bot should not start moving on startup

//Variables used other places, but not initialized here
int sum;

// Declaring the side sensors
// HCSR04 name(trigger pin, echo pin, minimum distance in mm, maximum distance in mm)
HCSR04 SensorRight(S2_TRIG, S2_ECHO, 25, 4000);
HCSR04 SensorLeft(S3_TRIG, S3_ECHO, 25, 4000);

void setup() {
  Serial.begin(9600);
  pinMode(S1_PIN, INPUT);
}

void loop() {
  if (moving) {
    // TODO: Add code to stop moving if the sensors detect certain things
    Serial.println ("Front: " + String(checkSensor1()) + " Right: " + String(SensorRight.distanceInMillimeters()/25.0) + " Left: " + String(SensorLeft.distanceInMillimeters()/25.0));
    delay(250);
  }
  else {
    // TODO: Add code to start moving / move differently if certain conditions are met
    moving = true;
  }
}

int checkSensor1() {
  sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(S1_PIN);
  }
  return sum / NUM_SAMPLES;
}
