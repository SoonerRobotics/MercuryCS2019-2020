#include <hcsr04.h> //Bifrost Library for HC SR04 sensors

/* Declaring constants that will not be changing throughout the program */
// First sensor, polling one that goes on the front of the bot
const int S1_TRIG = 9;
const int S1_ECHO = 3;
/* Deprecated Sensor code
const int S1_PIN = A0;
const double NUM_SAMPLES = 20.0;*/
// Second sensor, polling one that goes on the right side of the bot
const int S2_TRIG = 5;
const int S2_ECHO = 4;
// Third sensor, polling one that goes on the left side of the bot
const int S3_TRIG = 6;
const int S3_ECHO = 7;

bool moving = false; //OMG, the bot should not start moving on startup
const double CLEARANCE = 75.0; //This double is the clearance needed on a side for the robot to rotate in place, given in Millimeters

/*Variables used other places, but not initialized here*/
double sum;
bool bool1, bool2;

/* Declaring the side sensors */
// HCSR04 name(trigger pin, echo pin, minimum distance in mm, maximum distance in mm)
HCSR04 SensorFront(S1_TRIG, S1_ECHO, 25, 4000);
HCSR04 SensorRight(S2_TRIG, S2_ECHO, 25, 4000);
HCSR04 SensorLeft(S3_TRIG, S3_ECHO, 25, 4000);

void setup() {
  Serial.begin(9600);
  //pinMode(S1_PIN, INPUT);
}

void loop() {
  if (moving) {
    // TODO: Add code to stop moving if the sensors detect certain things
    sum = SensorFront.distanceInMillimeters();
    Serial.println ("Front: " + String(sum/25.0) + " Right: " + String(SensorRight.distanceInMillimeters()/25.0) + " Left: " + String(SensorLeft.distanceInMillimeters()/25.0));
    if (sum  < CLEARANCE) {
      moving = false;
      Serial.println("Stopping");
    }
    delay(250);
  }
  else {
    if (SensorFront.distanceInMillimeters() > CLEARANCE) {
      Serial.println("Way in front cleared, moving again.");
      // TODO: Add code to actually control motors on the bot
      moving = true;
    }
    else {
      bool1 = SensorRight.distanceInMillimeters() < CLEARANCE;
      bool2 = SensorLeft.distanceInMillimeters() < CLEARANCE;

      if (bool1 && bool2) {
        Serial.println("Backing up, sides blocked");
        // TODO: Include code to move backwards
      }
      else if (bool1 && !(bool2)) {
        Serial.println("Backing up, Right Side blocked");
        // TODO: Include code to move backwards and to the left
      }
      else if (bool2 && !(bool1)) { // order of bools switched to take advantage of short circuit execution
        Serial.println("Backing up, Left Side blocked");
        // TODO: Include code to move backwards and to the right
      }
      else {
        Serial.println("Spinning in place to get new heading");
        // TODO: Include code to rotate in place
      }
      moving = true;
    }
  }
}
/* Deprecated Sensor Code
int checkSensor1() {
  sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(S1_PIN);
  }
  return sum / 2 / NUM_SAMPLES;
} */
