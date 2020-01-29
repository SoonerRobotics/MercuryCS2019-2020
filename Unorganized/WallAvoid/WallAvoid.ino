#include <RobotLib.h>
// How many milliseconds we want the robot to turn
const int TURN_TIME = 500;
// How fast the wheel motors spin, from 0 to 1
const double DRIVE_SPEED = 0.4;
// Distance from the wall at which the robot slows down
const double SLOW_DISTANCE = 30.0;
// Distance from the wall at which the robot starts to turn
const double TURN_DISTANCE = 15.0;
// Number of samples we average from the distance sensor
const int NUM_SAMPLES = 20;
// Pin that the distance sensor is connected to
const int DIST_PIN = A5;

Motor motorLeft;
Motor motorRight;
double sum;
double distance;

void setup() {
  Serial.begin(9600);
  pinMode(DIST_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  motorLeft.begin(8,9,5).reverse();
  motorRight.begin(10,12,6).reverse();
}

void loop() {
  // Take average of distance sensor readings
  sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(DIST_PIN);
  }
  distance = sum / (double)NUM_SAMPLES;

  // If we're very close to a wall
  if (distance < TURN_DISTANCE) {
    digitalWrite(LED_BUILTIN, HIGH);
    // Turn left
    motorLeft.output(-1.0 * DRIVE_SPEED);
    motorRight.output(DRIVE_SPEED);
    // Turn for TURN_TIME milliseconds
    delay(TURN_TIME);
  }
  // If we're somewhat close to a wall
  else if (distance < SLOW_DISTANCE) {
    digitalWrite(LED_BUILTIN, HIGH);
    // Drive forward at slower speed
    motorLeft.output(DRIVE_SPEED / 1.5);
    motorRight.output(DRIVE_SPEED / 1.5);
  }
  // If we're not close to a wall
  else {
    digitalWrite(LED_BUILTIN, LOW);
    // Drive forward at full speed
    motorLeft.output(DRIVE_SPEED);
    motorRight.output(DRIVE_SPEED);
  }
  // Print the average distance
  Serial.println(distance);
}
