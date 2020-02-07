//Heavily based on the wall_avoid.ino code, but modified to function with an ultrasonic sensor which requires a ping to operate
#include <RobotLib.h>


Motor motorLeft;
Motor motorRight;


void setup() {
  Serial.begin(9600);
  pinMode(DIST_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  motorLeft.begin(8,9,5).reverse();
  motorRight.begin(10,12,6).reverse();
}

void loop() {
  //Code for determining distance with the sensor

  //Following code straight copied:
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
