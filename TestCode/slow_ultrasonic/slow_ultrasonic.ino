#include <hcsr04.h>

//Heavily based on the wall_avoid.ino code, but modified to function with an ultrasonic sensor which requires a ping to operate
//#include <RobotLib.h>

//New variables:
const int ECHO_PIN1 = 8; //Change as necessary
const int TRIG_PIN1 = 7; //Change as necessary
const int ECHO_PIN2 = 10; //Change as necessary
const int TRIG_PIN2 = 9; //Change as necessary

//Copied variables:
const int TURN_TIME = 500;
// How fast the wheel motors spin, from 0 to 1
const double DRIVE_SPEED = 0.4;
// Distance from the wall at which the robot slows down
const double SLOW_DISTANCE = 30.0;
// Distance from the wall at which the robot starts to turn
const double TURN_DISTANCE = 15.0;

//Motor motorLeft;
//Motor motorRight;
double distance;
int sum;


HCSR04 sensor1(TRIG_PIN1, ECHO_PIN1, 20, 4000);
HCSR04 sensor2(TRIG_PIN2, ECHO_PIN2, 20, 4000);

void setup() {
  Serial.begin(9600);
  // pinMode(ECHO_PIN, INPUT);
  // pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(TRIG_PIN, OUTPUT);
  //motorLeft.begin(8,9,5).reverse();
  //motorRight.begin(10,12,6).reverse();
}

void loop() {
  //Code for determining distance with the sensor
  /*
  sum = 0;
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  //Wait for echo
  Serial.println(pulseIn(ECHO_PIN, HIGH)/148.0);*/
  Serial.println("Sensor 1: " + String((sensor1.distanceInMillimeters()/25.0)));
  Serial.println("Sensor 2: " + String((sensor2.distanceInMillimeters()/25.0)));
  delay(250);

  /*
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
  Serial.println(distance);*/

}
