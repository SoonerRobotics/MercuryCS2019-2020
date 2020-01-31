#include <RobotLib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

Motor leftMotor;
Motor rightMotor;

float turnPower;

unsigned long startTime;

const float TURN_DEADZONE = 5.0;
// Time to finish a turn in milliseconds
const unsigned long TURN_FINISH_TIME = 5000;

const float ANGLE_CHANGE = 90.0;

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t orientationData;

// Heading control  
PIDController yawController;
float targetAngle = 180.0;

// Speed control
PIDController speedController;
float targetSpeed = 0.0;

/////////////////////////////////
// Timing
/////////////////////////////////
#define LOOP_RATE   200
#define LOOP_PERIOD (float)(1.0f / (float)LOOP_RATE)
#define MILLIS_PER_SECOND 1000

unsigned long last_loop_time;

float constrainAngle(float x)
{
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

float angleDiff(float a,float b)
{
    float dif = fmod(b - a + 180,360);
    if (dif < 0)
        dif += 360;
    return dif - 180;
}

void turnToHeading(float currentHeading) {
    // Achieve the current target heading by locking the IMU to the desired yaw
    turnPower = -yawController.update(0, angleDiff(constrainAngle(orientationData.orientation.x), targetAngle));
    //turnPower = RLUtil::clamp(turnPower, -1, 1);
    Serial.print("Heading: " + String(orientationData.orientation.x));
    Serial.println("    turnPower: " + String(turnPower));
    leftMotor.output(-1 * turnPower);
    rightMotor.output(turnPower);
}

void setup() {
    Serial.begin(9600);

    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);
    // Set to mode where magnetometer is turned off
    bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
    // Set up the yaw PID controller
    yawController.begin(0, 0.01, 0.01, 0);
    leftMotor.begin(8,9,5).reverse();
    rightMotor.begin(10,12,6).reverse();
    // Initialize loop timer
    last_loop_time = millis();
}

void loop() {
    // Get BNO055 orientation data
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // The robot is not close to the desired heading
    if ((fabs(angleDiff(constrainAngle(orientationData.orientation.x), targetAngle)) - TURN_DEADZONE) > 0) {
      turnToHeading(orientationData.orientation.x);
    }  
    // The robot is close enough to the desired heading
    else {
      startTime = millis();
      while ((millis() - startTime) < TURN_FINISH_TIME) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        turnToHeading(orientationData.orientation.x);
      }
      leftMotor.output(0.0);
      rightMotor.output(0.0);
      delay(3000);
      targetAngle = constrainAngle(targetAngle + ANGLE_CHANGE);
      Serial.println("New target heading: " + String(targetAngle));
    }
}
