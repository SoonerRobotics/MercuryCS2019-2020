#include <RobotLib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Motor leftMotor;
Motor rightMotor;

float turnPower;

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler, accel;
sensors_event_t event;

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
    leftMotor.begin(8,9,5).reverse();
    rightMotor.begin(10,12,6).reverse();
    // Initialize loop timer
    last_loop_time = millis();
}

void loop() {
    // BNO055 sensor data
    bno.getEvent(&event);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // Achieve the current target heading by locking the IMU to the desired yaw
    turnPower = yawController.update(0, angleDiff(constrainAngle(euler.x()), targetAngle));
    Serial.println(euler.x());
    leftMotor.output(-1 * turnPower);
    rightMotor.output(turnPower);
}
