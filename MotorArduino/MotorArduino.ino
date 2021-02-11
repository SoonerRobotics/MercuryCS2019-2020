#include <RobotLib.h>
#include <ArduinoJson.h>
#include <Adafruit_BNO055.h>

const unsigned long CONNECTION_TIMEOUT = 200;

// TODO: set correct pin numbers
const int ENC_LEFT_PIN_A = 2;
const int ENC_LEFT_PIN_B = 4;
const int ENC_RIGHT_PIN_A = 2;
const int ENC_RIGHT_PIN_B = 4;

// Enough space for 1 object with 2 members (leftMotor and rightMotor)
StaticJsonDocument<JSON_OBJECT_SIZE(2)> jsonIn;
// Enough space for 1 object with 3 members
StaticJsonDocument<JSON_OBJECT_SIZE(4)> jsonOut; // TODO: determine what all is being sent back
// Buffer for incoming JSON
char jsonInBuffer[128];

Motor motorLeft;
Motor motorRight;

QuadratureEncoder encoderLeft;
QuadratureEncoder encoderRight;

// IMU that reads heading
Adafruit_BNO055 imuHeading = Adafruit_BNO055(55);
sensors_event_t orientationData;

unsigned long lastReceiveTime = 0;

void encoderLeftInterrupt() {
  encoderLeft.process();
}

void encoderRightInterrupt() {
  encoderRight.process();
}

void setup() {
  // Initialize serial port
  Serial.begin(38400); // TODO: decide on Serial rate
  Serial.setTimeout(50);
  
  // Set up motors on specified pins
  motorLeft.begin(8,9,5).reverse();
  motorRight.begin(10,12,6).reverse();
  
  // Set up encoders on specified pins
  encoderLeft.begin(ENC_LEFT_PIN_A, ENC_LEFT_PIN_B, 1);
  encoderRight.begin(ENC_RIGHT_PIN_B, ENC_RIGHT_PIN_B, 1);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN_A), encoderLeftInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN_A), encoderRightInterrupt, CHANGE);

  // Set up IMU that reads heading
  imuHeading.begin();
  delay(1000);
  // Set to mode where magnetometer is turned off
  imuHeading.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
}

void loop() {
  // If there is Serial data to be read
  if (Serial.available()) {
    lastReceiveTime = millis();
    // Read the JSON over serial
    Serial.readBytesUntil('\n', jsonInBuffer, 128);
    // Deserialize the JSON document so we can access its data
    DeserializationError jsonErr = deserializeJson(jsonIn, jsonInBuffer);
    // Make sure there was no error deserializing the JSON
    if (!jsonErr) {
      // Set left and right motor speed based on controller stick values received
      motorLeft.output(jsonIn["leftMotor"]);
      motorRight.output(jsonIn["rightMotor"]);
    }
  }
  // If there is no Serial data to be read
  else {
    // Check if we have lost connection
    if (millis() - lastReceiveTime > CONNECTION_TIMEOUT) {
      // Stop motors
      motorLeft.output(0);
      motorRight.output(0);
    }
  }

  // Indicate that this Arduino is not the sensor Arduino
  jsonOut["sensor"] = 0;
  
  // Add encoder data to JSON
  jsonOut["enc_left"] = encoderLeft.getValue();
  jsonOut["enc_right"] = encoderRight.getValue();

  // Get BNO055 orientation data
  imuHeading.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // Add heading data to JSON
  jsonOut["heading"] = orientationData.orientation.x;

  // Send JSON back over serial
  serializeJson(jsonOut, Serial);
  Serial.println();
}
