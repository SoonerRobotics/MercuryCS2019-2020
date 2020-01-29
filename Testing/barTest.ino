// I was using this for testing ways to remove sensor noise to it's kinda messy
// It transmits (somewhat smoothed?) sensor data formatted as JSON
// I might clean this up/explain more later
#include <ArduinoJson.h>
char json[128];
StaticJsonDocument<200> doc;
const int NUM_SAMPLES = 100;
double previous;
double current;
double weight = 0.05;
double error = 10;
int timeout = 100;
double data[NUM_SAMPLES];
int dataIndex = 0;
double sum;
void setup() {
  pinMode(A0, INPUT);
  Serial.begin(38400);
  Serial.setTimeout(50);
  previous = analogRead(A0);
  current = analogRead(A0);
  for (int i = 0; i < NUM_SAMPLES; i++) {
    data[i] = analogRead(A0);
  }
  while (!Serial) continue;
}

void loop() {
  while ((current > abs(previous) + error) && timeout != 0) {
    current = analogRead(A0);
    --timeout;
  }
  timeout = 100;
  //Serial.println(current * weight + (1 - weight) * previous);
  doc["sensor"] = current * weight + (1 - weight) * previous;
  serializeJson(doc, Serial);
  Serial.println();
  previous = current;
  current = analogRead(A0);
  delay(5);
/*
  sum = 0;
  data[dataIndex] = analogRead(A1);
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += data[i];
  }
  Serial.println(sum / (double)NUM_SAMPLES);
  if (dataIndex < NUM_SAMPLES) {
    ++dataIndex;
  }
  else {
    dataIndex = 0;
  }
  */
}
