//declare variables and constants
const byte ledPin = 3;
const byte interruptPin = 2;
volatile byte state = LOW;
int val=0;

void setup() 
{
  //assign pins and begin serial connection
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), test, RISING);
  Serial.begin(9600);
}

void loop() 
{
  digitalWrite(ledPin, state);
  Serial.println(val/2);
}

void test() 
{
  state = !state;
  val++;
}
