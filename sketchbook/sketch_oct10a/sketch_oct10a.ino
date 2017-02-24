const byte ledPin = 13;
const byte interruptPin = 2;
volatile int counter = 0;
int analogPin1=0;
int analogPin2=1;
int val1=0;
int val2=0;

void setup() {
  pinMode(ledPin, OUTPUT);
  attachInterrupt(2, blink, CHANGE);
  Serial.begin(9600);
}

void loop() {
  
 val1= analogRead(analogPin1);
 val2= analogRead(analogPin2);
  
  Serial.println(val2);
  delay(10);

}

void blink() {
  counter=counter+1;
}
