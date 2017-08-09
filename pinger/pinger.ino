// Arduino Pins for HC-SR04 distance sensor (using analog 4 and 5 as digital)
#define PINGTRIGGER 3
#define PINGRETURN  4

void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(PINGTRIGGER, OUTPUT);
  pinMode(PINGRETURN, INPUT);
  pinMode(13, OUTPUT);
}

int distance = 1000;

void loop() {
  // put your main code here, to run repeatedly:
  distance = ping_distance();
  Serial.println(distance);
  if (distance < 50) {
      digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
  delay(50);
}

long ping_distance() {
  long duration, distance;
  digitalWrite(PINGTRIGGER, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(PINGTRIGGER, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(PINGTRIGGER, LOW);
  duration = pulseIn(PINGRETURN, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}
