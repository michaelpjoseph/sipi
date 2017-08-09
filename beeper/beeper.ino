
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);  // primary beeper
  pinMode(12, OUTPUT);  // optional secondary beeper
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(13, HIGH);   // turn the pin on (HIGH is the voltage level)
  //digitalWrite(12, LOW);
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the pin off by making the voltage LOW
   //digitalWrite(12, HIGH);
  delay(1000);              // wait for a second
}

  
