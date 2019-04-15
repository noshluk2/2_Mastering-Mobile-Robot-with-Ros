int r_enc=D1;
int interruptCounter = 0;

void setup() {
  // put your setup code here, to run once:
   pinMode(r_enc, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(r_enc), handleInterrupt, RISING);
   Serial.begin(115200);
}

void loop() {
  

}

 void handleInterrupt() {
  interruptCounter++;
  Serial.print("Window number : ");
  Serial.println(interruptCounter);
}
