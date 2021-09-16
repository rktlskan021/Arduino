#define PIN7 7

void setup() {
  pinMode(PIN7, OUTPUT);
}

void loop() {
  digitalWrite(PIN7, HIGH);
  delay(1000);
  for(int i = 0;i<5;i++){
      digitalWrite(PIN7, LOW);
    delay(100);
    digitalWrite(PIN7, HIGH);
    delay(100);
  }
  digitalWrite(PIN7, LOW);
  while(1){
    
  }
}
