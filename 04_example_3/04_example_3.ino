#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial){
    ;
  }
  Serial.println("Hello World");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle); // toggle LED value
  digitalWrite(PIN_LED, toggle); // update LED status
  delay(1000);
}

int toggle_state(int toggle){
  if(toggle){
    return 0;
  }
  else{
    return 1;
  }
}
