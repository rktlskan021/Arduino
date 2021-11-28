#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
Servo myservo;

// configurable parameters
#define _DUTY_MIN 1144 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 1692 // servo full counterclockwise position (180 degree)

#define _SERVO_SPEED 100 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

int a, b; // unit: mm
double duty_chg_per_interval; // maximum duty difference per interval
int duty_target;
double duty_curr;
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
unsigned long last_sampling_time_dist;

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = _DUTY_NEU;
// initialize serial port
  Serial.begin(57600);
  a = 68;
  b = 270;
  myservo.writeMicroseconds(duty_curr);
  
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;
  
  // initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  unsigned long time_curr = millis();
////  if(time_curr >= last_sampling_time_dist + INTERVAL){
//      last_sampling_time_dist += INTERVAL;
//  }
  
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(duty_target > duty_curr) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }
  myservo.writeMicroseconds(duty_curr);
  Serial.print("min:100,max:450,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);

  if(dist_cali > 300) {
//    myservo.writeMicroseconds(_DUTY_MIN);
    duty_target = _DUTY_MIN;
  }
  else {
//    myservo.writeMicroseconds(_DUTY_MAX);
      duty_target = _DUTY_MAX;
  }
//  delay(20);
}
