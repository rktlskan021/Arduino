#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

#define _DIST_TARGET 255  //탁구공위치
#define _DIST_MIN 100   //거리 최소
#define _DIST_MAX 410    //거리 최대
#define _DIST_ALPHA 0.35 //EMA 필터링을 위한 알파값(0~1)

#define _DUTY_MIN 1600    
#define _DUTY_NEU 1250  
#define _DUTY_MAX 900    

#define _SERVO_ANGLE 30.0  
#define _SERVO_SPEED 600

#define _INTERVAL_DIST 30  //각 event 사이에 지정한 시간 간격
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

#define DELAY_MICROS  1500

#define _KP 0.07

#define a 78
#define b 312

Servo myservo;

float dist_target; // location to send the ball
float dist_raw;
float dist_ema=0; //측정된 값과 ema 필터를 적용한 값
float samples_num = 3;  

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
//각 event의 진행 시간 저장 변수 
bool event_dist, event_servo, event_serial; 
//각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))

// Servo speed control
int duty_chg_per_interval; //주기 당 서보 duty값 변화량
int duty_target, duty_curr; //목표 위치와 현재 위치
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo 
myservo.attach(PIN_SERVO); // attach servo
pinMode(PIN_LED,OUTPUT); // initialize GPIO pins

// initialize global variables

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU);
duty_curr = _DUTY_NEU;

// initialize serial port
Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MIN - _DUTY_MAX) * (_SERVO_SPEED / _SERVO_ANGLE ) * (_INTERVAL_SERVO / 1000.0);
}
  

void loop() {
/////////////////////

// Event generator //
///////////////////// 

unsigned long time_curr = millis();
if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
}

if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
}

if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
}

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
     dist_ema = ir_distance_filtered();

  // PID control logic
    error_curr = dist_ema - _DIST_TARGET;
    pterm = error_curr;
    iterm = 0; 
    dterm = 0;
    control = _KP * (-pterm) + iterm + dterm;

duty_target = _DUTY_NEU + control * ((control>0)?(_DUTY_MIN - _DUTY_NEU):(_DUTY_NEU - _DUTY_MAX))  * _SERVO_ANGLE / 180;


  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }
    else if(duty_target < _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
  }
  
  if(event_servo) {
    event_servo=false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
  duty_curr += duty_chg_per_interval;
  if(duty_curr > duty_target) duty_curr = duty_target;
     }
    else {
  duty_curr -= duty_chg_per_interval;
  if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
     myservo.writeMicroseconds(duty_curr);
  }   

  if(event_serial) {
    event_serial = false; 
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 300.0 / (b - a) * (val - a) + 100;
}
// ================
float under_noise_filter(void){ 
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float ir_distance_filtered(void){ 
  int currReading;
  int lowestReading = 1024;
  dist_raw = ir_distance(); 
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }

  dist_ema = _DIST_ALPHA *lowestReading + (1-_DIST_ALPHA )*dist_ema;
  return dist_ema;
}
