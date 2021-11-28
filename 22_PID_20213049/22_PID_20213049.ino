#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9  //LED in 9
#define PIN_SERVO 10 // SERVO in 10
#define PIN_IR A0  // Infrared sensor signal in A0

// Framework setting
#define _DIST_MIN 110
#define _DIST_TARGET 315  // Target distance = 25.5cm
#define _DIST_MAX 400

// Distance sensor
#define _DIST_ALPHA 0.35  // EMA filter

// global variables
const float coE[] = {0.0000204, -0.0136853, 4.0482617, -156.8686997};

// Servo range
#define _DUTY_MIN 1212  // SERVO DUTY's minimum 
#define _DUTY_NEU 1412  // SERVO DUTY's neutral 
#define _DUTY_MAX 1612  // SERVO DUTY's maximum 

// Servo speed control
#define _SERVO_ANGLE 30  // SERVO ANGLE 
#define _SERVO_SPEED 100  // SERVO SPEED 
#define _RAMPUP_TIME 360 // servo speed rampup (0 to max) time (unit: ms)


// Event periods
#define _INTERVAL_DIST 15 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _Kp 1.0 // PID Propotional gain for P
#define _Ki 0.0075  // PID Propotional gain for I
#define _Kd 850  // PID Propotional gain for D

// global variables 
float dist_min, dist_max, dist_raw, dist_ema, alpha;

// Servo instance
Servo myservo; 

// Distance sensor
float dist_target;  // location to send the ball

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval,duty_chg_max,duty_chg_adjust,toggle_interval, toggle_interval_cnt;

float pause_time; // unit: sec
int duty_target, duty_curr;


#define START _DUTY_MIN + 100
#define END _DUTY_MAX - 100

// PID variables
float error_curr, error_prev, control, P_term, I_term, D_term;
float D_termS;

// Filter
#define _INTERVAL_DIST 30
#define DELAY_MICROS  1500
float filtered_dist;
float samples_num = 3;  

void setup() {
  // initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO); 
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_target = _DIST_TARGET;
  alpha = _DIST_ALPHA;

  duty_target = duty_curr;

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
  Serial.begin(115200);


// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE) * ((float)_INTERVAL_SERVO / 1000); //[3128]

// servo related variables
  //duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
  //duty_chg_adjust = (float) duty_chg_max * _INTERVAL_SERVO / _RAMPUP_TIME;
  //duty_chg_per_interval = 0; // initial speed is set to 0.


// initialize variables for duty_target update.
  pause_time = 0.5;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / _INTERVAL_SERVO;
  toggle_interval = 1; // to demonstrate overshoot
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time_servo = 0;
}
  
void loop() {  // Event generator

  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) {
    
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

// Event handlers
  if(event_dist) {
 
    event_dist = false;
  // get a distance reading from the distance sensor
  
    dist_raw = filtered_ir_distance();
  // PID control logic
    error_curr = dist_target - dist_raw;
    P_term = _Kp * error_curr;
    I_term += _Ki * error_curr;
    D_term = _Kd * (error_curr - error_prev) / 20;
    control = P_term + D_term + I_term;

    //update error_prev
    error_prev = error_curr;
    D_termS = D_term;
    if (D_termS > 800) D_termS = 800;

    
  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;
    
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
      if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
      if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
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
    
    Serial.print("IR:");
    Serial.print(dist_raw );
    Serial.print(" ,T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(P_term,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(D_termS,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(I_term,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245, +G:265, m:0, M:800");
    
  
  }
}

float ir_distance(void){ // return value unit: mm 
  //distance check : Infrared sensor 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {

    float x = ir_distance();
    currReading = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
    
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){

  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }

  dist_ema = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*dist_ema;
  return dist_ema;
}
