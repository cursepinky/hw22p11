#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9         // LED for Bang-Bang control
#define PIN_SERVO 10      // Servo pin
#define PIN_IR A0         // Infrared distance sensor pin

// Event interval parameters
#define _INTERVAL_DIST    20  // Distance sensor interval (ms)
#define _INTERVAL_SERVO   20  // Servo interval (ms)
#define _INTERVAL_SERIAL  100 // Serial output interval (ms)

// EMA filter configuration
#define _EMA_ALPHA 0.9        // EMA smoothing factor

// Servo adjustment parameters
#define _DUTY_NEU 1700  // Servo angle: 0 degree
#define _DUTY_MAX 2100  // Servo angle: D degree
#define _DUTY_MIN 670   // Servo angle: E degree
#define _SERVO_ANGLE_DIFF 90
#define _SERVO_SPEED 150  // Reduced servo speed (degrees/second)

#define _BANGBANG_RANGE 100   // Reduced range for less aggressive control

// Global variables
float dist_filtered, dist_ema, dist_target = 175; // Target: center of the rail (mm)
Servo myservo;
int duty_change_per_interval, duty_target, duty_curr;
unsigned long last_sampling_time_dist = 0, last_sampling_time_servo = 0, last_sampling_time_serial = 0;
bool event_dist = false, event_servo = false, event_serial = false;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _DUTY_NEU;

  duty_change_per_interval = 
    (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / (float)_SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0);

  Serial.begin(1000000);
}

void loop() {
  unsigned long time_curr = millis();

  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  if (event_dist) {
    event_dist = false;
    dist_filtered = volt_to_distance(ir_sensor_filtered(10, 0.5));
    dist_ema = _EMA_ALPHA * dist_ema + (1.0 - _EMA_ALPHA) * dist_filtered;

    float deadband = 5.0; // ±5 mm tolerance
    if (abs(dist_target - dist_ema) <= deadband) {
        duty_target = _DUTY_NEU; // Neutral position within deadband
    } else if (dist_target < dist_ema) {
        duty_target = _DUTY_MIN + _BANGBANG_RANGE;
        digitalWrite(PIN_LED, LOW);
    } else {
        duty_target = _DUTY_MAX - _BANGBANG_RANGE;
        digitalWrite(PIN_LED, HIGH);
    }
  }

  if (event_servo) {
    event_servo = false;
    if (duty_target > duty_curr) {
      duty_curr += duty_change_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    } else {
      duty_curr -= duty_change_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }
    if (duty_curr > _DUTY_MAX) duty_curr = _DUTY_MAX;
    if (duty_curr < _DUTY_MIN) duty_curr = _DUTY_MIN;

    myservo.writeMicroseconds(duty_curr);
  }

  if (event_serial) {
    event_serial = false;
    Serial.print("TARGET:"); Serial.print(dist_target);
    Serial.print(", DIST:"); Serial.println(dist_ema);
  }
}

float volt_to_distance(int a_value) {
  return 1466 + (-8.28 * a_value) + 0.0165 * pow(a_value, 2) - (1.13E-5) * pow(a_value, 3);
}

unsigned int ir_sensor_filtered(unsigned int n, float position) {
  unsigned int *ir_val = (unsigned int*) malloc(sizeof(unsigned int) * n);
  for (int i = 0; i < n; i++) ir_val[i] = analogRead(PIN_IR);
  qsort(ir_val, n, sizeof(unsigned int), compare);
  unsigned int ret_val = ir_val[(unsigned int)(n * position)];
  free(ir_val);
  return ret_val;
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}
