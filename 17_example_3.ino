#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    0         // IR sensor at Pin A0
#define PIN_LED   9         // LED pin
#define PIN_SERVO 10       // Servo pin

// Servo duty cycle values
#define _DUTY_MIN 1000     // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500     // servo neutral position (90 degree)
#define _DUTY_MAX 2000     // servo full counter-clockwise position (180 degree)

// Distance range
#define _DIST_MIN  100.0   // minimum distance 100mm
#define _DIST_MAX  250.0   // maximum distance 250mm

// EMA filter coefficient (alpha)
#define EMA_ALPHA  0.1    // Alpha for EMA filter (adjustable)

// Loop interval (unit: msec)
#define LOOP_INTERVAL 20  // 20 msec loop interval

Servo myservo;             // Servo object
unsigned long last_loop_time = 0;   // unit: msec

float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  
  Serial.begin(1000000);  // 1,000,000 bps for Serial Monitor
}

void loop() {
  unsigned long time_curr = millis();
  int duty;
  float a_value, dist_raw;

  // Wait until next event time (to control loop interval)
  if (time_curr < (last_loop_time + LOOP_INTERVAL)) 
    return;
  
  last_loop_time = time_curr;

  a_value = analogRead(PIN_IR);  // Read IR sensor value
  dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0 - 60.0;  // Convert raw value to distance
  
  // Apply range filter: only accept values between _DIST_MIN and _DIST_MAX
  if (dist_raw < _DIST_MIN || dist_raw > _DIST_MAX) {
    dist_raw = dist_prev;  // Use the previous distance if out of range
    digitalWrite(PIN_LED, LOW);  // Turn off LED if out of range
  } else {
    digitalWrite(PIN_LED, HIGH);  // Turn on LED if in range
  }

  // Apply EMA filter to smooth the distance readings
  dist_ema = EMA_ALPHA * dist_raw + (1 - EMA_ALPHA) * dist_prev;
  
  // Map the filtered distance to the servo duty cycle (without using map())
  // The distance ranges from _DIST_MIN (100mm) to _DIST_MAX (250mm)
  // The duty cycle ranges from _DUTY_MIN (1000us) to _DUTY_MAX (2000us)

  if (dist_ema <= _DIST_MIN) {
    duty = _DUTY_MIN;  // Minimum duty (0 degree)
  } else if (dist_ema >= _DIST_MAX) {
    duty = _DUTY_MAX;  // Maximum duty (180 degree)
  } else {
    // Calculate the duty cycle using linear interpolation
    duty = _DUTY_MIN + (dist_ema - _DIST_MIN) * (_DUTY_MAX - _DUTY_MIN) / (_DIST_MAX - _DIST_MIN);
  }

  myservo.writeMicroseconds(duty);  // Move the servo to the calculated position

  // Print debugging information to the Serial Monitor
  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print("_DIST_MIN:");  Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println("");

  // Update previous distance for the next loop
  dist_prev = dist_ema;
}
