/*
  Kitty Kaan Cat Ear Firmware
  Board: Seeed XIAO RA4M1
  IMU: MPU6050/GY-521 style module
  Servos: 4 expressive cat-ear servos

  Schematic pin mapping:
    SERVO_D1  -> D0
    IMU_INT   -> D1
    HAPTIC    -> D2, unused here
    SERVO_D2  -> D3
    SDA       -> D4
    SCL       -> D5
    SERVO_EN  -> D6
    SERVO_D4  -> D7
    SERVO_D3  -> D10
*/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#if defined(ARDUINO_ARCH_RENESAS)
  #include <Arduino.h>
#endif

// -------------------- Pins --------------------

const int PIN_SERVO_1 = D0;
const int PIN_IMU_INT = D1;
const int PIN_HAPTIC  = D2;
const int PIN_SERVO_2 = D3;
const int PIN_SERVO_EN = D6;
const int PIN_SERVO_4 = D7;
const int PIN_SERVO_3 = D10;

const int PIN_STATUS_LED = LED_BUILTIN;

// Battery ADC pin.
// If your XIAO core exposes PIN_VBAT or BATTERY_PIN, this will use it.
// If not, battery monitoring will safely default to 100%.
#if defined(PIN_VBAT)
const int PIN_BATTERY_ADC = PIN_VBAT;
#elif defined(BATTERY_PIN)
const int PIN_BATTERY_ADC = BATTERY_PIN;
#elif defined(VBAT_ENABLE)
const int PIN_BATTERY_ADC = VBAT_ENABLE;
#else
const int PIN_BATTERY_ADC = -1;
#endif

// -------------------- Hardware --------------------

Adafruit_MPU6050 mpu;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

volatile bool imuInterruptSeen = false;

// -------------------- Tuning --------------------

// Servo center positions.
// Adjust these if your mechanical center is not exactly 90.
int center1 = 90;
int center2 = 90;
int center3 = 90;
int center4 = 90;

// Servo travel limits.
const int SERVO_MIN = 35;
const int SERVO_MAX = 145;

// Overall expressiveness.
const float PITCH_GAIN = 0.95f;
const float ROLL_GAIN  = 0.80f;
const float SWAY_GAIN  = 1.15f;

// Smoothness.
// Higher = smoother/slower.
const float SERVO_SMOOTHING = 0.13f;

// Movement thresholds.
const float ACTIVE_ACCEL_DELTA_G = 0.065f;
const float ACTIVE_GYRO_DPS      = 4.0f;

// Sleep after this much stillness.
const unsigned long SLEEP_AFTER_MS = 90000UL;

// Low battery cutoff.
const float LOW_BATTERY_PERCENT = 20.0f;

// LiPo voltage range.
const float LIPO_EMPTY_V = 3.30f;
const float LIPO_FULL_V  = 4.20f;

// If your battery ADC reads through a divider, set this.
// Example: 2.0 if ADC sees half the battery voltage.
// For built-in VBAT sensing this is often already handled by the core,
// but leave this here for easy adjustment.
const float BATTERY_DIVIDER_RATIO = 1.0f;

// XIAO / Arduino ADC reference assumption.
// Adjust only if your board core reports a different ADC scale.
const float ADC_REFERENCE_V = 3.3f;
const int ADC_MAX_COUNTS = 4095;

// Servo update rate.
const unsigned long SERVO_UPDATE_MS = 20;

// -------------------- State --------------------

float servoPos1 = 90;
float servoPos2 = 90;
float servoPos3 = 90;
float servoPos4 = 90;

float sway = 0.0f;
float swayVelocity = 0.0f;

unsigned long lastMotionMs = 0;
unsigned long lastServoUpdateMs = 0;
unsigned long lastBatteryCheckMs = 0;
unsigned long lastLowBatteryBlinkMs = 0;

float batteryPercent = 100.0f;
bool lowBattery = false;
bool servosEnabled = false;

// -------------------- Helpers --------------------

void imuISR() {
  imuInterruptSeen = true;
}

int clampServo(int v) {
  if (v < SERVO_MIN) return SERVO_MIN;
  if (v > SERVO_MAX) return SERVO_MAX;
  return v;
}

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float lipoVoltageToPercent(float v) {
  // Simple single-cell LiPo percentage curve.
  // This is intentionally conservative around the lower end.
  if (v >= 4.20f) return 100.0f;
  if (v <= 3.30f) return 0.0f;

  if (v >= 4.10f) return 90.0f + (v - 4.10f) * 100.0f;
  if (v >= 4.00f) return 75.0f + (v - 4.00f) * 150.0f;
  if (v >= 3.90f) return 55.0f + (v - 3.90f) * 200.0f;
  if (v >= 3.80f) return 35.0f + (v - 3.80f) * 200.0f;
  if (v >= 3.70f) return 20.0f + (v - 3.70f) * 150.0f;
  if (v >= 3.60f) return 10.0f + (v - 3.60f) * 100.0f;
  return (v - 3.30f) * 33.33f;
}

float readBatteryVoltage() {
  if (PIN_BATTERY_ADC < 0) {
    return 4.20f;
  }

  analogReadResolution(12);

  const int samples = 16;
  long total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(PIN_BATTERY_ADC);
    delay(2);
  }

  float raw = total / float(samples);
  float measured = (raw / ADC_MAX_COUNTS) * ADC_REFERENCE_V;
  return measured * BATTERY_DIVIDER_RATIO;
}

void updateBatteryStatus() {
  unsigned long now = millis();

  if (now - lastBatteryCheckMs < 5000UL) {
    return;
  }

  lastBatteryCheckMs = now;

  float vbat = readBatteryVoltage();
  batteryPercent = lipoVoltageToPercent(vbat);
  lowBattery = batteryPercent < LOW_BATTERY_PERCENT;
}

void enableServos() {
  if (servosEnabled) return;

  digitalWrite(PIN_SERVO_EN, HIGH);
  delay(100);

  servo1.attach(PIN_SERVO_1);
  servo2.attach(PIN_SERVO_2);
  servo3.attach(PIN_SERVO_3);
  servo4.attach(PIN_SERVO_4);

  servosEnabled = true;
}

void disableServos() {
  if (!servosEnabled) return;

  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();

  delay(20);
  digitalWrite(PIN_SERVO_EN, LOW);

  servosEnabled = false;
}

void writeServoSmooth(Servo &s, float &current, int target) {
  current += (target - current) * SERVO_SMOOTHING;
  s.write(clampServo((int)(current + 0.5f)));
}

void centerEarsAndStop() {
  enableServos();

  for (int i = 0; i < 60; i++) {
    writeServoSmooth(servo1, servoPos1, center1);
    writeServoSmooth(servo2, servoPos2, center2);
    writeServoSmooth(servo3, servoPos3, center3);
    writeServoSmooth(servo4, servoPos4, center4);
    delay(20);
  }

  disableServos();
}

void blinkLowBattery() {
  unsigned long now = millis();

  if (now - lastLowBatteryBlinkMs >= 500UL) {
    lastLowBatteryBlinkMs = now;
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
  }
}

void configureMPUMotionInterrupt() {
  // Motion interrupt is used to wake from idle/light sleep.
  // Threshold is in MPU6050 motion units, not g directly.
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(3);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

void clearMPUInterrupt() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  imuInterruptSeen = false;
}

void enterLightSleepUntilMotion() {
  centerEarsAndStop();

  clearMPUInterrupt();
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuISR, RISING);

  digitalWrite(PIN_STATUS_LED, LOW);

  while (!imuInterruptSeen) {
    updateBatteryStatus();

    if (lowBattery) {
      blinkLowBattery();
      delay(100);
      continue;
    }

    // ARM light sleep. An external interrupt wakes execution here.
    #if defined(__arm__) || defined(ARDUINO_ARCH_RENESAS)
      __DSB();
      __WFI();
      __ISB();
    #else
      delay(250);
    #endif
  }

  detachInterrupt(digitalPinToInterrupt(PIN_IMU_INT));
  clearMPUInterrupt();

  lastMotionMs = millis();
  digitalWrite(PIN_STATUS_LED, HIGH);
}

// -------------------- Setup --------------------

void setup() {
  pinMode(PIN_SERVO_EN, OUTPUT);
  digitalWrite(PIN_SERVO_EN, LOW);

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);

  pinMode(PIN_HAPTIC, OUTPUT);
  digitalWrite(PIN_HAPTIC, LOW);

  pinMode(PIN_IMU_INT, INPUT);

  Wire.begin();

  if (!mpu.begin()) {
    // IMU failed. Blink fast forever.
    while (1) {
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
      delay(120);
    }
  }

  // IMU tuning.
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  configureMPUMotionInterrupt();

  enableServos();

  servo1.write(center1);
  servo2.write(center2);
  servo3.write(center3);
  servo4.write(center4);

  servoPos1 = center1;
  servoPos2 = center2;
  servoPos3 = center3;
  servoPos4 = center4;

  lastMotionMs = millis();
}

// -------------------- Main Loop --------------------

void loop() {
  updateBatteryStatus();

  if (lowBattery) {
    centerEarsAndStop();

    while (true) {
      updateBatteryStatus();
      blinkLowBattery();

      if (!lowBattery) {
        enableServos();
        lastMotionMs = millis();
        break;
      }

      delay(100);
    }
  }

  unsigned long now = millis();

  if (now - lastServoUpdateMs < SERVO_UPDATE_MS) {
    return;
  }

  lastServoUpdateMs = now;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x / 9.80665f;
  float ay = accel.acceleration.y / 9.80665f;
  float az = accel.acceleration.z / 9.80665f;

  float gx = gyro.gyro.x * 57.2958f;
  float gy = gyro.gyro.y * 57.2958f;
  float gz = gyro.gyro.z * 57.2958f;

  float accelMag = sqrt(ax * ax + ay * ay + az * az);
  float accelDelta = fabs(accelMag - 1.0f);
  float gyroMag = sqrt(gx * gx + gy * gy + gz * gz);

  bool moving = accelDelta > ACTIVE_ACCEL_DELTA_G || gyroMag > ACTIVE_GYRO_DPS;

  if (moving) {
    lastMotionMs = now;
  }

  if (now - lastMotionMs > SLEEP_AFTER_MS) {
    enterLightSleepUntilMotion();
    return;
  }

  enableServos();

  // Estimate head tilt.
  float pitchDeg = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958f;
  float rollDeg  = atan2(ay, az) * 57.2958f;

  pitchDeg = clampFloat(pitchDeg, -35.0f, 35.0f);
  rollDeg  = clampFloat(rollDeg, -35.0f, 35.0f);

  // Springy sway from head motion.
  // Gyro Z gives a nice side-to-side laggy ear motion.
  float swayForce = gz * 0.018f;
  swayVelocity += swayForce;
  swayVelocity *= 0.88f;
  sway += swayVelocity;
  sway *= 0.92f;

  sway = clampFloat(sway, -28.0f, 28.0f);

  float pitchMove = pitchDeg * PITCH_GAIN;
  float rollMove  = rollDeg  * ROLL_GAIN;
  float swayMove  = sway     * SWAY_GAIN;

  // Four-servo expressive mapping.
  //
  // Assumption:
  //   servo1/servo2 = one ear pair
  //   servo3/servo4 = other ear pair
  //
  // If an ear moves backwards from what you want,
  // flip the sign for that servo below.
  int target1 = center1 + pitchMove + rollMove + swayMove;
  int target2 = center2 - pitchMove + rollMove + swayMove * 0.65f;
  int target3 = center3 + pitchMove - rollMove - swayMove;
  int target4 = center4 - pitchMove - rollMove - swayMove * 0.65f;

  target1 = clampServo(target1);
  target2 = clampServo(target2);
  target3 = clampServo(target3);
  target4 = clampServo(target4);

  writeServoSmooth(servo1, servoPos1, target1);
  writeServoSmooth(servo2, servoPos2, target2);
  writeServoSmooth(servo3, servoPos3, target3);
  writeServoSmooth(servo4, servoPos4, target4);
}
