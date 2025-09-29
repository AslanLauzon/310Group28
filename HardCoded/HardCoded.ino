#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Encoder.h>

// -------------------- Pins --------------------
#define STEP_PIN 9
#define DIR_PIN 10
#define ENCODER_PIN_A A2
#define ENCODER_PIN_B A1

// -------------------- Mechanics --------------------
#define STEPS_PER_REV 3200L   // adjust for your motor & microstepping
#define ENCODER_CPR   2048L   // adjust for your encoder

// -------------------- Motion profile --------------------
const float PROFILE_SPEED = 4000.0f;   // steps/s
const float PROFILE_ACCEL = 70000.0f;  // steps/s^2
const long  PROFILE_DIST  = 5000;      // steps (relative move)

// -------------------- Objects --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Timekeeping
elapsedMillis csvTimer;
elapsedMillis runTime;
const uint16_t SAMPLE_PERIOD_MS = 20; // log at 50 Hz

// -------------------- Helpers --------------------
inline float encCountsToDeg(long counts) {
  return (counts * 360.0f) / (float)ENCODER_CPR;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Stepper setup
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  stepper.setMaxSpeed(PROFILE_SPEED);
  stepper.setAcceleration(PROFILE_ACCEL);
  stepper.setCurrentPosition(0);

  // Encoder setup
  encoder.write(0);

  // CSV header
  Serial.println(F("time_ms,position_step,position_deg,target_step"));

  // Start profile move
  stepper.move(PROFILE_DIST);

  csvTimer = 0;
  runTime = 0;
  delay(10000);
}

void loop() {
  stepper.run(); // drive motor

  if (csvTimer >= SAMPLE_PERIOD_MS) {
    csvTimer = 0;

    unsigned long t_ms = runTime;
    long stepPos = stepper.currentPosition();
    long target  = stepper.targetPosition();
    long encCnt  = encoder.read();
    float angle_deg = encCountsToDeg(encCnt);

    Serial.print(t_ms);
    Serial.print(',');
    Serial.print(stepPos);
    Serial.print(',');
    Serial.print(angle_deg, 3);
    Serial.print(',');
    Serial.println(target);
  }
}
