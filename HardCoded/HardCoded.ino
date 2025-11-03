#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Encoder.h>
#include <math.h>

// -------------------- Pins --------------------
#define STEP_PIN 9
#define DIR_PIN 10
#define ENCODER_PIN_A A2
#define ENCODER_PIN_B A1

// -------------------- Mechanics --------------------
#define ENCODER_CPR 2400

// -------------------- Motion profile --------------------
const float PROFILE_SPEED  = 5400.0f;   // steps/s
const float PROFILE_ACCEL  = 74000.0f;  // steps/s^2
const long  STEPSPERMM     = 40;        // steps/mm
const long  GOAL           = -120;      // mm
const long  PROFILE_DIST   = GOAL * STEPSPERMM;  // steps (relative move)

// -------------------- Objects --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Timekeeping
elapsedMillis csvTimer;
elapsedMillis runTime;
const uint16_t SAMPLE_PERIOD_MS = 5;  // 200 Hz

// -------------------- Logging buffers --------------------
const size_t MAX_SAMPLES = 5000;
unsigned long t_array[MAX_SAMPLES];
long          stepPosArray[MAX_SAMPLES];
long          targetArray[MAX_SAMPLES];
float         angleArray[MAX_SAMPLES];
size_t        i = 0;

// -------------------- Helpers --------------------
inline float encCountsToDeg(long counts) {
  return (counts * 360.0f) / (float)ENCODER_CPR;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  stepper.setMaxSpeed(PROFILE_SPEED);
  stepper.setAcceleration(PROFILE_ACCEL);
  stepper.setCurrentPosition(0);

  encoder.write(0);

  // CSV header
  Serial.println(F("time_ms,position_step,position_deg,target_step"));

  // Start profile move
  stepper.move(PROFILE_DIST);

  csvTimer = 0;
  runTime = 0;
  delay(5000);
}

void loop() {
  stepper.run();  // drive motor

  if (csvTimer >= SAMPLE_PERIOD_MS) {
    csvTimer = 0;

    // Acquire signals once per sample
    unsigned long t_ms = runTime;
    long stepPos = stepper.currentPosition();
    long target  = stepper.targetPosition();
    long encCnt  = encoder.read();
    float angle_deg = encCountsToDeg(encCnt);

    // Stream CSV
    Serial.print(t_ms);
    Serial.print(',');
    Serial.print(stepPos);
    Serial.print(',');
    Serial.print(angle_deg, 3);
    Serial.print(',');
    Serial.println(target);
  }

  if (runTime > 14000) {
  Serial.println("STOP");
  while(1);
  } 



  // Stop condition if desired
  // if (stepper.distanceToGo() == 0) { /* no-op or disable driver */ }
}

