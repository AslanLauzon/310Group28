#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Encoder.h>

// -------------------- Pins --------------------
#define STEP_PIN 9
#define DIR_PIN 10
#define ENCODER_PIN_A A2
#define ENCODER_PIN_B A1

// -------------------- Tuning --------------------
#define STEPS_PER_REV 3200L   // 200 steps * 16 microstepping
#define ENCODER_CPR   2048L   // 512 PPR * 4

// Max random move size (in steps)
#define MAX_JUMP_STEPS 1000

// CSV timing
const uint16_t SAMPLE_PERIOD_MS = 20;
const bool USE_TIME_MS = true;
const bool USE_TIME_S  = false;

// -------------------- Objects --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

elapsedMillis csvTimer;
elapsedMillis runTime;

// -------------------- Helpers --------------------
inline float stepsToDeg(long steps) {
  return (steps * 360.0f) / (float)STEPS_PER_REV;
}
inline float encCountsToDeg(long counts) {
  return (counts * 360.0f) / (float)ENCODER_CPR;
}

void printCsvHeader() {
  if (USE_TIME_MS) {
    Serial.println(F("time_ms,position_step,position_deg"));
  } else if (USE_TIME_S) {
    Serial.println(F("time_s,position_step,position_deg"));
  } else {
    Serial.println(F("time,position_step,position_deg"));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(1000);
  stepper.setCurrentPosition(0);

  encoder.write(0);

  Serial.println(F("Stepper Autonomous Random Mode"));
  printCsvHeader();

  csvTimer = 0;
  runTime = 0;

  randomSeed(analogRead(A0)); // seed RNG from floating pin
  queueRandomMove();
}

void loop() {
  stepper.run();

  // If finished with a move, queue a new random one
  if (!stepper.isRunning() && stepper.distanceToGo() == 0) {
    queueRandomMove();
  }

  // CSV logging
  if (csvTimer >= SAMPLE_PERIOD_MS) {
    csvTimer = 0;
    unsigned long t_ms = runTime;
    long stepPos = stepper.currentPosition();
    long encCnt  = encoder.read();
    float angle_deg = encCountsToDeg(encCnt);

    if (USE_TIME_MS) {
      Serial.print(t_ms);
    } else if (USE_TIME_S) {
      Serial.print(t_ms / 1000.0f, 3);
    } else {
      Serial.print(t_ms);
    }
    Serial.print(',');
    Serial.print(stepPos);
    Serial.print(',');
    Serial.println(angle_deg, 3);
  }
}

// -------------------- Functions --------------------
void queueRandomMove() {
  long delta = random(-MAX_JUMP_STEPS, MAX_JUMP_STEPS + 1);
  stepper.move(delta);
  Serial.print(F("New target: "));
  Serial.print(delta);
  Serial.print(F(" steps ("));
  Serial.print(stepsToDeg(delta), 2);
  Serial.println(F(" deg)"));
}
