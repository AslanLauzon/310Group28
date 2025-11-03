#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Encoder.h>

/* ============================== Pins ============================== */
#define STEP_PIN        9
#define DIR_PIN         10
#define ENCODER_PIN_A   A2
#define ENCODER_PIN_B   A1
#define ENABLE_PIN      8     // set USE_ENABLE=1 if wired
#define USE_ENABLE      0     // 1 = drive ENABLE pin, 0 = ignore

/* ============================ Mechanics =========================== */
#define ENCODER_CPR     2400

/* ======================= Motion configuration ===================== */
const float   MAX_SPEED_STEPS_S = 5000.0f;   // steps/s
const float   ACCEL_STEPS_S2    = 74000.0f;  // steps/s^2
const long    JOG1_STEPS        = 115L * 40L;
const long    JOG2_STEPS        = 120L * 40L;
const uint32_t MOVE_PAUSE_MS    = 277;       // pause between moves

/* ============================ Logging ============================= */
const uint16_t SAMPLE_PERIOD_MS = 5;         // 5 ms = 200 Hz
const uint32_t LOG_DURATION_MS  = 12000;     // dump and stop at 15 s
const uint16_t MAX_SAMPLES      = LOG_DURATION_MS / SAMPLE_PERIOD_MS; // 3000

/* ============================= Objects ============================ */
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
elapsedMillis csvTimer;
elapsedMillis runTime;
elapsedMillis stateTimer;

/* ============================== State ============================= */
enum class RunState : uint8_t { Move1, Pause, Move2, Done };
RunState state = RunState::Move1;

/* ============================ Buffers ============================= */
uint32_t t_array[MAX_SAMPLES];
long     stepPos_array[MAX_SAMPLES];
float    angleDeg_array[MAX_SAMPLES];
long     target_array[MAX_SAMPLES];
uint16_t sampleCount = 0;
bool     dumped = false;

/* ============================= Helpers ============================ */
inline float encCountsToDeg(long counts) {
  return (counts * 360.0f) / (float)ENCODER_CPR;
}
inline void scheduleRelativeMove(long deltaSteps) {
  stepper.moveTo(stepper.currentPosition() + deltaSteps);
}

/* ============================== Setup ============================= */
void setup() {
  Serial.begin(115200);

  stepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  stepper.setAcceleration(ACCEL_STEPS_S2);
#if USE_ENABLE
  stepper.setEnablePin(ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);  // invert ENABLE if active-LOW
  stepper.enableOutputs();
#endif
  stepper.setCurrentPosition(0);
  encoder.write(0);

  // start first move
  scheduleRelativeMove(JOG1_STEPS);
  csvTimer = 0;
  runTime  = 0;
  stateTimer = 0;

  // settle if needed
  delay(5000);
}

/* =============================== Loop ============================= */
void loop() {
  // drive the stepper every loop
  stepper.run();

  // ---------- State machine ----------
  switch (state) {
    case RunState::Move1:
      if (stepper.distanceToGo() == 0) {
        state = RunState::Pause;
        stateTimer = 0;
      }
      break;

    case RunState::Pause:
      if (stateTimer >= MOVE_PAUSE_MS) {
        scheduleRelativeMove(JOG2_STEPS);
        state = RunState::Move2;
      }
      break;

    case RunState::Move2:
      if (stepper.distanceToGo() == 0) {
#if USE_ENABLE
        stepper.disableOutputs(); // optional
#endif
        state = RunState::Done;
      }
      break;

    case RunState::Done:
      // no further moves
      break;
  }

  // ---------- Sample into buffers ----------
  if (csvTimer >= SAMPLE_PERIOD_MS && sampleCount < MAX_SAMPLES) {
    csvTimer = 0;

    const uint32_t t_ms   = runTime;
    const long stepPos    = stepper.currentPosition();
    const long target     = stepper.targetPosition();
    const long encCnt     = encoder.read();
    const float angle_deg = encCountsToDeg(encCnt);

    t_array[sampleCount]        = t_ms;
    stepPos_array[sampleCount]  = stepPos;
    angleDeg_array[sampleCount] = angle_deg;
    sampleCount++;
  }

  // ---------- Dump once at 15 s ----------
  if (!dumped && runTime >= LOG_DURATION_MS) {
    dumped = true;

    // header
    Serial.println(F("time_ms,position_step,position_deg,target_step"));

    // rows
    for (uint16_t i = 0; i < sampleCount; i++) {
      Serial.print(t_array[i]);         Serial.print(',');
      Serial.print(stepPos_array[i]);   Serial.print(',');
      Serial.print(angleDeg_array[i], 3); Serial.print(',');
    }

    Serial.println(F("STOP"));

    // halt
    while (1) { /* stop */ }
  }
}
