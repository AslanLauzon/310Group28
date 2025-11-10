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
const float MAX_SPEED_STEPS_S   = 5000.0f;   // steps/s
const float ACCEL_STEPS_S2      = 74000.0f;  // steps/s^2
const long  JOG1_STEPS          = 155L * 40L;
const long  JOG2_STEPS          = 5L * 40L;
const uint32_t MOVE_PAUSE_MS    = 554;       // pause between moves

/* ============================ Logging ============================= */
const uint16_t SAMPLE_PERIOD_MS = 5;         // 5 ms = 200 Hz

/* ============================= Objects ============================ */
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
elapsedMillis csvTimer;
elapsedMillis runTime;
elapsedMillis stateTimer;

/* ============================== State ============================= */
enum class RunState : uint8_t { Move1, Pause, Move2, Done };
RunState state = RunState::Move1;

/* ============================= Helpers ============================ */
inline float encCountsToDeg(long counts) {
  return (counts * 360.0f) / (float)ENCODER_CPR;
}

inline void scheduleRelativeMove(long deltaSteps) {
  stepper.moveTo(stepper.currentPosition() + deltaSteps);
}

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

  Serial.println(F("time_ms,position_step,position_deg,target_step"));

  // Start first move
  scheduleRelativeMove(JOG1_STEPS);
  csvTimer = 0;
  runTime  = 0;
  stateTimer = 0;
  delay(5000);
}

void loop() {
  // Always drive the stepper
  stepper.run();

  // ---------- State machine ----------
  switch (state) {
    case RunState::Move1:
      if (stepper.distanceToGo() == 0) {
        state = RunState::Pause;
        stateTimer = 0;                 // start inter-move pause
        savedTime = time_ms
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
        stepper.disableOutputs();       // optional: remove holding torque
#endif
        state = RunState::Done;         // finished
      }
      break;

    case RunState::Done:
      // no further moves
      break;
  }

  // ---------- Logging ----------
  if (csvTimer >= SAMPLE_PERIOD_MS) {
    csvTimer = 0;

    const unsigned long t_ms = runTime;
    const long stepPos       = stepper.currentPosition();
    const long target        = stepper.targetPosition();
    const long encCnt        = encoder.read();
    const float angle_deg    = encCountsToDeg(encCnt);

    Serial.print(t_ms);        Serial.print(',');
    Serial.print(stepPos);     Serial.print(',');
    Serial.print(angle_deg, 3);Serial.print(',');
    Serial.println(target);
  }

  if (runTime > 15000){
    Serial.println("STOP");
    while(1);
  }
}
