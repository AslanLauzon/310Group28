#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Encoder.h>

// -------------------- Pins --------------------
#define STEP_PIN 9
#define DIR_PIN 10
#define ENCODER_PIN_A A2
#define ENCODER_PIN_B A1

// -------------------- Mechanics --------------------
#define ENCODER_CPR 2400
const float STEPS_PER_MM = 42.4f;

// -------------------- Motion goal --------------------
const float GOAL_MM = 120.0f;     // total move
const float FN_HZ   = 1.79f;      // measured natural frequency
const float T_SEC   = 1.0f / FN_HZ;
const float T2_SEC  = 0.5f * T_SEC;

// ZV shaper: gains [0.5, 0.5] at times [0, T/2]
const float seg_mm[2]   = { GOAL_MM * 0.5f, GOAL_MM * 0.5f };
const float seg_time_s[2] = { 0.0f, T2_SEC };

// -------------------- Stepper limits --------------------
// Use highest acceleration you can set
const float A_MAX_STEPS = 74000.0f;   // steps/s^2 (given)
// Choose a high max speed that your driver can reliably generate.
// 10k steps/s meets the 0.96 s constraint for 120 mm with this accel.
const float V_MAX_STEPS = 5800.0f;   // steps/s

// -------------------- Objects --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Timekeeping
elapsedMillis csvTimer;
elapsedMillis runTime;
const uint16_t SAMPLE_PERIOD_MS = 5;

// State
uint8_t nextSeg = 0;
unsigned long t0_ms = 0;

inline float encCountsToDeg(long counts){ return (counts * 360.0f) / (float)ENCODER_CPR; }

void startSeg(float mm){
  long steps = lroundf(mm * STEPS_PER_MM);
  // Relative move. If called during motion, this reshapes the command.
  stepper.move(steps);
}

void setup(){
  Serial.begin(115200);
  while(!Serial){}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  stepper.setAcceleration(A_MAX_STEPS);
  stepper.setMaxSpeed(V_MAX_STEPS);
  stepper.setCurrentPosition(0);

  encoder.write(0);

  Serial.println(F("time_ms,position_step,angle_deg,target_step"));

  // Start timing and first impulse immediately
  t0_ms = millis();
  startSeg(seg_mm[0]);

  csvTimer = 0;
  runTime = 0;
  nextSeg = 1; // second impulse pending
  delay(5000);
}

void loop(){
  stepper.run();

  // Schedule second ZV impulse exactly at T/2
  if(nextSeg < 2){
    float now_s = (millis() - t0_ms) / 1000.0f;
    if(now_s >= seg_time_s[nextSeg]){
      startSeg(seg_mm[nextSeg]);
      nextSeg++;
    }
  }

  // Logging
  if(csvTimer >= SAMPLE_PERIOD_MS){
    csvTimer = 0;
    unsigned long t_ms = runTime;
    long stepPos = stepper.currentPosition();
    long target  = stepper.targetPosition();
    long encCnt  = encoder.read();
    float angle_deg = encCountsToDeg(encCnt);

    Serial.print(t_ms);       Serial.print(',');
    Serial.print(stepPos);    Serial.print(',');
    Serial.print(angle_deg,3);Serial.print(',');
    Serial.println(target);
  }
}
