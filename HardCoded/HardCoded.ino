#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Encoder.h>
#include <math.h>

/* ============================================================
   =============== HARDWARE PIN DEFINITIONS ====================
   ============================================================ */
#define STEP_PIN        9     // Step signal to DRV8825 driver
#define DIR_PIN         10    // Direction signal to DRV8825 driver
#define ENCODER_PIN_A   A2    // Encoder channel A input
#define ENCODER_PIN_B   A1    // Encoder channel B input

/* ============================================================
   ================= MECHANICAL PARAMETERS =====================
   ============================================================ */
#define ENCODER_CPR     2400  // Encoder counts per full revolution (quadrature)

/* ============================================================
   ================== MOTION CONFIGURATION =====================
   ============================================================ */
const float PROFILE_SPEED  = 5400.0f;   // Max step rate [steps/s]
const float PROFILE_ACCEL  = 74000.0f;  // Step acceleration [steps/s^2]
const long  STEPSPERMM     = 40;        // Conversion: 1 mm motion = 40 steps
const long  GOAL           = 105;       // Target travel distance [mm]
const long  PROFILE_DIST   = 400;  // Target move in steps

/* ============================================================
   ====================== OBJECT CREATION ======================
   ============================================================ */
// Stepper motor driver object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Quadrature encoder object
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Elapsed time trackers (auto-incrementing)
elapsedMillis csvTimer;   // Controls data logging rate
elapsedMillis runTime;    // Tracks total elapsed time

// Logging frequency: 5 ms = 200 Hz sampling
const uint16_t SAMPLE_PERIOD_MS = 5;

/* ============================================================
   ====================== LOGGING ARRAYS =======================
   ============================================================ */
// Preallocate memory for up to 5000 samples (≈25 s at 200 Hz)
const size_t MAX_SAMPLES = 5000;
unsigned long t_array[MAX_SAMPLES];     // Time [ms]
long          stepPosArray[MAX_SAMPLES]; // Stepper position [steps]
long          targetArray[MAX_SAMPLES];  // Target position [steps]
float         angleArray[MAX_SAMPLES];   // Encoder-measured angle [deg]
size_t        i = 0;                     // Sample counter

/* ============================================================
   ===================== HELPER FUNCTIONS ======================
   ============================================================ */

// Convert encoder counts to mechanical degrees
inline float encCountsToDeg(long counts) {
  return (counts * 360.0f) / (float)ENCODER_CPR;
}

/* ============================================================
   ========================= SETUP =============================
   ============================================================ */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // Wait for serial to initialize (USB boards)

  // Configure step and direction pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Configure stepper motion limits
  stepper.setMaxSpeed(PROFILE_SPEED);
  stepper.setAcceleration(PROFILE_ACCEL);

  // Reset internal counters
  stepper.setCurrentPosition(0);
  encoder.write(0);

  // Print CSV header for clarity
  Serial.println(F("time_ms,position_step,position_deg,target_step"));

  // Schedule a motion to the target position
  stepper.move(PROFILE_DIST);

  // Reset timers
  csvTimer = 0;
  runTime  = 0;

  // 5-second delay before starting (setup stabilization)
  delay(5000);
}

/* ============================================================
   ========================== LOOP =============================
   ============================================================ */
void loop() {
  /* ---------------------- Stepper Update ---------------------- */
  // This must be called as often as possible for smooth motion
  stepper.run();

  /* ---------------------- Data Logging ------------------------ */
  // Log every SAMPLE_PERIOD_MS (e.g. every 5 ms)
  if (csvTimer >= SAMPLE_PERIOD_MS) {
    csvTimer = 0;

    // Capture instantaneous data
    unsigned long t_ms   = runTime;                    // Elapsed time [ms]
    long stepPos          = stepper.currentPosition();  // Commanded step position
    long target           = stepper.targetPosition();   // Current move target
    long encCnt           = encoder.read();             // Encoder count
    float angle_deg       = encCountsToDeg(encCnt);     // Convert to degrees

    // Stream data live via Serial as CSV
    Serial.print(t_ms);
    Serial.print(',');
    Serial.print(stepPos);
    Serial.print(',');
    Serial.print(angle_deg, 3);  // Three decimal places
    Serial.print(',');
    Serial.println(target);

    // (Optional) could store into arrays if offline analysis is needed
    // but currently data is streamed directly.
  }

  /* -------------------- Stop Condition ------------------------ */
  // Stop logging and execution after 14 seconds
  if (runTime > 14000) {
    Serial.println("STOP");
    while (1); // Halt program indefinitely
  }
}
