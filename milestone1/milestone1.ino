#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Encoder.h>  // PJRC Encoder library (Install via Library Manager)

// -------------------- Pins --------------------
#define STEP_PIN 9
#define DIR_PIN 10
// Quadrature encoder on analog pins:
#define ENCODER_PIN_A A2
#define ENCODER_PIN_B A1

// -------------------- Tuning --------------------
// Motor steps per mechanical rev (include microstepping & gearing)
#define STEPS_PER_REV 3200L   // e.g., 200-step motor @ 1/16 microstep = 3200
// Encoder counts-per-rev using 4x decoding (A/B both edges)
#define ENCODER_CPR   2048L   // e.g., 512 PPR => CPR = 512*4 = 2048

// CSV timing
const uint16_t SAMPLE_PERIOD_MS = 20;  // 50 Hz stream
// Choose time column units: set ONE true
const bool USE_TIME_MS = true;
const bool USE_TIME_S  = false;

// -------------------- Objects --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Serial command buffer
String inputLine;

// Timekeepers
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
  while (!Serial) { }

  // Motion setup
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  stepper.setMaxSpeed(1500);     // rpm-equivalent depends on steps/second; this is in steps/s
  stepper.setAcceleration(1000); // steps/s^2
  stepper.setCurrentPosition(0);

  // Encoder starts at 0
  encoder.write(0);

  Serial.println(F("Stepper Command Mode"));
  Serial.println(F("Type a number of steps (e.g. 200 or -200) and press Enter"));
  Serial.println(F("Type 'z' to zero step & encoder positions"));

  printCsvHeader();
  csvTimer = 0;
  runTime = 0;
}

void loop() {
  // Keep the stepper moving toward its target
  stepper.run();

  // Handle serial input
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;  // ignore CR
    if (c == '\n') {
      handleCommand(inputLine);
      inputLine = "";
    } else {
      inputLine += c;
    }
  }

  // CSV streaming at fixed period
  if (csvTimer >= SAMPLE_PERIOD_MS) {
    csvTimer = 0;

    // Time
    unsigned long t_ms = runTime;

    // Positions
    long stepPos = stepper.currentPosition(); // commanded steps
    long encCnt  = encoder.read();            // measured encoder counts

    // Convert encoder to degrees
    float angle_deg = encCountsToDeg(encCnt);

    // Print CSV
    if (USE_TIME_MS) {
      Serial.print(t_ms);
    } else if (USE_TIME_S) {
      // print as seconds with 3 decimal places
      Serial.print(t_ms / 1000.0f, 3);
    } else {
      Serial.print(t_ms); // fallback
    }
    Serial.print(',');
    Serial.print(stepPos);
    Serial.print(',');
    Serial.println(angle_deg, 3);
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Zero command
  if (cmd.equalsIgnoreCase("z")) {
    stepper.setCurrentPosition(0);
    encoder.write(0);
    Serial.println(F("Zeroed stepper and encoder positions."));
    return;
  }

  // Numeric move (relative steps)
  long steps = cmd.toInt();
  if (steps == 0 && cmd != "0") {
    Serial.println(F("Invalid input. Type a number like 200 or -200, or 'z' to zero."));
    return;
  }

  stepper.move(steps);
  Serial.print(F("Moving "));
  Serial.print(steps);
  Serial.print(F(" steps ("));
  Serial.print(stepsToDeg(steps), 2);
  Serial.println(F(" deg)"));
}
