// Barebones: send target distance (mm) over Serial, it moves there, then prints angle (deg)

#include <AccelStepper.h>
#include <Encoder.h>

// ---- Pins ----
#define STEP_PIN 9
#define DIR_PIN 10
#define ENCODER_PIN_A A2
#define ENCODER_PIN_B A1

// ---- Motion/geometry ----
#define STEPS_PER_MM 42.4    // set for your mechanics (e.g., 16x microstep + 2 mm pitch -> 1600/2 = 800)
#define ENCODER_CPR  2048L   // counts per revolution (quadrature *4)

// ---- Objects ----
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

void setup() {
  Serial.begin(115200);
  stepper.setMaxSpeed(5000);       // tune
  stepper.setAcceleration(74000);   // tune
  stepper.setCurrentPosition(0);
  encoder.write(0);                // zero angle at start
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // parse distance in mm
    float dist_mm = line.toFloat();
    long target_steps = lround(dist_mm * STEPS_PER_MM);

    // move and wait until done
    stepper.moveTo(target_steps);
    while (stepper.distanceToGo() != 0) stepper.run();

    // read angle from encoder and print degrees
    long counts = encoder.read();
    float angle_deg = 360.0f * (float)counts / (float)ENCODER_CPR;
    Serial.println(angle_deg, 3);  // prints e.g., 12.345
  }
}
