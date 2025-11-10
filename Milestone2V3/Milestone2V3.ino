/*
  Pendulum Controller
  - Trapezoid move: MOVE REL <mm>
  - Quintic move:   QMOVE REL <mm> <T_ms|AUTO>
  - Auto-reset:     RESET ON|OFF  (wait 3s after a move, then trapezoid back)
  - Auto-record:    AUTOREC ON|OFF (capture starts before move, ends 10s after stop)
  - Buffered logging, CSV dump only after post-hold

  CSV burst format:
    #BEGIN DUMP N
    time_ms,pos_step,angle_deg,target_step
    ...
    #END DUMP
  Move metadata (for filename):
    #MOVE TYPE=<TRAP|QUINTIC|RESET> DIST_MM=<signed_float> T_MS=<int|AUTO>
*/

#include <AccelStepper.h>
#include <Encoder.h>
#include <elapsedMillis.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------- Pins ----------
#define STEP_PIN        9
#define DIR_PIN         10
#define ENCODER_PIN_A   A2
#define ENCODER_PIN_B   A1

// ---------- Mechanics ----------
#define ENCODER_CPR     2400   // encoder counts per rev

// ---------- Objects ----------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// ---------- Limits / scaling ----------
float maxVel_steps_s  = 5400.0f;
float maxAcc_steps_s2 = 74000.0f;
long  stepsPerMM      = 40;

// ---------- Sampling ----------
uint16_t     sample_ms = 5;
elapsedMillis runTime;
elapsedMillis sampleTimer;

// ---------- Auto features ----------
bool autoReset  = false;               // RESET ON|OFF
bool autoRecord = false;               // AUTOREC ON|OFF
const uint32_t POST_HOLD_MS = 10000;   // record 10 s after stop
const uint32_t RESET_WAIT_MS = 3000;   // wait 3 s before reset move

// ---------- Quintic state ----------
bool   quinticActive = false;
float  x0_mm = 0.0f, x1_mm = 0.0f;
unsigned long T_ms = 0, t0_ms = 0;

// ---------- Move bookkeeping ----------
enum MoveType { MT_NONE=0, MT_TRAP=1, MT_QUINTIC=2, MT_RESET=3 };
MoveType currentMove = MT_NONE;
float    currentDist_mm = 0.0f;
long     moveStartSteps = 0;

bool     postHoldActive = false;
elapsedMillis postHoldTimer;
bool     resetPending   = false;
elapsedMillis resetWaitTimer;

// ---------- Buffered logging ----------
struct __attribute__((packed)) Sample {
  uint16_t t_tick;   // time_ms / sample_ms
  int32_t  pos;      // step position
  int32_t  tgt;      // target position
  int32_t  enc;      // encoder counts
};
Sample* buf = nullptr;
size_t  cap = 0;
size_t  n   = 0;
bool    captureOn = false;

// ---------- Free RAM estimator ----------
extern "C" char* sbrk(int);
size_t freeRamBytes(){ char top; return (size_t)(&top - sbrk(0)); }

void allocBuffer(){
  const size_t SAFETY = 8 * 1024; // leave headroom
  size_t freeB = freeRamBytes();
  size_t room  = (freeB > SAFETY) ? (freeB - SAFETY) : 0;
  size_t recB  = sizeof(Sample);
  cap = room / recB;
  if (cap > 12000) cap = 12000;
  if (cap < 500)   cap = 500;
  buf = (Sample*)malloc(cap * recB);
}

void clearBuffer(){ n = 0; }
bool bufferFull(){ return n >= cap; }

inline float encCountsToDeg(long counts){
  return (counts * 360.0f) / (float)ENCODER_CPR;
}

// ---------- Motion helpers ----------
inline void applyLimits(){
  stepper.setMaxSpeed(maxVel_steps_s);
  stepper.setAcceleration(maxAcc_steps_s2);
}

// quintic profile u(s) = 10s^3 - 15s^4 + 6s^5
inline float polyQuintic(float s){
  float s2=s*s, s3=s2*s, s4=s3*s, s5=s4*s;
  return 10*s3 - 15*s4 + 6*s5;
}

void startQuintic(float start_mm, float end_mm, unsigned long dur_ms){
  x0_mm = start_mm; x1_mm = end_mm; T_ms = dur_ms;
  t0_ms = millis(); quinticActive = true;
  currentMove = MT_QUINTIC;
  currentDist_mm = end_mm - start_mm;
}

void serviceQuintic(){
  if (!quinticActive) return;
  unsigned long t = millis() - t0_ms;
  float s = (T_ms == 0) ? 1.0f : (t >= T_ms ? 1.0f : (float)t / (float)T_ms);
  float u = polyQuintic(s);
  float x_mm = x0_mm + (x1_mm - x0_mm) * u;
  long targetSteps = lround(x_mm * stepsPerMM);
  stepper.moveTo(targetSteps);
  if (s >= 1.0f) quinticActive = false;
}

// duration guard for quintic (velocity/acceleration)
unsigned long checkOrClampDuration(float dx_mm,
                                   unsigned long T_req_ms,
                                   bool hardFail,
                                   float safety = 1.08f)
{
  float Vmm = maxVel_steps_s  / (float)stepsPerMM; // mm/s
  float Amm = maxAcc_steps_s2 / (float)stepsPerMM; // mm/s^2
  if (Vmm <= 0 || Amm <= 0) return 0;

  float dx = fabsf(dx_mm);
  if (dx == 0.0f) return (T_req_ms > 0) ? T_req_ms : 10;

  float Tmin_v = 1.875f * dx / Vmm;
  float Tmin_a = sqrtf(5.7735f * dx / Amm);
  float Tmin   = fmaxf(Tmin_v, Tmin_a);

  unsigned long Tmin_ms = (unsigned long)lroundf(1000.0f * Tmin);
  if (Tmin_ms < 10) Tmin_ms = 10;

  if (T_req_ms >= Tmin_ms) return T_req_ms;
  if (hardFail) return 0;

  unsigned long Tauto_ms = (unsigned long)lroundf(Tmin_ms * safety);
  if (Tauto_ms < 10) Tauto_ms = 10;
  return Tauto_ms;
}

// ---------- Recording ----------
void startCapture() {
  captureOn = true;
  runTime = 0;
  sampleTimer = 0;
  clearBuffer();
}

void stopAndDumpCapture() {
  dumpBuffer();
  clearBuffer();
  captureOn = false;
}

void logSample(){
  if (!captureOn || bufferFull()) return;
  if (sampleTimer < sample_ms) return;
  sampleTimer = 0;

  Sample s;
  s.t_tick = (uint16_t)(((uint32_t)runTime) / (uint32_t)sample_ms);
  s.pos    = stepper.currentPosition();
  s.tgt    = stepper.targetPosition();
  s.enc    = encoder.read();
  buf[n++] = s;
}

void dumpBuffer(){
  Serial.print(F("#BEGIN DUMP ")); Serial.println(n);
  Serial.println(F("time_ms,pos_step,angle_deg,target_step"));
  for (size_t k=0; k<n; ++k){
    const Sample& s = buf[k];
    uint32_t t_ms = (uint32_t)s.t_tick * (uint32_t)sample_ms;
    float angle_deg = encCountsToDeg(s.enc);
    Serial.print(t_ms);   Serial.print(',');
    Serial.print(s.pos);  Serial.print(',');
    Serial.print(angle_deg, 3); Serial.print(',');
    Serial.println(s.tgt);
  }
  Serial.println(F("#END DUMP"));
}

// ---------- Serial protocol ----------
void printlnOK(){ Serial.println(F("OK")); }
void printlnERR(const char* e){ Serial.print(F("ERR ")); Serial.println(e); }

bool nextTok(char*& ctx, char* out, size_t n){
  char* tok = strtok_r(nullptr, " \t\r\n", &ctx);
  if (!tok) return false; strncpy(out, tok, n-1); out[n-1] = 0; return true;
}

void announceMove(MoveType t, float dist_mm, const char* tms) {
  const char* s = (t==MT_TRAP)?"TRAP":(t==MT_QUINTIC)?"QUINTIC":"RESET";
  Serial.print(F("#MOVE TYPE=")); Serial.print(s);
  Serial.print(F(" DIST_MM=")); Serial.print(dist_mm, 3);
  Serial.print(F(" T_MS=")); Serial.println(tms);
}

void handleCommand(char* line){
  char* ctx=nullptr;
  char* cmd=strtok_r(line," \t\r\n",&ctx);
  if (!cmd) return;

  if (!strcmp(cmd,"SET")){
    char key[16]; if (!nextTok(ctx,key,sizeof(key))) { printlnERR("SET key"); return; }
    if (!strcmp(key,"VEL"))   { char v[24]; if (!nextTok(ctx,v,sizeof(v))) { printlnERR("VEL"); return; }
      maxVel_steps_s = atof(v); applyLimits(); printlnOK();
    } else if (!strcmp(key,"ACCEL")) { char v[24]; if (!nextTok(ctx,v,sizeof(v))) { printlnERR("ACCEL"); return; }
      maxAcc_steps_s2 = atof(v); applyLimits(); printlnOK();
    } else if (!strcmp(key,"STEPSPERMM")) { char v[24]; if (!nextTok(ctx,v,sizeof(v))) { printlnERR("SPMM"); return; }
      stepsPerMM = atol(v); printlnOK();
    } else if (!strcmp(key,"SAMPLE")) { char v[24]; if (!nextTok(ctx,v,sizeof(v))) { printlnERR("SAMPLE"); return; }
      sample_ms = (uint16_t)atoi(v); printlnOK();
    } else {
      printlnERR("SET key");
    }

  } else if (!strcmp(cmd,"RESET")){
    char v[8]; if (!nextTok(ctx,v,sizeof(v))) { printlnERR("RESET"); return; }
    if (!strcmp(v,"ON"))  { autoReset = true; printlnOK(); }
    else if (!strcmp(v,"OFF")) { autoReset = false; printlnOK(); }
    else { printlnERR("RESET val"); }

  } else if (!strcmp(cmd,"AUTOREC")){
    char v[8]; if (!nextTok(ctx,v,sizeof(v))) { printlnERR("AUTOREC"); return; }
    if (!strcmp(v,"ON"))  { autoRecord = true; printlnOK(); }
    else if (!strcmp(v,"OFF")) { autoRecord = false; printlnOK(); }
    else { printlnERR("AUTOREC val"); }

  } else if (!strcmp(cmd,"MOVE")){
    // Basic trapezoid relative move
    char mode[8]; if (!nextTok(ctx,mode,sizeof(mode))) { printlnERR("mode"); return; }
    if (strcmp(mode,"REL")) { printlnERR("use: MOVE REL <mm>"); return; }
    char a[24]; if (!nextTok(ctx,a,sizeof(a))) { printlnERR("mm"); return; }
    float mm = atof(a);

    moveStartSteps = stepper.currentPosition();
    currentMove = MT_TRAP;
    currentDist_mm = mm;

    announceMove(MT_TRAP, mm, "NA");

    if (autoRecord) startCapture();

    long steps = lround(mm * stepsPerMM);
    stepper.move(steps);            // AccelStepper trapezoid
    quinticActive = false;          // ensure quintic is not driving
    printlnOK();

  } else if (!strcmp(cmd,"QMOVE")){
    // Quintic rest-to-rest relative move with guard and AUTO time option
    char mode[8]; if (!nextTok(ctx,mode,sizeof(mode))) { printlnERR("mode"); return; }
    if (strcmp(mode,"REL")) { printlnERR("use: QMOVE REL <mm> <T_ms|AUTO>"); return; }
    char a[24]; if (!nextTok(ctx,a,sizeof(a))) { printlnERR("mm"); return; }
    char b[24]; if (!nextTok(ctx,b,sizeof(b))) { printlnERR("T_ms|AUTO"); return; }

    float mm = atof(a);
    float start_mm = stepper.currentPosition() / (float)stepsPerMM;
    float end_mm   = start_mm + mm;
    float dx_mm    = end_mm - start_mm;

    unsigned long dur_ms = 0;
    if (!strcmp(b,"AUTO")){
      dur_ms = checkOrClampDuration(dx_mm, 0, /*hardFail=*/false, /*safety=*/1.08f);
      if (dur_ms == 0) { printlnERR("AUTO_FAIL"); return; }
      announceMove(MT_QUINTIC, mm, "AUTO");
      Serial.print(F("INFO T_MS ")); Serial.println(dur_ms);
    } else {
      dur_ms = (unsigned long)atol(b);
      const bool HARD_FAIL = true;
      unsigned long checked = checkOrClampDuration(dx_mm, dur_ms, HARD_FAIL, 1.08f);
      if (checked == 0) { printlnERR("TIME_TOO_SHORT_FOR_LIMITS"); return; }
      if (!HARD_FAIL && checked != dur_ms){
        Serial.print(F("WARN CLAMPED_T_MS ")); Serial.println(checked);
      }
      announceMove(MT_QUINTIC, mm, b);
      dur_ms = checked;
    }

    moveStartSteps = stepper.currentPosition();
    currentMove = MT_QUINTIC;
    currentDist_mm = mm;
    if (autoRecord) startCapture();

    startQuintic(start_mm, end_mm, dur_ms);
    printlnOK();

  } else if (!strcmp(cmd,"DUMP")){
    stopAndDumpCapture(); printlnOK();

  } else if (!strcmp(cmd,"CLEAR")){
    clearBuffer(); printlnOK();

  } else if (!strcmp(cmd,"ZEROENC")){
    encoder.write(0); printlnOK();

  } else if (!strcmp(cmd,"HOME")){
    stepper.setCurrentPosition(0); printlnOK();

  } else if (!strcmp(cmd,"STATUS")){
    Serial.print(F("VEL "));         Serial.println(maxVel_steps_s);
    Serial.print(F("ACCEL "));       Serial.println(maxAcc_steps_s2);
    Serial.print(F("STEPSPERMM "));  Serial.println(stepsPerMM);
    Serial.print(F("SAMPLE "));      Serial.println(sample_ms);
    Serial.print(F("POS "));         Serial.println(stepper.currentPosition());
    Serial.print(F("TARGET "));      Serial.println(stepper.targetPosition());
    Serial.print(F("QUINTIC "));     Serial.println(quinticActive ? "ACTIVE" : "IDLE");
    Serial.print(F("AUTOREC "));     Serial.println(autoRecord ? "ON" : "OFF");
    Serial.print(F("RESET "));       Serial.println(autoReset ? "ON" : "OFF");
    Serial.print(F("CAPACITY "));    Serial.println((unsigned long)cap);
    Serial.print(F("NSAMP "));       Serial.println((unsigned long)n);
    Serial.print(F("MOVING "));      Serial.println((stepper.distanceToGo()!=0 || quinticActive) ? "YES":"NO");
    printlnOK();

  } else {
    printlnERR("cmd");
  }
}

void serviceSerial(){
  static char buf[96]; static size_t idx=0;
  while (Serial.available()){
    char c = Serial.read();
    if (c=='\n' || c=='\r'){
      if (idx){ buf[idx]=0; handleCommand(buf); idx=0; }
    } else if (idx < sizeof(buf)-1){
      buf[idx++] = c;
    }
  }
}

// ---------- Setup/loop ----------
void setup(){
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  applyLimits();
  stepper.setCurrentPosition(0);
  encoder.write(0);

  allocBuffer();

  Serial.println(F("# Ready. MOVE REL, QMOVE REL, AUTOREC, RESET, buffered logging"));
  if (!buf) Serial.println(F("# ERROR: malloc failed"));
  else {
    Serial.print(F("# Buffer capacity: "));
    Serial.print((unsigned long)cap);
    Serial.print(F(" records @ SAMPLE="));
    Serial.print(sample_ms);
    Serial.println(F(" ms"));
  }
}

void loop(){
  serviceSerial();
  serviceQuintic();
  stepper.run();

  // capture to RAM
  logSample();

  // Motion state machine
  static bool wasMoving = false;
  bool movingNow = (quinticActive || (stepper.distanceToGo()!=0));

  // Transition: moving -> stopped
  if (wasMoving && !movingNow) {
    if (autoRecord) {
      postHoldActive = true;
      postHoldTimer = 0;
    } else if (autoReset) {
      resetPending = true;
      resetWaitTimer = 0;
    }
  }
  wasMoving = movingNow;

  // Post-hold completion
  if (postHoldActive && postHoldTimer >= POST_HOLD_MS) {
    stopAndDumpCapture();
    postHoldActive = false;
    if (autoReset) {
      resetPending = true;
      resetWaitTimer = 0;
    }
  }

  // Reset sequence (only after recording finished, if enabled)
  if (resetPending && !movingNow && !postHoldActive) {
    if (resetWaitTimer >= RESET_WAIT_MS) {
      long backSteps = moveStartSteps; // absolute target
      float dist_mm = (backSteps - stepper.currentPosition()) / (float)stepsPerMM;

      // Announce and optionally record this reset move
      announceMove(MT_RESET, dist_mm, "NA");
      if (autoRecord) startCapture();

      stepper.moveTo(backSteps);       // trapezoid back to start
      quinticActive = false;
      resetPending = false;
    }
  }
}
