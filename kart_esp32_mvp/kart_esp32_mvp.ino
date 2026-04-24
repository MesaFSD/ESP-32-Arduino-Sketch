/* =============================================================================
 *  FSD Autonomous Go-Kart — ESP32 Low-Level Actuator Controller (Tier 1 MVP)
 * =============================================================================
 *
 *  Role:
 *    This sketch runs on the ESP32 WROOM and is the actuation + safety layer
 *    between the Pixhawk Cube Orange+ (running ArduRover) and the physical
 *    actuators on the kart.
 *
 *  Signal flow:
 *    Cube PWM  -->  ESP32 reads PWM  -->  ESP32 drives actuators
 *       steering PWM  -->  PID with encoder feedback  -->  Talon SRX
 *       throttle PWM  -->  12-bit value  -->  MCP4725 DAC  -->  EZkontrol 0-5V
 *       throttle PWM <1500us  -->  reverse sequencing  -->  EZkontrol reverse
 *       failsafe / RC brake    -->  brake solenoid (active-LOW)
 *
 *  What the ESP32 is NOT doing:
 *    No navigation, no GPS parsing, no mission logic. The Cube handles all
 *    of that. This sketch only translates Cube PWM commands into the right
 *    signal type for each actuator, plus closed-loop steering and failsafe.
 *
 *  Libraries required (install via Arduino Library Manager):
 *    - ESP32Servo            (PWM output to Talon SRX)
 *    - Adafruit_MCP4725      (DAC driver)
 *    - Wire                  (built-in, I2C)
 *
 *  Board selection in Arduino IDE:
 *    Tools -> Board -> "ESP32 Dev Module"
 * =============================================================================
 */

#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ESP32Servo.h>

// =============================================================================
//  PIN ASSIGNMENTS — TODO: FILL IN BEFORE FLASHING
// =============================================================================
//  Current board labels:
//    D34  -> Pixhawk throttle PWM input  (confirmed)
//    D18  -> Brake solenoid signal       (confirmed, active-LOW)
//    D19  -> Reverse signal to EZkontrol (confirmed)
//    GPIO 21 -> I2C SDA to MCP4725       (confirmed)
//    GPIO 22 -> I2C SCL to MCP4725       (confirmed)
//    RX2 / TX2 -> Pixhawk telemetry      (wired but unused in MVP)
//
//  TO ASSIGN — leave placeholders until hardware is confirmed:
// -----------------------------------------------------------------------------

// #define PIN_CUBE_STEERING_PWM   XX   // Pixhawk steering PWM input (input-only pin suggested, e.g. GPIO 35)
// #define PIN_CUBE_THROTTLE_PWM   34   // Pixhawk throttle PWM input (already on D34)
// #define PIN_STEERING_PWM_OUT    XX   // PWM output to Talon SRX (LEDC-capable pin, e.g. GPIO 25)
// #define PIN_ENCODER_A           XX   // REV encoder channel A (interrupt-capable, e.g. GPIO 32)
// #define PIN_ENCODER_B           XX   // REV encoder channel B (interrupt-capable, e.g. GPIO 33)
// #define PIN_BRAKE_OUT           18   // Brake solenoid signal (confirmed D18, active-LOW)
// #define PIN_REVERSE_OUT         19   // Reverse signal to EZkontrol (confirmed D19)

// Placeholders so the sketch compiles — REPLACE with real pin numbers before flashing.
#define PIN_CUBE_STEERING_PWM    0
#define PIN_CUBE_THROTTLE_PWM   34
#define PIN_STEERING_PWM_OUT     0
#define PIN_ENCODER_A            0
#define PIN_ENCODER_B            0
#define PIN_BRAKE_OUT           18
#define PIN_REVERSE_OUT         19

// =============================================================================
//  TUNABLE CONSTANTS — adjust via serial commands or edit and re-flash
// =============================================================================

// --- PWM input range from Cube (standard servo pulse width) ---
const uint16_t PWM_MIN_US        = 1000;   // full reverse / hard left / brake off equivalent
const uint16_t PWM_MAX_US        = 2000;   // full forward / hard right / brake on equivalent
const uint16_t PWM_NEUTRAL_US    = 1500;   // center / zero throttle
const uint16_t PWM_DEADBAND_US   = 40;     // around 1500 us, treat as zero

// --- Failsafe timing ---
const uint32_t FAILSAFE_TIMEOUT_MS = 500;  // no valid PWM pulse for this long = failsafe

// --- Throttle calibration (MCP4725 output) ---
// Option (c): parameterized defaults matching the old Mega behavior (0.59 - 1.00 V)
// Derived from old Mega PWM range 30-51 out of 255 on a 5V rail.
// Update THROTTLE_DAC_MIN and THROTTLE_DAC_MAX once real calibration is done.
// DAC value = (desired_voltage / 5.0) * 4095
//   0.59 V  -> DAC 483
//   1.00 V  -> DAC 819
//   4.20 V  -> DAC 3440   (standard hall throttle max — use once calibrated)
uint16_t THROTTLE_DAC_MIN   = 483;         // DAC value at "barely moving"
uint16_t THROTTLE_DAC_MAX   = 819;         // DAC value at "max desired speed" (conservative default)
const uint16_t DAC_ZERO     = 0;           // 0 V output = throttle off

// --- Throttle ramp rate (prevent current spikes + smooth direction changes) ---
const uint16_t THROTTLE_RAMP_STEP = 15;    // DAC units added per control loop toward target
                                            // At 200 Hz loop, 15 units/step = ~3000/sec = ~1 sec full scale

// --- Steering PID gains (carried over from proven Mega values) ---
float Kp = 0.8f;
float Ki = 0.01f;
float Kd = 0.2f;

// --- Steering software end-stops (TODO: calibrate on actual kart) ---
// Units are encoder counts. 2048 CPR x 4 (quadrature) = 8192 counts/rev.
// During calibration: rotate steering full-left, record count; full-right, record count;
// then set these with a safety margin pulled in from each end.
int32_t STEER_COUNT_CENTER = 0;            // encoder count at straight-ahead
int32_t STEER_COUNT_MIN    = -2000;        // PLACEHOLDER — replace with real calibration
int32_t STEER_COUNT_MAX    =  2000;        // PLACEHOLDER — replace with real calibration
const int32_t STEER_DEADBAND_COUNTS = 5;   // within this many counts of target, PID output = 0

// --- Steering PWM output range to Talon SRX ---
const uint16_t STEER_PWM_MIN_US = 1000;
const uint16_t STEER_PWM_MAX_US = 2000;
const uint16_t STEER_PWM_NEUTRAL_US = 1500;
const bool INVERT_STEERING = false;        // flip if left/right are reversed

// --- Control loop timing ---
const uint32_t CONTROL_LOOP_PERIOD_MS = 5;  // 200 Hz loop

// =============================================================================
//  GLOBAL STATE
// =============================================================================

Adafruit_MCP4725 dac;
Servo steeringServo;

// PWM input capture state (updated in ISRs)
volatile uint32_t cubeSteeringRiseTime_us = 0;
volatile uint16_t cubeSteeringPulse_us    = PWM_NEUTRAL_US;
volatile uint32_t cubeSteeringLastRx_ms   = 0;

volatile uint32_t cubeThrottleRiseTime_us = 0;
volatile uint16_t cubeThrottlePulse_us    = PWM_NEUTRAL_US;
volatile uint32_t cubeThrottleLastRx_ms   = 0;

// Encoder state (updated in ISRs)
volatile int32_t encoderCount = 0;
volatile uint8_t encoderLastState = 0;

// PID state
float pidIntegral    = 0.0f;
float pidLastError   = 0.0f;
uint32_t pidLastTime_us = 0;

// Throttle ramp state
int16_t currentThrottleDAC = DAC_ZERO;     // signed so we can track direction

// Reverse state machine
enum ReverseState { REV_FORWARD, REV_RAMPING_TO_ZERO, REV_REVERSE };
ReverseState reverseState = REV_FORWARD;
bool reverseCommanded = false;

// Safety state
bool inFailsafe = true;                    // start in failsafe until valid PWM arrives

// Loop timing
uint32_t lastControlLoop_ms = 0;

// =============================================================================
//  INTERRUPT HANDLERS — PWM input capture
// =============================================================================
//  On each edge, record rise time. On falling edge, compute pulse width.
//  ESP32 has IRAM_ATTR requirement for ISRs.

void IRAM_ATTR onCubeSteeringEdge() {
  uint32_t now_us = micros();
  if (digitalRead(PIN_CUBE_STEERING_PWM) == HIGH) {
    cubeSteeringRiseTime_us = now_us;
  } else {
    uint32_t pulse = now_us - cubeSteeringRiseTime_us;
    // Sanity check: ignore anything outside reasonable servo pulse range
    if (pulse >= 800 && pulse <= 2200) {
      cubeSteeringPulse_us  = (uint16_t)pulse;
      cubeSteeringLastRx_ms = millis();
    }
  }
}

void IRAM_ATTR onCubeThrottleEdge() {
  uint32_t now_us = micros();
  if (digitalRead(PIN_CUBE_THROTTLE_PWM) == HIGH) {
    cubeThrottleRiseTime_us = now_us;
  } else {
    uint32_t pulse = now_us - cubeThrottleRiseTime_us;
    if (pulse >= 800 && pulse <= 2200) {
      cubeThrottlePulse_us  = (uint16_t)pulse;
      cubeThrottleLastRx_ms = millis();
    }
  }
}

// =============================================================================
//  INTERRUPT HANDLERS — Quadrature encoder
// =============================================================================
//  Simple state-machine decoder. For 2048 CPR encoder wired to interrupts
//  on both A and B, this yields 4x decoding = 8192 counts/rev.

void IRAM_ATTR onEncoderChange() {
  uint8_t a = digitalRead(PIN_ENCODER_A);
  uint8_t b = digitalRead(PIN_ENCODER_B);
  uint8_t state = (a << 1) | b;

  // Lookup table for quadrature transitions:
  // Rows = previous state, cols = new state. Values: +1, -1, or 0 (invalid/same).
  static const int8_t transitionTable[4][4] = {
    //   00  01  10  11
    {    0, +1, -1,  0 },  // from 00
    {   -1,  0,  0, +1 },  // from 01
    {   +1,  0,  0, -1 },  // from 10
    {    0, -1, +1,  0 }   // from 11
  };

  encoderCount += transitionTable[encoderLastState][state];
  encoderLastState = state;
}

// =============================================================================
//  HELPERS
// =============================================================================

// Constrain an integer safely
int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Map PWM pulse width (1000-2000 us) to a normalized value in [-1.0, +1.0]
// with deadband around center.
float pwmToNormalized(uint16_t pulse_us) {
  int32_t delta = (int32_t)pulse_us - (int32_t)PWM_NEUTRAL_US;
  if (abs(delta) < (int32_t)PWM_DEADBAND_US) return 0.0f;
  // Remove deadband so the usable range is continuous
  if (delta > 0) delta -= PWM_DEADBAND_US;
  else           delta += PWM_DEADBAND_US;
  float half_range = (float)((PWM_MAX_US - PWM_NEUTRAL_US) - PWM_DEADBAND_US);
  float norm = (float)delta / half_range;
  if (norm >  1.0f) norm =  1.0f;
  if (norm < -1.0f) norm = -1.0f;
  return norm;
}

// Put everything in a safe state. Called on boot and on failsafe.
void enterSafeState() {
  // Throttle off
  dac.setVoltage(DAC_ZERO, false);
  currentThrottleDAC = DAC_ZERO;
  // Brake ON (active-LOW)
  digitalWrite(PIN_BRAKE_OUT, LOW);
  // Reverse OFF
  digitalWrite(PIN_REVERSE_OUT, LOW);
  reverseState = REV_FORWARD;
  // Steering centered
  steeringServo.writeMicroseconds(STEER_PWM_NEUTRAL_US);
  // Reset PID state so it doesn't kick when we re-arm
  pidIntegral  = 0.0f;
  pidLastError = 0.0f;
}

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n[ESP32 Kart Controller] Booting..."));

  // --- Configure digital outputs first, so we start in safe state ---
  pinMode(PIN_BRAKE_OUT,   OUTPUT);
  pinMode(PIN_REVERSE_OUT, OUTPUT);
  digitalWrite(PIN_BRAKE_OUT,   LOW);   // brake engaged (active-LOW)
  digitalWrite(PIN_REVERSE_OUT, LOW);   // reverse off

  // --- Start I2C + DAC ---
  Wire.begin(21, 22);                   // SDA=21, SCL=22 (ESP32 defaults, explicit for clarity)
  if (!dac.begin(0x60)) {               // 0x60 is the default MCP4725 address; check your module
    Serial.println(F("[ERROR] MCP4725 not found on I2C. Check wiring and address."));
    // Stay in safe state — do not proceed.
    while (true) { delay(1000); }
  }
  dac.setVoltage(DAC_ZERO, false);      // throttle output = 0 V immediately

  // --- Configure PWM inputs ---
  pinMode(PIN_CUBE_STEERING_PWM, INPUT);
  pinMode(PIN_CUBE_THROTTLE_PWM, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_CUBE_STEERING_PWM), onCubeSteeringEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CUBE_THROTTLE_PWM), onCubeThrottleEdge, CHANGE);

  // --- Configure encoder inputs ---
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  encoderLastState = (digitalRead(PIN_ENCODER_A) << 1) | digitalRead(PIN_ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), onEncoderChange, CHANGE);

  // --- Configure steering PWM output ---
  ESP32PWM::allocateTimer(0);           // reserve a hardware timer for the Servo library
  steeringServo.setPeriodHertz(50);     // 50 Hz = standard servo frame rate
  steeringServo.attach(PIN_STEERING_PWM_OUT, STEER_PWM_MIN_US, STEER_PWM_MAX_US);
  steeringServo.writeMicroseconds(STEER_PWM_NEUTRAL_US);

  // --- Final safe state ---
  enterSafeState();
  inFailsafe = true;

  pidLastTime_us = micros();
  lastControlLoop_ms = millis();

  Serial.println(F("[ESP32 Kart Controller] Boot complete. Waiting for Cube PWM..."));
  Serial.println(F("Serial commands (runtime tuning):"));
  Serial.println(F("  k <Kp> <Ki> <Kd>       set PID gains"));
  Serial.println(F("  t <dac_min> <dac_max>  set throttle DAC range"));
  Serial.println(F("  s                      print status"));
}

// =============================================================================
//  MAIN LOOP — fixed-rate control at CONTROL_LOOP_PERIOD_MS
// =============================================================================

void loop() {
  // Handle any serial tuning commands (non-blocking)
  handleSerialCommands();

  uint32_t now_ms = millis();
  if (now_ms - lastControlLoop_ms < CONTROL_LOOP_PERIOD_MS) return;
  lastControlLoop_ms = now_ms;

  // ---- 1. Snapshot volatile inputs ----
  noInterrupts();
  uint16_t steerPulse_us    = cubeSteeringPulse_us;
  uint16_t throttlePulse_us = cubeThrottlePulse_us;
  uint32_t steerLastRx_ms   = cubeSteeringLastRx_ms;
  uint32_t throttleLastRx_ms = cubeThrottleLastRx_ms;
  int32_t  currentCount     = encoderCount;
  interrupts();

  // ---- 2. Failsafe check ----
  bool steerStale    = (now_ms - steerLastRx_ms)    > FAILSAFE_TIMEOUT_MS;
  bool throttleStale = (now_ms - throttleLastRx_ms) > FAILSAFE_TIMEOUT_MS;
  if (steerStale || throttleStale) {
    if (!inFailsafe) {
      Serial.println(F("[FAILSAFE] Cube PWM lost — entering safe state."));
      inFailsafe = true;
    }
    enterSafeState();
    return;                             // skip normal control this cycle
  } else {
    if (inFailsafe) {
      Serial.println(F("[FAILSAFE] Cube PWM recovered — resuming normal control."));
      inFailsafe = false;
    }
  }

  // ---- 3. Throttle + reverse sequencing ----
  // Cube sends single throttle PWM: >1500 = forward, <1500 = reverse, ~1500 = zero.
  float throttleNorm = pwmToNormalized(throttlePulse_us);  // -1..+1
  bool  wantReverse  = (throttleNorm < 0.0f);
  float throttleMag  = fabs(throttleNorm);                 // 0..1

  // State machine: must ramp to zero before toggling reverse pin.
  switch (reverseState) {
    case REV_FORWARD:
      if (wantReverse) {
        // Start ramping down — do not toggle reverse yet
        reverseState = REV_RAMPING_TO_ZERO;
      }
      break;
    case REV_RAMPING_TO_ZERO:
      if (currentThrottleDAC <= DAC_ZERO + THROTTLE_RAMP_STEP) {
        // Reached zero — safe to toggle direction
        digitalWrite(PIN_REVERSE_OUT, wantReverse ? HIGH : LOW);
        reverseState = wantReverse ? REV_REVERSE : REV_FORWARD;
      }
      break;
    case REV_REVERSE:
      if (!wantReverse) {
        reverseState = REV_RAMPING_TO_ZERO;
      }
      break;
  }

  // Compute target DAC value (only applies actual motion when in correct direction state)
  uint16_t targetDAC;
  bool directionMatches = (reverseState == REV_FORWARD && !wantReverse) ||
                          (reverseState == REV_REVERSE &&  wantReverse);

  if (directionMatches && throttleMag > 0.0f) {
    // Map magnitude to DAC range
    targetDAC = THROTTLE_DAC_MIN +
                (uint16_t)(throttleMag * (float)(THROTTLE_DAC_MAX - THROTTLE_DAC_MIN));
  } else {
    targetDAC = DAC_ZERO;                // ramping through zero or idle
  }

  // Ramp current DAC toward target
  if (currentThrottleDAC < (int16_t)targetDAC) {
    currentThrottleDAC += THROTTLE_RAMP_STEP;
    if (currentThrottleDAC > (int16_t)targetDAC) currentThrottleDAC = targetDAC;
  } else if (currentThrottleDAC > (int16_t)targetDAC) {
    currentThrottleDAC -= THROTTLE_RAMP_STEP;
    if (currentThrottleDAC < (int16_t)targetDAC) currentThrottleDAC = targetDAC;
  }
  dac.setVoltage((uint16_t)currentThrottleDAC, false);

  // ---- 4. Brake control ----
  // MVP minimal behavior: brake is OFF during normal operation, ON in failsafe.
  // (Failsafe path above already forces brake ON.)
  // TODO: wire a dedicated Cube brake channel or RC passthrough if desired.
  digitalWrite(PIN_BRAKE_OUT, HIGH);     // release brake during active control

  // ---- 5. Steering: PID to encoder target ----
  float steerNorm = pwmToNormalized(steerPulse_us);  // -1..+1
  if (INVERT_STEERING) steerNorm = -steerNorm;

  // Map normalized steering to target encoder count, within software limits.
  int32_t range = (steerNorm >= 0.0f)
                    ? (STEER_COUNT_MAX - STEER_COUNT_CENTER)
                    : (STEER_COUNT_CENTER - STEER_COUNT_MIN);
  int32_t targetCount = STEER_COUNT_CENTER + (int32_t)(steerNorm * (float)range);
  targetCount = clamp_i32(targetCount, STEER_COUNT_MIN, STEER_COUNT_MAX);

  // Compute PID
  int32_t error = targetCount - currentCount;
  uint32_t now_us = micros();
  float dt = (now_us - pidLastTime_us) / 1.0e6f;
  if (dt <= 0.0f) dt = 1.0e-3f;          // guard against weirdness
  pidLastTime_us = now_us;

  float output = 0.0f;
  if (abs(error) > STEER_DEADBAND_COUNTS) {
    pidIntegral += (float)error * dt;
    // Anti-windup clamp on integral term
    const float I_CLAMP = 500.0f / (Ki > 0.0f ? Ki : 1.0f);
    if (pidIntegral >  I_CLAMP) pidIntegral =  I_CLAMP;
    if (pidIntegral < -I_CLAMP) pidIntegral = -I_CLAMP;

    // Derivative on measurement (encoder count change), not on error, to avoid
    // derivative kicks when target changes suddenly.
    static int32_t lastCount = 0;
    float derivative = -((float)(currentCount - lastCount)) / dt;
    lastCount = currentCount;

    output = Kp * (float)error + Ki * pidIntegral + Kd * derivative;
  } else {
    // Within deadband: hold and let integral bleed off slowly
    pidIntegral *= 0.95f;
  }

  // Convert PID output (units: arbitrary) to servo PWM around neutral.
  // Scale factor is heuristic — tune if needed.
  const float OUTPUT_TO_US = 1.0f;       // 1 PID unit = 1 us offset from 1500
  int32_t steer_pwm = STEER_PWM_NEUTRAL_US + (int32_t)(output * OUTPUT_TO_US);
  steer_pwm = clamp_i32(steer_pwm, STEER_PWM_MIN_US, STEER_PWM_MAX_US);
  steeringServo.writeMicroseconds((uint16_t)steer_pwm);
}

// =============================================================================
//  SERIAL COMMAND HANDLER — runtime tuning
// =============================================================================

void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  switch (cmd) {
    case 'k': {
      float p, i, d;
      if (sscanf(line.c_str(), "k %f %f %f", &p, &i, &d) == 3) {
        Kp = p; Ki = i; Kd = d;
        Serial.print(F("[OK] PID gains set: Kp=")); Serial.print(Kp);
        Serial.print(F(" Ki="));                    Serial.print(Ki);
        Serial.print(F(" Kd="));                    Serial.println(Kd);
      }
      break;
    }
    case 't': {
      int lo, hi;
      if (sscanf(line.c_str(), "t %d %d", &lo, &hi) == 2) {
        if (lo >= 0 && hi <= 4095 && lo < hi) {
          THROTTLE_DAC_MIN = lo;
          THROTTLE_DAC_MAX = hi;
          Serial.print(F("[OK] Throttle DAC range: "));
          Serial.print(THROTTLE_DAC_MIN); Serial.print(F(" - "));
          Serial.println(THROTTLE_DAC_MAX);
        } else {
          Serial.println(F("[ERR] Invalid DAC range."));
        }
      }
      break;
    }
    case 's': {
      noInterrupts();
      int32_t cnt = encoderCount;
      uint16_t sp = cubeSteeringPulse_us;
      uint16_t tp = cubeThrottlePulse_us;
      interrupts();
      Serial.println(F("--- STATUS ---"));
      Serial.print(F("  Failsafe:        ")); Serial.println(inFailsafe ? "YES" : "no");
      Serial.print(F("  Steering PWM in: ")); Serial.print(sp); Serial.println(F(" us"));
      Serial.print(F("  Throttle PWM in: ")); Serial.print(tp); Serial.println(F(" us"));
      Serial.print(F("  Encoder count:   ")); Serial.println(cnt);
      Serial.print(F("  Throttle DAC:    ")); Serial.println(currentThrottleDAC);
      Serial.print(F("  Reverse state:   ")); Serial.println((int)reverseState);
      Serial.print(F("  PID gains:       Kp=")); Serial.print(Kp);
      Serial.print(F(" Ki="));                   Serial.print(Ki);
      Serial.print(F(" Kd="));                   Serial.println(Kd);
      break;
    }
    default:
      Serial.println(F("[?] Commands: k <Kp> <Ki> <Kd> | t <dac_min> <dac_max> | s"));
      break;
  }
}
