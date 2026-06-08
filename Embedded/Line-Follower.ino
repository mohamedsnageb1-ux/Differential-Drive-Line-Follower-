// ============================================================
//  LINE-FOLLOWING DIFFERENTIAL DRIVE ROBOT
//  Hardware : Arduino Uno + L298N + 5 IR Sensors + 2 DC Motors
//  Kinematics: Differential Drive (Unicycle) Model
//
//  Physical Parameters:
//    R (wheel radius) = 0.06 m
//    L (wheelbase)    = 0.25 m  (centre-to-centre)
//
//  Reference: Differential Drive Kinematics & Control Guide
// ============================================================

#include <math.h>

// ============================================================
//  SECTION 1 — PIN DEFINITIONS
// ============================================================

// LEFT motor  → L298N channel A
#define PIN_ENA   5   // PWM speed (must be a PWM pin)
#define PIN_IN1   6   // Direction bit 1
#define PIN_IN2   7   // Direction bit 2

// RIGHT motor → L298N channel B
#define PIN_ENB   10  // PWM speed (must be a PWM pin)
#define PIN_IN3   8   // Direction bit 1
#define PIN_IN4   9   // Direction bit 2

// IR Sensor Array  — A0 = Far Right … A4 = Far Left
#define PIN_S1  A0
#define PIN_S2  A1
#define PIN_S3  A2
#define PIN_S4  A3
#define PIN_S5  A4

// Auxiliary
#define PIN_ESTOP  2   // Emergency-stop button (interrupt-capable)
#define PIN_LED   13   // Lap-completion LED / buzzer

// ============================================================
//  SECTION 2 — PHYSICAL & KINEMATIC CONSTANTS
// ============================================================

const float WHEEL_RADIUS = 0.06f;   // R (metres)
const float WHEEL_BASE   = 0.25f;   // L (metres, centre-to-centre)

// Sensor spatial weights  [Guide §3.1]
// Physical wiring: A0 = Far Right (+14) … A4 = Far Left (−14)
// Positive error → line is RIGHT → steer right
// Negative error → line is LEFT  → steer left
const int SENSOR_WEIGHT[5] = { 14, 5, 0, -5, -14 };

// Motor speed range (PWM counts 0–255)
const int BASE_PWM = 75;    // Your tuned base speed
const int MAX_PWM  = 200;   // Hard ceiling for any single motor

// Approximate max linear wheel speed at MAX_PWM (m/s)
const float V_MAX_MPS = 1.5f;

// Track geometry  [confirmed from simulation: total = 3.10 m]
const float LAP_PERIMETER  = 3.10f;
const float LAP_REARM_DIST = 0.31f;

// ── Speed Scaling ────────────────────────────────────────────
// The robot slows down proportionally to how large the error is.
// At error = 0   → runs at BASE_PWM (full speed)
// At error = ±14 → drops to CORNER_PWM (slow turn)
// Formula: adaptivePWM = BASE_PWM - (|error| / MAX_ERROR) × (BASE_PWM - CORNER_PWM)
const int   CORNER_PWM = 45;   // minimum speed during a sharp corner
const float MAX_ERROR  = 14.0f; // largest possible error (your far sensor weight)

// ============================================================
//  SECTION 3 — PID GAINS
//
//  THE CRITICAL FIX — WHY Kd CHANGED FROM 6.5 TO 0.06:
//  With loop dt ≈ 2 ms and an error step of 5:
//    D = Kd × (5 / 0.002) = Kd × 2500
//    At Kd = 6.5  → D = 16,250  → ALWAYS clamped to MAX_PWM (200)
//    At Kd = 0.06 → D =    750 for a large step, ≈30 for a small step ✓
//  Kd = 6.5 was saturating the output on every single sensor update,
//  causing the constant zig-zag oscillation.
//
//  TUNING LADDER (start here, change ONE value at a time):
//    1. Kp=5.5, Kd=0.06, Ki=0, BASE_PWM=65  ← flash this first
//    2. If still tail-wagging → raise Kd slightly (0.08, 0.10)
//    3. If sluggish on corners → raise Kp (6.0, 7.0)
//    4. Once stable → raise BASE_PWM to 80, then 100
//    5. If drifts on 180° semicircle → add Ki=0.001
// ============================================================
float Kp =  2.2f;
float Ki =  0.002f;   // start at 0 — add only after Kp/Kd are stable
float Kd =  0.08f;  // FIXED from 6.5 — was saturating every loop

// ============================================================
//  SECTION 4 — STATE VARIABLES
// ============================================================

// PID internal state
float g_error     = 0.0f;
float g_prevError = 0.0f;
float g_integral  = 0.0f;

// Odometry (dead-reckoning) state  [Guide §2.3]
float g_x     = 0.0f;  // metres
float g_y     = 0.0f;  // metres
float g_theta = 0.0f;  // radians  (0 = robot faces +X)

// Lap tracking
int   g_lapCount     = 0;
bool  g_lapArmed     = false;
float g_distSinceLap = 0.0f;

// Timing
unsigned long g_prevTimeMicros = 0;

// E-stop flag (written inside ISR, read in loop)
volatile bool g_eStop = false;

// Non-blocking LED blink state
bool          g_blinking       = false;
int           g_blinkCount     = 0;
bool          g_ledState       = false;
unsigned long g_lastBlinkTogMs = 0;
const int     BLINK_TOTAL      = 6;
const int     BLINK_PERIOD_MS  = 120;

// ── Line-loss recovery state ─────────────────────────────────
// When the robot loses the line it performs a slow recovery spin
// toward the last known error direction for up to RECOVERY_MS ms
// before giving up and stopping. This handles sharp corners where
// the dead-stop would otherwise freeze the robot.
float         g_lastErrorSign  = 0.0f;  // +1 = was turning right, -1 = left
bool          g_recovering     = false;
unsigned long g_recoveryStartMs = 0;
const unsigned long RECOVERY_MS  = 1500;  // search window (ms)
const int          RECOVERY_PWM  = 50;    // slow recovery spin speed

// ============================================================
//  SECTION 5 — FUNCTION DECLARATIONS
// ============================================================
void  readSensors   (int vals[5]);
float calcError     (const int vals[5]);
float runPID        (float error, float dt);
void  driveMotors   (int leftPWM, int rightPWM);
void  stopMotors    ();
void  updateOdometry(int leftPWM, int rightPWM, float dt);
void  checkLap      ();
void  handleLapBlink();
void  eStopISR      ();
void  printDebug    (float error, float pid, int lPWM, int rPWM);

// ============================================================
//  SECTION 6 — SETUP
// ============================================================
void setup()
{
  Serial.begin(9600);

  // Motor driver pins
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);

  // IR sensor pins
  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);
  pinMode(PIN_S3, INPUT);
  pinMode(PIN_S4, INPUT);
  pinMode(PIN_S5, INPUT);

  // E-stop: active-LOW with internal pull-up
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), eStopISR, FALLING);

  // Lap indicator LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  stopMotors();

  Serial.println(F("Line-follower ready. Starting in 2 s..."));
  delay(2000);

  g_prevTimeMicros = micros();
}

// ============================================================
//  SECTION 7 — MAIN CONTROL LOOP
// ============================================================
void loop()
{
  // ── 7.0 Emergency Stop (highest priority) ───────────────
  if (g_eStop) {
    stopMotors();
    digitalWrite(PIN_LED, HIGH);
    Serial.println(F("E-STOP ACTIVE. Reset Arduino to resume."));
    while (true) {}
  }

  // ── 7.1 Compute Time Step Δt ────────────────────────────
  unsigned long now = micros();
  float dt = (now - g_prevTimeMicros) * 1e-6f;
  g_prevTimeMicros = now;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

  // ── 7.2 Read IR Sensor Array ────────────────────────────
  int sensorVals[5];
  readSensors(sensorVals);

  int activeCount = sensorVals[0] + sensorVals[1] + sensorVals[2]
                  + sensorVals[3] + sensorVals[4];

  // ── 7.3 Declare output variables (visible to debug below) ─
  int   leftPWM = 0, rightPWM = 0;
  float pidOut  = 0.0f;

  // ── 7.4 Control — three states ──────────────────────────

  if (activeCount > 0) {
    // ── STATE A: Line visible — normal PID ────────────────
    g_recovering = false;             // cancel any active recovery
    g_error      = calcError(sensorVals);
    pidOut       = runPID(g_error, dt);

    // Remember which side the line was on for recovery
    if (g_error != 0.0f)
      g_lastErrorSign = (g_error > 0.0f) ? 1.0f : -1.0f;

    // Speed scaling: slow down on corners, full speed on straights
    // adaptivePWM drops linearly from BASE_PWM → CORNER_PWM as error grows
    float errorRatio  = fabsf(g_error) / MAX_ERROR;          // 0.0 (straight) → 1.0 (max corner)
    int   adaptivePWM = BASE_PWM - (int)(errorRatio * (BASE_PWM - CORNER_PWM));

    leftPWM  = adaptivePWM + (int)pidOut;
    rightPWM = adaptivePWM - (int)pidOut;
    leftPWM  = constrain(leftPWM,  -MAX_PWM, MAX_PWM);
    rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

    driveMotors(leftPWM, rightPWM);
    updateOdometry(leftPWM, rightPWM, dt);

  } else if (!g_recovering) {
    // ── STATE B: Line just lost — start recovery spin ─────
    g_recovering      = true;
    g_recoveryStartMs = millis();
    g_integral        = 0.0f;  // reset PID memory
    Serial.println(F("Line lost — searching..."));

    // Spin toward last known error direction
    // lastErrorSign > 0 means line was to the RIGHT → spin right
    leftPWM  =  (int)(RECOVERY_PWM * g_lastErrorSign);
    rightPWM = -(int)(RECOVERY_PWM * g_lastErrorSign);
    driveMotors(leftPWM, rightPWM);
    updateOdometry(leftPWM, rightPWM, dt);

  } else if (millis() - g_recoveryStartMs < RECOVERY_MS) {
    // ── STATE C: Still recovering — keep spinning ─────────
    leftPWM  =  (int)(RECOVERY_PWM * g_lastErrorSign);
    rightPWM = -(int)(RECOVERY_PWM * g_lastErrorSign);
    driveMotors(leftPWM, rightPWM);
    updateOdometry(leftPWM, rightPWM, dt);

  } else {
    // ── STATE D: Recovery timed out — full stop ───────────
    stopMotors();
    g_integral   = 0.0f;
    g_recovering = false;
    updateOdometry(0, 0, dt);
    Serial.println(F("Recovery failed. Place robot on line."));
  }

  // ── 7.5 Lap Detection ───────────────────────────────────
  checkLap();

  // ── 7.6 Non-blocking LED blink service ──────────────────
  handleLapBlink();

  // ── 7.7 Serial Debug (throttled to ~10 Hz) ──────────────
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs >= 100) {
    lastPrintMs = millis();
    printDebug(g_error, pidOut, leftPWM, rightPWM);
  }
}

// ============================================================
//  SECTION 8 — SENSOR READING
// ============================================================
/**
 * Your TCRT5000-type sensors:
 *   Both LEDs on  = white floor → digitalRead = LOW  (0)
 *   1 LED on      = black line  → digitalRead = HIGH (1)
 *
 * vals[i] = 1 means LINE DETECTED.
 */
void readSensors(int vals[5])
{
  vals[0] = digitalRead(PIN_S1);
  vals[1] = digitalRead(PIN_S2);
  vals[2] = digitalRead(PIN_S3);
  vals[3] = digitalRead(PIN_S4);
  vals[4] = digitalRead(PIN_S5);
}

// ============================================================
//  SECTION 9 — WEIGHTED CENTROID ERROR  [Guide §3.2]
// ============================================================
float calcError(const int vals[5])
{
  float weightedSum = 0.0f;
  int   activeCount = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += (float)(SENSOR_WEIGHT[i] * vals[i]);
    activeCount += vals[i];
  }

  if (activeCount > 0) return weightedSum / (float)activeCount;
  return 0.0f;
}

// ============================================================
//  SECTION 10 — PID CONTROLLER  [Guide §4.1]
// ============================================================
/**
 * ω_pid = Kp×e + Ki×∫e dt + Kd×de/dt
 *
 * Called EXACTLY ONCE per loop iteration.
 * Side effects: updates g_integral and g_prevError.
 * NEVER call this a second time in the same loop for printing.
 */
float runPID(float error, float dt)
{
  float P = Kp * error;

  g_integral += error * dt;
  g_integral  = constrain(g_integral, -500.0f, 500.0f);
  float I = Ki * g_integral;

  float D = Kd * (error - g_prevError) / dt;
  g_prevError = error;

  float output = P + I + D;
  output = constrain(output, -(float)MAX_PWM, (float)MAX_PWM);
  return output;
}

// ============================================================
//  SECTION 11 — MOTOR DRIVER  [Guide §2.4]
// ============================================================
void driveMotors(int leftPWM, int rightPWM)
{
  if (leftPWM >= 0) {
    digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, leftPWM);
  } else {
    digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, -leftPWM);
  }

  if (rightPWM >= 0) {
    digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, rightPWM);
  } else {
    digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENB, -rightPWM);
  }
}

// ============================================================
//  SECTION 12 — MOTOR STOP (brake mode)
// ============================================================
void stopMotors()
{
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW); analogWrite(PIN_ENB, 0);
}

// ============================================================
//  SECTION 13 — ODOMETRY  [Guide §2.1, §2.3]
// ============================================================
void updateOdometry(int leftPWM, int rightPWM, float dt)
{
  float vL = ((float)leftPWM  / (float)MAX_PWM) * V_MAX_MPS;
  float vR = ((float)rightPWM / (float)MAX_PWM) * V_MAX_MPS;

  float v     = (vR + vL) * 0.5f;
  float omega = (vR - vL) / WHEEL_BASE;

  g_x     += v * cosf(g_theta) * dt;
  g_y     += v * sinf(g_theta) * dt;
  g_theta += omega * dt;

  while (g_theta >  (float)M_PI) g_theta -= 2.0f * (float)M_PI;
  while (g_theta < -(float)M_PI) g_theta += 2.0f * (float)M_PI;

  g_distSinceLap += fabsf(v) * dt;
}

// ============================================================
//  SECTION 14 — LAP DETECTION
// ============================================================
void checkLap()
{
  if (!g_lapArmed && g_distSinceLap >= LAP_REARM_DIST)
    g_lapArmed = true;

  if (g_lapArmed && g_distSinceLap >= LAP_PERIMETER) {
    g_lapCount++;
    g_distSinceLap = 0.0f;
    g_lapArmed     = false;

    g_blinking       = true;
    g_blinkCount     = 0;
    g_ledState       = false;
    g_lastBlinkTogMs = millis();

    Serial.print(F(">>> LAP COMPLETE! Total laps: "));
    Serial.println(g_lapCount);
  }
}

// ============================================================
//  SECTION 15 — NON-BLOCKING LED BLINK SERVICE
// ============================================================
void handleLapBlink()
{
  if (!g_blinking) return;

  unsigned long now = millis();
  if (now - g_lastBlinkTogMs >= (unsigned long)BLINK_PERIOD_MS) {
    g_lastBlinkTogMs = now;
    g_ledState = !g_ledState;
    digitalWrite(PIN_LED, g_ledState ? HIGH : LOW);
    if (++g_blinkCount >= BLINK_TOTAL) {
      g_blinking = false;
      digitalWrite(PIN_LED, LOW);
    }
  }
}

// ============================================================
//  SECTION 16 — EMERGENCY STOP ISR
// ============================================================
void eStopISR()
{
  g_eStop = true;
}

// ============================================================
//  SECTION 17 — SERIAL DEBUG OUTPUT
// ============================================================
void printDebug(float error, float pid, int lPWM, int rPWM)
{
  Serial.print(F("Err:"));   Serial.print(error, 1);
  Serial.print(F(" PID:"));  Serial.print(pid,   1);
  Serial.print(F(" L:"));    Serial.print(lPWM);
  Serial.print(F(" R:"));    Serial.print(rPWM);
  Serial.print(F(" x:"));    Serial.print(g_x,   3);
  Serial.print(F(" y:"));    Serial.print(g_y,   3);
  Serial.print(F(" θ°:"));   Serial.print(g_theta * 180.0f / (float)M_PI, 1);
  Serial.print(F(" Dist:")); Serial.print(g_distSinceLap, 2);
  Serial.print(F(" Laps:")); Serial.println(g_lapCount);
}

// ============================================================
//  END OF FILE
// ============================================================
