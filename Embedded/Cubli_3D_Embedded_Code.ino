#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "MagneticSensorPIOI2C.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include <SerialBT.h>

/* ============================ CONFIG ============================ */
#define Bluetooth 1            // 1 = Bluetooth serial, 0 = USB serial

#define POLE_PAIRS       11

#define VOLTAGE_SUPPLY   24.0f
#define VOLTAGE_LIMIT    12.0f
#define VELOCITY_LIMIT   110.0f

#define DUMP_ON_VOLTAGE  26.0f
#define DUMP_OFF_VOLTAGE 25.0f

bool motorEnabledAtBoot[3] = { true, true, true };

// Per-motor electrical calibration.
//   M1 = Y axis, M2 = X axis, M3 = Z axis
const float     ZERO_ELECTRIC_ANGLE[3] = { 4.17f,            2.08f,            4.81f };
const Direction SENSOR_DIRECTION[3]    = { Direction::CW,    Direction::CCW,   Direction::CW };

// Check with M<n> <V> and compare the RPM printed by VB ON: the mapping is crossed, so
// the two should have opposite signs. If they match, flip that wheel's sign.
float WHEEL_SIGN[3] = { 1.0f, -1.0f, 1.0f };

#define IMU_PERIOD_US       2500
#define CONTROL_PERIOD_US   18110
#define ADC_PERIOD_US       1000
#define PRINT_PERIOD_US     20000

#define IMU_PRIORITY       0xC0
#define CONTROL_PRIORITY   0x80

#define TILT_LIMIT_DEG     6.0f
#define IMU_STALE_US       50000

/* ============================ PINS ============================= */
#define M1_EN   2
#define M1_IN3  3
#define M1_IN2  4
#define M1_IN1  5

#define M2_EN   6
#define M2_IN3  7
#define M2_IN2  8
#define M2_IN1  9

#define M3_EN   10
#define M3_IN3  11
#define M3_IN2  12
#define M3_IN1  13

#define M1_SDA 16
#define M1_SCL 17
#define M2_SDA 18
#define M2_SCL 19
#define M3_SDA 20
#define M3_SCL 21

#define IMU_SDA 26
#define IMU_SCL 27

#define MOSFET_EN 22
#define BUS_VOLTAGE_ADC 28

#define IMU_RST 0
#define M1_RST  1
#define M2_RST  14
#define M3_RST  15
#define RST_ACTIVE_LOW 1

/* ================= CROSS-CORE SHARED STATE ================= */
volatile bool  core0Ready              = false;

volatile float sharedVelocity[3]       = { 0, 0, 0 };
volatile float sharedTarget[3]         = { 0, 0, 0 };

volatile bool  motorEnableRequest[3]   = { false, false, false };
volatile bool  motorDisableRequest[3]  = { false, false, false };
volatile bool  motorEnabled[3]         = { false, false, false };
volatile bool  motorInitialised[3]     = { false, false, false };

volatile bool resetDriversRequest = false;
volatile uint32_t focLoopCount         = 0;

#define C_PRINT(...)   if (Bluetooth) { SerialBT.print(__VA_ARGS__); } else { Serial.print(__VA_ARGS__); }
#define C_PRINTLN(...) if (Bluetooth) { SerialBT.println(__VA_ARGS__); } else { Serial.println(__VA_ARGS__); }


/* ================================================================
   ============================ CORE 0 ============================
   ================================================================ */

BLDCMotor motor1 = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_IN1, M1_IN2, M1_IN3, M1_EN);
MagneticSensorPIOI2C sensor1(pio0, 0, M1_SDA, M1_SCL);

BLDCMotor motor2 = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_IN1, M2_IN2, M2_IN3, M2_EN);
MagneticSensorPIOI2C sensor2(pio0, 1, M2_SDA, M2_SCL);

BLDCMotor motor3 = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(M3_IN1, M3_IN2, M3_IN3, M3_EN);
MagneticSensorPIOI2C sensor3(pio0, 2, M3_SDA, M3_SCL);

BLDCMotor*      motors[3]  = { &motor1, &motor2, &motor3 };
BLDCDriver3PWM* drivers[3] = { &driver1, &driver2, &driver3 };
MagneticSensorPIOI2C* sensors[3] = { &sensor1, &sensor2, &sensor3 };
const int       EN_PINS[3] = { M1_EN, M2_EN, M3_EN };


void configureMotor(BLDCMotor &motor, BLDCDriver3PWM &driver, Sensor &sensor,
                    float zeroElecAngle, Direction sensorDir) {
  driver.voltage_power_supply = VOLTAGE_SUPPLY;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.foc_modulation     = FOCModulationType::SpaceVectorPWM;
  motor.controller         = MotionControlType::torque;
  motor.torque_controller  = TorqueControlType::voltage;

  motor.voltage_limit  = VOLTAGE_LIMIT;
  motor.velocity_limit = VELOCITY_LIMIT;
  motor.LPF_velocity.Tf = 0.01f;
  motor.motion_downsample = 0;

  motor.init();

  if (_isset(zeroElecAngle))            motor.zero_electric_angle = zeroElecAngle;
  if (sensorDir != Direction::UNKNOWN)  motor.sensor_direction    = sensorDir;
  motor.initFOC();
}

static void resetPeripherals() {
  const uint8_t pins[] = { IMU_RST, M1_RST, M2_RST, M3_RST };
  const uint8_t ASSERT_LVL  = RST_ACTIVE_LOW ? LOW  : HIGH;
  const uint8_t RELEASE_LVL = RST_ACTIVE_LOW ? HIGH : LOW;

  for (uint8_t p : pins) { pinMode(p, OUTPUT); digitalWrite(p, ASSERT_LVL); }
  delay(50);
  for (uint8_t p : pins) { digitalWrite(p, RELEASE_LVL); }
  delay(200);
}

static void resetDrivers() {
  const uint8_t pins[] = { M1_RST, M2_RST, M3_RST };
  const uint8_t ASSERT_LVL  = RST_ACTIVE_LOW ? LOW  : HIGH;
  const uint8_t RELEASE_LVL = RST_ACTIVE_LOW ? HIGH : LOW;

  for (uint8_t p : pins) { pinMode(p, OUTPUT); digitalWrite(p, ASSERT_LVL); }
  delay(50);
  for (uint8_t p : pins) { digitalWrite(p, RELEASE_LVL); }
  delay(200);
  resetDriversRequest = false;
}

void setup() {
  resetPeripherals();
  Serial.begin(115200);
  if (Bluetooth) {
    SerialBT.setName("Cubli");
    SerialBT.begin();
  }

  delay(7000);

  while (!Serial && !SerialBT) {
    delay(10);
  }

  C_PRINTLN(F("Cubli Starting..."));

  for (int i = 0; i < 3; i++) {
    if (motorEnabledAtBoot[i]) {
      sensors[i]->init();
      configureMotor(*motors[i], *drivers[i], *sensors[i],
                     ZERO_ELECTRIC_ANGLE[i], SENSOR_DIRECTION[i]);
      motorInitialised[i] = true;
      motorEnabled[i]     = true;
      C_PRINT(F("Motor ")); C_PRINT(i + 1); C_PRINT(F(" ready"));
    } else {
      pinMode(EN_PINS[i], OUTPUT);
      digitalWrite(EN_PINS[i], LOW);
    }
  }

  core0Ready = true;
}

void loop() {
  for (int i = 0; i < 3; i++) {

    if (motorEnableRequest[i]) {
      motorEnableRequest[i] = false;
      if (motorInitialised[i]) {
        motors[i]->enable();
        motorEnabled[i] = true;
      }
    }
    if (motorDisableRequest[i]) {
      motorDisableRequest[i] = false;
      if (motorInitialised[i]) motors[i]->disable();
      motorEnabled[i] = false;
      sharedVelocity[i] = 0.0f;
    }

    if (motorEnabled[i]) {
      motors[i]->loopFOC();
      motors[i]->move(sharedTarget[i]);
      sharedVelocity[i] = motors[i]->shaft_velocity;
    }
  }
  if (resetDriversRequest)
    resetDrivers();

  focLoopCount = focLoopCount + 1;
}


/* ================================================================
   ============================ CORE 1 ============================
   ================================================================ */

Adafruit_BNO08x bno08x;
sh2_SensorValue_t imuValue;
bool verbose = false;
bool dumpLoadOn = false;
bool refAdaptEnabled = true;

/* ---- Motor model ---- */
const float R_MOTOR    = 4.3f;
const float KE[3]      = {0.157f, 0.157f, 0.157f};
const float KT[3]      = {0.25f,  0.25f,  0.25f};
const float TAU_RATED  = 0.35f;
const float V_MAX      = 12.0f;
const float I_LIM      = 1.8f;
const float MAX_VOLTAGE_SLEW = 12.0f;
float lastVoltageCmd[3] = { 0, 0, 0 };

/* ---- Motor <-> body axis mapping ---- */
const uint8_t MOTOR_AXIS[3] = { 1, 0, 2 };
const char    AXIS_NAME[3]  = { 'X', 'Y', 'Z' };

/* ---- 3D LQR ---- */
const float X0_NOMINAL[4]    = { 0.8881f, 0.325f, -0.3252f, 0.0f };
const float EDGE_TRIM_DEG[3] = { -1.4f, -0.25f, 2.4f }; // 1D trim values

float x0[10] = { 0.8881f, 0.325f, -0.3252f, 0.0f, 0, 0, 0, 0, 0, 0 };

// 3x10 LQR gain matrix from MATLAB
const float K[3][10] = {
  {   -4.360454925f, 0.639880389527f, -11.2686312801f, -2.61077494728f, 0.174627136077f, -0.350384593493f, 0.174447349484f, 0.00573343254315f, 0.00587939810591f, 0.00587633074301f },
  {   4.3577068904f, -11.2677185026f, 0.639824649546f, -2.61230077604f, -0.350086019372f, 0.174635390405f, 0.174560472364f, 0.00587995023894f, 0.00574307703276f, 0.00588142836565f },
  { -3.05144081963e-05f,  5.00054394536f,  4.99738524722f, -10.1603946657f, 0.174307266507f, 0.174202589947f, -0.350724763678f, 0.00584250609322f, 0.00584702523161f, 0.00570255339411f }
};

// Common-mode wheel-speed regulator (drives mean wheel speed -> 0)
const float K_COMMON = 0.003f;


/* ---- 3D adaptive balance reference ----*/
#define REF_RATE  0.4f
#define REF_MAX   radians(1.0f)
volatile float refRot[3] = { 0.0f, 0.0f, 0.0f };
float refMap[3][3]       = { {0,0,0}, {0,0,0}, {0,0,0} };
bool  refMapValid        = false;

const float TILT_LIMIT_RAD = TILT_LIMIT_DEG * 0.0174532925f;

/* ---- 1D single-axis mode ----*/
#define D1_TILT_LIMIT_DEG   10.0f
#define D1_EDGE_MAX_UA      0.25f
#define D1_TRIM_MAX         radians(4.0f)
const float D1_TILT_LIMIT_RAD = D1_TILT_LIMIT_DEG * 0.0174532925f;

volatile bool    balanceControlActive = false;
volatile bool    axisTestActive = false;
volatile uint8_t axisMotor      = 0;
float            edgeTarget[3]  = { 0, 0, 0 };
volatile float   d1Trim         = 0.0f;
float            d1TrimInit[3]  = { radians(-0.25f), radians(-1.4f), radians(2.4f) };// per motor, TR <n> <deg>

const float D1_GAIN_DEFAULT[3][3] = {
  { -7.0287f, -0.3722f, -0.001f },
  { -7.0287f, -0.3722f, -0.001f },
  { -7.0287f, -0.3722f, -0.001f }
};
float d1Gain[3][3] = {
  { -7.0287f, -0.3722f, -0.001f },
  { -7.0287f, -0.3722f, -0.001f },
  { -7.0287f, -0.3722f, -0.001f }
};
volatile float d1Theta = 0.0f, d1Omega = 0.0f, d1Ua = 0.0f;

/* ---- Measurement smoothing ---- */
const float RATE_ALPHA  = 0.15f;

/* ---- Live sensor state ---- */
// Exact geometric quaternion for the diagonal IMU mount
const float q_BS[4]     = {0.279848f, -0.115917f, -0.880476f, 0.364705f};
float rawQuat[4]        = { 1, 0, 0, 0 };
float bodyQuat[4]       = { 1, 0, 0, 0 };
float bodyRateFilt[3]   = { 0, 0, 0 };
float tiltErr           = 0.0f;
uint32_t lastQuatUs     = 0;
volatile bool imuDataValid = false;

struct Snap {
  float rawQ[4];
  float bodyQ[4];
  float rate[3];
  float wheel[3];
  float tilt;
  uint32_t quatStampUs;
  bool valid;

  void take() {
    uint32_t st = save_and_disable_interrupts();
    memcpy(rawQ, rawQuat, sizeof(rawQuat));
    memcpy(bodyQ, bodyQuat, sizeof(bodyQuat));
    for (int i = 0; i < 3; i++) rate[i] = bodyRateFilt[i];
    tilt        = tiltErr;
    quatStampUs = lastQuatUs;
    valid       = imuDataValid;
    restore_interrupts(st);

    for (int i = 0; i < 3; i++) wheel[i] = sharedVelocity[i] * WHEEL_SIGN[i];
  }
};

/* ---- Control outputs / trip reporting ---- */
volatile float   tauCmd[3]   = { 0, 0, 0 };
volatile uint8_t tripReason  = 0; // 0 none, 1 tilt, 2 motor disabled, 3 IMU stale
volatile float   tripTilt    = 0.0f;

/* ---- Loop timing ---- */
int my_alarms[2];
uint32_t now = 0;
uint32_t lastAdcUs = 0, lastPrintUs = 0, lastFocCount = 0;
volatile uint32_t lastControlUs = 0;
volatile uint32_t controlPeriodDeltaUs = 0;

#if !Bluetooth
char buffer[512];
#else
char buffer[32];
#endif

/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////    HELPER FUNCTIONS   ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

/* ================= QUATERNION & VECTOR HELPERS ================= */
void quatMultiply(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void quatNormalize(float q[4]) {
  float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n < 1e-6f) return;
  for (int i = 0; i < 4; i++) q[i] /= n;
}

void quatRotateVector(const float q[4], const float v[3], float out[3]) {
  float qv[4] = { 0, v[0], v[1], v[2] };
  float qc[4] = { q[0], -q[1], -q[2], -q[3] };
  float t1[4], t2[4];
  quatMultiply(q, qv, t1);
  quatMultiply(t1, qc, t2);
  out[0] = t2[1]; out[1] = t2[2]; out[2] = t2[3];
}

static void bodyUpVector(const float q[4], float u[3]) {
  const float worldUp[3] = { 0.0f, 0.0f, 1.0f };
  const float qc[4] = { q[0], -q[1], -q[2], -q[3] };
  quatRotateVector(qc, worldUp, u);
}

float tiltAngleUpVector(const float q[4], const float ref[4]) {
  float bodyUp[3], targetUp[3];
  bodyUpVector(q, bodyUp);
  bodyUpVector(ref, targetUp);

  float dot = bodyUp[0]*targetUp[0] + bodyUp[1]*targetUp[1] + bodyUp[2]*targetUp[2];
  float cx = bodyUp[1]*targetUp[2] - bodyUp[2]*targetUp[1];
  float cy = bodyUp[2]*targetUp[0] - bodyUp[0]*targetUp[2];
  float cz = bodyUp[0]*targetUp[1] - bodyUp[1]*targetUp[0];
  return atan2f(sqrtf(cx*cx + cy*cy + cz*cz), dot);
}

void relativeQuat(const float q_body[4], float q_rel[4]) {
  float x0_conj[4] = { x0[0], -x0[1], -x0[2], -x0[3] };
  quatMultiply(x0_conj, q_body, q_rel);
  if (q_rel[0] < 0.0f) {
    for (int i = 0; i < 4; i++) q_rel[i] = -q_rel[i];
  }
}

void buildRefQuat(const float r[3], float out[4]) {
  float dq[4] = { 1.0f, 0.5f * r[0], 0.5f * r[1], 0.5f * r[2] };
  float n = sqrtf(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
  for (int i = 0; i < 4; i++) dq[i] /= n;
  quatMultiply(x0, dq, out);
}

void applyEdgeTrimsToX0() {
  float th[3], mean = 0.0f;
  for (int i = 0; i < 3; i++) { th[i] = EDGE_TRIM_DEG[i] * 0.0174532925f; mean += th[i] / 3.0f; }
  float r[3];
  for (int i = 0; i < 3; i++) r[i] = (2.0f / 3.0f) * (th[i] - mean);

  float dq[4] = { 1.0f, 0.5f * r[0], 0.5f * r[1], 0.5f * r[2] };
  quatNormalize(dq);
  float nom[4] = { X0_NOMINAL[0], X0_NOMINAL[1], X0_NOMINAL[2], X0_NOMINAL[3] };
  quatNormalize(nom);
  float out[4];
  quatMultiply(nom, dq, out);
  quatNormalize(out);
  for (int i = 0; i < 4; i++) x0[i] = out[i];
}

void removeYaw(const float q[4], float out[4]) {
  const float x0c[4] = { x0[0], -x0[1], -x0[2], -x0[3] };
  float qe[4];
  quatMultiply(q, x0c, qe);
  float n = sqrtf(qe[0]*qe[0] + qe[3]*qe[3]);
  if (n < 1e-6f) { memcpy(out, q, 4 * sizeof(float)); return; }
  const float tc[4] = { qe[0] / n, 0.0f, 0.0f, -qe[3] / n };
  quatMultiply(tc, q, out);
}

static float axisTilt(const float u[3], const float t[3], int a) {
  const int p = (a + 1) % 3, q = (a + 2) % 3;
  return atan2f(t[q]*u[p] - t[p]*u[q], t[p]*u[p] + t[q]*u[q]);
}

static int splitTokens(const String &s, String tok[], int maxTok) {
  int n = 0, i = 0, len = s.length();
  while (i < len && n < maxTok) {
    while (i < len && s.charAt(i) == ' ') i++;
    if (i >= len) break;
    int j = i;
    while (j < len && s.charAt(j) != ' ') j++;
    tok[n++] = s.substring(i, j);
    i = j;
  }
  return n;
}

/* ================= ADAPTIVE-REFERENCE MAP ================= */
static void computeHq(float Hq[3][3]) {
  for (int a = 0; a < 3; a++) {
    float ea[4] = { 0, 0, 0, 0 };
    ea[a + 1] = 1.0f;
    float d[4];
    quatMultiply(x0, ea, d);
    for (int k = 0; k < 4; k++) d[k] *= 0.5f;
    for (int i = 0; i < 3; i++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) sum += K[i][k] * d[k];
      Hq[i][a] = sum;
    }
  }
}

static bool computeRefMap() {
  float Hq[3][3];
  computeHq(Hq);
  float kth = 0.0f, kw = 0.0f;
  for (int m = 0; m < 3; m++) {
    const int am = MOTOR_AXIS[m];
    float oH = 0.0f, oW = 0.0f;
    for (int i = 1; i < 3; i++) { oH += Hq[m][(am + i) % 3]; oW += K[m][7 + (m + i) % 3]; }
    kth += (Hq[m][am]   - 0.5f * oH) / 3.0f;
    kw  += (K[m][7 + m] - 0.5f * oW) / 3.0f;
  }
  if (fabsf(kth) < 1e-6f) return false;
  const float ratio = kw / kth;
  for (int a = 0; a < 3; a++)
    for (int j = 0; j < 3; j++)
      refMap[a][j] = ratio * ((a == MOTOR_AXIS[j] ? 1.0f : 0.0f) - 1.0f / 3.0f);
  return true;
}

static void printD1Gains() {
  for (int m = 0; m < 3; m++) {
    C_PRINT(F("D1 M")); C_PRINT(m + 1); C_PRINT(F(" (axis ")); C_PRINT(AXIS_NAME[MOTOR_AXIS[m]]);
    C_PRINT(F("): kTheta=")); C_PRINT(d1Gain[m][0], 5);
    C_PRINT(F("  kOmega="));  C_PRINT(d1Gain[m][1], 5);
    C_PRINT(F("  kW="));      C_PRINTLN(d1Gain[m][2], 5);
  }
}

/* ================= CONTROLLER FUNCTIONS ================= */
float torqueToVoltage(float tau_des, float w, int i) {
  float Ihi = fminf( I_LIM, ( V_MAX - KE[i]*w) / R_MOTOR);
  float Ilo = fmaxf(-I_LIM, (-V_MAX - KE[i]*w) / R_MOTOR);
  float tHi = fminf( TAU_RATED, KT[i]*Ihi);
  float tLo = fmaxf(-TAU_RATED, KT[i]*Ilo);
  float tau = fmaxf(tLo, fminf(tHi, tau_des));
  return (tau / KT[i]) * R_MOTOR + KE[i]*w;
}

float applySlewLimit(int i, float v_cmd) {
  float delta = v_cmd - lastVoltageCmd[i];
  if (delta >  MAX_VOLTAGE_SLEW) delta =  MAX_VOLTAGE_SLEW;
  if (delta < -MAX_VOLTAGE_SLEW) delta = -MAX_VOLTAGE_SLEW;
  lastVoltageCmd[i] += delta;
  return lastVoltageCmd[i];
}

// tau = -K * (x - xr)
void computeControl(const float x[10], const float xr[10], float tau_out[3]) {
  float e[10];
  for (int j = 0; j < 10; j++) e[j] = x[j] - xr[j];

  for (int i = 0; i < 3; i++) {
    float sum = 0.0f;
    for (int j = 0; j < 10; j++) sum += K[i][j] * e[j];
    tau_out[i] = -sum;
  }
}

void disengageFromISR(uint8_t reason, float tilt) {
  balanceControlActive = false;
  for (int i = 0; i < 3; i++) {
    sharedTarget[i]   = 0.0f;
    lastVoltageCmd[i] = 0.0f;
  }
  tripTilt   = tilt;
  tripReason = reason;
}

void zeroAllTargets() {
  for (int i = 0; i < 3; i++) {
    sharedTarget[i]   = 0.0f;
    lastVoltageCmd[i] = 0.0f;
  }
}

/* ================= ADC / DUMP LOAD ================= */
float readBusVoltage() {
  float adcVoltage = (analogRead(BUS_VOLTAGE_ADC) / 4095.0f) * 3.3f;
  // slope = 0.0825, VBUS = 12 + (VADC - 1.06) / 0.0825
  return 12.0f + (adcVoltage - 1.06f) / 0.0825f;
}

void updateDumpLoad() {
  float busVoltage = readBusVoltage();
  if (!dumpLoadOn && busVoltage >= DUMP_ON_VOLTAGE) {
    dumpLoadOn = true;
    digitalWrite(MOSFET_EN, HIGH);
  } else if (dumpLoadOn && busVoltage <= DUMP_OFF_VOLTAGE) {
    dumpLoadOn = false;
    digitalWrite(MOSFET_EN, LOW);
  }
}

/* ================= IMU SAMPLING ================= */
void updateIMU() {
  static float qRaw[4]   = { 1, 0, 0, 0 };
  static float qBody[4]  = { 1, 0, 0, 0 };
  static float rBody[3]  = { 0, 0, 0 };
  static float rateF[3]  = { 0, 0, 0 };
  static float tilt      = 0.0f;
  static bool  haveQ = false, haveG = false;
  bool newQuat = false;

  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 2500);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2500);
    balanceControlActive = false;
  }

  while (bno08x.getSensorEvent(&imuValue)) {
    if (imuValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      qRaw[0] = imuValue.un.gameRotationVector.real;
      qRaw[1] = imuValue.un.gameRotationVector.i;
      qRaw[2] = imuValue.un.gameRotationVector.j;
      qRaw[3] = imuValue.un.gameRotationVector.k;
      newQuat = true;
      haveQ   = true;
    } else if (imuValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      float rawX = imuValue.un.gyroscope.x;
      float rawY = imuValue.un.gyroscope.y;
      float rawZ = imuValue.un.gyroscope.z;

      // Maps the diagonal-mounted IMU directly to the orthogonal motors
      rBody[0] = -0.816497f * rawX + 0.000000f * rawY - 0.577350f * rawZ;  // Body X (Motor 2)
      rBody[1] =  0.408248f * rawX + 0.707107f * rawY - 0.577350f * rawZ;  // Body Y (Motor 1)
      rBody[2] =  0.408248f * rawX - 0.707107f * rawY - 0.577350f * rawZ;  // Body Z (Motor 3)

      haveG = true;
    }
  }

  if (newQuat) {
    float q_BS_conj[4] = {q_BS[0], -q_BS[1], -q_BS[2], -q_BS[3]};
    quatMultiply(qRaw, q_BS_conj, qBody);
    quatNormalize(qBody);

    float d = qBody[0]*x0[0] + qBody[1]*x0[1] + qBody[2]*x0[2] + qBody[3]*x0[3];
    if (d < 0.0f) for (int k = 0; k < 4; k++) qBody[k] = -qBody[k];
    tilt = tiltAngleUpVector(qBody, x0);
  }

  for (int i = 0; i < 3; i++) rateF[i] += RATE_ALPHA * (rBody[i] - rateF[i]);

  uint32_t st = save_and_disable_interrupts();
  for (int i = 0; i < 4; i++) { rawQuat[i] = qRaw[i]; bodyQuat[i] = qBody[i]; }
  for (int i = 0; i < 3; i++) bodyRateFilt[i] = rateF[i];
  tiltErr = tilt;
  if (newQuat) lastQuatUs = micros();
  imuDataValid = haveQ && haveG;
  restore_interrupts(st);
}

/* ================= SERIAL COMMAND INTERFACE ================= */
void printHelp() {
  C_PRINTLN(F("Commands:"));
  C_PRINTLN(F("  BAL ON / BAL OFF - Engage/disengage balancing (3 motors enabled = 3D, 1 motor enabled = 1D)"));
  C_PRINTLN(F("  M<n> <V>         - Set manual voltage on motor n=1..3 (when BAL OFF)"));
  C_PRINTLN(F("  M<n> ON / OFF    - Enable/disable individual drivers"));
  C_PRINTLN(F("  WS <n> <+1/-1>   - Flip motor<->body sign of motor n (if BAL ON pushes the cube away)"));
  C_PRINTLN(F("  STOP             - Zero all targets & disengage"));
  C_PRINTLN(F("  VB ON / VB OFF   - Verbose print (tilt / ref / rates / wheel RPM / voltage)"));
  C_PRINTLN(F("  REF ON / OFF     - Enable/freeze adaptive balance reference"));
  C_PRINTLN(F("  REF RESET        - Set learned balance reference back to its start"));
  C_PRINTLN(F("  D1 / D1 RESET    - 1D: show / restore default gains;  D1 <n> <kTh> <kOm> <kW> sets them"));
  C_PRINTLN(F("  TR / TR <n> <deg>- 1D: starting trim (deg), the angle treated as balance"));
  C_PRINTLN(F("  SETUP            - Sets all motors to 1V"));
  C_PRINTLN(F("  REBOOT           - Software reboot"));
  C_PRINTLN(F("  RST DRIVERS      - Reset Drivers"));
}

void processCommand(String line) {
  line.trim();
  line.toUpperCase();

  if (line == "BAL ON") {
    if (!imuDataValid) {
      C_PRINTLN(F("No valid IMU data yet - not engaging."));
      return;
    }

    int enabledCount = motorEnabled[0] + motorEnabled[1] + motorEnabled[2];

    if (enabledCount == 0) {
      C_PRINTLN(F("At least one motor must be enabled - not engaging."));
      return;
    }
    if (enabledCount == 2) {
      C_PRINTLN(F("Two motors enabled is not a valid configuration. Enable all 3 (3D balance) or exactly 1 (1D balance)."));
      return;
    }

    zeroAllTargets();
    for (int a = 0; a < 3; a++) refRot[a] = 0.0f;

    if (enabledCount == 1) {
      const int m = motorEnabled[0] ? 0 : (motorEnabled[1] ? 1 : 2);
      const int a = MOTOR_AXIS[m];
      const int p = (a + 1) % 3, q = (a + 2) % 3;

      Snap s;
      s.take();
      float u[3];
      bodyUpVector(s.bodyQ, u);

      if (fabsf(u[a]) > D1_EDGE_MAX_UA) {
        C_PRINT(F("Not on the M")); C_PRINT(m + 1); C_PRINT(F(" edge"));
        return;
      }

      float t[3] = { 0.0f, 0.0f, 0.0f };
      t[p] = (u[p] >= 0.0f ? 1.0f : -1.0f) * 0.70710678f;
      t[q] = (u[q] >= 0.0f ? 1.0f : -1.0f) * 0.70710678f;

      const float th = axisTilt(u, t, a);
      if (fabsf(th) > D1_TILT_LIMIT_RAD) {
        C_PRINT(F("Cube is ")); C_PRINT(th * 57.29578f, 1); C_PRINTLN(F(" deg from the nearest edge balance pose. Not engaging."));
        return;
      }

      for (int k = 0; k < 3; k++) edgeTarget[k] = t[k];
      d1Trim = d1TrimInit[m];
      axisMotor = (uint8_t)m;
      refAdaptEnabled = true;
      axisTestActive = true;
      balanceControlActive = true;

      C_PRINT(F("1D BALANCE ENGAGED: M")); C_PRINT(m + 1); C_PRINT(F(" on axis ")); C_PRINT(AXIS_NAME[a]);
      C_PRINT(F(". Start theta=")); C_PRINT(th * 57.29578f, 2); C_PRINT(F(" deg. kTheta/kOmega/kW = "));
      C_PRINT(d1Gain[m][0], 4); C_PRINT(F(" / ")); C_PRINT(d1Gain[m][1], 4); C_PRINT(F(" / ")); C_PRINTLN(d1Gain[m][2], 4);
      return;
    }

    axisTestActive = false;
    balanceControlActive = true;
    C_PRINTLN(F("LQR Balance Control ENGAGED."));
    return;
  }
  if (line == "BAL OFF" || line == "STOP") {
    balanceControlActive = false;
    zeroAllTargets();
    C_PRINTLN(F("Balance Control DISENGAGED. Motors set to 0."));
    return;
  }
  if (line == "VB ON")  { verbose = true;  return; }
  if (line == "VB OFF") { verbose = false; return; }
  if (line == "SETUP")  { sharedTarget[0] = 1.0f; sharedTarget[1] = 1.0f; sharedTarget[2] = 1.0f; return; }
  if (line == "REBOOT") { rp2040.reboot(); return; }
  if (line == "REF ON")    { refAdaptEnabled = true;  C_PRINTLN(F("Adaptive ref ON"));     return; }
  if (line == "REF OFF")   { refAdaptEnabled = false; C_PRINTLN(F("Adaptive ref frozen")); return; }
  if (line == "REF RESET") {
    for (int a = 0; a < 3; a++) refRot[a] = 0.0f;
    if (axisTestActive && balanceControlActive) d1Trim = d1TrimInit[axisMotor];
    C_PRINTLN(F("Reference reset"));
    return;
  }
  if (line == "RST DRIVERS") { resetDriversRequest = true; return; }

  if (line.startsWith("D1")) {
    if (balanceControlActive) { C_PRINTLN(F("Send BAL OFF first.")); return; }
    String t[5];
    int n = splitTokens(line, t, 5);
    if (n == 2 && t[1] == "RESET") {
      memcpy(d1Gain, D1_GAIN_DEFAULT, sizeof(d1Gain));
      printD1Gains();
      return;
    }
    if (n == 5) {
      int idx = t[1].toInt() - 1;
      if (idx < 0 || idx > 2) { C_PRINTLN(F("Usage: D1 <1..3> <kTheta> <kOmega> <kW>   |   D1 RESET   |   D1")); return; }
      d1Gain[idx][0] = t[2].toFloat();
      d1Gain[idx][1] = t[3].toFloat();
      d1Gain[idx][2] = t[4].toFloat();
    }
    printD1Gains();
    return;
  }

  if (line.startsWith("TR")) {
    if (balanceControlActive) { C_PRINTLN(F("Send BAL OFF first.")); return; }
    String t[3];
    int n = splitTokens(line, t, 3);
    if (n == 3) {
      int idx = t[1].toInt() - 1;
      if (idx < 0 || idx > 2) { C_PRINTLN(F("Usage: TR <1..3> <deg>   |   TR")); return; }
      float deg = t[2].toFloat();
      if (fabsf(deg) > 4.0f) { C_PRINTLN(F("Trim limited to +/-4 deg.")); return; }
      d1TrimInit[idx] = deg * 0.0174532925f;
    }
    for (int m = 0; m < 3; m++) {
      C_PRINT(F("TR M")); C_PRINT(m + 1); C_PRINT(F(" = ")); C_PRINT(d1TrimInit[m] * 57.29578f, 3); C_PRINTLN(F(" deg"));
    }
    return;
  }

  if (line.startsWith("WS ")) {
    if (balanceControlActive) { C_PRINTLN(F("Send BAL OFF first.")); return; }
    String args = line.substring(3);
    args.trim();
    int sp = args.indexOf(' ');
    int idx = (sp < 0) ? -1 : args.substring(0, sp).toInt() - 1;
    if (idx < 0 || idx > 2) { C_PRINTLN(F("Usage: WS <1..3> <+1/-1>")); return; }
    float v = args.substring(sp + 1).toFloat();
    WHEEL_SIGN[idx] = (v < 0.0f) ? -1.0f : 1.0f;
    C_PRINT(F("WHEEL_SIGN[M")); C_PRINT(idx + 1); C_PRINT(F("] = ")); C_PRINTLN(WHEEL_SIGN[idx], 0);
    return;
  }

  int space = line.indexOf(' ');
  if (space < 0 || line.charAt(0) != 'M') return;

  int idx = line.substring(1, space).toInt() - 1;
  if (idx < 0 || idx > 2) return;

  String arg = line.substring(space + 1);
  arg.trim();

  if (arg == "ON") {
    if (!motorInitialised[idx]) {
      C_PRINTLN(F("That motor was not initialised at boot (motorEnabledAtBoot)."));
      return;
    }
    motorEnableRequest[idx] = true;
    return;
  }
  if (arg == "OFF") {
    balanceControlActive = false;
    zeroAllTargets();
    motorDisableRequest[idx] = true;
    return;
  }
  if (motorEnabled[idx] && !balanceControlActive) {
    sharedTarget[idx] = constrain(arg.toFloat(), -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  }
}

void handleSerial() {
  static String usbLine = "";
  static String btLine = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbLine.length() > 0) processCommand(usbLine);
      usbLine = "";
    } else usbLine += c;
  }

  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (btLine.length() > 0) processCommand(btLine);
      btLine = "";
    } else btLine += c;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////    CALLBACKS   /////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

static void runAxisTest(const float bodyQ[4], const float rate[3], const float wheel[3], uint32_t quatStampUs, float snapTilt) {
  const int m = axisMotor;
  const int a = MOTOR_AXIS[m];

  if (!motorEnabled[m]){ disengageFromISR(2, snapTilt); return; }
  if ((uint32_t)(micros() - quatStampUs) > IMU_STALE_US){ disengageFromISR(3, snapTilt); return; }

  float u[3];
  bodyUpVector(bodyQ, u);
  const float theta = axisTilt(u, edgeTarget, a);
  if (fabsf(theta) > D1_TILT_LIMIT_RAD){ disengageFromISR(1, theta); return; }

  const float kTh = d1Gain[m][0], kOm = d1Gain[m][1], kW = d1Gain[m][2];
  const float omega = rate[a];
  const float w     = wheel[m] - x0[7 + m];

  if (refAdaptEnabled && fabsf(kTh) > 1e-6f) {
    float tr = d1Trim - REF_RATE * (kW / kTh) * w * (CONTROL_PERIOD_US * 1e-6f);
    d1Trim = fmaxf(-D1_TRIM_MAX, fminf(D1_TRIM_MAX, tr));
  }

  const float tau = -(kTh * (theta - d1Trim) + kOm * omega + kW * w);

  float v = torqueToVoltage(tau, wheel[m], m);
  v = applySlewLimit(m, v);

  for (int i = 0; i < 3; i++) tauCmd[i] = 0.0f;
  tauCmd[m]       = tau;
  sharedTarget[m] = WHEEL_SIGN[m] * v;

  d1Theta = theta;
  d1Omega = omega;
  d1Ua    = u[a];
}

void imu_callback(uint alarm_num) {
  bool suc = hardware_alarm_set_target(alarm_num, make_timeout_time_us(IMU_PERIOD_US));
  if (suc) { hardware_alarm_force_irq(alarm_num); }

  updateIMU();
}

void control_callback(uint alarm_num) {
  bool suc = hardware_alarm_set_target(alarm_num, make_timeout_time_us(CONTROL_PERIOD_US));
  if (suc) { hardware_alarm_force_irq(alarm_num); }

  uint32_t currentUs = micros();
  if (lastControlUs != 0) controlPeriodDeltaUs = currentUs - lastControlUs;
  lastControlUs = currentUs;

  if (!balanceControlActive) return;

  Snap s;
  s.take();
  if (!s.valid) return;

  if (axisTestActive) { runAxisTest(s.bodyQ, s.rate, s.wheel, s.quatStampUs, s.tilt); return; }

  if (!motorEnabled[0] && !motorEnabled[1] && !motorEnabled[2]) { disengageFromISR(2, s.tilt); return; }
  if (s.tilt > TILT_LIMIT_RAD)                             { disengageFromISR(1, s.tilt); return; }
  if ((uint32_t)(micros() - s.quatStampUs) > IMU_STALE_US) { disengageFromISR(3, s.tilt); return; }

  const float wheelMean = (s.wheel[0] + s.wheel[1] + s.wheel[2]) * (1.0f / 3.0f);

  float qCtl[4];
  removeYaw(s.bodyQ, qCtl);

  float r[3] = { refRot[0], refRot[1], refRot[2] };
  if (refAdaptEnabled && refMapValid) {
    float we[3];
    for (int j = 0; j < 3; j++) we[j] = s.wheel[j] - x0[7 + j];
    for (int a = 0; a < 3; a++) {
      float acc = 0.0f;
      for (int j = 0; j < 3; j++) acc += refMap[a][j] * we[j];
      r[a] -= REF_RATE * acc * (CONTROL_PERIOD_US * 1e-6f);
      r[a] = fmaxf(-REF_MAX, fminf(REF_MAX, r[a]));
      refRot[a] = r[a];
    }
  }
  float qRef[4];
  buildRefQuat(r, qRef);

  float xr[10];
  for (int k = 0; k < 10; k++) xr[k] = x0[k];
  for (int k = 0; k < 4; k++)  xr[k] = qRef[k];

  float x[10] = {
    qCtl[0], qCtl[1], qCtl[2], qCtl[3],
    s.rate[0],  s.rate[1],  s.rate[2],
    s.wheel[0], s.wheel[1], s.wheel[2]
  };

  float tau[3];
  computeControl(x, xr, tau);

  const float commonTau = -K_COMMON * wheelMean;
  for (int i = 0; i < 3; i++) {
    tau[i] += commonTau;
    float v = torqueToVoltage(tau[i], s.wheel[i], i);
    v = applySlewLimit(i, v);
    tauCmd[i]       = tau[i];
    sharedTarget[i] = WHEEL_SIGN[i] * v;
  }
}

static bool enableIMUReports() {
  bool a = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 2500);
  bool b = bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2500);
  return a && b;
}

static bool initIMU() {
  for (int attempt = 0; attempt < 6; attempt++) {
    pinMode(IMU_RST, OUTPUT);
    digitalWrite(IMU_RST, LOW);  delay(20);
    digitalWrite(IMU_RST, HIGH); delay(500);

    if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1)) continue;
    delay(100);
    enableIMUReports();

    uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
      if (bno08x.wasReset()) enableIMUReports();
      if (bno08x.getSensorEvent(&imuValue) && imuValue.sensorId == SH2_GAME_ROTATION_VECTOR) return true;
      delay(2);
    }
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////    SETUP1   //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void setup1() {
  while (!core0Ready) { delay(1); }

  pinMode(LED_BUILTIN, OUTPUT);

  /* ---- Setup IMU ---- */
  Wire1.setSDA(IMU_SDA);
  Wire1.setSCL(IMU_SCL);
  Wire1.begin();
  Wire1.setClock(400000);

  bool imuOk = initIMU();

  /* ---- Setup Mosfet ---- */
  pinMode(MOSFET_EN, OUTPUT);
  digitalWrite(MOSFET_EN, LOW);
  pinMode(BUS_VOLTAGE_ADC, INPUT);
  analogReadResolution(12);

  C_PRINTLN(imuOk ? F("IMU Ready")
                  : F("IMU Setup Failed"));

  /* ---- Balance quaternion & adaptive reference map ---- */
  applyEdgeTrimsToX0();
  refMapValid = computeRefMap();
  if (!refMapValid) { C_PRINTLN(F("WARNING: adaptive reference map is singular - REF disabled.")); }

  /* ---- Setup Timers ---- */
  my_alarms[0] = hardware_alarm_claim_unused(true);
  my_alarms[1] = hardware_alarm_claim_unused(true);

  hardware_alarm_set_callback(my_alarms[0], imu_callback);
  hardware_alarm_set_callback(my_alarms[1], control_callback);

  irq_set_priority(hardware_alarm_get_irq_num(my_alarms[0]), IMU_PRIORITY);
  irq_set_priority(hardware_alarm_get_irq_num(my_alarms[1]), CONTROL_PRIORITY);

  hardware_alarm_set_target(my_alarms[0], make_timeout_time_us(IMU_PERIOD_US));
  hardware_alarm_set_target(my_alarms[1], make_timeout_time_us(CONTROL_PERIOD_US));

  if (imuOk) { printHelp(); }
}


/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////    LOOP1   ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void loop1() {
  now = micros();

  if (now - lastAdcUs >= ADC_PERIOD_US) { lastAdcUs = now; updateDumpLoad(); }
  handleSerial();

  uint8_t reason = tripReason;
  if (reason) {
    tripReason = 0;
    switch (reason) {
      case 1: C_PRINT(F("SAFETY DISENGAGE: tilt limit exceeded, ")); C_PRINT(tripTilt * 57.29578f, 1); C_PRINTLN(F(" deg from target.")); verbose = false; break;
      case 2: C_PRINTLN(F("SAFETY DISENGAGE: a motor was disabled while balancing.")); verbose = false; break;
      case 3: C_PRINTLN(F("SAFETY DISENGAGE: IMU data went stale.")); verbose = false; break;
    }
  }

  float tilt = fabsf(tiltErr);
  digitalWrite(LED_BUILTIN, (tilt >= 0.00174533f && tilt < 0.0436332f) ? HIGH : LOW);

  if (verbose && (now - lastPrintUs >= PRINT_PERIOD_US)) {
    uint32_t dtUs = now - lastPrintUs;
    lastPrintUs = now;

    Snap s;
    s.take();

    uint32_t fc = focLoopCount;
    float focHz = (float)(fc - lastFocCount) * 1e6f / (float)dtUs;
    lastFocCount = fc;

    const float RAD_TO_RPM = 60.0f / (2.0f * PI);

    if (axisTestActive && balanceControlActive) {
      const int m = axisMotor;
      if (!Bluetooth) {
        snprintf(buffer, sizeof(buffer),
          "1D M%d(%c) Theta:% .3f deg\tTrim:% .3f deg\tOmega:% .3f\tUa:% .3f\tRPM:% .1f\tTau:% .4f\tVolt:% .2f\tFOC_Hz:%.0f\tCtrl_dt:%lu",
          m + 1, AXIS_NAME[MOTOR_AXIS[m]],
          (float)d1Theta * 57.29578f, (float)d1Trim * 57.29578f, (float)d1Omega, (float)d1Ua,
          s.wheel[m] * RAD_TO_RPM, (float)tauCmd[m], (float)sharedTarget[m],
          focHz, (unsigned long)controlPeriodDeltaUs);
      } else {
        snprintf(buffer, sizeof(buffer), "Th:%.2f Tr:%.2f R:%.0f",
          (float)d1Theta * 57.29578f, (float)d1Trim * 57.29578f, s.wheel[m] * RAD_TO_RPM);
      }
      C_PRINTLN(buffer);
      return;
    }

    if (!Bluetooth) {
      float qrel[4], qc[4];
      removeYaw(s.bodyQ, qc);
      relativeQuat(qc, qrel);

      snprintf(buffer, sizeof(buffer),
        "Tilt:%-7.4f\tRef:% .3f,% .3f,% .3f\tQrel:% .5f,% .5f,% .5f,% .5f\t"
        "RawQ:% .5f,% .5f,% .5f,% .5f\tRate:%-6.3f,%-6.3f,%-6.3f\t"
        "RPM:%-6.1f,%-6.1f,%-6.1f\tTau:%-7.4f,%-7.4f,%-7.4f\t"
        "Volt:%-6.2f,%-6.2f,%-6.2f",
        s.tilt * 57.29578f,
        (float)refRot[0] * 57.29578f, (float)refRot[1] * 57.29578f, (float)refRot[2] * 57.29578f,
        qrel[0], qrel[1], qrel[2], qrel[3],
        s.rawQ[0], s.rawQ[1], s.rawQ[2], s.rawQ[3],
        s.rate[0], s.rate[1], s.rate[2],
        s.wheel[0] * RAD_TO_RPM, s.wheel[1] * RAD_TO_RPM, s.wheel[2] * RAD_TO_RPM,
        (float)tauCmd[0], (float)tauCmd[1], (float)tauCmd[2],
        (float)sharedTarget[0], (float)sharedTarget[1], (float)sharedTarget[2]);
    } else {
      snprintf(buffer, sizeof(buffer), "Tilt:%-7.4f", s.tilt * 57.29578f);
    }
    C_PRINTLN(buffer);
  }
}
