/* ============================================================================
 *  HENES T870 - Low Level Controller (LLC)  for Arduino Mega 2560
 * ----------------------------------------------------------------------------
 *  Talks RS232 to the HLC (PC, ROS vehicle_bridge node).
 *
 *  Protocol (T870 spec + Phase B closed-loop-on-HLC):
 *    HLC -> LLC : 13 byte, BIG-endian      (command)
 *      'S''T''X' | A/M | ESTOP | GEAR | SPEED(u16,0..1000) | STEER(i16) | ALIVE | 0x0D 0x0A
 *    LLC -> HLC : 17 byte, LITTLE-endian   (feedback)
 *      'S''T''X' | A/M | ESTOP | GEAR | SPEED(u16) | STEER(u16) | ENC(i32) | ALIVE | 0x0D 0x0A
 *    A/M  : 0=Manual 1=Auto      ESTOP: 0=off 1=on
 *    GEAR : 0=Forward 1=Neutral 2=Backward
 *    PHASE B steering: the HLC owns the steering loop. The LLC is a dumb actuator.
 *      HLC->LLC STEER  = signed steer-motor PWM (-255..+255). sign = direction, magnitude = PWM.
 *      LLC->HLC SPEED  = safe-clamped analogRead(A8) (0..1000)  <- HLC closes the loop on this
 *      LLC->HLC STEER  = steer PWM the LLC actually drove (0..255, after safety guard)
 *    RS232: 115200 8N1, cycle 50 ms (20 Hz)
 *
 *  Pin map (PDF "Low Level Controller Layout"):
 *    Drive motor driver (Cytron MD20A): PWM=4,  DIR=22
 *    Steer motor driver (MD10C R3)    : PWM=5,  DIR=24
 *    Drive encoder                    : ENC_A=3, ENC_B=2   (INT1, INT0)
 *    Steering potentiometer           : POT=A8
 *    RC receiver (Spektrum AR620)     : THR=A0 STR=A1 RUDD=A2 ELEV=A3 GEAR=A4 AorM=A5
 *    RS232 to HLC                     : Serial2 (RX2=17, TX2=16)
 * ========================================================================== */

#include <Arduino.h>

// ---------------------------------------------------------------- pin map
const uint8_t PIN_DRIVE_PWM = 4;
const uint8_t PIN_DRIVE_DIR = 22;
const uint8_t PIN_STEER_PWM = 5;
const uint8_t PIN_STEER_DIR = 24;
const uint8_t PIN_ENC_A     = 3;   // INT1
const uint8_t PIN_ENC_B     = 2;   // INT0
const uint8_t PIN_STEER_POT = A8;

#define USE_RC 0                   // 1 = RC transmitter manual override (see notes at bottom)
const uint8_t PIN_RC_AORM   = A5;
const uint8_t PIN_RC_THR    = A0;
const uint8_t PIN_RC_STR    = A1;
const uint8_t PIN_RC_GEAR   = A4;

#define HLC Serial                 // USB link to PC (ttyACM0)
// DBG disabled: HLC=Serial shares the USB port, debug text corrupts binary protocol
#define DBG_ENABLED 0

// ---------------------------------------------------------------- protocol
const uint8_t  STX0 = 0x53, STX1 = 0x54, STX2 = 0x58;   // 'S' 'T' 'X'
const uint8_t  ETX0 = 0x0D, ETX1 = 0x0A;
const uint8_t  RX_LEN = 13;
const uint8_t  TX_LEN = 17;

const uint8_t  AM_MANUAL = 0, AM_AUTO = 1;
const uint8_t  ES_OFF = 0, ES_ON = 1;
const uint8_t  GEAR_FWD = 0, GEAR_NEU = 1, GEAR_BWD = 2;

// ---------------------------------------------------------------- calibration  (TUNE ON HARDWARE)
const uint16_t SPEED_MAX_RAW   = 1000;   // HLC SPEED full-scale
const uint8_t  DRIVE_PWM_MAX   = 255;    // analogWrite max (cap for safety, e.g. 180)

// Phase B: the steering control loop moved to the HLC. The LLC only keeps these two
// pot limits as a hardware end-stop guard, and a PWM safety cap.
const int      POT_FULL_LEFT   = 30;     // left end-stop guard (measure & set)
const int      POT_FULL_RIGHT  = 970;    // right end-stop guard (measure & set)
const uint8_t  STEER_PWM_MAX   = 220;    // steer motor pwm safety cap (HLC must stay <= this)
// (removed Phase A internal P-control / dead-reckoning consts: POT_CENTER, STEER_CMD_MAX,
//  STEER_DEADBAND, STEER_KP, STEER_PWM_MIN, POT_DR_RATE, POT_SETTLE -> now HLC params)

const uint16_t CONTROL_PERIOD_MS = 50;   // 20 Hz
const uint16_t COMM_TIMEOUT_MS   = 300;  // no HLC packet -> safety stop

const long     ENC_PPR         = 144;    // pulses per revolution (info only)

// ADC reporting guard. The pot can jump across the electrical wrap boundary; keep the
// reported feedback pinned at the nearest edge instead of feeding a false full-scale jump.
const int      ADC_SAFE_MIN    = 0;
const int      ADC_SAFE_MAX    = 1000;
const int      ADC_WRAP_LOW    = 100;
const int      ADC_WRAP_HIGH   = 900;
int            g_safe_adc      = 500;

// DIR polarity: flip if a wheel/steer turns the wrong way.
const uint8_t  DRIVE_FWD_LEVEL = LOW;    // DIR level for forward
const uint8_t  STEER_POS_LEVEL = LOW;    // DIR level to move pot toward RIGHT

// ---------------------------------------------------------------- state
volatile long  g_enc = 0;                // encoder count (ISR)

struct Cmd {
  uint8_t  am    = AM_MANUAL;
  uint8_t  estop = ES_ON;                 // boot safe: stopped
  uint8_t  gear  = GEAR_NEU;
  uint16_t speed = 0;                     // 0..1000
  int16_t  steer = 0;                     // Phase B: signed steer-motor PWM (-255..+255)
  uint8_t  alive = 0;
} g_cmd;

uint8_t  g_rx[RX_LEN];
uint8_t  g_rx_n   = 0;
uint32_t g_last_rx_ms = 0;
uint8_t  g_tx_alive  = 0;
uint32_t g_last_ctrl_ms = 0;
int      g_steer_applied = 0; // steer PWM actually driven (signed); echoed in feedback

// ---------------------------------------------------------------- pot read
// Phase B: plain averaged ADC read. NOT used for control (the HLC closes the loop on the
// raw A8 feedback) -- only for the LLC's mechanical end-stop safety guard. During motor
// motion this is EMI-corrupted, so the guard is best-effort with margin, not precise.
int readPot() {
  long sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    sum += analogRead(PIN_STEER_POT);
    delayMicroseconds(300);
  }
  return (int)(sum / 8);
}

int safeAdcValue(int raw_adc) {
  if (g_safe_adc > ADC_WRAP_HIGH && raw_adc < ADC_WRAP_LOW) {
    raw_adc = ADC_SAFE_MAX;
  } else if (g_safe_adc < ADC_WRAP_LOW && raw_adc > ADC_WRAP_HIGH) {
    raw_adc = ADC_SAFE_MIN;
  }

  if (raw_adc > ADC_SAFE_MAX) raw_adc = ADC_SAFE_MAX;
  if (raw_adc < ADC_SAFE_MIN) raw_adc = ADC_SAFE_MIN;

  g_safe_adc = raw_adc;
  return g_safe_adc;
}

// ---------------------------------------------------------------- encoder ISR
void isrEncA() {
  // quadrature: on A rising, B level gives direction
  if (digitalRead(PIN_ENC_B)) g_enc++;
  else                        g_enc--;
}

// ---------------------------------------------------------------- setup
void setup() {
  pinMode(PIN_DRIVE_PWM, OUTPUT);
  pinMode(PIN_DRIVE_DIR, OUTPUT);
  pinMode(PIN_STEER_PWM, OUTPUT);
  pinMode(PIN_STEER_DIR, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_STEER_POT, INPUT);

  driveStop();
  analogWrite(PIN_STEER_PWM, 0);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, RISING);

  HLC.begin(115200);

  g_last_rx_ms   = millis();
  g_last_ctrl_ms = millis();
}

// ---------------------------------------------------------------- main loop
void loop() {
  readSerial();                            // ingest HLC bytes -> g_cmd

  uint32_t now = millis();
  if (now - g_last_ctrl_ms >= CONTROL_PERIOD_MS) {
    g_last_ctrl_ms += CONTROL_PERIOD_MS;

    int pot = readPot();                   // read once per cycle (slew-rate filter inside)
    bool comm_lost = (now - g_last_rx_ms) > COMM_TIMEOUT_MS;
    bool stop = g_cmd.estop || comm_lost;

#if USE_RC
    applyRcOverride(stop);                 // RC manual switch can take over
#endif

    if (stop) {
      driveStop();
      steerHold();                          // steer motor off
    } else {
      driveControl(g_cmd.gear, g_cmd.speed);
      steerApply(g_cmd.steer, pot);         // Phase B: pass HLC's PWM through (+ safety guard)
    }

    sendFeedback(comm_lost, pot);
  }
}

// ---------------------------------------------------------------- drive motor
void driveStop() {
  analogWrite(PIN_DRIVE_PWM, 0);
}

void driveControl(uint8_t gear, uint16_t speed_raw) {
  if (gear == GEAR_NEU || speed_raw == 0) { driveStop(); return; }

  if (gear == GEAR_FWD) digitalWrite(PIN_DRIVE_DIR, DRIVE_FWD_LEVEL);
  else                  digitalWrite(PIN_DRIVE_DIR, !DRIVE_FWD_LEVEL);  // backward

  uint16_t pwm = (uint32_t)speed_raw * DRIVE_PWM_MAX / SPEED_MAX_RAW;
  if (pwm > DRIVE_PWM_MAX) pwm = DRIVE_PWM_MAX;
  analogWrite(PIN_DRIVE_PWM, (uint8_t)pwm);
}

// ---------------------------------------------------------------- steer motor (Phase B: HLC-driven PWM passthrough + safety guard)
// The HLC closes the steering loop and sends a signed motor PWM. The LLC clamps it to a
// safe magnitude and refuses to push past the mechanical end-stops, then drives the motor.
void steerApply(int16_t pwm_cmd, int pot) {
  int pwm = pwm_cmd;
  if (pwm >  STEER_PWM_MAX) pwm =  STEER_PWM_MAX;   // safety cap
  if (pwm < -STEER_PWM_MAX) pwm = -STEER_PWM_MAX;

  // mechanical end-stop guard (pot increases toward RIGHT via STEER_POS_LEVEL)
  if (pwm > 0 && pot >= POT_FULL_RIGHT) pwm = 0;    // already at/past right limit
  if (pwm < 0 && pot <= POT_FULL_LEFT)  pwm = 0;    // already at/past left limit

  int mag = abs(pwm);
  if (mag == 0) { analogWrite(PIN_STEER_PWM, 0); g_steer_applied = 0; return; }

  digitalWrite(PIN_STEER_DIR, (pwm > 0) ? STEER_POS_LEVEL : !STEER_POS_LEVEL);
  analogWrite(PIN_STEER_PWM, (uint8_t)mag);
  g_steer_applied = pwm;                            // signed; magnitude echoed in feedback
}

void steerHold() {
  analogWrite(PIN_STEER_PWM, 0);
  g_steer_applied = 0;
}

// ---------------------------------------------------------------- RS232 receive (13B, big-endian)
void readSerial() {
  while (HLC.available()) {
    uint8_t b = HLC.read();

    // resync on header
    if (g_rx_n == 0) { if (b == STX0) g_rx[g_rx_n++] = b; continue; }
    if (g_rx_n == 1) { g_rx[g_rx_n++] = b; if (b != STX1) g_rx_n = 0; continue; }
    if (g_rx_n == 2) { g_rx[g_rx_n++] = b; if (b != STX2) g_rx_n = 0; continue; }

    g_rx[g_rx_n++] = b;
    if (g_rx_n >= RX_LEN) {
      g_rx_n = 0;
      if (g_rx[11] == ETX0 && g_rx[12] == ETX1) parseCmd();
    }
  }
}

void parseCmd() {
  g_cmd.am    = g_rx[3];
  g_cmd.estop = g_rx[4];
  g_cmd.gear  = g_rx[5];
  g_cmd.speed = ((uint16_t)g_rx[6] << 8) | g_rx[7];        // big-endian drive speed 0..1000
  g_cmd.steer = (int16_t)(((uint16_t)g_rx[8] << 8) | g_rx[9]);  // Phase B: signed steer PWM
  g_cmd.alive = g_rx[10];
  g_last_rx_ms = millis();
}

// ---------------------------------------------------------------- RS232 send (17B, little-endian)
void sendFeedback(bool comm_lost, int pot) {
  // PHASE B: SPEED field = safe-clamped analogRead(A8) (0..1000). The HLC closes the
  // steering loop on this. STEER field = the steer PWM the LLC actually drove (0..255,
  // after the safety cap + end-stop guard), for HLC monitoring/debug.
  (void)pot;                                                        // guard uses pot in loop(), not here
  uint16_t raw_a8   = (uint16_t)safeAdcValue(analogRead(PIN_STEER_POT));
  uint16_t pwm_fb   = (uint16_t)abs(g_steer_applied);               // applied steer PWM magnitude
  long     enc;
  noInterrupts(); enc = g_enc; interrupts();
  uint8_t  es_fb    = (g_cmd.estop || comm_lost) ? ES_ON : ES_OFF;

  uint8_t t[TX_LEN];
  t[0] = STX0; t[1] = STX1; t[2] = STX2;
  t[3] = g_cmd.am;
  t[4] = es_fb;
  t[5] = g_cmd.gear;
  t[6] = (uint8_t)(raw_a8 & 0xFF); t[7] = (uint8_t)(raw_a8 >> 8);    // SPEED u16 LE = raw A8
  t[8] = (uint8_t)(pwm_fb & 0xFF); t[9] = (uint8_t)(pwm_fb >> 8);    // STEER u16 LE = applied PWM
  t[10] = (uint8_t)(enc        & 0xFF);                              // ENC i32 LE
  t[11] = (uint8_t)((enc >> 8)  & 0xFF);
  t[12] = (uint8_t)((enc >> 16) & 0xFF);
  t[13] = (uint8_t)((enc >> 24) & 0xFF);
  t[14] = g_tx_alive++;
  t[15] = ETX0; t[16] = ETX1;

  HLC.write(t, TX_LEN);
}

// ---------------------------------------------------------------- RC override (optional)
#if USE_RC
// NOTE: A0..A5 on the Mega have NO pin-change interrupt, so we read RC pulses
// with pulseIn (blocking). Keep timeouts short; only read what we use.
uint16_t rcPulse(uint8_t pin) { return pulseIn(pin, HIGH, 25000); }   // us, 0 = no signal

void applyRcOverride(bool &stop) {
  uint16_t aorm = rcPulse(PIN_RC_AORM);
  if (aorm == 0) return;                    // no RC -> leave HLC in control
  bool rc_manual = (aorm < 1500);           // switch low = manual (tune)
  if (!rc_manual) { g_cmd.am = AM_AUTO; return; }

  g_cmd.am = AM_MANUAL;
  uint16_t thr  = rcPulse(PIN_RC_THR);      // 1000..2000 us
  uint16_t str  = rcPulse(PIN_RC_STR);
  uint16_t gear = rcPulse(PIN_RC_GEAR);

  if (thr) {
    int v = (int)thr - 1500;                // -500..+500
    if (abs(v) < 40) { g_cmd.gear = GEAR_NEU; g_cmd.speed = 0; }
    else {
      g_cmd.gear  = (v > 0) ? GEAR_FWD : GEAR_BWD;
      g_cmd.speed = map(abs(v), 40, 500, 0, SPEED_MAX_RAW);
    }
  }
  // Phase B: steer is now a direct motor PWM; RC stick drives it open-loop (manual override)
  if (str)  g_cmd.steer = map((int)str, 1000, 2000, -STEER_PWM_MAX, STEER_PWM_MAX);
  if (gear && gear > 1700) g_cmd.gear = GEAR_BWD;   // example aux-channel use
  stop = false;                              // RC manual drives even without HLC
}
#endif
