/*
 ╔═══════════════════════════════════════════════════════════════════╗
 ║        TDP EE3 Team 7  —  ESP32-WROOM-32UE Rover Controller      ║
 ║                                                                   ║
 ║  Core 0 / Task 1: HC-SR04 ranging · 74HC165 encoder · 2-D map   ║
 ║                   servo spin · obstacle-avoidance FSM            ║
 ║  Core 1 / Task 2: WiFi AP · WebSocket dashboard · UDP joystick  ║
 ║                   I²C L432KC slave RX · ADC sense/batt · UART  ║
 ╚═══════════════════════════════════════════════════════════════════╝

 Required libraries (Library Manager / PlatformIO):
   ESPAsyncWebServer  (by lacamera / me-no-dev)
   AsyncTCP           (by dvarrel  / me-no-dev)

 ─── IMPORTANT UART NOTE ────────────────────────────────────────────
 Serial (UART0, GPIO1=TX, GPIO3=RX) is the hardware connection to
 the L432KC.  USB-serial is the same UART on the ESP32, so Serial
 cannot be used for USB debug in deployment.  All diagnostic output
 is routed to the web dashboard debug panel via the WebSocket.
 ────────────────────────────────────────────────────────────────────
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <driver/ledc.h>
#include <stdarg.h>

// ═══════════════════════════════════════════════════════════════════
//  PIN MAP  (PCB3 schematic, page 5 of uploaded PDF)
// ═══════════════════════════════════════════════════════════════════
#define PIN_TRIG          22   // OUT  → Trig_HC04
#define PIN_ECHO          21   // IN   ← Echo_HC04
#define PIN_SERVO         19   // OUT  → Servo_Ctrl     (LEDC ch 0)
#define PIN_ENC_LATCH     12   // OUT  → Encoder_Latch  (74HC165 PL, active-LOW)
#define PIN_ENC_CLK       33   // OUT  → Encoder_Clock
#define PIN_ENC_DATA      36   // IN   ← Encoder_Data   (SENSOR_VP, input-only GPIO)
#define PIN_MEAS_INT      23   // IN   ← Dist_Meas_Intr (RISING edge from enc board)
#define PIN_OBS_ACTIVE    32   // OUT  → ObsAv_Active   (interrupt to L432KC)
#define PIN_OBS_CA1       27   // OUT  → ObsAv_CA1
#define PIN_OBS_CA2       14   // OUT  → ObsAv_CA2
#define PIN_JS_SPEED      26   // OUT  → JS_Speed       (ESP32 DAC2)
#define PIN_JS_TURN       25   // OUT  → JS_Turning     (ESP32 DAC1)
#define PIN_JS_CYCLE      18   // OUT  → JS_Cycle_Mode  (pulse per cycle)
#define PIN_FOLLOW_ME      4   // OUT  → Follow_Me_MCU  (toggle)
#define PIN_STOP           0   // OUT  → Stop_MCU       (toggle) ⚠ boot-strap pin
#define PIN_SDA           17   // I²C  → I2C_SDA
#define PIN_SCL           16   // I²C  → I2C_Clk
#define PIN_BATT          39   // IN   ← Batt_Lvl_Prot  (SENSOR_VN, input-only)
#define PIN_SENSE_L       34   // IN   ← Sense_A_Prot   (left  motor current)
#define PIN_SENSE_R       35   // IN   ← Sense_B_Prot   (right motor current)
#define PIN_BUZZER         5   // OUT  → Buzzer_MCU
// Serial (UART0): TX=GPIO1 (UART_TX_ESP), RX=GPIO3 (UART_TX_L432K from L432KC)

// ═══════════════════════════════════════════════════════════════════
//  NETWORK CONFIG
// ═══════════════════════════════════════════════════════════════════
static const char*      AP_SSID  = "TDP3 Team 7";
static const char*      AP_PASS  = "saaddideverything";
static const IPAddress  AP_IP    (192, 168, 4, 1);
static const IPAddress  AP_GW    (192, 168, 4, 1);
static const IPAddress  AP_MASK  (255, 255, 255, 0);
static const uint16_t   UDP_PORT = 4210;
static const uint8_t    KL_ADDR  = 0x50;   // ESP32 own I²C slave address (L432KC is master)

// ═══════════════════════════════════════════════════════════════════
//  HARDWARE CONSTANTS
// ═══════════════════════════════════════════════════════════════════
static const int   N_SW           = 22;
static const float DEG_PER_SW     = 360.0f / N_SW;   // ≈16.36°

// Rover sensor insets from rover body edge (mm)
static const float INSET_FRONT    =  80.0f;
static const float INSET_SIDE     = 130.0f;
static const float INSET_BACK     = 150.0f;

// Obstacle detection thresholds — raw sensor distance (mm)
static const float THR_FRONT      = 400.0f;
static const float THR_SIDE       = 300.0f;
static const float THR_BACK       = 350.0f;

// GHM-01 (12 V, 200 RPM, 620 g·cm rated)
static const float MTR_RPM_NL     = 200.0f;
static const float MTR_RPM_RT     = 177.0f;
static const float MTR_I_NL       = 0.113f;
static const float MTR_I_RT       = 0.233f;
static const float MTR_TQ_RT      = 0.06079f;  // N·m  (620 g·cm)
static const float MTR_Kt         = MTR_TQ_RT / (MTR_I_RT - MTR_I_NL);   // ≈ 0.507 N·m/A
static const float MTR_SPD_REG    = (MTR_RPM_NL - MTR_RPM_RT) / (MTR_I_RT - MTR_I_NL); // RPM/A

// Battery — 10-cell NiMH, max 14.5 V
// Voltage divider: Vin(≤15 V) → 3.3 V   ratio = 3.3/15 = 0.22
static const float VDIV           = 3.3f / 15.0f;
static const float BATT_MAX       = 14.5f;
static const float BATT_MIN       = 10.0f;   // 1.0 V/cell depleted

// Servo LEDC — 50 Hz, 16-bit, 5 % duty = 1 ms pulse = full CW spin
static const uint32_t SRV_HZ     = 50;
static const uint8_t  SRV_BITS   = 16;
static const uint32_t SRV_DUTY   = (uint32_t)(0.05f * ((1u << SRV_BITS) - 1)); // 3277

// ═══════════════════════════════════════════════════════════════════
//  DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════
enum RoverState : uint8_t {
  RS_FOLLOWING=0, RS_TURNING90, RS_SEARCHING,
  RS_TRAFFICSTOP, RS_OBSTACLEAVOID, RS_MANUAL
};

struct L432KTelem {
  uint8_t trafficLight;   // 0=unknown 1=green 2=amber 3=red
  uint8_t roverState;     // RoverState
  uint8_t errorFlags;     // bit-field
  uint8_t lineSensors;    // FL[3] LC[2] LR[1] FR[0]
};

struct MapPt { float angle; float dist; };

// ─── Single shared data block (guarded by sdMutex) ─────────────────
struct SharedData {
  MapPt      map[N_SW];
  bool       obstacleDetected;

  float      iL, iR;          // motor current  (A)
  float      tqL, tqR;        // torque         (N·m)
  float      rpmL, rpmR;      // speed          (RPM)

  float      battV;
  float      battSOC;          // %

  L432KTelem mcu;
  char       dbgBuf[1024];    // circular UART debug log

  float      pitch, roll, yaw, thr;
  uint16_t   jsBtns;
  bool       jsEnabled;
  bool       followMeOn;
  bool       stopOn;
} SD;

static SemaphoreHandle_t sdMutex;

// ─── ISR flag ────────────────────────────────────────────────────────
static volatile bool measFlag = false;
static portMUX_TYPE  muxISR   = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onMeasInt() {
  portENTER_CRITICAL_ISR(&muxISR);
  measFlag = true;
  portEXIT_CRITICAL_ISR(&muxISR);
}

// ─── Task handles ────────────────────────────────────────────────────
static TaskHandle_t hTask1, hTask2;

// ─── Async server objects ─────────────────────────────────────────────
AsyncWebServer httpServer(80);
AsyncWebSocket wsServer("/ws");
WiFiUDP        udpSock;

// ═══════════════════════════════════════════════════════════════════
//  INTERNAL DEBUG LOGGER → dashboard debug panel
// ═══════════════════════════════════════════════════════════════════
static void dbg(const char* fmt, ...) {
  char line[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line)-2, fmt, ap);
  va_end(ap);
  strcat(line, "\n");

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(8)) == pdTRUE) {
    size_t cur = strlen(SD.dbgBuf), add = strlen(line);
    if (cur + add >= sizeof(SD.dbgBuf) - 1) {
      // drop oldest line
      char* nl = strchr(SD.dbgBuf, '\n');
      if (nl) memmove(SD.dbgBuf, nl+1, strlen(nl));
    }
    strncat(SD.dbgBuf, line, sizeof(SD.dbgBuf) - strlen(SD.dbgBuf) - 1);
    xSemaphoreGive(sdMutex);
  }
}

// ═══════════════════════════════════════════════════════════════════
//  SWITCH ENCODER  (3 × 74HC165 daisy-chain = 24 bits, 22 used)
//  Chain order in schematic: U3→U2→U1→DATA pin
//  First bits shifted = U3 D7..D0 (switches 17-22 + 2 unused)
//  Then U2 D7..D0 (switches 9-16), then U1 D7..D0 (switches 1-8)
// ═══════════════════════════════════════════════════════════════════
static uint32_t readEncoder() {
  uint32_t val = 0;
  // Parallel-load strobe (PL active-LOW)
  digitalWrite(PIN_ENC_LATCH, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_ENC_LATCH, HIGH);
  delayMicroseconds(5);
  // Clock out 24 bits, MSB first
  for (int i = 0; i < 24; i++) {
    val = (val << 1) | (digitalRead(PIN_ENC_DATA) ? 1u : 0u);
    digitalWrite(PIN_ENC_CLK, HIGH); delayMicroseconds(2);
    digitalWrite(PIN_ENC_CLK, LOW);  delayMicroseconds(2);
  }
  // Remap: bits arrive U3→U2→U1; re-order so bit-i = switch-(i+1)
  // After 24-clock read, lower 22 bits already map to SW1..SW22
  // (verified by calibration; reverse if sense is opposite)
  return val & 0x3FFFFFu;
}

// Returns 0-based index of first active switch, or -1
static int firstActive(uint32_t enc) {
  for (int i = 0; i < N_SW; i++) if (enc & (1u << i)) return i;
  return -1;
}

// ═══════════════════════════════════════════════════════════════════
//  HC-SR04  (blocking — lives only in Task-1)
// ═══════════════════════════════════════════════════════════════════
static float measureMM() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  // 35 ms timeout ≈ 6 m
  long us = pulseIn(PIN_ECHO, HIGH, 35000);
  return (us == 0) ? -1.0f : us * 0.17150f;  // mm: (us × 343 000/2) / 1e6
}

// ═══════════════════════════════════════════════════════════════════
//  OBSTACLE GEOMETRY
// ═══════════════════════════════════════════════════════════════════
static float threshForAngle(float deg) {
  while (deg < 0)   deg += 360;
  while (deg >= 360) deg -= 360;
  if (deg <= 45 || deg >= 315) return THR_FRONT;
  if (deg >= 135 && deg <= 225) return THR_BACK;
  return THR_SIDE;
}
static bool isObstacle(int swi, float mm) {
  return (mm > 0) && (mm < threshForAngle(swi * DEG_PER_SW));
}

// ═══════════════════════════════════════════════════════════════════
//  SKID-STEER HELPER  (CA1, CA2 truth table)
// ═══════════════════════════════════════════════════════════════════
static inline void setCA(uint8_t a, uint8_t b) {
  digitalWrite(PIN_OBS_CA1, a); digitalWrite(PIN_OBS_CA2, b);
}
#define SKID_FWD()   setCA(0,0)
#define SKID_RIGHT() setCA(1,0)
#define SKID_LEFT()  setCA(0,1)
#define SKID_BACK()  setCA(1,1)

// ═══════════════════════════════════════════════════════════════════
//  TASK 1 — Obstacle Avoidance + 2-D Map  (pinned to Core 0)
// ═══════════════════════════════════════════════════════════════════
static void task1(void*) {
  dbg("[T1] Core %d: obstacle/map task started", xPortGetCoreID());

  // Start servo spinning continuously at 5 % duty
  ledcSetup(0, SRV_HZ, SRV_BITS);
  ledcAttachPin(PIN_SERVO, 0);
  ledcWrite(0, SRV_DUTY);
  dbg("[T1] Servo: %lu Hz, duty=%lu (5%%)", SRV_HZ, SRV_DUTY);

  // Enable measurement interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_MEAS_INT), onMeasInt, RISING);
  dbg("[T1] Meas interrupt attached on GPIO%d", PIN_MEAS_INT);

  // ── Obstacle FSM state ──────────────────────────────────────────
  bool     obsActive  = false;
  int      obsStep    = 0;
  uint32_t stepStart  = 0;
  // Timings (ms) — tune to rover geometry
  const uint32_t T_BACK  =  800;
  const uint32_t T_RIGHT =  600;
  const uint32_t T_FWD   = 1200;
  const uint32_t T_LEFT  =  600;

  for (;;) {
    // ── Measurement interrupt ──────────────────────────────────────
    bool doMeas = false;
    portENTER_CRITICAL(&muxISR);
    if (measFlag) { measFlag = false; doMeas = true; }
    portEXIT_CRITICAL(&muxISR);

    if (doMeas && !obsActive) {
      uint32_t enc = readEncoder();
      int swi = firstActive(enc);
      if (swi >= 0) {
        float mm    = measureMM();
        float angle = swi * DEG_PER_SW;

        // Update 2-D map
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          SD.map[swi] = {angle, mm};
          xSemaphoreGive(sdMutex);
        }
        dbg("[T1] sw=%2d  ang=%5.1f°  dist=%6.1fmm", swi, angle, mm);

        // Front position (switch 0 = 0° = forward)
        if (swi == 0 && isObstacle(0, mm)) {
          dbg("[T1] !! OBSTACLE FRONT  dist=%.1f mm !!", mm);
          digitalWrite(PIN_OBS_ACTIVE, HIGH);   // raise interrupt to L432KC
          if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            SD.obstacleDetected = true;
            xSemaphoreGive(sdMutex);
          }
          obsActive = true; obsStep = 0; stepStart = millis();
        }
      }
    }

    // ── Obstacle avoidance FSM ─────────────────────────────────────
    if (obsActive) {
      uint32_t el = millis() - stepStart;
      switch (obsStep) {
        case 0: // Back up
          SKID_BACK();
          if (el > T_BACK)  { obsStep=1; stepStart=millis(); dbg("[T1] OA: back→right"); }
          break;
        case 1: // Turn right
          SKID_RIGHT();
          if (el > T_RIGHT) { obsStep=2; stepStart=millis(); dbg("[T1] OA: right→fwd");  }
          break;
        case 2: // Forward (clear the obstacle)
          SKID_FWD();
          if (el > T_FWD)   { obsStep=3; stepStart=millis(); dbg("[T1] OA: fwd→left");   }
          break;
        case 3: // Turn left to re-align
          SKID_LEFT();
          if (el > T_LEFT)  {
            obsActive = false;
            digitalWrite(PIN_OBS_ACTIVE, LOW);
            SKID_FWD();
            if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
              SD.obstacleDetected = false;
              xSemaphoreGive(sdMutex);
            }
            dbg("[T1] OA: complete, resumed");
          }
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ═══════════════════════════════════════════════════════════════════
//  ADC TELEMETRY
// ═══════════════════════════════════════════════════════════════════
static void readTelemetry() {
  // ── Battery ───────────────────────────────────────────────────────
  float vadc = analogRead(PIN_BATT) / 4095.0f * 3.3f;
  float bV   = vadc / VDIV;
  float soc  = constrain((bV - BATT_MIN) / (BATT_MAX - BATT_MIN) * 100.0f, 0, 100);

  // ── Motor current (sense formula from spec) ────────────────────────
  // iL = SenseL.read() / (0.45 * 1.0269) / 2   (L432KC mbed normalised read 0-1)
  // ESP32 ADC gives 0-4095, convert to 0-1 first
  float sL = analogRead(PIN_SENSE_L) / 4095.0f;
  float sR = analogRead(PIN_SENSE_R) / 4095.0f;
  float iL = sL / (0.45f * 1.0269f) / 2.0f;
  float iR = sR / (0.44f * 1.0269f) / 2.0f;

  // ── Motor torque & RPM — linear model ─────────────────────────────
  auto motorCalc = [](float I, float& tq, float& rpm) {
    float Ia = max(0.0f, I - MTR_I_NL);
    tq  = MTR_Kt * Ia;
    rpm = max(0.0f, MTR_RPM_NL - MTR_SPD_REG * Ia);
  };
  float tqL,rpmL,tqR,rpmR;
  motorCalc(iL,tqL,rpmL);
  motorCalc(iR,tqR,rpmR);

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    SD.battV=bV; SD.battSOC=soc;
    SD.iL=iL; SD.iR=iR;
    SD.tqL=tqL; SD.tqR=tqR;
    SD.rpmL=rpmL; SD.rpmR=rpmR;
    xSemaphoreGive(sdMutex);
  }
}

// ═══════════════════════════════════════════════════════════════════
//  I²C SLAVE — RECEIVE FROM L432KC
//  The L432KC is the master and pushes a 4-byte telemetry packet
//  whenever any field changes.  Wire.onReceive fires in a dedicated
//  FreeRTOS task (not a true ISR), so a short-timeout mutex is safe.
// ═══════════════════════════════════════════════════════════════════
static void onReceiveFromL432K(int numBytes) {
  if (numBytes < 4) {
    // drain any partial packet
    while (Wire.available()) Wire.read();
    dbg("[I2C] Short packet from L432K: %d bytes", numBytes);
    return;
  }
  L432KTelem t;
  t.trafficLight = Wire.read();
  t.roverState   = Wire.read();
  t.errorFlags   = Wire.read();
  t.lineSensors  = Wire.read();
  // drain any extra bytes the master accidentally sent
  while (Wire.available()) Wire.read();

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    SD.mcu = t;
    xSemaphoreGive(sdMutex);
  }
  if (t.errorFlags)
    dbg("[I2C] L432K error flags: 0x%02X", t.errorFlags);
}

// ═══════════════════════════════════════════════════════════════════
//  UDP JOYSTICK PACKET (18-byte little-endian: uint16 + 4×float32)
// ═══════════════════════════════════════════════════════════════════
static bool prev7=false, prev11=false, prev12=false;

static void processJoystick(const uint8_t* buf, size_t len) {
  if (len < 18) { dbg("[UDP] Short packet: %u bytes", len); return; }

  uint16_t btns; float roll, pitch, yaw, thr;
  memcpy(&btns,  buf,      2);
  memcpy(&roll,  buf + 2,  4);
  memcpy(&pitch, buf + 6,  4);
  memcpy(&yaw,   buf + 10, 4);
  memcpy(&thr,   buf + 14, 4);

  bool b1  = (btns >>  0) & 1;   // hold to enable output
  bool b7  = (btns >>  6) & 1;   // cycle L432KC mode
  bool b11 = (btns >> 10) & 1;   // toggle follow-me
  bool b12 = (btns >> 11) & 1;   // toggle stop

  // ── DAC speed & turning ───────────────────────────────────────────
  // speed   = pitch × (−throttle)    [main × scaler]
  // turning = roll  ×  yaw           [main × scaler]
  if (b1) {
    float spd = pitch * (-thr);
    float trn = roll  *  yaw;
    dacWrite(PIN_JS_SPEED, (uint8_t)constrain((spd + 1.0f) * 127.5f, 0, 255));
    dacWrite(PIN_JS_TURN,  (uint8_t)constrain((trn + 1.0f) * 127.5f, 0, 255));
  } else {
    dacWrite(PIN_JS_SPEED, 128);   // neutral
    dacWrite(PIN_JS_TURN,  128);
  }

  // ── Button 7: cycle mode (rising edge → 50 ms pulse) ─────────────
  if (b7 && !prev7) {
    dbg("[JS] B7: cycle L432K mode");
    digitalWrite(PIN_JS_CYCLE, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
    digitalWrite(PIN_JS_CYCLE, LOW);
  }
  prev7 = b7;

  // ── Button 11: toggle follow-me ───────────────────────────────────
  if (b11 && !prev11) {
    bool st;
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      SD.followMeOn = !SD.followMeOn; st=SD.followMeOn; xSemaphoreGive(sdMutex);
    }
    digitalWrite(PIN_FOLLOW_ME, st ? HIGH : LOW);
    dbg("[JS] B11: Follow-Me %s", st?"ON":"OFF");
  }
  prev11 = b11;

  // ── Button 12: toggle stop ────────────────────────────────────────
  if (b12 && !prev12) {
    bool st;
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      SD.stopOn = !SD.stopOn; st=SD.stopOn; xSemaphoreGive(sdMutex);
    }
    digitalWrite(PIN_STOP, st ? HIGH : LOW);
    dbg("[JS] B12: Stop %s", st?"ON":"OFF");
  }
  prev12 = b12;

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    SD.jsBtns=btns; SD.pitch=pitch; SD.roll=roll;
    SD.yaw=yaw; SD.thr=thr; SD.jsEnabled=b1;
    xSemaphoreGive(sdMutex);
  }
}

// ═══════════════════════════════════════════════════════════════════
//  WEBSOCKET JSON PUSH
// ═══════════════════════════════════════════════════════════════════
static void pushWS() {
  if (wsServer.count() == 0) return;

  SharedData snap;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(25)) != pdTRUE) return;
  snap = SD;
  xSemaphoreGive(sdMutex);

  static const char* STATES[] = {
    "FOLLOWING","TURNING90","SEARCHING",
    "TRAFFICSTOP","OBSTACLEAVOID","MANUAL"
  };
  static const char* TLS[] = {"UNKNOWN","GREEN","AMBER","RED"};
  uint8_t rs = snap.mcu.roverState;
  uint8_t tl = snap.mcu.trafficLight;

  // Build JSON
  String j;
  j.reserve(900);
  j  = "{\"batt\":"  + String(snap.battV,2)
    + ",\"soc\":"    + String(snap.battSOC,1)
    + ",\"iL\":"     + String(snap.iL,3)
    + ",\"iR\":"     + String(snap.iR,3)
    + ",\"tqL\":"    + String(snap.tqL,4)
    + ",\"tqR\":"    + String(snap.tqR,4)
    + ",\"rpmL\":"   + String(snap.rpmL,1)
    + ",\"rpmR\":"   + String(snap.rpmR,1)
    + ",\"state\":\"" + String(rs<6?STATES[rs]:"?") + "\""
    + ",\"tl\":\""   + String(tl<4?TLS[tl]:"?") + "\""
    + ",\"err\":"    + String(snap.mcu.errorFlags)
    + ",\"line\":"   + String(snap.mcu.lineSensors)
    + ",\"obs\":"    + (snap.obstacleDetected?"true":"false")
    + ",\"jsEn\":"   + (snap.jsEnabled?"true":"false")
    + ",\"btns\":"   + String(snap.jsBtns)
    + ",\"pitch\":"  + String(snap.pitch,3)
    + ",\"roll\":"   + String(snap.roll,3)
    + ",\"yaw\":"    + String(snap.yaw,3)
    + ",\"thr\":"    + String(snap.thr,3)
    + ",\"follow\":" + (snap.followMeOn?"true":"false")
    + ",\"stop\":"   + (snap.stopOn?"true":"false")
    + ",\"map\":[";
  for (int i=0; i<N_SW; i++) {
    if (i) j += ",";
    j += "[" + String(snap.map[i].angle,1) + "," + String(snap.map[i].dist,1) + "]";
  }
  j += "],\"dbg\":\"";
  for (const char* p=snap.dbgBuf; *p; p++) {
    if      (*p=='\n') j += "\\n";
    else if (*p=='"')  j += "\\\"";
    else if (*p=='\\') j += "\\\\";
    else               j += *p;
  }
  j += "\"}";

  wsServer.textAll(j);
}

// ═══════════════════════════════════════════════════════════════════
//  DASHBOARD HTML — industrial-terminal aesthetic (stored in flash)
// ═══════════════════════════════════════════════════════════════════
static const char HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>TDP EE3 Team 7 — Rover</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@600;900&display=swap" rel="stylesheet">
<style>
:root{
  --bg:#070d14;--panel:#0b1520;--border:#1a3a5c;--accent:#00e5ff;
  --red:#ff3a3a;--amber:#ffb300;--green:#00e676;--dim:#3a5a7a;
  --text:#b0cce0;--mono:'Share Tech Mono',monospace;--head:'Orbitron',sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:var(--mono);font-size:11px;height:100vh;overflow:hidden;display:flex;flex-direction:column}
/* scanline overlay */
body::after{content:'';position:fixed;inset:0;background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,0,0,.15) 2px,rgba(0,0,0,.15) 4px);pointer-events:none;z-index:999}

/* ── HEADER ── */
.hdr{display:grid;grid-template-columns:1fr auto 1fr;align-items:center;padding:6px 12px;background:#060c13;border-bottom:1px solid var(--border);gap:8px;flex-shrink:0}
.hdr-l h1{font-family:var(--head);font-size:13px;color:var(--accent);letter-spacing:3px;text-shadow:0 0 12px var(--accent)}
.hdr-l .sub{font-size:9px;color:var(--dim);letter-spacing:2px;margin-top:1px}
.hdr-c{display:flex;align-items:center;gap:10px}
.tl{display:flex;gap:4px}
.tl-d{width:14px;height:14px;border-radius:50%;background:#111;border:1px solid #333;transition:all .3s}
.tl-r.on{background:var(--red);box-shadow:0 0 10px var(--red)}
.tl-a.on{background:var(--amber);box-shadow:0 0 10px var(--amber)}
.tl-g.on{background:var(--green);box-shadow:0 0 10px var(--green)}
.conn{font-size:9px;padding:2px 6px;border-radius:2px;border:1px solid currentColor}
.conn.ok{color:var(--green)} .conn.bad{color:var(--red)}
.state-badge{font-family:var(--head);font-size:10px;padding:3px 8px;border:1px solid var(--accent);color:var(--accent);border-radius:2px;letter-spacing:1px}
.hdr-r{text-align:right}
.batt-v{font-family:var(--head);font-size:18px;color:var(--green)}
.batt-sub{font-size:9px;color:var(--dim)}

/* ── GRID ── */
.grid{display:grid;grid-template-columns:190px 1fr 210px;grid-template-rows:auto 1fr 70px;gap:6px;padding:6px;flex:1;min-height:0}
.panel{background:var(--panel);border:1px solid var(--border);padding:7px;overflow:hidden;position:relative}
.panel::before{content:'';position:absolute;top:0;left:0;right:0;height:1px;background:linear-gradient(90deg,transparent,var(--accent),transparent);opacity:.4}
.pt{font-family:var(--head);font-size:9px;color:var(--accent);letter-spacing:2px;margin-bottom:5px;padding-bottom:3px;border-bottom:1px solid var(--border)}

/* ── ROVER STATUS ── */
.mode-row{display:flex;align-items:center;gap:6px;margin-bottom:6px}
.mode-val{font-family:var(--head);font-size:10px;color:var(--amber);letter-spacing:1px}
.rover-wrap{display:flex;justify-content:center;margin:4px 0}
.rover-svg{width:90px;height:110px}
.motor-grid{display:grid;grid-template-columns:1fr 1fr;gap:3px;margin-top:4px}
.mg-cell{background:#0a1825;padding:3px;border:1px solid var(--border)}
.mg-label{font-size:8px;color:var(--dim);display:block}
.mg-val{color:var(--green);font-size:11px}
.obs-badge{margin-top:5px;text-align:center;font-family:var(--head);font-size:9px;padding:2px;border-radius:2px}
.obs-on{background:#3a0808;color:var(--red);border:1px solid var(--red);animation:blink .6s infinite}
.obs-off{background:#082a10;color:var(--green);border:1px solid var(--green)}
@keyframes blink{50%{opacity:.4}}

/* ── SPEEDOMETER ── */
canvas.speedo{width:100%;display:block}

/* ── SONAR ── */
#sonarCanvas{width:100%;height:100%;display:block}
.sonar-panel{grid-row:1/3;grid-column:2}

/* ── LINE SENSORS ── */
.line-panel{grid-row:3;grid-column:2}
.line-row{display:flex;gap:8px;align-items:center;justify-content:center;height:30px}
.sen-wrap{text-align:center}
.sen-dot{width:18px;height:18px;border-radius:50%;background:#0a1020;border:1px solid var(--dim);transition:all .1s;margin:0 auto}
.sen-dot.act{background:var(--red);box-shadow:0 0 6px var(--red);border-color:var(--red)}
.sen-lbl{font-size:8px;color:var(--dim);margin-top:1px}
.btn-row{display:flex;gap:4px;margin-top:4px;justify-content:center}
.db{padding:2px 7px;border-radius:2px;font-family:var(--mono);font-size:9px;cursor:pointer;border:1px solid;background:transparent;letter-spacing:1px}
.db-cyan{border-color:var(--accent);color:var(--accent)} .db-cyan:hover{background:var(--accent);color:#000}
.db-grn{border-color:var(--green);color:var(--green)} .db-grn:hover{background:var(--green);color:#000}
.db-red{border-color:var(--red);color:var(--red)} .db-red:hover{background:var(--red);color:#000}
.db-dim{border-color:var(--dim);color:var(--dim)} .db-dim:hover{background:var(--dim);color:#000}

/* ── CONTROLLER ── */
.ctrl-panel{grid-row:1;grid-column:3}
.axis-lbl{font-size:9px;color:var(--dim);margin-top:3px}
.axis-track{height:8px;background:#0a1020;border:1px solid var(--border);border-radius:1px;position:relative;margin-bottom:1px}
.axis-fill{position:absolute;top:0;height:100%;background:var(--accent);opacity:.7;transition:all .04s}
.axis-mid{position:absolute;left:50%;top:0;width:1px;height:100%;background:var(--dim)}
.btn-grid2{display:grid;grid-template-columns:repeat(6,1fr);gap:2px;margin-top:4px}
.bi{height:13px;background:#0a1020;border:1px solid var(--border);font-size:8px;text-align:center;line-height:13px;border-radius:1px;color:var(--dim)}
.bi.on{background:var(--accent);color:#000;border-color:var(--accent)}
.tog-row{display:flex;gap:4px;margin-top:4px;flex-wrap:wrap}
.tog{font-size:8px;padding:1px 5px;border-radius:2px;border:1px solid}
.tog-g{border-color:var(--green);color:var(--green)} .tog-r{border-color:var(--red);color:var(--red)}
.js-en{font-family:var(--head);font-size:8px;margin-bottom:3px}

/* ── MOTOR DETAIL ── */
.mtr-panel{grid-row:2;grid-column:3}
.mtr-row{display:flex;justify-content:space-between;margin:2px 0;font-size:10px}
.mval{color:var(--green)}
.err-row{margin-top:5px;font-size:9px;color:var(--amber)}

/* ── DEBUG ── */
.dbg-panel{grid-row:3;grid-column:1/3}
#dbgLog{height:42px;overflow-y:auto;background:#050b10;padding:3px 5px;font-size:9px;color:#4a9;line-height:1.4;white-space:pre-wrap;word-break:break-all;border:1px solid var(--border)}
</style></head><body>

<!-- HEADER -->
<div class="hdr">
  <div class="hdr-l">
    <h1>TDP EE3 · TEAM 7</h1>
    <div class="sub">ROVER MASTER CONTROLLER v1.0</div>
  </div>
  <div class="hdr-c">
    <div class="tl">
      <div id="tlR" class="tl-d tl-r"></div>
      <div id="tlA" class="tl-d tl-a"></div>
      <div id="tlG" class="tl-d tl-g"></div>
    </div>
    <div id="connBadge" class="conn bad">OFFLINE</div>
    <div id="stateBadge" class="state-badge">--</div>
  </div>
  <div class="hdr-r">
    <div class="batt-v"><span id="bV">--</span>V &nbsp;<span id="bSOC">--</span>%</div>
    <div class="batt-sub">10-CELL NiMH SOC</div>
  </div>
</div>

<!-- GRID -->
<div class="grid">

  <!-- Rover status -->
  <div class="panel" style="grid-row:1;grid-column:1">
    <div class="pt">ROVER STATUS</div>
    <div class="mode-row">Mode: <span id="modeVal" class="mode-val">--</span></div>
    <div class="rover-wrap">
      <svg class="rover-svg" viewBox="0 0 90 110">
        <!-- Body -->
        <rect x="15" y="15" width="60" height="80" rx="4" fill="#0d1e30" stroke="#1a3a5c" stroke-width="1.5"/>
        <!-- Wheels -->
        <rect x="2" y="12" width="13" height="22" rx="3" fill="#1a2a3a" stroke="#3a5a7a" stroke-width="1"/>
        <rect x="75" y="12" width="13" height="22" rx="3" fill="#1a2a3a" stroke="#3a5a7a" stroke-width="1"/>
        <rect x="2" y="76" width="13" height="22" rx="3" fill="#1a2a3a" stroke="#3a5a7a" stroke-width="1"/>
        <rect x="75" y="76" width="13" height="22" rx="3" fill="#1a2a3a" stroke="#3a5a7a" stroke-width="1"/>
        <!-- Sensor turret -->
        <circle cx="45" cy="55" r="12" fill="#081525" stroke="#00e5ff" stroke-width="1.5"/>
        <line id="srvNeedle" x1="45" y1="55" x2="45" y2="43" stroke="#00e5ff" stroke-width="1.5" stroke-linecap="round"/>
        <!-- Forward arrow -->
        <polygon points="45,18 40,26 50,26" fill="#00e676" opacity=".8"/>
        <!-- RPM labels -->
        <text x="8" y="10" fill="#4a9" font-size="7" font-family="monospace" text-anchor="middle" id="rpmFL">--</text>
        <text x="82" y="10" fill="#4a9" font-size="7" font-family="monospace" text-anchor="middle" id="rpmFR">--</text>
        <text x="8" y="104" fill="#4a9" font-size="7" font-family="monospace" text-anchor="middle" id="rpmBL">--</text>
        <text x="82" y="104" fill="#4a9" font-size="7" font-family="monospace" text-anchor="middle" id="rpmBR">--</text>
      </svg>
    </div>
    <div class="motor-grid">
      <div class="mg-cell"><span class="mg-label">I LEFT</span><span id="iL" class="mg-val">--A</span></div>
      <div class="mg-cell"><span class="mg-label">I RIGHT</span><span id="iR" class="mg-val">--A</span></div>
      <div class="mg-cell"><span class="mg-label">τ LEFT</span><span id="tqL" class="mg-val">--Nm</span></div>
      <div class="mg-cell"><span class="mg-label">τ RIGHT</span><span id="tqR" class="mg-val">--Nm</span></div>
    </div>
    <div id="obsBadge" class="obs-badge obs-off">CLEAR</div>
  </div>

  <!-- Speedometer -->
  <div class="panel" style="grid-row:2;grid-column:1">
    <div class="pt">SPEED (m/s)</div>
    <canvas id="speedoC" width="175" height="85" class="speedo"></canvas>
  </div>

  <!-- Sonar radar -->
  <div class="panel sonar-panel">
    <div class="pt">SONAR RETURN — 2-D MAP</div>
    <canvas id="sonarC"></canvas>
  </div>

  <!-- Line sensors + buttons -->
  <div class="panel line-panel">
    <div class="pt">LINE SENSORS</div>
    <div class="line-row">
      <div class="sen-wrap"><div class="sen-dot" id="sFL"></div><div class="sen-lbl">FL</div></div>
      <div class="sen-wrap"><div class="sen-dot" id="sLC"></div><div class="sen-lbl">LC</div></div>
      <div class="sen-wrap"><div class="sen-dot" id="sLR"></div><div class="sen-lbl">LR</div></div>
      <div class="sen-wrap"><div class="sen-dot" id="sFR"></div><div class="sen-lbl">FR</div></div>
    </div>
    <div class="btn-row">
      <button class="db db-cyan" onclick="cmd('lights')">LIGHTS</button>
      <button class="db db-grn" onclick="cmd('follow')">FOLLOW</button>
      <button class="db db-red" onclick="cmd('stop')">STOP</button>
      <button class="db db-dim" onclick="cmd('off')">OFF</button>
    </div>
  </div>

  <!-- Controller monitor -->
  <div class="panel ctrl-panel">
    <div class="pt">CONTROLLER MONITOR</div>
    <div class="js-en">JS OUT: <span id="jsEnBadge">DISABLED</span></div>
    <div class="axis-lbl">PITCH (speed)</div>
    <div class="axis-track"><div class="axis-mid"></div><div id="aPitch" class="axis-fill"></div></div>
    <div class="axis-lbl">ROLL (turn)</div>
    <div class="axis-track"><div class="axis-mid"></div><div id="aRoll" class="axis-fill"></div></div>
    <div class="axis-lbl">YAW (turn scale)</div>
    <div class="axis-track"><div class="axis-mid"></div><div id="aYaw" class="axis-fill"></div></div>
    <div class="axis-lbl">THROTTLE (speed scale)</div>
    <div class="axis-track"><div class="axis-mid"></div><div id="aThr" class="axis-fill" style="background:#ffb300"></div></div>
    <div class="axis-lbl">OUTPUT VOLTS  L:<span id="outVL">--</span>V  R:<span id="outVR">--</span>V</div>
    <div class="btn-grid2" id="btnGrid"></div>
    <div class="tog-row">
      <span class="tog tog-r" id="followTog">FOLLOW:OFF</span>
      <span class="tog tog-r" id="stopTog">STOP:OFF</span>
    </div>
  </div>

  <!-- Motor detail -->
  <div class="panel mtr-panel">
    <div class="pt">MOTOR TELEMETRY</div>
    <div class="mtr-row"><span>L Current</span><span id="mIL" class="mval">--</span></div>
    <div class="mtr-row"><span>R Current</span><span id="mIR" class="mval">--</span></div>
    <div class="mtr-row"><span>L Torque</span><span id="mTQL" class="mval">--</span></div>
    <div class="mtr-row"><span>R Torque</span><span id="mTQR" class="mval">--</span></div>
    <div class="mtr-row"><span>L RPM</span><span id="mRPML" class="mval">--</span></div>
    <div class="mtr-row"><span>R RPM</span><span id="mRPMR" class="mval">--</span></div>
    <div class="err-row">ERR: <span id="errFlags">0x00</span></div>
  </div>

  <!-- Debug -->
  <div class="panel dbg-panel">
    <div class="pt">UART DEBUG — L432KC / ESP32</div>
    <div id="dbgLog"></div>
  </div>

</div>

<script>
// ── Button grid ──────────────────────────────────────────────────
const bg=document.getElementById('btnGrid');
for(let i=1;i<=12;i++){const d=document.createElement('div');d.className='bi';d.id='b'+i;d.textContent=i;bg.appendChild(d);}

// ── WebSocket ────────────────────────────────────────────────────
let ws, prevDbg='', needleAngle=0, lastMapData=null;

function connect(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.onopen=()=>{document.getElementById('connBadge').textContent='ONLINE';document.getElementById('connBadge').className='conn ok';}
  ws.onclose=()=>{document.getElementById('connBadge').textContent='OFFLINE';document.getElementById('connBadge').className='conn bad';setTimeout(connect,2000);}
  ws.onmessage=e=>update(JSON.parse(e.data));
}
connect();

function cmd(c){if(ws&&ws.readyState===1)ws.send(JSON.stringify({cmd:c}));}

// ── Axis bar helper ──────────────────────────────────────────────
function setAxis(id,v){
  const el=document.getElementById(id);
  const c=50,p=(v+1)*50;
  const l=Math.min(p,c),w=Math.abs(p-c);
  el.style.left=l+'%';el.style.width=w+'%';
}

// ── Update dashboard ─────────────────────────────────────────────
function update(d){
  // Battery
  const socC=d.soc>50?'#00e676':d.soc>20?'#ffb300':'#ff3a3a';
  document.getElementById('bV').textContent=d.batt.toFixed(2);
  document.getElementById('bSOC').style.color=socC;
  document.getElementById('bSOC').textContent=d.soc.toFixed(1);

  // State
  const STATE_COL={FOLLOWING:'#00e676',TURNING90:'#00e5ff',SEARCHING:'#ffb300',TRAFFICSTOP:'#ff3a3a',OBSTACLEAVOID:'#ffb300',MANUAL:'#00e5ff'};
  document.getElementById('modeVal').textContent=d.state;
  document.getElementById('modeVal').style.color=STATE_COL[d.state]||'#ccc';
  document.getElementById('stateBadge').textContent=d.state;

  // Traffic light
  document.getElementById('tlR').className='tl-d tl-r'+(d.tl==='RED'?' on':'');
  document.getElementById('tlA').className='tl-d tl-a'+(d.tl==='AMBER'?' on':'');
  document.getElementById('tlG').className='tl-d tl-g'+(d.tl==='GREEN'?' on':'');

  // Obstacle
  const ob=document.getElementById('obsBadge');
  ob.className='obs-badge '+(d.obs?'obs-on':'obs-off');
  ob.textContent=d.obs?'⚠ OBSTACLE':'CLEAR';

  // Motor
  ['iL','iR','tqL','tqR','rpmL','rpmR'].forEach(k=>{
    const v=d[k]; const el=document.getElementById(k);
    if(el) el.textContent=k.startsWith('rpm')?v.toFixed(0)+'rpm':k.startsWith('tq')?v.toFixed(4)+'Nm':v.toFixed(3)+'A';
  });
  document.getElementById('mIL').textContent=d.iL.toFixed(3)+'A';
  document.getElementById('mIR').textContent=d.iR.toFixed(3)+'A';
  document.getElementById('mTQL').textContent=d.tqL.toFixed(4)+'Nm';
  document.getElementById('mTQR').textContent=d.tqR.toFixed(4)+'Nm';
  document.getElementById('mRPML').textContent=d.rpmL.toFixed(1);
  document.getElementById('mRPMR').textContent=d.rpmR.toFixed(1);
  document.getElementById('errFlags').textContent='0x'+d.err.toString(16).padStart(2,'0').toUpperCase();

  // RPM on rover SVG wheels
  ['rpmFL','rpmFR','rpmBL','rpmBR'].forEach(id=>{
    const el=document.getElementById(id); if(el) el.textContent=d.rpmL.toFixed(0);
  });
  document.getElementById('rpmFR').textContent=d.rpmR.toFixed(0);
  document.getElementById('rpmBR').textContent=d.rpmR.toFixed(0);

  // Rotate sonar needle SVG (spin continuously, just for aesthetics)
  needleAngle=(needleAngle+3.6)%360;
  const r=needleAngle*Math.PI/180;
  const cx=45,cy=55,l=12;
  document.getElementById('srvNeedle').setAttribute('x2', cx+l*Math.sin(r));
  document.getElementById('srvNeedle').setAttribute('y2', cy-l*Math.cos(r));

  // Line sensors
  const ls=d.line;
  [{id:'sFL',b:3},{id:'sLC',b:2},{id:'sLR',b:1},{id:'sFR',b:0}].forEach(s=>{
    document.getElementById(s.id).className='sen-dot'+((ls>>s.b&1)?' act':'');
  });

  // Joystick axes
  const en=d.jsEn;
  const jsb=document.getElementById('jsEnBadge');
  jsb.textContent=en?'ENABLED':'DISABLED'; jsb.style.color=en?'#00e676':'#ff3a3a';
  setAxis('aPitch',d.pitch); setAxis('aRoll',d.roll);
  setAxis('aYaw',d.yaw);   setAxis('aThr',d.thr);

  // Output volts (DAC 0-3.3V, 128=1.65V neutral)
  const speedDac=(d.pitch*(-d.thr)+1)*127.5;
  const turnDac=(d.roll*d.yaw+1)*127.5;
  document.getElementById('outVL').textContent=((speedDac/255)*3.3).toFixed(2);
  document.getElementById('outVR').textContent=((turnDac/255)*3.3).toFixed(2);

  // Button indicators
  for(let i=0;i<12;i++){
    const el=document.getElementById('b'+(i+1));
    if(el) el.className='bi'+((d.btns>>i&1)?' on':'');
  }

  // Toggle badges
  const ft=document.getElementById('followTog');
  ft.textContent='FOLLOW:'+(d.follow?'ON':'OFF');
  ft.className='tog '+(d.follow?'tog-g':'tog-r');
  const st=document.getElementById('stopTog');
  st.textContent='STOP:'+(d.stop?'ON':'OFF');
  st.className='tog '+(d.stop?'tog-r':'tog-g');

  // Sonar
  lastMapData=d.map; drawSonar();

  // Speed
  const spd=Math.max(d.rpmL,d.rpmR)/200*2.0;
  drawSpeedo(spd);

  // Debug log
  if(d.dbg&&d.dbg!==prevDbg){
    const log=document.getElementById('dbgLog');
    const lines=d.dbg.split('\\n').filter(s=>s.trim());
    log.textContent=lines.join('\n');
    log.scrollTop=log.scrollHeight;
    prevDbg=d.dbg;
  }
}

// ── Sonar radar ──────────────────────────────────────────────────
function drawSonar(){
  const c=document.getElementById('sonarC');
  if(!c) return;
  const p=c.parentElement;
  c.width=p.clientWidth-14; c.height=p.clientHeight-28;
  const ctx=c.getContext('2d');
  const W=c.width,H=c.height,cx=W/2,cy=H/2;
  const R=Math.min(W,H)*0.45, maxD=2500;
  const sc=R/maxD;

  ctx.fillStyle='#020810'; ctx.fillRect(0,0,W,H);

  // Grid rings
  [500,1000,1500,2000,2500].forEach((d,i)=>{
    const r=d*sc;
    ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2);
    ctx.strokeStyle=`rgba(0,60,100,${0.3+i*0.05})`; ctx.lineWidth=1; ctx.stroke();
    ctx.fillStyle='#1a4060'; ctx.font='8px Share Tech Mono';
    ctx.fillText((d/1000).toFixed(1)+'m', cx+r+2, cy-2);
  });
  // Crosshairs
  ctx.strokeStyle='rgba(0,80,120,.4)'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,H); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0,cy); ctx.lineTo(W,cy); ctx.stroke();
  // Angle spokes
  for(let a=0;a<360;a+=30){
    const r=a*Math.PI/180;
    ctx.beginPath(); ctx.moveTo(cx,cy);
    ctx.lineTo(cx+R*Math.sin(r),cy-R*Math.cos(r));
    ctx.strokeStyle='rgba(0,80,120,.2)'; ctx.stroke();
  }

  // Rover body (to scale: 200×160 mm estimate)
  const bw=160*sc, bh=200*sc;
  ctx.strokeStyle='#e94560'; ctx.lineWidth=1.5;
  ctx.strokeRect(cx-bw/2,cy-bh/2,bw,bh);
  ctx.fillStyle='rgba(233,69,96,.08)'; ctx.fillRect(cx-bw/2,cy-bh/2,bw,bh);

  // Points
  if(lastMapData){
    lastMapData.forEach(pt=>{
      const [a,dist]=pt;
      if(dist<=0||dist>maxD) return;
      const rad=(a-90)*Math.PI/180; // 0°=up=forward
      const x=cx+dist*sc*Math.cos(rad);
      const y=cy+dist*sc*Math.sin(rad);
      const ratio=dist/maxD;
      const gr=Math.round(230*(1-ratio)), gg=Math.round(230*ratio);
      ctx.beginPath(); ctx.arc(x,y,3.5,0,Math.PI*2);
      ctx.fillStyle=`rgb(${gr},${gg},80)`; ctx.fill();
      ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(x,y);
      ctx.strokeStyle=`rgba(${gr},${gg},80,0.15)`; ctx.lineWidth=1; ctx.stroke();
    });
  }

  // Labels
  ctx.fillStyle='#00e676'; ctx.font='bold 9px Share Tech Mono'; ctx.textAlign='center';
  ctx.fillText('FWD',cx,12);
  ctx.fillText('BCK',cx,H-3);
  ctx.textAlign='left';
}

// ── Speedometer ───────────────────────────────────────────────────
function drawSpeedo(spd){
  const c=document.getElementById('speedoC');
  const ctx=c.getContext('2d');
  const W=c.width,H=c.height;
  ctx.clearRect(0,0,W,H);
  const cx=W/2, cy=H*0.9, R=H*0.8;
  const startA=Math.PI, endA=0, maxSpd=2.0;
  const curA=startA+(Math.min(spd,maxSpd)/maxSpd)*Math.PI;

  // BG arc
  ctx.beginPath(); ctx.arc(cx,cy,R,startA,endA);
  ctx.strokeStyle='#0a1825'; ctx.lineWidth=10; ctx.stroke();

  // Speed arc
  const col=spd>1.5?'#ff3a3a':spd>0.8?'#ffb300':'#00e676';
  ctx.beginPath(); ctx.arc(cx,cy,R,startA,curA);
  ctx.strokeStyle=col; ctx.lineWidth=10; ctx.stroke();

  // Needle
  ctx.beginPath(); ctx.moveTo(cx,cy);
  ctx.lineTo(cx+(R-6)*Math.cos(curA),cy+(R-6)*Math.sin(curA));
  ctx.strokeStyle='#e94560'; ctx.lineWidth=2; ctx.stroke();

  // Labels
  ctx.fillStyle='#3a6a8a'; ctx.font='9px Share Tech Mono';
  ctx.fillText('0',cx-R-4,cy+4);
  ctx.fillText('2',cx+R-4,cy+4);
  ctx.fillText('1',cx-4,cy-R+10);

  // Value
  ctx.fillStyle=col; ctx.font='bold 13px Orbitron';
  ctx.textAlign='center';
  ctx.fillText(spd.toFixed(2)+' m/s',cx,cy-12);
  ctx.textAlign='left';
}

// Resize sonar on window resize
window.addEventListener('resize',drawSonar);
</script></body></html>
)HTML";

// ═══════════════════════════════════════════════════════════════════
//  WEBSOCKET EVENT HANDLER
// ═══════════════════════════════════════════════════════════════════
static void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* cli,
                      AwsEventType t, void* arg, uint8_t* data, size_t len) {
  if (t == WS_EVT_CONNECT) {
    dbg("[WS] Client #%u connected", cli->id());
  } else if (t == WS_EVT_DISCONNECT) {
    dbg("[WS] Client #%u disconnected", cli->id());
  } else if (t == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
      String msg((char*)data, len);
      dbg("[WS] cmd: %s", msg.c_str());
      // Simple command dispatch
      if (msg.indexOf("follow") >= 0) {
        bool st;
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          SD.followMeOn = !SD.followMeOn; st=SD.followMeOn; xSemaphoreGive(sdMutex);
        }
        digitalWrite(PIN_FOLLOW_ME, st ? HIGH : LOW);
      } else if (msg.indexOf("stop") >= 0) {
        bool st;
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          SD.stopOn = !SD.stopOn; st=SD.stopOn; xSemaphoreGive(sdMutex);
        }
        digitalWrite(PIN_STOP, st ? HIGH : LOW);
      } else if (msg.indexOf("off") >= 0) {
        dbg("[WS] OFF command received");
        // Application-level E-stop: pull STOP line high
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          SD.stopOn = true; xSemaphoreGive(sdMutex);
        }
        digitalWrite(PIN_STOP, HIGH);
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//  TASK 2 — WiFi AP + Dashboard + Comms  (pinned to Core 1)
// ═══════════════════════════════════════════════════════════════════
static void task2(void*) {
  dbg("[T2] Core %d: WiFi/dashboard task starting", xPortGetCoreID());

  // ── WiFi Access Point ────────────────────────────────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  dbg("[WiFi] AP: '%s'  IP: 192.168.4.1", AP_SSID);

  // ── WebSocket ────────────────────────────────────────────────────
  wsServer.onEvent(onWsEvent);
  httpServer.addHandler(&wsServer);

  // ── HTTP — serve dashboard ────────────────────────────────────────
  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", HTML);
  });
  httpServer.begin();
  dbg("[HTTP] Server started on port 80");

  // ── UDP ───────────────────────────────────────────────────────────
  udpSock.begin(UDP_PORT);
  dbg("[UDP] Joystick RX on port %d", UDP_PORT);

  // ── I²C (slave — L432KC is master, pushes data on change) ────────
  Wire.begin((uint8_t)KL_ADDR, PIN_SDA, PIN_SCL);
  Wire.onReceive(onReceiveFromL432K);
  dbg("[I2C] Slave addr=0x%02X on SDA=%d SCL=%d", KL_ADDR, PIN_SDA, PIN_SCL);

  uint32_t tTelem=0, tWS=0;
  uint8_t  udpBuf[20];

  for (;;) {
    uint32_t now = millis();

    // ── UDP joystick packet ─────────────────────────────────────────
    int pkt = udpSock.parsePacket();
    if (pkt >= 18) {
      int got = udpSock.read(udpBuf, sizeof(udpBuf));
      processJoystick(udpBuf, got);
    }

    // ── UART debug from L432KC ──────────────────────────────────────
    while (Serial.available()) {
      String ln = Serial.readStringUntil('\n');
      ln.trim();
      if (ln.length() > 0) dbg("[L432K] %s", ln.c_str());
    }

    // (I²C updates arrive asynchronously via onReceiveFromL432K callback)

    // ── ADC telemetry every 200 ms ──────────────────────────────────
    if (now - tTelem > 200) { readTelemetry(); tTelem=now; }

    // ── WebSocket push every 100 ms ─────────────────────────────────
    if (now - tWS > 100) {
      wsServer.cleanupClients();
      pushWS();
      tWS=now;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  // L432KC UART (115200 8N1)
  Serial.begin(115200);

  // ── GPIO init ────────────────────────────────────────────────────
  pinMode(PIN_TRIG,        OUTPUT);   digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,        INPUT);
  pinMode(PIN_ENC_LATCH,   OUTPUT);   digitalWrite(PIN_ENC_LATCH, HIGH);
  pinMode(PIN_ENC_CLK,     OUTPUT);   digitalWrite(PIN_ENC_CLK,  LOW);
  pinMode(PIN_ENC_DATA,    INPUT);    // input-only GPIO36
  pinMode(PIN_MEAS_INT,    INPUT_PULLUP);
  pinMode(PIN_OBS_ACTIVE,  OUTPUT);   digitalWrite(PIN_OBS_ACTIVE, LOW);
  pinMode(PIN_OBS_CA1,     OUTPUT);   digitalWrite(PIN_OBS_CA1,   LOW);
  pinMode(PIN_OBS_CA2,     OUTPUT);   digitalWrite(PIN_OBS_CA2,   LOW);
  pinMode(PIN_JS_CYCLE,    OUTPUT);   digitalWrite(PIN_JS_CYCLE,  LOW);
  pinMode(PIN_FOLLOW_ME,   OUTPUT);   digitalWrite(PIN_FOLLOW_ME, LOW);
  // PIN_STOP (GPIO0): after boot, safe to use; default HIGH avoids download mode
  pinMode(PIN_STOP,        OUTPUT);   digitalWrite(PIN_STOP, HIGH);
  pinMode(PIN_BUZZER,      OUTPUT);   digitalWrite(PIN_BUZZER, LOW);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // 0–3.3 V full scale

  // DAC defaults (neutral = 128 ≈ 1.65 V)
  dacWrite(PIN_JS_SPEED, 128);
  dacWrite(PIN_JS_TURN,  128);

  // Mutex + shared data
  sdMutex = xSemaphoreCreateMutex();
  memset(&SD, 0, sizeof(SD));
  snprintf(SD.dbgBuf, sizeof(SD.dbgBuf), "[ESP32] Boot OK\n");

  // Startup beep
  digitalWrite(PIN_BUZZER, HIGH); delay(80); digitalWrite(PIN_BUZZER, LOW);

  // Launch tasks on separate cores
  xTaskCreatePinnedToCore(task1, "Task1_Obs", 8192,  NULL, 2, &hTask1, 0);
  xTaskCreatePinnedToCore(task2, "Task2_Wifi",16384, NULL, 1, &hTask2, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }   // everything runs in RTOS tasks
