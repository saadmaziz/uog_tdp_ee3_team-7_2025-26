/*
 * ESP32 Joystick WiFi Receiver  v5  --  ACCESS POINT MODE  (3-pin output)
 * ============================================================
 * The ESP32 broadcasts its OWN WiFi network.
 * Connect your Windows laptop to that network, then run
 * joystick_client.py.  No router needed.
 *
 *   WiFi SSID     : ESP32_Joystick
 *   WiFi Password : joystick123
 *   ESP32 IP      : 192.168.4.1  (AP default, fixed)
 *   UDP port      : 4210
 *   Dashboard     : http://192.168.4.1/
 *
 * OUTPUTS:
 *   GPIO 25  -> Aout1  TURN RATE  = roll          (8-bit DAC, 0-3.3 V, centre = 1.65 V)
 *   GPIO 26  -> Aout2  SPEED      = pitch x (-throttle) (8-bit DAC, 0-3.3 V, centre = 1.65 V)
 *   GPIO 18  -> HIGH while joystick Button 1 OR Button 7 is pressed
 *
 * Board  : ESP32 Dev Module
 * Core   : ESP32 by Espressif 3.x
 * ============================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// ---- Access Point settings --------------------------------------------------
const char*      AP_SSID     = "ESP32_Joystick";
const char*      AP_PASSWORD = "joystick123";
const IPAddress  AP_IP      (192, 168, 4, 1);
const IPAddress  AP_GATEWAY (192, 168, 4, 1);
const IPAddress  AP_SUBNET  (255, 255, 255, 0);
// -----------------------------------------------------------------------------

const uint16_t UDP_PORT    = 4210;
const uint32_t BAUD_RATE   = 115200;
const uint16_t PACKET_SIZE = 18;

// ---- Pin definitions --------------------------------------------------------
#define PIN_AOUT1   25    // DAC1 -- Turn Rate (roll)
#define PIN_AOUT2   26    // DAC2 -- Speed (pitch * -throttle)
#define PIN_BTN_OUT 18    // HIGH when Button 1 or Button 7 pressed

// Button bit positions in the 16-bit mask (0-indexed)
#define BTN1_BIT  0   // joystick button 1
#define BTN7_BIT  6   // joystick button 7

// ---- Shared state -----------------------------------------------------------
volatile float    g_roll     = 0.0f;
volatile float    g_pitch    = 0.0f;
volatile float    g_yaw      = 0.0f;
volatile float    g_throttle = 0.0f;
volatile float    g_turnRate = 0.0f;
volatile float    g_speed    = 0.0f;
volatile uint16_t g_btnMask  = 0;
volatile uint32_t g_lastPkt  = 0;

// ---- Objects ----------------------------------------------------------------
WiFiUDP   udp;
WebServer server(80);
uint8_t   packetBuf[PACKET_SIZE];

// ---- Helpers ----------------------------------------------------------------
inline uint8_t floatToDAC(float v) {
    v = constrain(v, -1.0f, 1.0f);
    return (uint8_t)((v + 1.0f) * 0.5f * 255.0f + 0.5f);
}
float readFloat(const uint8_t* b, int o) {
    float v; memcpy(&v, b + o, sizeof(float)); return v;
}
uint16_t readU16(const uint8_t* b) {
    return (uint16_t)(b[0] | (b[1] << 8));
}

// ---- Web: /data  (JSON) -----------------------------------------------------
void handleData() {
    bool live   = (millis() - g_lastPkt) < 2000;
    bool btnOut = (g_btnMask & (1 << BTN1_BIT)) || (g_btnMask & (1 << BTN7_BIT));

    String j  = "{";
    j += "\"live\":"     + String(live ? "true" : "false") + ",";
    j += "\"roll\":"     + String(g_roll,     3) + ",";
    j += "\"pitch\":"    + String(g_pitch,    3) + ",";
    j += "\"yaw\":"      + String(g_yaw,      3) + ",";
    j += "\"throttle\":" + String(g_throttle, 3) + ",";
    j += "\"turnRate\":" + String(g_turnRate, 3) + ",";
    j += "\"speed\":"    + String(g_speed,    3) + ",";
    j += "\"btnMask\":"  + String(g_btnMask)     + ",";
    j += "\"btn1\":"     + String((g_btnMask & (1 << BTN1_BIT)) ? "true" : "false") + ",";
    j += "\"btn7\":"     + String((g_btnMask & (1 << BTN7_BIT)) ? "true" : "false") + ",";
    j += "\"btnOut\":"   + String(btnOut ? "true" : "false") + ",";
    j += "\"aout1_v\":"  + String(g_turnRate * 1.65f + 1.65f, 3) + ",";
    j += "\"aout2_v\":"  + String(g_speed    * 1.65f + 1.65f, 3);
    j += "}";

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", j);
}

// ---- Web: /  (dashboard) ----------------------------------------------------
void handleRoot() {
    static const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Joystick Monitor</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--bdr:#30363d;--acc:#58a6ff;
      --grn:#3fb950;--yel:#d29922;--red:#f85149;--txt:#e6edf3;--sub:#8b949e}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',system-ui,sans-serif;padding:18px}
h1{text-align:center;font-size:1.3rem;letter-spacing:.12em;color:var(--acc);
   margin-bottom:2px;text-transform:uppercase}
#apinfo{text-align:center;font-size:.72rem;color:var(--sub);margin-bottom:4px}
#status{text-align:center;font-size:.78rem;margin-bottom:16px;color:var(--sub)}
#dot{display:inline-block;width:8px;height:8px;border-radius:50%;
     background:var(--red);margin-right:5px;vertical-align:middle;transition:background .3s}
#dot.on{background:var(--grn)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(255px,1fr));
      gap:14px;max-width:860px;margin:0 auto}
.card{background:var(--card);border:1px solid var(--bdr);border-radius:12px;padding:16px}
.card h2{font-size:.7rem;text-transform:uppercase;letter-spacing:.12em;
         color:var(--sub);margin-bottom:12px}
.aout-row{display:flex;gap:12px}
.aout-box{flex:1;background:#21262d;border-radius:9px;padding:13px;text-align:center}
.aout-box .nm{font-size:.62rem;text-transform:uppercase;letter-spacing:.1em;
              color:var(--sub);margin-bottom:3px}
.aout-box .vv{font-size:1.9rem;font-weight:700;color:var(--acc);font-variant-numeric:tabular-nums}
.aout-box .sv{font-size:.68rem;color:var(--sub);margin-top:2px}
.abar{height:7px;background:#0d1117;border-radius:4px;margin-top:7px;overflow:hidden}
.abar-f{height:100%;border-radius:4px;background:var(--acc);transition:width .09s}
.btn-row{display:flex;gap:14px;justify-content:center;margin-top:4px}
.btn-box{flex:1;max-width:140px;background:#21262d;border-radius:9px;padding:16px;
         text-align:center;border:2px solid var(--bdr);transition:all .1s}
.btn-box.on{background:var(--acc);border-color:var(--acc)}
.btn-box .bn{font-size:.65rem;text-transform:uppercase;letter-spacing:.1em;color:var(--sub)}
.btn-box.on .bn{color:#000}
.btn-box .bs{font-size:1.3rem;font-weight:700;margin-top:4px;color:var(--sub)}
.btn-box.on .bs{color:#000}
.out-box{margin-top:12px;background:#21262d;border-radius:9px;padding:14px;text-align:center;
         border:2px solid var(--bdr);transition:all .1s}
.out-box.on{background:var(--grn);border-color:var(--grn)}
.out-box .on-lbl{font-size:.65rem;text-transform:uppercase;letter-spacing:.12em;color:var(--sub)}
.out-box.on .on-lbl{color:#000}
.out-box .on-val{font-size:1.1rem;font-weight:700;color:var(--sub);margin-top:4px}
.out-box.on .on-val{color:#000}
</style></head><body>
<h1>&#9679; Joystick Monitor</h1>
<p id="apinfo">WiFi: ESP32_Joystick &nbsp;|&nbsp; IP: 192.168.4.1 &nbsp;|&nbsp; UDP port 4210</p>
<p id="status"><span id="dot"></span><span id="stxt">Waiting for joystick data...</span></p>
<div class="grid">

  <div class="card">
    <h2>Analogue Outputs (True DAC)</h2>
    <div class="aout-row">
      <div class="aout-box">
        <div class="nm">GPIO 25 &mdash; Turn Rate</div>
        <div class="vv"><span id="a1v">1.65</span> V</div>
        <div class="sv">roll = <span id="a1r">0.000</span></div>
        <div class="abar"><div class="abar-f" id="a1b" style="width:50%"></div></div>
      </div>
      <div class="aout-box">
        <div class="nm">GPIO 26 &mdash; Speed</div>
        <div class="vv"><span id="a2v">1.65</span> V</div>
        <div class="sv">pitch &times; thr = <span id="a2r">0.000</span></div>
        <div class="abar"><div class="abar-f" id="a2b" style="width:50%"></div></div>
      </div>
    </div>
  </div>

  <div class="card">
    <h2>GPIO 18 &mdash; Button Output</h2>
    <div class="btn-row">
      <div class="btn-box" id="bb1">
        <div class="bn">Button 1</div>
        <div class="bs">OFF</div>
      </div>
      <div class="btn-box" id="bb7">
        <div class="bn">Button 7</div>
        <div class="bs">OFF</div>
      </div>
    </div>
    <div class="out-box" id="io18">
      <div class="on-lbl">GPIO 18 output</div>
      <div class="on-val" id="io18v">LOW</div>
    </div>
  </div>

</div>
<script>
function pct(v) { return ((v + 1) / 2 * 100).toFixed(1) + '%'; }
function col(v) {
  var a = Math.abs(v);
  return a > 0.8 ? 'var(--red)' : a > 0.4 ? 'var(--yel)' : 'var(--grn)';
}
function poll() {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', '/data', true);
  xhr.onload = function() {
    if (xhr.status !== 200) return;
    var d = JSON.parse(xhr.responseText);
    document.getElementById('dot').className = d.live ? 'on' : '';
    document.getElementById('stxt').textContent = d.live ? 'Joystick live' : 'No data - check client';
    document.getElementById('a1v').textContent = d.aout1_v.toFixed(2);
    document.getElementById('a1r').textContent = d.turnRate.toFixed(3);
    var a1b = document.getElementById('a1b');
    a1b.style.width = pct(d.turnRate); a1b.style.background = col(d.turnRate);
    document.getElementById('a2v').textContent = d.aout2_v.toFixed(2);
    document.getElementById('a2r').textContent = d.speed.toFixed(3);
    var a2b = document.getElementById('a2b');
    a2b.style.width = pct(d.speed); a2b.style.background = col(d.speed);
    var b1 = document.getElementById('bb1');
    b1.className = 'btn-box' + (d.btn1 ? ' on' : '');
    b1.querySelector('.bs').textContent = d.btn1 ? 'ON' : 'OFF';
    var b7 = document.getElementById('bb7');
    b7.className = 'btn-box' + (d.btn7 ? ' on' : '');
    b7.querySelector('.bs').textContent = d.btn7 ? 'ON' : 'OFF';
    var io = document.getElementById('io18');
    io.className = 'out-box' + (d.btnOut ? ' on' : '');
    document.getElementById('io18v').textContent = d.btnOut ? 'HIGH' : 'LOW';
  };
  xhr.send();
}
setInterval(poll, 100);
poll();
</script>
</body></html>
)rawliteral";

    server.send_P(200, "text/html", PAGE);
}

// ---- Setup ------------------------------------------------------------------
void setup() {
    Serial.begin(BAUD_RATE);
    delay(400);
    Serial.println("\n[BOOT] ESP32 Joystick Receiver v5 -- 3-pin output");

    // GPIO 18: digital output for Button 1 / Button 7
    pinMode(PIN_BTN_OUT, OUTPUT);
    digitalWrite(PIN_BTN_OUT, LOW);

    // DAC outputs at mid-scale (1.65 V)
    dacWrite(PIN_AOUT1, 128);
    dacWrite(PIN_AOUT2, 128);

    // ---- Start as Access Point ----------------------------------------------
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    Serial.println("[WiFi] Access Point started!");
    Serial.printf ("[WiFi] SSID     : %s\n", AP_SSID);
    Serial.printf ("[WiFi] Password : %s\n", AP_PASSWORD);
    Serial.printf ("[WiFi] IP       : %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf ("[WiFi] Dashboard: http://%s/\n", WiFi.softAPIP().toString().c_str());
    Serial.printf ("[WiFi] UDP port : %u\n\n", UDP_PORT);

    // Web routes
    server.on("/",     handleRoot);
    server.on("/data", handleData);
    server.begin();
    Serial.println("[HTTP] Web server started on port 80");

    // UDP
    udp.begin(UDP_PORT);
    Serial.printf("[UDP]  Listening on port %u\n\n", UDP_PORT);

    Serial.println("==============================================");
    Serial.println(" Outputs:");
    Serial.println("   GPIO 25 -- Aout1 Turn Rate (DAC)");
    Serial.println("   GPIO 26 -- Aout2 Speed     (DAC)");
    Serial.println("   GPIO 18 -- HIGH if Btn1 or Btn7 pressed");
    Serial.println("==============================================\n");
}

// ---- Loop -------------------------------------------------------------------
void loop() {
    server.handleClient();

    int len = udp.parsePacket();
    if (len < PACKET_SIZE) return;
    udp.read(packetBuf, PACKET_SIZE);

    uint16_t btnMask = readU16(packetBuf);
    float roll       = readFloat(packetBuf,  2);
    float pitch      = readFloat(packetBuf,  6);
    float yaw        = readFloat(packetBuf, 10);
    float throttle   = readFloat(packetBuf, 14);

    // Aout1: Turn Rate = roll  (-1 to +1)
    float turnRate = roll;

    // Aout2: Speed = pitch * (-throttle)
    //   throttle raw: -1=full forward, +1=idle  ->  negate so fwd = positive
    float speed = constrain(pitch * (-throttle), -1.0f, 1.0f);

    // ---- Hardware outputs ---------------------------------------------------
    dacWrite(PIN_AOUT1, floatToDAC(turnRate));
    dacWrite(PIN_AOUT2, floatToDAC(speed));

    // GPIO 18 HIGH if Button 1 (bit 0) OR Button 7 (bit 6) is pressed
    bool btnOut = (btnMask & (1 << BTN1_BIT)) || (btnMask & (1 << BTN7_BIT));
    digitalWrite(PIN_BTN_OUT, btnOut ? HIGH : LOW);

    // Update web state
    g_roll     = roll;
    g_pitch    = pitch;
    g_yaw      = yaw;
    g_throttle = throttle;
    g_turnRate = turnRate;
    g_speed    = speed;
    g_btnMask  = btnMask;
    g_lastPkt  = millis();

    Serial.printf(
        "Aout1(Turn):%+.3f=%.2fV  Aout2(Speed):%+.3f=%.2fV  GPIO18:%s\n",
        turnRate, turnRate * 1.65f + 1.65f,
        speed,    speed    * 1.65f + 1.65f,
        btnOut ? "HIGH" : "LOW"
    );
}
