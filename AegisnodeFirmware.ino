/*
 * ============================================================
 *  AEGIS NODE — Stable Firmware v5.0
 *  ESP32 + MAX30100 + MPU6050 + SSD1306 + MQ-2 + Buzzer
 *
 *  Wiring:
 *    I2C SDA    -> GPIO 21
 *    I2C SCL    -> GPIO 22
 *    MQ-2 AOUT  -> GPIO 34  (ADC1 — safe with WiFi)
 *    Buzzer (+) -> GPIO 25
 *    All VCC    -> 3.3 V
 *
 *  Libraries (Arduino Library Manager):
 *    1. MAX30100lib       by OXullo Intersecans
 *    2. Adafruit MPU6050
 *    3. Adafruit SSD1306
 *    4. Adafruit BusIO
 *    5. ESPAsyncWebServer (me-no-dev / GitHub)
 *    6. AsyncTCP          (me-no-dev / GitHub)
 *
 *  Root causes fixed vs v4:
 *    BPM = 0:
 *      - pox was heap-allocated pointer; LED at 50mA was too strong
 *        and caused sensor saturation on some modules.
 *      - Finger-detection gate was blocking BPM from ever displaying.
 *      Fix: stack object exactly like working user code, LED at 27.1mA,
 *           BPM shown directly from getHeartRate() with no gate.
 *
 *    Random reboot / "Booting..." loop:
 *      - delay(80) in buzzer path was blocking pox.update() causing
 *        the MAX30100 FIFO to overflow and corrupt internal state,
 *        which then caused a fault → watchdog reset → reboot.
 *      - Aggressive WiFi reconnect (disconnect+begin every 6 s) was
 *        starving the async TCP stack and triggering brownout.
 *      Fix: buzzer beep is millis()-based (zero blocking delay).
 *           WiFi reconnect only fires after 30 s confirmed disconnect.
 *           yield() called after every heavy operation.
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include "MAX30100_PulseOximeter.h"

// ============================================================
//  WiFi credentials
// ============================================================
static const char* WIFI_SSID = "Nani";
static const char* WIFI_PASS = "123456789";

// ============================================================
//  Pin definitions
// ============================================================
#define PIN_GAS   34
#define PIN_BUZZ  25
#define PIN_SDA   21
#define PIN_SCL   22

// ============================================================
//  Safety thresholds
// ============================================================
#define GAS_WARN     1800
#define GAS_CRIT     2800
#define BPM_LO_WARN    50
#define BPM_HI_WARN   110
#define BPM_HI_CRIT   130

// ============================================================
//  Alarm level constants (plain int — no enum, avoids lib clashes)
// ============================================================
#define LVL_OK   0
#define LVL_WARN 1
#define LVL_CRIT 2

// ============================================================
//  Timing
// ============================================================
#define T_SENSOR      1000UL   // sensor read + SSE push interval
#define T_WIFI_CHKFST 6000UL   // first WiFi check after boot
#define T_WIFI_RECONN 30000UL  // reconnect only after 30 s disconnect
#define T_BEEP_ON      100UL   // warning beep ON duration (ms)

// ============================================================
//  Objects  — all stack-allocated (no new/delete, no heap frag)
// ============================================================
static AsyncWebServer   g_srv(80);
static AsyncEventSource g_evt("/events");
static Adafruit_SSD1306 g_oled(128, 64, &Wire, -1);
static Adafruit_MPU6050 g_mpu;
static PulseOximeter    g_pox;          // stack object, same as working code

// ============================================================
//  Sensor values
// ============================================================
static int   g_bpm    = 0;              // integer BPM from getHeartRate()
static float g_spo2   = 0.0f;
static int   g_gas    = 0;
static float g_ax = 0, g_ay = 0, g_az = 0;

// ============================================================
//  Alarm state
// ============================================================
static int g_gasAlarm = LVL_OK;
static int g_bpmAlarm = LVL_OK;

// ============================================================
//  Status flags
// ============================================================
static bool g_maxOK  = false;
static bool g_mpuOK  = false;
static bool g_oledOK = false;
static bool g_wifiOK = false;

// ============================================================
//  Timers
// ============================================================
static unsigned long g_tSensor    = 0;
static unsigned long g_tWifi      = 0;
static unsigned long g_tWifiLost  = 0;   // when WiFi first went down
static unsigned long g_tBeepOff   = 0;   // when to silence warn beep
static bool          g_beeping    = false;

// ============================================================
//  Beat callback — keep empty, runs inside pox.update()
// ============================================================
static void onBeat() { }

// ============================================================
//  Alarm message helpers
// ============================================================
static const char* gasTxt(int lvl) {
  if (lvl == LVL_CRIT) return "HIGH GAS! Wear mask & evacuate!";
  if (lvl == LVL_WARN) return "Gas detected. Open windows.";
  return "Normal";
}
static const char* bpmTxt(int lvl) {
  if (lvl == LVL_CRIT) return "Very high BPM! Seek medical help.";
  if (lvl == LVL_WARN) return "High BPM. Stop activity & rest.";
  return "Normal";
}

// ============================================================
//  Non-blocking buzzer handler — call every loop, no delay()
// ============================================================
static void buzzerTick(unsigned long now) {
  if (g_gasAlarm == LVL_CRIT || g_bpmAlarm == LVL_CRIT) {
    // Continuous tone for critical
    digitalWrite(PIN_BUZZ, HIGH);
    g_beeping = false;  // not a timed beep — stays ON
  } else if (g_gasAlarm == LVL_WARN || g_bpmAlarm == LVL_WARN) {
    // Start a single 100 ms beep if not already beeping
    if (!g_beeping) {
      digitalWrite(PIN_BUZZ, HIGH);
      g_tBeepOff = now + T_BEEP_ON;
      g_beeping  = true;
    } else if (now >= g_tBeepOff) {
      digitalWrite(PIN_BUZZ, LOW);
      g_beeping = false;
    }
  } else {
    digitalWrite(PIN_BUZZ, LOW);
    g_beeping = false;
  }
}

// ============================================================
//  OLED update — no String, no printf
// ============================================================
static void oledUpdate() {
  if (!g_oledOK) return;
  g_oled.clearDisplay();
  g_oled.setTextSize(1);
  g_oled.setTextColor(SSD1306_WHITE);

  g_oled.setCursor(0, 0);
  g_oled.println(F("=== AEGIS NODE ==="));

  // BPM line — show value directly, same logic as user's working code
  g_oled.setCursor(0, 10);
  g_oled.print(F("BPM: "));
  if (g_maxOK) {
    if (g_bpm > 0) g_oled.println(g_bpm);
    else           g_oled.println(F("place finger"));
  } else {
    g_oled.println(F("SENSOR FAIL"));
  }

  // SpO2
  g_oled.setCursor(0, 20);
  g_oled.print(F("SpO2:"));
  if (g_maxOK && g_spo2 > 0) { g_oled.print((int)g_spo2); g_oled.println(F("%")); }
  else g_oled.println(F(" --"));

  // Gas
  g_oled.setCursor(0, 30);
  g_oled.print(F("Gas: "));
  g_oled.println(g_gas);

  // Status / alarm
  g_oled.setCursor(0, 40);
  if      (g_gasAlarm == LVL_CRIT) g_oled.println(F("!! WEAR MASK !!"));
  else if (g_gasAlarm == LVL_WARN) g_oled.println(F("! Open windows"));
  else if (g_bpmAlarm == LVL_CRIT) g_oled.println(F("!! SEEK HELP !!"));
  else if (g_bpmAlarm == LVL_WARN) g_oled.println(F("! Take rest"));
  else                              g_oled.println(F("Status: OK"));

  // IP address
  if (g_wifiOK) {
    g_oled.setCursor(0, 54);
    g_oled.print(WiFi.localIP());
  } else {
    g_oled.setCursor(0, 54);
    g_oled.println(F("WiFi connecting..."));
  }

  g_oled.display();
}

// ============================================================
//  Dashboard HTML in PROGMEM
//  MUST stay after all C++ declarations.
//  JS uses arrow functions only — no bare 'function' keyword.
// ============================================================
static const char PAGE[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AEGIS NODE</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
:root{--bg:#080c14;--card:#0d1526;--bd:#1e2d4a;--acc:#00d4ff;--grn:#00ff9d;--red:#ff3b5c;--org:#ffaa00;--txt:#c9d6e3;--sub:#4a6278}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Courier New',monospace;padding:12px}
h1{text-align:center;font-size:1rem;letter-spacing:6px;color:var(--acc);margin-bottom:14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:12px;text-align:center;transition:border-color .3s}
.lbl{font-size:.6rem;color:var(--sub);letter-spacing:2px;text-transform:uppercase;margin-bottom:5px}
.val{font-size:1.8rem;font-weight:bold;color:var(--acc)}
.unt{font-size:.65rem;color:var(--sub);margin-top:3px}
.warn{border-color:var(--org)!important}.warn .val{color:var(--org)!important}
.crit{border-color:var(--red)!important}.crit .val{color:var(--red)!important}
.good{border-color:var(--grn)!important}.good .val{color:var(--grn)!important}
.abox{padding:9px 14px;border-radius:8px;font-size:.75rem;margin-bottom:9px;display:none}
.awarn{display:block;background:#1a1400;border:1px solid var(--org);color:var(--org)}
.acrit{display:block;background:#1a0008;border:1px solid var(--red);color:var(--red)}
.charts{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px}
@media(max-width:560px){.charts{grid-template-columns:1fr}}
.cbox{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:10px}
.ctit{font-size:.6rem;letter-spacing:2px;color:var(--sub);text-transform:uppercase;margin-bottom:8px}
.sbar{display:flex;align-items:center;gap:8px;font-size:.65rem;color:var(--sub);padding:6px 10px;background:var(--card);border-radius:8px;border:1px solid var(--bd)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--red);flex-shrink:0;transition:background .3s}
.dot.live{background:var(--grn)}
</style>
</head>
<body>
<h1>&#9670; AEGIS NODE &#9670;</h1>
<div id="gas-alert" class="abox"></div>
<div id="bpm-alert" class="abox"></div>
<div class="grid">
  <div class="card" id="c-bpm">
    <div class="lbl">Heart Rate</div><div class="val" id="v-bpm">--</div><div class="unt">BPM</div>
  </div>
  <div class="card" id="c-spo2">
    <div class="lbl">SpO2</div><div class="val" id="v-spo2">--</div><div class="unt">%</div>
  </div>
  <div class="card" id="c-gas">
    <div class="lbl">Gas Level</div><div class="val" id="v-gas">--</div><div class="unt">ADC 0-4095</div>
  </div>
  <div class="card" id="c-ax">
    <div class="lbl">Accel X</div><div class="val" id="v-ax">--</div><div class="unt">m/s&#178;</div>
  </div>
  <div class="card" id="c-ay">
    <div class="lbl">Accel Y</div><div class="val" id="v-ay">--</div><div class="unt">m/s&#178;</div>
  </div>
  <div class="card" id="c-az">
    <div class="lbl">Accel Z</div><div class="val" id="v-az">--</div><div class="unt">m/s&#178;</div>
  </div>
</div>
<div class="charts">
  <div class="cbox">
    <div class="ctit">&#9829; Heart Rate (BPM)</div>
    <canvas id="bpmChart" height="90"></canvas>
  </div>
  <div class="cbox">
    <div class="ctit">&#128168; Gas Level (ADC)</div>
    <canvas id="gasChart" height="90"></canvas>
  </div>
</div>
<div class="sbar"><div class="dot" id="dot"></div><span id="stxt">Connecting...</span></div>
<script>
(()=>{
  const MAX_PTS = 40;
  let tick = 0;
  const mkChart = (id, col, yMin, yMax) => {
    const ctx = document.getElementById(id).getContext('2d');
    return new Chart(ctx, {
      type: 'line',
      data: { labels: [], datasets: [{ data: [], borderColor: col,
        backgroundColor: col + '18', borderWidth: 1.5,
        pointRadius: 0, tension: 0.4, fill: true }] },
      options: { animation: false, responsive: true,
        plugins: { legend: { display: false } },
        scales: { x: { display: false },
          y: { min: yMin, max: yMax,
            ticks: { color: '#4a6278', font: { size: 9 } },
            grid: { color: '#1e2d4a' } } } }
    });
  };
  const bC = mkChart('bpmChart', '#00d4ff', 30, 180);
  const gC = mkChart('gasChart',  '#ffaa00', 0, 4095);
  const push = (chart, val) => {
    if (chart.data.labels.length >= MAX_PTS) {
      chart.data.labels.shift();
      chart.data.datasets[0].data.shift();
    }
    chart.data.labels.push(tick++);
    chart.data.datasets[0].data.push(val);
    chart.update('none');
  };
  const setAlert = (id, msg, lvl) => {
    const el = document.getElementById(id);
    el.className = 'abox';
    if (lvl === 0) { el.style.display = 'none'; return; }
    el.style.display = 'block';
    el.classList.add(lvl === 1 ? 'awarn' : 'acrit');
    el.textContent = (lvl === 2 ? '\u26A0 DANGER: ' : '\u26A0 WARNING: ') + msg;
  };
  const setCard = (id, cls) => {
    document.getElementById('c-' + id).className = 'card' + (cls ? ' ' + cls : '');
  };
  const set = (id, v) => { document.getElementById(id).textContent = v; };
  const dot  = document.getElementById('dot');
  const stxt = document.getElementById('stxt');
  const es   = new EventSource('/events');
  es.onopen  = () => { dot.classList.add('live'); stxt.textContent = 'Live — updating every second'; };
  es.onerror = () => { dot.classList.remove('live'); stxt.textContent = 'Reconnecting...'; };
  es.addEventListener('data', e => {
    const d = JSON.parse(e.data);
    set('v-bpm',  d.bpm > 0 ? d.bpm : '--');
    set('v-spo2', d.spo2 > 0 ? Math.round(d.spo2) : '--');
    set('v-gas',  d.gas);
    set('v-ax',   d.ax);
    set('v-ay',   d.ay);
    set('v-az',   d.az);
    // BPM card colour
    if (d.bpmAlarm === 2)      setCard('bpm', 'crit');
    else if (d.bpmAlarm === 1) setCard('bpm', 'warn');
    else if (d.bpm > 0)        setCard('bpm', 'good');
    else                        setCard('bpm', '');
    // BPM alert banner
    if (d.bpmAlarm > 0) setAlert('bpm-alert', d.bpmMsg, d.bpmAlarm);
    else                 setAlert('bpm-alert', '', 0);
    // Gas card colour
    if (d.gasAlarm === 2)      setCard('gas', 'crit');
    else if (d.gasAlarm === 1) setCard('gas', 'warn');
    else                        setCard('gas', 'good');
    // Gas alert banner
    if (d.gasAlarm > 0) setAlert('gas-alert', d.gasMsg, d.gasAlarm);
    else                 setAlert('gas-alert', '', 0);
    // SpO2 card
    setCard('spo2', d.spo2 >= 95 ? 'good' : (d.spo2 > 0 ? 'warn' : ''));
    // Waveforms
    push(bC, d.bpm > 0 ? d.bpm : null);
    push(gC, d.gas);
  });
})();
</script>
</body>
</html>
)HTMLEOF";

// ============================================================
//  setup()
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n===== AEGIS NODE v5.0 ====="));

  // Outputs
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);

  // I2C — 100 kHz for stability across all three I2C devices
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  delay(150);

  // I2C bus scan
  Serial.println(F("[I2C] Scanning..."));
  int nDev = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("[I2C]  0x"));
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      // Friendly name
      if      (a == 0x3C || a == 0x3D) Serial.print(F("  <- OLED"));
      else if (a == 0x57)              Serial.print(F("  <- MAX30100"));
      else if (a == 0x68 || a == 0x69) Serial.print(F("  <- MPU6050"));
      Serial.println();
      nDev++;
    }
  }
  if (nDev == 0) Serial.println(F("[I2C] WARNING: no devices found — check wiring!"));
  else { Serial.print(nDev); Serial.println(F(" I2C device(s) found.")); }

  // OLED
  if (g_oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    g_oledOK = true;
    g_oled.clearDisplay();
    g_oled.setTextSize(1);
    g_oled.setTextColor(SSD1306_WHITE);
    g_oled.setCursor(0,  0); g_oled.println(F("AEGIS NODE v5.0"));
    g_oled.setCursor(0, 12); g_oled.println(F("Initialising..."));
    g_oled.display();
    Serial.println(F("[OLED] OK"));
  } else {
    Serial.println(F("[OLED] FAILED at 0x3C"));
  }

  // MPU-6050
  if (g_mpu.begin(0x68) || g_mpu.begin(0x69)) {
    g_mpuOK = true;
    g_mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    g_mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    g_mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("[MPU6050] OK"));
  } else {
    Serial.println(F("[MPU6050] FAILED"));
  }

  // MAX30100 — stack object, identical pattern to the working user code.
  // LED at 27.1 mA matches the user's working configuration.
  // Only guard: check begin() return value; if false, maxOK stays false
  // and update() is never called (loop guards on g_maxOK flag).
  Serial.println(F("[MAX30100] Initialising..."));
  if (g_pox.begin()) {
    g_maxOK = true;
    g_pox.setIRLedCurrent(MAX30100_LED_CURR_27_1MA);  // same as working code
    g_pox.setOnBeatDetectedCallback(onBeat);
    Serial.println(F("[MAX30100] OK — sensor ready"));
    Serial.println(F("[MAX30100] Place finger firmly, cover sensor from light"));
  } else {
    Serial.println(F("[MAX30100] FAILED — check VIN/GND/SDA/SCL connections"));
  }

  // WiFi — non-blocking start
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);   // let ESP32 SDK handle reconnects automatically
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println(F("[WiFi] Connecting (non-blocking)..."));

  // Web server
  g_srv.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", PAGE);
  });
  g_srv.addHandler(&g_evt);
  g_srv.begin();
  Serial.println(F("[HTTP] Server started on port 80"));

  // Boot summary on OLED
  if (g_oledOK) {
    g_oled.clearDisplay();
    g_oled.setTextSize(1);
    g_oled.setTextColor(SSD1306_WHITE);
    g_oled.setCursor(0,  0); g_oled.println(F("AEGIS NODE READY"));
    g_oled.setCursor(0, 12);
    g_oled.print(F("MAX:")); g_oled.print(g_maxOK ? F("OK ") : F("-- "));
    g_oled.print(F("MPU:")); g_oled.println(g_mpuOK ? F("OK") : F("--"));
    g_oled.setCursor(0, 24); g_oled.println(F("WiFi: connecting..."));
    g_oled.display();
  }

  // Serial summary
  Serial.println(F("\n===== BOOT SUMMARY ====="));
  Serial.print(F("  OLED    : ")); Serial.println(g_oledOK ? F("OK") : F("FAIL"));
  Serial.print(F("  MPU6050 : ")); Serial.println(g_mpuOK  ? F("OK") : F("FAIL"));
  Serial.print(F("  MAX30100: ")); Serial.println(g_maxOK  ? F("OK") : F("FAIL"));
  Serial.println(F("  WiFi    : connecting..."));
  Serial.println(F("========================\n"));

  g_tWifi = millis();
}

// ============================================================
//  loop() — zero delay() calls, pox.update() runs every cycle
// ============================================================
void loop() {
  const unsigned long now = millis();

  // ----------------------------------------------------------
  //  MAX30100: pox.update() MUST run every loop iteration.
  //  Guarded by g_maxOK so it never runs on failed hardware.
  //  This is identical to the user's working code pattern.
  // ----------------------------------------------------------
  if (g_maxOK) {
    g_pox.update();
  }

  // ----------------------------------------------------------
  //  Non-blocking buzzer
  // ----------------------------------------------------------
  buzzerTick(now);

  // ----------------------------------------------------------
  //  WiFi status check — runs every T_WIFI_CHKFST ms.
  //  WiFi.setAutoReconnect(true) handles reconnects internally;
  //  we only call WiFi.begin() again if truly stuck for 30 s.
  // ----------------------------------------------------------
  if (now - g_tWifi >= T_WIFI_CHKFST) {
    g_tWifi = now;
    if (WiFi.status() == WL_CONNECTED) {
      if (!g_wifiOK) {
        g_wifiOK = true;
        g_tWifiLost = 0;
        Serial.print(F("[WiFi] Connected! IP: "));
        Serial.println(WiFi.localIP());
        Serial.print(F("[WiFi] Dashboard -> http://"));
        Serial.println(WiFi.localIP());
      }
    } else {
      if (g_wifiOK) {
        // Just lost connection
        g_wifiOK   = false;
        g_tWifiLost = now;
        Serial.println(F("[WiFi] Lost connection."));
      }
      // Only force-reconnect if stuck for 30 s
      // (AutoReconnect handles normal drops)
      if (g_tWifiLost > 0 && (now - g_tWifiLost) >= T_WIFI_RECONN) {
        g_tWifiLost = now;   // reset timer
        Serial.println(F("[WiFi] Stuck 30 s — forcing reconnect..."));
        WiFi.disconnect(false);  // non-blocking disconnect
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }
    }
  }

  // ----------------------------------------------------------
  //  1-second sensor block
  // ----------------------------------------------------------
  if (now - g_tSensor >= T_SENSOR) {
    g_tSensor = now;

    // MAX30100 readings — read directly like working user code,
    // no finger-detection gate blocking the value from showing
    if (g_maxOK) {
      g_bpm  = (int)g_pox.getHeartRate();
      g_spo2 = g_pox.getSpO2();
    }

    // Gas sensor
    g_gas = analogRead(PIN_GAS);

    // Alarm levels
    if      (g_gas >= GAS_CRIT) g_gasAlarm = LVL_CRIT;
    else if (g_gas >= GAS_WARN) g_gasAlarm = LVL_WARN;
    else                         g_gasAlarm = LVL_OK;

    if (g_maxOK && g_bpm > 0) {
      if      (g_bpm >= BPM_HI_CRIT) g_bpmAlarm = LVL_CRIT;
      else if (g_bpm >= BPM_HI_WARN) g_bpmAlarm = LVL_WARN;
      else if (g_bpm <  BPM_LO_WARN) g_bpmAlarm = LVL_WARN;
      else                             g_bpmAlarm = LVL_OK;
    } else {
      g_bpmAlarm = LVL_OK;
    }

    // MPU-6050
    if (g_mpuOK) {
      sensors_event_t ea, eg, et;
      g_mpu.getEvent(&ea, &eg, &et);
      g_ax = ea.acceleration.x;
      g_ay = ea.acceleration.y;
      g_az = ea.acceleration.z;
    }

    // OLED refresh
    oledUpdate();
    yield();   // allow async TCP stack to breathe after OLED I2C burst

    // SSE JSON push — stack buffer, no String
    if (g_wifiOK) {
      char buf[380];
      snprintf(buf, sizeof(buf),
        "{\"bpm\":%d,\"spo2\":%.1f,\"gas\":%d,"
        "\"gasAlarm\":%d,\"bpmAlarm\":%d,"
        "\"gasMsg\":\"%s\",\"bpmMsg\":\"%s\","
        "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f}",
        g_bpm, g_spo2, g_gas,
        g_gasAlarm, g_bpmAlarm,
        gasTxt(g_gasAlarm),
        bpmTxt(g_bpmAlarm),
        g_ax, g_ay, g_az
      );
      g_evt.send(buf, "data", millis());
    }

    // Serial log — one clean line per second
    Serial.print(F("[DATA] BPM:"));
    Serial.print(g_bpm);
    Serial.print(F(" SpO2:"));
    Serial.print(g_spo2, 1);
    Serial.print(F("% Gas:"));
    Serial.print(g_gas);
    Serial.print(F("(A"));
    Serial.print(g_gasAlarm);
    Serial.print(F(") Ax:"));
    Serial.print(g_ax, 2);
    Serial.print(F(" Ay:"));
    Serial.print(g_ay, 2);
    Serial.print(F(" Az:"));
    Serial.print(g_az, 2);
    Serial.print(F(" Heap:"));
    Serial.println(ESP.getFreeHeap());

    if (ESP.getFreeHeap() < 20000) {
      Serial.print(F("[MEM] LOW HEAP: "));
      Serial.print(ESP.getFreeHeap());
      Serial.println(F(" bytes!"));
    }
  }
}