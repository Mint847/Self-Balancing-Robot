#include <Wire.h>
#include <MPU6050_light.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
// Motor A (Left)
const int PWMA = 19;
const int AIN1 = 18;
const int AIN2 = 5;

// Motor B (Right)
const int PWMB = 25;
const int BIN1 = 26;
const int BIN2 = 27;

// Encoders
const int ENC_L_A = 32;
const int ENC_L_B = 33;
const int ENC_R_A = 13;
const int ENC_R_B = 14;

// ==========================================
// WIFI CONFIGURATION
// ==========================================
// Connect your phone to this network, then open 192.168.4.1
const char* AP_SSID = "SelfBalance-Tuner";
const char* AP_PASS = "change-me-8chars";   // CHANGE THIS (min 8 characters)

WebServer server(80);

// ==========================================
// CONFIGURATION
// ==========================================
const int   maxPWM        = 255;
const float LOOP_HZ       = 200.0f;                       // inner loop rate
const unsigned long LOOP_US = (unsigned long)(1000000.0f / LOOP_HZ);
const float MAX_TILT_OFFSET = 5.0f;                       // max lean the speed loop may request (deg)
const float MAX_TRIM     = 15.0f;                        // max auto-learned setpoint shift (deg) = steepest slope handled
const float I_LIMIT       = 100.0f;                       // max PWM contributed by the integral term
const float GYRO_SIGN     = 1.0f;                         // set to -1.0f if Kd makes oscillation WORSE
const bool  DEBUG_SERIAL  = false;                        // serial prints block the loop; keep off when balancing

// ==========================================
// TUNABLE PARAMETERS (floats: atomic between the two cores)
// ==========================================
Preferences prefs;
volatile float Kp = 45.0f;       // angle P
volatile float Ki = 1.0f;        // angle I
volatile float Kd = 1.0f;        // angle D (applied to gyro rate, deg/s)
volatile float setpoint = -4.83f;// mechanical neutral angle (deg)
volatile float deadband = 0.0f;  // PWM needed just to make the motors move
volatile float Kv = 0.30f;       // speed loop: deg of lean per 100 ticks/s  (the anti-drift term)
volatile float Kt = 0.30f;       // auto-trim rate: how fast the setpoint learns the slope (deg/s per deg of lean). 0 = off
volatile float Kx = 0.02f;       // position loop: deg of lean per 100 ticks of displacement

// ==========================================
// STATE
// ==========================================
MPU6050 mpu(Wire);
volatile float pitch = 0;
volatile float gyroRate = 0;
volatile float outputPWM = 0;
volatile float dynamicSetpoint = -4.83f;
volatile float filteredSpeed = 0;     // avg ticks/s per wheel (+ = driving forward)
float posTicks = 0;                   // integrated wheel position (ticks, avg of both wheels)
float tiltOffset = 0;
volatile float trim = 0;              // learned setpoint shift for the current surface slope (deg)
float iTerm = 0;
float prevTicks = 0;                  // previous averaged tick count of the working encoders

volatile long leftTicks = 0;
volatile long rightTicks = 0;
bool leftOk = true, rightOk = true;  // false = that encoder is dead and is ignored
int leftSign = 1, rightSign = 1;      // encoder direction correction (set by the encoder test)

volatile bool motorEnabled = true;
bool robotFallen = false;

// Requests from the web task, executed by the control loop (keeps I2C/motors on one core)
volatile bool calRequested = false, calDone = false;
volatile bool encTestRequested = false, encTestDone = false;
String encResult = "";

void setMotors(int leftSpeed, int rightSpeed);

// ==========================================
// ENCODER ISRs
// ==========================================
void IRAM_ATTR isrLeftEncoder() {
  if (digitalRead(ENC_L_B) == HIGH) leftTicks++;
  else leftTicks--;
}

void IRAM_ATTR isrRightEncoder() {
  if (digitalRead(ENC_R_B) == LOW) rightTicks++;
  else rightTicks--;
}

// ==========================================
// WEB PAGE
// ==========================================
String paramRow(const char* id, const char* label, const char* desc, float mn, float mx, float val) {
  String v = String(val, 2), lo = String(mn, 2), hi = String(mx, 2);
  String s = "<div class='param-label'><strong>" + String(label) + "</strong> <span style='color:#aaa;font-size:0.85em;'>" + String(desc) + "</span></div>";
  s += "<div class='slider-row'><input type='range' id='" + String(id) + "_s' min='" + lo + "' max='" + hi + "' step='0.01' value='" + v + "' oninput=\"syncFromSlider('" + String(id) + "')\">";
  s += "<input type='number' class='num-input' id='" + String(id) + "' min='" + lo + "' max='" + hi + "' step='0.01' value='" + v + "' oninput=\"syncFromNumber('" + String(id) + "')\">";
  s += "<div class='step-btns'><button class='sbtn' onclick=\"step('" + String(id) + "',0.01)\">&#9650;</button>";
  s += "<button class='sbtn' onclick=\"step('" + String(id) + "',-0.01)\">&#9660;</button></div></div>";
  return s;
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>PID Tuner</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: -apple-system, sans-serif; background: #1a1a2e; color: #eee; padding: 20px; }
    h1 { text-align: center; color: #e94560; margin-bottom: 24px; font-size: 1.4em; }
    .card { background: #16213e; border-radius: 16px; padding: 20px; margin-bottom: 16px; }
    .param-label { display: flex; justify-content: space-between; align-items: center; font-size: 0.95em; margin-bottom: 6px; color: #aaa; }
    .param-label strong { color: #eee; }
    .slider-row { display: flex; align-items: center; gap: 10px; margin-bottom: 18px; }
    .slider-row input[type=range] { flex: 1; accent-color: #e94560; height: 6px; }
    .num-input { width: 72px; background: #0f3460; border: 1.5px solid #e94560; border-radius: 8px;
                 color: #e94560; font-size: 1em; font-weight: bold; text-align: center;
                 padding: 6px 4px; -moz-appearance: textfield; }
    .num-input::-webkit-inner-spin-button, .num-input::-webkit-outer-spin-button { -webkit-appearance: none; }
    .num-input:focus { outline: none; border-color: #ff9800; color: #ff9800; }
    .step-btns { display: flex; flex-direction: column; gap: 3px; }
    .sbtn { width: 32px; height: 28px; background: #0f3460; border: 1.5px solid #555;
            border-radius: 7px; color: #eee; font-size: 1em; font-weight: bold;
            cursor: pointer; display: flex; align-items: center; justify-content: center; padding: 0;
            touch-action: manipulation; user-select: none; -webkit-user-select: none; }
    .sbtn:active { background: #e94560; border-color: #e94560; color: white; }
    .btn { display: block; width: 100%; padding: 16px; background: #e94560; color: white;
           border: none; border-radius: 12px; font-size: 1.1em; font-weight: bold;
           cursor: pointer; margin-top: 8px; }
    .btn:active { background: #c73652; }
    .btn-stop { background: #ff5722; }
    .btn-resume { background: #4caf50; }
    .btn-recal { background: #ff9800; }
    .btn-enc { background: #2196f3; }
    .status { text-align: center; color: #4caf50; margin-top: 12px; font-size: 0.9em; min-height: 20px; }
    .pitch-box { text-align: center; background: #0f3460; border-radius: 12px; padding: 14px; margin-bottom: 16px; }
    .pitch-val { font-size: 2.5em; font-weight: bold; color: #e94560; }
    .pitch-label { font-size: 0.8em; color: #aaa; margin-top: 4px; }
    .stopped-banner { text-align:center; background:#ff5722; border-radius:10px; padding:10px; margin-bottom:14px; font-weight:bold; display:none; }
    .hint { color:#888; font-size:0.8em; margin-bottom:14px; }
  </style>
</head>
<body>
  <h1>&#9881; PID Tuner</h1>

  <div class='pitch-box'>
    <div class='pitch-val' id='pitchVal'>--</div>
    <div class='pitch-label'>Current Pitch (degrees)</div>
    <div style='margin-top:8px;font-size:0.85em;color:#4caf50;' id='tele'>--</div>
  </div>

  <div class='card'>
    <h3 style='color:#e94560;margin-bottom:12px;font-size:1.05em;'>&#9881; Balance Controller (Inner Loop)</h3>
)rawliteral";

  html += paramRow("kp", "Kp", "Proportional", 0, 100, Kp);
  html += paramRow("ki", "Ki", "Integral", 0, 10, Ki);
  html += paramRow("kd", "Kd", "Damping (gyro)", 0, 20, Kd);
  html += paramRow("sp", "Setpoint", "Neutral Angle (deg)", -15, 15, setpoint);
  html += paramRow("db", "Deadband", "Min motor PWM", 0, 100, deadband);

  html += R"rawliteral(
  </div>

  <div class='card'>
    <h3 style='color:#e94560;margin-bottom:12px;font-size:1.05em;'>&#128640; Anti-Drift Controller (Outer Loop)</h3>
    <div class='hint'>Uses wheel encoders: if the robot rolls away, it leans back to return. Run the encoder test first (below). Start Kv at 0.30; raise it if it still drifts, lower it if it oscillates slowly. Kx pulls it back to its start spot.</div>
)rawliteral";

  html += paramRow("kv", "Kv", "Lean per speed", 0, 3, Kv);
  html += paramRow("kx", "Kx", "Lean per distance", 0, 1, Kx);
  html += paramRow("kt", "Auto-trim", "Slope learning rate", 0, 2, Kt);

  html += R"rawliteral(
    <button class='btn' onclick='sendValues()'>&#9654; Apply Values</button>
    <div class='status' id='status'></div>
  </div>

  <div class='stopped-banner' id='stoppedBanner'>&#9888; MOTORS STOPPED</div>

  <div class='card'>
    <button class='btn btn-stop' id='stopBtn' onclick='stopMotors()'>&#9632; Stop Motors</button>
    <button class='btn btn-resume' id='resumeBtn' onclick='resumeMotors()' style='display:none;margin-top:10px;'>&#9654; Resume Balancing</button>
  </div>

  <div class='card'>
    <button class='btn btn-enc' onclick='encTest()'>&#9881; Test Encoders (wheels in the air!)</button>
    <div class='status' id='encStatus'></div>
    <div class='hint' style='margin-top:8px;'>Lift the robot so the wheels spin freely. Both wheels spin forward briefly; the encoder directions are detected and saved automatically. Do this once.</div>
  </div>

  <div class='card'>
    <button class='btn btn-recal' onclick='calibrateGyro()'>&#8635; Calibrate Gyro (Keep Still)</button>
    <div class='status' id='recalStatus'></div>
    <div class='hint' style='margin-top:8px;'>Place the robot completely still at any angle. Saves gyro bias to Flash.</div>
  </div>

  <script>
    function step(id, delta) {
      var el = document.getElementById(id);
      var v = parseFloat(el.value) || 0;
      v = Math.round((v + delta) * 100) / 100;
      v = Math.min(parseFloat(el.max), Math.max(parseFloat(el.min), v));
      el.value = v.toFixed(2);
      document.getElementById(id + '_s').value = v;
    }
    function syncFromNumber(id) {
      var v = parseFloat(document.getElementById(id).value);
      if (!isNaN(v)) document.getElementById(id + '_s').value = v;
    }
    function syncFromSlider(id) {
      document.getElementById(id).value = parseFloat(document.getElementById(id + '_s').value).toFixed(2);
    }
    function sendValues() {
      var keys = ['kp','ki','kd','sp','db','kv','kx','kt'];
      var url = '/set?' + keys.map(function(k){ return k + '=' + parseFloat(document.getElementById(k).value).toFixed(2); }).join('&');
      fetch(url).then(function(r){ return r.text(); }).then(function(){
        document.getElementById('status').innerText = '✓ Applied & Saved!';
        setTimeout(function(){ document.getElementById('status').innerText = ''; }, 3000);
      }).catch(function(){ document.getElementById('status').innerText = '✗ Failed. Is the robot on?'; });
    }
    function stopMotors() {
      fetch('/stop?state=0').then(function(){
        document.getElementById('stopBtn').style.display = 'none';
        document.getElementById('resumeBtn').style.display = 'block';
        document.getElementById('stoppedBanner').style.display = 'block';
      });
    }
    function resumeMotors() {
      fetch('/stop?state=1').then(function(){
        document.getElementById('resumeBtn').style.display = 'none';
        document.getElementById('stopBtn').style.display = 'block';
        document.getElementById('stoppedBanner').style.display = 'none';
      });
    }
    function calibrateGyro() {
      var btn = document.querySelector('.btn-recal');
      btn.disabled = true; btn.innerText = 'Calibrating...';
      document.getElementById('recalStatus').innerText = 'Keep robot completely still!';
      fetch('/calgyro').then(function(){
        btn.disabled = false; btn.innerText = '↺ Calibrate Gyro (Keep Still)';
        document.getElementById('recalStatus').innerText = '✓ Gyro bias saved!';
        setTimeout(function(){ document.getElementById('recalStatus').innerText = ''; }, 3000);
      });
    }
    function encTest() {
      document.getElementById('encStatus').innerText = 'Testing...';
      fetch('/enctest').then(function(r){ return r.text(); }).then(function(t){
        document.getElementById('encStatus').innerText = t;
      }).catch(function(){ document.getElementById('encStatus').innerText = 'Test failed'; });
    }
    setInterval(function(){
      fetch('/pitch').then(function(r){ return r.text(); }).then(function(t){
        var p = t.split(',');
        document.getElementById('pitchVal').innerText = parseFloat(p[0]).toFixed(2) + '°';
        document.getElementById('tele').innerHTML = 'Speed: ' + p[1] + ' t/s | Target: ' + p[2] + '° | Motor: ' + p[3] + ' | Trim: ' + p[4] + '°<br>Raw encoders  L: ' + p[5] + '  R: ' + p[6];
      }).catch(function(){});
    }, 400);
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ==========================================
// WEB HANDLERS (run on core 0, never touch I2C or motors directly)
// ==========================================
void handleSet() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  if (server.hasArg("sp")) setpoint = server.arg("sp").toFloat();
  if (server.hasArg("db")) deadband = server.arg("db").toFloat();
  if (server.hasArg("kv")) Kv = server.arg("kv").toFloat();
  if (server.hasArg("kx")) Kx = server.arg("kx").toFloat();
  if (server.hasArg("kt")) Kt = server.arg("kt").toFloat();

  prefs.putFloat("v2_kp", Kp);
  prefs.putFloat("v2_ki", Ki);
  prefs.putFloat("v2_kd", Kd);
  prefs.putFloat("v2_sp", setpoint);
  prefs.putFloat("v2_db", deadband);
  prefs.putFloat("v2_kv", Kv);
  prefs.putFloat("v2_kx", Kx);
  prefs.putFloat("v2_kt", Kt);
  server.send(200, "text/plain", "OK");
}

void handlePitch() {
  String data = String(pitch, 2) + "," + String((int)filteredSpeed) + "," + String(dynamicSetpoint, 2) + "," + String((int)outputPWM) + "," + String(trim, 2) + "," + String(leftTicks) + "," + String(rightTicks);
  server.send(200, "text/plain", data);
}

void handleStop() {
  if (server.hasArg("state")) motorEnabled = (server.arg("state") == "1");
  server.send(200, "text/plain", "OK");
}

void handleCalGyro() {
  calDone = false;
  calRequested = true;
  unsigned long t0 = millis();
  while (!calDone && millis() - t0 < 8000) delay(10);
  server.send(200, "text/plain", "OK");
}

void handleEncTest() {
  encTestDone = false;
  encTestRequested = true;
  unsigned long t0 = millis();
  while (!encTestDone && millis() - t0 < 4000) delay(10);
  server.send(200, "text/plain", encTestDone ? encResult : String("Timed out"));
}

void webTask(void*) {
  for (;;) {
    server.handleClient();
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// HELPERS
// ==========================================
void resetControl() {
  iTerm = 0;
  tiltOffset = 0;
  trim = 0;
  posTicks = 0;
  filteredSpeed = 0;
  outputPWM = 0;
  leftTicks = 0; rightTicks = 0; prevTicks = 0;
  dynamicSetpoint = setpoint;
}

void saveGyroOffsets() {
  prefs.putFloat("gx", mpu.getGyroXoffset());
  prefs.putFloat("gy", mpu.getGyroYoffset());
  prefs.putFloat("gz", mpu.getGyroZoffset());
  prefs.putBool("hasGyro", true);
}

void runEncoderTest() {
  setMotors(0, 0);
  delay(200);
  leftTicks = 0; rightTicks = 0;
  setMotors(150, 150);
  delay(500);
  setMotors(0, 0);
  delay(100);
  long l = leftTicks, r = rightTicks;
  encResult = "Left: " + String(l) + " ticks, Right: " + String(r) + " ticks. ";
  leftOk  = abs(l) >= 3;
  rightOk = abs(r) >= 3;
  if (leftOk)  leftSign  = (l > 0) ? 1 : -1;
  if (rightOk) rightSign = (r > 0) ? 1 : -1;
  prefs.putBool("v2_lo", leftOk);
  prefs.putBool("v2_ro", rightOk);
  prefs.putInt("v2_ls", leftSign);
  prefs.putInt("v2_rs", rightSign);
  encResult += String("Left: ") + (!leftOk ? "DEAD (ignored, check wiring)" : (leftSign > 0 ? "ok" : "inverted (fixed)")) + ". ";
  encResult += String("Right: ") + (!rightOk ? "DEAD (ignored, check wiring)" : (rightSign > 0 ? "ok" : "inverted (fixed)")) + ". ";
  if (!leftOk && !rightOk) encResult += "No working encoder: anti-drift loop disabled.";
  else encResult += "Saved.";
  resetControl();
  encTestDone = true;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(4, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(4, HIGH); delay(200);
    digitalWrite(4, LOW);  delay(200);
  }

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  setMotors(0, 0);

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightEncoder, RISING);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
    Serial.println("Could not connect to MPU6050. Check wiring!");
    digitalWrite(4, !digitalRead(4));
    delay(500);
  }
  digitalWrite(4, HIGH); delay(500); digitalWrite(4, LOW);

  prefs.begin("balance", false);

  // Gyro offsets
  if (prefs.getBool("hasGyro", false)) {
    mpu.setGyroOffsets(prefs.getFloat("gx", 0.0), prefs.getFloat("gy", 0.0), prefs.getFloat("gz", 0.0));
  } else {
    Serial.println(F("First run: calibrating gyro (keep robot still)..."));
    delay(500);
    mpu.calcOffsets(true, false);   // gyro only - never the accelerometer
    saveGyroOffsets();
  }

  // Fast-converge the angle estimate so it doesn't start from 0 deg
  mpu.setFilterGyroCoef(0.5);
  for (int i = 0; i < 300; i++) { mpu.update(); delay(5); }
  mpu.setFilterGyroCoef(0.98);

  // Load saved parameters (new keys, so old incompatible values are ignored)
  Kp       = prefs.getFloat("v2_kp", 45.0f);
  Ki       = prefs.getFloat("v2_ki", 1.0f);
  Kd       = prefs.getFloat("v2_kd", 1.0f);
  setpoint = prefs.getFloat("v2_sp", -4.83f);
  deadband = prefs.getFloat("v2_db", 0.0f);
  Kv       = prefs.getFloat("v2_kv", 0.30f);
  Kx       = prefs.getFloat("v2_kx", 0.02f);
  Kt       = prefs.getFloat("v2_kt", 0.30f);
  leftSign  = prefs.getInt("v2_ls", 1);
  rightSign = prefs.getInt("v2_rs", 1);
  leftOk    = prefs.getBool("v2_lo", true);
  rightOk   = prefs.getBool("v2_ro", true);
  dynamicSetpoint = setpoint;

  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/",        handleRoot);
  server.on("/set",     handleSet);
  server.on("/pitch",   handlePitch);
  server.on("/stop",    handleStop);
  server.on("/calgyro", handleCalGyro);
  server.on("/enctest", handleEncTest);
  server.begin();

  // Web server on core 0 so it can never stall the control loop on core 1
  xTaskCreatePinnedToCore(webTask, "web", 8192, NULL, 1, NULL, 0);

  Serial.println(F("Ready. Connect to WiFi 'SelfBalance-Tuner', open 192.168.4.1"));
}

// ==========================================
// MAIN CONTROL LOOP (fixed 200 Hz)
// ==========================================
void loop() {
  static unsigned long lastUs = micros();
  unsigned long nowUs = micros();
  if (nowUs - lastUs < LOOP_US) return;
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 0.02f) dt = 0.02f;

  // 1. Sensor
  mpu.update();
  pitch = mpu.getAngleY();
  gyroRate = GYRO_SIGN * mpu.getGyroY();

  // 2. Requests from the web page
  if (calRequested) {
    setMotors(0, 0);
    delay(300);
    mpu.calcOffsets(true, false);
    saveGyroOffsets();
    mpu.setFilterGyroCoef(0.5);
    for (int i = 0; i < 100; i++) { mpu.update(); delay(5); }
    mpu.setFilterGyroCoef(0.98);
    resetControl();
    calRequested = false;
    calDone = true;
    lastUs = micros();
    return;
  }
  if (encTestRequested) {
    runEncoderTest();
    encTestRequested = false;
    lastUs = micros();
    return;
  }

  float tilt = pitch - (setpoint + trim);

  // 3. Safety states
  if (!motorEnabled) {
    setMotors(0, 0);
    resetControl();
    return;
  }
  if (fabs(tilt) > 35.0f) {
    robotFallen = true;
    setMotors(0, 0);
    resetControl();
    return;
  }
  if (robotFallen) {
    if (fabs(tilt) < 8.0f) {
      robotFallen = false;
      resetControl();
    } else {
      setMotors(0, 0);
      return;
    }
  }

  // 4. Outer loop: wheel speed + position -> lean offset (~25 Hz)
  static unsigned long lastSpeedUs = 0;
  if (nowUs - lastSpeedUs >= 40000UL) {
    float sdt = (nowUs - lastSpeedUs) * 1e-6f;
    lastSpeedUs = nowUs;
    if (sdt > 0.1f) sdt = 0.04f;

    // Average only the encoders that actually work, each with its corrected direction
    float total = 0; int n = 0;
    if (leftOk)  { total += (float)(leftTicks  * leftSign);  n++; }
    if (rightOk) { total += (float)(rightTicks * rightSign); n++; }
    float deltaAvg = 0;
    if (n > 0) { total /= n; deltaAvg = total - prevTicks; prevTicks = total; }

    filteredSpeed = filteredSpeed * 0.7f + (deltaAvg / sdt) * 0.3f;

    float kx = Kx, kv = Kv;
    if (kx > 0.0001f) {
      posTicks += deltaAvg;
      float lim = MAX_TILT_OFFSET * 100.0f / kx;       // anti-windup: position term alone can't exceed max lean
      posTicks = constrain(posTicks, -lim, lim);
    } else {
      posTicks = 0;
    }

    // Moving forward (+) -> lean back (lower target angle), and vice versa
    float target = (n > 0) ? -(kv * filteredSpeed + kx * posTicks) / 100.0f : 0.0f;
    target = constrain(target, -MAX_TILT_OFFSET, MAX_TILT_OFFSET);
    tiltOffset += 0.4f * (target - tiltOffset);        // smooth

    // Auto-trim: a lean that is requested persistently means the real balance angle has moved
    // (slope, battery, load). Fold it into the setpoint so the robot re-learns its equilibrium.
    // (only while the balance loop is in control, so a fall/runaway can't wind the trim up)
    if (fabs(pitch - dynamicSetpoint) < 6.0f) trim += Kt * tiltOffset * sdt;
    trim = constrain(trim, -MAX_TRIM, MAX_TRIM);
    dynamicSetpoint = setpoint + trim + tiltOffset;
  }

  // 5. Inner loop: angle PID with gyro-rate derivative
  float err = pitch - dynamicSetpoint;
  iTerm += Ki * err * dt;
  iTerm = constrain(iTerm, -I_LIMIT, I_LIMIT);

  float u = Kp * err + iTerm + Kd * gyroRate;
  u = constrain(u, -(float)maxPWM, (float)maxPWM);

  // Motor deadband compensation
  float db = deadband;
  float cmd = 0;
  if (fabs(u) > 1.0f) {
    cmd = (u > 0 ? 1.0f : -1.0f) * (db + fabs(u) * (maxPWM - db) / maxPWM);
  }
  outputPWM = cmd;
  setMotors((int)cmd, (int)cmd);

  if (DEBUG_SERIAL) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.printf("P:%.2f spd:%.0f sp:%.2f out:%.0f\n", pitch, filteredSpeed, dynamicSetpoint, outputPWM);
    }
  }
}

// ==========================================
// MOTOR CONTROL
// ==========================================
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -maxPWM, maxPWM);
  rightSpeed = constrain(rightSpeed, -maxPWM, maxPWM);

  // LEFT
  if (leftSpeed > 0)      { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else if (leftSpeed < 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  else                    { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW); }
  analogWrite(PWMA, abs(leftSpeed));

  // RIGHT (direction swapped to match left motor)
  if (rightSpeed > 0)      { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else if (rightSpeed < 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else                     { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW); }
  analogWrite(PWMB, abs(rightSpeed));
}
