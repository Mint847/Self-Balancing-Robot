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

// Driving (web D-pad)
const float DRIVE_SIGN    = 1.0f;     // flip to -1.0f if the "forward" button drives backward
const float TURN_SIGN     = 1.0f;     // flip to -1.0f if left/right turn the wrong way
const float DRIVE_ACCEL   = 400.0f;   // how fast the target speed ramps (ticks/s per second)
const unsigned long DRIVE_TIMEOUT_MS = 300;  // no command for this long -> stop driving (failsafe)

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
volatile float driveSpeed = 300.0f; // target wheel speed at full forward (ticks/s)
volatile float turnPower  = 40.0f;  // PWM difference between wheels when turning

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
volatile bool robotFallen = false;

// Drive commands from the web page (-1..1), with failsafe timestamp
volatile float driveCmd = 0, turnCmd = 0;
volatile unsigned long lastDriveMs = 0;
float speedTarget = 0;                // ramped target wheel speed (ticks/s)

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
// WEB PAGE (static; values are loaded by the page from /config)
// ==========================================
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#0b0f17">
<title>Balance Bot</title>
<style>
:root{--bg:#0b0f17;--card:#121826;--card2:#182033;--line:#253048;--text:#e7ecf5;--muted:#8b97ad;
--accent:#4f8cff;--accent2:#7aa7ff;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;--r:16px}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
html,body{background:var(--bg);color:var(--text);font:15px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
body{padding-bottom:96px}
header{position:sticky;top:0;z-index:5;display:flex;align-items:center;justify-content:space-between;
padding:14px 16px;background:rgba(11,15,23,.92);backdrop-filter:blur(8px);border-bottom:1px solid var(--line)}
.brand{display:flex;align-items:center;gap:10px;font-weight:700;font-size:17px}
.logo{width:28px;height:28px;border-radius:8px;background:linear-gradient(135deg,var(--accent),#9b6bff);display:grid;place-items:center;font-size:15px}
.status{display:flex;align-items:center;gap:10px}
.dot{width:10px;height:10px;border-radius:50%;background:var(--bad);box-shadow:0 0 0 3px rgba(239,68,68,.2);transition:.3s}
.dot.on{background:var(--ok);box-shadow:0 0 0 3px rgba(34,197,94,.2)}
.pill{font-size:12px;font-weight:700;letter-spacing:.06em;padding:5px 10px;border-radius:999px;background:var(--card2);color:var(--muted)}
.pill.ok{background:rgba(34,197,94,.15);color:var(--ok)}.pill.warn{background:rgba(245,158,11,.15);color:var(--warn)}
.pill.bad{background:rgba(239,68,68,.15);color:var(--bad)}.pill.drive{background:rgba(79,140,255,.18);color:var(--accent2)}
main{max-width:560px;margin:0 auto;padding:16px;display:grid;gap:14px}
.card{background:var(--card);border:1px solid var(--line);border-radius:var(--r);padding:16px}
h2{font-size:13px;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin-bottom:12px;font-weight:700}
.live-top{display:flex;align-items:center;gap:12px}
.tilt{width:130px;height:100px;flex:none}
.ground{stroke:var(--line);stroke-width:2}.tline{stroke:var(--warn);stroke-width:1.5;stroke-dasharray:3 3}
.body{fill:var(--accent)}.face{fill:#0b0f17}.wheel{fill:#2b3550;stroke:var(--accent2);stroke-width:2}
.pitchbox{flex:1;text-align:right}
.lbl{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}
.big{font-size:44px;font-weight:800;line-height:1.05;font-variant-numeric:tabular-nums}
.sub{color:var(--muted);font-size:13px;font-variant-numeric:tabular-nums}
.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:14px}
.tile{background:var(--card2);border-radius:12px;padding:10px 8px;text-align:center;min-width:0}
.tile b{display:block;font-size:17px;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.tile span{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.05em}
.mbar{height:4px;background:var(--line);border-radius:2px;margin-top:6px;position:relative;overflow:hidden}
.mbar i{position:absolute;top:0;bottom:0;background:var(--accent);left:50%;width:0}
.enc{margin-top:10px;font-size:12px;color:var(--muted);text-align:center;font-variant-numeric:tabular-nums}
.pad{display:grid;grid-template-columns:repeat(3,76px);grid-template-rows:repeat(3,76px);gap:10px;justify-content:center;margin:4px 0 14px;touch-action:none;user-select:none;-webkit-user-select:none}
.pb{border:1px solid var(--line);background:var(--card2);color:var(--text);border-radius:18px;font-size:28px;display:grid;place-items:center;cursor:pointer;touch-action:none;transition:transform .06s,background .1s}
.pb.on{background:var(--accent);border-color:var(--accent);transform:scale(.95);box-shadow:0 0 22px rgba(79,140,255,.45)}
.pb.mid{font-size:13px;font-weight:800;letter-spacing:.06em;color:var(--muted)}
.pb.mid:active{background:var(--line)}
.u{grid-area:1/2}.l{grid-area:2/1}.m{grid-area:2/2}.r{grid-area:2/3}.d{grid-area:3/2}
.hint{text-align:center;font-size:12px;color:var(--muted);margin-bottom:12px}
.row{padding:10px 0;border-top:1px solid var(--line)}.row:first-child{border-top:0}
.rl{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:8px}
.rn{font-weight:700}.rd{font-size:12px;color:var(--muted)}
.rc{display:flex;align-items:center;gap:8px}
input[type=range]{flex:1;min-width:0;accent-color:var(--accent);height:28px}
.num{width:76px;background:var(--bg);border:1px solid var(--line);border-radius:10px;color:var(--text);font-weight:600;font-size:15px;font-family:inherit;text-align:center;padding:8px 4px;-moz-appearance:textfield;font-variant-numeric:tabular-nums}
.num:focus{outline:none;border-color:var(--accent)}
.num.dirty{border-color:var(--warn);color:var(--warn)}
.num::-webkit-inner-spin-button,.num::-webkit-outer-spin-button{-webkit-appearance:none}
.sb{width:34px;height:34px;flex:none;border-radius:10px;border:1px solid var(--line);background:var(--card2);color:var(--text);font-size:18px;cursor:pointer;touch-action:manipulation}
.sb:active{background:var(--accent)}
details.card{padding:0}
details summary{list-style:none;cursor:pointer;padding:16px;display:flex;justify-content:space-between;align-items:center}
details summary::-webkit-details-marker{display:none}
details summary h2{margin:0}
details summary:after{content:"\25BE";color:var(--muted);transition:.2s}
details[open] summary:after{transform:rotate(180deg)}
details .inner{padding:0 16px 8px}
.btn{width:100%;border:0;border-radius:12px;padding:14px;font-weight:700;font-size:15px;font-family:inherit;cursor:pointer;color:#fff;background:var(--card2);border:1px solid var(--line)}
.btn:disabled{opacity:.5}
.btn.stop{background:var(--bad);border-color:var(--bad)}
.btn.go{background:var(--ok);border-color:var(--ok)}
.btn.blue{background:var(--accent);border-color:var(--accent)}
.sys{display:grid;gap:10px}
.note{font-size:12px;color:var(--muted);margin-top:-2px}
.res{font-size:13px;color:var(--accent2);min-height:1em}
.savebar{position:fixed;left:0;right:0;bottom:0;z-index:6;padding:12px 16px calc(12px + env(safe-area-inset-bottom));
background:rgba(18,24,38,.97);border-top:1px solid var(--line);display:flex;gap:10px;align-items:center;justify-content:center;
transform:translateY(110%);transition:transform .25s}
.savebar.show{transform:none}
.savebar span{flex:1;max-width:240px;color:var(--warn);font-weight:600;font-size:14px}
.savebar .btn{width:auto;padding:12px 16px}
.toast{position:fixed;left:50%;top:70px;transform:translate(-50%,-20px);opacity:0;z-index:9;background:var(--card2);border:1px solid var(--line);
padding:10px 16px;border-radius:12px;font-weight:600;transition:.25s;pointer-events:none}
.toast.show{opacity:1;transform:translate(-50%,0)}
@media(max-width:380px){.pad{grid-template-columns:repeat(3,68px);grid-template-rows:repeat(3,68px)}.big{font-size:36px}.stats{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body>
<header>
  <div class="brand"><div class="logo">&#9881;</div>Balance Bot</div>
  <div class="status"><span id="pill" class="pill">CONNECTING</span><span id="dot" class="dot"></span></div>
</header>

<main>
  <section class="card">
    <div class="live-top">
      <svg class="tilt" viewBox="-60 -72 120 90" aria-label="Tilt indicator (exaggerated)">
        <line class="ground" x1="-58" y1="12" x2="58" y2="12"/>
        <g id="tgtG"><line class="tline" x1="0" y1="6" x2="0" y2="-66"/></g>
        <g id="bot">
          <rect class="body" x="-10" y="-54" width="20" height="56" rx="6"/>
          <rect class="face" x="-6" y="-47" width="12" height="9" rx="3"/>
          <circle class="wheel" cx="0" cy="5" r="7"/>
        </g>
      </svg>
      <div class="pitchbox">
        <div class="lbl">Pitch</div>
        <div class="big" id="pitch">--</div>
        <div class="sub">target <span id="tgt">--</span>&deg;</div>
      </div>
    </div>
    <div class="stats">
      <div class="tile"><b id="spd">--</b><span>Speed t/s</span></div>
      <div class="tile"><b id="mot">--</b><span>Motor</span><div class="mbar"><i id="mbar"></i></div></div>
      <div class="tile"><b id="trim">--</b><span>Trim &deg;</span></div>
      <div class="tile"><b id="err">--</b><span>Error &deg;</span></div>
    </div>
    <div class="enc">Encoders &nbsp;L <span id="encL">--</span> &nbsp;&middot;&nbsp; R <span id="encR">--</span></div>
  </section>

  <section class="card">
    <h2>Drive</h2>
    <div class="pad" id="pad">
      <button class="pb u" data-dir="up" aria-label="Forward">&#9650;</button>
      <button class="pb l" data-dir="left" aria-label="Turn left">&#9664;</button>
      <button class="pb mid m" id="halt" aria-label="Stop driving">STOP</button>
      <button class="pb r" data-dir="right" aria-label="Turn right">&#9654;</button>
      <button class="pb d" data-dir="down" aria-label="Backward">&#9660;</button>
    </div>
    <div class="hint">Hold to drive &middot; release to stop &middot; arrow keys / WASD on a PC</div>
    <div id="driveRows"></div>
    <button class="btn stop" id="motorBtn" style="margin-top:12px">Stop motors</button>
  </section>

  <details class="card" open>
    <summary><h2>Balance</h2></summary>
    <div class="inner" id="balRows"></div>
  </details>

  <details class="card">
    <summary><h2>Anti-drift</h2></summary>
    <div class="inner"><div class="note" style="margin:0 0 4px">Uses the wheel encoders to stop rolling away and to learn slopes.</div><div id="adRows"></div></div>
  </details>

  <details class="card">
    <summary><h2>System</h2></summary>
    <div class="inner sys" style="padding-bottom:16px">
      <button class="btn blue" id="encBtn">Test encoders</button>
      <div class="note">Lift the robot first: both wheels spin forward briefly.</div>
      <div class="res" id="encRes"></div>
      <button class="btn" id="gyroBtn">Calibrate gyro</button>
      <div class="note">Robot must be completely still (any angle).</div>
      <div class="res" id="gyroRes"></div>
    </div>
  </details>
</main>

<div class="savebar" id="savebar">
  <span>Unsaved changes</span>
  <button class="btn" id="revertBtn">Revert</button>
  <button class="btn blue" id="applyBtn">Apply &amp; Save</button>
</div>
<div class="toast" id="toast"></div>

<script>
var P=[
 {id:'kp',g:'bal',n:'Kp',d:'Proportional',min:0,max:100,st:0.01},
 {id:'ki',g:'bal',n:'Ki',d:'Integral',min:0,max:10,st:0.01},
 {id:'kd',g:'bal',n:'Kd',d:'Damping (gyro)',min:0,max:20,st:0.01},
 {id:'sp',g:'bal',n:'Setpoint',d:'Balance angle (deg)',min:-15,max:15,st:0.01},
 {id:'db',g:'bal',n:'Deadband',d:'Min motor PWM',min:0,max:100,st:1},
 {id:'kv',g:'ad',n:'Kv',d:'Lean per speed',min:0,max:3,st:0.01},
 {id:'kx',g:'ad',n:'Kx',d:'Lean per distance',min:0,max:1,st:0.01},
 {id:'kt',g:'ad',n:'Auto-trim',d:'Slope learning rate',min:0,max:2,st:0.01},
 {id:'ds',g:'drv',n:'Drive speed',d:'ticks/s at full',min:50,max:1000,st:10},
 {id:'tp',g:'drv',n:'Turn power',d:'PWM difference',min:0,max:150,st:1}
];
var byId={},cur={},saved={};
function $(i){return document.getElementById(i)}
function dec(p){return p.st>=1?0:2}
function fmt(p,v){return Number(v).toFixed(dec(p))}
function toast(t){var e=$('toast');e.textContent=t;e.classList.add('show');clearTimeout(e._t);e._t=setTimeout(function(){e.classList.remove('show')},1800)}

function build(){
  var tgt={bal:$('balRows'),ad:$('adRows'),drv:$('driveRows')};
  P.forEach(function(p){
    byId[p.id]=p;
    var r=document.createElement('div');r.className='row';
    r.innerHTML='<div class="rl"><span class="rn">'+p.n+'</span><span class="rd">'+p.d+'</span></div>'+
      '<div class="rc"><button class="sb" data-id="'+p.id+'" data-d="-1">&minus;</button>'+
      '<input type="range" id="'+p.id+'_s" min="'+p.min+'" max="'+p.max+'" step="'+p.st+'">'+
      '<button class="sb" data-id="'+p.id+'" data-d="1">+</button>'+
      '<input type="number" class="num" id="'+p.id+'" min="'+p.min+'" max="'+p.max+'" step="'+p.st+'" inputmode="decimal"></div>';
    tgt[p.g].appendChild(r);
    $(p.id+'_s').addEventListener('input',function(e){setVal(p.id,e.target.value)});
    $(p.id).addEventListener('change',function(e){setVal(p.id,e.target.value)});
  });
  document.querySelectorAll('.sb').forEach(function(b){
    b.addEventListener('click',function(){var p=byId[b.dataset.id];setVal(p.id,cur[p.id]+p.st*b.dataset.d)});
  });
}
function setVal(id,v,isSaved){
  var p=byId[id];v=parseFloat(v);if(isNaN(v))v=cur[id];
  v=Math.min(p.max,Math.max(p.min,v));v=parseFloat(fmt(p,v));
  cur[id]=v;if(isSaved)saved[id]=v;
  $(id).value=fmt(p,v);$(id+'_s').value=v;
  $(id).classList.toggle('dirty',saved[id]!==v);
  checkDirty();
}
function checkDirty(){
  var d=P.some(function(p){return saved[p.id]!==cur[p.id]});
  $('savebar').classList.toggle('show',d);
}
function loadConfig(){
  fetch('/config').then(function(r){return r.json()}).then(function(c){
    P.forEach(function(p){if(c[p.id]!==undefined)setVal(p.id,c[p.id],true)});
  }).catch(function(){setTimeout(loadConfig,1500)});
}
$('applyBtn').onclick=function(){
  var q=P.map(function(p){return p.id+'='+cur[p.id]}).join('&');
  fetch('/set?'+q).then(function(r){if(!r.ok)throw 0;
    P.forEach(function(p){setVal(p.id,cur[p.id],true)});toast('Saved to robot');
  }).catch(function(){toast('Save failed - connected?')});
};
$('revertBtn').onclick=function(){P.forEach(function(p){setVal(p.id,saved[p.id])})};

/* ---------- Telemetry ---------- */
var lastOk=0,en=1,fl=0,polling=false;
function applyState(s){
  lastOk=Date.now();en=s.en;fl=s.fl;
  var e=s.p-s.t,rel=s.p-s.sp,trel=s.t-s.sp;
  $('pitch').textContent=s.p.toFixed(2)+'°';
  $('tgt').textContent=s.t.toFixed(2);
  $('spd').textContent=s.s;$('mot').textContent=s.m;
  $('trim').textContent=s.tr.toFixed(2);$('err').textContent=e.toFixed(2);
  $('encL').textContent=s.l;$('encR').textContent=s.r;
  var w=Math.min(50,Math.abs(s.m)/255*50),b=$('mbar');
  b.style.width=w+'%';b.style.left=(s.m<0?50-w:50)+'%';b.style.background=Math.abs(s.m)>230?'var(--bad)':'var(--accent)';
  var k=3,cl=function(a){return Math.max(-60,Math.min(60,a*k))};
  $('bot').setAttribute('transform','rotate('+cl(rel)+' 0 5)');
  $('tgtG').setAttribute('transform','rotate('+cl(trel)+' 0 5)');
  var mb=$('motorBtn');
  mb.textContent=en?'Stop motors':'Resume balancing';mb.className='btn '+(en?'stop':'go');
  pill();
}
function pill(){
  var p=$('pill'),on=Date.now()-lastOk<1500;
  $('dot').classList.toggle('on',on);
  if(!on){p.textContent='OFFLINE';p.className='pill bad';return}
  if(fl){p.textContent='FALLEN';p.className='pill bad'}
  else if(!en){p.textContent='STOPPED';p.className='pill warn'}
  else if(held.size){p.textContent='DRIVING';p.className='pill drive'}
  else{p.textContent='BALANCING';p.className='pill ok'}
}
function poll(){
  if(driveTimer||polling)return;polling=true;
  fetch('/state').then(function(r){return r.json()}).then(applyState).catch(function(){})
  .then(function(){polling=false});
}
setInterval(poll,200);setInterval(pill,500);

/* ---------- Driving ---------- */
var held=new Set(),driveTimer=null,inflight=false;
function fc(){return{f:(held.has('up')?1:0)-(held.has('down')?1:0),t:(held.has('right')?1:0)-(held.has('left')?1:0)}}
function sendDrive(){
  if(inflight)return;inflight=true;var c=fc();
  fetch('/drive?f='+c.f+'&t='+c.t).then(function(r){return r.json()}).then(applyState).catch(function(){})
  .then(function(){inflight=false});
}
function driveUpdate(){
  document.querySelectorAll('.pb[data-dir]').forEach(function(b){b.classList.toggle('on',held.has(b.dataset.dir))});
  if(held.size){if(!driveTimer){sendDrive();driveTimer=setInterval(sendDrive,100)}}
  else{if(driveTimer){clearInterval(driveTimer);driveTimer=null}fetch('/drive?f=0&t=0').catch(function(){})}
  pill();
}
document.querySelectorAll('.pb[data-dir]').forEach(function(b){
  var dir=b.dataset.dir;
  b.addEventListener('pointerdown',function(e){e.preventDefault();try{b.setPointerCapture(e.pointerId)}catch(x){}held.add(dir);driveUpdate()});
  ['pointerup','pointercancel','lostpointercapture'].forEach(function(ev){
    b.addEventListener(ev,function(){if(held.has(dir)){held.delete(dir);driveUpdate()}});
  });
  b.addEventListener('contextmenu',function(e){e.preventDefault()});
});
$('halt').addEventListener('pointerdown',function(e){e.preventDefault();held.clear();driveUpdate()});
var keys={ArrowUp:'up',w:'up',W:'up',ArrowDown:'down',s:'down',S:'down',ArrowLeft:'left',a:'left',A:'left',ArrowRight:'right',d:'right',D:'right'};
document.addEventListener('keydown',function(e){
  if(e.target.tagName==='INPUT')return;var d=keys[e.key];if(!d)return;
  e.preventDefault();if(!held.has(d)){held.add(d);driveUpdate()}
});
document.addEventListener('keyup',function(e){var d=keys[e.key];if(d&&held.has(d)){held.delete(d);driveUpdate()}});
function releaseAll(){if(held.size){held.clear();driveUpdate()}}
window.addEventListener('blur',releaseAll);
document.addEventListener('visibilitychange',function(){if(document.hidden)releaseAll()});

/* ---------- System ---------- */
$('motorBtn').onclick=function(){
  releaseAll();
  fetch('/stop?state='+(en?0:1)).then(function(){en=en?0:1;toast(en?'Balancing resumed':'Motors stopped')});
};
$('encBtn').onclick=function(){
  if(!confirm('Lift the robot so the wheels spin freely. Run encoder test?'))return;
  var b=this;b.disabled=true;$('encRes').textContent='Testing...';
  fetch('/enctest').then(function(r){return r.text()}).then(function(t){$('encRes').textContent=t})
  .catch(function(){$('encRes').textContent='Test failed'}).then(function(){b.disabled=false});
};
$('gyroBtn').onclick=function(){
  if(!confirm('Keep the robot completely still. Calibrate gyro now?'))return;
  var b=this;b.disabled=true;$('gyroRes').textContent='Calibrating - keep still...';
  fetch('/calgyro').then(function(){$('gyroRes').textContent='Gyro calibrated and saved'})
  .catch(function(){$('gyroRes').textContent='Calibration failed'}).then(function(){b.disabled=false});
};

build();loadConfig();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", PAGE_HTML);
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
  if (server.hasArg("ds")) driveSpeed = constrain(server.arg("ds").toFloat(), 0.0f, 1500.0f);
  if (server.hasArg("tp")) turnPower  = constrain(server.arg("tp").toFloat(), 0.0f, 255.0f);

  prefs.putFloat("v2_kp", Kp);
  prefs.putFloat("v2_ki", Ki);
  prefs.putFloat("v2_kd", Kd);
  prefs.putFloat("v2_sp", setpoint);
  prefs.putFloat("v2_db", deadband);
  prefs.putFloat("v2_kv", Kv);
  prefs.putFloat("v2_kx", Kx);
  prefs.putFloat("v2_kt", Kt);
  prefs.putFloat("v2_ds", driveSpeed);
  prefs.putFloat("v2_tp", turnPower);
  server.send(200, "text/plain", "OK");
}

String stateJson() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"p\":%.2f,\"t\":%.2f,\"sp\":%.2f,\"s\":%d,\"m\":%d,\"tr\":%.2f,\"l\":%ld,\"r\":%ld,\"en\":%d,\"fl\":%d}",
    (float)pitch, (float)dynamicSetpoint, (float)setpoint, (int)filteredSpeed, (int)outputPWM,
    (float)trim, (long)leftTicks, (long)rightTicks, motorEnabled ? 1 : 0, robotFallen ? 1 : 0);
  return String(buf);
}

void handleState() {
  server.send(200, "application/json", stateJson());
}

void handleConfig() {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"sp\":%.2f,\"db\":%.2f,\"kv\":%.2f,\"kx\":%.2f,\"kt\":%.2f,\"ds\":%.0f,\"tp\":%.0f}",
    (float)Kp, (float)Ki, (float)Kd, (float)setpoint, (float)deadband, (float)Kv, (float)Kx, (float)Kt,
    (float)driveSpeed, (float)turnPower);
  server.send(200, "application/json", String(buf));
}

// Hold-to-drive: the page repeats this every ~100 ms while a button is held
void handleDrive() {
  float f = server.hasArg("f") ? server.arg("f").toFloat() : 0.0f;
  float t = server.hasArg("t") ? server.arg("t").toFloat() : 0.0f;
  driveCmd = constrain(f, -1.0f, 1.0f);
  turnCmd  = constrain(t, -1.0f, 1.0f);
  lastDriveMs = millis();
  server.send(200, "application/json", stateJson());
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
  speedTarget = 0;
  driveCmd = 0;
  turnCmd = 0;
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
  driveSpeed = prefs.getFloat("v2_ds", 300.0f);
  turnPower  = prefs.getFloat("v2_tp", 40.0f);
  dynamicSetpoint = setpoint;

  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/",        handleRoot);
  server.on("/set",     handleSet);
  server.on("/state",   handleState);
  server.on("/pitch",   handleState);   // old name, kept as alias
  server.on("/config",  handleConfig);
  server.on("/drive",   handleDrive);
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

  // Drive failsafe: released button, closed tab or lost WiFi -> stop driving
  if (millis() - lastDriveMs > DRIVE_TIMEOUT_MS) { driveCmd = 0; turnCmd = 0; }

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

    // Ramp the commanded speed so starts/stops are smooth
    float want = DRIVE_SIGN * driveCmd * driveSpeed;
    float maxStep = DRIVE_ACCEL * sdt;
    speedTarget += constrain(want - speedTarget, -maxStep, maxStep);
    bool driving = fabs(speedTarget) > 1.0f || turnCmd != 0.0f;

    float kx = Kx, kv = Kv;
    if (kx > 0.0001f && !driving) {        // while driving, don't pull back to the old spot
      posTicks += deltaAvg;
      float lim = MAX_TILT_OFFSET * 100.0f / kx;       // anti-windup: position term alone can't exceed max lean
      posTicks = constrain(posTicks, -lim, lim);
    } else {
      posTicks = 0;
    }

    // Faster than target (+) -> lean back (lower target angle), and vice versa.
    // With speedTarget = 0 this is the plain anti-drift loop.
    float target = (n > 0) ? -(kv * (filteredSpeed - speedTarget) + kx * posTicks) / 100.0f : 0.0f;
    target = constrain(target, -MAX_TILT_OFFSET, MAX_TILT_OFFSET);
    tiltOffset += 0.4f * (target - tiltOffset);        // smooth

    // Auto-trim: a lean that is requested persistently means the real balance angle has moved
    // (slope, battery, load). Fold it into the setpoint so the robot re-learns its equilibrium.
    // (only while the balance loop is in control, so a fall/runaway can't wind the trim up)
    // (paused while driving: a lean to accelerate is not a slope)
    if (!driving && fabs(pitch - dynamicSetpoint) < 6.0f) trim += Kt * tiltOffset * sdt;
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

  // Turning: add a left/right difference on top of the balance command
  float turn = TURN_SIGN * turnCmd * turnPower;
  setMotors((int)(cmd + turn), (int)(cmd - turn));

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
