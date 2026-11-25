/*
 * ROBOT LAU NHÀ TỰ ĐỘNG - ESP32 (BẢN TỐI ƯU: WiFi + HC-05 + OTA + Non-blocking US + Soft-Start)
 * ---------------------------------------------------------------------------------------------
 * Mở trình duyệt (điện thoại hoặc máy tính cùng WiFi) vào  http://<IP của ESP32>
 * (hoặc http://robot.local). IP được in ra Serial Monitor 115200 khi khởi động.
 * Nếu không vào được WiFi nhà, ESP32 tự phát WiFi riêng:
 *      Tên: RobotLauNha   Mật khẩu: 88888888   ->  http://192.168.4.1
 *
 * Trang web có: nút Bắt đầu / Dừng lau tự động, bật/tắt chổi, phím điều khiển tay,
 * thanh chỉnh tốc độ, radar LiDAR, bản đồ phòng có vệt di chuyển và chế độ nhả chân D0/D1
 * để nạp lại Arduino Uno qua USB mà không cần tháo mối hàn.
 *
 * API (có thể gọi từ app khác / Android Studio):
 *   GET /api/status                 -> JSON trạng thái
 *   GET /api/scan                   -> JSON 360 giá trị khoảng cách LiDAR (mm)
 *   GET /api/scan.bin               -> Binary 720 byte (360 x uint16_t LE) cho Mobile App
 *   GET /api/plan                   -> JSON trục làn, danh sách làn, edge, perim
 *   GET /api/settings[?mop=&cruise=&edge=&perim=] -> đọc / lưu cài đặt vào Flash
 *   GET /api/cmd?c=start|stop|F|B|L|R|S|mopon|mopoff[&v=80..210]
 *   GET /api/unomode?en=1|0         -> bật/tắt chế độ High-Z GPIO18/19 để nạp Uno qua USB
 *
 * Đấu dây ESP32 (giữ nguyên 100% phần cứng đã hàn):
 *   LD14 TX -> GPIO16 | VCC 5V | GND
 *   ESP32 GPIO19 (TX) -> Arduino RX (chân 0)
 *   ESP32 GPIO18 (RX) <- Arduino TX (chân 1) QUA CẦU PHÂN ÁP 5V->3.3V (1k/2k)
 *   GND chung. Arduino chuyển tiếp lệnh HC-05 sang ESP32 và gửi trạng thái lên điện thoại.
 *   HC-05 (nối với Arduino): A = lau tự động, X = dừng, F/B/L/R = điều khiển tay, W/w = chổi
 *   Siêu âm (ECHO qua cầu phân áp 1k/2k):
 *       Trước TRIG 25/ECHO 26 | Trái TRIG 27/ECHO 14 | Phải TRIG 32/ECHO 33
 *
 * Quy ước góc LiDAR: 0 = trước, tăng theo chiều kim đồng hồ.
 * Hệ toạ độ phòng: gốc ở góc sau-trái, +Y = hướng robot lúc bắt đầu, +X = bên phải.
 * psi: 0 = +Y, 90 = +X, thuận chiều kim đồng hồ.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <vector>
#include <math.h>

// Khai báo kiểu dữ liệu trước mọi hàm để Arduino IDE sinh prototype chính xác
struct Wall { bool ok; float dist; float phi; };
enum State { ST_IDLE, ST_CALIB, ST_RUN, ST_DONE, ST_ERR };
enum StepType { GO_X, GO_Y };
enum StepRole { ROLE_HOME, ROLE_LANE, ROLE_SHIFT, ROLE_RECOVER, ROLE_PERIM };
struct Step { StepType type; float val; StepRole role; };
enum Phase { PH_TURN, PH_DRIVE, PH_BACKOFF };

// ======================= WIFI =======================
#define WIFI_SSID  "Ngoc Tram_2G"
#define WIFI_PASS  "trungkien"
#define AP_SSID    "RobotLauNha"
#define AP_PASS    "88888888"

// ======================= CẤU HÌNH PHẦN CỨNG =======================
#define LIDAR_RX_PIN   16
#define LINK_TX_PIN    19
#define LINK_RX_PIN    18
const int US_TRIG[3] = {25, 27, 32};   // 0 trước, 1 trái, 2 phải
const int US_ECHO[3] = {26, 14, 33};

#define LIDAR_BAUD         230400
#define LINK_BAUD          38400
#define LIDAR_OFFSET_DEG   0
#define LIDAR_CLOCKWISE    true
#define LIDAR_MIN_MM       100
#define LIDAR_MAX_MM       8000
#define LIDAR_MIN_INTENS   10

// ======================= THÔNG SỐ ROBOT & BẢO VỆ L293D =======================
#define ROBOT_RADIUS_MM    170
#define DEF_EDGE_MM        (ROBOT_RADIUS_MM + 40)
#define DEF_MOP_WIDTH_MM   180.0f
#define V_MAX_MM_S         300.0f
#define SPIN_MAX_DEG_S     150.0f

// Giới hạn trần PWM & bước tăng tốc mềm (mỗi 50 ms) để giảm dòng đỉnh bảo vệ L293D
#define MAX_SAFE_PWM       210
#define DEF_CRUISE_PWM     165
#define SLOW_PWM           110
#define MIN_TURN_PWM       120
#define MAX_TURN_PWM       175
#define MOP_PWM            200
#define PWM_RAMP_STEP      30    // Tăng/giảm tối đa 30 PWM / 50 ms (~0.3s từ 0 -> 170)
#define PWM_STOP_STEP      90    // Giảm tốc nhanh hơn khi về 0

#define KT                 3.0f
#define KH                 2.5f
#define TURN_TOL_DEG       3.0f
#define K_YAW              0.8f
#define K_POS              0.5f
#define POS_JUMP_MAX_MM    300.0f

#define US_BLOCK_MM        55
#define LIDAR_BLOCK_MM     (ROBOT_RADIUS_MM + 25)
#define SIDE_NEAR_MM       60
#define WALL_HALF_DEG      25
#define US_TIMEOUT_MS      14    // ~2.4 m, không block CPU
#define US_PERIOD_MS       30
#define BLOCK_CONFIRM_CNT  3     // 3 chu kỳ dữ liệu cảm biến thực liên tiếp
#define LANE_OVERLAP_RATIO 0.15f // Chồng lấn tối thiểu 15% giữa 2 làn liên tiếp
#define RECOVER_MIN_REM_MM 450.0f // Chèn bước bọc lại nếu phần làn bỏ sót > 450 mm
#define MAX_PLAN_STEPS     200

// ======================= BIẾN TOÀN CỤC =======================
WebServer server(80);
State state = ST_IDLE;

std::vector<Step> steps;
std::vector<float> planLanes;
bool planAxisY = true;           // true: quét làn dọc trục Y, false: dọc trục X
int  stepIdx = 0;
bool stepStarted = false;
Phase phase = PH_TURN;
float tgtHeading = 0;
unsigned long stepStartMs = 0;
unsigned long backoffUntilMs = 0;
uint8_t stepRetries = 0;

float px_mm = 0, py_mm = 0, psi = 0;
float roomW = 0, roomL = 0;
bool  haveRoom = false;
unsigned long lastPoseMs = 0;

// LiDAR
uint16_t acc[360];
uint16_t scan[360];
volatile bool newScan = false;
float lastStartAngle = 0;
unsigned long lastScanMs = 0;
uint8_t crcTable[256];

// Siêu âm Non-blocking (ngắt ngoài + lọc trung vị 3 mẫu)
volatile uint32_t usRiseUs[3]  = {0, 0, 0};
volatile uint32_t usWidthUs[3] = {0, 0, 0};
volatile bool     usHigh[3]    = {false, false, false};
volatile bool     usDone[3]    = {false, false, false};

uint16_t us[3] = {9999, 9999, 9999};
uint16_t usHist[3][3] = {{9999, 9999, 9999}, {9999, 9999, 9999}, {9999, 9999, 9999}};
uint8_t  usHistIdx[3] = {0, 0, 0};
int      usTurn = 0;
bool     usBusy = false;
unsigned long lastUsMs = 0;
unsigned long usTrigMs = 0;
bool     newUsFront = false;

// Điều khiển động cơ (target vs output thực tế qua soft-start)
int cmdL = 0, cmdR = 0;
int outL = 0, outR = 0;
int mopPwm = 0, lastSentMop = -1;
unsigned long lastSendMs = 0, lastMopMs = 0, lastTelMs = 0;
int blockCount = 0;

// Hiệu chỉnh phòng
int calibOk = 0, calibFail = 0;
float calibD[4] = {0, 0, 0, 0};
float calibSin = 0, calibCos = 0;

// Điều khiển tay
unsigned long manualUntil = 0;
int manualL = 0, manualR = 0;

// Chế độ nhả chân GPIO18/19 (High-Z) để nạp code Arduino Uno qua USB
bool unoFlashMode = false;

String errMsg = "";

// Cài đặt lưu trong Flash (NVS)
Preferences prefs;
float cfgMopWidth = DEF_MOP_WIDTH_MM;
int   cfgCruise   = DEF_CRUISE_PWM;
int   cfgEdge     = DEF_EDGE_MM;
bool  cfgPerim    = true;        // Lau viền 2 cạnh đầu/cuối làn sau khi quét xong
String myIP = "";

// ======================= TRANG WEB =======================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Robot lau nha</title>
<style>
body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#eee;padding:10px;touch-action:manipulation}
h1{font-size:18px;margin:0 0 8px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0;align-items:center}
button{background:#2a2a2a;color:#eee;border:1px solid #444;border-radius:10px;padding:14px 18px;font-size:16px;user-select:none;-webkit-user-select:none;touch-action:none}
button.go{background:#1b7a3a}button.stop{background:#b3261e}button.warn{background:#8a5a00}
.pad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,60px);gap:6px;justify-content:center;margin:10px 0}
.pad button{padding:0}
canvas{background:#000;border:1px solid #333;border-radius:8px;max-width:100%}
#st{font-family:monospace;font-size:13px;line-height:1.5;background:#1b1b1b;padding:8px;border-radius:8px;white-space:pre}
.cv{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}
</style></head><body>
<h1>Robot lau nhà</h1>
<div id="st">Đang kết nối...</div>
<div style="background:#222;border-radius:6px;height:10px;margin:6px 0"><div id="bar" style="background:#1b7a3a;height:10px;border-radius:6px;width:0%"></div></div>
<div class="row">
<button class="go" id="bStart">▶ Bắt đầu lau</button>
<button class="stop" id="bStop">■ Dừng</button>
<button id="bMop">Chổi: TẮT</button>
</div>
<div class="row">Tốc độ tay: <input type="range" id="spd" min="100" max="210" value="165"> <span id="spv">165</span></div>
<div class="pad">
<div></div><button id="bF">▲</button><div></div>
<button id="bL">◀</button><button id="bS">■</button><button id="bR">▶</button>
<div></div><button id="bB">▼</button><div></div>
</div>
<details style="margin:10px 0"><summary>Cài đặt & Bảo trì</summary>
<div class="row">Bề rộng vệt lau (mm) <input id="sMop" type="number" min="100" max="400" style="width:80px"></div>
<div class="row">Tốc độ lau (PWM 100-210) <input id="sCr" type="number" min="100" max="210" style="width:80px"></div>
<div class="row">Cách tường (mm) <input id="sEd" type="number" min="180" max="500" style="width:80px"></div>
<div class="row"><label><input id="sPer" type="checkbox" checked> Lau viền 2 đầu sau khi quét làn</label></div>
<div class="row"><button id="bSave">Lưu</button><span id="sMsg"></span></div>
<hr style="border-color:#333">
<div class="row"><button class="warn" id="bUno">Nạp Arduino Uno (Nhả D0/D1): TẮT</button></div>
<div style="font-size:12px;color:#aaa">Bật chế độ này khi cần cắm USB nạp code cho Arduino Uno mà không phải tháo dây hàn D0/D1 (giữ ESP32 có nguồn).</div>
</details>
<div class="cv">
<div><div>Radar LiDAR (4 m)</div><canvas id="radar" width="300" height="300"></canvas></div>
<div><div>Bản đồ phòng</div><canvas id="map" width="300" height="340"></canvas></div>
</div>
<script>
const $=id=>document.getElementById(id);
function cmd(c,v){return fetch('/api/cmd?c='+c+(v?'&v='+v:'')).catch(()=>{})}
$('bStart').onclick=()=>{trail=[];lanes=[];cmd('start')};
$('bStop').onclick=()=>cmd('stop');
$('bS').onclick=()=>cmd('S');
let mopOn=false,unoMode=false;
$('bMop').onclick=()=>{mopOn=!mopOn;cmd(mopOn?'mopon':'mopoff');$('bMop').textContent='Chổi: '+(mopOn?'BẬT':'TẮT')};
$('bUno').onclick=async()=>{
  const r=await fetch('/api/unomode?en='+(unoMode?0:1));
  if(!r.ok){alert('Hãy dừng robot trước khi bật chế độ nạp Uno');return}
  unoMode=!unoMode;
  $('bUno').textContent='Nạp Arduino Uno (Nhả D0/D1): '+(unoMode?'BẬT':'TẮT');
};
$('spd').oninput=()=>{$('spv').textContent=$('spd').value};
function hold(el,c){
  let timer=null;
  const send=()=>cmd(c,$('spd').value);
  const down=e=>{e.preventDefault();if(timer)return;send();timer=setInterval(send,150)};
  const up=()=>{if(timer){clearInterval(timer);timer=null;cmd('S')}};
  el.addEventListener('pointerdown',down);
  ['pointerup','pointerleave','pointercancel'].forEach(ev=>el.addEventListener(ev,up));
}
hold($('bF'),'F');hold($('bB'),'B');hold($('bL'),'L');hold($('bR'),'R');

let trail=[],lanes=[],planEdge=210,planAxis='Y',planPerim=1;
function drawRadar(a){
  const c=$('radar'),g=c.getContext('2d'),w=c.width,h=c.height;
  g.fillStyle='#000';g.fillRect(0,0,w,h);
  const sc=(w/2-6)/4000;
  g.strokeStyle='#222';
  for(let r=1000;r<=4000;r+=1000){g.beginPath();g.arc(w/2,h/2,r*sc,0,7);g.stroke()}
  g.fillStyle='#0f0';
  for(let i=0;i<360;i++){
    const d=a[i];if(!d)continue;
    const t=i*Math.PI/180;
    g.fillRect(w/2+d*Math.sin(t)*sc-1,h/2-d*Math.cos(t)*sc-1,2,2);
  }
  g.fillStyle='#f55';g.beginPath();g.arc(w/2,h/2,4,0,7);g.fill();
  g.strokeStyle='#f55';g.beginPath();g.moveTo(w/2,h/2);g.lineTo(w/2,h/2-16);g.stroke();
}
function drawMap(s){
  const c=$('map'),g=c.getContext('2d'),w=c.width,h=c.height;
  g.fillStyle='#000';g.fillRect(0,0,w,h);
  if(!s.W||!s.L){g.fillStyle='#888';g.fillText('Chưa đo phòng',10,20);return}
  const m=10,sc=Math.min((w-2*m)/s.W,(h-2*m)/s.L);
  const X=x=>m+x*sc,Y=y=>h-m-y*sc;
  g.strokeStyle='#4af';g.strokeRect(X(0),Y(s.L),s.W*sc,s.L*sc);
  if(planPerim){
    g.strokeStyle='#536';
    g.strokeRect(X(planEdge),Y(s.L-planEdge),(s.W-2*planEdge)*sc,(s.L-2*planEdge)*sc);
  }
  g.strokeStyle='#335';
  lanes.forEach(v=>{
    g.beginPath();
    if(planAxis==='X'){g.moveTo(X(planEdge),Y(v));g.lineTo(X(s.W-planEdge),Y(v))}
    else{g.moveTo(X(v),Y(planEdge));g.lineTo(X(v),Y(s.L-planEdge))}
    g.stroke();
  });
  if(s.state==='RUN'||s.state==='IDLE'||s.state==='DONE'){
    trail.push([s.x,s.y]);if(trail.length>800)trail.shift();
  }
  g.strokeStyle='#2a6';g.beginPath();
  trail.forEach((p,i)=>{i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1]))});
  g.stroke();
  const t=s.psi*Math.PI/180;
  g.fillStyle='#f55';g.beginPath();g.arc(X(s.x),Y(s.y),5,0,7);g.fill();
  g.strokeStyle='#f55';g.beginPath();g.moveTo(X(s.x),Y(s.y));
  g.lineTo(X(s.x)+16*Math.sin(t),Y(s.y)-16*Math.cos(t));g.stroke();
}
async function poll(){
  try{
    const s=await (await fetch('/api/status')).json();
    $('st').textContent='Trạng thái: '+s.state+(s.uno?' [NHẢ D0/D1 NẠP UNO]':'')+
      '\nVị trí: x='+s.x+' y='+s.y+' mm, góc='+s.psi+'° | PWM: '+s.oL+'/'+s.oR+
      '\nPhòng: '+s.W+' x '+s.L+' mm | Bước: '+s.step+'/'+s.total+
      '\nSiêu âm Trước/Trái/Phải: '+s.us0+'/'+s.us1+'/'+s.us2+' mm'+(s.err?'\nLỗi: '+s.err:'');
    mopOn=s.mop>0;$('bMop').textContent='Chổi: '+(mopOn?'BẬT':'TẮT');
    unoMode=!!s.uno;$('bUno').textContent='Nạp Arduino Uno (Nhả D0/D1): '+(unoMode?'BẬT':'TẮT');
    $('bar').style.width=(s.total?Math.round(100*s.step/s.total):0)+'%';
    if(s.state==='RUN'&&!lanes.length){const p=await (await fetch('/api/plan')).json();lanes=p.lanes;planEdge=p.edge;planAxis=p.axis||'Y';planPerim=p.perim||0}
    drawMap(s);
    const a=await (await fetch('/api/scan')).json();
    drawRadar(a);
  }catch(e){$('st').textContent='Mất kết nối với robot...'}
  finally{setTimeout(poll,500)}
}
async function loadSet(){try{const s=await (await fetch('/api/settings')).json();$('sMop').value=s.mop;$('sCr').value=s.cruise;$('sEd').value=s.edge;$('sPer').checked=!!s.perim}catch(e){}}
$('bSave').onclick=async()=>{const r=await fetch('/api/settings?mop='+$('sMop').value+'&cruise='+$('sCr').value+'&edge='+$('sEd').value+'&perim='+($('sPer').checked?1:0));$('sMsg').textContent=r.ok?' Đã lưu':' Không lưu được (robot đang chạy)';loadSet()};
loadSet();
poll();
</script></body></html>
)rawliteral";

// ======================= TIỆN ÍCH =======================
float wrap180(float a) { while (a > 180.0f) a -= 360.0f; while (a <= -180.0f) a += 360.0f; return a; }
float wrap90(float a)  { a = fmodf(a, 90.0f); if (a > 45.0f) a -= 90.0f; if (a < -45.0f) a += 90.0f; return a; }
int   clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

void drive(int l, int r) {
  cmdL = clampi(l, -MAX_SAFE_PWM, MAX_SAFE_PWM);
  cmdR = clampi(r, -MAX_SAFE_PWM, MAX_SAFE_PWM);
}

void stopMotors() { cmdL = cmdR = 0; }

// Dừng khẩn cấp: đưa output về 0 ngay lập tức (dùng khi va chạm, stopAll, OTA)
void emergencyStopMotors() {
  cmdL = cmdR = 0;
  outL = outR = 0;
  if (!unoFlashMode) {
    Serial1.print("M,0,0\n");
    lastSendMs = millis();
  }
}

// Tăng/giảm tốc mềm & dead-time đảo chiều 50 ms để giảm dòng đỉnh trên L293D
int rampAxis(int cur, int tgt) {
  if (tgt == 0) {
    if (abs(cur) <= PWM_STOP_STEP) return 0;
    return cur + (cur > 0 ? -PWM_STOP_STEP : PWM_STOP_STEP);
  }
  if ((cur > 0 && tgt < 0) || (cur < 0 && tgt > 0)) {
    return 0;  // Ép qua 0 tối thiểu 1 chu kỳ (50 ms) trước khi đảo chiều
  }
  int diff = tgt - cur;
  if (diff > PWM_RAMP_STEP)  return cur + PWM_RAMP_STEP;
  if (diff < -PWM_RAMP_STEP) return cur - PWM_RAMP_STEP;
  return tgt;
}

// ======================= CHẾ ĐỘ NHẢ CHÂN NẠP UNO =======================
void setUnoFlashMode(bool en) {
  if (en == unoFlashMode) return;
  if (en) {
    cmdL = cmdR = outL = outR = 0;
    mopPwm = 0;
    manualUntil = 0;
    Serial1.print("M,0,0\nW,0\n");
    Serial1.flush();
    delay(20);
    Serial1.end();
    pinMode(LINK_TX_PIN, INPUT);
    pinMode(LINK_RX_PIN, INPUT);
    unoFlashMode = true;
    Serial.println("UNO FLASH MODE: ON (GPIO18/19 = High-Z)");
  } else {
    Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    unoFlashMode = false;
    lastSentMop = -1;
    Serial.println("UNO FLASH MODE: OFF (UART1 resumed)");
  }
}

// ======================= LiDAR LD14 =======================
void initCrc() {
  for (int i = 0; i < 256; i++) {
    uint8_t c = (uint8_t)i;
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
    crcTable[i] = c;
  }
}

uint8_t calcCrc(const uint8_t* p, int len) {
  uint8_t c = 0;
  for (int i = 0; i < len; i++) c = crcTable[(c ^ p[i]) & 0xFF];
  return c;
}

void handlePacket(const uint8_t* b) {
  float startA = (b[4] | (b[5] << 8)) / 100.0f;
  float endA   = (b[42] | (b[43] << 8)) / 100.0f;

  if (startA < lastStartAngle - 180.0f) {
    memcpy(scan, acc, sizeof(acc));
    memset(acc, 0, sizeof(acc));
    newScan = true;
    lastScanMs = millis();
  }
  lastStartAngle = startA;

  if (endA < startA) endA += 360.0f;
  float step = (endA - startA) / 11.0f;

  for (int i = 0; i < 12; i++) {
    uint16_t d = b[6 + 3 * i] | (b[7 + 3 * i] << 8);
    uint8_t inten = b[8 + 3 * i];
    if (d < LIDAR_MIN_MM || d > LIDAR_MAX_MM || inten < LIDAR_MIN_INTENS) continue;
    float ang = startA + step * i;
    int deg = ((int)(ang + 0.5f) + LIDAR_OFFSET_DEG) % 360;
    if (deg < 0) deg += 360;
    if (!LIDAR_CLOCKWISE) deg = (360 - deg) % 360;
    acc[deg] = d;
  }
}

void pollLidar() {
  static uint8_t buf[47];
  static int idx = 0;
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    if (idx == 0) { if (b != 0x54) continue; }
    else if (idx == 1) { if (b != 0x2C) { idx = 0; continue; } }
    buf[idx++] = b;
    if (idx == 47) {
      idx = 0;
      if (calcCrc(buf, 46) == buf[46]) handlePacket(buf);
    }
  }
}

// ======================= KHỚP TƯỜNG (PCA / TLS) =======================
Wall fitWall(int center) {
  Wall w = {false, 0, 0};
  float X[64], Y[64];
  int n = 0;
  for (int a = -WALL_HALF_DEG; a <= WALL_HALF_DEG && n < 64; a++) {
    uint16_t d = scan[((center + a) % 360 + 360) % 360];
    if (!d) continue;
    float r = radians((float)(center + a));
    X[n] = d * sinf(r);
    Y[n] = d * cosf(r);
    n++;
  }
  if (n < 10) return w;
  float mx = 0, my = 0;
  for (int i = 0; i < n; i++) { mx += X[i]; my += Y[i]; }
  mx /= n; my /= n;
  float sxx = 0, syy = 0, sxy = 0;
  for (int i = 0; i < n; i++) {
    float dx = X[i] - mx, dy = Y[i] - my;
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
  }
  float phi = 0.5f * atan2f(2.0f * sxy, sxx - syy);
  float nx = -sinf(phi), ny = cosf(phi);
  float res = 0;
  for (int i = 0; i < n; i++) {
    float e = (X[i] - mx) * nx + (Y[i] - my) * ny;
    res += e * e;
  }
  float rms = sqrtf(res / n);
  if (rms > 25.0f) return w;
  w.ok = true;
  w.dist = fabsf(mx * nx + my * ny);
  w.phi = degrees(phi);
  return w;
}

// ======================= ĐỊNH VỊ =======================
void predictPose() {
  unsigned long now = millis();
  float dt = (now - lastPoseMs) / 1000.0f;
  lastPoseMs = now;
  if (dt <= 0 || dt > 0.5f) return;
  // Dùng outL/outR (PWM thực tế sau ramp) để ước lượng động học sát hơn
  float v = (outL + outR) / 510.0f * V_MAX_MM_S;
  float w = (outL - outR) / 510.0f * SPIN_MAX_DEG_S;
  float mid = psi + 0.5f * w * dt;
  px_mm += v * dt * sinf(radians(mid));
  py_mm += v * dt * cosf(radians(mid));
  psi = wrap180(psi + w * dt);
}

void correctPose() {
  const int al[4] = {0, 90, 180, 270};
  Wall w[4];
  float sx = 0, sy = 0;
  int n = 0;
  for (int k = 0; k < 4; k++) {
    w[k] = fitWall(al[k]);
    if (w[k].ok) {
      float e = wrap90(w[k].phi - 90.0f);
      sx += sinf(radians(4.0f * e));
      sy += cosf(radians(4.0f * e));
      n++;
    }
  }
  if (n == 0) return;
  float eps = degrees(atan2f(sx, sy)) / 4.0f;
  float meas = psi + wrap90(eps - psi);
  psi = wrap180(psi + K_YAW * (meas - psi));

  if (!haveRoom) return;
  float psq = roundf(psi / 90.0f) * 90.0f;
  if (fabsf(psi - psq) > 12.0f) return;

  float xs = 0, ys = 0;
  int nx = 0, ny = 0;
  for (int k = 0; k < 4; k++) {
    if (!w[k].ok) continue;
    int q = ((al[k] + (int)psq) % 360 + 360) % 360;
    float d = w[k].dist;
    if      (q == 0)   { ys += roomL - d; ny++; }
    else if (q == 90)  { xs += roomW - d; nx++; }
    else if (q == 180) { ys += d;         ny++; }
    else               { xs += d;         nx++; }
  }
  if (nx) {
    float mxv = xs / nx;
    if (fabsf(mxv - px_mm) < POS_JUMP_MAX_MM) px_mm += K_POS * (mxv - px_mm);
  }
  if (ny) {
    float myv = ys / ny;
    if (fabsf(myv - py_mm) < POS_JUMP_MAX_MM) py_mm += K_POS * (myv - py_mm);
  }
}

// ======================= HIỆU CHỈNH BAN ĐẦU & LẬP ĐƯỜNG =======================
void buildPlan();

void setError(const char* m) {
  errMsg = m;
  state = ST_ERR;
  mopPwm = 0;
  emergencyStopMotors();
}

void doCalibrate() {
  const int al[4] = {0, 90, 180, 270};
  Wall w[4];
  bool all = true;
  for (int k = 0; k < 4; k++) { w[k] = fitWall(al[k]); if (!w[k].ok) all = false; }
  if (!all) {
    if (++calibFail > 60) setError("Khong thay du 4 buc tuong");
    return;
  }
  float sx = 0, sy = 0;
  for (int k = 0; k < 4; k++) {
    float e = wrap90(w[k].phi - 90.0f);
    sx += sinf(radians(4.0f * e)); sy += cosf(radians(4.0f * e));
    calibD[k] += w[k].dist;
  }
  calibSin += sx; calibCos += sy;
  if (++calibOk < 5) return;

  float eps = degrees(atan2f(calibSin, calibCos)) / 4.0f;
  float front = calibD[0] / 5.0f, right = calibD[1] / 5.0f, back = calibD[2] / 5.0f, left = calibD[3] / 5.0f;
  psi = eps;
  roomW = left + right;
  roomL = front + back;
  px_mm = left;
  py_mm = back;
  haveRoom = true;
  if (roomW < 2 * cfgEdge + 200 || roomL < 2 * cfgEdge + 200) { setError("Phong qua nho"); return; }
  buildPlan();
  stepIdx = 0; stepStarted = false; stepRetries = 0;
  mopPwm = MOP_PWM;
  state = ST_RUN;
  Serial.printf("Phong: %.0f x %.0f mm, %d buoc\n", roomW, roomL, (int)steps.size());
}

void buildPlan() {
  steps.clear();
  planLanes.clear();

  // 1. Tự động chọn trục làn song song cạnh dài hơn để tối thiểu số lần quay 90°
  planAxisY = (roomL >= roomW);
  StepType longType  = planAxisY ? GO_Y : GO_X;
  StepType crossType = planAxisY ? GO_X : GO_Y;

  float longMin  = (float)cfgEdge;
  float longMax  = (planAxisY ? roomL : roomW) - (float)cfgEdge;
  float crossMin = (float)cfgEdge;
  float crossMax = (planAxisY ? roomW : roomL) - (float)cfgEdge;
  float longPos  = planAxisY ? py_mm : px_mm;
  float crossPos = planAxisY ? px_mm : py_mm;

  // 2. Chia làn đều với độ chồng lấn tối thiểu 15% (bù sai số trượt bánh)
  float span = crossMax - crossMin;
  float effWidth = clampf(cfgMopWidth * (1.0f - LANE_OVERLAP_RATIO), 60.0f, 400.0f);
  int N = (int)ceilf(span / effWidth) + 1;
  if (N < 2) N = 2;
  float spacing = span / (N - 1);

  // 3. Xuất phát từ góc gần robot nhất (Nearest-Corner Start)
  bool startAtCrossMin = (crossPos <= 0.5f * (crossMin + crossMax));
  bool startAtLongMin  = (longPos  <= 0.5f * (longMin  + longMax));

  float crossStart = startAtCrossMin ? crossMin : crossMax;
  float crossEnd   = startAtCrossMin ? crossMax : crossMin;
  float longStart  = startAtLongMin  ? longMin  : longMax;
  float longOther  = startAtLongMin  ? longMax  : longMin;

  // Homing ngắn nhất về góc xuất phát (bước nào < 30 mm sẽ tự bỏ qua trong runNav)
  steps.push_back({crossType, crossStart, ROLE_HOME});
  steps.push_back({longType,  longStart,  ROLE_HOME});

  // 4. Sinh N làn Boustrophedon
  float curLongEnd = longStart;
  for (int i = 0; i < N; i++) {
    float ci = startAtCrossMin ? (crossMin + i * spacing) : (crossMax - i * spacing);
    planLanes.push_back(ci);
    if (i > 0) steps.push_back({crossType, ci, ROLE_SHIFT});
    curLongEnd = (i % 2 == 0) ? longOther : longStart;
    steps.push_back({longType, curLongEnd, ROLE_LANE});
  }

  // 5. Vòng lau viền 2 cạnh đầu/cuối làn (robot đang đứng sẵn ở góc (crossEnd, curLongEnd))
  if (cfgPerim) {
    float oppLong = (curLongEnd == longMax) ? longMin : longMax;
    steps.push_back({crossType, crossStart, ROLE_PERIM});
    steps.push_back({longType,  oppLong,    ROLE_PERIM});
    steps.push_back({crossType, crossEnd,   ROLE_PERIM});
  }

  Serial.printf("Ke hoach: truc %c, %d lan, buoc %.0f mm (chong lan %.0f mm), %d buoc%s\n",
                planAxisY ? 'Y' : 'X', N, spacing, cfgMopWidth - spacing,
                (int)steps.size(), cfgPerim ? " + vien" : "");
}

// ======================= CẢM BIẾN SIÊU ÂM NON-BLOCKING =======================
static inline void IRAM_ATTR handleUsIsr(int idx) {
  if (digitalRead(US_ECHO[idx]) == HIGH) {
    usRiseUs[idx] = micros();
    usHigh[idx] = true;
  } else if (usHigh[idx]) {
    usWidthUs[idx] = micros() - usRiseUs[idx];
    usHigh[idx] = false;
    usDone[idx] = true;
  }
}
void IRAM_ATTR isrUs0() { handleUsIsr(0); }
void IRAM_ATTR isrUs1() { handleUsIsr(1); }
void IRAM_ATTR isrUs2() { handleUsIsr(2); }

void pushUsSample(int idx, uint16_t mm) {
  uint8_t p = usHistIdx[idx];
  usHist[idx][p] = mm;
  usHistIdx[idx] = (p + 1) % 3;
  us[idx] = median3(usHist[idx][0], usHist[idx][1], usHist[idx][2]);
  if (idx == 0) newUsFront = true;
}

void pollUltrasonic() {
  unsigned long now = millis();
  if (!usBusy) {
    if (now - lastUsMs < US_PERIOD_MS) return;
    lastUsMs = now;
    noInterrupts();
    usDone[usTurn] = false;
    usHigh[usTurn] = false;
    usWidthUs[usTurn] = 0;
    interrupts();

    digitalWrite(US_TRIG[usTurn], LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG[usTurn], HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG[usTurn], LOW);
    usTrigMs = now;
    usBusy = true;
    return;
  }

  if (usDone[usTurn]) {
    noInterrupts();
    uint32_t w = usWidthUs[usTurn];
    usDone[usTurn] = false;
    interrupts();
    uint16_t mm = (w >= 115 && w <= 12000) ? (uint16_t)(w * 0.1715f) : 9999;
    pushUsSample(usTurn, mm);
    usTurn = (usTurn + 1) % 3;
    usBusy = false;
  } else if (now - usTrigMs >= US_TIMEOUT_MS) {
    noInterrupts();
    usHigh[usTurn] = false;
    usDone[usTurn] = false;
    interrupts();
    pushUsSample(usTurn, 9999);
    usTurn = (usTurn + 1) % 3;
    usBusy = false;
  }
}

bool frontBlockedRaw() {
  if (us[0] < US_BLOCK_MM) return true;
  uint16_t m = 9999;
  for (int a = -15; a <= 15; a++) {
    uint16_t d = scan[(a + 360) % 360];
    if (d && d < m) m = d;
  }
  return m < LIDAR_BLOCK_MM;
}

// Khoảng hở dự kiến từ mép robot tới tường bên trái / bên phải theo hướng hiện tại
void expectedSideClearance(float heading, float &leftGap, float &rightGap) {
  if (!haveRoom) { leftGap = rightGap = 9999.0f; return; }
  int h = ((int)roundf(heading / 90.0f) * 90 % 360 + 360) % 360;
  if (h == 0) {           // +Y
    leftGap  = px_mm - ROBOT_RADIUS_MM;
    rightGap = (roomW - px_mm) - ROBOT_RADIUS_MM;
  } else if (h == 180) {  // -Y
    leftGap  = (roomW - px_mm) - ROBOT_RADIUS_MM;
    rightGap = px_mm - ROBOT_RADIUS_MM;
  } else if (h == 90) {   // +X
    leftGap  = (roomL - py_mm) - ROBOT_RADIUS_MM;
    rightGap = py_mm - ROBOT_RADIUS_MM;
  } else {                // 270 (-X)
    leftGap  = py_mm - ROBOT_RADIUS_MM;
    rightGap = (roomL - py_mm) - ROBOT_RADIUS_MM;
  }
}

// ======================= ĐIỀU HƯỚNG =======================
void finishCleaning() {
  mopPwm = 0;
  emergencyStopMotors();
  state = ST_DONE;
  Serial.println("Hoan thanh!");
}

void runNav(bool sensorTick) {
  if (stepIdx >= (int)steps.size()) { finishCleaning(); return; }
  Step &s = steps[stepIdx];

  if (!stepStarted) {
    float cur = (s.type == GO_X) ? px_mm : py_mm;
    if (fabsf(s.val - cur) < 30.0f) { stepIdx++; stepRetries = 0; return; }
    if (s.type == GO_X) tgtHeading = (s.val > px_mm) ? 90.0f : 270.0f;
    else                tgtHeading = (s.val > py_mm) ? 0.0f  : 180.0f;
    stepStarted = true;
    phase = PH_TURN;
    stepStartMs = millis();
    blockCount = 0;
  }

  // Pha lùi ngắn khi chuyển làn GO_X bị vướng mép vật cản
  if (phase == PH_BACKOFF) {
    if (millis() < backoffUntilMs) {
      drive(-SLOW_PWM, -SLOW_PWM);
      return;
    }
    emergencyStopMotors();
    stepStarted = false;  // Thử định hướng và chạy lại bước GO_X
    return;
  }

  float err = wrap180(tgtHeading - psi);

  if (phase == PH_TURN) {
    if (fabsf(err) < TURN_TOL_DEG) { phase = PH_DRIVE; stopMotors(); blockCount = 0; return; }
    if (millis() - stepStartMs > 15000) {
      stepStarted = false;
      stepRetries = 0;
      stepIdx++;
      emergencyStopMotors();
      return;
    }
    int t = (int)fabsf(err * KT);
    t = clampi(t, MIN_TURN_PWM, MAX_TURN_PWM);
    if (err > 0) drive(t, -t); else drive(-t, t);
    return;
  }

  // PH_DRIVE
  float pos = (s.type == GO_X) ? px_mm : py_mm;
  bool positive = (tgtHeading == 0.0f || tgtHeading == 90.0f);
  bool reached = positive ? (pos >= s.val) : (pos <= s.val);

  // Chỉ đếm vật cản khi có mẫu cảm biến mới (không đếm theo tốc độ loop)
  if (sensorTick) {
    if (frontBlockedRaw()) blockCount++;
    else blockCount = 0;
  }
  bool blocked = (blockCount >= BLOCK_CONFIRM_CNT);

  if (reached) {
    stopMotors();
    stepIdx++;
    stepStarted = false;
    stepRetries = 0;
    return;
  }

  if (blocked) {
    emergencyStopMotors();
    blockCount = 0;

    // 1. Nếu đang chuyển làn (ROLE_SHIFT) và chưa retry -> lùi nhẹ 450 ms rồi thử lại 1 lần
    if (s.role == ROLE_SHIFT && stepRetries == 0) {
      stepRetries = 1;
      phase = PH_BACKOFF;
      backoffUntilMs = millis() + 450;
      Serial.println("Vat can khi chuyen lan -> lui nhe thu lai");
      return;
    }

    // 2. Chống bỏ sót kép: nếu bị chặn sớm trên làn chính (còn > RECOVER_MIN_REM_MM)
    //    và ngay sau là [ROLE_SHIFT -> ROLE_LANE], chèn bước ROLE_RECOVER sau ROLE_SHIFT
    //    để làn kế tiếp tiến lên phủ kín vùng phía sau vật cản trước khi quay ngược.
    float remOnBlock = fabsf(s.val - pos);
    if (s.role == ROLE_LANE && remOnBlock > RECOVER_MIN_REM_MM &&
        (int)steps.size() < MAX_PLAN_STEPS && stepIdx + 2 < (int)steps.size() &&
        steps[stepIdx + 1].role == ROLE_SHIFT && steps[stepIdx + 2].role == ROLE_LANE) {
      Step rec = { s.type, s.val, ROLE_RECOVER };
      steps.insert(steps.begin() + stepIdx + 2, rec);
      Serial.println("Vat can giua lan -> chen buoc boc phu nua lan ke tiep");
    } else {
      Serial.println("Gap vat can -> ket thuc buoc");
    }

    stepIdx++;
    stepStarted = false;
    stepRetries = 0;
    return;
  }

  float remain = fabsf(s.val - pos);
  int base = (remain < 300.0f) ? SLOW_PWM : cfgCruise;
  int corr = (int)(KH * err);
  corr = clampi(corr, -60, 60);

  // Chỉ né vật cản hông nếu đó không phải bức tường biên theo kế hoạch
  float leftWallGap, rightWallGap;
  expectedSideClearance(tgtHeading, leftWallGap, rightWallGap);
  if (us[1] < SIDE_NEAR_MM && leftWallGap  > SIDE_NEAR_MM + 40.0f) corr += 25;
  if (us[2] < SIDE_NEAR_MM && rightWallGap > SIDE_NEAR_MM + 40.0f) corr -= 25;

  drive(base + corr, base - corr);

  if (fabsf(err) > 25.0f) phase = PH_TURN;
}

// ======================= LỆNH ĐIỀU KHIỂN =======================
void startCleaning() {
  if (unoFlashMode) return;
  steps.clear();
  planLanes.clear();
  calibOk = calibFail = 0;
  calibSin = calibCos = 0;
  for (int k = 0; k < 4; k++) calibD[k] = 0;
  haveRoom = false; errMsg = "";
  manualUntil = 0; mopPwm = 0;
  stepRetries = 0; blockCount = 0;
  emergencyStopMotors();
  state = ST_CALIB;
  Serial.println("Bat dau hieu chinh...");
}

void stopAll() {
  mopPwm = 0;
  manualUntil = 0;
  state = ST_IDLE;
  emergencyStopMotors();
}

// ======================= WEB SERVER =======================
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  const char* names[] = {"IDLE", "CALIB", "RUN", "DONE", "ERR"};
  String safeErr = errMsg;
  safeErr.replace("\"", "'");
  char b[360];
  snprintf(b, sizeof(b),
           "{\"state\":\"%s\",\"x\":%d,\"y\":%d,\"psi\":%d,\"W\":%d,\"L\":%d,"
           "\"step\":%d,\"total\":%d,\"us0\":%d,\"us1\":%d,\"us2\":%d,"
           "\"mop\":%d,\"oL\":%d,\"oR\":%d,\"uno\":%d,\"err\":\"%s\"}",
           names[state], (int)px_mm, (int)py_mm, (int)psi,
           haveRoom ? (int)roomW : 0, haveRoom ? (int)roomL : 0,
           stepIdx, (int)steps.size(), us[0], us[1], us[2],
           mopPwm, outL, outR, unoFlashMode ? 1 : 0, safeErr.c_str());
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

void handleScan() {
  String s;
  s.reserve(2200);
  s += '[';
  for (int i = 0; i < 360; i++) {
    if (i) s += ',';
    s += scan[i];
  }
  s += ']';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

// Trả 720 byte nhị phân (360 x uint16_t little-endian) cho Mobile App (Android Studio)
void handleScanBin() {
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(sizeof(scan));
  server.send(200, "application/octet-stream", "");
  server.client().write((const uint8_t*)scan, sizeof(scan));
}

void applyCmd(const String& c, int v) {
  if (unoFlashMode) return;
  v = clampi(v, 80, MAX_SAFE_PWM);
  if (c == "start")       startCleaning();
  else if (c == "stop")   stopAll();
  else if (c == "mopon")  { if (state != ST_RUN) mopPwm = MOP_PWM; }
  else if (c == "mopoff") { if (state != ST_RUN) mopPwm = 0; }
  else if (c == "S")      { manualUntil = 0; stopMotors(); }
  else if (state != ST_RUN && state != ST_CALIB) {
    if      (c == "F") { manualL =  v; manualR =  v; manualUntil = millis() + 400; }
    else if (c == "B") { manualL = -v; manualR = -v; manualUntil = millis() + 400; }
    else if (c == "L") { manualL = -v; manualR =  v; manualUntil = millis() + 400; }
    else if (c == "R") { manualL =  v; manualR = -v; manualUntil = millis() + 400; }
  }
}

void handleCmd() {
  int v = server.hasArg("v") ? server.arg("v").toInt() : cfgCruise;
  applyCmd(server.arg("c"), v);
  server.send(200, "text/plain", "ok");
}

void handleUnoMode() {
  if (server.hasArg("en")) {
    bool en = (server.arg("en").toInt() != 0);
    if (en && (state == ST_RUN || state == ST_CALIB)) {
      server.send(409, "text/plain", "busy");
      return;
    }
    setUnoFlashMode(en);
  }
  server.send(200, "text/plain", unoFlashMode ? "1" : "0");
}

// Lệnh từ HC-05 (Arduino chuyển tiếp dạng "C,<ký tự>\n")
void pollLink() {
  if (unoFlashMode) return;
  static char line[24];
  static int n = 0;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      line[n] = 0;
      if (n >= 3 && line[0] == 'C' && line[1] == ',') {
        char k = line[2];
        if      (k == 'A') applyCmd("start", cfgCruise);
        else if (k == 'X') applyCmd("stop", cfgCruise);
        else if (k == 'F') applyCmd("F", cfgCruise);
        else if (k == 'B') applyCmd("B", cfgCruise);
        else if (k == 'L') applyCmd("L", cfgCruise);
        else if (k == 'R') applyCmd("R", cfgCruise);
        else if (k == 'S') applyCmd("S", cfgCruise);
        else if (k == 'W') applyCmd("mopon", cfgCruise);
        else if (k == 'w') applyCmd("mopoff", cfgCruise);
      }
      n = 0;
    } else if (c != '\r' && n < (int)sizeof(line) - 1) {
      line[n++] = c;
    }
  }
}

// ---- Cài đặt lưu trong Flash ----
void loadSettings() {
  prefs.begin("robot", true);
  cfgMopWidth = clampf(prefs.getFloat("mop", DEF_MOP_WIDTH_MM), 100.0f, 400.0f);
  cfgCruise   = clampi(prefs.getInt("cruise", DEF_CRUISE_PWM), 100, MAX_SAFE_PWM);
  cfgEdge     = clampi(prefs.getInt("edge", DEF_EDGE_MM), ROBOT_RADIUS_MM + 10, 500);
  cfgPerim    = prefs.getBool("perim", true);
  prefs.end();
}

void handleSettings() {
  bool changed = server.hasArg("mop") || server.hasArg("cruise") ||
                 server.hasArg("edge") || server.hasArg("perim");
  if (changed) {
    if (state == ST_RUN || state == ST_CALIB) {
      server.send(409, "text/plain", "busy");
      return;
    }
    if (server.hasArg("mop"))
      cfgMopWidth = clampf(server.arg("mop").toFloat(), 100.0f, 400.0f);
    if (server.hasArg("cruise"))
      cfgCruise   = clampi(server.arg("cruise").toInt(), 100, MAX_SAFE_PWM);
    if (server.hasArg("edge"))
      cfgEdge     = clampi(server.arg("edge").toInt(), ROBOT_RADIUS_MM + 10, 500);
    if (server.hasArg("perim"))
      cfgPerim    = (server.arg("perim").toInt() != 0);

    prefs.begin("robot", false);
    prefs.putFloat("mop", cfgMopWidth);
    prefs.putInt("cruise", cfgCruise);
    prefs.putInt("edge", cfgEdge);
    prefs.putBool("perim", cfgPerim);
    prefs.end();
  }
  char b[112];
  snprintf(b, sizeof(b), "{\"mop\":%d,\"cruise\":%d,\"edge\":%d,\"perim\":%d}",
           (int)cfgMopWidth, cfgCruise, cfgEdge, cfgPerim ? 1 : 0);
  server.send(200, "application/json", b);
}

void handlePlan() {
  String j = "{\"axis\":\"";
  j += planAxisY ? 'Y' : 'X';
  j += "\",\"lanes\":[";
  for (size_t i = 0; i < planLanes.size(); i++) {
    if (i) j += ',';
    j += (int)planLanes[i];
  }
  j += "],\"edge\":" + String(cfgEdge) + ",\"perim\":" + String(cfgPerim ? 1 : 0) + "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Dang ket noi WiFi");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(250); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    myIP = WiFi.localIP().toString();
    Serial.println("Da ket noi WiFi nha.");
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    myIP = WiFi.softAPIP().toString();
    Serial.printf("Khong vao duoc WiFi nha -> phat WiFi rieng '%s' (mat khau %s)\n", AP_SSID, AP_PASS);
  }
  Serial.println("==============================================");
  Serial.println("Mo trinh duyet:  http://" + myIP);
  Serial.println("hoac:            http://robot.local");
  Serial.println("==============================================");
  if (MDNS.begin("robot")) MDNS.addService("http", "tcp", 80);
}

void sendLink() {
  if (unoFlashMode) return;
  unsigned long now = millis();

  // Cập nhật PWM thực tế qua bộ ramp mềm và gửi ở 20 Hz (50 ms)
  if (now - lastSendMs >= 50) {
    lastSendMs = now;
    outL = rampAxis(outL, cmdL);
    outR = rampAxis(outR, cmdR);
    Serial1.printf("M,%d,%d\n", outL, outR);
  }

  // Gửi lệnh chổi ngay khi thay đổi trạng thái hoặc mỗi 300 ms
  if (mopPwm != lastSentMop || now - lastMopMs >= 300) {
    lastMopMs = now;
    lastSentMop = mopPwm;
    Serial1.printf("W,%d\n", mopPwm);
  }

  // Telemetry gọn (< 32 byte, gửi mỗi 2s lệch pha với gói M để tránh nghẽn SoftwareSerial của Uno)
  if (now - lastTelMs >= 2000 && now - lastSendMs >= 20) {
    lastTelMs = now;
    const char* nm[] = {"IDLE", "CAL", "RUN", "DONE", "ERR"};
    Serial1.printf("T,%s %d,%d %da %d/%d\n",
                   nm[state], (int)px_mm, (int)py_mm, (int)psi, stepIdx, (int)steps.size());
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  loadSettings();

  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  Serial2.setRxBufferSize(2048);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, -1);

  for (int i = 0; i < 3; i++) {
    pinMode(US_TRIG[i], OUTPUT);
    digitalWrite(US_TRIG[i], LOW);
    pinMode(US_ECHO[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(US_ECHO[0]), isrUs0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_ECHO[1]), isrUs1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_ECHO[2]), isrUs2, CHANGE);

  initCrc();
  memset(acc, 0, sizeof(acc));
  memset(scan, 0, sizeof(scan));

  startWiFi();
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/scan", handleScan);
  server.on("/api/scan.bin", handleScanBin);
  server.on("/api/cmd", handleCmd);
  server.on("/api/settings", handleSettings);
  server.on("/api/plan", handlePlan);
  server.on("/api/unomode", handleUnoMode);
  server.begin();

  // Nạp code qua WiFi (Arduino IDE -> Tools -> Port -> robot)
  ArduinoOTA.setHostname("robot");
  ArduinoOTA.setPassword("robot1234");
  ArduinoOTA.setMdnsEnabled(false);
  ArduinoOTA.onStart([]() {
    cmdL = cmdR = outL = outR = 0;
    mopPwm = 0;
    if (!unoFlashMode) {
      Serial1.print("M,0,0\nW,0\n");
      Serial1.flush();
    }
  });
  ArduinoOTA.begin();

  lastPoseMs = millis();
  lastScanMs = millis();
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  pollLink();
  pollLidar();
  pollUltrasonic();
  predictPose();

  bool fresh = newScan;
  if (fresh) newScan = false;
  bool usTick = newUsFront;
  if (usTick) newUsFront = false;

  switch (state) {
    case ST_IDLE: case ST_DONE: case ST_ERR:
      if (!unoFlashMode && millis() < manualUntil) {
        int l = manualL, r = manualR;
        if (l > 0 && r > 0 && us[0] < US_BLOCK_MM) {
          emergencyStopMotors();
        } else {
          drive(l, r);
        }
      } else {
        stopMotors();
      }
      break;

    case ST_CALIB:
      stopMotors();
      if (fresh) doCalibrate();
      break;

    case ST_RUN:
      if (millis() - lastScanMs > 1000) {
        emergencyStopMotors();
        if (millis() - lastScanMs > 5000) setError("Mat LiDAR");
        break;
      }
      if (fresh) correctPose();
      runNav(fresh || usTick);
      break;
  }

  sendLink();
}
