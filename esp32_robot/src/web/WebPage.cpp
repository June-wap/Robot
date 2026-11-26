#include "WebPage.h"

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
<div class="row">Chồng lấn (0-30%) <input id="sOv" type="number" min="0" max="30" step="1" style="width:70px">
<select id="sDir"><option value="AUTO">Hướng: Tự động</option><option value="X">Hướng: Trục X</option><option value="Y">Hướng: Trục Y</option></select></div>
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

let trail=[],lanes=[],planEdge=210,planAxis='Y',lastPlanFetch=0;
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
function laneColor(st){
  if(st==='COMPLETED')return '#2b8a3e';
  if(st==='RUNNING')return '#f59f00';
  if(st==='PARTIAL')return '#e8590c';
  if(st==='BLOCKED'||st==='FAILED')return '#e03131';
  return '#335';
}
function drawMap(s){
  const c=$('map'),g=c.getContext('2d'),w=c.width,h=c.height;
  g.fillStyle='#000';g.fillRect(0,0,w,h);
  if(!s.W||!s.L){g.fillStyle='#888';g.fillText('Chưa đo phòng',10,20);return}
  const m=10,sc=Math.min((w-2*m)/s.W,(h-2*m)/s.L);
  const X=x=>m+x*sc,Y=y=>h-m-y*sc;
  g.strokeStyle='#4af';g.strokeRect(X(0),Y(s.L),s.W*sc,s.L*sc);
  lanes.forEach(ln=>{
    g.strokeStyle=laneColor(ln.status);
    g.lineWidth=(ln.status==='RUNNING')?2:1;
    g.beginPath();
    g.moveTo(X(ln.start_x),Y(ln.start_y));
    g.lineTo(X(ln.end_x),Y(ln.end_y));
    g.stroke();
  });
  g.lineWidth=1;
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
    $('st').textContent='Trạng thái: '+s.state+' ('+s.cleaning_result+')'+(s.uno?' [NHẢ D0/D1 NẠP UNO]':'')+
      '\nVị trí: x='+s.x+' y='+s.y+' mm, góc='+s.psi+'° | PWM: '+s.oL+'/'+s.oR+
      '\nPhòng: '+s.W+' x '+s.L+' mm | Hướng: '+s.cleaning_direction+' ('+s.start_corner+')'+
      '\nLàn: '+s.completed_lanes+'/'+s.total_lanes+' xong, '+s.blocked_lanes+' cản, '+s.partial_lanes+' phần ('+s.completed_percent+'%)'+
      '\nSiêu âm Trước/Trái/Phải: '+s.us0+'/'+s.us1+'/'+s.us2+' mm'+(s.err?'\nLỗi: '+s.err:'');
    mopOn=s.mop>0;$('bMop').textContent='Chổi: '+(mopOn?'BẬT':'TẮT');
    unoMode=!!s.uno;$('bUno').textContent='Nạp Arduino Uno (Nhả D0/D1): '+(unoMode?'BẬT':'TẮT');
    $('bar').style.width=Math.round(s.processed_percent||0)+'%';
    const now=Date.now();
    if((s.state==='RUN'||s.state==='DONE')&&(now-lastPlanFetch>1500||!lanes.length)){
      lastPlanFetch=now;
      const p=await (await fetch('/api/plan')).json();
      lanes=p.lanes||[];planEdge=p.edge_mm||210;planAxis=p.direction||'Y';
    }
    drawMap(s);
    const a=await (await fetch('/api/scan')).json();
    drawRadar(a);
  }catch(e){$('st').textContent='Mất kết nối với robot...'}
  finally{setTimeout(poll,500)}
}
async function loadSet(){try{const s=await (await fetch('/api/settings')).json();$('sMop').value=s.mop;$('sCr').value=s.cruise;$('sEd').value=s.edge;$('sOv').value=s.overlap;$('sDir').value=s.direction}catch(e){}}
$('bSave').onclick=async()=>{const r=await fetch('/api/settings?mop='+$('sMop').value+'&cruise='+$('sCr').value+'&edge='+$('sEd').value+'&overlap='+$('sOv').value+'&direction='+$('sDir').value);$('sMsg').textContent=r.ok?' Đã lưu':(r.status===400?' Giá trị không hợp lệ':' Không lưu được (robot đang chạy)');loadSet()};
loadSet();
poll();
</script></body></html>
)rawliteral";
