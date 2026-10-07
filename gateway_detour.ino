// NODE 2 (Cockpit HUD) - STEP 9d (detours + distance): live map + ntfy alerts + routing (pothole 10s, bump 15s, +5s per extra report)
// ESP-NOW receiver + OLED alerts + relay buzzer (SOS only) + joins your phone hotspot
// and serves a test page + JSON API that the live map will use next.
//
// BEFORE FLASHING: type your hotspot name and password into WIFI_SSID / WIFI_PASS below.
// The hotspot MUST be 2.4 GHz (ESP32 cannot use 5 GHz).
//   Android: Hotspot settings > AP band > 2.4 GHz
//   iPhone : Personal Hotspot > turn ON 'Maximize Compatibility'
// Libraries: Adafruit SSD1306, Adafruit GFX (Library Manager)
// Wiring: OLED SDA=21 SCL=22 (0x3C) | Relay IN = GPIO25 (buzzer on COM/NO)
//
// Serial commands (115200, Newline):
//   pol low | pol high   relay polarity (low = relay ON when pin is LOW). Saved.
//   beep                 test beep
//   alarm                test alarm pattern (type 'clear' to stop)
//   clear                clear alarms / screen
//   status               link status

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RELAY_PIN 25
#define LED_PIN   2
#define BUZZ_ON_ROLLOVER false   // set true if you also want the buzzer on rollover

// ---- YOUR HOTSPOT (edit these two lines) ----
const char *WIFI_SSID = "UR MY SLAVE";
const char *WIFI_PASS = "TAKEURSALARY";

WebServer server(80);

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Preferences prefs;
bool activeLow = true;   // changeable with 'pol low|high|open'
bool openDrain = false;  // 'pol open': active-LOW module that stays ON from 3.3V logic

// ---------- packet (must match Node 1) ----------
struct __attribute__((packed)) Packet {
  uint8_t  type;
  uint8_t  conf;
  uint16_t seq;
  uint32_t vehicleId;
  float    lat, lon;
  float    speedKmh;
  float    peakG;
  uint8_t  segment;
  uint8_t  gpsFix;
  uint32_t uptimeMs;
  uint16_t checksum;
};

uint16_t csum(const uint8_t *d, size_t n) {
  uint16_t s = 0;
  for (size_t i = 0; i < n; i++) s = ((s << 1) | (s >> 15)) ^ d[i];
  return s;
}

// ---------- receive queue ----------
Packet q[8];
volatile uint8_t qh = 0, qt = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len != sizeof(Packet)) return;
  uint8_t n = (qh + 1) & 7;
  if (n == qt) return;               // queue full
  memcpy(&q[qh], data, sizeof(Packet));
  qh = n;
}

// ---------- relay + buzzer patterns ----------
// pattern = durations in ms: ON, OFF, ON, OFF, ...
const uint16_t POT_P[]   = {700};
const uint16_t BUMP_P[]  = {500, 250, 500};
const uint16_t ALARM_P[] = {1200, 400};

const uint16_t *pat = nullptr;
int patLen = 0, patIdx = 0;
bool patLoop = false, patActive = false;
unsigned long patNext = 0;

void relaySet(bool on) {
  if (openDrain) {
    // ON = pull pin LOW, OFF = release the pin (module's own pull-up turns it off)
    if (on) { pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW); }
    else    { pinMode(RELAY_PIN, INPUT); }
  } else {
    digitalWrite(RELAY_PIN, activeLow ? !on : on);
  }
}

void startPattern(const uint16_t *p, int n, bool loop) {
  pat = p; patLen = n; patIdx = 0; patLoop = loop;
  patActive = true; patNext = 0;
}

void stopPattern() { patActive = false; relaySet(false); }

void buzzerService(unsigned long now) {
  if (!patActive || now < patNext) return;
  if (patIdx >= patLen) {
    if (patLoop) patIdx = 0;
    else { stopPattern(); return; }
  }
  relaySet(patIdx % 2 == 0);
  patNext = now + pat[patIdx];
  patIdx++;
}

// ---------- alert state ----------
uint8_t alertType = 0;            // 0 none, 1 pothole, 2 bump, 3 rollover, 4 sos
unsigned long alertSince = 0;
uint8_t alertConf = 0, alertSeg = 0, alertFix = 0;
float alertLat = 0, alertLon = 0;
uint16_t lastSeq = 0;
unsigned long lastRx = 0;
int cntPot = 0, cntBump = 0;

// ---------- event log (newest 20, served to the web page) ----------
struct Ev { uint16_t seq; uint8_t type, conf, fix; float lat, lon, peak; uint32_t t; };
const int EVMAX = 40;
Ev evs[EVMAX];
int evN = 0;
float podLat = 0, podLon = 0, podSpeed = 0;
uint8_t podFix = 0;

void logEvent(const Packet &p, unsigned long now) {
  Ev &e = evs[evN % EVMAX];
  e.seq = p.seq; e.type = p.type; e.conf = p.conf; e.fix = p.gpsFix;
  e.lat = p.lat; e.lon = p.lon; e.peak = p.peakG; e.t = now;
  evN++;
}

const unsigned long SHORT_ALERT_MS = 5000;
const unsigned long ALARM_TIMEOUT_MS = 120000;

bool alarmActive() { return alertType == 3 || alertType == 4; }

void clearAlert() {
  alertType = 0;
  stopPattern();
}

// ---------- ntfy phone notifications (SOS / rollover) ----------
// Subscribe to this topic in the ntfy app. Anyone who knows the topic name can read it, so keep it private.
const char *NTFY_TOPIC = "rg-sos-v4m8q2xz";

struct NtfyMsg { char title[40]; char body[200]; char click[100]; char tags[28]; uint8_t prio; };
QueueHandle_t ntfyQ = NULL;

void ntfyTask(void *arg) {
  NtfyMsg m;
  for (;;) {
    if (xQueueReceive(ntfyQ, &m, portMAX_DELAY) != pdTRUE) continue;
    for (int attempt = 0; attempt < 2; attempt++) {
      if (WiFi.status() != WL_CONNECTED) { Serial.println("ntfy: no Wi-Fi, will retry"); vTaskDelay(2000 / portTICK_PERIOD_MS); continue; }
      WiFiClientSecure client;
      client.setInsecure();                      // demo: skips certificate check
      HTTPClient http;
      String url = String("https://ntfy.sh/") + NTFY_TOPIC;
      int code = -1;
      if (http.begin(client, url)) {
        http.setTimeout(8000);
        http.addHeader("Content-Type", "text/plain");
        http.addHeader("Title", m.title);
        http.addHeader("Priority", String(m.prio));
        http.addHeader("Tags", m.tags);
        if (m.click[0]) http.addHeader("Click", m.click);
        code = http.POST((uint8_t *)m.body, strlen(m.body));
        http.end();
      }
      Serial.printf("ntfy -> HTTP %d\n", code);
      if (code >= 200 && code < 300) break;
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
  }
}

void ntfyPush(const char *title, const char *body, uint8_t prio, const char *tags, const char *click) {
  if (!ntfyQ) return;
  NtfyMsg m; memset(&m, 0, sizeof m);
  strlcpy(m.title, title, sizeof m.title);
  strlcpy(m.body, body, sizeof m.body);
  strlcpy(m.tags, tags, sizeof m.tags);
  if (click) strlcpy(m.click, click, sizeof m.click);
  m.prio = prio;
  xQueueSend(ntfyQ, &m, 0);
}

void ntfyAlert(const Packet &p, unsigned long now) {
  static unsigned long lastSent[8] = {0};
  if (p.type >= 8) return;
  if (lastSent[p.type] && now - lastSent[p.type] < 8000) return;   // avoid spamming
  lastSent[p.type] = now;
  const char *what = (p.type == 4) ? "Driver pressed SOS" : "Vehicle rollover detected";
  char body[200], click[100] = "";
  if (p.gpsFix) {
    snprintf(click, sizeof click, "https://maps.google.com/?q=%.6f,%.6f", p.lat, p.lon);
    snprintf(body, sizeof body, "%s. Location: %.6f, %.6f (tap to open map)", what, p.lat, p.lon);
  } else {
    snprintf(body, sizeof body, "%s. Location unavailable (no GPS fix).", what);
  }
  ntfyPush(p.type == 4 ? "SOS - Road Guard" : "ROLLOVER - Road Guard", body,
           p.type == 4 ? 5 : 4, p.type == 4 ? "rotating_light,sos" : "warning,car", click);
}

void handlePacket(const Packet &p, unsigned long now) {
  if (csum((const uint8_t *)&p, sizeof(Packet) - 2) != p.checksum) {
    Serial.println("bad checksum, dropped");
    return;
  }
  if (p.seq == lastSeq) return;      // duplicate (SOS repeats)
  lastSeq = p.seq;
  lastRx = now;
  podLat = p.lat; podLon = p.lon; podFix = p.gpsFix; podSpeed = p.speedKmh;   // live pod position
  if (p.type == 6) return;           // heartbeat

  Serial.printf("[%u] type=%d conf=%d peak=%.2f seg=%d fix=%d lat=%.6f lon=%.6f\n",
                p.seq, p.type, p.conf, p.peakG, p.segment, p.gpsFix, p.lat, p.lon);

  if (p.type == 5) {
    if (alertType >= 3) ntfyPush("Alert cleared - Road Guard", "Vehicle alert cleared (back to normal).", 3, "white_check_mark", "");
    clearAlert(); return;
  }

  if (p.type == 1) cntPot++;
  if (p.type == 2) cntBump++;
  if (p.type >= 1 && p.type <= 4) logEvent(p, now);
  if (p.type == 3 || p.type == 4) ntfyAlert(p, now);

  // pothole/bump must not interrupt an active SOS / rollover alarm
  if ((p.type == 1 || p.type == 2) && alarmActive()) return;

  alertType = p.type; alertSince = now; alertConf = p.conf;
  alertSeg = p.segment; alertFix = p.gpsFix; alertLat = p.lat; alertLon = p.lon;

  // Buzzer/relay only for SOS (touch wire). Potholes/bumps just show on the OLED.
  if (p.type == 4) startPattern(ALARM_P, 2, true);
  else if (p.type == 3 && BUZZ_ON_ROLLOVER) startPattern(ALARM_P, 2, true);
}

// ---------- OLED ----------
void drawLocation(int y) {
  display.setTextSize(1);
  display.setCursor(0, y);
  if (alertSeg) display.printf("Road segment: %d", alertSeg);
  else if (alertFix) {
    display.printf("%.5f", alertLat);
    display.setCursor(0, y + 9);
    display.printf("%.5f", alertLon);
  } else display.print("Location: no GPS fix");
}

void drawScreen(unsigned long now) {
  display.clearDisplay();

  if (alertType == 0) {
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);  display.print("ROAD GUARD - NODE 2");
    display.drawLine(0, 10, 127, 10, WHITE);
    bool online = (now - lastRx < 6000) && lastRx != 0;
    display.setCursor(0, 16); display.printf("Scout: %s", online ? "ONLINE" : "searching...");
    uint8_t ch = 0; wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);
    display.setCursor(0, 28);
    if (WiFi.status() == WL_CONNECTED) display.print(WiFi.localIP());
    else display.print("Wi-Fi: connecting...");
    display.setCursor(0, 40); display.printf("Ch%d Pot:%d Bump:%d", ch, cntPot, cntBump);
    display.setCursor(0, 54); display.print("Status: ALL CLEAR");
    display.display();
    return;
  }

  bool crit = alarmActive();
  bool inv = crit && ((now / 400) % 2);
  if (inv) display.fillRect(0, 0, 128, 64, WHITE);
  display.setTextColor(inv ? BLACK : WHITE);

  const char *title = alertType == 1 ? "POTHOLE" : alertType == 2 ? "BUMP" : alertType == 3 ? "ROLLOVER" : "SOS!";
  display.setTextSize(2);
  display.setCursor(0, 0); display.print(title);

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(crit ? "AVOID THIS ROAD" : "Slow down ahead");

  drawLocation(32);

  display.setTextSize(1);
  display.setCursor(0, 54);
  if (crit) display.printf("Active %lus", (now - alertSince) / 1000);
  else display.printf("%s  %lus ago", alertConf >= 60 ? "Confirmed" : "Suspected", (now - alertSince) / 1000);
  display.display();
}

// ---------- web server ----------
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Road Guard - Node 2</title>
<style>
body{font-family:system-ui,sans-serif;background:#10141a;color:#e8edf2;margin:0;padding:16px}
h1{font-size:20px;margin:0 0 4px}.sub{color:#8a97a6;font-size:13px;margin-bottom:14px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
.card{background:#1a212b;border-radius:10px;padding:10px 14px;min-width:90px}
.card b{display:block;font-size:20px}.card span{color:#8a97a6;font-size:12px}
.ok{color:#4ade80}.bad{color:#f87171}
table{width:100%;border-collapse:collapse;font-size:13px}
td,th{padding:6px 4px;border-bottom:1px solid #26303c;text-align:left}
</style></head><body>
<h1>Road Guard - Node 2 is reachable</h1>
<div class="sub">If you can read this, phone-to-ESP32 over the hotspot works. <a href="/map" style="color:#60a5fa">Open live map</a></div>
<div class="row">
<div class="card"><span>Scout</span><b id="sc">-</b></div>
<div class="card"><span>Potholes</span><b id="po">-</b></div>
<div class="card"><span>Bumps</span><b id="bu">-</b></div>
<div class="card"><span>Alert</span><b id="al">-</b></div>
<div class="card"><span>Pod GPS</span><b id="gp">-</b></div>
</div>
<table><thead><tr><th>Event</th><th>Conf</th><th>Lat, Lon</th><th>Ago</th></tr></thead><tbody id="ev"></tbody></table>
<script>
const N=["none","POTHOLE","BUMP","ROLLOVER","SOS"];
async function tick(){try{
const r=await fetch("/api/state",{cache:"no-store"});const d=await r.json();
const sc=document.getElementById("sc");sc.textContent=d.scout?"ONLINE":"OFFLINE";sc.className=d.scout?"ok":"bad";
po.textContent=d.pot;bu.textContent=d.bump;al.textContent=N[d.alert]||"-";
gp.textContent=d.pod.fix?d.pod.lat.toFixed(5)+", "+d.pod.lon.toFixed(5):"no fix";
ev.innerHTML=d.events.map(e=>"<tr><td>"+N[e.type]+"</td><td>"+e.conf+"</td><td>"+(e.fix?e.lat.toFixed(5)+", "+e.lon.toFixed(5):"no fix")+"</td><td>"+Math.round(e.ago/1000)+"s</td></tr>").join("");
}catch(x){sc.textContent="NO LINK";sc.className="bad"}}
tick();setInterval(tick,1000);
</script></body></html>)HTML";

const char PAGEMAP[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Road Guard Live Map</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<style>
html,body,#m{height:100%;margin:0}
#bar{position:absolute;z-index:1000;top:8px;left:8px;right:8px;display:flex;gap:6px;flex-wrap:wrap;font:13px system-ui,sans-serif;pointer-events:none}
#bar>*{pointer-events:auto;background:#fffffff2;border-radius:8px;padding:6px 10px;box-shadow:0 1px 4px #0005}
#al{display:none;background:#dc2626;color:#fff;font-weight:700;width:100%;text-align:center}
.pulse{animation:p .8s infinite}@keyframes p{50%{opacity:.25}}
.pin{width:20px;height:20px;border-radius:50%;border:3px solid #fff;box-shadow:0 0 6px #000}
.sos{background:#dc2626}.roll{background:#111}.st{background:#16a34a}.en{background:#2563eb}.ok{color:#16a34a}.bad{color:#dc2626}
#rp{position:absolute;z-index:1000;bottom:26px;left:8px;right:8px;background:#fffffff2;border-radius:8px;padding:8px 10px;font:13px system-ui,sans-serif;box-shadow:0 1px 4px #0005}
button{font:inherit;border:0}
</style></head><body>
<div id="bar"><div id="al"></div><div>Scout <b id="sc">-</b></div>
<div>Potholes <b id="po">0</b> | Bumps <b id="bu">0</b></div><div id="nf" style="display:none"></div>
<button id="fl">Follow pod: ON</button><button id="sp">Start = pod</button><button id="cr">Clear route</button><button id="tp">Test pothole</button><button id="tb">Test bump</button><button id="rs">Reset</button></div>
<div id="rp">Tap the map to set start (A), then destination (B)</div>
<div id="m"></div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<script>
const map=L.map('m').setView([21.1236,79.0507],15);
const osm=L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);
const car=L.tileLayer('https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors &copy; CARTO'});
L.control.layers({'OpenStreetMap':osm,'Carto (backup)':car}).addTo(map);
let Z=[],P=[],seen=new Set(),nofix=0,follow=true,first=true,pod=null,lastMs=0,dirty=false,T=0,S=null,E=null,sm=null,em=null,R=[],rl=[];
const $=id=>document.getElementById(id);
function dist(a,b,c,d){const p=Math.PI/180,x=(c-a)*p,y=(d-b)*p*Math.cos(a*p);return 6371000*Math.hypot(x,y)}
function sty(z){const c=z.type==1?'#dc2626':'#f59e0b';z.c.setLatLng([z.lat,z.lon]);z.c.setRadius(Math.max(8,z.sp+6));
z.c.setStyle(z.n>1?{color:c,fillColor:c,fillOpacity:.45,weight:3,dashArray:null}:{color:c,fillColor:c,fillOpacity:.15,weight:2,dashArray:'4 4'});
z.c.bindTooltip((z.test?'TEST (simulated) ':'')+(z.type==1?'Pothole':'Bump')+' zone: '+z.n+(z.n>1?' reports (confirmed)':' report (suspected)'))}
function addZone(e){let z=Z.find(q=>!q.test&&q.type==e.type&&dist(q.lat,q.lon,e.lat,e.lon)<15);
if(!z){z={type:e.type,lat:e.lat,lon:e.lon,n:0,sp:0,c:L.circle([e.lat,e.lon]).addTo(map)};Z.push(z)}
z.sp=Math.max(z.sp,dist(z.lat,z.lon,e.lat,e.lon));z.lat=(z.lat*z.n+e.lat)/(z.n+1);z.lon=(z.lon*z.n+e.lon)/(z.n+1);z.n++;sty(z);dirty=true}
function addPin(e){const m=L.marker([e.lat,e.lon],{icon:L.divIcon({className:'',html:'<div class="pin '+(e.type==4?'sos':'roll')+'"></div>',iconSize:[20,20]})}).addTo(map);
m.bindTooltip(e.type==4?'SOS':'Rollover');m.type=e.type;P.push(m);dirty=true}
function clearAll(){Z.forEach(z=>map.removeLayer(z.c));P.forEach(m=>map.removeLayer(m));Z=[];P=[];seen.clear();nofix=0;dirty=true}
async function tick(){try{
const d=await(await fetch('/api/state',{cache:'no-store'})).json();
if(d.ms<lastMs)clearAll();lastMs=d.ms;
$('sc').textContent=d.scout?'ONLINE':'OFFLINE';$('sc').className=d.scout?'ok':'bad';
$('po').textContent=d.pot;$('bu').textContent=d.bump;
for(const e of d.events.slice().reverse()){const k=e.type+'-'+(d.ms-e.ago);if(seen.has(k))continue;seen.add(k);
if(!e.fix){nofix++;continue}if(e.type<3)addZone(e);else addPin(e)}
if(dirty){dirty=false;score()}
const nf=$('nf');nf.style.display=nofix?'block':'none';nf.textContent=nofix+' event(s) had no GPS fix (not plotted)';
const a=$('al');a.style.display=d.alert>=3?'block':'none';a.textContent=d.alert==4?'SOS - AVOID THIS ROAD':'ROLLOVER - AVOID THIS ROAD';a.className=d.alert==4?'pulse':'';
P.forEach(m=>{const el=m.getElement();if(el)el.firstChild.classList.toggle('pulse',d.alert==4&&m.type==4)});
if(d.pod.fix){const ll=[d.pod.lat,d.pod.lon];
if(!pod)pod=L.circleMarker(ll,{radius:8,color:'#fff',weight:3,fillColor:'#2563eb',fillOpacity:1}).addTo(map);else pod.setLatLng(ll);
if(follow)map.setView(ll,first?17:map.getZoom());first=false}
}catch(x){$('sc').textContent='NO LINK';$('sc').className='bad'}}
$('fl').onclick=()=>{follow=!follow;$('fl').textContent='Follow pod: '+(follow?'ON':'OFF')};
map.on('dragstart',()=>{follow=false;$('fl').textContent='Follow pod: OFF'});
$('rs').onclick=async()=>{try{await fetch('/api/reset')}catch(x){}clearAll()};
// ---- routing: OSRM routes + hazard penalties + automatic detours ----
const POT=10,BUMP=15,SUSP=5;   // seconds added per hazard zone (tunable)
const OSRM='https://router.project-osrm.org/route/v1/driving/';
let busy=false,dk='',ign=0;
function segd(p,a,b){const k=Math.cos(p[0]*Math.PI/180),m=111320,px=p[1]*k*m,py=p[0]*m,ax=a[0]*k*m,ay=a[1]*m,bx=b[0]*k*m,by=b[1]*m,dx=bx-ax,dy=by-ay,l=dx*dx+dy*dy;
let t=l?((px-ax)*dx+(py-ay)*dy)/l:0;t=Math.max(0,Math.min(1,t));return Math.hypot(px-ax-t*dx,py-ay-t*dy)}
function near(g,p){let d=1e9;for(let i=0;i<g.length-1;i++)d=Math.min(d,segd(p,g[i],g[i+1]));return d}
function ptd(a,b){return dist(a[0],a[1],b[0],b[1])}
function hazards(){const h=[];
Z.forEach(z=>h.push({p:[z.lat,z.lon],r:Math.max(8,z.sp+6)+8,pen:z.n>1?(z.type==1?POT:BUMP)+5*Math.min(z.n-2,6):(z.type==1?SUSP:SUSP/2),blk:false}));
P.forEach(m=>{const l=m.getLatLng();h.push({p:[l.lat,l.lng],r:40,pen:0,blk:true})});
const k=h.filter(x=>!(S&&ptd([S.lat,S.lng],x.p)<x.r)&&!(E&&ptd([E.lat,E.lng],x.p)<x.r));ign=h.length-k.length;return k}
function evalR(r,hz){const g=r.geometry.coordinates;r.pen=0;r.hits=0;r.blk=false;r.hz=[];
hz.forEach(x=>{if(near(g,x.p)<=x.r){r.hits++;r.pen+=x.pen;r.hz.push(x.p);if(x.blk)r.blk=true}});r.cost=r.duration+r.pen}
function score(){if(!R.length||!S||!E)return;const hz=hazards();R.forEach(r=>evalR(r,hz));
const fast=R.reduce((a,b)=>a.duration<=b.duration?a:b),ok=R.filter(r=>!r.blk),best=(ok.length?ok:R).reduce((a,b)=>a.cost<=b.cost?a:b);
draw(fast,best);
if(best.hits&&!busy){const k=S.lat+','+S.lng+','+E.lat+','+E.lng+'|'+hz.map(x=>x.p[0].toFixed(5)+x.p[1].toFixed(5)).join(';');if(k!==dk){dk=k;detour(best,hz)}}}
function draw(f,b){rl.forEach(l=>map.removeLayer(l));rl=[];
const ll=r=>r.geometry.coordinates.map(c=>[c[1],c[0]]),mn=r=>(r.duration/60).toFixed(1),km=d=>d<1000?Math.round(d)+' m':(d/1000).toFixed(1)+' km',sd=d=>(d>=0?'+':'-')+km(Math.abs(d));
const c=R.filter(r=>!r.hits&&!r.blk).sort((x,y)=>x.duration-y.duration)[0],extra=c&&c!==b&&c!==f;
if(f!==b)rl.push(L.polyline(ll(f),{color:'#6b7280',weight:5,dashArray:'8 8'}).addTo(map));
if(extra)rl.push(L.polyline(ll(c),{color:'#16a34a',weight:6,dashArray:'2 10'}).addTo(map));
rl.push(L.polyline(ll(b),{color:b.blk?'#dc2626':'#2563eb',weight:7,opacity:.85}).addTo(map));
let h='<b>Fastest:</b> '+mn(f)+' min, '+km(f.distance)+', '+f.hits+' hazard zone(s)'+(f.blk?' - BLOCKED':'');
if(f!==b)h+='<br><b>Recommended:</b> '+mn(b)+' min, '+km(b.distance)+' ('+sd(b.distance-f.distance)+', +'+((b.duration-f.duration)/60).toFixed(1)+' min), '+b.hits+' zone(s)'+(b.blk?' - BLOCKED':'');
if(extra)h+='<br><b style="color:#16a34a">Hazard-free route:</b> '+mn(c)+' min, '+km(c.distance)+' ('+sd(c.distance-f.distance)+', +'+((c.duration-f.duration)/60).toFixed(1)+' min)';
else if(f===b)h+=f.hits||f.blk?'<br>No clear alternative found':'<br>Fastest route is clear';
if(ign)h+='<br><i>'+ign+' hazard(s) at the start/destination ignored</i>';
$('rp').innerHTML=h}
async function detour(b,hz){busy=true;$('rp').innerHTML+='<br><i>Searching for a detour...</i>';
try{const g=b.geometry.coordinates,via=[];
for(const hp of b.hz.slice(0,2)){let bi=0,bd=1e9;g.forEach((q,i)=>{const d=ptd([q[1],q[0]],hp);if(d<bd){bd=d;bi=i}});
const a=g[Math.max(0,bi-3)],e=g[Math.min(g.length-1,bi+3)],k=Math.cos(hp[0]*Math.PI/180),dx=(e[0]-a[0])*k,dy=e[1]-a[1],l=Math.hypot(dx,dy)||1,nx=-dy/l,ny=dx/l;
for(const off of[200,400])for(const sg of[1,-1])via.push([hp[0]+sg*ny*off/111320,hp[1]+sg*nx*off/(111320*k)])}
for(const v of via){if(!S||!E)break;
const j=await(await fetch(OSRM+S.lng+','+S.lat+';'+v[1]+','+v[0]+';'+E.lng+','+E.lat+'?overview=full&geometries=geojson')).json();
const r=j.routes&&j.routes[0];
if(r&&!R.some(q=>Math.abs(q.duration-r.duration)<1&&Math.abs(q.distance-r.distance)<5)){R.push(r);evalR(r,hz);if(!r.hits)break}
await new Promise(t=>setTimeout(t,1100))}
}catch(x){}
busy=false;score()}
async function route(){if(!S||!E)return;$('rp').textContent='Finding routes...';
try{const j=await(await fetch(OSRM+S.lng+','+S.lat+';'+E.lng+','+E.lat+'?overview=full&geometries=geojson&alternatives=3')).json();
if(!j.routes||!j.routes.length)throw 0;R=j.routes;dk='';score()}
catch(x){$('rp').textContent=R.length?'Routing server unreachable - showing last routes':'Routing server unreachable (internet needed)'}}
function pinAt(ll,c,t){return L.marker(ll,{icon:L.divIcon({className:'',html:'<div class="pin '+c+'" style="color:#fff;text-align:center;font:700 11px/14px system-ui">'+t+'</div>',iconSize:[20,20]})}).addTo(map)}
function setS(ll){S=ll;sm&&map.removeLayer(sm);sm=pinAt(ll,'st','A')}
function clr(){[sm,em].forEach(m=>m&&map.removeLayer(m));sm=em=S=E=null;R=[];dk='';rl.forEach(l=>map.removeLayer(l));rl=[];$('rp').textContent='Tap the map to set start (A), then destination (B)'}
map.on('click',e=>{if(T){const z={type:T,lat:e.latlng.lat,lon:e.latlng.lng,n:2,sp:0,test:true,c:L.circle(e.latlng).addTo(map)};Z.push(z);sty(z);dirty=true;T=0;$('tp').textContent='Test pothole';$('tb').textContent='Test bump';return}if(!S)setS(e.latlng);else if(!E){E=e.latlng;em=pinAt(E,'en','B');route()}else{clr();setS(e.latlng)}});
$('sp').onclick=()=>{if(!pod)return;const l=pod.getLatLng();if(E){const e=E;clr();E=e;em=pinAt(E,'en','B')}setS(l);if(E)route()};
$('cr').onclick=clr;
$('tp').onclick=()=>{T=1;$('tp').textContent='Tap the map...'};$('tb').onclick=()=>{T=2;$('tb').textContent='Tap the map...'};
tick();setInterval(tick,1000);
</script></body></html>)HTML";

String stateJson(unsigned long now) {
  String j; j.reserve(6000);
  j += "{\"ms\":"; j += (unsigned long)now; j += ",\"up\":";    j += (unsigned long)(now / 1000);
  j += ",\"scout\":"; j += (lastRx && now - lastRx < 6000) ? "true" : "false";
  j += ",\"rssi\":";  j += WiFi.RSSI();
  j += ",\"alert\":"; j += alertType;
  j += ",\"pot\":";   j += cntPot;
  j += ",\"bump\":";  j += cntBump;
  j += ",\"pod\":{\"fix\":"; j += podFix;
  j += ",\"lat\":";   j += String(podLat, 6);
  j += ",\"lon\":";   j += String(podLon, 6);
  j += ",\"kmh\":";   j += String(podSpeed, 1);
  j += "},\"events\":[";
  int n = evN < EVMAX ? evN : EVMAX;
  for (int i = 0; i < n; i++) {
    const Ev &e = evs[(evN - 1 - i) % EVMAX];
    if (i) j += ',';
    j += "{\"seq\":";  j += e.seq;
    j += ",\"type\":"; j += e.type;
    j += ",\"conf\":"; j += e.conf;
    j += ",\"fix\":";  j += e.fix;
    j += ",\"lat\":";  j += String(e.lat, 6);
    j += ",\"lon\":";  j += String(e.lon, 6);
    j += ",\"peak\":"; j += String(e.peak, 2);
    j += ",\"ago\":";  j += (unsigned long)(now - e.t);
    j += '}';
  }
  j += "]}";
  return j;
}

void setupWeb() {
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/map", []() { server.send_P(200, "text/html", PAGEMAP); });
  server.on("/api/state", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", stateJson(millis()));
  });
  server.on("/api/reset", []() {
    evN = 0; cntPot = 0; cntBump = 0; clearAlert();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "ok");
  });
  server.on("/api/ntfytest", []() {
    ntfyPush("Road Guard test", "Test notification from Node 2. If you see this, ntfy works.", 3, "white_check_mark", "");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "test sent");
  });
  server.begin();
}

// ---------- serial commands ----------
void handleCmd(String s) {
  s.trim();
  if (!s.length()) return;
  if (s == "pol low" || s == "pol high" || s == "pol open") {
    activeLow = (s != "pol high");
    openDrain = (s == "pol open");
    pinMode(RELAY_PIN, OUTPUT);
    prefs.begin("n2", false);
    prefs.putBool("low", activeLow);
    prefs.putBool("open", openDrain);
    prefs.end();
    stopPattern();
    Serial.printf("relay mode = %s\n", openDrain ? "OPEN (active-LOW, pin released for OFF)" :
                  activeLow ? "active-LOW (ON when pin LOW)" : "active-HIGH (ON when pin HIGH)");
  }
  else if (s == "beep") startPattern(POT_P, 1, false);
  else if (s == "ntfy") { ntfyPush("Road Guard test", "Test notification from Node 2. If you see this, ntfy works.", 3, "white_check_mark", ""); Serial.println("ntfy test queued"); }
  else if (s == "alarm") { alertType = 4; alertSince = millis(); alertSeg = 1; startPattern(ALARM_P, 2, true); }
  else if (s == "clear") { clearAlert(); Serial.println("cleared"); }
  else if (s == "status") {
    Serial.printf("last packet %lu ms ago, polarity=%s, potholes=%d bumps=%d\n",
                  lastRx ? millis() - lastRx : 0, activeLow ? "active-LOW" : "active-HIGH", cntPot, cntBump);
    if (WiFi.status() == WL_CONNECTED)
      Serial.printf("Wi-Fi OK  IP=%s  RSSI=%d  channel=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel());
    else Serial.println("Wi-Fi NOT connected");
  }
  else Serial.println("unknown command");
}

// ---------- main ----------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  prefs.begin("n2", false);
  activeLow = prefs.getBool("low", true);
  openDrain = prefs.getBool("open", false);
  prefs.end();
  pinMode(RELAY_PIN, OUTPUT);
  relaySet(false);                    // buzzer OFF at boot

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED not found at 0x3C");
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20); display.println("ROAD GUARD");
  display.println("Starting...");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);               // keeps ESP-NOW reception reliable while on Wi-Fi
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to hotspot '%s' ", WIFI_SSID);
  display.clearDisplay(); display.setCursor(0, 0);
  display.println("Connecting Wi-Fi..."); display.println(WIFI_SSID); display.display();
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(300); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWi-Fi OK. Open on your phone:  http://%s/  (channel %d)\n",
                  WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    Serial.println("\nWi-Fi FAILED (will keep retrying). Check name/password and that the hotspot is 2.4 GHz.");
  }
  setupWeb();
  ntfyQ = xQueueCreate(4, sizeof(NtfyMsg));
  xTaskCreate(ntfyTask, "ntfy", 12288, NULL, 1, NULL);

  if (esp_now_init() != ESP_OK) Serial.println("ESP-NOW init FAILED");
  esp_now_register_recv_cb(onRecv);
  Serial.print("Node 2 MAC: "); Serial.println(WiFi.macAddress());
  Serial.printf("Relay mode: %s. If the buzzer is stuck ON, try 'pol high', then 'pol open'.\n",
                openDrain ? "OPEN" : activeLow ? "active-LOW" : "active-HIGH");

  Serial.println("Ready. Buzzer sounds only for SOS. Type 'beep' to test.");
}

void loop() {
  unsigned long now = millis();
  if (Serial.available()) handleCmd(Serial.readStringUntil('\n'));
  server.handleClient();

  static unsigned long lastWifiTry = 0;
  if (WiFi.status() != WL_CONNECTED && now - lastWifiTry > 10000) {
    lastWifiTry = now;
    WiFi.reconnect();
  }

  while (qt != qh) {
    Packet p = q[qt];
    qt = (qt + 1) & 7;
    handlePacket(p, now);
  }

  // timeouts
  if ((alertType == 1 || alertType == 2) && now - alertSince > SHORT_ALERT_MS) clearAlert();
  if (alarmActive() && now - alertSince > ALARM_TIMEOUT_MS) clearAlert();

  buzzerService(now);

  static unsigned long lastDraw = 0;
  if (now - lastDraw > 150) { lastDraw = now; drawScreen(now); }
}
