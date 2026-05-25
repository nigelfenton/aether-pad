// ###########################################################################
// #                                                                         #
// #   BUILD MARKER:  SPLASH / PERSONA MENU v5  (boot-phase status strip)    #
// #                                                                         #
// #   Boot -> splash screen with 3 persona buttons (TCI / RC-28 /           #
// #   AetherControl) and a 3 s auto-continue (defaults to TCI for now).    #
// #   Tap any button on splash = enter that persona's operator screen.     #
// #   Tap the title bar on the operator screen = re-open the splash as a   #
// #   menu (auto-continues back to current persona on no-tap).             #
// #   10m tile long-press still enters TEST mode (unchanged).               #
// #                                                                         #
// ###########################################################################

/*
 * aether_pad.ino
 * Standalone AetherSDR control surface — Arduino Giga R1 WiFi + Display Shield
 * G0JKN/W3 — shack-experiments
 *
 * Hardware
 *   Arduino Giga R1 WiFi
 *   Giga Display Shield (800×480 landscape, GT911 touch, USB-C on right edge)
 *   Optical encoder (NPN open-collector — pick type below via #define):
 *     LPD3806-600BM-G5-24C : 600 PPR, 5-24 V supply  → power from Giga 5 V pin
 *     Autonics E50S-360B   : 360 PPR, 12-24 V supply → power from shack 13.8 V railyou stiring up trouble?
 
 *     A → D2  (10 kΩ external pull-up to 3.3 V — Giga INPUT_PULLUP alone is too weak)
 *     B → D3  (10 kΩ external pull-up to 3.3 V — Giga INPUT_PULLUP alone is too weak)
 *     +V → 5 V (LPD3806) or external 13.8 V (Autonics — LED needs 12-24 V)
 *     0 V → shared GND with Giga
 *
 *   3.5 mm stereo TRS jack for CW iambic paddle:
 *     tip    (dit) → D4  (INPUT_PULLUP, paddle closes to GND)
 *     ring   (dah) → D5  (INPUT_PULLUP, paddle closes to GND)
 *     sleeve       → Giga GND
 *   Straight-key operators wire their key directly to the radio's CW
 *   jack — bypasses the pad/WiFi entirely for sub-ms keying response.
 *
 * UI (800×480 landscape) — modal-encoder design (redesigned 2026-05-07)
 *   Title bar : "AETHER_PAD G0JKN/W3" + host info + LIVE/.../SCAN pill
 *   Freq      : big centred VFO frequency — TAP to arm encoder for VFO
 *   Chips     : three modal status cards, TAP any to arm the encoder
 *                  [ VOLUME ]   [ MODE ]   [ STEP ]
 *                The currently-armed chip glows cyan. The encoder always
 *                works; what it controls follows the last chip you tapped.
 *   Status    : thin strip showing current band, encoder target, debug info
 *   Bands     : 10 direct band tiles in 5×2 grid — TAP to jump to band's
 *                last-known frequency (RAM band-stack, sensible defaults).
 *                Active band (containing the current VFO) glows cyan.
 *                  [160m][80m][60m][40m][30m]
 *                  [ 20m][17m][15m][12m][10m]
 *   Encoder   : modal — VFO/VOL/MODE/STEP per active chip. Default boot
 *                target is VFO so out-of-the-box behaviour is unchanged.
 *
 * Network
 *   WiFi    : tinkerbell
 *   Discovery (outbound, find AetherSDR):
 *              UDP probe "AETHERPAD?" → port 40002, listens for unicast reply
 *              "AETHERSDR ip=<addr> tci=<port>". Falls back to 10.0.0.107:40001
 *              after 5 s if no responder is on the LAN yet.
 *   Discovery (inbound, advertise self):
 *              mDNS / DNS-SD under _tci._tcp.local per
 *              github.com/ten9876/AetherSDR/docs/tci-discovery.md.  Class
 *              "controller", model "aether-pad".  SRV port is 0 — aether_pad
 *              is a TCI client, not a server.  See mdns_tci.h/.cpp.
 *   Control : TCI WebSocket client to ws://<host>:<port>/ — hand-rolled framing
 *             (same minimal approach as ShackController firmware).
 *
 * Serial commands (115200 baud)
 *   ?              Show state
 *   ip <a.b.c.d>   Override AetherSDR IP and reconnect
 *   div NN         Set encoder divider (raw counts per VFO step), default 24
 *   step NN        Set encoder VFO step in Hz, default 100 (buttons fixed 10 kHz)
 *   vol N          Set volume directly (0-100, AetherSDR audioGain percent)
 *   freq NNN       Set frequency directly (Hz)
 *   tm 0..3        Touch: 0=default (verified), 1=180°, 2=raw, 3=flip-both
 *   wpm N          CW keyer speed (5..100 WPM, default 25)
 *   iambic A|B     Iambic mode (default B — opposite-paddle memory)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "Arduino_H7_Video.h"
#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"
#include "mdns_tci.h"
#include "rc28_hid.h"

// USB HID device — impersonates Icom RC-28 for in-bench testing of
// AetherSDR's IcomRC28Parser (PR ten9876/AetherSDR#2870). Enumerated at
// boot; reports are only sent when the operator has entered TEST mode
// from the touchscreen (long-press on the 10m band tile). NORMAL mode
// leaves the device enumerated but silent so the host sees no spurious
// tuning events while aether-pad is in everyday TCI use.
AetherPad::Rc28Hid rc28;

// ---------------------------------------------------------------------------
// Build identification — printed at boot and in `?` output so you can
// confirm the running firmware matches your latest reflash.  __DATE__ and
// __TIME__ are filled in by the compiler at build time.
// ---------------------------------------------------------------------------
#define BUILD_TAG  __DATE__ " " __TIME__

// ---------------------------------------------------------------------------
// Encoder type — pick ONE, comment out the other
// ---------------------------------------------------------------------------
#define ENCODER_LPD3806            // 600 PPR, 5-24 V (small 38mm body, runs off Giga 5 V)
//#define ENCODER_AUTONICS_E50S    // 360 PPR, 12-24 V (needs shack 13.8 V rail)

#if defined(ENCODER_LPD3806)
  const char* ENC_NAME        = "LPD3806-600BM (600 PPR, 5-24 V)";
  const int   ENC_DIV_DEFAULT = 40;   // 600 PPR × 40 ≈ same feel as Autonics × 24
#elif defined(ENCODER_AUTONICS_E50S)
  const char* ENC_NAME        = "Autonics E50S-360B (360 PPR, 12-24 V)";
  const int   ENC_DIV_DEFAULT = 24;
#else
  #error "Pick an encoder: define ENCODER_LPD3806 or ENCODER_AUTONICS_E50S"
#endif

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
const char* WIFI_SSID = "tinkerbell";
const char* WIFI_PASS = "disneybell";

const uint16_t  DISCOVERY_PORT      = 40002;
const char*     DISCOVERY_PROBE     = "AETHERPAD?";
const uint32_t  DISCOVERY_INTERVAL  = 2000;   // ms between probes while not connected
const uint32_t  DISCOVERY_FALLBACK  = 5000;   // ms before falling back to hardcoded IP
IPAddress       FALLBACK_HOST(10, 0, 0, 107);
const uint16_t  FALLBACK_PORT       = 40001;

// VFO / encoder
const int  ENC_PIN_A = 2;
const int  ENC_PIN_B = 3;
int        encDivider     = ENC_DIV_DEFAULT;  // raw quadrature counts per emitted step
long       vfoStepHz      = 100;   // step per emitted encoder click & per VFO± tap (long-press cycles)
const unsigned long LONG_PRESS_MS = 600;   // touch-hold duration that triggers long-press

// CW iambic keyer — paddle pin assignment matches Nigel's paddle wiring
// (tip→dah, ring→dit) so left=dit, right=dah feels natural. If your
// paddle is wired the other way, swap KEY_PIN_DIT and KEY_PIN_DAH.
// Straight-key operators wire their key directly to the radio's CW
// jack instead of through the pad — sub-ms response and no WiFi-
// dependence (tested 2026-05-08: straight key over WiFi was unusable).
const int  KEY_PIN_DIT = 5;     // paddle ring → D5 (INPUT_PULLUP, closes to GND)
const int  KEY_PIN_DAH = 4;     // paddle tip  → D4 (INPUT_PULLUP, closes to GND)
const int  KEY_PIN_OUT = 6;     // hardware key output → D6 (active HIGH)
                                // Wire D6 to a 2N3904 base via 1 kΩ, emitter to GND,
                                // collector to the radio's CW key jack tip.  D6 HIGH
                                // saturates the transistor, pulling the key line to
                                // GND (= key down).  Optocoupler (PC817 / 4N25) is
                                // safer if the radio's key input shares ground with
                                // its high-voltage rails — see README for circuit.
int        cwWpm       = 25;    // 5-100 WPM, advertised to AetherSDR via cw_keyer_speed
char       iambicMode  = 'B';   // 'A' or 'B'; B is the most common (memory of opposite paddle)

// Mode rotation
const char* MODES[] = {"am","sam","dsb","lsb","usb","cw","digl","digu","fm","dfm","spec","rtty"};
const int   N_MODES = 12;

// Volume range — AetherSDR's TCI volume is per-slice audioGain, 0-100 percent
// (see TciProtocol.cpp:741). Step matches AetherSDR's own UI click increment.
const int VOL_MIN  = 0;
const int VOL_MAX  = 100;
const int VOL_STEP = 5;

// VFO step rotation — used when encoder is armed for STEP target
const long STEP_CYCLE[] = {10L, 100L, 1000L, 10000L, 100000L};
const int  N_STEPS = 5;

// ---------------------------------------------------------------------------
// Modal encoder target — the encoder always works; what it controls depends
// on which on-screen chip the user last tapped. Default after boot is VFO so
// the device behaves as before until a chip is touched.
// ---------------------------------------------------------------------------
enum EncoderTarget : uint8_t {
    TGT_VFO = 0,
    TGT_VOL,
    TGT_MODE,
    TGT_STEP,
    TGT_KEYER       // CW keyer WPM
};
EncoderTarget encTarget = TGT_VFO;

// Rate-limit non-VFO encoder dispatches. Smooth optical encoders have no
// detents, so a quick flick can spit out 5-10 steps before the operator
// reacts. For VFO that's desirable (fast tuning); for STEP/MODE/VOL/WPM
// it makes the chip feel out of control. 400 ms feels deliberate without
// being sluggish; combined with ENC_NONVFO_DIV_MULT it forces the operator
// to commit to a real rotation before each emission.
const unsigned long ENC_NONVFO_MIN_MS    = 400;
// Multiplier on encDivider when armed for non-VFO targets — requires
// ENC_NONVFO_DIV_MULT × encDivider raw counts per emitted step. With the
// default LPD3806 divider of 40, that's 80 counts per step → roughly half
// a turn of the knob. Plenty of feedback before each change.
const int           ENC_NONVFO_DIV_MULT  = 2;

// ---------------------------------------------------------------------------
// Band stack — one entry per band tile shown on the bottom of the screen.
// `lastHz` is the most recent frequency the operator was on within that band;
// it's seeded with a sensible centre and gets updated whenever the VFO moves
// inside the band. RAM-only for now; persistence (KVStore on Giga) is a TODO.
// ---------------------------------------------------------------------------
struct BandEntry {
    const char* name;
    long        loHz;     // band edge low (used for VFO→band tracking)
    long        hiHz;     // band edge high
    long        lastHz;   // last operator frequency within this band
};
BandEntry bandStack[] = {
    {"160m",  1810000L,  2000000L,  1840000L},
    {"80m",   3500000L,  4000000L,  3700000L},
    {"60m",   5260000L,  5410000L,  5330000L},
    {"40m",   7000000L,  7300000L,  7150000L},
    {"30m",  10100000L, 10150000L, 10125000L},
    {"20m",  14000000L, 14350000L, 14250000L},
    {"17m",  18068000L, 18168000L, 18118000L},
    {"15m",  21000000L, 21450000L, 21250000L},
    {"12m",  24890000L, 24990000L, 24940000L},
    {"10m",  28000000L, 29700000L, 28500000L}
};
const int N_BANDS = sizeof(bandStack) / sizeof(bandStack[0]);

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
Arduino_H7_Video         Display(800, 480, GigaDisplayShield);
GigaDisplay_GFX          tft;
Arduino_GigaDisplayTouch Touch;

// AetherSDR-themed RGB565 palette
#define C_BG     0x0810   // dark navy  #0d1628
#define C_PANEL  0x040C
#define C_CYAN   0x07FF
#define C_AMBER  0xFD20
#define C_RED    0xF800
#define C_GREEN  0x07E0
#define C_MUTED  0x4A49
#define C_WHITE  0xFFFF
#define C_BORDER 0x1C2A

// ---------------------------------------------------------------------------
// Live state mirrored from TCI feedback
// ---------------------------------------------------------------------------
struct State {
    bool      tciConnected = false;
    bool      hostKnown    = false;
    IPAddress host;
    uint16_t  port = FALLBACK_PORT;

    // Mirrored values
    long      freqHz   = 14250000;
    int       modeIdx  = 4;     // index into MODES[] — default usb
    int       volume   = 50;    // 0-100 percent (AetherSDR audioGain)
    char      bandName[8] = "20m";
} st;

// ---------------------------------------------------------------------------
// Networking
// ---------------------------------------------------------------------------
WiFiUDP    udp;
WiFiClient tcp;     // raw TCP, used as the WebSocket transport
bool       wsHandshakeDone = false;
String     wsRxBuf;     // accumulated raw bytes from the WS socket
unsigned long lastDiscoveryTx = 0;
unsigned long discoveryStarted = 0;
bool       fallbackArmed = false;

// mDNS / DNS-SD advertiser — announces aether_pad on the LAN per the
// _tci._tcp.local schema (ten9876/AetherSDR docs/tci-discovery.md).  Brought
// up in setup() after WiFi associates; polled from loop().  Owns its own
// WiFiUDP socket bound to 5353 — independent of `udp` above (which handles
// the existing AETHERPAD?/40002 outbound discovery probe).
TciMdnsAdvertiser mdns;

// ---------------------------------------------------------------------------
// HTTP status / control web UI (port 80)
// ---------------------------------------------------------------------------
WiFiServer    httpServer(80);
WiFiClient    httpClient;
String        httpReq;
unsigned long httpStart = 0;
const unsigned long HTTP_TIMEOUT_MS = 1500;
const size_t        HTTP_REQ_MAX    = 1024;

const char HTML_PAGE[] = R"HTML(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>aether_pad — G0JKN</title>
<style>
:root{--bg:#0a1424;--panel:#0d1c33;--border:#1c2a44;--cyan:#00d4ff;
 --amber:#ffb020;--green:#10c060;--red:#ff4040;--muted:#5a6680;--white:#e8f0ff;}
*{box-sizing:border-box;margin:0;padding:0;}
body{background:var(--bg);color:var(--white);
 font:14px/1.4 system-ui,-apple-system,sans-serif;padding:14px;min-height:100vh;
 max-width:900px;margin:0 auto;}
h1{color:var(--cyan);font-size:18px;letter-spacing:1px;text-transform:uppercase;}
.sub{color:var(--muted);font-size:11px;margin:2px 0 14px;letter-spacing:1px;}
.pill{display:inline-block;padding:3px 10px;border-radius:4px;font-size:11px;
 font-weight:bold;letter-spacing:1px;background:#0a1830;color:var(--muted);
 border:1px solid var(--border);margin-left:8px;}
.pill.on{background:#082916;color:var(--green);border-color:var(--green);}
.pill.warn{background:#3a2a08;color:var(--amber);border-color:var(--amber);}
.hero{background:var(--panel);border:1px solid var(--cyan);border-radius:8px;
 padding:18px;margin-bottom:14px;text-align:center;}
.freq{font-size:54px;font-weight:bold;color:var(--cyan);font-variant-numeric:tabular-nums;
 letter-spacing:2px;line-height:1;}
.freq-unit{color:var(--muted);font-size:13px;letter-spacing:2px;margin-top:6px;}
.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-bottom:14px;}
@media(max-width:600px){.cards{grid-template-columns:repeat(2,1fr);}}
.card{background:var(--panel);border:1px solid var(--border);border-radius:6px;
 padding:10px 12px;}
.card .lbl{color:var(--muted);font-size:10px;letter-spacing:2px;text-transform:uppercase;}
.card .val{font-size:22px;font-weight:bold;font-variant-numeric:tabular-nums;
 line-height:1;margin-top:6px;}
.mode .val{color:var(--amber);} .vol .val{color:var(--green);}
.band .val{color:var(--cyan);} .step .val{color:var(--white);}
section{background:var(--panel);border:1px solid var(--border);border-radius:6px;
 padding:14px;margin-bottom:14px;}
section h2{color:var(--muted);font-size:11px;letter-spacing:2px;text-transform:uppercase;
 margin-bottom:10px;}
.row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin:6px 0;}
button{flex:1;min-width:60px;padding:10px;background:#0a1830;
 border:1px solid var(--border);color:var(--white);border-radius:5px;
 font:bold 11px/1 system-ui;letter-spacing:1px;cursor:pointer;transition:all .15s;}
button:hover{background:#10243f;border-color:var(--cyan);color:var(--cyan);}
button.amber:hover{border-color:var(--amber);color:var(--amber);}
button.green:hover{border-color:var(--green);color:var(--green);}
input[type=number],input[type=text],select{background:#0a1830;color:var(--white);
 border:1px solid var(--border);border-radius:4px;padding:8px;font:inherit;
 font-variant-numeric:tabular-nums;}
input[type=number]{width:120px;}
.diag{font-size:11px;color:var(--muted);font-family:ui-monospace,monospace;
 padding:8px;background:#04101e;border-radius:4px;}
.status{margin-top:10px;font-size:11px;color:var(--muted);text-align:center;
 letter-spacing:1px;}
.status.live{color:var(--green);}
</style></head>
<body>
<h1>aether_pad <span class="pill" id="pill">…</span></h1>
<div class="sub" id="sub">G0JKN · Arduino Giga R1 WiFi</div>

<div class="hero">
 <div class="freq" id="freq">—</div>
 <div class="freq-unit">MHz</div>
</div>

<div class="cards">
 <div class="card mode"><div class="lbl">Mode</div><div class="val" id="mode">—</div></div>
 <div class="card vol"><div class="lbl">Volume</div><div class="val" id="vol">—</div></div>
 <div class="card band"><div class="lbl">Band</div><div class="val" id="band">—</div></div>
 <div class="card step"><div class="lbl">Step</div><div class="val" id="step">—</div></div>
</div>

<section>
 <h2>VFO</h2>
 <div class="row">
  <button onclick="cmd('vfo_down')">− 10 kHz</button>
  <input type="number" id="freqIn" placeholder="Frequency Hz">
  <button onclick="setFreq()">Tune</button>
  <button onclick="cmd('vfo_up')">+ 10 kHz</button>
 </div>
</section>

<section>
 <h2>Mode</h2>
 <div class="row">
  <button class="amber" onclick="cmd('mode_down')">−</button>
  <select id="modeSel" onchange="setMode()">
   <option value="am">AM</option><option value="sam">SAM</option>
   <option value="lsb">LSB</option><option value="usb">USB</option>
   <option value="cw">CW</option><option value="cwr">CWR</option>
   <option value="digl">DIGL</option><option value="digu">DIGU</option>
   <option value="fm">FM</option><option value="nfm">NFM</option>
   <option value="rtty">RTTY</option>
  </select>
  <button class="amber" onclick="cmd('mode_up')">+</button>
 </div>
</section>

<section>
 <h2>Volume</h2>
 <div class="row">
  <button class="green" onclick="cmd('vol_down')">−</button>
  <input type="number" id="volIn" min="0" max="100" step="5">
  <button class="green" onclick="setVol()">Set</button>
  <button class="green" onclick="cmd('vol_up')">+</button>
 </div>
</section>

<section>
 <h2>Band</h2>
 <div class="row">
  <button onclick="cmd('band_down')">− Band</button>
  <button onclick="cmd('band_up')">+ Band</button>
 </div>
</section>

<section>
 <h2>Encoder Step</h2>
 <div class="row">
  <button onclick="setStep(10)">10 Hz</button>
  <button onclick="setStep(100)">100 Hz</button>
  <button onclick="setStep(1000)">1 kHz</button>
  <button onclick="setStep(10000)">10 kHz</button>
 </div>
</section>

<section>
 <h2>Diagnostics</h2>
 <div class="diag" id="diag">—</div>
</section>

<div class="status" id="st">Connecting…</div>

<script>
const fmtFreq = hz => {
 const m = Math.floor(hz/1000000);
 const k = Math.floor(hz/1000)%1000;
 const h = hz%1000;
 return m+'.'+String(k).padStart(3,'0')+'.'+String(h).padStart(3,'0');
};
const fmtStep = hz => hz>=1000 ? (hz/1000)+' kHz' : hz+' Hz';

async function poll(){
 try{
  const r=await fetch('/api/state',{cache:'no-store'});
  if(!r.ok) throw 0;
  const s=await r.json();
  document.getElementById('freq').textContent=fmtFreq(s.freq);
  document.getElementById('mode').textContent=(s.mode||'').toUpperCase();
  document.getElementById('vol').textContent=s.vol+' %';
  document.getElementById('band').textContent=s.band||'—';
  document.getElementById('step').textContent=fmtStep(s.step);
  const sel=document.getElementById('modeSel');
  if(sel.value!==s.mode) sel.value=s.mode;
  const pill=document.getElementById('pill');
  if(s.tci){pill.textContent='LIVE';pill.className='pill on';}
  else if(s.host){pill.textContent='…';pill.className='pill warn';}
  else{pill.textContent='SCAN';pill.className='pill';}
  document.getElementById('sub').textContent=
   s.host ? ('AetherSDR @ '+s.host+':'+s.port) : 'Searching for AetherSDR…';
  document.getElementById('diag').textContent=
   `enc raw ${s.encRaw}  ISRs ${s.encIsrs}  div ${s.div}  touch ${s.tm}`;
  const st=document.getElementById('st');
  st.textContent='LIVE'; st.className='status live';
 }catch(e){
  const st=document.getElementById('st');
  st.textContent='Connection lost'; st.className='status';
 }
}
async function cmd(op,v){
 const url='/api/cmd?op='+op+(v!==undefined?'&v='+encodeURIComponent(v):'');
 try{ await fetch(url,{cache:'no-store'}); }catch(e){}
 poll();
}
function setFreq(){const v=document.getElementById('freqIn').value; if(v) cmd('freq',v);}
function setMode(){cmd('mode',document.getElementById('modeSel').value);}
function setVol(){const v=document.getElementById('volIn').value; if(v) cmd('vol',v);}
function setStep(v){cmd('step',v);}
setInterval(poll,400);
poll();
</script>
</body></html>
)HTML";

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
bool uiNeedsRedraw = true;

struct Btn { int x, y, w, h; const char* lbl; };

// New layout (800×480 landscape):
//
//   y=  0.. 30  title bar (AETHER_PAD G0JKN · host · status pill)
//   y= 36..100  big VFO frequency — tap to arm encoder for VFO
//   y=104..184  three modal status chips (VOL · MODE · STEP) — tap to arm
//   y=190..210  status text strip (band, encoder target, debug)
//   y=216..340  band tiles row 1 (160 / 80 / 60 / 40 / 30)
//   y=346..470  band tiles row 2 (20 / 17 / 15 / 12 / 10)
//
// All four "chips" (the freq area + the three status cards) double as touch
// targets that re-aim the encoder. The currently-armed chip glows cyan.
// All ten band tiles jump straight to that band's last-known frequency.
//
// Note: ASCII hyphen-minus only — the GFX font has no glyph for U+2212 ("−");
// keep all strings hitting tft.print() ASCII.

// Tappable freq display — the whole big number is one big button
static Btn btnFreqChip = {  0, 36, 800, 64, "VFO" };

// Four modal chips — VOL / MODE / STEP / KEYER. Each 196 wide × 80 tall,
// 4 px gaps. Total = 4 + 196 + 4 + 196 + 4 + 196 + 4 + 196 + 4 = 800.
static Btn btnVolChip   = {  4, 104, 196, 80, "VOL"   };
static Btn btnModeChip  = {204, 104, 196, 80, "MODE"  };
static Btn btnStepChip  = {404, 104, 196, 80, "STEP"  };
static Btn btnKeyerChip = {604, 104, 196, 80, "KEYER" };

// Bottom-row tile slots — 10 fixed positions in a 5×2 grid, 800×254 area
// starting at y=216. The tile CONTENT (label + action) depends on the
// current `bottomRow` mode; the tile coords are shared across all modes.
static Btn btnTile[10] = {
    {  4, 216, 156, 124, ""},
    {164, 216, 156, 124, ""},
    {324, 216, 156, 124, ""},
    {484, 216, 156, 124, ""},
    {644, 216, 156, 124, ""},
    {  4, 346, 156, 124, ""},
    {164, 346, 156, 124, ""},
    {324, 346, 156, 124, ""},
    {484, 346, 156, 124, ""},
    {644, 346, 156, 124, ""}
};

// Bottom-row mode — which "tool tray" the operator is looking at. Default
// after boot is BANDS (matches pre-redesign behaviour). Tapping a chip
// changes both `encTarget` (encoder target) and `bottomRow` (tray
// content) so the bottom area reveals controls relevant to that chip.
enum BottomRowMode : uint8_t {
    ROW_BANDS = 0,
    ROW_MODE,
    ROW_KEYER
};
BottomRowMode bottomRow = ROW_BANDS;

// Tile actions — what a tap on a bottom-row tile does. The exact effect
// depends on TileData::arg (band index, mode index, WPM value, etc.).
enum TileAction : uint8_t {
    ACT_NONE = 0,
    ACT_BAND_SELECT,
    ACT_MODE_SET,
    ACT_WPM_SET,           // legacy preset (no longer used in default tray)
    ACT_IAMBIC_A,
    ACT_IAMBIC_B,
    ACT_DISPLAY_WPM        // live "N WPM" indicator — encoder adjusts it,
                           //   tap is a no-op (display only). Glows when
                           //   the KEYER tray is the active row.
};

// Tile content for each row mode. 10 slots each — empty label + ACT_NONE
// renders as a disabled / reserved slot (drawn but not clickable).
struct TileData {
    const char* label;
    TileAction  action;
    int         arg;
};

// ROW_BANDS — index in bandStack[] is the arg
static const TileData kBandData[10] = {
    {"160m", ACT_BAND_SELECT, 0},
    {"80m",  ACT_BAND_SELECT, 1},
    {"60m",  ACT_BAND_SELECT, 2},
    {"40m",  ACT_BAND_SELECT, 3},
    {"30m",  ACT_BAND_SELECT, 4},
    {"20m",  ACT_BAND_SELECT, 5},
    {"17m",  ACT_BAND_SELECT, 6},
    {"15m",  ACT_BAND_SELECT, 7},
    {"12m",  ACT_BAND_SELECT, 8},
    {"10m",  ACT_BAND_SELECT, 9}
};

// ROW_MODE — arg is the index into MODES[]
//   MODES[] order: am(0) sam(1) dsb(2) lsb(3) usb(4) cw(5) digl(6) digu(7) fm(8) dfm(9) spec(10) rtty(11)
static const TileData kModeData[10] = {
    {"LSB",  ACT_MODE_SET, 3},
    {"USB",  ACT_MODE_SET, 4},
    {"CW",   ACT_MODE_SET, 5},
    {"AM",   ACT_MODE_SET, 0},
    {"FM",   ACT_MODE_SET, 8},
    {"SAM",  ACT_MODE_SET, 1},
    {"DSB",  ACT_MODE_SET, 2},
    {"DIGL", ACT_MODE_SET, 6},
    {"DIGU", ACT_MODE_SET, 7},
    {"RTTY", ACT_MODE_SET, 11}
};

// ROW_KEYER — single live WPM display (encoder adjusts it) + iambic
// toggles + reserved slots for future CW macros (CQ, 73, name, etc.).
// The WPM tile's label is rendered dynamically in drawDynamic — the
// "" placeholder here is overwritten with "<N> WPM" before drawing.
static const TileData kKeyerData[10] = {
    {"",       ACT_DISPLAY_WPM, 0},   // live WPM display
    {"",       ACT_NONE,        0},
    {"",       ACT_NONE,        0},
    {"",       ACT_NONE,        0},
    {"",       ACT_NONE,        0},
    {"IAMB A", ACT_IAMBIC_A,    0},
    {"IAMB B", ACT_IAMBIC_B,    0},
    {"",       ACT_NONE,        0},
    {"",       ACT_NONE,        0},
    {"",       ACT_NONE,        0}
};

// Look up the active tile data array for the current bottom-row mode.
static const TileData* currentTileRow() {
    switch (bottomRow) {
        case ROW_MODE:  return kModeData;
        case ROW_KEYER: return kKeyerData;
        case ROW_BANDS: default: return kBandData;
    }
}

// Button identity for the touch handler — needed so we can track which button
// is being held across multiple loop() iterations (for long-press detection).
enum BtnId : int8_t {
    BTN_NONE = -1,
    BTN_FREQ_CHIP,             // big VFO display — arm TGT_VFO
    BTN_VOL_CHIP,              // VOL card        — arm TGT_VOL
    BTN_MODE_CHIP,             // MODE card       — arm TGT_MODE
    BTN_STEP_CHIP,             // STEP card       — arm TGT_STEP
    BTN_KEYER_CHIP,            // KEYER card      — arm TGT_KEYER (WPM)
    BTN_TITLE,                 // title bar strip — long-press = persona carousel
    BTN_TILE_BASE              // bottom-row tiles 0..9 are BTN_TILE_BASE+idx
};
const int N_TILES = 10;

// Tile cache — remembers each tile's last-drawn (label, active) pair so
// drawDynamic can skip the fillRoundRect+text repaint when nothing has
// changed.  This is the cure for the visible flicker on the BAND tiles:
// in steady state nothing changes between 250 ms refreshes, so the cache
// hits and drawTile() never runs for those slots.  Invalidated by
// drawStatic() (and any code path that wipes the tile region) so the
// next drawDynamic forces every tile to redraw.
struct TileLastState {
    char  label[16];
    bool  active;
    bool  valid;
};
static TileLastState gTileLast[N_TILES] = {{0}};

static void invalidateTileCache() {
    for (int i = 0; i < N_TILES; i++) gTileLast[i].valid = false;
}

// ---------------------------------------------------------------------------
// Encoder (interrupt-driven, ISR-safe)
// ---------------------------------------------------------------------------
volatile long encRaw    = 0;     // raw quadrature count
volatile uint8_t encLast = 0;    // last A,B state (for direction detect)
volatile uint32_t encIsrCount = 0;  // diagnostic: bumps on every ISR fire

// ---------------------------------------------------------------------------
// CW keyer state — iambic mode B by default, polled from main loop every ms
// ---------------------------------------------------------------------------
enum KeyerState : uint8_t {
    KS_IDLE,
    KS_DIT_ON,    KS_DIT_GAP,
    KS_DAH_ON,    KS_DAH_GAP
};
struct KeyerCtx {
    KeyerState state    = KS_IDLE;
    uint32_t   stateAt  = 0;       // millis() when this state began
    bool       keyDown  = false;   // current TX state actually sent on the wire
    bool       memDit   = false;   // mode-B memory: dit was queued during a dah
    bool       memDah   = false;   // mode-B memory: dah was queued during a dit
    uint32_t   txCount  = 0;       // diagnostic: total elements sent
} kr;

// ---------------------------------------------------------------------------
// Forward declarations (avoid Arduino auto-prototype quirks with overloads)
// ---------------------------------------------------------------------------
void wifiConnect();
void discoveryTick();
void wsTick();
bool wsConnect();
void wsSendText(const char* s);
void wsSendText(const String& s);
void wsParseIncoming();
void parseTciMessage(const String& msg);
void handleTouch();
void handleSerial();
void encISR();
void drainEncoder();
void keyerTick();
void keyerOutput(bool down);
void keyerSendWpm();
void drawStatic();
void drawDynamic();
void drawButton(const Btn& b, uint16_t bg, uint16_t fg);
void formatFreq(long hz, char* out, size_t n);
const char* freqToBand(long hz);
void cmdVfoDelta(long deltaHz);
void cmdVfoStepCycle(int dir);
void cmdModeStep(int dir);
void cmdVolStep(int dir);
void cmdBandSelect(int idx);
void cmdEncoderTarget(EncoderTarget t);
void cmdWpmStep(int dir);
void cmdIambicToggle();
int  bandIdxForFreq(long hz);
const char* encTargetName(EncoderTarget t);

// Persona stubs — declared here for use from switchToPersona().
void tciResume();
void rc28PersonaBegin();
void aetherControlBegin();

// ===========================================================================
// RC-28 emulator TEST mode  (long-press the 10m band tile to enter)
// ===========================================================================
//
// State machine:   NORMAL  ──[10m long-press]──▶  TEST
//                     ▲                            │
//                     └────[Return tap]────────────┘
//
// In TEST mode the TCI WebSocket is disconnected and the screen swaps to
// a 6-button grid (A..F) + Return.  Each button fires a specific Icom
// RC-28 HID report sequence via the `rc28` object so we can drive
// AetherSDR's IcomRC28Parser with known stimuli (see PR #2870).
//
// On 10m long-press: 200ms dead-zone (so a normal tap still selects 10m),
// then a left-to-right red curtain animates across the screen over ~1300ms.
// If the operator releases before the curtain reaches the right edge, the
// gesture is aborted and the normal UI is repainted.  Reaching the right
// edge commits to TEST mode.

// AetherPadMode is the master state. Three Normal* personas (the device's
// USB role) plus a single Test harness mode that's sibling to all three.
// Numerically the Normal* values are 0..2 (used by the persona carousel as
// indices into PERSONA_LABEL[]); Test is 3 and is entered/exited via its
// own gesture (10m tile long-press / Return tap), never via the carousel.
enum class AetherPadMode : uint8_t {
    NormalTci = 0,             // WiFi/TCI to AetherSDR — historical default
    NormalRc28,                // USB HID — Icom RC-28 emulator
    NormalAetherControl,       // USB CDC — FlexControl emulator (stub)
    Test                       // RC-28 test harness (existing 10m-tile path)
};
static AetherPadMode currentMode      = AetherPadMode::NormalTci;
static AetherPadMode g_prevPersonaMode = AetherPadMode::NormalTci;   // restored on TEST exit

inline bool isTestMode()   { return currentMode == AetherPadMode::Test; }
inline bool isNormalMode() { return !isTestMode(); }

static const char* const PERSONA_LABEL[] = {
    "TCI",                     // NormalTci
    "RC-28",                   // NormalRc28
    "AetherControl",           // NormalAetherControl
    "TEST",                    // Test (not shown by the carousel)
};
// Short labels for the in-screen persona pill (limited width — "AetherControl"
// would overflow a 100-px pill at size 2).
static const char* const PERSONA_PILL[] = {
    "TCI",                     // NormalTci
    "RC-28",                   // NormalRc28
    "AeCtrl",                  // NormalAetherControl
    "TEST",                    // Test (pill is hidden in TEST mode anyway)
};
const uint8_t PERSONA_CAROUSEL_COUNT = 3;   // Normal* only

// Per-persona accent colour. All operator-screen "highlight" surfaces
// (title text, armed chip border, active tile border + label) source their
// accent from here so an operator can tell at a glance which persona is
// currently active. Splash buttons already use these same three accents
// (see drawSplash); this just propagates them into the operator UI.
static inline uint16_t personaAccent() {
    switch (currentMode) {
        case AetherPadMode::NormalRc28:           return C_AMBER;
        case AetherPadMode::NormalAetherControl:  return C_GREEN;
        case AetherPadMode::NormalTci:
        default:                                  return C_CYAN;
    }
}

// Splash screen / persona menu — drawn at boot and re-opened by tapping the
// title bar on any operator screen. Three big buttons let the operator pick
// TCI / RC-28 / AetherControl. After SPLASH_TIMEOUT_MS with no tap, the
// splash auto-continues to autoPick: at boot that's TCI (TODO: persist
// last-used persona in flash and use it as boot default); when re-opened
// via the title-bar menu, autoPick is currentMode so a stray no-tap doesn't
// switch persona on us.
const unsigned long SPLASH_TIMEOUT_MS = 3000;
struct Splash {
    bool          active        = true;
    bool          armed         = false;  // false = boot/wifi phase (no countdown), true = timer running
    unsigned long startMs       = 0;
    AetherPadMode autoPick      = AetherPadMode::NormalTci;
    int           lastSecLeft   = -1;     // for incremental countdown redraw
};
static Splash g_splash;

// Splash button rects (3 across, ~250 px wide each, centered band of screen)
const int SPLASH_BTN_Y = 230;
const int SPLASH_BTN_H = 130;
const int SPLASH_BTN_W = 240;
const int SPLASH_BTN_GAP = 20;
const int SPLASH_BTN_TCI_X    =  20;
const int SPLASH_BTN_RC28_X   = SPLASH_BTN_TCI_X  + SPLASH_BTN_W + SPLASH_BTN_GAP;
const int SPLASH_BTN_ACTRL_X  = SPLASH_BTN_RC28_X + SPLASH_BTN_W + SPLASH_BTN_GAP;

// Forward decls that need AetherPadMode in scope — must come after the enum.
void switchToPersona(AetherPadMode m);
void openSplash(bool autoToCurrent);
void openSplashBoot();
void drawSplash();
void splashTick();
void splashHandleTouch();
void splashStatus(const char* msg, uint16_t fg);

// 10m tile gesture state
static bool          gestureActive    = false;   // 10m held past dead-zone
static unsigned long gestureStartMs   = 0;       // when the touch first started
static int           gestureFillPx    = 0;       // current red-curtain x position
static unsigned long gestureLastFrameMs = 0;
const  unsigned long GESTURE_DEAD_ZONE_MS = 200;
const  unsigned long GESTURE_WIPE_MS      = 1300; // 200 + 1300 = ~1.5 s commit
const  uint16_t      GESTURE_FILL_COLOR   = 0xC000; // dark red

// TEST screen button IDs — must not collide with BtnId enum values (which
// max out at BTN_TILE_BASE + N_TILES - 1 = 14).
const int8_t BTN_TEST_A      = 20;
const int8_t BTN_TEST_B      = 21;
const int8_t BTN_TEST_C      = 22;
const int8_t BTN_TEST_D      = 23;
const int8_t BTN_TEST_E      = 24;
const int8_t BTN_TEST_F      = 25;
const int8_t BTN_TEST_RETURN = 26;

// RC-28 persona button IDs (the three operator buttons on a real Icom RC-28).
const int8_t BTN_RC28_F1     = 30;
const int8_t BTN_RC28_F2     = 31;
const int8_t BTN_RC28_TX     = 32;

// AetherControl persona button IDs (the three Aux buttons in AE's popout —
// host-side configurable, hardware just emits X1S/X2S/X3S).
const int8_t BTN_ACTRL_AUX1  = 40;
const int8_t BTN_ACTRL_AUX2  = 41;
const int8_t BTN_ACTRL_AUX3  = 42;

// TEST screen layout (800x480 landscape)
struct TestRect { int x, y, w, h; };
const TestRect TEST_BANNER       = {   0,   0, 800,  44 };
const TestRect TEST_STATUS       = {   0,  46, 800,  34 };
const int      TEST_BTN_W        = 200;
const int      TEST_BTN_H        = 110;
const int      TEST_BTN_GAP_X    =  30;
const int      TEST_ROW1_Y       =  98;
const int      TEST_ROW2_Y       = 220;
const int      TEST_GRID_LEFT    = (800 - 3 * TEST_BTN_W - 2 * TEST_BTN_GAP_X) / 2;
const TestRect TEST_BTN_A_R      = { TEST_GRID_LEFT + 0 * (TEST_BTN_W + TEST_BTN_GAP_X), TEST_ROW1_Y, TEST_BTN_W, TEST_BTN_H };
const TestRect TEST_BTN_B_R      = { TEST_GRID_LEFT + 1 * (TEST_BTN_W + TEST_BTN_GAP_X), TEST_ROW1_Y, TEST_BTN_W, TEST_BTN_H };
const TestRect TEST_BTN_C_R      = { TEST_GRID_LEFT + 2 * (TEST_BTN_W + TEST_BTN_GAP_X), TEST_ROW1_Y, TEST_BTN_W, TEST_BTN_H };
const TestRect TEST_BTN_D_R      = { TEST_GRID_LEFT + 0 * (TEST_BTN_W + TEST_BTN_GAP_X), TEST_ROW2_Y, TEST_BTN_W, TEST_BTN_H };
const TestRect TEST_BTN_E_R      = { TEST_GRID_LEFT + 1 * (TEST_BTN_W + TEST_BTN_GAP_X), TEST_ROW2_Y, TEST_BTN_W, TEST_BTN_H };
const TestRect TEST_BTN_F_R      = { TEST_GRID_LEFT + 2 * (TEST_BTN_W + TEST_BTN_GAP_X), TEST_ROW2_Y, TEST_BTN_W, TEST_BTN_H };
const TestRect TEST_BTN_RETURN_R = { 800/2 - 110, 370, 220, 90 };

// Last-sent stimulus description for the status line.
static char gLastSent[64] = "(no reports sent yet)";

static bool inTestRect(const TestRect& r, int x, int y) {
    return x >= r.x && x < (r.x + r.w) && y >= r.y && y < (r.y + r.h);
}

static int8_t testBtnAtPoint(int x, int y) {
    if (inTestRect(TEST_BTN_A_R,      x, y)) return BTN_TEST_A;
    if (inTestRect(TEST_BTN_B_R,      x, y)) return BTN_TEST_B;
    if (inTestRect(TEST_BTN_C_R,      x, y)) return BTN_TEST_C;
    if (inTestRect(TEST_BTN_D_R,      x, y)) return BTN_TEST_D;
    if (inTestRect(TEST_BTN_E_R,      x, y)) return BTN_TEST_E;
    if (inTestRect(TEST_BTN_F_R,      x, y)) return BTN_TEST_F;
    if (inTestRect(TEST_BTN_RETURN_R, x, y)) return BTN_TEST_RETURN;
    return BTN_NONE;
}

static void drawTestButton(const TestRect& r, const char* label, uint16_t bg, uint16_t fg) {
    tft.fillRect(r.x, r.y, r.w, r.h, bg);
    tft.drawRect(r.x, r.y, r.w, r.h, C_WHITE);
    tft.setTextColor(fg);
    tft.setTextSize(5);
    // Crude centring — assumes single-char or short labels
    int approxCharW = 6 * 5;  // GFX default font is 6px per char at size 1
    int tw = (int)strlen(label) * approxCharW;
    int th = 8 * 5;
    tft.setCursor(r.x + (r.w - tw) / 2, r.y + (r.h - th) / 2);
    tft.print(label);
}

static void drawTestStatic() {
    tft.fillScreen(C_BG);
    // Banner
    tft.fillRect(TEST_BANNER.x, TEST_BANNER.y, TEST_BANNER.w, TEST_BANNER.h, C_RED);
    tft.setTextColor(C_WHITE);
    tft.setTextSize(3);
    tft.setCursor(40, TEST_BANNER.y + 10);
    tft.print("TEST MODE   -   TCI DISCONNECTED");
    // 6 buttons
    drawTestButton(TEST_BTN_A_R, "A", C_PANEL, C_WHITE);
    drawTestButton(TEST_BTN_B_R, "B", C_PANEL, C_WHITE);
    drawTestButton(TEST_BTN_C_R, "C", C_PANEL, C_WHITE);
    drawTestButton(TEST_BTN_D_R, "D", C_PANEL, C_WHITE);
    drawTestButton(TEST_BTN_E_R, "E", C_PANEL, C_WHITE);
    drawTestButton(TEST_BTN_F_R, "F", C_PANEL, C_WHITE);
    // Return — orange/amber so it's clearly distinct
    drawTestButton(TEST_BTN_RETURN_R, "Return", C_AMBER, 0x0000);
}

static void drawTestDynamic() {
    // Just the status line; rest of the screen is static.
    tft.fillRect(TEST_STATUS.x, TEST_STATUS.y, TEST_STATUS.w, TEST_STATUS.h, C_BORDER);
    tft.setTextColor(C_WHITE);
    tft.setTextSize(2);
    tft.setCursor(20, TEST_STATUS.y + 9);
    tft.print("Last sent: ");
    tft.print(gLastSent);
}

// ===========================================================================
// RC-28 persona operator screen — the aether-pad pretending to be an Icom
// RC-28 (a small puck with one knob + three function buttons: F1, F2, TX).
// USB role-switching + HID emission live in a future phase; this is the
// visual surface only. Taps update gRc28LastTap so the operator gets
// immediate feedback that the touch registered, even though no event
// leaves the device yet.
// ===========================================================================
struct PRect { int x, y, w, h; };

const PRect RC28_FREQ_BAND   = {   0,  44, 800, 130 };   // big freq display
const PRect RC28_BTN_F1_R    = {  60, 200, 200, 170 };
const PRect RC28_BTN_F2_R    = { 300, 200, 200, 170 };
const PRect RC28_BTN_TX_R    = { 540, 200, 200, 170 };
const PRect RC28_STATUS_R    = {   0, 400, 800,  60 };

static char gRc28LastTap[32]  = "ready";

static inline bool inPRect(const PRect& r, int x, int y) {
    return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}

// Filled-rect, persona-amber-outlined operator button.
static void drawPersonaButton(const PRect& r, const char* label,
                              uint16_t accent, uint16_t fg) {
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 12, C_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 12, accent);
    tft.drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 11, accent);
    int sz = 5;
    int tw = (int)strlen(label) * 6 * sz;
    int th = 8 * sz;
    tft.setTextSize(sz); tft.setTextColor(fg);
    tft.setCursor(r.x + (r.w - tw) / 2, r.y + (r.h - th) / 2);
    tft.print(label);
}

static void drawRc28OperatorStatic() {
    tft.fillScreen(C_BG);

    // Title bar (amber-tinted, plus persona pill drawn by Dynamic pass).
    tft.setTextColor(C_AMBER); tft.setTextSize(3);
    tft.setCursor(8, 4); tft.print("AETHER_PAD");
    tft.setTextColor(C_MUTED); tft.setTextSize(2);
    tft.setCursor(220, 10); tft.print("G0JKN/W3");

    // Three operator buttons. TX is red-tinted so the operator can't miss
    // which one will key the radio when wired.
    drawPersonaButton(RC28_BTN_F1_R, "F1", C_AMBER, C_AMBER);
    drawPersonaButton(RC28_BTN_F2_R, "F2", C_AMBER, C_AMBER);
    drawPersonaButton(RC28_BTN_TX_R, "TX", C_RED,   C_RED);
}

static void drawRc28OperatorDynamic() {
    char buf[64];

    // Persona pill (top-right) — same coords as the TCI persona's pill so
    // it lives in the same spot regardless of which screen you're on.
    tft.fillRect(586, 2, 100, 28, C_PANEL);
    tft.drawRect(586, 2, 100, 28, C_AMBER);
    tft.setTextColor(C_AMBER); tft.setTextSize(2);
    const char* pillTxt = "RC-28";
    int ppw = strlen(pillTxt) * 12;
    tft.setCursor(586 + (100 - ppw) / 2, 8);
    tft.print(pillTxt);

    // Big frequency display. Sourced from st.freqHz (the TCI cache); will
    // read 0 if TCI isn't connected, which is fine while USB-only operation
    // hasn't been wired yet. When the HID path lands we'll switch this to
    // mirror what was actually emitted.
    formatFreq(st.freqHz, buf, sizeof(buf));
    tft.fillRect(RC28_FREQ_BAND.x, RC28_FREQ_BAND.y,
                 RC28_FREQ_BAND.w, RC28_FREQ_BAND.h, C_BG);
    tft.setTextSize(7); tft.setTextColor(C_AMBER);
    int fw = strlen(buf) * 42;       // 6*7 = 42 px per char at size 7
    tft.setCursor((800 - fw) / 2, RC28_FREQ_BAND.y + 30);
    tft.print(buf);

    // Status line — last event + USB enumeration hint
    tft.fillRect(RC28_STATUS_R.x, RC28_STATUS_R.y,
                 RC28_STATUS_R.w, RC28_STATUS_R.h, C_PANEL);
    tft.drawRect(RC28_STATUS_R.x, RC28_STATUS_R.y,
                 RC28_STATUS_R.w, RC28_STATUS_R.h, C_BORDER);
    tft.setTextSize(2); tft.setTextColor(C_MUTED);
    tft.setCursor(20, RC28_STATUS_R.y + 12);
    tft.print("Last: ");
    tft.setTextColor(C_AMBER);
    tft.print(gRc28LastTap);
    tft.setTextColor(C_MUTED);
    tft.setCursor(20, RC28_STATUS_R.y + 36);
    tft.print("USB HID role-switching pending (Phase 4)");
}

// ===========================================================================
// AetherControl persona operator screen — mirrors AE's FlexControl popout
// (PR #2888): three LED indicator pills, three big Aux buttons, status
// strip at the bottom. USB CDC + the I-back LED protocol land in a later
// phase; this is the visual surface only.
// ===========================================================================
const PRect ACTRL_LED_MODE_R   = {  40,  60, 220,  50 };
const PRect ACTRL_LED_ONOFF_R  = { 290,  60, 220,  50 };
const PRect ACTRL_LED_TOGGLE_R = { 540,  60, 220,  50 };
const PRect ACTRL_BTN_AUX1_R   = {  40, 140, 220, 170 };
const PRect ACTRL_BTN_AUX2_R   = { 290, 140, 220, 170 };
const PRect ACTRL_BTN_AUX3_R   = { 540, 140, 220, 170 };
const PRect ACTRL_STATUS_R     = {   0, 330, 800, 140 };

static char gACtrlLastTap[32]  = "ready";
// Indicator LED states, driven by AetherSDR's I-back messages (I100;/I010;/
// I001;/I000;). For now they default off and aren't driven by anything —
// the visual is there so the layout is complete.
static bool gACtrlLed[3] = { false, false, false };

static void drawACtrlLedPill(const PRect& r, const char* label, bool lit) {
    // Outlined pill. When lit, the LED dot on the left fills green; otherwise
    // it shows as a dim circle outline.
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 10, C_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 10, C_BORDER);
    int cx = r.x + 22, cy = r.y + r.h / 2;
    if (lit) {
        tft.fillCircle(cx, cy, 10, C_GREEN);
    } else {
        tft.drawCircle(cx, cy, 10, C_MUTED);
    }
    tft.setTextSize(2); tft.setTextColor(lit ? C_GREEN : C_MUTED);
    tft.setCursor(r.x + 50, r.y + (r.h - 16) / 2);
    tft.print(label);
}

static void drawACtrlStatic() {
    tft.fillScreen(C_BG);

    // Title bar (green-tinted, persona pill drawn by Dynamic).
    tft.setTextColor(C_GREEN); tft.setTextSize(3);
    tft.setCursor(8, 4); tft.print("AETHER_PAD");
    tft.setTextColor(C_MUTED); tft.setTextSize(2);
    tft.setCursor(220, 10); tft.print("G0JKN/W3");

    // Three Aux buttons. Labels stay generic — AE configures what each
    // does host-side, so the device has no way to know the assignments.
    drawPersonaButton(ACTRL_BTN_AUX1_R, "Aux1", C_GREEN, C_GREEN);
    drawPersonaButton(ACTRL_BTN_AUX2_R, "Aux2", C_GREEN, C_GREEN);
    drawPersonaButton(ACTRL_BTN_AUX3_R, "Aux3", C_GREEN, C_GREEN);
}

static void drawACtrlDynamic() {
    char buf[64];

    // Persona pill
    tft.fillRect(586, 2, 100, 28, C_PANEL);
    tft.drawRect(586, 2, 100, 28, C_GREEN);
    tft.setTextColor(C_GREEN); tft.setTextSize(2);
    const char* pillTxt = "AeCtrl";
    int ppw = strlen(pillTxt) * 12;
    tft.setCursor(586 + (100 - ppw) / 2, 8);
    tft.print(pillTxt);

    // Three LED indicators (MODE / ON/OFF / TOGGLE). State lives in gACtrlLed[].
    drawACtrlLedPill(ACTRL_LED_MODE_R,   "MODE",   gACtrlLed[0]);
    drawACtrlLedPill(ACTRL_LED_ONOFF_R,  "ON/OFF", gACtrlLed[1]);
    drawACtrlLedPill(ACTRL_LED_TOGGLE_R, "TOGGLE", gACtrlLed[2]);

    // Status strip — WHEEL / SLICE / FREQUENCY / STEP in a 2×2 grid mirroring
    // AE's popout. Frequency + step come from the TCI cache; wheel + slice
    // are static placeholders until the CDC path lands.
    tft.fillRect(ACTRL_STATUS_R.x, ACTRL_STATUS_R.y,
                 ACTRL_STATUS_R.w, ACTRL_STATUS_R.h, C_BG);

    const int col1X = 40, col2X = 420;
    const int rowAY = 340, rowBY = 410;

    tft.setTextSize(2); tft.setTextColor(C_MUTED);
    tft.setCursor(col1X, rowAY);     tft.print("WHEEL");
    tft.setCursor(col2X, rowAY);     tft.print("SLICE");
    tft.setCursor(col1X, rowBY);     tft.print("FREQUENCY");
    tft.setCursor(col2X, rowBY);     tft.print("STEP");

    tft.setTextSize(3); tft.setTextColor(C_WHITE);
    tft.setCursor(col1X, rowAY + 22); tft.print("Tune Slice");
    tft.setCursor(col2X, rowAY + 22); tft.print("A");
    formatFreq(st.freqHz, buf, sizeof(buf));
    tft.setCursor(col1X, rowBY + 22); tft.print(buf);
    if (vfoStepHz >= 1000) snprintf(buf, sizeof(buf), "%ld kHz", vfoStepHz / 1000);
    else                   snprintf(buf, sizeof(buf), "%ld Hz",  vfoStepHz);
    tft.setCursor(col2X, rowBY + 22); tft.print(buf);

    // Last-tap toast — small line under the pill, only when not "ready"
    if (strcmp(gACtrlLastTap, "ready") != 0) {
        tft.fillRect(580, 32, 220, 18, C_BG);
        tft.setTextSize(1); tft.setTextColor(C_GREEN);
        tft.setCursor(580, 36); tft.print("Last: "); tft.print(gACtrlLastTap);
    }
}

static void enterTestMode() {
    Serial.println("TEST MODE: entered (10m long-press committed)");
    g_prevPersonaMode = currentMode;          // remember persona to restore on exit
    currentMode = AetherPadMode::Test;
    // Drop the TCI WebSocket — wsTick will see the disconnect on next tick
    // and stop trying to reconnect while currentMode == Test (see wsTick mod).
    if (tcp.connected()) tcp.stop();
    st.tciConnected = false;
    wsHandshakeDone = false;
    snprintf(gLastSent, sizeof(gLastSent), "(no reports sent yet)");
    drawTestStatic();
    drawTestDynamic();
}

static void exitTestMode() {
    Serial.println("TEST MODE: exited (Return tapped)");
    currentMode = g_prevPersonaMode;          // back to whichever persona was active
    // wsTick will auto-reconnect on its next 3-second retry.
    // Force a FULL repaint — drawStatic() does fillScreen() and rebuilds
    // the chip + band-tile layout; drawDynamic() then fills in the live
    // values. uiNeedsRedraw alone would only run drawDynamic, leaving
    // the TEST banner/buttons showing through (partial-return bug).
    drawStatic();
    drawDynamic();
    uiNeedsRedraw = false;
}

// Send a test stimulus based on which button was pressed.  Each updates
// gLastSent for the status line and schedules a dynamic redraw.
static void fireTestStimulus(int8_t btn) {
    switch (btn) {
        case BTN_TEST_A:
            rc28.sendDetent(AetherPad::Rc28Dir::Cw);
            snprintf(gLastSent, sizeof(gLastSent),
                     "CW 1 detent  (seq=01 dir=01 btn=07; then seq=02)");
            break;
        case BTN_TEST_B:
            rc28.sendDetent(AetherPad::Rc28Dir::Ccw);
            snprintf(gLastSent, sizeof(gLastSent),
                     "CCW 1 detent (seq=01 dir=02 btn=07; then seq=02)");
            break;
        case BTN_TEST_C:
            rc28.sendButton(AetherPad::Rc28Button::F1);
            delay(150);
            rc28.sendButton(AetherPad::Rc28Button::Idle);
            snprintf(gLastSent, sizeof(gLastSent),
                     "F1 press+release  (btn=05 -> 07)");
            break;
        case BTN_TEST_D:
            rc28.sendButton(AetherPad::Rc28Button::F2);
            delay(150);
            rc28.sendButton(AetherPad::Rc28Button::Idle);
            snprintf(gLastSent, sizeof(gLastSent),
                     "F2 press+release  (btn=03 -> 07)");
            break;
        case BTN_TEST_E:
            rc28.sendButton(AetherPad::Rc28Button::Tx);
            delay(150);
            rc28.sendButton(AetherPad::Rc28Button::Idle);
            snprintf(gLastSent, sizeof(gLastSent),
                     "TX bar press+release  (btn=06 -> 07)");
            break;
        case BTN_TEST_F:
            rc28.sendMacOsEdgeCase();
            snprintf(gLastSent, sizeof(gLastSent),
                     "macOS edge case  (seq=01 CW; trigger for #2870 macOS bug)");
            break;
        default: return;
    }
    drawTestDynamic();
}

// Called every loop() iteration. Drives the 10m long-press wipe animation
// when gestureActive is true. Does nothing if no gesture is in progress
// or if we're already in TEST mode.
static void gestureWipeTick() {
    if (currentMode == AetherPadMode::Test) return;
    if (!gestureActive) {
        // If a previous gesture was aborted, gestureFillPx may be > 0;
        // the abort handler in handleTouch sets uiNeedsRedraw to fully
        // repaint the screen, which clears the red curtain naturally.
        return;
    }
    unsigned long now = millis();
    unsigned long held = now - gestureStartMs;
    if (held < GESTURE_DEAD_ZONE_MS) return;        // still in dead-zone
    if (now - gestureLastFrameMs < 25) return;      // ~40fps cap
    gestureLastFrameMs = now;

    long progress = (long)(held - GESTURE_DEAD_ZONE_MS);
    int targetPx = (int)((progress * 800L) / (long)GESTURE_WIPE_MS);
    if (targetPx > 800) targetPx = 800;
    if (targetPx > gestureFillPx) {
        tft.fillRect(gestureFillPx, 0, targetPx - gestureFillPx, 480, GESTURE_FILL_COLOR);
        gestureFillPx = targetPx;
    }
    if (gestureFillPx >= 800) {
        gestureActive    = false;
        gestureFillPx    = 0;
        gestureLastFrameMs = 0;
        enterTestMode();
    }
}

// ===========================================================================
// Setup / loop
// ===========================================================================
void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\naether_pad — booting");
    Serial.print  ("Build: "); Serial.println(BUILD_TAG);
    Serial.print  ("Encoder: "); Serial.println(ENC_NAME);

    Display.begin();
    tft.begin();
    // GigaDisplay_GFX native is portrait 480×800. setRotation(1) rotates 90° CW
    // to give the 800×480 landscape canvas the rest of the layout assumes.
    tft.setRotation(1);
    tft.fillScreen(C_BG);

    Touch.begin();

    pinMode(ENC_PIN_A, INPUT_PULLUP);
    pinMode(ENC_PIN_B, INPUT_PULLUP);
    encLast = (digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), encISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), encISR, CHANGE);

    pinMode(KEY_PIN_DIT, INPUT_PULLUP);
    pinMode(KEY_PIN_DAH, INPUT_PULLUP);
    pinMode(KEY_PIN_OUT, OUTPUT);
    digitalWrite(KEY_PIN_OUT, LOW);     // key up at boot — never leave the
                                        // radio in key-down on power-cycle

    // Splash first — operator sees the persona menu immediately, no flash
    // of the operator UI while WiFi associates. Countdown is disabled here;
    // wifiConnect paints its status message into the splash's status strip.
    openSplashBoot();

    wifiConnect();
    udp.begin(DISCOVERY_PORT);
    discoveryStarted = millis();

    // WiFi up — arm the auto-continue timer.
    // TODO(phase 2): read last-used persona from flash and seed
    // g_splash.autoPick with it instead of always defaulting to TCI.
    openSplash(/*autoToCurrent=*/false);

    // Announce ourselves over mDNS / DNS-SD so AetherSDR's Peripherals
    // browse can list us without manual IP entry.  Port 0 = TCI client only,
    // no inbound server endpoint.  Revisit if/when aether_pad grows a
    // small admin/status TCI endpoint.
    mdns.begin(/*model*/      "aether-pad",
               /*class*/      "controller",
               /*tciVersion*/ "1.9",
               /*instance*/   "aether_pad G0JKN",
               /*hostname*/   "aether-pad-g0jkn",
               /*tciPort*/    0);
}

void loop() {
    discoveryTick();
    mdns.loop();
    wsTick();
    handleSerial();
    drainEncoder();
    keyerTick();

    // Splash / persona menu owns the screen — bypass normal touch + draw
    // paths while it's up. Splash drives its own touch and countdown.
    if (g_splash.active) {
        splashHandleTouch();
        splashTick();
        return;
    }

    // Touch is cheap (GT911 I2C, ~5-10 ms) — always run so taps register
    // even when the keyer is mid-element. The earlier skip-during-keyer
    // version blocked iambic A/B chip taps whenever D4/D5 noise put the
    // state machine into a non-IDLE cycle.
    handleTouch();

    // Animate the 10m long-press wipe (no-op unless gestureActive).
    gestureWipeTick();

    // Drawing is the expensive one (~30-50 ms of GFX). Two paths:
    //   1. Explicit user action (uiNeedsRedraw flag set) → redraw NOW,
    //      regardless of keyer state. Operator just tapped a chip /
    //      band tile / iambic toggle and needs immediate visual
    //      confirmation. Worth the timing hit on this one element.
    //   2. Periodic refresh (every 250 ms) → only when keyer is
    //      fully idle so element boundaries above ~15 WPM aren't
    //      smeared by a redraw landing mid-element.
    static unsigned long lastDraw = 0;
    if (uiNeedsRedraw) {
        drawDynamic();
        lastDraw = millis();
        uiNeedsRedraw = false;
    } else if (kr.state == KS_IDLE && millis() - lastDraw > 250) {
        drawDynamic();
        lastDraw = millis();
    }
}

// ===========================================================================
// WiFi
// ===========================================================================
void wifiConnect() {
    // Status message goes into the splash's bottom strip — doesn't overlap
    // the persona buttons.
    splashStatus("Connecting to WiFi...", C_CYAN);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("WiFi: ");
    while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
    Serial.println();
    Serial.print("WiFi connected, IP = "); Serial.println(WiFi.localIP());
}

// ===========================================================================
// Discovery — UDP probe + reply listener, fallback to hardcoded host
// ===========================================================================
void discoveryTick() {
    if (st.hostKnown) return;
    unsigned long now = millis();

    // Probe every DISCOVERY_INTERVAL ms
    if (now - lastDiscoveryTx > DISCOVERY_INTERVAL) {
        lastDiscoveryTx = now;
        IPAddress bcast = WiFi.localIP();
        bcast[3] = 255;       // subnet broadcast (assumes /24)
        udp.beginPacket(bcast, DISCOVERY_PORT);
        udp.write((const uint8_t*)DISCOVERY_PROBE, strlen(DISCOVERY_PROBE));
        udp.endPacket();
        Serial.print("Discovery probe → "); Serial.println(bcast);
    }

    // Listen for reply
    int n = udp.parsePacket();
    if (n > 0) {
        char buf[128];
        int got = udp.read(buf, min(n, (int)sizeof(buf) - 1));
        buf[max(0, got)] = '\0';
        Serial.print("Discovery reply: "); Serial.println(buf);
        // Expected: "AETHERSDR ip=10.0.0.107 tci=40001"
        // IPAddress::fromString() rejects the trailing " tci=..." so we have
        // to chop the IP into its own NUL-terminated buffer first.
        char* ip = strstr(buf, "ip=");
        char* tp = strstr(buf, "tci=");
        if (ip && tp) {
            char ipBuf[20];
            const char* ipStart = ip + 3;
            const char* ipEnd   = strchr(ipStart, ' ');
            size_t ipLen = ipEnd ? (size_t)(ipEnd - ipStart) : strlen(ipStart);
            if (ipLen >= sizeof(ipBuf)) ipLen = sizeof(ipBuf) - 1;
            memcpy(ipBuf, ipStart, ipLen);
            ipBuf[ipLen] = '\0';

            IPAddress h;
            if (h.fromString(ipBuf)) {
                st.host = h;
                st.port = (uint16_t)atoi(tp + 4);
                st.hostKnown = true;
                uiNeedsRedraw = true;
                Serial.print("AetherSDR @ "); Serial.print(h);
                Serial.print(":"); Serial.println(st.port);
            } else {
                Serial.print("Discovery: bad IP '"); Serial.print(ipBuf); Serial.println("'");
            }
        }
    }

    // Fallback after timeout
    if (!st.hostKnown && !fallbackArmed && now - discoveryStarted > DISCOVERY_FALLBACK) {
        st.host = FALLBACK_HOST;
        st.port = FALLBACK_PORT;
        st.hostKnown = true;
        fallbackArmed = true;
        uiNeedsRedraw = true;
        Serial.print("Discovery timeout — falling back to ");
        Serial.print(st.host); Serial.print(":"); Serial.println(st.port);
    }
}

// ===========================================================================
// WebSocket TCI client (hand-rolled, like ShackController)
// ===========================================================================
void wsTick() {
    if (!st.hostKnown) return;
    // While in RC-28 TEST mode, the TCI socket is intentionally torn down
    // (see enterTestMode). Don't reconnect until the operator exits TEST.
    if (currentMode == AetherPadMode::Test) return;

    if (!tcp.connected()) {
        if (st.tciConnected) {
            st.tciConnected = false;
            wsHandshakeDone = false;
            wsRxBuf = "";
            uiNeedsRedraw = true;
            Serial.println("TCI: disconnected");
        }
        static unsigned long lastTry = 0;
        if (millis() - lastTry < 3000) return;
        lastTry = millis();
        wsConnect();
        return;
    }

    if (!wsHandshakeDone) return;   // still reading 101 response
    wsParseIncoming();
}

bool wsConnect() {
    Serial.print("TCI: connecting to "); Serial.print(st.host);
    Serial.print(":"); Serial.println(st.port);
    if (!tcp.connect(st.host, st.port)) {
        Serial.println("TCI: TCP connect failed");
        return false;
    }

    // Generate 16 random bytes → base64 key
    uint8_t key[16];
    for (int i = 0; i < 16; i++) key[i] = (uint8_t)random(0, 256);
    static const char* B64 =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char b64[25] = {0};
    for (int i = 0, j = 0; i < 15; i += 3, j += 4) {
        uint32_t v = (key[i] << 16) | (key[i+1] << 8) | key[i+2];
        b64[j  ] = B64[(v >> 18) & 0x3F];
        b64[j+1] = B64[(v >> 12) & 0x3F];
        b64[j+2] = B64[(v >>  6) & 0x3F];
        b64[j+3] = B64[ v        & 0x3F];
    }
    // last byte: pad with one '='
    uint32_t v = (key[15] << 16);
    b64[20] = B64[(v >> 18) & 0x3F];
    b64[21] = B64[(v >> 12) & 0x3F];
    b64[22] = '=';
    b64[23] = '=';

    tcp.print("GET / HTTP/1.1\r\n");
    tcp.print("Host: "); tcp.print(st.host); tcp.print(":"); tcp.print(st.port); tcp.print("\r\n");
    tcp.print("Upgrade: websocket\r\n");
    tcp.print("Connection: Upgrade\r\n");
    tcp.print("Sec-WebSocket-Key: "); tcp.print(b64); tcp.print("\r\n");
    tcp.print("Sec-WebSocket-Version: 13\r\n\r\n");

    // Read response until "\r\n\r\n"
    String resp;
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
        while (tcp.available()) {
            char c = tcp.read();
            resp += c;
            if (resp.endsWith("\r\n\r\n")) {
                if (resp.indexOf("101") > 0) {
                    wsHandshakeDone = true;
                    st.tciConnected = true;
                    wsRxBuf = "";
                    uiNeedsRedraw = true;
                    Serial.println("TCI: WebSocket handshake OK");
                    keyerSendWpm();   // advertise current keyer speed
                    return true;
                } else {
                    Serial.println("TCI: handshake rejected:");
                    Serial.println(resp);
                    tcp.stop();
                    return false;
                }
            }
        }
    }
    Serial.println("TCI: handshake timeout");
    tcp.stop();
    return false;
}

void wsSendText(const char* s) {
    if (!st.tciConnected) return;
    size_t n = strlen(s);
    uint8_t hdr[4];
    int hl;
    if (n < 126) {
        hdr[0] = 0x81; hdr[1] = (uint8_t)n; hl = 2;
    } else {
        hdr[0] = 0x81; hdr[1] = 126; hdr[2] = (n >> 8) & 0xFF; hdr[3] = n & 0xFF; hl = 4;
    }
    tcp.write(hdr, hl);
    tcp.write((const uint8_t*)s, n);
    Serial.print("TCI→ "); Serial.println(s);
}

void wsSendText(const String& s) { wsSendText(s.c_str()); }

void wsParseIncoming() {
    while (tcp.available()) {
        char c = tcp.read();
        if (wsRxBuf.length() < 2048) wsRxBuf += c;
    }
    // Try to extract complete frames
    while (wsRxBuf.length() >= 2) {
        uint8_t b0 = (uint8_t)wsRxBuf[0];
        uint8_t b1 = (uint8_t)wsRxBuf[1];
        bool   masked = (b1 & 0x80) != 0;
        uint16_t len  = b1 & 0x7F;
        int hdr = 2;
        if (len == 126) {
            if (wsRxBuf.length() < 4) return;
            len = ((uint8_t)wsRxBuf[2] << 8) | (uint8_t)wsRxBuf[3];
            hdr = 4;
        } else if (len == 127) {
            wsRxBuf = "";    // 64-bit length not used by TCI; flush on corruption
            return;
        }
        if (masked) hdr += 4;
        size_t total = hdr + len;
        if (wsRxBuf.length() < total) return;

        uint8_t opcode = b0 & 0x0F;
        if (opcode == 0x8) {                // close frame
            tcp.stop();
            wsRxBuf = "";
            return;
        }
        if (opcode == 0x1) {                // text frame
            String payload;
            payload.reserve(len);
            if (masked) {
                for (uint16_t i = 0; i < len; i++) {
                    payload += (char)((uint8_t)wsRxBuf[hdr + i] ^ (uint8_t)wsRxBuf[hdr - 4 + (i & 3)]);
                }
            } else {
                payload = wsRxBuf.substring(hdr, hdr + len);
            }
            parseTciMessage(payload);
        }
        wsRxBuf = wsRxBuf.substring(total);
    }
}

void parseTciMessage(const String& msg) {
    // Skip the high-rate noise (200ms broadcasts) so the log stays useful.
    if (!msg.startsWith("rx_smeter") && !msg.startsWith("tx_") &&
        !msg.startsWith("vox") && !msg.startsWith("mic_")) {
        Serial.print("TCI← "); Serial.println(msg);
    }
    int start = 0;
    while (start < (int)msg.length()) {
        int end = msg.indexOf(';', start);
        if (end < 0) break;
        String cmd = msg.substring(start, end);
        cmd.trim();
        start = end + 1;
        int colon = cmd.indexOf(':');
        if (colon <= 0) continue;
        String name = cmd.substring(0, colon);
        String args = cmd.substring(colon + 1);

        if (name == "vfo") {
            // trx, vfo, freq
            int c1 = args.indexOf(','); if (c1 < 0) continue;
            int c2 = args.indexOf(',', c1 + 1); if (c2 < 0) continue;
            int trx  = args.substring(0, c1).toInt();
            int vfon = args.substring(c1 + 1, c2).toInt();
            long freq = atol(args.substring(c2 + 1).c_str());
            if (trx == 0 && vfon == 0) {
                st.freqHz = freq;
                strncpy(st.bandName, freqToBand(freq), sizeof(st.bandName) - 1);
                st.bandName[sizeof(st.bandName) - 1] = '\0';
                uiNeedsRedraw = true;
            }
        } else if (name == "modulation") {
            int c1 = args.indexOf(','); if (c1 < 0) continue;
            int trx = args.substring(0, c1).toInt();
            String m = args.substring(c1 + 1); m.toLowerCase();
            if (trx == 0) {
                for (int i = 0; i < N_MODES; i++) {
                    if (m == MODES[i]) { st.modeIdx = i; uiNeedsRedraw = true; break; }
                }
            }
        } else if (name == "volume") {
            // Legacy `volume:N;` notification — single arg, master/first slice
            st.volume = args.toInt();
            uiNeedsRedraw = true;
        } else if (name == "rx_volume") {
            // `rx_volume:trx,value;` — per-receiver volume. Track trx 0 only.
            int c1 = args.indexOf(','); if (c1 < 0) continue;
            int trx = args.substring(0, c1).toInt();
            int vol = args.substring(c1 + 1).toInt();
            if (trx == 0) {
                st.volume = vol;
                uiNeedsRedraw = true;
            }
        }
    }
}

// ===========================================================================
// Touch — landscape 800×480. The GT911 on this shield reports in 480×800
// regardless of GFX rotation, so we have to un-rotate manually.
//
// Mode 0 (default) verified 2026-05-02 by 4-corner tap test:
//   tx = raw_y, ty = 479 - raw_x?
//   0 = default (verified)            tx = raw_y;       ty = 479 - raw_x
//   1 = 180° flipped from default     tx = 799 - raw_y; ty = raw_x
//   2 = raw — no transform            tx = raw_x;       ty = raw_y
//   3 = flip-both in 800×480 frame    tx = 799 - raw_x; ty = 479 - raw_y
// ===========================================================================
int touchMode = 0;

static bool inBtn(const Btn& b, int x, int y) {
    return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h;
}

static int8_t btnAtPoint(int x, int y) {
    // TEST mode has a completely different button layout; redirect.
    if (currentMode == AetherPadMode::Test) return testBtnAtPoint(x, y);

    if (inBtn(btnFreqChip,  x, y)) return BTN_FREQ_CHIP;
    // RC-28 persona: only the three function buttons are tappable (plus
    // the title bar, handled at the bottom of this function).
    if (currentMode == AetherPadMode::NormalRc28) {
        if (inPRect(RC28_BTN_F1_R, x, y)) return BTN_RC28_F1;
        if (inPRect(RC28_BTN_F2_R, x, y)) return BTN_RC28_F2;
        if (inPRect(RC28_BTN_TX_R, x, y)) return BTN_RC28_TX;
        if (y >= 0 && y < 30) return BTN_TITLE;
        return BTN_NONE;
    }
    // AetherControl persona: three Aux buttons + title bar.
    if (currentMode == AetherPadMode::NormalAetherControl) {
        if (inPRect(ACTRL_BTN_AUX1_R, x, y)) return BTN_ACTRL_AUX1;
        if (inPRect(ACTRL_BTN_AUX2_R, x, y)) return BTN_ACTRL_AUX2;
        if (inPRect(ACTRL_BTN_AUX3_R, x, y)) return BTN_ACTRL_AUX3;
        if (y >= 0 && y < 30) return BTN_TITLE;
        return BTN_NONE;
    }

    if (inBtn(btnVolChip,   x, y)) return BTN_VOL_CHIP;
    if (inBtn(btnModeChip,  x, y)) return BTN_MODE_CHIP;
    if (inBtn(btnStepChip,  x, y)) return BTN_STEP_CHIP;
    if (inBtn(btnKeyerChip, x, y)) return BTN_KEYER_CHIP;
    for (int i = 0; i < N_TILES; i++) {
        if (inBtn(btnTile[i], x, y)) return (int8_t)(BTN_TILE_BASE + i);
    }
    // Title bar strip (y = 0..30) — last because nothing else lives there.
    // Used only by the persona carousel long-press; a plain tap is a no-op.
    if (y >= 0 && y < 30) return BTN_TITLE;
    return BTN_NONE;
}

// Dispatch a tile press by looking up its action in the current row.
// Reserved tiles (ACT_NONE) are silently ignored — operator can't tell
// they exist apart from the empty label.
static void firePressTile(int idx) {
    const TileData* row = currentTileRow();
    const TileData& t   = row[idx];
    switch (t.action) {
        case ACT_BAND_SELECT: cmdBandSelect(t.arg);                  break;
        case ACT_MODE_SET:    cmdModeStep(t.arg - st.modeIdx);       break;
        case ACT_WPM_SET:     cmdWpmStep(t.arg - cwWpm);             break;
        case ACT_IAMBIC_A:
            if (iambicMode != 'A') cmdIambicToggle();
            break;
        case ACT_IAMBIC_B:
            if (iambicMode != 'B') cmdIambicToggle();
            break;
        case ACT_NONE:
        default:
            break;
    }
}

static void firePress(int8_t btn) {
    // TEST mode buttons get routed first so the normal handlers don't
    // accidentally fire (BTN_TEST_* values are above the BTN_TILE_BASE range).
    if (btn >= BTN_TEST_A && btn <= BTN_TEST_F) {
        fireTestStimulus(btn);
        return;
    }
    if (btn == BTN_TEST_RETURN) {
        exitTestMode();
        return;
    }
    // RC-28 persona button taps — display-only for now (no HID emit until
    // Phase 4). Update gRc28LastTap so the status line gives the operator
    // immediate "yes that registered" feedback.
    if (btn == BTN_RC28_F1) { snprintf(gRc28LastTap, sizeof(gRc28LastTap), "F1");
                              uiNeedsRedraw = true; return; }
    if (btn == BTN_RC28_F2) { snprintf(gRc28LastTap, sizeof(gRc28LastTap), "F2");
                              uiNeedsRedraw = true; return; }
    if (btn == BTN_RC28_TX) { snprintf(gRc28LastTap, sizeof(gRc28LastTap), "TX");
                              uiNeedsRedraw = true; return; }
    // AetherControl persona Aux button taps — display-only for now (no CDC
    // emit until Phase 4).
    if (btn == BTN_ACTRL_AUX1) { snprintf(gACtrlLastTap, sizeof(gACtrlLastTap), "Aux1 (X1S)");
                                 uiNeedsRedraw = true; return; }
    if (btn == BTN_ACTRL_AUX2) { snprintf(gACtrlLastTap, sizeof(gACtrlLastTap), "Aux2 (X2S)");
                                 uiNeedsRedraw = true; return; }
    if (btn == BTN_ACTRL_AUX3) { snprintf(gACtrlLastTap, sizeof(gACtrlLastTap), "Aux3 (X3S)");
                                 uiNeedsRedraw = true; return; }
    if (btn >= BTN_TILE_BASE && btn < BTN_TILE_BASE + N_TILES) {
        firePressTile(btn - BTN_TILE_BASE);
        return;
    }
    switch (btn) {
        case BTN_FREQ_CHIP:  cmdEncoderTarget(TGT_VFO);    break;
        case BTN_VOL_CHIP:   cmdEncoderTarget(TGT_VOL);    break;
        case BTN_MODE_CHIP:  cmdEncoderTarget(TGT_MODE);   break;
        case BTN_STEP_CHIP:  cmdEncoderTarget(TGT_STEP);   break;
        case BTN_KEYER_CHIP: cmdEncoderTarget(TGT_KEYER);  break;
        case BTN_TITLE:      openSplash(/*autoToCurrent=*/true); break;
    }
}

static void fireLongPress(int8_t /*btn*/) {
    // Long-press is now reserved — the 10m tile gesture is handled
    // separately in handleTouch (red curtain → TEST mode). Title-bar
    // touches open the persona menu via firePress, not long-press.
}

void handleTouch() {
    // Long-press support requires tracking touch state across loop iterations:
    //  - heldBtn         : which button (if any) the finger is currently on
    //  - heldSince       : when that touch began (long-press measurement)
    //  - longPressFired  : prevents the long-press action firing repeatedly
    //                      while the finger is still down past the threshold
    //  - lastTouchSeen   : last loop tick where the panel reported n>0; used
    //                      to filter momentary GT911 dropouts that would
    //                      otherwise appear as release+press and re-fire the
    //                      same button (root cause of the "mode jumps 2-3
    //                      modes per tap" bug — discovered 2026-05-07).
    //  - lastTouchDown   : 220 ms debounce so a single tap can't double-fire
    static int8_t        heldBtn        = BTN_NONE;
    static unsigned long heldSince      = 0;
    static bool          longPressFired = false;
    static unsigned long lastTouchDown  = 0;
    static unsigned long lastTouchSeen  = 0;
    const  unsigned long RELEASE_MS     = 80;   // n=0 must persist this long to count as a release

    GDTpoint_t tp[5];
    uint8_t n = Touch.getTouchPoints(tp);

    if (n == 0) {
        // Only treat this as a real release if no touch has been seen for at
        // least RELEASE_MS. Brief n=0 glitches during a hold are common on
        // the GT911 and would otherwise reset heldBtn → next loop iteration
        // would fire the same button again past the 220ms debounce window.
        if (heldBtn != BTN_NONE && (millis() - lastTouchSeen) >= RELEASE_MS) {
            // 10m-tile gesture aborted before committing — repaint normal UI
            // to clear the red curtain that gestureWipeTick painted.
            if (gestureActive) {
                gestureActive   = false;
                gestureFillPx   = 0;
                gestureLastFrameMs = 0;
                uiNeedsRedraw   = true;
            }
            heldBtn        = BTN_NONE;
            longPressFired = false;
        }
        return;
    }

    lastTouchSeen = millis();

    int rx = tp[0].x, ry = tp[0].y;
    int tx, ty;
    switch (touchMode) {
        case 1:  tx = 799 - ry; ty = rx;       break;  // 180° from default
        case 2:  tx = rx;       ty = ry;       break;  // raw
        case 3:  tx = 799 - rx; ty = 479 - ry; break;  // flip both 800×480
        default: tx = ry;       ty = 479 - rx; break;  // mode 0: verified 2026-05-02
    }

    int8_t btn = btnAtPoint(tx, ty);

    if (btn != heldBtn) {
        // Either a brand-new touch on a button, or the finger slid off / onto a
        // different one. We treat the move as a new press (with debounce) so a
        // slid touch doesn't accidentally re-fire the original button's action.
        if (btn != BTN_NONE && (millis() - lastTouchDown >= 220)) {
            lastTouchDown  = millis();
            heldBtn        = btn;
            heldSince      = millis();
            longPressFired = false;

            Serial.print("Touch raw "); Serial.print(rx); Serial.print(","); Serial.print(ry);
            Serial.print(" → "); Serial.print(tx); Serial.print(","); Serial.print(ty);
            Serial.print(" btn "); Serial.println((int)btn);

            firePress(btn);
        } else {
            heldBtn        = btn;          // could be BTN_NONE
            longPressFired = false;
        }
    } else if (heldBtn != BTN_NONE && !longPressFired
               && (millis() - heldSince) >= LONG_PRESS_MS) {
        // Same button held long enough to qualify — fire the long-press action
        // exactly once. To cycle further the user must release and press again.
        fireLongPress(heldBtn);
        longPressFired = true;
    }

    // RC-28 test-mode gesture: detect 10m tile held past the dead-zone.
    // gestureWipeTick() (called from loop()) does the per-frame animation
    // and commits to TEST mode when the wipe completes.  We just flip the
    // flag here so the wipe tick knows it's allowed to run.
    const int8_t kBtnTenM = (int8_t)(BTN_TILE_BASE + 9);   // 10m tile
    if (isNormalMode()
        && heldBtn == kBtnTenM
        && !gestureActive
        && (millis() - heldSince) >= GESTURE_DEAD_ZONE_MS) {
        gestureActive    = true;
        gestureStartMs   = heldSince;
        gestureFillPx    = 0;
        gestureLastFrameMs = 0;
        Serial.println("TEST MODE: 10m long-press past dead-zone, starting wipe");
    }
}

// ===========================================================================
// Encoder — 4-state lookup table, ISR per A/B edge
// ===========================================================================
static const int8_t QUAD[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

void encISR() {
    encIsrCount++;
    uint8_t now = (digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B);
    encRaw += QUAD[(encLast << 2) | now];
    encLast = now;
}

void drainEncoder() {
    static long          lastEmittedRaw = 0;
    static unsigned long lastNonVfoMs   = 0;
    long raw;
    noInterrupts(); raw = encRaw; interrupts();
    long delta = raw - lastEmittedRaw;
    if (delta == 0) return;

    // Modal dispatch — what the encoder controls depends on the last chip
    // the operator tapped. Default after boot is TGT_VFO so behaviour
    // pre-redesign is preserved until a chip is touched. For non-VFO
    // targets we use a coarser effective divider AND rate-limit to one
    // emission per ENC_NONVFO_MIN_MS so a smooth optical encoder doesn't
    // fly through cycles. STEP only has 5 entries — a quick flick used
    // to land you 4 places along.
    if (encTarget == TGT_VFO) {
        long steps = delta / encDivider;
        if (steps == 0) return;
        lastEmittedRaw += steps * encDivider;
        cmdVfoDelta(steps * vfoStepHz);
        return;
    }

    long divNonVfo = (long)encDivider * ENC_NONVFO_DIV_MULT;
    long steps = delta / divNonVfo;
    if (steps == 0) return;
    lastEmittedRaw += steps * divNonVfo;

    unsigned long now = millis();
    if (now - lastNonVfoMs < ENC_NONVFO_MIN_MS) return;
    lastNonVfoMs = now;

    int dir = (steps > 0) ? +1 : -1;
    switch (encTarget) {
        case TGT_VOL:    cmdVolStep(dir);        break;
        case TGT_MODE:   cmdModeStep(dir);       break;
        case TGT_STEP:   cmdVfoStepCycle(dir);   break;
        case TGT_KEYER:  cmdWpmStep(dir);        break;
        case TGT_VFO:    /* handled above */     break;
    }
}

// ===========================================================================
// CW Iambic keyer — paddles on D4 (dit) / D5 (dah), output on D6 (KEY_PIN_OUT).
//
// State machine ticks every loop iteration; element timing derives from cwWpm.
// Output path is GPIO-direct: writing D6 HIGH/LOW reaches the radio in well
// under a microsecond.  TCI WebSocket-based keying was tried first
// (`keyer:0,true|false;` to AetherSDR) but WiFi latency + jitter made dit/dah
// timing audibly unstable even at 20 WPM.  The bullet-proof path is to drive
// a key-line transistor / optocoupler directly from the Giga.
//
// If you want AetherSDR to *also* see the keyer events for UI / sidetone
// purposes, set KEYER_TCI_ECHO to 1 below and the function will additionally
// publish the on/off events over TCI.  The radio is keyed by the GPIO line
// regardless — the TCI echo is informational, never the source of truth for
// timing.
// ===========================================================================
#ifndef KEYER_TCI_ECHO
#define KEYER_TCI_ECHO 0
#endif

void keyerOutput(bool down) {
    if (kr.keyDown == down) return;
    kr.keyDown = down;
    digitalWrite(KEY_PIN_OUT, down ? HIGH : LOW);
#if KEYER_TCI_ECHO
    wsSendText(down ? "keyer:0,true;" : "keyer:0,false;");
#endif
    if (down) kr.txCount++;
    uiNeedsRedraw = true;
}

void keyerSendWpm() {
    char buf[32];
    snprintf(buf, sizeof(buf), "cw_keyer_speed:%d;", cwWpm);
    wsSendText(buf);
}

void keyerTick() {
    // Element duration (PARIS standard): dit_ms = 1200 / WPM
    const uint32_t ditMs = 1200UL / (uint32_t)cwWpm;
    const uint32_t dahMs = ditMs * 3;

    // Paddle debounce — the Giga's INPUT_PULLUP is too weak for these
    // floating inputs (per project feedback memory; same issue as the
    // encoder lines). Bench noise produces sub-ms LOW spikes that the
    // state machine would interpret as full paddle presses, generating
    // random dits and dahs at idle. Require LOW to persist for
    // PADDLE_DEBOUNCE_MS before treating as pressed. Real squeezes hold
    // 50+ ms so 5 ms is well below any meaningful element while rejecting
    // bench noise. Permanent fix: external 10 kΩ pull-ups from D4 and D5
    // to 3.3 V — same convention as the encoder lines.
    // 15 ms is well below the 50+ ms hold of any deliberate paddle press
    // but rejects all but the most stubborn floating-pin noise. Combine
    // with external 10 kΩ pull-ups to 3.3 V on D4/D5 for permanent fix.
    const unsigned long PADDLE_DEBOUNCE_MS = 15;
    static unsigned long ditChange = 0,    dahChange = 0;
    static int           ditRaw    = HIGH, dahRaw    = HIGH;
    static bool          ditStable = false, dahStable = false;

    int rDit = digitalRead(KEY_PIN_DIT);
    if (rDit != ditRaw) { ditChange = millis(); ditRaw = rDit; }
    if (millis() - ditChange >= PADDLE_DEBOUNCE_MS) ditStable = (rDit == LOW);

    int rDah = digitalRead(KEY_PIN_DAH);
    if (rDah != dahRaw) { dahChange = millis(); dahRaw = rDah; }
    if (millis() - dahChange >= PADDLE_DEBOUNCE_MS) dahStable = (rDah == LOW);

    const bool dit = ditStable;
    const bool dah = dahStable;

    const uint32_t now = millis();
    const uint32_t inState = now - kr.stateAt;

    auto enter = [&](KeyerState s) { kr.state = s; kr.stateAt = now; };

    switch (kr.state) {
        case KS_IDLE:
            if (dit && !dah)      { keyerOutput(true);  enter(KS_DIT_ON); }
            else if (dah && !dit) { keyerOutput(true);  enter(KS_DAH_ON); }
            else if (dit && dah)  { keyerOutput(true);  enter(KS_DIT_ON); }  // dit-first when squeeze starts
            break;

        case KS_DIT_ON:
            // Mode-B: while sending dit, queue dah if paddle held
            if (dah) kr.memDah = true;
            if (inState >= ditMs) { keyerOutput(false); enter(KS_DIT_GAP); }
            break;

        case KS_DIT_GAP:
            if (dah) kr.memDah = true;
            if (inState >= ditMs) {
                // After dit gap, decide next element
                if (dah || (iambicMode == 'B' && kr.memDah)) {
                    kr.memDah = false; keyerOutput(true); enter(KS_DAH_ON);
                } else if (dit) {
                    keyerOutput(true); enter(KS_DIT_ON);
                } else {
                    kr.memDah = false; kr.memDit = false;
                    enter(KS_IDLE);
                }
            }
            break;

        case KS_DAH_ON:
            if (dit) kr.memDit = true;
            if (inState >= dahMs) { keyerOutput(false); enter(KS_DAH_GAP); }
            break;

        case KS_DAH_GAP:
            if (dit) kr.memDit = true;
            if (inState >= ditMs) {
                if (dit || (iambicMode == 'B' && kr.memDit)) {
                    kr.memDit = false; keyerOutput(true); enter(KS_DIT_ON);
                } else if (dah) {
                    keyerOutput(true); enter(KS_DAH_ON);
                } else {
                    kr.memDit = false; kr.memDah = false;
                    enter(KS_IDLE);
                }
            }
            break;
    }
}

// ===========================================================================
// Command issuers — update local state then push TCI command
// ===========================================================================
void cmdVfoStepCycle(int dir) {
    // Step the active VFO step size up or down through STEP_CYCLE[].
    // dir > 0 = bigger step, dir < 0 = smaller. Wraps at the ends so the
    // encoder feels continuous when armed for STEP.
    int idx = 0;
    for (int i = 0; i < N_STEPS; i++) {
        if (STEP_CYCLE[i] == vfoStepHz) { idx = i; break; }
    }
    idx += (dir > 0) ? +1 : -1;
    if (idx < 0)         idx = N_STEPS - 1;
    if (idx >= N_STEPS)  idx = 0;
    vfoStepHz = STEP_CYCLE[idx];
    Serial.print("VFO step = "); Serial.print(vfoStepHz); Serial.println(" Hz");
    uiNeedsRedraw = true;
}

// Map a frequency back to its band-stack index (so VFO tuning updates the
// right entry's lastHz). Returns -1 if the freq is between bands.
int bandIdxForFreq(long hz) {
    for (int i = 0; i < N_BANDS; i++) {
        if (hz >= bandStack[i].loHz && hz <= bandStack[i].hiHz) return i;
    }
    return -1;
}

void cmdVfoDelta(long deltaHz) {
    long target = st.freqHz + deltaHz;
    if (target < 100000)     target = 100000;       // 100 kHz floor
    if (target > 60000000UL) target = 60000000UL;   // 60 MHz ceiling
    st.freqHz = target;
    strncpy(st.bandName, freqToBand(target), sizeof(st.bandName) - 1);
    st.bandName[sizeof(st.bandName) - 1] = '\0';
    int bi = bandIdxForFreq(target);
    if (bi >= 0) bandStack[bi].lastHz = target;     // remember last freq per band
    char buf[40];
    snprintf(buf, sizeof(buf), "vfo:0,0,%ld;", target);
    wsSendText(buf);
    uiNeedsRedraw = true;
}

void cmdModeStep(int dir) {
    // Wrap-around so the encoder feels continuous when armed for MODE.
    int next = st.modeIdx + dir;
    if (next < 0)         next = N_MODES - 1;
    if (next >= N_MODES)  next = 0;
    if (next == st.modeIdx) return;
    st.modeIdx = next;
    char buf[40];
    snprintf(buf, sizeof(buf), "modulation:0,%s;", MODES[next]);
    wsSendText(buf);
    uiNeedsRedraw = true;
}

void cmdVolStep(int dir) {
    int next = st.volume + dir * VOL_STEP;
    if (next < VOL_MIN) next = VOL_MIN;
    if (next > VOL_MAX) next = VOL_MAX;
    if (next == st.volume) return;
    st.volume = next;
    // `volume:N;` — TCI v2.0 global master volume command. After the
    // AetherSDR-side fix for issue #1764, this routes through MainWindow's
    // applyMasterVolume() to the same path the title bar slider uses
    // (AudioEngine::setRxVolume or RadioModel lineout, depending on
    // PcAudioEnabled). On older AetherSDR builds the pad volume will
    // appear to flicker but not change actual audio — flash a fixed
    // AetherSDR build to make the volume actually move.
    char buf[24];
    snprintf(buf, sizeof(buf), "volume:%d;", next);
    wsSendText(buf);
    uiNeedsRedraw = true;
}

void cmdBandSelect(int idx) {
    // Direct band selection — jump straight to the band's last-known freq.
    // Tapping the same band tile that's already active is a no-op for now;
    // future enhancement: cycle through stack memories within that band.
    if (idx < 0 || idx >= N_BANDS) return;
    long target = bandStack[idx].lastHz;
    Serial.print("Band → "); Serial.print(bandStack[idx].name);
    Serial.print(" @ "); Serial.println(target);
    cmdVfoDelta(target - st.freqHz);     // re-uses VFO clamp + TCI send + redraw
}

const char* encTargetName(EncoderTarget t) {
    switch (t) {
        case TGT_VFO:   return "VFO";
        case TGT_VOL:   return "VOL";
        case TGT_MODE:  return "MODE";
        case TGT_STEP:  return "STEP";
        case TGT_KEYER: return "KEYER";
    }
    return "?";
}

void cmdEncoderTarget(EncoderTarget t) {
    // Toggle-tap: tapping an already-armed chip releases focus back to
    // the home target (VFO + BANDS tray). Lets the operator collapse
    // the KEYER/MODE tray without hunting for the freq chip. The freq
    // chip itself doesn't toggle — it's already the home target.
    if (t == encTarget) {
        if (t == TGT_VFO) return;
        t = TGT_VFO;
    }
    encTarget = t;

    // Bottom-row "tool tray" follows the armed chip — operator gets
    // both fine control (encoder spin) and coarse jump (tile tap) in
    // one tap. VFO/VOL/STEP fall back to BANDS since those targets
    // don't have a dedicated tray (yet).
    switch (t) {
        case TGT_MODE:  bottomRow = ROW_MODE;  break;
        case TGT_KEYER: bottomRow = ROW_KEYER; break;
        case TGT_VFO:
        case TGT_VOL:
        case TGT_STEP:
        default:        bottomRow = ROW_BANDS; break;
    }

    Serial.print("Encoder armed for "); Serial.print(encTargetName(t));
    Serial.print(" (tray="); Serial.print((int)bottomRow); Serial.println(")");
    uiNeedsRedraw = true;
}

void cmdWpmStep(int dir) {
    int next = cwWpm + dir;
    if (next < 5)   next = 5;
    if (next > 100) next = 100;
    if (next == cwWpm) return;
    cwWpm = next;
    keyerSendWpm();
    Serial.print("CW WPM = "); Serial.println(cwWpm);
    uiNeedsRedraw = true;
}

void cmdIambicToggle() {
    iambicMode = (iambicMode == 'A') ? 'B' : 'A';
    Serial.print("Iambic "); Serial.println(iambicMode);
    uiNeedsRedraw = true;
}

// ===========================================================================
// Display rendering
// ===========================================================================
void drawStatic() {
    // TEST mode owns the whole screen — divert.
    if (currentMode == AetherPadMode::Test) { drawTestStatic(); drawTestDynamic(); return; }
    // RC-28 and AetherControl personas each have their own dedicated layout
    // (parallel to TEST — paint static + dynamic, then return so the TCI
    // operator UI further down doesn't render on top).
    if (currentMode == AetherPadMode::NormalRc28) {
        drawRc28OperatorStatic(); drawRc28OperatorDynamic(); return;
    }
    if (currentMode == AetherPadMode::NormalAetherControl) {
        drawACtrlStatic(); drawACtrlDynamic(); return;
    }

    tft.fillScreen(C_BG);

    // Title bar — text colour follows the current persona's accent so a
    // glance at the screen tells the operator which page they're on (cyan
    // = TCI, amber = RC-28, green = AetherControl). The persona pill drawn
    // by drawPersonaPill() (called from drawDynamic) carries the textual
    // "you are here" badge.
    const uint16_t accent = personaAccent();
    tft.setTextColor(accent); tft.setTextSize(3);
    tft.setCursor(8, 4); tft.print("AETHER_PAD");
    tft.setTextColor(C_MUTED); tft.setTextSize(2);
    tft.setCursor(220, 10); tft.print("G0JKN/W3");

    // Tile borders — drawn once. Tile contents (label + active highlight)
    // are repainted dynamically because the row mode is operator-driven.
    for (int i = 0; i < N_TILES; i++) {
        const Btn& b = btnTile[i];
        tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, C_BORDER);
    }

    // The screen-wipe above invalidates the tile cache — force the next
    // drawDynamic to repaint every tile from scratch.
    invalidateTileCache();
}

// ===========================================================================
// Splash / persona menu — boot screen and re-openable via title-bar tap.
// ===========================================================================
static void drawSplashButton(int x, const char* label, uint16_t accent, bool highlight) {
    // Background panel
    tft.fillRoundRect(x, SPLASH_BTN_Y, SPLASH_BTN_W, SPLASH_BTN_H, 10,
                      highlight ? C_PANEL : C_BG);
    tft.drawRoundRect(x, SPLASH_BTN_Y, SPLASH_BTN_W, SPLASH_BTN_H, 10, accent);
    if (highlight) {
        tft.drawRoundRect(x + 2, SPLASH_BTN_Y + 2,
                          SPLASH_BTN_W - 4, SPLASH_BTN_H - 4, 8, accent);
    }
    // Label centred
    int textSize = 3;
    int w = (int)strlen(label) * 6 * textSize;
    int h = 8 * textSize;
    tft.setTextColor(accent); tft.setTextSize(textSize);
    tft.setCursor(x + (SPLASH_BTN_W - w) / 2,
                  SPLASH_BTN_Y + (SPLASH_BTN_H - h) / 2);
    tft.print(label);
}

void drawSplash() {
    tft.fillScreen(C_BG);

    // Title
    const char* title = "aether-pad";
    const int titleSize = 5;
    int tw = (int)strlen(title) * 6 * titleSize;
    tft.setTextColor(C_CYAN); tft.setTextSize(titleSize);
    tft.setCursor((800 - tw) / 2, 30); tft.print(title);

    // Subtitle: callsign + version + build date
    const char* sub = "G0JKN/W3   -   persona menu";
    const int subSize = 2;
    int sw = (int)strlen(sub) * 6 * subSize;
    tft.setTextColor(C_MUTED); tft.setTextSize(subSize);
    tft.setCursor((800 - sw) / 2, 120); tft.print(sub);

    // Three persona buttons. Highlight whichever is the current autoPick.
    drawSplashButton(SPLASH_BTN_TCI_X,   "TCI",
                     C_CYAN,
                     g_splash.autoPick == AetherPadMode::NormalTci);
    drawSplashButton(SPLASH_BTN_RC28_X,  "RC-28",
                     C_AMBER,
                     g_splash.autoPick == AetherPadMode::NormalRc28);
    drawSplashButton(SPLASH_BTN_ACTRL_X, "AetherControl",
                     C_GREEN,
                     g_splash.autoPick == AetherPadMode::NormalAetherControl);

    // Countdown line at the bottom (updated incrementally by splashTick)
    g_splash.lastSecLeft = -1;     // force first draw

    invalidateTileCache();
}

void splashTick() {
    if (!g_splash.active) return;
    if (!g_splash.armed) return;          // boot/WiFi phase — no countdown yet
    unsigned long elapsed = millis() - g_splash.startMs;

    // Auto-continue when timeout elapses
    if (elapsed >= SPLASH_TIMEOUT_MS) {
        AetherPadMode pick = g_splash.autoPick;
        g_splash.active = false;
        Serial.print("Splash: auto-continuing to ");
        Serial.println(PERSONA_LABEL[(uint8_t)pick]);
        switchToPersona(pick);
        return;
    }

    // Countdown only changes once per second — only repaint when the
    // visible integer flips, otherwise we waste a screen redraw 40x/sec.
    int secLeft = (int)((SPLASH_TIMEOUT_MS - elapsed + 999) / 1000);
    if (secLeft != g_splash.lastSecLeft) {
        g_splash.lastSecLeft = secLeft;
        // Wipe + redraw just the countdown strip at the bottom
        tft.fillRect(0, 430, 800, 30, C_BG);
        char buf[64];
        snprintf(buf, sizeof(buf), "Auto-continue in %d  (tap a button to choose)", secLeft);
        const int sz = 2;
        int w = (int)strlen(buf) * 6 * sz;
        tft.setTextColor(C_MUTED); tft.setTextSize(sz);
        tft.setCursor((800 - w) / 2, 432); tft.print(buf);
    }
}

// Paint a status string in the splash's bottom strip — used by wifiConnect()
// to show "Connecting..." over the splash before the countdown is armed.
void splashStatus(const char* msg, uint16_t fg) {
    tft.fillRect(0, 430, 800, 30, C_BG);
    const int sz = 2;
    int w = (int)strlen(msg) * 6 * sz;
    tft.setTextColor(fg); tft.setTextSize(sz);
    tft.setCursor((800 - w) / 2, 432); tft.print(msg);
}

void splashHandleTouch() {
    if (!g_splash.active) return;

    static unsigned long lastTapMs = 0;
    if (millis() - lastTapMs < 220) return;          // simple debounce

    GDTpoint_t tp[5];
    uint8_t n = Touch.getTouchPoints(tp);
    if (n == 0) return;

    int rx = tp[0].x, ry = tp[0].y;
    int tx, ty;
    switch (touchMode) {
        case 1:  tx = 799 - ry; ty = rx;       break;
        case 2:  tx = rx;       ty = ry;       break;
        case 3:  tx = 799 - rx; ty = 479 - ry; break;
        default: tx = ry;       ty = 479 - rx; break;
    }

    // Hit-test the three persona buttons
    if (ty < SPLASH_BTN_Y || ty >= SPLASH_BTN_Y + SPLASH_BTN_H) return;

    AetherPadMode pick;
    if      (tx >= SPLASH_BTN_TCI_X   && tx < SPLASH_BTN_TCI_X   + SPLASH_BTN_W) pick = AetherPadMode::NormalTci;
    else if (tx >= SPLASH_BTN_RC28_X  && tx < SPLASH_BTN_RC28_X  + SPLASH_BTN_W) pick = AetherPadMode::NormalRc28;
    else if (tx >= SPLASH_BTN_ACTRL_X && tx < SPLASH_BTN_ACTRL_X + SPLASH_BTN_W) pick = AetherPadMode::NormalAetherControl;
    else return;

    lastTapMs = millis();
    g_splash.active = false;
    Serial.print("Splash: operator picked ");
    Serial.println(PERSONA_LABEL[(uint8_t)pick]);
    switchToPersona(pick);
}

void openSplash(bool autoToCurrent) {
    g_splash.active      = true;
    g_splash.armed       = true;          // timer runs immediately
    g_splash.startMs     = millis();
    g_splash.autoPick    = autoToCurrent ? currentMode : AetherPadMode::NormalTci;
    g_splash.lastSecLeft = -1;
    drawSplash();
}

// Boot-phase entry: splash drawn but timer disabled. Used so wifiConnect()
// can show "Connecting..." over the splash without the countdown stealing
// seconds during the WiFi associate.
void openSplashBoot() {
    g_splash.active      = true;
    g_splash.armed       = false;
    g_splash.autoPick    = AetherPadMode::NormalTci;
    g_splash.lastSecLeft = -1;
    drawSplash();
}

// Persona entry points. TCI is the historical default and stays wired
// through the existing WiFi/TCI/drawDynamic path. RC-28 and AetherControl
// are stubs at this layer — the persona switch logs the change to Serial;
// actual USB-side enable/disable will land alongside the AetherControl
// emitter once the AetherSDR alias path is settled.
void tciResume() {
    Serial.println("Persona: TCI (resumed)");
}

void rc28PersonaBegin() {
    Serial.println("Persona: RC-28 (HID device already enumerated in Phase A)");
}

void aetherControlBegin() {
    Serial.println("Persona: AetherControl (stub — USB CDC emitter not yet wired)");
}

void switchToPersona(AetherPadMode m) {
    if ((uint8_t)m >= PERSONA_CAROUSEL_COUNT) return;   // ignore Test or out-of-range
    Serial.print("switchToPersona: "); Serial.println(PERSONA_LABEL[(uint8_t)m]);
    currentMode = m;
    switch (m) {
        case AetherPadMode::NormalTci:           tciResume();          break;
        case AetherPadMode::NormalRc28:          rc28PersonaBegin();   break;
        case AetherPadMode::NormalAetherControl: aetherControlBegin(); break;
        default: break;
    }
    drawStatic();           // wipe the carousel frame, restore operator UI
    uiNeedsRedraw = true;   // force drawDynamic on the next loop iteration
}

void drawButton(const Btn& b, uint16_t bg, uint16_t fg) {
    tft.fillRoundRect(b.x, b.y, b.w, b.h, 8, bg);
    tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, C_BORDER);
    tft.setTextColor(fg); tft.setTextSize(4);
    int tw = strlen(b.lbl) * 24;
    tft.setCursor(b.x + (b.w - tw) / 2, b.y + (b.h - 32) / 2);
    tft.print(b.lbl);
}

// Visual icon ID for modal status chips. Drawn alongside the value text
// in the chip so the operator can identify the chip's purpose at a
// glance without reading the label. ICON_NONE = no icon, fall back to
// the original "label-on-top + value-below" text layout.
enum ChipIcon : uint8_t {
    ICON_NONE = 0,
    ICON_SPEAKER,      // VOLUME chip — speaker cone with sound waves
    ICON_WAVE,         // MODE chip — sine wave (one cycle)
    ICON_STAIRS,       // STEP chip — three rising steps
    ICON_KEY           // KEYER chip — straight key (lever + pivot + contact)
};

// Icon footprint helpers — kept in sync with each draw fn's geometry so
// callers can lay out chips around the icons without hard-coding magic
// numbers. Returned width/height are the bounding box around the visible
// pixels, NOT including any internal anchor offset.
static int speakerIconWidth (int s) { return 4 * s + 2; }
static int speakerIconHeight(int s) { return (7 * s) / 2; }
static int waveIconWidth    (int s) { return 4 * s; }
static int waveIconHeight   (int s) { return 2 * s; }
static int stairsIconWidth  (int s) { return 3 * s; }
static int stairsIconHeight (int s) { return 3 * s; }
static int keyIconWidth     (int s) { return 4 * s; }
static int keyIconHeight    (int s) { return 2 * s; }

// Draw a small loudspeaker icon centred on (cx, cy). `s` is a size in
// pixels — body height is roughly 2*s, total width about 4*s. Drawn from
// GFX primitives (no bitmaps) so it scales cleanly. The two "sound wave"
// arcs use Adafruit_GFX's drawCircleHelper with the right-half quadrant
// mask (0x1 | 0x8 = top-right + bottom-right).
static void drawSpeakerIcon(int cx, int cy, int s, uint16_t color) {
    // Speaker body — narrow rectangle on the left
    int bodyW = s;
    int bodyH = (3 * s) / 2;
    tft.fillRect(cx - 2 * s,           cy - bodyH / 2, bodyW, bodyH, color);
    // Speaker cone — trapezoid that widens to the right of the body.
    // Built from a centre fillRect + two triangles to form the flares.
    int coneW = (3 * s) / 2;
    int coneH = bodyH;
    tft.fillRect(cx - 2 * s + bodyW,   cy - coneH / 2, coneW, coneH, color);
    int flareH = s;        // how far the cone flares above/below the body
    int xCone = cx - 2 * s + bodyW;
    int xRim  = xCone + coneW;
    tft.fillTriangle(xCone, cy - coneH / 2,
                     xRim,  cy - coneH / 2 - flareH,
                     xRim,  cy - coneH / 2,            color);
    tft.fillTriangle(xCone, cy + coneH / 2,
                     xRim,  cy + coneH / 2,
                     xRim,  cy + coneH / 2 + flareH,   color);
    // Two right-half-circle "sound waves" radiating from the cone rim.
    // Adafruit_GFX corner bits: 1=upper-left, 2=upper-right,
    // 4=lower-right, 8=lower-left. Right half = 0x2 | 0x4 = 0x6.
    const uint8_t kRightHalf = 0x2 | 0x4;
    tft.drawCircleHelper(xRim + 2, cy, s,           kRightHalf, color);
    tft.drawCircleHelper(xRim + 2, cy, (3 * s) / 2, kRightHalf, color);
}

// Sine wave — one full cycle. Top semicircle on the left, bottom
// semicircle on the right. Footprint 4s × 2s.
static void drawWaveIcon(int cx, int cy, int s, uint16_t color) {
    const uint8_t kUpperHalf = 0x1 | 0x2;   // upper-left + upper-right
    const uint8_t kLowerHalf = 0x4 | 0x8;   // lower-right + lower-left
    // Draw twice with radius s and s-1 to get a 2-pixel-thick stroke
    // — single-pixel arcs disappear at typical viewing distance.
    tft.drawCircleHelper(cx - s, cy, s,     kUpperHalf, color);
    tft.drawCircleHelper(cx - s, cy, s - 1, kUpperHalf, color);
    tft.drawCircleHelper(cx + s, cy, s,     kLowerHalf, color);
    tft.drawCircleHelper(cx + s, cy, s - 1, kLowerHalf, color);
}

// Rising stairs — three filled steps stepping up to the right. Footprint
// 3s × 3s. Each step is s × s.
static void drawStairsIcon(int cx, int cy, int s, uint16_t color) {
    // Anchor: bottom-left of bounding box at (cx - 3s/2, cy + 3s/2).
    int x0 = cx - (3 * s) / 2;
    int y0 = cy + (3 * s) / 2;
    tft.fillRect(x0,             y0 - s,         s, s, color);   // bottom step
    tft.fillRect(x0 + s,         y0 - 2 * s,     s, s, color);   // middle step
    tft.fillRect(x0 + 2 * s,     y0 - 3 * s,     s, s, color);   // top step
}

// Straight CW key — horizontal lever resting on a pivot, with a contact
// knob at the right end. Footprint 4s × 2s.
//
//   ▌════════════●     ← lever + knob
//   ──────────────     ← base
//
static void drawKeyIcon(int cx, int cy, int s, uint16_t color) {
    int leverY = cy - s / 2;
    int leverH = s / 2; if (leverH < 2) leverH = 2;
    int leverX = cx - 2 * s;
    int leverW = 4 * s - s;            // leave room for the knob on the right
    // Pivot — small vertical block at the left end of the lever
    tft.fillRect(leverX,         leverY - s / 2, s / 2, s + s / 2, color);
    // Lever — horizontal bar
    tft.fillRect(leverX + s / 2, leverY,         leverW - s / 2, leverH, color);
    // Contact knob — filled circle at the right tip
    tft.fillCircle(leverX + leverW + s / 2, leverY + leverH / 2, s / 2, color);
    // Base — horizontal line beneath, full width
    int baseY = cy + s / 2 + 2;
    tft.fillRect(leverX, baseY, 4 * s, 2, color);
}

// Draw a modal status chip. Two layouts depending on whether an icon is
// supplied:
//
//   icon == ICON_NONE  →  label-on-top (size 2) + value-below (size 4)
//   icon != ICON_NONE  →  icon-on-left + value-on-right, both at value scale
//                          (icon size matches the value's character height,
//                          ~32 px). The icon serves as the visual label so
//                          the small text label is dropped.
//
// `armed` adds a persona-accent glow border so the operator can see at a
// glance which parameter the encoder is currently controlling. The accent
// colour (cyan / amber / green) tracks the current persona.
static void drawChip(const Btn& b, const char* label, const char* value,
                     uint16_t valColor, bool armed, ChipIcon icon = ICON_NONE) {
    uint16_t border = armed ? personaAccent() : C_BORDER;
    uint16_t bg     = armed ? 0x0820 : C_PANEL;
    tft.fillRoundRect(b.x, b.y, b.w, b.h, 8, bg);
    tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, border);
    if (armed) {
        // Double-stroke for a glow effect — keeps the chip from looking
        // identical to the inactive ones from across the bench.
        tft.drawRoundRect(b.x + 1, b.y + 1, b.w - 2, b.h - 2, 7, border);
    }

    if (icon != ICON_NONE) {
        // Icon-as-label layout. Icon and value sized to match each other
        // visually (s=10 → ~32-42 px icon vs size-4 text 32 px tall).
        const int s = 10;
        int iconW = 0;
        switch (icon) {
            case ICON_SPEAKER: iconW = speakerIconWidth(s); break;
            case ICON_WAVE:    iconW = waveIconWidth(s);    break;
            case ICON_STAIRS:  iconW = stairsIconWidth(s);  break;
            case ICON_KEY:     iconW = keyIconWidth(s);     break;
            default:           iconW = 0; break;
        }
        const bool hasValue = (value && value[0] != '\0');
        const int valW  = hasValue ? (int)strlen(value) * 24 : 0;
        const int gap   = hasValue ? 12 : 0;
        const int totalW = iconW + gap + valW;
        const int gx     = b.x + (b.w - totalW) / 2;
        const int cy     = b.y + b.h / 2;
        const int iconCx = gx + iconW / 2;
        switch (icon) {
            case ICON_SPEAKER: drawSpeakerIcon(iconCx, cy, s, valColor); break;
            case ICON_WAVE:    drawWaveIcon   (iconCx, cy, s, valColor); break;
            case ICON_STAIRS:  drawStairsIcon (iconCx, cy, s, valColor); break;
            case ICON_KEY:     drawKeyIcon    (iconCx, cy, s, valColor); break;
            default: break;
        }
        if (hasValue) {
            tft.setTextSize(4); tft.setTextColor(valColor);
            tft.setCursor(gx + iconW + gap, cy - 16);  // size-4 = 32 tall, half = 16
            tft.print(value);
        }
    } else {
        // Label-on-top + value-below layout. Used for chips that don't
        // (yet) have a dedicated icon — MODE, STEP, KEYER.
        tft.setTextSize(2); tft.setTextColor(C_MUTED);
        tft.setCursor(b.x + 10, b.y + 6);
        tft.print(label);
        tft.setTextSize(4); tft.setTextColor(valColor);
        int vw = strlen(value) * 24;
        int vx = b.x + (b.w - vw) / 2; if (vx < b.x + 8) vx = b.x + 8;
        tft.setCursor(vx, b.y + 36);
        tft.print(value);
    }
}

// Draw a band tile. `active` highlights the band the current VFO is in.
// Generic tile renderer used by every bottom-row mode. `label` may be
// empty for reserved/disabled slots — body still drawn but no text.
// `active` triggers the cyan glow border for the currently-active item
// (current band in BANDS, current mode in MODE, current WPM/iambic in
// KEYER). Text size auto-shrinks for longer labels (e.g. "18 WPM"
// wouldn't fit at size 4 in a 156-wide tile).
static void drawTile(const Btn& b, const char* label, bool active) {
    const uint16_t accent = personaAccent();
    uint16_t border = active ? accent : C_BORDER;
    uint16_t bg     = active ? 0x0820 : C_PANEL;
    uint16_t fg     = active ? accent : C_WHITE;
    tft.fillRoundRect(b.x, b.y, b.w, b.h, 8, bg);
    tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, border);
    if (active) {
        tft.drawRoundRect(b.x + 1, b.y + 1, b.w - 2, b.h - 2, 7, border);
    }
    if (!label || !label[0]) return;            // reserved slot — empty
    int len = (int)strlen(label);
    // Pick text size that fits comfortably with side margin.
    // Size 4 char width = 24, size 3 = 18, size 2 = 12.
    int sz = 4;
    if (len * 24 > b.w - 16) sz = 3;
    if (len * 18 > b.w - 16) sz = 2;
    int charW = sz == 4 ? 24 : (sz == 3 ? 18 : 12);
    int charH = sz == 4 ? 32 : (sz == 3 ? 24 : 16);
    int tw = len * charW;
    tft.setTextSize(sz); tft.setTextColor(fg);
    tft.setCursor(b.x + (b.w - tw) / 2, b.y + (b.h - charH) / 2);
    tft.print(label);
}

void drawDynamic() {
    // TEST mode draws its own dynamic surface (status line refresh only).
    if (currentMode == AetherPadMode::Test) { drawTestDynamic(); return; }
    // Splash / persona menu owns the screen while active.
    if (g_splash.active) return;
    // Per-persona dynamic renderers — each persona's static was already drawn
    // by drawStatic(); here we only refresh the bits that change.
    if (currentMode == AetherPadMode::NormalRc28) {
        drawRc28OperatorDynamic(); return;
    }
    if (currentMode == AetherPadMode::NormalAetherControl) {
        drawACtrlDynamic(); return;
    }

    char buf[64];

    // ── Persona pill (left of the connection pill) ────────────────
    // 100 × 28 outlined badge showing the current persona (TCI / RC-28 /
    // AeCtrl). Coloured-text-on-dark to match the connection pill's look.
    // Hidden in TEST mode (we never reach here from TEST — drawTestDynamic
    // handles that screen — but defensive `isNormalMode()` check anyway).
    if (isNormalMode()) {
        const uint16_t accent = personaAccent();
        const char* personaTxt = PERSONA_PILL[(uint8_t)currentMode];
        tft.fillRect(586, 2, 100, 28, C_PANEL);
        tft.drawRect(586, 2, 100, 28, accent);
        tft.setTextColor(accent); tft.setTextSize(2);
        int ppw = strlen(personaTxt) * 12;
        tft.setCursor(586 + (100 - ppw) / 2, 8);
        tft.print(personaTxt);
    }

    // ── Connection status pill (top right of title bar) ───────────
    uint16_t pillBg, pillFg; const char* pillTxt;
    if (st.tciConnected)      { pillBg = 0x0340; pillFg = C_GREEN; pillTxt = "LIVE"; }
    else if (st.hostKnown)    { pillBg = 0x4200; pillFg = C_AMBER; pillTxt = "...";  }
    else                      { pillBg = 0x2104; pillFg = C_MUTED; pillTxt = "SCAN"; }
    // Pill: 100 wide × 28 tall, text size 2 centred. Aligns vertically
    // with the size-3 title to the left.
    tft.fillRect(696, 2, 100, 28, pillBg);
    tft.drawRect(696, 2, 100, 28, C_BORDER);
    tft.setTextColor(pillFg); tft.setTextSize(2);
    int pw = strlen(pillTxt) * 12;
    tft.setCursor(696 + (100 - pw) / 2, 8);
    tft.print(pillTxt);

    // Host info now lives on the status strip below the chips (drawn
    // later in this function) — there's no room above the freq chip
    // once the title is at size 3.

    // ── Big VFO frequency (size 5, centred, tappable to arm encoder) ──
    formatFreq(st.freqHz, buf, sizeof(buf));
    bool vfoArmed = (encTarget == TGT_VFO);
    uint16_t vfoBg = vfoArmed ? 0x0820 : C_BG;
    tft.fillRect(btnFreqChip.x, btnFreqChip.y, btnFreqChip.w, btnFreqChip.h, vfoBg);
    const uint16_t vfoAccent = personaAccent();
    if (vfoArmed) {
        tft.drawRoundRect(btnFreqChip.x + 4, btnFreqChip.y + 2,
                          btnFreqChip.w - 8, btnFreqChip.h - 4, 6, vfoAccent);
    }
    tft.setTextSize(5); tft.setTextColor(vfoAccent);
    int fw = strlen(buf) * 30;
    tft.setCursor((800 - fw) / 2, btnFreqChip.y + 8); tft.print(buf);
    tft.setTextSize(1); tft.setTextColor(C_MUTED);
    const char* unit = vfoArmed ? "MHz · ENCODER" : "MHz";
    int uw = strlen(unit) * 6;
    tft.setCursor((800 - uw) / 2, btnFreqChip.y + 56); tft.print(unit);

    // ── Four modal status chips ───────────────────────────────────
    // VOL / MODE / STEP all show icon + value. KEYER is icon-only
    // (the WPM and iambic state live on the KEYER tray when armed,
    // where the operator can act on them — they're not constantly
    // useful glanceable info on the chip itself).
    snprintf(buf, sizeof(buf), "%d", st.volume);
    drawChip(btnVolChip,   "VOLUME", buf, C_GREEN, encTarget == TGT_VOL,   ICON_SPEAKER);

    String modeStr = String(MODES[st.modeIdx]); modeStr.toUpperCase();
    drawChip(btnModeChip,  "MODE",   modeStr.c_str(), C_AMBER, encTarget == TGT_MODE, ICON_WAVE);

    if (vfoStepHz >= 1000) snprintf(buf, sizeof(buf), "%ldk", vfoStepHz / 1000);
    else                   snprintf(buf, sizeof(buf), "%ld",  vfoStepHz);
    drawChip(btnStepChip,  "STEP",   buf, C_WHITE, encTarget == TGT_STEP, ICON_STAIRS);

    // Empty value string → drawChip renders the key icon centered alone.
    drawChip(btnKeyerChip, "KEYER",  "",  C_CYAN,  encTarget == TGT_KEYER, ICON_KEY);

    // ── Status strip (host, band, encoder target, diagnostic) ────
    long raw;
    noInterrupts(); raw = encRaw; interrupts();
    tft.fillRect(0, 192, 800, 18, C_BG);
    tft.setTextSize(1); tft.setTextColor(C_MUTED);
    tft.setCursor(8, 196);
    if (st.hostKnown) {
        tft.print(st.host); tft.print(":"); tft.print(st.port);
        if (fallbackArmed) tft.print("(fb)");
    } else {
        tft.print("scanning...");
    }
    tft.print("   BAND "); tft.print(st.bandName);
    tft.print("   ENC ");  tft.print(encTargetName(encTarget));
    tft.print("   raw ");  tft.print(raw);
    tft.print("   tm ");   tft.print(touchMode);

    // ── Bottom-row tiles ──
    // Render whichever "tool tray" the operator is currently looking at
    // (BANDS / MODE / KEYER). The active highlight rule depends on the
    // tile's action: BAND tiles light up for the band containing the
    // current VFO, MODE tiles light up for the current modulation, WPM
    // tiles light up when the preset matches cwWpm, and IAMBIC A/B
    // light up for the active iambic mode.
    const TileData* row = currentTileRow();
    int activeBand = bandIdxForFreq(st.freqHz);
    for (int i = 0; i < N_TILES; i++) {
        const TileData& t = row[i];
        bool active = false;
        const char* label = t.label;
        char wpmBuf[12];

        switch (t.action) {
            case ACT_BAND_SELECT: active = (t.arg == activeBand);  break;
            case ACT_MODE_SET:    active = (t.arg == st.modeIdx);  break;
            case ACT_WPM_SET:     active = (t.arg == cwWpm);       break;
            case ACT_IAMBIC_A:    active = (iambicMode == 'A');    break;
            case ACT_IAMBIC_B:    active = (iambicMode == 'B');    break;
            case ACT_DISPLAY_WPM:
                // Live WPM display — overrides the empty placeholder
                // label with the current value, glows whenever the
                // KEYER tray is up because the encoder is wired to it.
                snprintf(wpmBuf, sizeof(wpmBuf), "%d WPM", cwWpm);
                label  = wpmBuf;
                active = (encTarget == TGT_KEYER);
                break;
            default:              active = false;                  break;
        }

        // Cache check — skip the repaint when this tile's (label, active)
        // pair matches what we last drew here.  Eliminates the flicker
        // that came from clearing and re-rendering identical content
        // every 250 ms.
        TileLastState& ls = gTileLast[i];
        const char* lbl   = label ? label : "";
        bool changed = !ls.valid
                    || ls.active != active
                    || strncmp(ls.label, lbl, sizeof(ls.label) - 1) != 0;
        if (changed) {
            drawTile(btnTile[i], lbl, active);
            strncpy(ls.label, lbl, sizeof(ls.label) - 1);
            ls.label[sizeof(ls.label) - 1] = 0;
            ls.active = active;
            ls.valid  = true;
        }
    }
}

// ===========================================================================
// Helpers
// ===========================================================================
void formatFreq(long hz, char* out, size_t n) {
    // 14250000 → "14.250.000"
    long mhz = hz / 1000000;
    long khz = (hz / 1000) % 1000;
    long  rem = hz % 1000;
    snprintf(out, n, "%ld.%03ld.%03ld", mhz, khz, rem);
}

const char* freqToBand(long hz) {
    long m = hz / 1000;     // kHz
    if (m >=  1810 && m <=  2000) return "160m";
    if (m >=  3500 && m <=  4000) return "80m";
    if (m >=  5260 && m <=  5410) return "60m";
    if (m >=  7000 && m <=  7300) return "40m";
    if (m >= 10100 && m <= 10150) return "30m";
    if (m >= 14000 && m <= 14350) return "20m";
    if (m >= 18068 && m <= 18168) return "17m";
    if (m >= 21000 && m <= 21450) return "15m";
    if (m >= 24890 && m <= 24990) return "12m";
    if (m >= 28000 && m <= 29700) return "10m";
    if (m >= 50000 && m <= 54000) return "6m";
    return "?";
}

// ===========================================================================
// Serial console
// ===========================================================================
void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line == "?") {
        Serial.println("aether_pad state:");
        Serial.print("  Build     "); Serial.println(BUILD_TAG);
        Serial.print("  WiFi      "); Serial.println(WiFi.localIP());
        Serial.print("  Host      ");
        if (st.hostKnown) { Serial.print(st.host); Serial.print(":"); Serial.println(st.port); }
        else              { Serial.println("(searching)"); }
        Serial.print("  TCI       "); Serial.println(st.tciConnected ? "connected" : "down");
        Serial.print("  Freq      "); Serial.print(st.freqHz); Serial.println(" Hz");
        Serial.print("  Mode      "); Serial.println(MODES[st.modeIdx]);
        Serial.print("  Volume    "); Serial.print(st.volume); Serial.println(" %");
        Serial.print("  Band      "); Serial.println(st.bandName);
        Serial.print("  Step      "); Serial.print(vfoStepHz); Serial.println(" Hz");
        Serial.print("  Enc tgt   "); Serial.println(encTargetName(encTarget));
        Serial.print("  Encoder   "); Serial.println(ENC_NAME);
        Serial.print("  Enc div   "); Serial.println(encDivider);
        Serial.print("  Enc raw   "); Serial.println(encRaw);
        Serial.print("  Enc ISRs  "); Serial.println(encIsrCount);
        Serial.print("  D2 (A)    "); Serial.println(digitalRead(ENC_PIN_A) ? "HIGH" : "LOW");
        Serial.print("  D3 (B)    "); Serial.println(digitalRead(ENC_PIN_B) ? "HIGH" : "LOW");
        Serial.print("  CW WPM    "); Serial.println(cwWpm);
        Serial.print("  Iambic    "); Serial.println(iambicMode);
        Serial.print("  CW TX     "); Serial.print(kr.keyDown ? "DOWN" : "up  ");
        Serial.print("  elements="); Serial.println(kr.txCount);
        Serial.print("  D4 (dit)  "); Serial.println(digitalRead(KEY_PIN_DIT) ? "HIGH" : "LOW");
        Serial.print("  D5 (dah)  "); Serial.println(digitalRead(KEY_PIN_DAH) ? "HIGH" : "LOW");
        return;
    }
    if (line.startsWith("ip ")) {
        IPAddress h;
        if (h.fromString(line.substring(3))) {
            st.host = h; st.port = FALLBACK_PORT;
            st.hostKnown = true; fallbackArmed = true;
            tcp.stop();   // force reconnect
            Serial.print("Host override → "); Serial.println(h);
            uiNeedsRedraw = true;
        } else {
            Serial.println("ip: bad address");
        }
        return;
    }
    if (line.startsWith("div ")) {
        int v = line.substring(4).toInt();
        if (v >= 1 && v <= 240) { encDivider = v; Serial.print("encDivider = "); Serial.println(v); }
        return;
    }
    if (line.startsWith("step ")) {
        long v = atol(line.substring(5).c_str());
        if (v >= 1 && v <= 1000000) { vfoStepHz = v; uiNeedsRedraw = true;
            Serial.print("vfoStepHz = "); Serial.println(v); }
        return;
    }
    if (line.startsWith("vol ")) {
        int v = line.substring(4).toInt();
        if (v >= VOL_MIN && v <= VOL_MAX) {
            st.volume = v;
            char buf[24]; snprintf(buf, sizeof(buf), "rx_volume:0,%d;", v);
            wsSendText(buf);
            uiNeedsRedraw = true;
        }
        return;
    }
    if (line.startsWith("freq ")) {
        long v = atol(line.substring(5).c_str());
        if (v >= 100000 && v <= 60000000L) {
            st.freqHz = v;
            strncpy(st.bandName, freqToBand(v), sizeof(st.bandName) - 1);
            char buf[40]; snprintf(buf, sizeof(buf), "vfo:0,0,%ld;", v);
            wsSendText(buf);
            uiNeedsRedraw = true;
        }
        return;
    }
    if (line.startsWith("tm ")) {
        int v = line.substring(3).toInt();
        if (v >= 0 && v <= 3) {
            touchMode = v;
            Serial.print("touchMode = "); Serial.println(v);
            Serial.println("  0 = default (verified)  1 = 180°  2 = raw  3 = flip-both");
            uiNeedsRedraw = true;
        }
        return;
    }
    if (line.startsWith("wpm ")) {
        int v = line.substring(4).toInt();
        if (v >= 5 && v <= 100) {
            cwWpm = v;
            keyerSendWpm();
            Serial.print("CW WPM = "); Serial.println(v);
            uiNeedsRedraw = true;
        } else {
            Serial.println("wpm: range is 5..100");
        }
        return;
    }
    if (line.startsWith("iambic ")) {
        char c = line.charAt(7);
        if (c == 'a' || c == 'A') { iambicMode = 'A'; Serial.println("Iambic A"); }
        else if (c == 'b' || c == 'B') { iambicMode = 'B'; Serial.println("Iambic B"); }
        else Serial.println("iambic: A or B");
        return;
    }
    Serial.println("commands: ?  ip <addr>  div N  step N  vol N  freq N");
    Serial.println("          tm 0..3  wpm N  iambic A|B");
}
