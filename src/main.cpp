#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Wire.h>
#include <ctype.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static const uint16_t ARTNET_PORT = 6454;

// ── Dimmer (LR7843 N-Kanal Logic-Level MOSFET, per PWM angesteuert) ───────────
static const uint8_t  MOSFET_PIN        = D1;
static const bool     MOSFET_ACTIVE_LOW = false; // MOSFET zieht Last gegen GND -> Gate high = an
static const uint16_t PWM_RANGE         = 1023;  // ESP8266 macht PWM in Software: bei 1 kHz ist
static const uint16_t PWM_FREQUENCY_HZ  = 1000;  // ~1000 Stufen die Auflösungsgrenze
static const float    GAMMA             = 2.2f;  // Wahrnehmung ist logarithmisch -> LUT statt linear

// Faellt die Quelle aus, geht der Ausgang aus (Art-Net empfiehlt ~4 s).
static const uint32_t DMX_TIMEOUT_MS = 4000;

// Ein OLED-Refresh dauert ~25 ms, DMX kommt alle ~23 ms: das Display darf
// deshalb nie im DMX-Pfad haengen, sondern nur gedrosselt aus loop() heraus.
static const uint32_t DISPLAY_MIN_INTERVAL_MS = 100;  // max. 10 Hz
static const uint32_t DISPLAY_FORCE_MS        = 1000; // Zwangsrefresh (IP/Status)

static const bool DEBUG_DMX = false; // pro-Paket-Logging kostet ~5 ms und bremst den Empfang

// ── OLED (SSD1306/SSD1315-kompatibel, 128x64) ────────────────────────────────
static const uint8_t OLED_SDA      = D6;
static const uint8_t OLED_SCL      = D5;
static const uint8_t OLED_ADDR     = 0x3C;
static const uint8_t SCREEN_WIDTH  = 128;
static const uint8_t SCREEN_HEIGHT = 64;
// Zweifarbiges Display: oben 16 Zeilen fix gelb, restliche 48 Zeilen fix blau
// (Hardware-Split). Das Layout haelt diese Grenze ein, damit keine Textzeile
// ueber beide Farben laeuft.
static const uint8_t OLED_YELLOW_H = 16;

struct Config {
    uint32_t magic;
    uint16_t universe;   // volle 15-Bit Port-Address (Net/Subnet/Universe), 0-32767
    uint16_t channel;    // 1-512
    uint8_t  trimMin;    // Prozent Duty bei DMX 1   (manche Treiber zuenden erst spaet)
    uint8_t  trimMax;    // Prozent Duty bei DMX 255
    uint16_t fadeMs;     // Weichzeichnen zwischen zwei Werten, 0 = aus
    char     name[18];   // Art-Net ShortName, zugleich OTA-/mDNS-Hostname
};
static const uint32_t CONFIG_MAGIC = 0xC0FFEE03;

static const char* AP_SSID       = "ArtNet-Setup";
static const char* DEFAULT_NAME  = "ArtNet Dimmer";

// Doppel-Reset-Erkennung fuer WLAN-Reset: RST liegt direkt am EN-Pin, waehrend
// der Knopf gedrueckt ist laeuft kein Code -> eine Haltedauer ist nicht messbar.
// Stattdessen: zweimal kurz hintereinander (< DR_TIMEOUT_MS) resetten setzt die
// WLAN-Zugangsdaten zurueck. Das Flag liegt im RTC-Speicher (ueberlebt Reset,
// nicht aber Stromausfall) an Slot 64, um SDK-interne Slots zu meiden.
static const uint32_t DR_MAGIC      = 0xD001D001;
static const uint32_t DR_RTC_ADDR   = 64;
static const uint32_t DR_TIMEOUT_MS = 4000;

bool          doubleResetPending = false;
unsigned long bootMillis         = 0;

Config           cfg;
ESP8266WebServer server(80);
WiFiUDP          udp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
uint8_t          packetBuf[530];
bool             oledOk = false;

// Ausgangs-/Quellenzustand
uint8_t  dmxValue    = 0;      // letzter empfangener Kanalwert
bool     haveDmx     = false;  // ueberhaupt schon ein Paket gesehen
bool     dmxLive     = false;  // innerhalb DMX_TIMEOUT_MS ein Paket gesehen
uint32_t lastDmxMs   = 0;
uint8_t  lastSeq     = 0;
bool     manualMode  = false;  // Test-Slider aus dem Web-UI hat Vorrang
uint8_t  manualValue = 0;

uint16_t gammaLut[256];
uint16_t curDuty      = 0xFFFF; // Sentinel: erster writeDuty() geht garantiert durch
uint16_t fadeFromDuty = 0;
uint16_t fadeToDuty   = 0;
uint32_t fadeStartMs  = 0;

bool     displayDirty  = true;
uint32_t lastDisplayMs = 0;

// ── Double-Reset ──────────────────────────────────────────────────────────────

bool checkAndArmDoubleReset() {
    uint32_t marker = 0;
    ESP.rtcUserMemoryRead(DR_RTC_ADDR, &marker, sizeof(marker));
    bool triggered = (marker == DR_MAGIC);

    marker = DR_MAGIC;
    ESP.rtcUserMemoryWrite(DR_RTC_ADDR, &marker, sizeof(marker));
    return triggered;
}

void clearDoubleResetFlag() {
    uint32_t marker = 0;
    ESP.rtcUserMemoryWrite(DR_RTC_ADDR, &marker, sizeof(marker));
}

// ── Config ────────────────────────────────────────────────────────────────────

static void setName(Config& c, const String& s) {
    size_t j = 0;
    for (size_t i = 0; i < s.length() && j < sizeof(c.name) - 1; i++) {
        char ch = s[i];
        if (ch < 0x20 || ch == 0x7F || ch == '"' || ch == '\\') continue; // JSON-/Art-Net-sicher
        c.name[j++] = ch;
    }
    c.name[j] = '\0';
    if (j == 0) strncpy(c.name, DEFAULT_NAME, sizeof(c.name) - 1);
}

void loadConfig() {
    EEPROM.begin(sizeof(Config));
    Config tmp;
    EEPROM.get(0, tmp);
    EEPROM.end();

    if (tmp.magic == CONFIG_MAGIC) {
        cfg = tmp;
        cfg.name[sizeof(cfg.name) - 1] = '\0';
        return;
    }

    cfg = {CONFIG_MAGIC, 0, 1, 0, 100, 0, {0}};
    strncpy(cfg.name, DEFAULT_NAME, sizeof(cfg.name) - 1);
    EEPROM.begin(sizeof(Config));
    EEPROM.put(0, cfg);
    EEPROM.commit();
    EEPROM.end();
}

void saveConfig(const Config& c) {
    cfg = c;
    cfg.magic = CONFIG_MAGIC;
    EEPROM.begin(sizeof(Config));
    EEPROM.put(0, cfg);
    EEPROM.commit();
    EEPROM.end();
}

String hostName() {
    String h;
    for (const char* p = cfg.name; *p; ++p) h += isalnum((unsigned char)*p) ? *p : '-';
    h.toLowerCase();
    return h.isEmpty() ? String("artnet-dimmer") : h;
}

// ── PWM-Ausgang ───────────────────────────────────────────────────────────────

void buildGammaLut() {
    gammaLut[0] = 0;
    for (uint16_t i = 1; i < 256; i++) {
        uint16_t v = (uint16_t)(powf(i / 255.0f, GAMMA) * PWM_RANGE + 0.5f);
        // Bei 1023 Stufen ergibt Gamma 2.2 fuer die untersten DMX-Werte rechnerisch 0.
        // Untergrenze 1 haelt die Kennlinie monoton; einen echten Boden setzt trimMin.
        gammaLut[i] = v > 0 ? v : 1;
    }
}

uint8_t activeValue() {
    if (manualMode) return manualValue;
    return dmxLive ? dmxValue : 0;
}

uint16_t dutyForValue(uint8_t value) {
    if (value == 0) return 0; // DMX 0 ist hart aus, unabhaengig von trimMin
    uint32_t lo = (uint32_t)PWM_RANGE * cfg.trimMin / 100;
    uint32_t hi = (uint32_t)PWM_RANGE * cfg.trimMax / 100;
    if (hi < lo) hi = lo;
    return (uint16_t)(lo + (uint32_t)gammaLut[value] * (hi - lo) / PWM_RANGE);
}

uint8_t outPercent() {
    return (uint8_t)((curDuty * 100UL + PWM_RANGE / 2) / PWM_RANGE);
}

void writeDuty(uint16_t duty) {
    if (duty == curDuty) return;
    curDuty = duty;
    analogWrite(MOSFET_PIN, MOSFET_ACTIVE_LOW ? PWM_RANGE - duty : duty);
}

// Neues Ziel setzen; bei konfiguriertem Fade uebernimmt serviceFade() das Nachziehen.
void setTarget(uint8_t value) {
    uint16_t duty = dutyForValue(value);
    if (duty == fadeToDuty) return;
    fadeToDuty = duty;
    if (cfg.fadeMs == 0) {
        writeDuty(duty);
        return;
    }
    fadeFromDuty = curDuty;
    fadeStartMs  = millis();
}

void serviceFade() {
    if (curDuty == fadeToDuty) return;
    if (cfg.fadeMs == 0) {
        writeDuty(fadeToDuty);
        return;
    }
    uint32_t t = millis() - fadeStartMs;
    if (t >= cfg.fadeMs) {
        writeDuty(fadeToDuty);
        return;
    }
    int32_t span = (int32_t)fadeToDuty - (int32_t)fadeFromDuty;
    writeDuty((uint16_t)((int32_t)fadeFromDuty + span * (int32_t)t / (int32_t)cfg.fadeMs));
}

// ── Display ───────────────────────────────────────────────────────────────────

void updateDisplay() {
    if (!oledOk) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Gelber Streifen (y 0-15): zwei Statuszeilen, je genau 8 px
    display.setCursor(0, 0);
    display.print("IP:");
    display.print(WiFi.isConnected() ? WiFi.localIP().toString() : String("---"));

    display.setCursor(0, 8);
    display.print("U:");
    display.print(cfg.universe);
    display.print(" Ch:");
    display.print(cfg.channel);

    const char* tag = manualMode ? "MAN" : (dmxLive ? "DMX" : "--");
    display.setCursor(SCREEN_WIDTH - strlen(tag) * 6, 8);
    display.print(tag);

    // Blauer Bereich (y 16-63): Balken mit dem rohen 8-Bit-Wert, darunter der
    // tatsaechliche Ausgang nach Gamma/Trim.
    const int16_t barX = 2, barY = 20, barW = SCREEN_WIDTH - 4, barH = 28;
    int16_t shown = (manualMode || haveDmx) ? (int16_t)activeValue() : -1;
    uint8_t value = shown < 0 ? 0 : (uint8_t)shown;

    display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
    display.fillRect(barX + 1, barY + 1, (uint16_t)value * (barW - 2) / 255, barH - 2,
                     SSD1306_WHITE);

    char valStr[4];
    if (shown < 0) strcpy(valStr, "--");
    else snprintf(valStr, sizeof(valStr), "%u", value);

    display.setTextSize(2);
    int16_t tx1, ty1; uint16_t tw, th;
    display.getTextBounds(valStr, 0, 0, &tx1, &ty1, &tw, &th);
    display.setCursor(barX + (barW - tw) / 2, barY + (barH - th) / 2);
    display.setTextColor(SSD1306_INVERSE); // XOR: lesbar im gefuellten und im leeren Teil
    display.print(valStr);

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    char out[14];
    snprintf(out, sizeof(out), "Out: %u%%", outPercent());
    display.setCursor((SCREEN_WIDTH - strlen(out) * 6) / 2, 52);
    display.print(out);

    display.display();
}

void serviceDisplay() {
    uint32_t now = millis();
    if (!displayDirty && now - lastDisplayMs < DISPLAY_FORCE_MS) return;
    if (now - lastDisplayMs < DISPLAY_MIN_INTERVAL_MS) return;
    lastDisplayMs = now;
    displayDirty  = false;
    updateDisplay();
}

void showSplashScreen() {
    if (!oledOk) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Gelb: Geraetename (TextSize 1 -> 21 Zeichen passen, Name ist max. 17)
    display.setTextSize(1);
    display.setCursor((SCREEN_WIDTH - strlen(cfg.name) * 6) / 2, 4);
    display.print(cfg.name);

    // Blau: grosser Titel + Statuszeile
    display.setTextSize(2);
    display.setCursor(28, 26);
    display.print("Dimmer");

    display.setTextSize(1);
    display.setCursor(37, 54);
    display.print("Starte...");

    display.display();
}

void showWifiResetScreen() {
    if (!oledOk) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(34, 4);
    display.print("WLAN-Reset");

    display.setCursor(0, 26);
    display.print("Doppel-Reset erkannt");
    display.setCursor(0, 38);
    display.print("WLAN-Daten werden");
    display.setCursor(0, 48);
    display.print("geloescht...");

    display.display();
}

void showAPModeScreen() {
    if (!oledOk) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(13, 4);
    display.print("WLAN-Setup noetig");

    display.setCursor(0, 18);
    display.print("Verbinde dich mit:");

    // SSID invertiert hervorgehoben statt in TextSize 2: 12 Zeichen waeren dort
    // 144 px breit und wuerden aus dem Display laufen.
    display.fillRect(0, 28, SCREEN_WIDTH, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor((SCREEN_WIDTH - strlen(AP_SSID) * 6) / 2, 30);
    display.print(AP_SSID);

    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 44);
    display.print("dann im Browser:");
    display.setCursor(0, 54);
    display.print(WiFi.softAPIP().toString());

    display.display();
}

// ── Web UI ────────────────────────────────────────────────────────────────────
// Die Seite ist statisch (send_P, kein String-Gebastel im Heap); alle Werte
// holt sie per /status als JSON und pollt sie zweimal pro Sekunde.

static const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ArtNet Dimmer</title>
<style>
body{font-family:sans-serif;max-width:400px;margin:24px auto;padding:0 16px;color:#222}
h2{margin:0 0 12px}
.box{background:#f4f4f4;padding:12px;border-radius:6px;margin-bottom:16px;font-size:.9em}
.bar{position:relative;height:28px;background:#ddd;border-radius:4px;overflow:hidden;margin:8px 0}
.bar>i{position:absolute;top:0;bottom:0;left:0;width:0;background:#0070f3;transition:width .2s}
.bar>span{position:absolute;top:0;right:0;bottom:0;left:0;display:flex;align-items:center;
  justify-content:center;font-weight:bold;color:#fff;mix-blend-mode:difference}
fieldset{border:1px solid #ccc;border-radius:6px;margin:0 0 16px}
legend{font-size:.85em;color:#666}
label{display:block;font-weight:bold;margin:10px 0 2px;font-size:.85em}
input[type=number],input[type=text]{width:100%;padding:8px;box-sizing:border-box;
  border:1px solid #ccc;border-radius:4px;font-size:1em}
input[type=range]{width:100%}
button{width:100%;padding:10px;margin-top:12px;background:#0070f3;color:#fff;
  border:none;border-radius:4px;font-size:1em;cursor:pointer}
button.alt{background:#777}
.hint{font-weight:normal;color:#666}
#msg{text-align:center;color:green;min-height:1.3em}
</style></head><body>
<h2 id="nm">ArtNet Dimmer</h2>
<div class="box">
  <div><b id="ip">--</b> &middot; <span id="st">--</span></div>
  <div class="bar"><i id="fill"></i><span id="pct">--</span></div>
  <div>DMX-Wert: <b id="val">--</b> / 255</div>
</div>
<fieldset><legend>Test ohne Controller</legend>
  <input type="range" id="sl" min="0" max="255" value="0">
  <button class="alt" id="rel">Wieder DMX folgen</button>
</fieldset>
<form id="f"><fieldset><legend>Konfiguration</legend>
  <label>Name <span class="hint">(im Netz erst nach Neustart)</span></label>
  <input type="text" name="name" maxlength="17">
  <label>Universe <span class="hint">0&#8211;32767</span></label>
  <input type="number" name="uni" min="0" max="32767">
  <label>Kanal <span class="hint">1&#8211;512</span></label>
  <input type="number" name="ch" min="1" max="512">
  <label>Min-Helligkeit <span class="hint">% Leistung bei DMX 1</span></label>
  <input type="number" name="min" min="0" max="100">
  <label>Max-Helligkeit <span class="hint">% Leistung bei DMX 255</span></label>
  <input type="number" name="max" min="0" max="100">
  <label>Fade <span class="hint">ms, 0 = aus</span></label>
  <input type="number" name="fade" min="0" max="5000">
  <button>Speichern</button>
</fieldset></form>
<p id="msg"></p>
<script>
const $=i=>document.getElementById(i);
let filled=false,hold=false,pend=null;
async function poll(){
 try{
  const s=await(await fetch('/status')).json();
  $('nm').textContent=s.name;document.title=s.name;
  $('ip').textContent=s.ip;
  $('st').textContent=s.man?'Manuell':(s.dmx?'DMX aktiv':'kein DMX');
  $('val').textContent=s.val<0?'--':s.val;
  $('pct').textContent=s.out+'%';$('fill').style.width=s.out+'%';
  const e=$('f').elements;
  if(!filled){e.name.value=s.name;e.uni.value=s.uni;e.ch.value=s.ch;
   e.min.value=s.min;e.max.value=s.max;e.fade.value=s.fade;filled=true;}
  if(!hold&&!s.man)$('sl').value=s.val<0?0:s.val;
 }catch(err){}
}
$('sl').oninput=()=>{hold=true;clearTimeout(pend);
 pend=setTimeout(()=>fetch('/set?v='+$('sl').value),60);};
$('rel').onclick=()=>{hold=false;fetch('/set?auto=1');};
$('f').onsubmit=async ev=>{ev.preventDefault();
 await fetch('/save',{method:'POST',body:new URLSearchParams(new FormData($('f')))});
 $('msg').textContent='Gespeichert!';setTimeout(()=>{$('msg').textContent=''},2000);};
setInterval(poll,500);poll();
</script></body></html>)HTML";

void handleRoot() {
    server.send_P(200, PSTR("text/html; charset=utf-8"), HTML);
}

void handleStatus() {
    char buf[288];
    snprintf(buf, sizeof(buf),
             "{\"ip\":\"%s\",\"name\":\"%s\",\"uni\":%u,\"ch\":%u,\"val\":%d,"
             "\"out\":%u,\"dmx\":%u,\"man\":%u,\"min\":%u,\"max\":%u,\"fade\":%u}",
             WiFi.localIP().toString().c_str(), cfg.name,
             cfg.universe, cfg.channel,
             (manualMode || haveDmx) ? (int)activeValue() : -1,
             outPercent(), dmxLive ? 1 : 0, manualMode ? 1 : 0,
             cfg.trimMin, cfg.trimMax, cfg.fadeMs);
    server.send(200, "application/json", buf);
}

void handleSave() {
    Config n = cfg;
    if (server.hasArg("uni"))  n.universe = constrain(server.arg("uni").toInt(),  0, 32767);
    if (server.hasArg("ch"))   n.channel  = constrain(server.arg("ch").toInt(),   1, 512);
    if (server.hasArg("min"))  n.trimMin  = constrain(server.arg("min").toInt(),  0, 100);
    if (server.hasArg("max"))  n.trimMax  = constrain(server.arg("max").toInt(),  0, 100);
    if (server.hasArg("fade")) n.fadeMs   = constrain(server.arg("fade").toInt(), 0, 5000);
    if (server.hasArg("name")) setName(n, server.arg("name"));
    if (n.trimMax < n.trimMin) n.trimMax = n.trimMin;

    saveConfig(n);
    setTarget(activeValue()); // Trim/Fade sofort wirksam
    displayDirty = true;
    Serial.printf("Config: '%s' U%u Ch%u trim %u-%u%% fade %ums\n",
                  cfg.name, cfg.universe, cfg.channel, cfg.trimMin, cfg.trimMax, cfg.fadeMs);
    server.send(200, "application/json", "{\"ok\":1}");
}

void handleSet() {
    if (server.hasArg("auto")) {
        manualMode = false;
    } else if (server.hasArg("v")) {
        manualValue = constrain(server.arg("v").toInt(), 0, 255);
        manualMode  = true;
    }
    setTarget(activeValue());
    displayDirty = true;
    server.send(200, "application/json", "{\"ok\":1}");
}

// ── ArtNet ────────────────────────────────────────────────────────────────────

static bool isArtNet(const uint8_t* buf) {
    return memcmp(buf, "Art-Net\0", 8) == 0;
}

static uint16_t opCode(const uint8_t* buf) {
    return buf[8] | (uint16_t(buf[9]) << 8);
}

static void sendPollReply(IPAddress dest) {
    uint8_t r[239];
    memset(r, 0, sizeof(r));
    memcpy(r, "Art-Net\0", 8);
    r[8]  = 0x00; r[9]  = 0x21;                 // OpPollReply
    IPAddress ip = WiFi.localIP();
    r[10] = ip[0]; r[11] = ip[1]; r[12] = ip[2]; r[13] = ip[3];
    r[14] = 0x36; r[15] = 0x19;                 // Port 6454, low byte first
    r[16] = 0x00; r[17] = 0x01;                 // VersInfo
    r[18] = (cfg.universe >> 8) & 0x7F;         // NetSwitch
    r[19] = (cfg.universe >> 4) & 0x0F;         // SubSwitch
    r[20] = 0x00; r[21] = 0xFF;                 // Oem: unbekannt/Prototyp
    r[23] = 0xE0;                               // Status1: Indikatoren normal, Adressierung per Netz
    r[24] = 0xFF; r[25] = 0x7F;                 // EstaMan: Prototyp-Bereich
    strncpy((char*)&r[26], cfg.name, 17);       // ShortName (18 Byte Feld)
    snprintf((char*)&r[44], 64, "%s (ESP8266 Art-Net Dimmer)", cfg.name); // LongName
    snprintf((char*)&r[108], 64, "#0001 [0000] %s",
             dmxLive ? "DMX OK" : "kein DMX");   // NodeReport
    r[173] = 0x01;                              // NumPortsLo: 1 Port
    r[174] = 0x80;                              // PortTypes[0]: DMX512-Ausgang
    r[182] = dmxLive ? 0x80 : 0x00;             // GoodOutput[0]: Daten werden ausgegeben
    r[190] = cfg.universe & 0x0F;               // SwOut[0]
    r[200] = 0x00;                              // Style: StNode
    uint8_t mac[6]; WiFi.macAddress(mac);
    memcpy(&r[201], mac, 6);
    r[212] = 0x08;                              // Status2: per Browser konfigurierbar

    udp.beginPacket(dest, ARTNET_PORT);
    udp.write(r, sizeof(r));
    udp.endPacket();
}

static void handleDMX(const uint8_t* buf, int len) {
    if (len < 18) return;
    uint8_t  seq      = buf[12];
    uint16_t universe = (buf[14] | (uint16_t(buf[15]) << 8)) & 0x7FFF;
    uint16_t dataLen  = (uint16_t(buf[16]) << 8) | buf[17];

    if (universe != cfg.universe) return;

    int idx = cfg.channel - 1;
    if ((int)dataLen <= idx || len < 18 + idx + 1) {
        if (DEBUG_DMX)
            Serial.printf("[DMX] nur %u Kanaele, Ch%u nicht enthalten\n", dataLen, cfg.channel);
        return;
    }

    // UDP darf umordnen. seq == 0 heisst "Sender nutzt keine Sequenz"; nach einer
    // Pause (Sender-Neustart) wird die Folge nicht mehr bewertet.
    uint32_t now = millis();
    if (seq != 0 && lastSeq != 0 && now - lastDmxMs < 1000 && (int8_t)(seq - lastSeq) <= 0) return;
    lastSeq   = seq;
    lastDmxMs = now;

    if (!dmxLive) {
        dmxLive      = true;
        displayDirty = true;
    }
    haveDmx = true;

    uint8_t val = buf[18 + idx];
    if (val != dmxValue) {
        dmxValue     = val;
        displayDirty = true;
        if (DEBUG_DMX)
            Serial.printf("[DMX] U%u Ch%u=%u\n", universe, cfg.channel, val);
    }
    if (!manualMode) setTarget(val);
}

void serviceDmxTimeout() {
    if (!dmxLive || millis() - lastDmxMs <= DMX_TIMEOUT_MS) return;
    dmxLive      = false;
    displayDirty = true;
    Serial.println("DMX-Ausfall — Ausgang aus");
    if (!manualMode) setTarget(0);
}

// ── Main ──────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println("\n--- ArtNet Dimmer Node ---");

    pinMode(MOSFET_PIN, OUTPUT);
    analogWriteFreq(PWM_FREQUENCY_HZ);
    analogWriteRange(PWM_RANGE);
    buildGammaLut();
    writeDuty(0);

    bootMillis         = millis();
    doubleResetPending = checkAndArmDoubleReset();

    loadConfig();
    Serial.printf("Config: '%s' U%u Ch%u trim %u-%u%% fade %ums\n",
                  cfg.name, cfg.universe, cfg.channel, cfg.trimMin, cfg.trimMax, cfg.fadeMs);

    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.beginTransmission(OLED_ADDR);
    oledOk = (Wire.endTransmission() == 0);
    if (oledOk) {
        display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
        showSplashScreen();
        delay(1500);
    } else {
        Serial.printf("Kein OLED auf I2C 0x%02X — laeuft ohne Display weiter\n", OLED_ADDR);
    }

    WiFiManager wm;
    if (doubleResetPending) {
        Serial.println("Doppel-Reset erkannt — loesche WLAN-Zugangsdaten");
        showWifiResetScreen();
        wm.resetSettings();
        delay(1500);
    }
    wm.setAPCallback([](WiFiManager*) {
        Serial.printf("AP-Modus: Verbinde dich mit '%s'\n", AP_SSID);
        showAPModeScreen();
    });
    if (!wm.autoConnect(AP_SSID)) {
        Serial.println("WiFi fehlgeschlagen — Neustart");
        delay(3000);
        ESP.restart();
    }

    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());

    server.on("/",       HTTP_GET,  handleRoot);
    server.on("/status", HTTP_GET,  handleStatus);
    server.on("/set",    HTTP_GET,  handleSet);
    server.on("/save",   HTTP_POST, handleSave);
    server.begin();

    String host = hostName();
    ArduinoOTA.setHostname(host.c_str()); // startet zugleich den mDNS-Responder
    ArduinoOTA.onStart([]() {
        setTarget(0);
        writeDuty(0); // waehrend des Flashens laeuft kein PWM-Timer
        if (oledOk) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(31, 4);
            display.print("OTA-Update");
            display.setCursor(0, 30);
            display.print("Firmware wird");
            display.setCursor(0, 40);
            display.print("geschrieben...");
            display.display();
        }
    });
    ArduinoOTA.begin();
    MDNS.addService("http", "tcp", 80);

    Serial.printf("Web: http://%s.local (%s), OTA-Host: %s\n",
                  host.c_str(), WiFi.localIP().toString().c_str(), host.c_str());

    udp.begin(ARTNET_PORT);
    Serial.printf("Art-Net Port %u, Universe %u, Ch%u\n",
                  ARTNET_PORT, cfg.universe, cfg.channel);

    displayDirty = true;
}

void loop() {
    server.handleClient();
    ArduinoOTA.handle();
    MDNS.update();

    static bool drFlagCleared = false;
    if (!drFlagCleared && millis() - bootMillis > DR_TIMEOUT_MS) {
        clearDoubleResetFlag();
        drFlagCleared = true;
    }

    // Bis zu 8 Pakete pro Durchlauf: holt den Empfang nach einem Display-Refresh
    // wieder ein, ohne loop() unbegrenzt zu blockieren.
    for (int i = 0; i < 8; i++) {
        if (udp.parsePacket() <= 0) break;
        int len = udp.read(packetBuf, sizeof(packetBuf));
        if (len < 12 || !isArtNet(packetBuf)) continue;
        switch (opCode(packetBuf)) {
            case 0x2000: sendPollReply(udp.remoteIP()); break;
            case 0x5000: handleDMX(packetBuf, len);     break;
        }
    }

    serviceFade();
    serviceDmxTimeout();
    serviceDisplay();
}
