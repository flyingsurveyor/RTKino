// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "WebUI.h"
#include "LeafletAssets.h"

// OTA Manager
#include "OTAManager.h"

// BLE Serial (for settings page)
#include "BLESerial.h"
#include "BleRtcmClient.h"

// Flash config (LittleFS-based config storage)
#include "FlashConfig.h"

// Time sync globals (defined in main.cpp)
extern char g_ntpServer[64];
extern char g_ntpTz[64];
extern char g_mdnsName[32];
extern volatile uint8_t g_timeSource;
extern volatile time_t  g_lastSyncEpoch;

// Time sync function (defined in main.cpp)
extern bool syncTimeFromNtp(const char* server);

// Timezone apply helper (defined in main.cpp)
extern void applyTimezone();

// mDNS apply helper (defined in main.cpp)
extern bool applyMdnsHostname(const char* hostname);

// Position and RTCM types — shared definition in gnss_types.h
#include "gnss_types.h"

extern GNSSPosition g_position;
extern SemaphoreHandle_t positionMutex;
extern bool getPosition(GNSSPosition& out);

extern RtcmStats g_rtcmStats;
extern SemaphoreHandle_t rtcmStatsMutex;
extern bool getRtcmStats(RtcmStats& out);
extern void resetRtcmStats();

// Survey State (for base position averaging)
struct SurveyResults {
  bool active;
  bool complete;
  uint32_t elapsed;
  uint32_t duration;
  uint32_t sampleCount;
  double latMean, lonMean, altMean;
  double latStdDev, lonStdDev, altStdDev;
  double altGround;
  float instrumentHeight;
  float arpOffset;
};
extern void startSurvey(uint32_t durationSec, float instrumentHeight, float arpOffset);
extern void stopSurvey();
extern bool getSurveyResults(SurveyResults& out);

// SurveyPoints (point survey feature — primary storage on flash)
#include "SurveyPoints.h"

// PointCodes (point code/category library)
#include "PointCodes.h"

// Stakeout (navigate to design points)
#include "Stakeout.h"

// ZED-F9P TMODE State (defined in main.cpp, struct in UbxValset.h)
#include "UbxValset.h"  // Assicurati che sia incluso in alto

extern ZedTmodeState g_zedTmode;  // La variabile globale da main.cpp
extern bool getZedTmode(ZedTmodeState& out);

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>
#include <algorithm>

#include "OledLogger.h"
#include "TcpStreamer.h"
#include "UbxValset.h"
#include "WifiProfiles.h"
#include "NtripClient.h"
#include "Buzzer.h"
#include "SystemLog.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// definito in main.cpp: mutex globale SD/SPI
extern SemaphoreHandle_t sdMutex;
extern SdFat sd;
extern bool sdOK;

static bool sdLock(uint32_t timeoutMs = 2000) {
  return (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}
static void sdUnlock(bool locked) {
  if (locked) xSemaphoreGive(sdMutex);
}

// === Mutex per ntripClient (definito in main.cpp) ===
extern SemaphoreHandle_t ntripMutex;
extern bool ntripLock(uint32_t timeoutMs);
extern void ntripUnlock();

struct SdLockGuard {
  bool locked;
  explicit SdLockGuard(uint32_t timeoutMs = 2000)
  : locked(sdLock(timeoutMs)) {}
  ~SdLockGuard() { sdUnlock(locked); }
};

extern NtripClient* ntripClient;
extern HardwareSerial RTCMSerial;
extern volatile bool ntripEnabled;

extern void sendCfgRateToZED(uint16_t rateMs);
extern String lastRateSet;
extern void toggleNtrip(bool enable);

// Wrapper (definiti in main.cpp)
extern void applyBaseValset(uint16_t stid /*=1*/);
extern void applyBaseFixedLLH(double lat_deg, double lon_deg, double h_m, uint16_t stid /*=1*/, uint8_t rtcmType /*=0*/);
extern void stopBaseMode();
extern void readZedTmode();
extern void switchToRover();

// Stato/controllo OUT (definiti in main.cpp)
bool startCasterOut(const String& host, uint16_t port, const String& mount, const String& pass);
void stopCasterOut();
bool startTcpOut(uint16_t port);
void stopTcpOut();
bool startTcpOutClient(const String& host, uint16_t port);
void stopTcpOutClient();
// ---- LAN IN (TCP) control (definiti in main.cpp) ----
bool startTcpIn(const String& host, int port);
void stopTcpIn();
extern bool tcpInEnabled;
extern String tcpin_host; extern int tcpin_port;

// ---- NTRIP IN global variables (defined in main.cpp) ----
extern String ntrip_host, mountpoint, ntrip_user, ntrip_pass;
extern int ntrip_port;

// ===== BLE support (defined in main.cpp) =====
extern bool g_bleEnabled;
extern char g_bleDeviceName[21];
extern uint32_t g_blePasskey;

// ===== BLE RTCM input (defined in main.cpp) =====
extern bool g_bleRtcmEnabled;
extern char g_bleRtcmTargetName[21];
extern uint32_t g_bleRtcmPasskey;
extern BleRtcmClient g_bleRtcm;
extern bool startBleRtcm(const String& targetName, uint32_t passkey);
extern void stopBleRtcm();
extern void toggleBleRtcm(bool enable);
extern bool loadBleName(char* out, size_t maxLen);
extern bool saveBleName(const char* name);
extern bool loadBlePin(uint32_t* out);
extern bool saveBlePin(uint32_t pin);
extern bool applyBleName(const char* newName);

// Stream mode extern
extern "C" {
  void setStreamModeRaw();
  void setStreamModeNmea();
  const char* getStreamModeName();
}

// --- RAW logging controls (defined in main.cpp) ---
extern void startLogging();
extern void stopLogging();
extern bool loggingActive;

// --- Buzzer and SystemLog (defined in main.cpp) ---
extern Buzzer* g_buzzer;
extern SystemLog* g_systemLog;

static SdFat* _sd;
static WebServer* _server;

// Audio upload configuration
#define MAX_MELODY_UPLOAD_SIZE 2048

// ========================================================================
// CSS IN PROGMEM - Modern responsive design
// ========================================================================
static const char CSS_CONTENT[] PROGMEM = R"CSS(
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #f5f5f5; }
.header { background: #2c3e50; color: white; padding: 12px 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
.header h1 { font-size: 22px; font-weight: 600; }
.nav { background: #34495e; display: flex; justify-content: space-around; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
.nav a { flex: 1; color: white; padding: 14px 10px; text-decoration: none; text-align: center; transition: background 0.3s; display: flex; align-items: center; justify-content: center; gap: 5px; }
.nav a:hover, .nav a.active { background: #2c3e50; }
.nav-icon { font-size: 20px; line-height: 1; }
.nav-text { font-size: 14px; }
.container { max-width: 1200px; margin: 20px auto; padding: 0 20px; }
.card { background: white; border-radius: 8px; padding: 20px; margin-bottom: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
.card h2 { color: #2c3e50; margin-bottom: 16px; font-size: 20px; border-bottom: 2px solid #3498db; padding-bottom: 8px; }
.card h3 { color: #34495e; margin: 16px 0 12px; font-size: 16px; }
.status-row { display: flex; align-items: center; margin: 8px 0; }
.status-led { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin: 0 8px 0 4px; }
.led-on { background: #2ecc71; box-shadow: 0 0 6px #2ecc71; }
.led-off { background: #7f8c8d; }
button, .btn { background: #3498db; color: white; border: none; padding: 10px 16px; border-radius: 4px; cursor: pointer; text-decoration: none; display: inline-block; margin: 4px 2px; transition: background 0.3s; font-size: 14px; }
button:hover, .btn:hover { background: #2980b9; }
.btn-success { background: #2ecc71; }
.btn-success:hover { background: #27ae60; }
.btn-danger { background: #e74c3c; }
.btn-danger:hover { background: #c0392b; }
.btn-secondary { background: transparent; border: 1px solid #95a5a6; color: #95a5a6; }
.btn-secondary:hover { background: #95a5a6; color: white; }
.btn-small { padding: 6px 12px; font-size: 12px; }
table { border-collapse: collapse; width: 100%; margin: 12px 0; }
th { background: #ecf0f1; color: #2c3e50; font-weight: 600; text-align: left; }
th, td { border: 1px solid #ddd; padding: 10px; }
tr:nth-child(even) { background: #f9f9f9; }
tr:hover { background: #f1f1f1; }
input, select, textarea { padding: 8px; border: 1px solid #ddd; border-radius: 4px; margin: 4px 0; font-size: 14px; font-family: inherit; }
input[type="text"], input[type="number"], input[type="password"], select { width: 100%; max-width: 320px; }
label { display: block; margin: 12px 0 4px; color: #2c3e50; font-weight: 500; }
form.inline { display: inline; }
.badge { padding: 4px 10px; border: 1px solid #bdc3c7; border-radius: 12px; font-size: 12px; background: #ecf0f1; color: #2c3e50; display: inline-block; }
.log-list { list-style: none; }
.log-list li { padding: 12px; border: 1px solid #ddd; border-radius: 4px; margin: 8px 0; background: #fafafa; }
.log-list li:hover { background: #f0f0f0; }
/* Mobile responsive styles */
@media screen and (max-width: 768px) {
  body {
    font-size: 14px;
    padding: 5px;
    margin: 0;
  }
  
  .container {
    padding: 5px;
    margin: 0;
  }
  
  .card {
    padding: 10px;
    margin-bottom: 10px;
  }
  
  h1 {
    font-size: 1.5em;
  }
  
  h2, h3 {
    font-size: 1.2em;
  }
  
  /* Tables: horizontal scroll instead of breaking layout */
  table {
    display: block;
    overflow-x: auto;
    white-space: nowrap;
    -webkit-overflow-scrolling: touch;
  }
  
  th, td {
    padding: 6px 8px;
    font-size: 12px;
  }
  
  /* Hide table headers on mobile */
  table thead,
  table tr th {
    display: none !important;
  }
  
  /* Form inputs full width */
  input[type="text"],
  input[type="number"],
  input[type="password"],
  select {
    width: 100%;
    max-width: 100%;
    box-sizing: border-box;
    font-size: 16px; /* 16px minimum prevents iOS Safari auto-zoom on input focus */
    padding: 10px;
  }
  
  /* Buttons touch-friendly */
  button, .btn {
    min-height: 44px;
    padding: 8px 12px;
    font-size: 12px;
    width: 100%;
    margin-bottom: 8px;
  }
  
  /* Status rows */
  .status-row {
    padding: 8px 0;
    border-bottom: 1px solid #eee;
  }
  
  /* Navigation - ALWAYS on one row */
  .nav {
    display: flex;
    flex-wrap: nowrap;
    overflow-x: auto;
    -webkit-overflow-scrolling: touch;
  }
  
  .nav a {
    flex: 1;
    min-width: 60px;
    padding: 14px 8px;
    font-size: 13px;
  }
  
  /* Hide text for Home and Settings on mobile - ICONS ONLY */
  .nav a[href="/"] .nav-text,
  .nav a[href="/settings"] .nav-text {
    display: none;
  }
  
  /* Bigger icons (only visible for Home/Settings) */
  .nav a .nav-icon {
    font-size: 26px;
  }
  
  /* Rover and Base keep text */
  .nav a[href="/rover"] .nav-text,
  .nav a[href="/base-cfg"] .nav-text {
    display: inline;
    font-size: 13px;
  }
  
  /* Quick actions grid */
  .quick-actions {
    display: flex;
    flex-direction: column;
  }
  
  .quick-actions button {
    margin-bottom: 8px;
  }
  
  /* Mobile responsive tables - card-based layout */
  .responsive-table table,
  .responsive-table thead,
  .responsive-table tbody,
  .responsive-table th,
  .responsive-table td,
  .responsive-table tr {
    display: block;
  }
  
  .responsive-table thead tr {
    position: absolute;
    top: -9999px;
    left: -9999px;
  }
  
  .responsive-table tr {
    margin-bottom: 15px;
    border: 1px solid #ddd;
    border-radius: 8px;
    background: white;
    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
  }
  
  .responsive-table td {
    border: none;
    position: relative;
    padding-left: 50%;
    text-align: right;
    min-height: 40px;
    display: flex;
    align-items: center;
    justify-content: flex-end;
  }
  
  .responsive-table td:before {
    content: attr(data-label);
    position: absolute;
    left: 10px;
    width: 45%;
    padding-right: 10px;
    white-space: nowrap;
    font-weight: bold;
    text-align: left;
  }
  
  .responsive-table td button,
  .responsive-table td .btn {
    margin: 2px;
  }
}

/* Extra small screens (phones in portrait) */
@media screen and (max-width: 480px) {
  body {
    font-size: 13px;
  }
  
  th, td {
    padding: 4px 6px;
    font-size: 11px;
  }
  
  h1 {
    font-size: 1.3em;
  }
}
)CSS";

// ========================================================================
// HELPER FUNCTIONS - MUST BE DEFINED BEFORE USE
// ========================================================================

// Chunked response helpers
static void sendChunk(const char* str) {
  _server->sendContent(str);
}

static void sendChunk(const String& str) {
  _server->sendContent(str);
}

// Send PROGMEM string in 512-byte chunks to avoid large heap allocations.
// On ESP32 flash is memory-mapped, but strlen_P/memcpy_P are used for
// correctness and forward compatibility with AVR-style PROGMEM targets.
static void sendChunkPROGMEM(const char* pgm_ptr) {
  const size_t CHUNK = 512;
  size_t len = strlen_P(pgm_ptr);
  size_t offset = 0;
  while (offset < len) {
    size_t n = (CHUNK < len - offset) ? CHUNK : (len - offset);
    char buf[CHUNK + 1];
    memcpy_P(buf, pgm_ptr + offset, n);
    buf[n] = '\0';
    _server->sendContent(buf);
    offset += n;
  }
}

// Time source name helper
static const char* timeSourceName(uint8_t src) {
  if (src == 1) return "NTP";
  if (src == 2) return "UBX-NAV-TIMEUTC";
  return "NON SYNC";
}

// NTP server file helpers
static bool loadNtpServerFile(String& out) {
  String content = FlashConfig::readFile("/config/ntp.txt");
  content.trim();
  if (content.length() < 2) return false;
  out = content;
  return true;
}

static bool saveNtpServerFile(const String& s) {
  return FlashConfig::writeFile("/config/ntp.txt", s + "\n");
}

// NTP timezone file helpers
static bool loadNtpTzFile(String& out) {
  String content = FlashConfig::readFile("/config/tz.txt");
  content.trim();
  if (content.length() < 2) return false;
  out = content;
  return true;
}

static bool saveNtpTzFile(const String& s) {
  return FlashConfig::writeFile("/config/tz.txt", s + "\n");
}

// mDNS hostname file helpers
static bool loadMdnsNameFile(String& out) {
  String content = FlashConfig::readFile("/config/mdns.txt");
  content.trim();
  if (content.endsWith(".local")) content = content.substring(0, content.length() - 6);
  content.toLowerCase();
  if (content.length() < 1) return false;
  out = content;
  return true;
}

static bool saveMdnsNameFile(const String& s) {
  return FlashConfig::writeFile("/config/mdns.txt", s + "\n");
}

static bool isValidMdnsHostLabel(const String& s) {
  int n = s.length();
  if (n < 1 || n > 32) return false;
  if (s[0] == '-' || s[n - 1] == '-') return false;
  for (int i = 0; i < n; ++i) {
    char c = s[i];
    bool ok = ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c == '-'));
    if (!ok) return false;
  }
  return true;
}

// String sanitizer
static String clampSemi(const String& s){
  String r=s; r.replace(";", "_"); r.replace("\n"," "); r.replace("\r"," "); return r;
}

// Helper function to replace all occurrences of a substring
static String replaceAll(const String& str, const String& from, const String& to) {
  String result = str;
  int pos = 0;
  while ((pos = result.indexOf(from, pos)) != -1) {
    result = result.substring(0, pos) + to + result.substring(pos + from.length());
    pos += to.length();
  }
  return result;
}

// HTML escape for safe display
static String htmlEscape(const String& s) {
  String r = s;
  r = replaceAll(r, "&", "&amp;");   // Must be first to avoid double-escaping
  r = replaceAll(r, "<", "&lt;");
  r = replaceAll(r, ">", "&gt;");
  r = replaceAll(r, "\"", "&quot;");
  r = replaceAll(r, "'", "&#39;");
  return r;
}

// JSON escape for safe JSON strings
static String jsonEscape(const String& s) {
  String r = s;
  r = replaceAll(r, "\\", "\\\\");
  r = replaceAll(r, "\"", "\\\"");
  r = replaceAll(r, "/", "\\/");
  r = replaceAll(r, "\b", "\\b");
  r = replaceAll(r, "\f", "\\f");
  r = replaceAll(r, "\n", "\\n");
  r = replaceAll(r, "\r", "\\r");
  r = replaceAll(r, "\t", "\\t");
  return r;
}

// JavaScript escape for safe use in onclick and other JavaScript string contexts
static String jsEscape(const String& s) {
  String r = s;
  r = replaceAll(r, "\\", "\\\\");  // Backslash must be first
  r = replaceAll(r, "'", "\\'");     // Escape single quotes
  r = replaceAll(r, "\"", "\\\"");   // Escape double quotes
  r = replaceAll(r, "<", "\\x3C");   // Escape < to prevent script injection
  r = replaceAll(r, ">", "\\x3E");   // Escape > to prevent script injection
  r = replaceAll(r, "&", "\\x26");   // Escape & 
  r = replaceAll(r, "/", "\\/");     // Escape / to prevent breaking </script>
  r = replaceAll(r, "\n", "\\n");    // Newline
  r = replaceAll(r, "\r", "\\r");    // Carriage return
  r = replaceAll(r, "\t", "\\t");    // Tab
  return r;
}

// Helper function for escaping strings used in onclick attributes
// Applies both JavaScript and HTML escaping to prevent injection attacks
static String escapeForOnclick(const String& s) {
  return htmlEscape(jsEscape(s));
}

// Path validation: reject path traversal and restrict to safe directories
static bool isValidLogPath(const String& path) {
  if (path.indexOf("..") >= 0) return false;
  if (path.startsWith("/gnss/system_log_")) return true;
  if (path.startsWith("/gnss/log_")) return true;
  return false;
}

static bool isValidGnssPath(const String& filename) {
  // filename is appended to "/gnss/" — reject traversal and slashes
  if (filename.indexOf("..") >= 0) return false;
  if (filename.indexOf("/") >= 0) return false;
  if (filename.indexOf("\\") >= 0) return false;
  return filename.length() > 0;
}

// URL encode for safe use in URLs
static String urlEncode(const String& s) {
  String encoded = "";
  char c;
  for (unsigned int i = 0; i < s.length(); i++) {
    c = s.charAt(i);
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += c;
    } else if (c == ' ') {
      encoded += '+';
    } else {
      encoded += '%';
      char hex[3];
      sprintf(hex, "%02X", (unsigned char)c);
      encoded += hex;
    }
  }
  return encoded;
}

// Header/Footer with chunked response
static void sendHeader(const char* title, const char* activePage = "") {
  _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
  _server->send(200, "text/html", "");
  
  sendChunk("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  sendChunk("<meta name='viewport' content='width=device-width,initial-scale=1.0,maximum-scale=5.0'/>");
  sendChunk("<title>");
  sendChunk(title);
  sendChunk(" - RTKino</title>");
  sendChunk("<link rel='stylesheet' href='/css'>");
  sendChunk("</head><body>");
  
  // Header
  sendChunk("<div class='header'><h1>RTKino - ");
  sendChunk(title);
  sendChunk("</h1></div>");
  
  // Navigation
  sendChunk("<div class='nav'>");
  String active = String(activePage);
  
  String nav = "<a href='/'";
  if (active == "home") nav += " class='active'";
  nav += " title='Home'><span class='nav-icon'>⌂</span><span class='nav-text'>Home</span></a>";
  sendChunk(nav);
  
  nav = "<a href='/survey'";
  if (active == "survey") nav += " class='active'";
  nav += " title='Survey'><span class='nav-text'>Survey</span></a>";
  sendChunk(nav);

  nav = "<a href='/stakeout'";
  if (active == "stakeout") nav += " class='active'";
  nav += " title='Stakeout'><span class='nav-text'>Stakeout</span></a>";
  sendChunk(nav);
  
  nav = "<a href='/rover'";
  if (active == "rover") nav += " class='active'";
  nav += " title='Rover'><span class='nav-text'>Rover</span></a>";
  sendChunk(nav);
  
  nav = "<a href='/base-cfg'";
  if (active == "base") nav += " class='active'";
  nav += " title='Base'><span class='nav-text'>Base</span></a>";
  sendChunk(nav);
  
  nav = "<a href='/settings'";
  if (active == "settings") nav += " class='active'";
  nav += " title='Settings'><span class='nav-icon'>⚙</span><span class='nav-text'>Settings</span></a>";
  sendChunk(nav);
  
  sendChunk("</div>");
  sendChunk("<div class='container'>");
}

static void sendFooter() {
  sendChunk("</div></body></html>");
  _server->sendContent("");
}

// ========================================================================
// DATA STRUCTURES AND FILE PATHS
// ========================================================================

// Antenna models
struct AntennaRec {
  String name;
  float offset;  // ARP to L1/L2 phase center offset in meters
};
static String antennasFilePath(){ return "/config/antennas.txt"; }

// Basi salvate (updated with antenna height correction)
struct BaseRec { 
  String name; 
  double lat, lon;
  double altGround;    // H ellipsoidal of GROUND POINT (stored)
  uint16_t stid;
  float hARP;          // Antenna height to ARP (editable each time)
  int antennaIdx;      // Index into antenna list (-1 = manual/none)
  uint8_t rtcmType;    // 0=MSM7 4 constellations, 1=MSM4 3 constellations (default 0)
};
static String basesFilePath(){ return "/config/bases.txt"; }

// NTRIP IN list
struct NtripIn {
  String name, host, mount, user, pass; int port;
};
static String ntripInListPath(){ return "/config/ntrip_in_list.txt"; }

// NTRIP OUT list
struct NtripOut {
  String name, host, mount, pass; int port, tcpPort;
};
static String ntripOutListPath(){ return "/config/ntrip_out_list.txt"; }

// TCP OUT CLIENT list
struct TcpOutClient {
  String name, host; int port;
};
static String tcpOutClientListPath(){ return "/config/tcp_out_client_list.txt"; }

// LAN IN (TCP)
struct TcpIn {
  String name, host; int port;
};
static String tcpInListPath(){ return "/config/tcp_in_lista.txt"; }

// ========================================================================
// LOAD/SAVE FUNCTIONS FOR DATA FILES
// ========================================================================

static void loadBases(std::vector<BaseRec>& out){
  out.clear();
  String content = FlashConfig::readFile(basesFilePath().c_str());
  if (content.length() == 0) return;

  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (line.length() == 0 || line[0] == '#') continue;
    int p1=line.indexOf(';'); if (p1<0) continue;
    int p2=line.indexOf(';',p1+1); if (p2<0) continue;
    int p3=line.indexOf(';',p2+1); if (p3<0) continue;
    int p4=line.indexOf(';',p3+1); if (p4<0) continue;

    BaseRec b;
    b.name = line.substring(0,p1);
    b.lat  = line.substring(p1+1,p2).toDouble();
    b.lon  = line.substring(p2+1,p3).toDouble();

    int p5=line.indexOf(';',p4+1);
    int p6=line.indexOf(';',p5>0?p5+1:p4+1);
    int p7=line.indexOf(';',p6>0?p6+1:p5+1);

    if (p5>0 && p6>0 && p7>0) {
      b.altGround = line.substring(p3+1,p4).toDouble();
      b.stid = (uint16_t)line.substring(p4+1,p5).toInt();
      b.hARP = line.substring(p5+1,p6).toFloat();
      b.antennaIdx = line.substring(p6+1,p7).toInt();
      b.rtcmType = (uint8_t)line.substring(p7+1).toInt();
    } else if (p5>0 && p6>0) {
      b.altGround = line.substring(p3+1,p4).toDouble();
      b.stid = (uint16_t)line.substring(p4+1,p5).toInt();
      b.hARP = line.substring(p5+1,p6).toFloat();
      b.antennaIdx = line.substring(p6+1).toInt();
      b.rtcmType = 0;
    } else {
      b.altGround = line.substring(p3+1,p4).toDouble();
      b.stid = (uint16_t)line.substring(p4+1).toInt();
      b.hARP = 0.0f;
      b.antennaIdx = -1;
      b.rtcmType = 0;
    }
    out.push_back(b);
  }
  std::sort(out.begin(), out.end(), [](const BaseRec&a, const BaseRec&b){ return a.stid < b.stid; });
}

static bool saveBases(const std::vector<BaseRec>& v){
  String content = "# name;lat;lon;altGround;stid;hARP;antennaIdx;rtcmType\n";
  for (auto& b: v){
    content += clampSemi(b.name); content += ';';
    content += String(b.lat,8); content += ';';
    content += String(b.lon,8); content += ';';
    content += String(b.altGround,3); content += ';';
    content += String((int)b.stid); content += ';';
    content += String(b.hARP,3); content += ';';
    content += String(b.antennaIdx); content += ';';
    content += String((int)b.rtcmType); content += '\n';
  }
  return FlashConfig::writeFile(basesFilePath().c_str(), content);
}

// ========================================================================
// ANTENNA LOAD/SAVE FUNCTIONS
// ========================================================================

static void loadAntennas(std::vector<AntennaRec>& out){
  out.clear();
  String content = FlashConfig::readFile(antennasFilePath().c_str());
  if (content.length() == 0) return;

  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (line.length() == 0 || line[0] == '#') continue;
    int p1=line.indexOf(';'); if (p1<0) continue;
    AntennaRec a;
    a.name = line.substring(0,p1);
    a.offset = line.substring(p1+1).toFloat();
    out.push_back(a);
  }
}

static bool saveAntennas(const std::vector<AntennaRec>& v){
  String content = "# name;offset\n";
  for (auto& a: v){
    content += clampSemi(a.name); content += ';';
    content += String(a.offset,3); content += '\n';
  }
  return FlashConfig::writeFile(antennasFilePath().c_str(), content);
}

static int loadNtripInList(std::vector<NtripIn>& out, int& lastIdx){
  out.clear(); lastIdx=-1;
  String content = FlashConfig::readFile(ntripInListPath().c_str());
  if (content.length() == 0) return 0;

  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (line.length() == 0) continue;
    if (line.startsWith("#")){
      int p=line.indexOf("LAST=");
      if (p>=0){ lastIdx = line.substring(p+5).toInt(); }
      continue;
    }
    int p1=line.indexOf(';'); if (p1<0) continue;
    int p2=line.indexOf(';',p1+1); if (p2<0) continue;
    int p3=line.indexOf(';',p2+1); if (p3<0) continue;
    int p4=line.indexOf(';',p3+1); if (p4<0) continue;
    int p5=line.indexOf(';',p4+1); if (p5<0) continue;
    NtripIn n;
    n.name  = line.substring(0,p1);
    n.host  = line.substring(p1+1,p2);
    n.port  = line.substring(p2+1,p3).toInt();
    n.mount = line.substring(p3+1,p4);
    n.user  = line.substring(p4+1,p5);
    n.pass  = line.substring(p5+1);
    out.push_back(n);
  }
  return (int)out.size();
}

static bool saveNtripInList(const std::vector<NtripIn>& v, int lastIdx){
  String content = String("# LAST=") + lastIdx + "\n";
  content += "# name;host;port;mount;user;pass\n";
  for (auto& n: v){
    content += clampSemi(n.name); content += ';';
    content += n.host; content += ';'; content += n.port; content += ';';
    content += n.mount; content += ';'; content += n.user; content += ';';
    content += n.pass; content += '\n';
  }
  return FlashConfig::writeFile(ntripInListPath().c_str(), content);
}

static int loadNtripOutList(std::vector<NtripOut>& out, int& lastIdx){
  out.clear(); lastIdx=-1;
  String content = FlashConfig::readFile(ntripOutListPath().c_str());
  if (content.length() == 0) return 0;

  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (!line.length()) continue;
    if (line.startsWith("#")){
      int p=line.indexOf("LAST=");
      if (p>=0){ lastIdx = line.substring(p+5).toInt(); }
      continue;
    }
    int p1=line.indexOf(';'); if (p1<0) continue;
    int p2=line.indexOf(';',p1+1); if (p2<0) continue;
    int p3=line.indexOf(';',p2+1); if (p3<0) continue;
    int p4=line.indexOf(';',p3+1); if (p4<0) continue;
    int p5=line.indexOf(';',p4+1); if (p5<0) continue;
    NtripOut n;
    n.name = line.substring(0,p1);
    n.host = line.substring(p1+1,p2);
    n.port = line.substring(p2+1,p3).toInt();
    n.mount= line.substring(p3+1,p4);
    n.pass = line.substring(p4+1,p5);
    n.tcpPort = line.substring(p5+1).toInt();
    out.push_back(n);
  }
  return (int)out.size();
}

static bool saveNtripOutList(const std::vector<NtripOut>& v, int lastIdx){
  String content = String("# LAST=") + lastIdx + "\n";
  content += "# name;host;port;mount;pass;tcpport\n";
  for (auto& n: v){
    content += clampSemi(n.name); content += ';';
    content += n.host; content += ';'; content += n.port; content += ';';
    content += n.mount; content += ';'; content += n.pass; content += ';';
    content += n.tcpPort; content += '\n';
  }
  return FlashConfig::writeFile(ntripOutListPath().c_str(), content);
}

static int loadTcpInList(std::vector<TcpIn>& out, int& lastIdx){
  out.clear(); lastIdx=-1;
  String content = FlashConfig::readFile(tcpInListPath().c_str());
  if (content.length() == 0) return 0;

  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (!line.length()) continue;
    if (line.startsWith("#")){
      int p=line.indexOf("LAST=");
      if (p>=0){ lastIdx = line.substring(p+5).toInt(); }
      continue;
    }
    int p1=line.indexOf(';'); if (p1<0) continue;
    int p2=line.indexOf(';',p1+1); if (p2<0) continue;
    TcpIn t; t.name = line.substring(0,p1);
    t.host = line.substring(p1+1,p2);
    t.port = line.substring(p2+1).toInt();
    out.push_back(t);
  }
  return (int)out.size();
}

static bool saveTcpInList(const std::vector<TcpIn>& v, int lastIdx){
  String content = String("# LAST=") + lastIdx + "\n";
  content += "# name;host;port\n";
  for (auto& t: v){
    content += clampSemi(t.name); content += ';'; content += t.host;
    content += ';'; content += t.port; content += '\n';
  }
  return FlashConfig::writeFile(tcpInListPath().c_str(), content);
}

static int loadTcpOutClientList(std::vector<TcpOutClient>& out, int& lastIdx){
  out.clear(); lastIdx=-1;
  String content = FlashConfig::readFile(tcpOutClientListPath().c_str());
  if (content.length() == 0) return 0;

  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (!line.length()) continue;
    if (line.startsWith("#")){
      int p=line.indexOf("LAST=");
      if (p>=0){ lastIdx = line.substring(p+5).toInt(); }
      continue;
    }
    int p1=line.indexOf(';'); if (p1<0) continue;
    int p2=line.indexOf(';',p1+1); if (p2<0) continue;

    TcpOutClient t;
    t.name = line.substring(0,p1);
    t.host = line.substring(p1+1,p2);
    t.port = line.substring(p2+1).toInt();
    out.push_back(t);
  }
  return (int)out.size();
}

static bool saveTcpOutClientList(const std::vector<TcpOutClient>& v, int lastIdx){
  String content = String("# LAST=") + lastIdx + "\n";
  content += "# name;host;port\n";
  for (auto& t: v){
    content += clampSemi(t.name); content += ';';
    content += t.host; content += ';'; content += t.port; content += '\n';
  }
  return FlashConfig::writeFile(tcpOutClientListPath().c_str(), content);
}

// Returns the name of the selected NTRIP IN profile, or empty string if none
static String getSelectedNtripInName() {
    std::vector<NtripIn> list;
    int lastIdx = -1;
    loadNtripInList(list, lastIdx);
    
    if (lastIdx >= 0 && lastIdx < (int)list.size()) {
        return list[lastIdx].name;
    }
    return "";
}

// ========================================================================
// CSS ENDPOINT (with caching)
// ========================================================================

static void handleCSS() {
  _server->sendHeader("Cache-Control", "public, max-age=86400");
  _server->send_P(200, "text/css", CSS_CONTENT);
}

// ========================================================================
// API ENDPOINTS
// ========================================================================

static void handleApiStatus() {
  String json = "{\"wifi\":";
  json += (WiFi.status() == WL_CONNECTED) ? "true" : "false";
  json += ",\"ntrip\":";
  json += ntripEnabled ? "true" : "false";
  if (ntripEnabled) {
    String profileName = getSelectedNtripInName();
    if (profileName.length() > 0) {
      json += ",\"ntripProfile\":\"" + jsonEscape(profileName) + "\"";
    }
  }
  json += ",\"log\":";
  json += loggingActive ? "true" : "false";
  json += ",\"tcpIn\":";
  json += tcpInEnabled ? "true" : "false";
  json += ",\"bleRtcm\":";
  json += g_bleRtcmEnabled ? "true" : "false";
  json += ",\"bleRtcmConnected\":";
  json += (g_bleRtcmEnabled && g_bleRtcm.isConnected()) ? "true" : "false";
  json += ",\"bleRtcmStreaming\":";
  json += (g_bleRtcmEnabled && g_bleRtcm.isStreaming()) ? "true" : "false";
  json += "}";
  _server->send(200, "application/json", json);
}

static void handleApiPosition() {
    GNSSPosition pos;
    bool ok = getPosition(pos);
    
    uint32_t ageMs = ok ? (millis() - pos.lastUpdate) : 999999;
    
    // Fix quality string
    const char* fixStr = "No Fix";
    switch(pos.fixQuality) {
        case 1: fixStr = "GPS"; break;
        case 2: fixStr = "DGPS"; break;
        case 4: fixStr = "RTK Fixed"; break;
        case 5: fixStr = "RTK Float"; break;
        case 6: fixStr = "Estimated"; break;
    }
    
    String json = "{";
    json += "\"valid\":" + String(ok && ageMs < 5000 ? "true" : "false") + ",";
    json += "\"lat\":" + String(pos.lat, 8) + ",";
    json += "\"lon\":" + String(pos.lon, 8) + ",";
    json += "\"alt\":" + String(pos.alt, 2) + ",";
    json += "\"fix\":" + String(pos.fixQuality) + ",";
    json += "\"fixStr\":\"" + String(fixStr) + "\",";
    json += "\"sats\":" + String(pos.numSats) + ",";
    json += "\"numSV\":" + String(pos.numSV) + ",";
    json += "\"hdop\":" + String(pos.hdop, 1) + ",";
    json += "\"pdop\":" + String(pos.pdop, 2) + ",";
    json += "\"vdop\":" + String(pos.vdop, 2) + ",";
    json += "\"age\":" + String(pos.age, 1) + ",";
    json += "\"carrSoln\":" + String(pos.carrSoln) + ",";
    json += "\"dataAge\":" + String(ageMs);
    json += "}";
    
    _server->send(200, "application/json", json);
}

static void handleApiRtcm() {
    RtcmStats stats;
    bool ok = getRtcmStats(stats);
    
    uint32_t now = millis();
    uint32_t age = ok ? (now - stats.lastUpdate) : 999999;
    
    String json = "{";
    json += "\"valid\":" + String(ok ? "true" : "false") + ",";
    json += "\"totalMessages\":" + String(stats.totalMessages) + ",";
    json += "\"dataAge\":" + String(age) + ",";
    json += "\"types\":[";
    
    for (int i = 0; i < stats.numTypes; i++) {
        if (i > 0) json += ",";
        json += "{";
        json += "\"type\":" + String(stats.msgs[i].msgType) + ",";
        json += "\"count\":" + String(stats.msgs[i].count) + ",";
        json += "\"errors\":" + String(stats.msgs[i].crcErrors) + ",";
        json += "\"age\":" + String(now - stats.msgs[i].lastSeen);
        json += "}";
    }
    
    json += "]}";
    _server->send(200, "application/json", json);
}

static void handleApiAntennas() {
    std::vector<AntennaRec> antennas;
    loadAntennas(antennas);
    
    String json = "{\"antennas\":[";
    for (size_t i = 0; i < antennas.size(); i++) {
        if (i > 0) json += ",";
        json += "{";
        json += "\"idx\":" + String(i) + ",";
        json += "\"name\":\"" + jsonEscape(antennas[i].name) + "\",";
        json += "\"offset\":" + String(antennas[i].offset, 3);
        json += "}";
    }
    json += "]}";
    _server->send(200, "application/json", json);
}

// ========================================================================
// BASE STATION API ENDPOINTS
// ========================================================================

static void handleApiBasesIdx() {
    if (!_server->hasArg("idx")) {
        _server->send(400, "application/json", "{\"error\":\"idx parameter missing\"}");
        return;
    }
    
    int idx = _server->arg("idx").toInt();
    std::vector<BaseRec> bases;
    loadBases(bases);
    
    if (idx < 0 || idx >= (int)bases.size()) {
        _server->send(404, "application/json", "{\"error\":\"Base not found\"}");
        return;
    }
    
    const auto& b = bases[idx];
    String json = "{";
    json += "\"idx\":" + String(idx) + ",";
    json += "\"name\":\"" + jsonEscape(b.name) + "\",";
    json += "\"lat\":" + String(b.lat, 8) + ",";
    json += "\"lon\":" + String(b.lon, 8) + ",";
    json += "\"altGround\":" + String(b.altGround, 3) + ",";
    json += "\"stid\":" + String(b.stid) + ",";
    json += "\"hARP\":" + String(b.hARP, 3) + ",";
    json += "\"antennaIdx\":" + String(b.antennaIdx);
    json += "}";
    
    _server->send(200, "application/json", json);
}

static void handleApiZedTmode() {
    ZedTmodeState state;
    if (!getZedTmode(state)) {
        _server->send(500, "application/json", "{\"error\":\"Failed to read TMODE state\"}");
        return;
    }
    
    String modeStr = "Unknown";
    if (state.mode == 0) modeStr = "Disabled";
    else if (state.mode == 1) modeStr = "Survey-in";
    else if (state.mode == 2) modeStr = "Fixed";
    
    String json = "{";
    json += "\"valid\":" + String(state.valid ? "true" : "false") + ",";
    json += "\"mode\":" + String(state.mode) + ",";
    json += "\"modeStr\":\"" + modeStr + "\",";
    json += "\"posType\":" + String(state.posType) + ",";
    json += "\"lat\":" + String(state.lat, 8) + ",";
    json += "\"lon\":" + String(state.lon, 8) + ",";
    json += "\"height\":" + String(state.height, 3) + ",";
    json += "\"lastCheck\":" + String(state.lastCheck);
    json += "}";
    
    _server->send(200, "application/json", json);
}

// ========================================================================
// SURVEY API ENDPOINTS
// ========================================================================

static void handleSurveyStart() {
    if (!_server->hasArg("duration") || !_server->hasArg("height") || !_server->hasArg("arp")) {
        _server->send(400, "application/json", "{\"error\":\"Missing parameters\"}");
        return;
    }
    
    uint32_t duration = _server->arg("duration").toInt();
    float height = _server->arg("height").toFloat();
    float arp = _server->arg("arp").toFloat();
    
    if (duration < 30) {
        _server->send(400, "application/json", "{\"error\":\"Duration must be at least 30 seconds\"}");
        return;
    }
    
    startSurvey(duration, height, arp);
    _server->send(200, "application/json", "{\"status\":\"started\"}");
}

static void handleSurveyStop() {
    stopSurvey();
    _server->send(200, "application/json", "{\"status\":\"stopped\"}");
}

static void handleSurveyStatus() {
    SurveyResults res;
    bool ok = getSurveyResults(res);
    
    if (!ok) {
        _server->send(500, "application/json", "{\"error\":\"Failed to get survey status\"}");
        return;
    }
    
    String json = "{";
    json += "\"active\":" + String(res.active ? "true" : "false") + ",";
    json += "\"complete\":" + String(res.complete ? "true" : "false") + ",";
    json += "\"progress\":" + String(res.duration > 0 ? (res.elapsed * 100 / res.duration) : 0) + ",";
    json += "\"duration\":" + String(res.duration) + ",";
    json += "\"elapsed\":" + String(res.elapsed) + ",";
    json += "\"samples\":" + String(res.sampleCount) + ",";
    json += "\"lat\":" + String(res.latMean, 8) + ",";
    json += "\"lon\":" + String(res.lonMean, 8) + ",";
    json += "\"altARP\":" + String(res.altMean, 3) + ",";
    json += "\"altGround\":" + String(res.altGround, 3) + ",";
    json += "\"instrumentHeight\":" + String(res.instrumentHeight, 3) + ",";
    json += "\"arpOffset\":" + String(res.arpOffset, 3) + ",";
    json += "\"stdLat\":" + String(res.latStdDev, 9) + ",";
    json += "\"stdLon\":" + String(res.lonStdDev, 9) + ",";
    json += "\"stdAlt\":" + String(res.altStdDev, 3);
    json += "}";
    
    _server->send(200, "application/json", json);
}

static void handleSurveySave() {
    if (!_server->hasArg("name") || !_server->hasArg("stid")) {
        _server->send(400, "application/json", "{\"error\":\"Missing parameters\"}");
        return;
    }
    
    String name = _server->arg("name");
    int stid = _server->arg("stid").toInt();
    int antennaIdx = _server->hasArg("antennaIdx") ? _server->arg("antennaIdx").toInt() : -1;
    
    if (stid < 1 || stid >= 4096) {
        _server->send(400, "application/json", "{\"error\":\"Invalid station ID\"}");
        return;
    }
    
    // Get final survey results
    SurveyResults res;
    if (!getSurveyResults(res)) {
        _server->send(500, "application/json", "{\"error\":\"Failed to get survey results\"}");
        return;
    }
    
    if (res.sampleCount == 0) {
        _server->send(400, "application/json", "{\"error\":\"No samples collected\"}");
        return;
    }
    
    // Create new base station record
    BaseRec b;
    b.name = clampSemi(name);
    b.lat = res.latMean;
    b.lon = res.lonMean;
    b.altGround = res.altGround;  // Use ground-corrected altitude
    b.stid = (uint16_t)stid;
    b.hARP = res.instrumentHeight;  // Save the instrument height used
    b.antennaIdx = antennaIdx;      // Save antenna index
    
    // Load existing bases
    std::vector<BaseRec> v;
    loadBases(v);
    v.push_back(b);
    
    // Sort by STID
    std::sort(v.begin(), v.end(), [](const BaseRec&a, const BaseRec&b){return a.stid<b.stid;});
    
    // Save to file
    if (!saveBases(v)) {
        _server->send(500, "application/json", "{\"error\":\"Failed to save base station\"}");
        return;
    }
    
    _server->send(200, "application/json", "{\"status\":\"saved\"}");
}

// ========================================================================
// DASHBOARD PAGE (/)
// ========================================================================

static void handleRoot() {
  sendHeader("Dashboard", "dashboard");
  
  // JavaScript functions for inline controls
  sendChunk("<script>");
  sendChunk("function toggleNtrip(e){fetch('/ntrip/toggle?enable='+e).then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function startLog(){fetch('/log/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error starting log: '+(err.message||err));})}");
  sendChunk("function stopLog(){fetch('/log/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error stopping log: '+(err.message||err));})}");
  sendChunk("function startTcpIn(){fetch('/lanin/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error starting TCP-IN: '+(err.message||err));})}");
  sendChunk("function stopTcpIn(){fetch('/lanin/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error stopping TCP-IN: '+(err.message||err));})}");
  sendChunk("function bleRtcmStart(){fetch('/api/blertcm/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function bleRtcmStop(){fetch('/api/blertcm/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("</script>");
  
  // Status card
  sendChunk("<div class='card'><h2>Status</h2>");
  
  // WiFi status
  String wifiStat = "<div class='status-row'><span class='status-led ";
  if (WiFi.status() == WL_CONNECTED) {
    wifiStat += "led-on'></span><strong>WiFi:</strong> Connected to " + WiFi.SSID() + " (" + WiFi.localIP().toString() + ")";
  } else {
    wifiStat += "led-off'></span><strong>WiFi:</strong> Not connected";
  }
  wifiStat += "</div>";
  sendChunk(wifiStat);
  
  // BLE status
  String bleStat = "<div class='status-row'><span class='status-led ";
  if (g_bleEnabled && BLESerial::isConnected()) {
    bleStat += "led-on'></span><strong>BLE:</strong> Connected";
    String clientName = BLESerial::getClientName();
    if (clientName.length() > 0) {
      bleStat += " (" + htmlEscape(clientName) + ")";
    }
  } else {
    bleStat += "led-off'></span><strong>BLE:</strong> ";
    if (g_bleEnabled) {
      bleStat += "Advertising (waiting)";
    } else {
      bleStat += "Disabled";
    }
  }
  bleStat += "</div>";
  sendChunk(bleStat);
  
  // NTRIP status
  String ntripStat = "<div class='status-row'><span class='status-led ";
  ntripStat += ntripEnabled ? "led-on" : "led-off";
  ntripStat += "'></span><strong>NTRIP IN:</strong> ";
  if (ntripEnabled) {
    String profileName = getSelectedNtripInName();
    if (profileName.length() > 0) {
      ntripStat += "Active (" + htmlEscape(profileName) + ")";
    } else {
      ntripStat += "Active";
    }
  } else {
    ntripStat += "Inactive";
  }
  // Inline control buttons
  ntripStat += "<span style='float:right;'>";
  ntripStat += "<button onclick='toggleNtrip(1)' style='background-color:#2ecc71;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='Enable NTRIP' aria-label='Enable NTRIP'>✓</button>";
  ntripStat += "<button onclick='toggleNtrip(0)' style='background-color:#e74c3c;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='Disable NTRIP' aria-label='Disable NTRIP'>✕</button>";
  ntripStat += "</span>";
  ntripStat += "</div>";
  sendChunk(ntripStat);
  
  // TCP IN status
  String tcpInStat = "<div class='status-row'><span class='status-led ";
  tcpInStat += tcpInEnabled ? "led-on" : "led-off";
  tcpInStat += "'></span><strong>TCP IN:</strong> ";
  tcpInStat += tcpInEnabled ? "Active" : "Inactive";
  // Inline control buttons
  tcpInStat += "<span style='float:right;'>";
  tcpInStat += "<button onclick='startTcpIn()' style='background-color:#2ecc71;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='Start TCP IN' aria-label='Start TCP IN'>✓</button>";
  tcpInStat += "<button onclick='stopTcpIn()' style='background-color:#e74c3c;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='Stop TCP IN' aria-label='Stop TCP IN'>✕</button>";
  tcpInStat += "</span>";
  tcpInStat += "</div>";
  sendChunk(tcpInStat);
  
  // BLE RTCM IN status
  String bleRtcmStat = "<div class='status-row'><span class='status-led ";
  if (g_bleRtcmEnabled && g_bleRtcm.isConnected()) {
    bleRtcmStat += "led-on'></span><strong>BLE RTCM:</strong> ";
    bleRtcmStat += g_bleRtcm.isStreaming() ? "Streaming" : "Connected";
  } else {
    bleRtcmStat += g_bleRtcmEnabled ? "led-off" : "led-off";
    bleRtcmStat += "'></span><strong>BLE RTCM:</strong> ";
    if (g_bleRtcmEnabled) {
      bleRtcmStat += g_bleRtcm.isScanning() ? "Scanning..." : "Waiting";
    } else {
      bleRtcmStat += "Inactive";
    }
  }
  bleRtcmStat += "<span style='float:right;'>";
  bleRtcmStat += "<button onclick='bleRtcmStart()' style='background-color:#2ecc71;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='Start BLE RTCM'>&#x25B6;</button>";
  bleRtcmStat += "<button onclick='bleRtcmStop()' style='background-color:#e74c3c;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='Stop BLE RTCM'>&#x25A0;</button>";
  bleRtcmStat += "</span></div>";
  sendChunk(bleRtcmStat);

  // Logging status
  String logStat = "<div class='status-row'><span class='status-led ";
  logStat += loggingActive ? "led-on" : "led-off";
  logStat += "'></span><strong>RAW Log:</strong> ";
  logStat += loggingActive ? "Recording" : "Stopped";
  // Inline control buttons
  logStat += "<span style='float:right;'>";
  logStat += "<button onclick='startLog()' style='background-color:#2ecc71;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='Start Log' aria-label='Start Log'>✓</button>";
  logStat += "<button onclick='stopLog()' style='background-color:#e74c3c;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='Stop Log' aria-label='Stop Log'>✕</button>";
  logStat += "<button onclick='location.href=\"/logs\"' class='btn-secondary' style='padding:6px 12px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:0.9em;' title='View Log Files' aria-label='View Log Files'>📁 Logs</button>";
  logStat += "</span>";
  logStat += "</div>";
  sendChunk(logStat);
  
  // TCP Stream Mode status
  String streamStat = "<div class='status-row'><strong>TCP Stream:</strong> ";
  streamStat += String(getStreamModeName());
  streamStat += "</div>";
  sendChunk(streamStat);
  
  // NTP Time status
  // g_timeSource values: 0=TIME_SRC_NONE, 1=TIME_SRC_NTP, 2=TIME_SRC_TIMEUTC
  sendChunk("<div class='status-row'><strong>NTP Time:</strong> ");
  if (g_timeSource == 0) {
    sendChunk("<span style='color:#e74c3c'>Not synced⚠️</span>");
  } else if (g_timeSource == 1) {
    sendChunk("<span style='color:#2ecc71'>Synced (");
    sendChunk(g_ntpServer);
    sendChunk(")</span>");
  } else if (g_timeSource == 2) {
    sendChunk("<span style='color:#2ecc71'>GPS Time</span>");
  }
  sendChunk("<span onclick=\"syncNtp()\" style=\"cursor:pointer;margin-left:8px;\" title=\"Sync NTP Now\" aria-label=\"Sync NTP\">🔄</span>");
  sendChunk("</div>");
  
  // ZED-F9P TMODE status
  ZedTmodeState tmode;
  if (getZedTmode(tmode) && tmode.valid) {
    sendChunk("<div class='status-row'><strong>ZED-F9P Mode:</strong> ");
    if (tmode.mode == 0) {
      sendChunk("<span style='color:#3498db'>ROVER</span> (TMODE disabled)");
    } else if (tmode.mode == 1) {
      sendChunk("<span style='color:#f39c12'>SURVEY-IN</span>");
    } else if (tmode.mode == 2) {
      String baseInfo = "<span style='color:#2ecc71'>BASE (Fixed)</span><br>";
      baseInfo += "&nbsp;&nbsp;Lat: " + String(tmode.lat, 8) + "°<br>";
      baseInfo += "&nbsp;&nbsp;Lon: " + String(tmode.lon, 8) + "°<br>";
      baseInfo += "&nbsp;&nbsp;H: " + String(tmode.height, 3) + " m";
      sendChunk(baseInfo);
    }
    uint32_t age = (millis() - tmode.lastCheck) / 1000;
    sendChunk("<br><span style='font-size:0.85em;color:#95a5a6'>Last check: " + String(age) + "s ago</span>");
    sendChunk("<span onclick=\"refreshTmode()\" style=\"cursor:pointer;margin-left:8px;\" title=\"Refresh\" aria-label=\"Refresh TMODE status\">🔄</span>");
    // Add "Switch to Rover" button only in BASE mode
    if (tmode.mode == 2) {
      sendChunk("<button onclick=\"switchToRover()\" style=\"margin-left:8px;background-color:#f39c12;color:white;border:none;padding:4px 12px;border-radius:4px;cursor:pointer;\" title=\"Switch to Rover Mode\">Rover</button>");
    }
    sendChunk("</div>");
  } else {
    sendChunk("<div class='status-row'><strong>ZED-F9P Mode:</strong> ");
    sendChunk("<span style='color:#95a5a6'>Unknown</span>");
    sendChunk("<span onclick=\"refreshTmode()\" style=\"cursor:pointer;margin-left:8px;\" title=\"Refresh\" aria-label=\"Refresh TMODE status\">🔄</span>");
    sendChunk("</div>");
  }
  
  sendChunk("</div>");
  
  // Position Status Card
  sendChunk("<div class='card'>");
  sendChunk("<h2>📍 Position Status</h2>");
  sendChunk("<div id='pos-status'>");
  sendChunk("<p>Loading...</p>");
  sendChunk("</div>");
  sendChunk("</div>");
  
  // RTCM Stream Card
  sendChunk("<div class='card'>");
  sendChunk("<h2>📡 RTCM Stream</h2>");
  sendChunk("<div id='rtcm-status'>");
  sendChunk("<p>Loading...</p>");
  sendChunk("</div>");
  sendChunk("</div>");
  
  // TCP Stream Mode (Viewer) - moved from Settings
  sendChunk("<div class='card'><h2>TCP Stream Mode (Viewer)</h2>");
  sendChunk("<p><strong>Current mode:</strong> " + String(getStreamModeName()) + "</p>");
  sendChunk("<button onclick='setMode(\"nmea\")'>NMEA+UBX/RTCM</button> ");
  sendChunk("<button onclick='setMode(\"raw\")'>RAW (UBX)</button>");
  sendChunk("<script>function setMode(m){fetch('/stream?mode='+m).then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error changing mode: '+(err.message||err));})}</script>");
  sendChunk("</div>");
  
  // JavaScript for position auto-refresh
  sendChunk("<script>");
  sendChunk("function updatePos(){");
  sendChunk("fetch('/api/position').then(r=>r.json()).then(d=>{");
  sendChunk("let h='<table style=\"width:100%\">';");
  sendChunk("let color='🔴';");  // default single
  sendChunk("if(d.carrSoln==2||d.fix==4)color='🟢';");  // RTK Fixed
  sendChunk("else if(d.carrSoln==1||d.fix==5)color='🟡';");  // RTK Float
  sendChunk("else if(d.fix==2)color='🔵';");  // DGPS
  sendChunk("h+='<tr><td><b>Lat:</b></td><td>'+d.lat.toFixed(8)+'°</td>';");
  sendChunk("h+='<td><b>Fix:</b></td><td>'+d.fixStr+' '+color+'</td></tr>';");
  sendChunk("h+='<tr><td><b>Lon:</b></td><td>'+d.lon.toFixed(8)+'°</td>';");
  sendChunk("h+='<td><b>Sats:</b></td><td>'+d.numSV+'</td></tr>';");
  sendChunk("h+='<tr><td><b>Alt:</b></td><td>'+d.alt.toFixed(2)+' m</td>';");
  sendChunk("h+='<td><b>HDOP:</b></td><td>'+d.hdop.toFixed(1)+'</td></tr>';");
  sendChunk("h+='<tr><td><b>Age:</b></td><td>'+d.age.toFixed(1)+' s</td>';");
  sendChunk("h+='<td><b>VDOP:</b></td><td>'+d.vdop.toFixed(2)+'</td></tr>';");
  sendChunk("h+='<tr><td><b>Data:</b></td><td>'+(d.dataAge<2000?'Live 🟢':'Stale 🔴')+'</td>';");
  sendChunk("h+='<td><b>PDOP:</b></td><td>'+d.pdop.toFixed(2)+'</td></tr>';");
  sendChunk("h+='</table>';");
  sendChunk("document.getElementById('pos-status').innerHTML=h;");
  sendChunk("}).catch(e=>{document.getElementById('pos-status').innerHTML='<p>Error loading position</p>';});");
  sendChunk("}");
  sendChunk("updatePos();setInterval(updatePos,1000);");
  
  sendChunk("function rtcmName(t){");
  sendChunk("const n={1005:'Base Position',1006:'Base Position+H',1074:'GPS MSM4',1077:'GPS MSM7',");
  sendChunk("1084:'GLONASS MSM4',1087:'GLONASS MSM7',1094:'Galileo MSM4',1097:'Galileo MSM7',");
  sendChunk("1124:'BeiDou MSM4',1127:'BeiDou MSM7',1230:'GLONASS Bias',4072:'u-blox Proprietary'};");
  sendChunk("return n[t]||('Type '+t);}");
  
  sendChunk("function updateRtcm(){");
  sendChunk("fetch('/api/rtcm').then(r=>r.json()).then(d=>{");
  sendChunk("if(!d.valid||d.types.length==0){document.getElementById('rtcm-status').innerHTML='<p>No RTCM data</p>';return;}");
  sendChunk("let h='<table style=\"width:100%\"><tr><th>Type</th><th>Description</th><th>Count</th><th>Status</th></tr>';");
  sendChunk("d.types.sort((a,b)=>a.type-b.type);");
  sendChunk("for(let m of d.types){");
  sendChunk("let st=m.age<5000?'🟢':(m.age<15000?'🟡':'🔴');");
  sendChunk("let err=m.errors>0?' ('+m.errors+' err)':'';");
  sendChunk("h+='<tr><td>'+m.type+'</td><td>'+rtcmName(m.type)+'</td><td>'+m.count+err+'</td><td>'+st+'</td></tr>';");
  sendChunk("}");
  sendChunk("h+='</table>';");
  sendChunk("h+='<p style=\"margin-top:10px\">Total: '+d.totalMessages+' msgs | Last: '+(d.dataAge<2000?'Live 🟢':'Stale 🔴')+'</p>';");
  sendChunk("document.getElementById('rtcm-status').innerHTML=h;");
  sendChunk("}).catch(e=>{document.getElementById('rtcm-status').innerHTML='<p>Error loading RTCM data</p>';});");
  sendChunk("}");
  sendChunk("updateRtcm();setInterval(updateRtcm,2000);");
  
  sendChunk("function refreshTmode(){");
  sendChunk("fetch('/api/zed/tmode/refresh').then(r=>r.json()).then(d=>{location.reload();}).catch(e=>{alert('Refresh failed: '+(e.message||e));});");
  sendChunk("}");
  
  sendChunk("function syncNtp(){");
  sendChunk("fetch('/ntp/sync').then(r=>r.text()).then(t=>{location.reload();}).catch(e=>{alert('NTP Sync Error: '+(e.message||e));});");
  sendChunk("}");
  
  sendChunk("function switchToRover(){");
  sendChunk("if(confirm('Are you sure you want to switch to Rover mode? This will stop RTCM output and restart the ZED.')){");
  sendChunk("fetch('/api/switchToRover').then(r=>r.text()).then(t=>{alert(t);setTimeout(function(){location.reload();},1500);}).catch(e=>{alert('Error: '+(e.message||e));});");
  sendChunk("}}");
  
  sendChunk("</script>");
  
  sendFooter();
}

// ========================================================================
// ROVER PAGE (/rover) - NTRIP IN + LAN TCP-IN
// ========================================================================

static void handleRoverPage() {
  sendHeader("Rover Configuration", "rover");
  
  // NTRIP IN section
  std::vector<NtripIn> ntripList; int ntripLast=-1; 
  loadNtripInList(ntripList, ntripLast);
  
  sendChunk("<div class='card'><h2>NTRIP IN Profiles</h2>");
  
  // Active profile indicator
  sendChunk("<p><strong>Active Profile:</strong> ");
  if (ntripLast>=0 && ntripLast<(int)ntripList.size()) {
    sendChunk("<span class='badge'>" + htmlEscape(ntripList[ntripLast].name) + "</span>");
  } else {
    sendChunk("<em>None</em>");
  }
  sendChunk("</p>");
  
  // NTRIP control buttons
  sendChunk("<button onclick='toggleNtrip(1)' class='btn-success' aria-label='Enable NTRIP client'>Enable NTRIP</button> ");
  sendChunk("<button onclick='toggleNtrip(0)' class='btn-danger' aria-label='Disable NTRIP client'>Disable NTRIP</button>");
  sendChunk("<script>function toggleNtrip(e){fetch('/ntrip/toggle?enable='+e).then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}</script>");
  
  if (!ntripList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Host</th><th>Port</th><th>Mount</th><th>User</th><th>Actions</th></tr>");
    for (size_t i=0;i<ntripList.size();++i){
      const auto& n=ntripList[i];
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(n.name) + "</td>";
      row += "<td data-label='Host'>" + htmlEscape(n.host) + "</td>";
      row += "<td data-label='Port'>" + String(n.port) + "</td>";
      row += "<td data-label='Mount'>" + htmlEscape(n.mount) + "</td>";
      row += "<td data-label='User'>" + htmlEscape(n.user) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/ntrip/select?idx=" + String(i) + "' class='btn btn-small'>✓ Select</a> ";
      row += "<a href='/ntrip/edit?idx=" + String(i) + "' class='btn btn-small'>✏️ Edit</a> ";
      row += "<a href='/ntrip/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(n.name) + "?\")'>❌ Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No NTRIP profiles saved.</em></p>");
  }
  
  // Add form
  sendChunk("<h3>Add NTRIP Profile</h3>");
  sendChunk("<form method='POST' action='/ntrip/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Host:</label><input name='host' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='2101' required><br>");
  sendChunk("<label>Mountpoint:</label><input name='mount' required><br>");
  sendChunk("<label>User:</label><input name='user'><br>");
  sendChunk("<label>Password:</label><input name='pwd' type='password'><br>");
  sendChunk("<button type='submit'>Add Profile</button>");
  sendChunk("</form></div>");
  
  // TCP IN section
  std::vector<TcpIn> tcpList; int tcpLast=-1;
  loadTcpInList(tcpList, tcpLast);
  
  sendChunk("<div class='card'><h2>LAN TCP-IN Profiles</h2>");
  
  // Active profile indicator
  sendChunk("<p><strong>Active Profile:</strong> ");
  if (tcpLast>=0 && tcpLast<(int)tcpList.size()) {
    sendChunk("<span class='badge'>" + htmlEscape(tcpList[tcpLast].name) + "</span>");
  } else {
    sendChunk("<em>None</em>");
  }
  sendChunk("</p>");
  
  // Status and control
  String tcpStatus = "<p><strong>Status:</strong> ";
  tcpStatus += tcpInEnabled ? "<span class='badge' style='background:#2ecc71;color:white'>Active</span>" : "<span class='badge'>Inactive</span>";
  tcpStatus += "</p>";
  sendChunk(tcpStatus);
  
  if (!tcpList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Host</th><th>Port</th><th>Actions</th></tr>");
    for (size_t i=0;i<tcpList.size();++i){
      const auto& t=tcpList[i];
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(t.name) + "</td>";
      row += "<td data-label='Host'>" + htmlEscape(t.host) + "</td>";
      row += "<td data-label='Port'>" + String(t.port) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/lanin/select?idx=" + String(i) + "' class='btn btn-small'>✓ Select</a> ";
      row += "<a href='/lanin/edit?idx=" + String(i) + "' class='btn btn-small'>✏️ Edit</a> ";
      row += "<a href='/lanin/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(t.name) + "?\")'>❌ Delete</a> ";
      row += "<a href='/lanin/start?id=" + String(i) + "' class='btn btn-small btn-success'>▶️ Start</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No TCP-IN profiles saved.</em></p>");
  }
  
  // Add form
  sendChunk("<h3>Add TCP-IN Profile</h3>");
  sendChunk("<form method='POST' action='/lanin/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Host/IP:</label><input name='host' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='2103' required><br>");
  sendChunk("<button type='submit'>Add Profile</button>");
  sendChunk("</form></div>");
  
  sendFooter();
}

// ========================================================================
// BASE PAGE (/base-cfg) - TMODE + Saved Bases + OUT
// ========================================================================

static void handleBasePage() {
  sendHeader("Base Configuration", "base");
  
  // TMODE Configuration
  sendChunk("<div class='card'><h2>Base Mode (TMODE VALSET)</h2>");
  sendChunk("<p>Configure base with one click (all settings in RAM):</p>");
  sendChunk("<ul>");
  sendChunk("<li>Apply <strong>Station ID</strong> (DF003) before messages</li>");
  sendChunk("<li>TMODE: <strong>FIXED LLH (HP)</strong></li>");
  sendChunk("<li>UART2 TX: <strong>RTCM3 only</strong></li>");
  sendChunk("<li>MSGOUT: 1005/1230 @10s; MSM7 @1s or MSM4 @1s</li>");
  sendChunk("</ul>");
  
  sendChunk("<h3>Fixed Coordinates + Station ID</h3>");
  sendChunk("<form method='POST' action='/base/llh'>");
  sendChunk("<label>Latitude [deg]:</label><input name='lat' required><br>");
  sendChunk("<label>Longitude [deg]:</label><input name='lon' required><br>");
  sendChunk("<label>Altitude ellips. [m]:</label><input name='alt' required><br>");
  sendChunk("<label>Station ID [1..4095]:</label><input name='stid' type='number' value='1' style='width:150px'><br>");
  sendChunk("<label>RTCM Messages:</label><select name='rtcm_type' style='width:350px'>");
  sendChunk("<option value='0'>MSM7 - 4 constellations (GPS, GLO, GAL, BDS) @1Hz</option>");
  sendChunk("<option value='1'>MSM4 - 3 constellations (GPS, GLO, GAL) @1Hz</option>");
  sendChunk("</select><br>");
  sendChunk("<button type='submit'>Start Base</button>");
  sendChunk("</form></div>");
  
  // Saved Bases
  std::vector<BaseRec> bases;
  loadBases(bases);
  
  // Load antennas for display
  std::vector<AntennaRec> antennas;
  loadAntennas(antennas);
  
  sendChunk("<div class='card'><h2>Saved Base Stations</h2>");
  
  if (!bases.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>STID</th><th>Name</th><th>Lat</th><th>Lon</th><th>H ground [m]</th><th>H ARP [m]</th><th>Antenna</th><th>RTCM</th><th>Actions</th></tr>");
    for (size_t i=0;i<bases.size();++i){
      auto& b=bases[i];
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='STID'>" + String(b.stid) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(b.name) + "</td>";
      row += "<td data-label='Lat'>" + String(b.lat,8) + "</td>";
      row += "<td data-label='Lon'>" + String(b.lon,8) + "</td>";
      row += "<td data-label='H ground [m]'>" + String(b.altGround,3) + "</td>";
      row += "<td data-label='H ARP [m]'>" + String(b.hARP,3) + "</td>";
      
      // Display antenna name or "None"
      String antennaName = "None";
      if (b.antennaIdx >= 0 && b.antennaIdx < (int)antennas.size()) {
        antennaName = htmlEscape(antennas[b.antennaIdx].name);
      }
      row += "<td data-label='Antenna'>" + antennaName + "</td>";
      
      // Display RTCM type
      String rtcmType = (b.rtcmType == 0) ? "MSM7 4c" : "MSM4 3c";
      row += "<td data-label='RTCM'>" + rtcmType + "</td>";
      
      row += "<td data-label='Actions'>";
      row += "<button onclick='confirmStartBase(" + String(i) + ")' class='btn btn-small btn-success'>▶️ Start</button> ";
      row += "<a href='/bases/edit?idx=" + String(i) + "' class='btn btn-small'>✏️ Edit</a> ";
      row += "<a href='/bases/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(b.name) + "?\")'>❌ Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No saved base stations.</em></p>");
  }
  
  // Add form
  sendChunk("<h3>Add Base Station</h3>");
  sendChunk("<form method='POST' action='/bases/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Latitude [deg]:</label><input name='lat' required><br>");
  sendChunk("<label>Longitude [deg]:</label><input name='lon' required><br>");
  sendChunk("<label>H ground [m]:</label><input name='alt' required> (ellipsoidal)<br>");
  sendChunk("<label>Station ID [1..4095]:</label><input name='stid' type='number' value='1' style='width:150px'><br>");
  sendChunk("<h4>Antenna Setup</h4>");
  sendChunk("<label>H antenna ARP [m]:</label><input name='harp' type='number' step='0.001' value='0.000' style='width:150px'> (ground to ARP)<br>");
  sendChunk("<label>Antenna model:</label><select name='antenna_idx'>");
  sendChunk("<option value='-1'>None / Manual</option>");
  for (size_t i = 0; i < antennas.size(); i++) {
    sendChunk("<option value='" + String(i) + "'>" + htmlEscape(antennas[i].name) + " (" + String(antennas[i].offset, 3) + "m)</option>");
  }
  sendChunk("</select><br>");
  sendChunk("<label>RTCM Messages:</label><select name='rtcm_type' style='width:350px'>");
  sendChunk("<option value='0'>MSM7 - 4 constellations (GPS, GLO, GAL, BDS) @1Hz</option>");
  sendChunk("<option value='1'>MSM4 - 3 constellations (GPS, GLO, GAL) @1Hz</option>");
  sendChunk("</select><br>");
  sendChunk("<button type='submit'>Add Base</button>");
  sendChunk("</form></div>");
  
  // OUT Profiles
  std::vector<NtripOut> outList; int outLast=-1;
  loadNtripOutList(outList, outLast);
  
  sendChunk("<div class='card'><h2>RTCM Output Profiles (NTRIP/TCP)</h2>");
  
  // Active profile indicator
  sendChunk("<p><strong>Active Profile:</strong> ");
  if (outLast>=0 && outLast<(int)outList.size()) {
    sendChunk("<span class='badge'>" + htmlEscape(outList[outLast].name) + "</span>");
  } else {
    sendChunk("<em>None</em>");
  }
  sendChunk("</p>");
  
  // Control buttons
  sendChunk("<button onclick='startOut()' class='btn-success' aria-label='Start RTCM output (NTRIP/TCP)'>Start OUT (active profile)</button> ");
  sendChunk("<button onclick='stopOut()' class='btn-danger' aria-label='Stop RTCM output'>Stop OUT</button>");
  sendChunk("<script>");
  sendChunk("function startOut(){fetch('/baseout/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error starting output: '+(err.message||err));})}");
  sendChunk("function stopOut(){fetch('/baseout/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error stopping output: '+(err.message||err));})}");
  sendChunk("</script>");
  
  if (!outList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Host</th><th>Port</th><th>Mount</th><th>TCP Port</th><th>Actions</th></tr>");
    for (size_t i=0;i<outList.size();++i){
      const auto& n=outList[i];
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(n.name) + "</td>";
      row += "<td data-label='Host'>" + htmlEscape(n.host) + "</td>";
      row += "<td data-label='Port'>" + String(n.port) + "</td>";
      row += "<td data-label='Mount'>" + htmlEscape(n.mount) + "</td>";
      row += "<td data-label='TCP Port'>" + String(n.tcpPort) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/baseout/select?idx=" + String(i) + "' class='btn btn-small'>✓ Select</a> ";
      row += "<a href='/baseout/edit?idx=" + String(i) + "' class='btn btn-small'>✏️ Edit</a> ";
      row += "<a href='/baseout/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(n.name) + "?\")'>❌ Delete</a> ";
      row += "<a href='/baseout/start?id=" + String(i) + "' class='btn btn-small btn-success'>▶️ Start</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No output profiles saved.</em></p>");
  }
  
  // Add form
  sendChunk("<h3>Add Output Profile</h3>");
  sendChunk("<form method='POST' action='/baseout/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Host:</label><input name='host' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='2101' required><br>");
  sendChunk("<label>Mountpoint:</label><input name='mount' required><br>");
  sendChunk("<label>Password:</label><input name='pass' type='password'><br>");
  sendChunk("<label>TCP Server Port:</label><input name='tcp' type='number' value='2102' required><br>");
  sendChunk("<button type='submit'>Add Profile</button>");
  sendChunk("</form></div>");

// ============================================================================
// TCP OUT CLIENT - UI Section for handleBasePage()
// Da inserire in handleBasePage() dopo la sezione NTRIP OUT (dopo riga 1561)
// PRIMA della sezione Survey Base Position
// ============================================================================

  // ===== TCP OUT CLIENT PROFILES =====
  std::vector<TcpOutClient> tcpClientList; int tcpClientLast=-1;
  loadTcpOutClientList(tcpClientList, tcpClientLast);
  
  sendChunk("<div class='card'><h2>TCP OUT Client Profiles</h2>");
  sendChunk("<p>Connect to external TCP servers and stream RTCM data.</p>");
  
  // Active profile indicator
  sendChunk("<p><strong>Active Profile:</strong> ");
  if (tcpClientLast>=0 && tcpClientLast<(int)tcpClientList.size()) {
    sendChunk("<span class='badge'>" + htmlEscape(tcpClientList[tcpClientLast].name) + "</span>");
  } else {
    sendChunk("<em>None</em>");
  }
  sendChunk("</p>");
  
  // Control buttons
  sendChunk("<button onclick='startTcpClient()' class='btn-success'>Start TCP Client</button> ");
  sendChunk("<button onclick='stopTcpClient()' class='btn-danger'>Stop TCP Client</button>");
  sendChunk("<script>");
  sendChunk("function startTcpClient(){fetch('/tcpclient/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+err);})}");
  sendChunk("function stopTcpClient(){fetch('/tcpclient/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+err);})}");
  sendChunk("</script>");
  
  if (!tcpClientList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Host</th><th>Port</th><th>Actions</th></tr>");
    for (size_t i=0;i<tcpClientList.size();++i){
      const auto& t=tcpClientList[i];
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(t.name) + "</td>";
      row += "<td data-label='Host'>" + htmlEscape(t.host) + "</td>";
      row += "<td data-label='Port'>" + String(t.port) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/tcpclient/select?idx=" + String(i) + "' class='btn btn-small'>✓ Select</a> ";
      row += "<a href='/tcpclient/edit?idx=" + String(i) + "' class='btn btn-small'>✏️ Edit</a> ";
      row += "<a href='/tcpclient/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(t.name) + "?\")'>❌ Delete</a> ";
      row += "<a href='/tcpclient/start?id=" + String(i) + "' class='btn btn-small btn-success'>▶️ Start</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No TCP client profiles saved.</em></p>");
  }
  
  // Add form
  sendChunk("<h3>Add TCP Client Profile</h3>");
  sendChunk("<form method='POST' action='/tcpclient/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Host:</label><input name='host' required placeholder='192.168.1.100 or rtk.server.com'><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='5000' required><br>");
  sendChunk("<button type='submit'>Add Profile</button>");
  sendChunk("</form></div>");


  // ===== BLE RTCM OUTPUT (to rtcm-lora via BLE) =====
  sendChunk("<div class='card'><h2>&#x1F4E1; BLE RTCM Output</h2>");
  sendChunk("<p>Send RTCM corrections to rtcm-lora Base via Bluetooth LE (replaces TCP/WiFi connection).</p>");

  // Status
  sendChunk("<div class='status-row'><span class='status-led ");
  if (g_bleRtcmEnabled && g_bleRtcm.isConnected()) {
    sendChunk("led-on'></span><strong>BLE RTCM OUT:</strong> Connected");
    sendChunk(" (" + htmlEscape(g_bleRtcm.connectedName()) + ")");
    char txBuf[48]; snprintf(txBuf, sizeof(txBuf), " &mdash; TX: %lu B", (unsigned long)g_bleRtcm.getTxBytes());
    sendChunk(txBuf);
  } else if (g_bleRtcmEnabled) {
    sendChunk("led-off'></span><strong>BLE RTCM OUT:</strong> ");
    sendChunk(g_bleRtcm.isScanning() ? "Scanning..." : "Waiting");
  } else {
    sendChunk("led-off'></span><strong>BLE RTCM OUT:</strong> Disabled");
  }
  sendChunk("</div>");

  // Control buttons
  sendChunk("<div style='margin-top:10px'>");
  sendChunk("<button onclick='startBleOut()' class='btn-success'>Start BLE OUT</button> ");
  sendChunk("<button onclick='stopBleOut()' class='btn-danger'>Stop BLE OUT</button>");
  sendChunk("</div>");
  sendChunk("<script>");
  sendChunk("function startBleOut(){fetch('/api/blertcm/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+err);})}");
  sendChunk("function stopBleOut(){fetch('/api/blertcm/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+err);})}");
  sendChunk("</script>");

  sendChunk("<div style='margin-top:12px; padding:10px; background:#d1ecf1; border-left:4px solid #17a2b8; border-radius:4px; font-size:0.9em;'>");
  sendChunk("Configure the rtcm-lora device name and PIN in <a href='/settings'>Settings &rarr; BLE RTCM Input</a>.<br>");
  sendChunk("In base mode, RTCM data is sent to rtcm-lora via BLE NUS instead of TCP.");
  sendChunk("</div>");
  sendChunk("</div>");


  // Load antennas for dropdown
  std::vector<AntennaRec> surveyAntennas;
  loadAntennas(surveyAntennas);
  
  sendChunk("<div class='card'><h2>📍 Survey Base Position</h2>");
  sendChunk("<p>Record averaged GNSS positions over time and save as a new base station.</p>");
  
  // Configuration form
  sendChunk("<h3>Configuration</h3>");
  sendChunk("<form id='surveyForm' onsubmit='startSurvey(event)'>");
  sendChunk("<label>Duration:</label>");
  sendChunk("<select id='duration' required>");
  sendChunk("<option value='30'>30 seconds</option>");
  sendChunk("<option value='60' selected>60 seconds</option>");
  sendChunk("<option value='120'>120 seconds</option>");
  sendChunk("<option value='180'>180 seconds</option>");
  sendChunk("<option value='300'>300 seconds</option>");
  sendChunk("</select><br>");
  
  sendChunk("<label>H antenna ARP [m]:</label>");
  sendChunk("<input id='instrHeight' type='number' step='0.001' value='1.500' required> (ground to ARP)<br>");
  
  sendChunk("<label>Antenna model:</label>");
  sendChunk("<select id='antennaIdx' onchange='updateArpFromAntenna()'>");
  sendChunk("<option value='-1' data-offset='0.000'>None / Manual (0.000m)</option>");
  for (size_t i = 0; i < surveyAntennas.size(); i++) {
    sendChunk("<option value='" + String(i) + "' data-offset='" + String(surveyAntennas[i].offset, 3) + "'>" + 
              htmlEscape(surveyAntennas[i].name) + " (" + String(surveyAntennas[i].offset, 3) + "m)</option>");
  }
  sendChunk("</select><br>");
  
  sendChunk("<label>ARP Offset [m]:</label>");
  sendChunk("<input id='arpOffset' type='number' step='0.001' value='0.000' required readonly> (auto from antenna)<br>");
  
  sendChunk("<button type='submit' class='btn-success' id='startBtn'>🟢 Start Survey</button> ");
  sendChunk("<button type='button' class='btn-danger' id='stopBtn' onclick='stopSurvey()' style='display:none'>🔴 Stop Survey</button>");
  sendChunk("</form>");
  
  // Progress display
  sendChunk("<div id='surveyProgress' style='display:none; margin-top:20px'>");
  sendChunk("<h3>Progress</h3>");
  sendChunk("<div style='background:#ecf0f1; border-radius:4px; height:30px; overflow:hidden; margin:10px 0'>");
  sendChunk("<div id='progressBar' style='background:#3498db; height:100%; width:0%; transition:width 0.5s'></div>");
  sendChunk("</div>");
  sendChunk("<p><strong>Time:</strong> <span id='timeProgress'>0/0 s</span> (<span id='progressPct'>0</span>%)</p>");
  sendChunk("<p><strong>Samples:</strong> <span id='sampleCount'>0</span></p>");
  sendChunk("<h3>Current Average:</h3>");
  sendChunk("<table style='width:100%'>");
  sendChunk("<tr><td><b>Lat:</b></td><td><span id='currLat'>-</span>°</td><td><b>σ:</b></td><td><span id='stdLat'>-</span>°</td></tr>");
  sendChunk("<tr><td><b>Lon:</b></td><td><span id='currLon'>-</span>°</td><td><b>σ:</b></td><td><span id='stdLon'>-</span>°</td></tr>");
  sendChunk("<tr><td><b>H (ARP):</b></td><td><span id='currAltARP'>-</span> m</td><td><b>σ:</b></td><td><span id='stdAlt'>-</span> m</td></tr>");
  sendChunk("<tr><td><b>H (Ground):</b></td><td colspan='3'><span id='currAltGround'>-</span> m</td></tr>");
  sendChunk("</table>");
  sendChunk("</div>");
  
  // Results display
  sendChunk("<div id='surveyResults' style='display:none; margin-top:20px'>");
  sendChunk("<h3>✅ Survey Complete!</h3>");
  sendChunk("<h4>Final Position (ground point):</h4>");
  sendChunk("<table style='width:100%'>");
  sendChunk("<tr><td><b>Latitude:</b></td><td><span id='finalLat'>-</span>°</td></tr>");
  sendChunk("<tr><td><b>Longitude:</b></td><td><span id='finalLon'>-</span>°</td></tr>");
  sendChunk("<tr><td><b>H (ellips):</b></td><td><span id='finalAlt'>-</span> m</td></tr>");
  sendChunk("<tr><td><b>Std Dev:</b></td><td><span id='finalStd'>-</span> m (horizontal)</td></tr>");
  sendChunk("</table>");
  
  sendChunk("<h4>Save as Base Station:</h4>");
  sendChunk("<form onsubmit='saveSurvey(event)'>");
  sendChunk("<label>Name:</label><input id='saveName' type='text' placeholder='e.g. Benchmark Via Roma' required><br>");
  sendChunk("<label>Station ID:</label><input id='saveStid' type='number' min='1' max='4095' value='1' required><br>");
  sendChunk("<button type='submit' class='btn-success'>💾 Save Base Station</button>");
  sendChunk("</form>");
  sendChunk("</div>");
  
  // JavaScript for survey control and polling
  sendChunk("<script>");
  sendChunk("let surveyPollInterval = null;");
  
  sendChunk("function updateArpFromAntenna() {");
  sendChunk("  const select = document.getElementById('antennaIdx');");
  sendChunk("  const option = select.options[select.selectedIndex];");
  sendChunk("  const offset = option.getAttribute('data-offset');");
  sendChunk("  document.getElementById('arpOffset').value = offset;");
  sendChunk("}");
  
  sendChunk("function startSurvey(e) {");
  sendChunk("  e.preventDefault();");
  sendChunk("  const duration = document.getElementById('duration').value;");
  sendChunk("  const height = document.getElementById('instrHeight').value;");
  sendChunk("  const arp = document.getElementById('arpOffset').value;");
  sendChunk("  fetch('/api/survey/start?duration='+duration+'&height='+height+'&arp='+arp)");
  sendChunk("  .then(r => r.json())");
  sendChunk("  .then(d => {");
  sendChunk("    if (d.status === 'started') {");
  sendChunk("      document.getElementById('startBtn').style.display = 'none';");
  sendChunk("      document.getElementById('stopBtn').style.display = 'inline-block';");
  sendChunk("      document.getElementById('surveyProgress').style.display = 'block';");
  sendChunk("      document.getElementById('surveyResults').style.display = 'none';");
  sendChunk("      surveyPollInterval = setInterval(updateSurvey, 1000);");
  sendChunk("    } else { alert('Failed to start survey'); }");
  sendChunk("  })");
  sendChunk("  .catch(e => alert('Error: ' + e));");
  sendChunk("}");
  
  sendChunk("function stopSurvey() {");
  sendChunk("  fetch('/api/survey/stop')");
  sendChunk("  .then(r => r.json())");
  sendChunk("  .then(d => {");
  sendChunk("    if (surveyPollInterval) { clearInterval(surveyPollInterval); surveyPollInterval = null; }");
  sendChunk("    document.getElementById('startBtn').style.display = 'inline-block';");
  sendChunk("    document.getElementById('stopBtn').style.display = 'none';");
  sendChunk("    document.getElementById('surveyProgress').style.display = 'none';");
  sendChunk("    document.getElementById('surveyResults').style.display = 'none';");
  sendChunk("  });");
  sendChunk("}");
  
  sendChunk("function updateSurvey() {");
  sendChunk("  fetch('/api/survey/status')");
  sendChunk("  .then(r => r.json())");
  sendChunk("  .then(d => {");
  sendChunk("    if (!d.active) {");
  sendChunk("      if (surveyPollInterval) { clearInterval(surveyPollInterval); surveyPollInterval = null; }");
  sendChunk("      document.getElementById('startBtn').style.display = 'inline-block';");
  sendChunk("      document.getElementById('stopBtn').style.display = 'none';");
  sendChunk("      return;");
  sendChunk("    }");
  sendChunk("    document.getElementById('progressBar').style.width = d.progress + '%';");
  sendChunk("    document.getElementById('timeProgress').textContent = d.elapsed + '/' + d.duration + ' s';");
  sendChunk("    document.getElementById('progressPct').textContent = d.progress;");
  sendChunk("    document.getElementById('sampleCount').textContent = d.samples;");
  sendChunk("    if (d.samples > 0) {");
  sendChunk("      document.getElementById('currLat').textContent = d.lat.toFixed(8);");
  sendChunk("      document.getElementById('currLon').textContent = d.lon.toFixed(8);");
  sendChunk("      document.getElementById('currAltARP').textContent = d.altARP.toFixed(3);");
  sendChunk("      document.getElementById('currAltGround').textContent = d.altGround.toFixed(3);");
  sendChunk("      document.getElementById('stdLat').textContent = d.stdLat.toFixed(9);");
  sendChunk("      document.getElementById('stdLon').textContent = d.stdLon.toFixed(9);");
  sendChunk("      document.getElementById('stdAlt').textContent = d.stdAlt.toFixed(3);");
  sendChunk("    }");
  sendChunk("    if (d.complete) {");
  sendChunk("      if (surveyPollInterval) { clearInterval(surveyPollInterval); surveyPollInterval = null; }");
  sendChunk("      document.getElementById('startBtn').style.display = 'inline-block';");
  sendChunk("      document.getElementById('stopBtn').style.display = 'none';");
  sendChunk("      document.getElementById('surveyProgress').style.display = 'none';");
  sendChunk("      document.getElementById('surveyResults').style.display = 'block';");
  sendChunk("      document.getElementById('finalLat').textContent = d.lat.toFixed(8);");
  sendChunk("      document.getElementById('finalLon').textContent = d.lon.toFixed(8);");
  sendChunk("      document.getElementById('finalAlt').textContent = d.altGround.toFixed(3);");
  sendChunk("      const horizStd = Math.sqrt(d.stdLat*d.stdLat + d.stdLon*d.stdLon);");
  sendChunk("      document.getElementById('finalStd').textContent = horizStd.toFixed(9);");
  sendChunk("    }");
  sendChunk("  })");
  sendChunk("  .catch(e => console.error('Survey update error:', e));");
  sendChunk("}");
  
  sendChunk("function saveSurvey(e) {");
  sendChunk("  e.preventDefault();");
  sendChunk("  const name = document.getElementById('saveName').value;");
  sendChunk("  const stid = document.getElementById('saveStid').value;");
  sendChunk("  const antennaIdx = document.getElementById('antennaIdx').value;");
  sendChunk("  fetch('/api/survey/save', {");
  sendChunk("    method: 'POST',");
  sendChunk("    headers: {'Content-Type': 'application/x-www-form-urlencoded'},");
  sendChunk("    body: 'name=' + encodeURIComponent(name) + '&stid=' + stid + '&antennaIdx=' + antennaIdx");
  sendChunk("  })");
  sendChunk("  .then(r => r.json())");
  sendChunk("  .then(d => {");
  sendChunk("    if (d.status === 'saved') {");
  sendChunk("      alert('Base station saved successfully!');");
  sendChunk("      location.reload();");
  sendChunk("    } else if (d.error) {");
  sendChunk("      alert('Error: ' + d.error);");
  sendChunk("    }");
  sendChunk("  })");
  sendChunk("  .catch(e => alert('Error saving: ' + e));");
  sendChunk("}");
  
  // ===== Base Start Confirmation Modal =====
  sendChunk("function confirmStartBase(idx) {");
  sendChunk("  Promise.all([");
  sendChunk("    fetch('/api/bases?idx=' + idx).then(r => r.json()),");
  sendChunk("    fetch('/api/antennas').then(r => r.json())");
  sendChunk("  ]).then(([base, antData]) => {");
  sendChunk("    showStartBaseModal(base, antData.antennas, idx);");
  sendChunk("  }).catch(e => alert('Error loading base data: ' + e));");
  sendChunk("}");
  
  sendChunk("function showStartBaseModal(base, antennas, idx) {");
  sendChunk("  let modal = document.createElement('div');");
  sendChunk("  modal.id = 'startBaseModal';");
  sendChunk("  modal.style.cssText = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.5);display:flex;align-items:center;justify-content:center;z-index:1000;';");
  
  sendChunk("  let content = `");
  sendChunk("  <div style='background:white;padding:24px;border-radius:8px;max-width:500px;width:90%;max-height:90vh;overflow:auto;'>");
  sendChunk("    <h2 style='margin-bottom:16px;'>⚠️ Confirm Base Station Start</h2>");
  sendChunk("    <p><strong>Base:</strong> ${base.name}</p>");
  sendChunk("    <h4>Ground coordinates:</h4>");
  sendChunk("    <p>Lat: ${base.lat.toFixed(8)}°<br>");
  sendChunk("       Lon: ${base.lon.toFixed(8)}°<br>");
  sendChunk("       H ground: ${base.altGround.toFixed(3)} m</p>");
  sendChunk("    <hr style='margin:16px 0;'>");
  sendChunk("    <label>Antenna:</label>");
  sendChunk("    <select id='modalAntenna' style='width:100%;padding:8px;margin:4px 0 12px;'>");
  sendChunk("      ${antennas.map((a,i) => `<option value='${i}' ${i==base.antennaIdx?'selected':''}>${a.name} (${a.offset.toFixed(3)}m)</option>`).join('')}");
  sendChunk("      <option value='-1' ${base.antennaIdx==-1?'selected':''}>None (0.000m)</option>");
  sendChunk("    </select>");
  sendChunk("    <label>H antenna ARP (m):</label>");
  sendChunk("    <input type='number' id='modalHarp' value='${base.hARP.toFixed(3)}' step='0.001' style='width:100%;padding:8px;margin:4px 0 12px;'>");
  sendChunk("    <hr style='margin:16px 0;'>");
  sendChunk("    <div id='calcDisplay' style='background:#f5f5f5;padding:12px;border-radius:4px;'>");
  sendChunk("      <strong>📐 H to send to ZED-F9P:</strong><br>");
  sendChunk("      <span id='calcFormula'></span>");
  sendChunk("    </div>");
  sendChunk("    <div style='margin-top:20px;display:flex;gap:12px;justify-content:flex-end;'>");
  sendChunk("      <button onclick='closeModal()' style='padding:10px 20px;background:#e74c3c;color:white;border:none;border-radius:4px;cursor:pointer;'>❌ Cancel</button>");
  sendChunk("      <button onclick='doStartBase(${idx})' style='padding:10px 20px;background:#2ecc71;color:white;border:none;border-radius:4px;cursor:pointer;'>✅ Start Base</button>");
  sendChunk("    </div>");
  sendChunk("  </div>`;");
  
  sendChunk("  modal.innerHTML = content;");
  sendChunk("  document.body.appendChild(modal);");
  
  sendChunk("  window.modalBase = base;");
  sendChunk("  window.modalAntennas = antennas;");
  
  sendChunk("  document.getElementById('modalAntenna').onchange = updateCalc;");
  sendChunk("  document.getElementById('modalHarp').oninput = updateCalc;");
  sendChunk("  updateCalc();");
  sendChunk("}");
  
  sendChunk("function updateCalc() {");
  sendChunk("  let hGround = window.modalBase.altGround;");
  sendChunk("  let hArp = parseFloat(document.getElementById('modalHarp').value) || 0;");
  sendChunk("  let antIdx = parseInt(document.getElementById('modalAntenna').value);");
  sendChunk("  let offset = (antIdx >= 0 && window.modalAntennas[antIdx]) ? window.modalAntennas[antIdx].offset : 0;");
  sendChunk("  let hTotal = hGround + hArp + offset;");
  
  sendChunk("  document.getElementById('calcFormula').innerHTML = ");
  sendChunk("    `${hGround.toFixed(3)} + ${hArp.toFixed(3)} + ${offset.toFixed(3)} = <strong>${hTotal.toFixed(3)} m</strong>`;");
  sendChunk("}");
  
  sendChunk("function closeModal() {");
  sendChunk("  let modal = document.getElementById('startBaseModal');");
  sendChunk("  if (modal) modal.remove();");
  sendChunk("}");
  
  sendChunk("function doStartBase(idx) {");
  sendChunk("  let hArp = parseFloat(document.getElementById('modalHarp').value) || 0;");
  sendChunk("  let antIdx = parseInt(document.getElementById('modalAntenna').value);");
  
  sendChunk("  fetch('/bases/start-confirm', {");
  sendChunk("    method: 'POST',");
  sendChunk("    headers: {'Content-Type': 'application/x-www-form-urlencoded'},");
  sendChunk("    body: `idx=${encodeURIComponent(idx)}&harp=${encodeURIComponent(hArp)}&antenna=${encodeURIComponent(antIdx)}`");
  sendChunk("  })");
  sendChunk("  .then(r => r.text())");
  sendChunk("  .then(msg => {");
  sendChunk("    closeModal();");
  sendChunk("    alert(msg);");
  sendChunk("    location.reload();");
  sendChunk("  })");
  sendChunk("  .catch(e => {");
  sendChunk("    closeModal();");
  sendChunk("    alert('Error: ' + e);");
  sendChunk("  });");
  sendChunk("}");
  
  sendChunk("</script>");
  sendChunk("</div>");
  
  sendFooter();
}

// ========================================================================
// Bluetooth Card Rendering Helper
// ========================================================================

static void renderBluetoothCard() {
  sendChunk("<div class='card'><h2>🔵 Bluetooth</h2>");
  sendChunk("<form method='POST' action='/settings/ble'>");
  
  // BLE Enable Toggle
  sendChunk("<div class='form-row'>");
  sendChunk("<label>BLE Enable:</label>");
  sendChunk("<select name='ble_enable'>");
  if (g_bleEnabled) {
    sendChunk("<option value='1' selected>ON</option>");
    sendChunk("<option value='0'>OFF</option>");
  } else {
    sendChunk("<option value='1'>ON</option>");
    sendChunk("<option value='0' selected>OFF</option>");
  }
  sendChunk("</select>");
  sendChunk("</div>");
  
  // Device Name Input
  sendChunk("<div class='form-row' style='margin-top:10px'>");
  sendChunk("<label>Device Name:</label>");
  sendChunk("<input type='text' name='ble_name' value='" + htmlEscape(String(g_bleDeviceName)) + "' maxlength='20' placeholder='RTKino' style='width:220px' />");
  sendChunk("<br><small style='color:#666'>Max 20 characters (a-z A-Z 0-9 _ -)</small>");
  sendChunk("</div>");
  
  // Pairing PIN Input
  sendChunk("<div class='form-row' style='margin-top:10px'>");
  sendChunk("<label>Pairing PIN:</label>");
  char pinStr[8];
  snprintf(pinStr, sizeof(pinStr), "%06u", g_blePasskey);
  sendChunk("<input type='text' name='ble_pin' value='");
  sendChunk(pinStr);
  sendChunk("' maxlength='6' pattern='[0-9]{6}' placeholder='123456' style='width:150px' />");
  sendChunk("<br><small style='color:#666'>6-digit PIN (000000-999999)</small>");
  sendChunk("</div>");
  
  sendChunk("<button type='submit' class='btn btn-primary' style='margin-top:15px'>Apply Bluetooth Settings</button>");
  sendChunk("</form>");
  
  // Security Warning Box
  sendChunk("<div style='margin-top:15px; padding:12px; background:#fff3cd; border-left:4px solid #ffc107; border-radius:4px;'>");
  sendChunk("<strong>🔒 Security Note:</strong><br>");
  sendChunk("• When connecting from app, enter the PIN above when prompted<br>");
  sendChunk("• After first pairing, device remembers (no PIN needed)<br>");
  sendChunk("• Change PIN to increase security<br>");
  sendChunk("• BLE is enabled by default on every boot");
  sendChunk("</div>");
  
  // Status Display (if BLE enabled)
  if (g_bleEnabled) {
    sendChunk("<div style='margin-top:15px; padding:10px; background:#d4edda; border-left:4px solid #28a745; border-radius:4px;'>");
    sendChunk("<strong>✓ BLE Active</strong><br>");
    sendChunk("Device: <strong>" + htmlEscape(String(g_bleDeviceName)) + "</strong><br>");
    sendChunk("Status: ");
    
    if (BLESerial::isConnected()) {
      sendChunk("<span style='color:green;'>Connected & Paired");
      String clientName = BLESerial::getClientName();
      if (clientName.length() > 0) {
        sendChunk(" (" + htmlEscape(clientName) + ")");
      }
      sendChunk("</span><br>");
      sendChunk("Security: Encrypted");
    } else {
      sendChunk("<span style='color:orange;'>Advertising (waiting for client)</span>");
    }
    
    sendChunk("</div>");
  }
  
  sendChunk("<div style='margin-top:10px; font-size:12px; color:#7f8c8d;'>");
  sendChunk("<strong>Compatible Apps:</strong> SW Maps, GNSS Master, Lefebure NTRIP Client");
  sendChunk("</div>");
  
  sendChunk("</div>");  // End card
}

// ========================================================================
// BLE RTCM Input Card (Settings page)
// ========================================================================

static void renderBleRtcmCard() {
  sendChunk("<div class='card'><h2>&#x1F4E1; BLE RTCM Input</h2>");
  sendChunk("<div style='margin-bottom:10px;color:#666;font-size:0.9em;'>");
  sendChunk("Receive RTCM corrections from a rtcm-lora radio bridge via Bluetooth LE.<br>");
  sendChunk("This is a correction source, like NTRIP or TCP-IN. Only one can be active.");
  sendChunk("</div>");

  sendChunk("<form method='POST' action='/settings/blertcm'>");

  // Enable toggle
  sendChunk("<div class='form-row'>");
  sendChunk("<label>BLE RTCM Enable:</label>");
  sendChunk("<select name='blertcm_enable'>");
  if (g_bleRtcmEnabled) {
    sendChunk("<option value='1' selected>ON</option><option value='0'>OFF</option>");
  } else {
    sendChunk("<option value='1'>ON</option><option value='0' selected>OFF</option>");
  }
  sendChunk("</select></div>");

  // Target device name
  sendChunk("<div class='form-row' style='margin-top:10px'>");
  sendChunk("<label>rtcm-lora Device Name:</label>");
  sendChunk("<input type='text' name='blertcm_target' value='" + htmlEscape(String(g_bleRtcmTargetName)) + "' maxlength='20' placeholder='rtcm-lora' style='width:220px' />");
  sendChunk("<br><small style='color:#666'>Name of the rtcm-lora BLE device to connect to</small>");
  sendChunk("</div>");

  // Pairing PIN
  sendChunk("<div class='form-row' style='margin-top:10px'>");
  sendChunk("<label>Pairing PIN:</label>");
  char pinStr[8]; snprintf(pinStr, sizeof(pinStr), "%06u", g_bleRtcmPasskey);
  sendChunk("<input type='text' name='blertcm_pin' value='");
  sendChunk(pinStr);
  sendChunk("' maxlength='6' pattern='[0-9]{6}' placeholder='123456' style='width:150px' />");
  sendChunk("<br><small style='color:#666'>Must match the PIN set on the rtcm-lora device</small>");
  sendChunk("</div>");

  sendChunk("<button type='submit' class='btn btn-primary' style='margin-top:15px'>Apply BLE RTCM Settings</button>");
  sendChunk("</form>");

  // Status display
  if (g_bleRtcmEnabled) {
    sendChunk("<div style='margin-top:15px; padding:10px; background:#d4edda; border-left:4px solid #28a745; border-radius:4px;'>");
    sendChunk("<strong>&#x2713; BLE RTCM Active</strong><br>");
    sendChunk("Target: <strong>" + htmlEscape(String(g_bleRtcmTargetName)) + "</strong><br>");
    sendChunk("Status: ");
    if (g_bleRtcm.isConnected()) {
      sendChunk("<span style='color:green;'>Connected");
      if (g_bleRtcm.isStreaming()) sendChunk(" &mdash; Streaming");
      sendChunk("</span><br>");
      char buf[64];
      snprintf(buf, sizeof(buf), "RX: %lu bytes (%lu chunks)",
               (unsigned long)g_bleRtcm.getRxBytes(), (unsigned long)g_bleRtcm.getRxChunks());
      sendChunk(buf);
    } else if (g_bleRtcm.isScanning()) {
      sendChunk("<span style='color:orange;'>Scanning...</span>");
    } else {
      sendChunk("<span style='color:orange;'>Waiting</span>");
    }
    sendChunk("</div>");
  }

  // Info box
  sendChunk("<div style='margin-top:15px; padding:12px; background:#d1ecf1; border-left:4px solid #17a2b8; border-radius:4px;'>");
  sendChunk("<strong>&#x2139;&#xFE0F; How it works:</strong><br>");
  sendChunk("1. Enable BLE on rtcm-lora (Rover role) and note its device name and PIN<br>");
  sendChunk("2. Enter that name and PIN here, then enable<br>");
  sendChunk("3. RTKino scans, pairs and connects automatically<br>");
  sendChunk("4. Use the &#x25B6;/&#x25A0; buttons on the Dashboard to start/stop RTCM flow<br>");
  sendChunk("5. Data path: Base &rarr; LoRa &rarr; rtcm-lora &rarr; BLE &rarr; ZED-F9P<br>");
  sendChunk("<br><small>Enabling BLE RTCM disables NTRIP and TCP-IN (one correction source at a time).</small>");
  sendChunk("</div>");

  sendChunk("</div>");
}

// ========================================================================
// SETTINGS PAGE (/settings) - WiFi, NTP, ZED rate, System
// ========================================================================

static void handleSettingsPage() {
  sendHeader("Settings", "settings");
  
  // System Controls Card (at top of settings)
  sendChunk("<div class='card'><h2>⚙️ System Controls</h2>");
  
  // ZED-F9P Reset buttons
  sendChunk("<h3>ZED-F9P Reset</h3>");
  sendChunk("<button onclick=\"resetZed('hot')\" class='btn'>🔄 Hot Reset</button> ");
  sendChunk("<button onclick=\"resetZed('cold')\" class='btn btn-warning'>🔄 Cold Reset</button>");
  sendChunk("<p style='font-size:0.85em;color:#666;margin-top:8px;'>");
  sendChunk("⚠️ <b>Hot Reset:</b> Keeps ephemeris, fast restart (~5s)<br>");
  sendChunk("⚠️ <b>Cold Reset:</b> Clears everything, slow restart (~30s)");
  sendChunk("</p>");
  
  // ESP32 Reboot button
  sendChunk("<h3>ESP32</h3>");
  sendChunk("<button onclick=\"rebootEsp()\" class='btn btn-danger'>🔄 Reboot ESP32</button>");
  sendChunk("<p style='font-size:0.85em;color:#666;margin-top:8px;'>");
  sendChunk("⚠️ Connection will be lost during reboot.");
  sendChunk("</p>");
  
  // JavaScript for confirmations
  sendChunk("<script>");
  sendChunk("function resetZed(type){");
  sendChunk("var coldMsg='Cold Reset will clear all ephemeris data.\\nFix will take ~30 seconds.\\n\\nContinue?';");
  sendChunk("var hotMsg='Hot Reset will restart ZED-F9P keeping ephemeris.\\n\\nContinue?';");
  sendChunk("var msg=type==='cold'?coldMsg:hotMsg;");
  sendChunk("if(confirm('⚠️ '+msg)){");
  sendChunk("fetch('/api/zed/reset?type='+type)");
  sendChunk(".then(r=>r.text())");
  sendChunk(".then(t=>{alert(t);location.reload();})");
  sendChunk(".catch(err=>{alert('Error resetting ZED: '+(err.message||err));});");
  sendChunk("}");
  sendChunk("}");
  sendChunk("function rebootEsp(){");
  sendChunk("if(confirm('⚠️ Reboot ESP32?\\nConnection will be lost.')){");
  sendChunk("location.href='/reboot';");
  sendChunk("}");
  sendChunk("}");
  sendChunk("</script>");
  
  sendChunk("</div>");
  
  // ===== BLE (Bluetooth Low Energy) Section - MOVED HERE (second position) =====
  renderBluetoothCard();
  renderBleRtcmCard();
  
  // WiFi Configuration
  std::vector<WifiCred> wifiList;
  WifiProfiles::loadFromFlash(wifiList);
  WifiProfiles::sortByPriority(wifiList);
  
  sendChunk("<div class='card'><h2>WiFi Networks</h2>");
  
  if (!wifiList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Priority</th><th>SSID</th><th>Actions</th></tr>");
    for (size_t i=0;i<wifiList.size();++i){
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Priority'>" + String(wifiList[i].priority) + "</td>";
      row += "<td data-label='SSID'>" + htmlEscape(wifiList[i].ssid) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/wifi/edit?idx=" + String(i) + "' class='btn btn-small'>✏️ Edit</a> ";
      row += "<a href='/wifi/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(wifiList[i].ssid) + "?\")'>❌ Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No WiFi networks saved.</em></p>");
  }
  
  sendChunk("<h3>Add WiFi Network</h3>");
  sendChunk("<form method='POST' action='/wifi/add'>");
  sendChunk("<label>Priority:</label><input name='prio' type='number' value='10' style='width:100px'><br>");
  sendChunk("<label>SSID:</label><input name='ssid' required><br>");
  sendChunk("<label>Password:</label><input name='pass' type='password'><br>");
  sendChunk("<button type='submit'>Add Network</button>");
  sendChunk("</form></div>");
  
  // NTP / Time
  String fromFile;
  if (loadNtpServerFile(fromFile)) {
    strncpy(g_ntpServer, fromFile.c_str(), sizeof(g_ntpServer) - 1);
    g_ntpServer[sizeof(g_ntpServer) - 1] = 0;
  }
  String tzFromFile;
  if (loadNtpTzFile(tzFromFile)) {
    strncpy(g_ntpTz, tzFromFile.c_str(), sizeof(g_ntpTz) - 1);
    g_ntpTz[sizeof(g_ntpTz) - 1] = 0;
  }
  
  time_t nowEpoch = time(nullptr);
  String nowStr = "n/a";
  if (nowEpoch > 1700000000) {
    struct tm lt {};
    localtime_r(&nowEpoch, &lt);
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
             lt.tm_hour, lt.tm_min, lt.tm_sec);
    nowStr = buf;
  }
  
  String lastStr = "n/a";
  if ((time_t)g_lastSyncEpoch > 1700000000) {
    time_t t = (time_t)g_lastSyncEpoch;
    struct tm lt2 {};
    localtime_r(&t, &lt2);
    char buf2[32];
    snprintf(buf2, sizeof(buf2), "%04d-%02d-%02d %02d:%02d:%02d",
             lt2.tm_year + 1900, lt2.tm_mon + 1, lt2.tm_mday,
             lt2.tm_hour, lt2.tm_min, lt2.tm_sec);
    lastStr = buf2;
  }
  
  sendChunk("<div class='card'><h2>NTP / Time</h2>");
  sendChunk("<p><strong>Current Time:</strong> " + nowStr + "</p>");
  sendChunk("<p><strong>Source:</strong> " + String(timeSourceName(g_timeSource)) + "</p>");
  sendChunk("<p><strong>Last Sync:</strong> " + lastStr + "</p>");
  
  sendChunk("<h3>NTP Server</h3>");
  sendChunk("<form method='POST' action='/ntp/save'>");
  sendChunk("<label>Server:</label><input name='server' value='" + htmlEscape(String(g_ntpServer)) + "' style='width:320px'><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("</form>");
  sendChunk("<p><a href='/ntp/sync' class='btn'>Sync Now</a></p>");

  sendChunk("<h3>Timezone</h3>");
  sendChunk("<p>Active POSIX string: <code>" + htmlEscape(String(g_ntpTz)) + "</code></p>");
  sendChunk("<form method='POST' action='/ntp/tz/save'>");
  sendChunk("<label>Timezone:</label><select name='tz' style='width:320px'>");
  static const struct { const char* label; const char* posix; } TZ_LIST[] = {
    { "UTC",                                  "UTC0"                                      },
    { "London (GMT/BST)",                     "GMT0BST,M3.5.0/1,M10.5.0"                 },
    { "Italy / Central Europe (CET/CEST)",    "CET-1CEST,M3.5.0/2,M10.5.0/3"            },
    { "Eastern Europe (EET/EEST)",            "EET-2EEST,M3.5.0/3,M10.5.0/4"            },
    { "Moscow (MSK)",                         "MSK-3"                                     },
    { "Dubai (GST)",                          "GST-4"                                     },
    { "India (IST)",                          "IST-5:30"                                  },
    { "Bangkok (ICT)",                        "ICT-7"                                     },
    { "China / Singapore (CST/SGT)",          "CST-8"                                     },
    { "Japan (JST)",                          "JST-9"                                     },
    { "Eastern Australia (AEST/AEDT)",        "AEST-10AEDT,M10.1.0,M4.1.0/3"            },
    { "New Zealand (NZST/NZDT)",              "NZST-12NZDT,M9.5.0,M4.1.0/3"             },
    { "Azores",                               "AZOT1AZOST,M3.5.0/0,M10.5.0/1"            },
    { "Brazil (BRT/BRST)",                    "BRT+3BRST,M10.3.0/0,M2.3.0/0"             },
    { "Argentina (ART)",                      "ART+3"                                     },
    { "Eastern US (EST/EDT)",                 "EST5EDT,M3.2.0,M11.1.0"                   },
    { "Central US (CST/CDT)",                 "CST6CDT,M3.2.0,M11.1.0"                   },
    { "Mountain US (MST/MDT)",                "MST7MDT,M3.2.0,M11.1.0"                   },
    { "Pacific US (PST/PDT)",                 "PST8PDT,M3.2.0,M11.1.0"                   },
  };
  String curTz = String(g_ntpTz);
  for (int i = 0; i < (int)(sizeof(TZ_LIST)/sizeof(TZ_LIST[0])); i++) {
    String sel = (curTz == TZ_LIST[i].posix) ? " selected" : "";
    sendChunk("<option value='" + htmlEscape(String(TZ_LIST[i].posix)) + "'" + sel + ">" +
              htmlEscape(String(TZ_LIST[i].label)) + "</option>");
  }
  sendChunk("</select><br>");
  sendChunk("<button type='submit'>Save</button>");
  sendChunk("</form>");
  sendChunk("</div>");
  
  // ZED Rate
  sendChunk("<div class='card'><h2>ZED-F9P Update Rate</h2>");
  sendChunk("<p><strong>Last rate set:</strong> " + lastRateSet + "</p>");
  sendChunk("<form method='POST' action='/setrate'>");
  sendChunk("<label>Select frequency:</label><br>");
  sendChunk("<select name='rate'>");
  sendChunk("<option value='1000'>1 Hz</option>");
  sendChunk("<option value='500'>2 Hz</option>");
  sendChunk("<option value='200'>5 Hz</option>");
  sendChunk("<option value='100'>10 Hz</option>");
  sendChunk("<option value='66'>15 Hz</option>");
  sendChunk("</select><br>");
  sendChunk("<button type='submit'>Set Rate</button>");
  sendChunk("</form></div>");
  
  // Antenna Models
  std::vector<AntennaRec> antennas;
  loadAntennas(antennas);
  
  sendChunk("<div class='card'><h2>📡 Antenna Models</h2>");
  sendChunk("<p>Manage antenna models with ARP to phase center offsets</p>");
  
  if (!antennas.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Offset (m)</th><th>Actions</th></tr>");
    for (size_t i=0;i<antennas.size();++i){
      auto& a=antennas[i];
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(a.name) + "</td>";
      row += "<td data-label='Offset (m)'>" + String(a.offset,3) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/antennas/edit?idx=" + String(i) + "' class='btn btn-small'>✏️ Edit</a> ";
      row += "<a href='/antennas/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(a.name) + "?\")'>❌ Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No antenna models configured.</em></p>");
  }
  
  sendChunk("<h3>Add Antenna Model</h3>");
  sendChunk("<form method='POST' action='/antennas/add'>");
  sendChunk("<label>Name:</label><input name='name' required placeholder='e.g. u-blox ANN-MB'><br>");
  sendChunk("<label>Offset (m):</label><input name='offset' type='number' step='0.001' value='0.000' style='width:150px' required> (ARP to phase center)<br>");
  sendChunk("<button type='submit'>Add Antenna</button>");
  sendChunk("</form></div>");

  // Point Codes card
  sendChunk("<div class='card'>");
  sendChunk("<h2>&#128205; Point Codes</h2>");
  sendChunk("<p style='color:#666;font-size:13px'>List of categories and codes used during the survey. Freely editable; use &ldquo;Reset&rdquo; to restore defaults.</p>");
  sendChunk("<div id='codes-editor'>");
  sendChunk("<textarea id='codes-ta' style='width:100%;height:300px;font-family:monospace;font-size:12px;border:1px solid #ddd;border-radius:4px;padding:8px'>Loading...</textarea>");
  sendChunk("</div>");
  sendChunk("<div style='margin-top:12px'>");
  sendChunk("<button onclick='saveCodes()' class='btn'>&#128190; Save</button>");
  sendChunk("<button onclick='resetCodes()' class='btn' style='margin-left:8px;background:#e67e22'>&#128260; Reset default</button>");
  sendChunk("</div>");
  sendChunk("</div>");

  // Backup & Restore
  sendChunk("<div class='card'>");
  sendChunk("<h2>&#128190; Backup &amp; Restore</h2>");
  
  // Export
  sendChunk("<div style='margin-bottom:20px'>");
  sendChunk("<button onclick=\"window.location='/api/config/export'\" class='btn'>📥 Export Config</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Download all settings as JSON</span>");
  sendChunk("</div>");
  
  // Sync Flash→SD
  sendChunk("<div style='margin-bottom:20px'>");
  sendChunk("<button onclick=\"syncToSD()\" class='btn'>💾 Sync Flash→SD</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Copia subito i settings dalla flash alla SD card</span>");
  sendChunk("<p id='syncStatus' style='margin-top:8px'></p>");
  sendChunk("</div>");
  
  // Import
  sendChunk("<div>");
  sendChunk("<input type='file' id='importFile' accept='.json' style='margin-bottom:10px'><br>");
  sendChunk("<button onclick='importConfig()' class='btn'>📤 Import Config</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Upload and restore settings</span>");
  sendChunk("<p id='importStatus' style='margin-top:10px'></p>");
  sendChunk("</div>");
  
  sendChunk("</div>");
  
  // JavaScript for import and sync
  sendChunk("<script>");
  sendChunk("function importConfig(){");
  sendChunk("const f=document.getElementById('importFile').files[0];");
  sendChunk("if(!f){alert('Select a file first');return;}");
  sendChunk("const r=new FileReader();");
  sendChunk("r.onload=function(e){");
  sendChunk("fetch('/api/config/import',{method:'POST',body:e.target.result,headers:{'Content-Type':'application/json'}})");
  sendChunk(".then(r=>r.json()).then(d=>{");
  sendChunk("if(d.success){document.getElementById('importStatus').innerHTML='<span style=\"color:green\">✅ Import successful! Reboot recommended.</span>';}");
  sendChunk("else{document.getElementById('importStatus').innerHTML='<span style=\"color:red\">❌ Import failed: '+d.errors+'</span>';}");
  sendChunk("}).catch(e=>{document.getElementById('importStatus').innerHTML='<span style=\"color:red\">❌ Error: '+e+'</span>';});");
  sendChunk("};");
  sendChunk("r.readAsText(f);");
  sendChunk("}");
  sendChunk("function syncToSD(){");
  sendChunk("document.getElementById('syncStatus').innerHTML='<span style=\"color:#888\">&#9203; Sync in corso...</span>';");
  sendChunk("fetch('/api/config/sync')");
  sendChunk(".then(r=>r.text())");
  sendChunk(".then(t=>{document.getElementById('syncStatus').innerHTML='<span style=\"color:green\">&#10003; '+t+'</span>';})");
  sendChunk(".catch(e=>{document.getElementById('syncStatus').innerHTML='<span style=\"color:red\">&#9888; Error: '+e+'</span>';});");
  sendChunk("}");
  sendChunk("function loadCodesEditor(){");
  sendChunk("  fetch('/api/codes').then(function(r){return r.json();}).then(function(d){");
  sendChunk("    var ta=document.getElementById('codes-ta');");
  sendChunk("    if(ta)ta.value=JSON.stringify(d,null,2);");
  sendChunk("  }).catch(function(){});");
  sendChunk("}");
  sendChunk("function saveCodes(){");
  sendChunk("  var ta=document.getElementById('codes-ta');");
  sendChunk("  if(!ta)return;");
  sendChunk("  fetch('/api/codes',{method:'POST',headers:{'Content-Type':'application/json'},body:ta.value})");
  sendChunk("    .then(function(r){return r.json();}).then(function(d){");
  sendChunk("      alert(d.ok?'Saved!':'Error: '+(d.error||'?'));");
  sendChunk("    }).catch(function(e){alert('Network error: '+e);});");
  sendChunk("}");
  sendChunk("function resetCodes(){");
  sendChunk("  if(!confirm('Restore default codes?'))return;");
  sendChunk("  fetch('/api/codes/reset',{method:'POST'}).then(function(r){return r.json();}).then(function(d){");
  sendChunk("    if(d.ok){alert('Reset OK');loadCodesEditor();}");
  sendChunk("  }).catch(function(e){alert('Error: '+e);});");
  sendChunk("}");
  sendChunk("loadCodesEditor();");
  sendChunk("</script>");

  // ===== Factory Reset Section =====
  sendChunk("<div class='card'>");
  sendChunk("<h2>&#9888;&#65039; Factory Reset</h2>");
  sendChunk("<p style='color:#888;font-size:13px;margin-bottom:15px'>Erase all configuration, survey data, and stakeout files from internal flash. RTKino will restart as if it were brand new.</p>");

  // Flash-only reset
  sendChunk("<div style='margin-bottom:15px'>");
  sendChunk("<button onclick='factoryReset(false)' class='btn btn-warning'>&#128465; Reset Flash Only</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Keeps SD card data intact</span>");
  sendChunk("</div>");

  // Flash + SD reset
  sendChunk("<div>");
  sendChunk("<button onclick='factoryReset(true)' class='btn btn-danger'>&#128465; Reset Flash + SD</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Wipes everything (config, surveys, stakeout) from both flash and SD</span>");
  sendChunk("</div>");

  sendChunk("<p id='resetStatus' style='margin-top:12px'></p>");
  sendChunk("</div>");

  sendChunk("<script>");
  sendChunk("function factoryReset(wipeSD){");
  sendChunk("  var msg=wipeSD?'⚠️ This will ERASE ALL DATA from flash AND SD card.\\n\\nWiFi, NTRIP, bases, surveys, stakeout — everything will be lost.\\n\\nAre you absolutely sure?'");
  sendChunk("                :'⚠️ This will ERASE ALL DATA from internal flash.\\n\\nWiFi, NTRIP, bases, surveys, stakeout config will be lost.\\nSD card data will NOT be erased.\\n\\nContinue?';");
  sendChunk("  if(!confirm(msg))return;");
  sendChunk("  if(wipeSD&&!confirm('LAST CHANCE — Wipe SD card too?\\n\\nThis cannot be undone.'))return;");
  sendChunk("  document.getElementById('resetStatus').innerHTML='<span style=\"color:#e67e22\">&#9203; Factory reset in progress...</span>';");
  sendChunk("  fetch('/api/factory-reset?wipeSD='+(wipeSD?'1':'0'))");
  sendChunk("  .then(r=>r.json()).then(d=>{");
  sendChunk("    if(d.status==='ok'){");
  sendChunk("      document.getElementById('resetStatus').innerHTML='<span style=\"color:green\">&#10003; Reset complete. Rebooting...</span>';");
  sendChunk("      setTimeout(function(){location.reload();},5000);");
  sendChunk("    }else{");
  sendChunk("      document.getElementById('resetStatus').innerHTML='<span style=\"color:red\">&#9888; Error: '+(d.error||'unknown')+'</span>';");
  sendChunk("    }");
  sendChunk("  }).catch(e=>{document.getElementById('resetStatus').innerHTML='<span style=\"color:red\">&#9888; '+e+'</span>';});");
  sendChunk("}");
  sendChunk("</script>");

  // ===== NEW: System Logs Section =====
  sendChunk("<div class='card'><h2>📝 System Logs</h2>");
  sendChunk("<p>View and download system event logs (max 3 files)</p>");
  sendChunk("<div id='logsList'>");
  sendChunk("<p>Loading...</p>");
  sendChunk("</div>");
  sendChunk("</div>");
  
  // ===== NEW: Audio/Buzzer Section =====
  sendChunk("<div class='card'><h2>🔊 Audio Settings</h2>");
  sendChunk("<p><em>Buzzer configuration for RTK events (GPIO 5)</em></p>");
  sendChunk("<form method='POST' action='/audio/save'>");
  
  // Check current buzzer state
  bool buzzerEnabled = (g_buzzer && g_buzzer->isEnabled());
  String checkedAttr = buzzerEnabled ? " checked" : "";
  sendChunk("<label><input type='checkbox' name='enabled' value='1'" + checkedAttr + "> Enable Audio Alerts</label><br>");
  
  sendChunk("<p style='margin-top:10px'><strong>Sound Events:</strong></p>");
  sendChunk("<p>• RTK Fix Acquired: 2 ascending beeps</p>");
  sendChunk("<p>• RTK Fix Lost: 3 descending beeps</p>");
  sendChunk("<button type='submit' class='btn'>Save Audio Settings</button>");
  sendChunk("</form>");
  
  // Custom Melody Upload
  sendChunk("<h4 style='margin-top:20px'>Custom Melody (JSON)</h4>");
  sendChunk("<form action='/audio/upload' method='POST' enctype='multipart/form-data'>");
  sendChunk("<input type='file' name='melody' accept='.json'>");
  sendChunk("<button type='submit' class='btn'>Upload</button>");
  sendChunk("</form>");
  sendChunk("<p style='font-size:0.85em;color:#666;'>");
  sendChunk("Format: {\"name\":\"RTK Fixed\",\"tones\":[{\"freq\":880,\"duration\":100},{\"freq\":1318,\"duration\":150}]}");
  sendChunk("</p>");
  
  sendChunk("</div>");
  
  // JavaScript for loading system logs
  sendChunk("<script>");
  sendChunk("function loadLogs(){");
  sendChunk("fetch('/api/logs').then(r=>r.json()).then(d=>{");
  sendChunk("let html='';");
  sendChunk("if(d.files && d.files.length>0){");
  sendChunk("html='<ul class=\"log-list\">';");
  sendChunk("for(let f of d.files){");
  sendChunk("html+='<li><strong>'+f+'</strong><br>';");
  sendChunk("html+='<a href=\"/api/logs/download?file='+encodeURIComponent(f)+'\" class=\"btn btn-small\">Download</a> ';");
  sendChunk("html+='<a href=\"/api/logs/delete?file='+encodeURIComponent(f)+'\" class=\"btn btn-small btn-danger\" onclick=\"return confirm(\\'Delete log?\\');\">Delete</a>';");
  sendChunk("html+='</li>';");
  sendChunk("}");
  sendChunk("html+='</ul>';");
  sendChunk("}else{html='<p><em>No system logs found.</em></p>';}");
  sendChunk("document.getElementById('logsList').innerHTML=html;");
  sendChunk("}).catch(e=>{document.getElementById('logsList').innerHTML='<p style=\"color:red\">Error loading logs</p>';});");
  sendChunk("}");
  sendChunk("loadLogs();");
  sendChunk("</script>");

  // ===== mDNS Hostname Section =====
  {
    String mdns = String(g_mdnsName);
    if (mdns.length() == 0) mdns = "rtkino";
    sendChunk("<div class='card'><h2>🌐 mDNS Hostname</h2>");
    sendChunk("<p>Access RTKino on the LAN without knowing the IP:</p>");
    sendChunk("<p><strong>Current:</strong> <code>http://" + htmlEscape(mdns) + ".local/</code></p>");
    sendChunk("<form method='POST' action='/mdns/save'>");
    sendChunk("<label>Hostname (letters, numbers, hyphen):</label>");
    sendChunk("<input name='hostname' value='" + htmlEscape(mdns) + "' style='width:220px' required> <span style='color:#666'>.local</span><br>");
    sendChunk("<button type='submit' class='btn'>Save</button>");
    sendChunk("</form>");
    sendChunk("<p style='font-size:0.85em;color:#666;margin-top:10px'>Changes take effect immediately if Wi-Fi is connected. Some networks may block mDNS.</p>");
    sendChunk("</div>");
  }
  
  // ===== Firmware OTA Section =====
  sendChunk("<div class='card'><h2>🔄 Firmware Update (OTA)</h2>");
  sendChunk("<p>Update RTKino firmware over-the-air via web browser</p>");
  
  // Current Partition Info
  sendChunk("<div style='background:#f8f9fa;padding:12px;border-radius:4px;margin:12px 0;'>");
  sendChunk("<strong>📍 Current:</strong> ");
  sendChunk(OTAManager::getCurrentPartitionInfo());
  sendChunk("<br><strong>📍 Next:</strong> ");
  sendChunk(OTAManager::getNextPartitionInfo());
  sendChunk("</div>");
  
  // Web Upload Form
  sendChunk("<h3>Upload Firmware</h3>");
  sendChunk("<p>Select and upload a <code>firmware.bin</code> file:</p>");
  
  sendChunk("<form id='otaUploadForm' method='POST' action='/firmware/upload' enctype='multipart/form-data'>");
  sendChunk("<input type='file' name='firmware' id='otaFirmwareFile' accept='.bin' required style='margin-bottom:10px;'><br>");
  sendChunk("<button type='submit' id='otaUploadBtn' class='btn btn-success'>📤 Upload & Update</button>");
  sendChunk("</form>");
  
  // Progress bar
  sendChunk("<div id='otaProgressContainer' style='display:none;margin-top:16px;'>");
  sendChunk("<div style='background:#e9ecef;border-radius:4px;height:24px;overflow:hidden;'>");
  sendChunk("<div id='otaProgressBar' style='background:#28a745;height:100%;width:0%;transition:width 0.3s;text-align:center;line-height:24px;color:white;font-weight:bold;'></div>");
  sendChunk("</div>");
  sendChunk("<p id='otaProgressText' style='margin-top:8px;'></p>");
  sendChunk("</div>");
  
  // Warnings
  sendChunk("<div style='background:#fff3cd;border-left:4px solid #ffc107;padding:12px;margin-top:16px;'>");
  sendChunk("<strong>⚠️ Important:</strong>");
  sendChunk("<ul style='margin:8px 0;padding-left:20px;'>");
  sendChunk("<li>Do NOT disconnect power during update (2-3 minutes)</li>");
  sendChunk("<li>Device will reboot automatically after update</li>");
  sendChunk("<li>All settings on SD card are preserved</li>");
  sendChunk("<li>Failed updates rollback automatically to previous firmware</li>");
  sendChunk("</ul>");
  sendChunk("</div>");
  
  // JavaScript for OTA upload
  sendChunk("<script>");
  
  // Upload handler
  sendChunk("document.getElementById('otaUploadForm').addEventListener('submit',function(e){");
  sendChunk("e.preventDefault();");
  sendChunk("var file=document.getElementById('otaFirmwareFile').files[0];");
  sendChunk("if(!file){alert('Please select a file');return;}");
  sendChunk("if(!file.name.endsWith('.bin')){alert('Please select a .bin file');return;}");
  sendChunk("var sizeMB=(file.size/(1024*1024)).toFixed(2);");
  sendChunk("if(!confirm('⚠️ Update firmware?\\n\\nFile: '+file.name+' ('+sizeMB+' MB)\\n\\nDevice will reboot after update.\\nDo NOT disconnect power!')){return;}");
  sendChunk("document.getElementById('otaUploadBtn').disabled=true;");
  sendChunk("document.getElementById('otaProgressContainer').style.display='block';");
  sendChunk("var formData=new FormData();");
  sendChunk("formData.append('firmware',file);");
  sendChunk("var xhr=new XMLHttpRequest();");
  sendChunk("xhr.upload.addEventListener('progress',function(e){");
  sendChunk("if(e.lengthComputable){");
  sendChunk("var percent=Math.round((e.loaded/e.total)*100);");
  sendChunk("document.getElementById('otaProgressBar').style.width=percent+'%';");
  sendChunk("document.getElementById('otaProgressBar').textContent=percent+'%';");
  sendChunk("document.getElementById('otaProgressText').textContent='Uploading: '+percent+'% ('+Math.round(e.loaded/1024)+' / '+Math.round(e.total/1024)+' KB)';");
  sendChunk("}");
  sendChunk("});");
  sendChunk("xhr.addEventListener('load',function(){");
  sendChunk("if(xhr.status===200){");
  sendChunk("document.getElementById('otaProgressText').innerHTML='<span style=\"color:#28a745\">✓ Update successful! Rebooting...</span>';");
  sendChunk("setTimeout(function(){location.href='/';},5000);");
  sendChunk("}else{");
  sendChunk("document.getElementById('otaProgressText').innerHTML='<span style=\"color:#dc3545\">✗ Failed: '+xhr.responseText+'</span>';");
  sendChunk("document.getElementById('otaUploadBtn').disabled=false;");
  sendChunk("}");
  sendChunk("});");
  sendChunk("xhr.addEventListener('error',function(){");
  sendChunk("document.getElementById('otaProgressText').innerHTML='<span style=\"color:#dc3545\">✗ Upload error</span>';");
  sendChunk("document.getElementById('otaUploadBtn').disabled=false;");
  sendChunk("});");
  sendChunk("xhr.open('POST','/firmware/upload',true);");
  sendChunk("xhr.send(formData);");
  sendChunk("});");
  
  sendChunk("</script>");
  
  sendChunk("</div>");
  
  sendFooter();
}

// ========================================================================
// FIRMWARE UPDATE PAGE (/firmware)
// ========================================================================

static void handleFirmwarePage() {
  sendHeader("Firmware Update", "firmware");
  
  // OTA Information Card
  sendChunk("<div class='card'><h2>🔄 Firmware Update (OTA)</h2>");
  sendChunk("<p>Update your RTKino firmware over-the-air via web browser.</p>");
  
  // Current Partition Info
  sendChunk("<div style='background:#f8f9fa;padding:12px;border-radius:4px;margin:12px 0;'>");
  sendChunk("<strong>📍 Current Partition:</strong> ");
  sendChunk(OTAManager::getCurrentPartitionInfo());
  sendChunk("<br><strong>📍 Next Partition:</strong> ");
  sendChunk(OTAManager::getNextPartitionInfo());
  sendChunk("</div>");
  
  sendChunk("</div>");
  
  // Web Upload
  sendChunk("<div class='card'><h3>📤 Upload Firmware</h3>");
  sendChunk("<p>Select a <code>firmware.bin</code> file to update your device:</p>");
  
  sendChunk("<form id='uploadForm' method='POST' action='/firmware/upload' enctype='multipart/form-data'>");
  sendChunk("<input type='file' name='firmware' id='firmwareFile' accept='.bin' required style='margin:12px 0;'><br>");
  sendChunk("<button type='submit' id='uploadBtn' class='btn btn-success'>📤 Upload & Install</button>");
  sendChunk("</form>");
  
  // Progress bar
  sendChunk("<div id='progressContainer' style='display:none;margin-top:16px;'>");
  sendChunk("<div style='background:#e9ecef;border-radius:4px;height:24px;overflow:hidden;'>");
  sendChunk("<div id='progressBar' style='background:#28a745;height:100%;width:0%;transition:width 0.3s;text-align:center;line-height:24px;color:white;font-weight:bold;'></div>");
  sendChunk("</div>");
  sendChunk("<p id='progressText' style='margin-top:8px;'></p>");
  sendChunk("</div>");
  
  // JavaScript for upload handling
  sendChunk("<script>");
  sendChunk("document.getElementById('uploadForm').addEventListener('submit',function(e){");
  sendChunk("e.preventDefault();");
  sendChunk("var file=document.getElementById('firmwareFile').files[0];");
  sendChunk("if(!file){alert('Please select a file');return;}");
  sendChunk("if(!file.name.endsWith('.bin')){alert('Please select a .bin file');return;}");
  sendChunk("if(!confirm('⚠️ Update firmware to '+file.name+'?\\n\\nDevice will reboot after update.\\n\\nDo NOT disconnect power during update!')){return;}");
  sendChunk("document.getElementById('uploadBtn').disabled=true;");
  sendChunk("document.getElementById('progressContainer').style.display='block';");
  sendChunk("var formData=new FormData();");
  sendChunk("formData.append('firmware',file);");
  sendChunk("var xhr=new XMLHttpRequest();");
  sendChunk("xhr.upload.addEventListener('progress',function(e){");
  sendChunk("if(e.lengthComputable){");
  sendChunk("var percent=Math.round((e.loaded/e.total)*100);");
  sendChunk("document.getElementById('progressBar').style.width=percent+'%';");
  sendChunk("document.getElementById('progressBar').textContent=percent+'%';");
  sendChunk("document.getElementById('progressText').textContent='Uploading: '+percent+'% ('+Math.round(e.loaded/1024)+' KB / '+Math.round(e.total/1024)+' KB)';");
  sendChunk("}");
  sendChunk("});");
  sendChunk("xhr.addEventListener('load',function(){");
  sendChunk("if(xhr.status===200){");
  sendChunk("document.getElementById('progressText').innerHTML='<span style=\"color:#28a745\">✓ Update successful! Device rebooting...</span>';");
  sendChunk("setTimeout(function(){location.href='/';},5000);");
  sendChunk("}else{");
  sendChunk("document.getElementById('progressText').innerHTML='<span style=\"color:#dc3545\">✗ Update failed: '+xhr.responseText+'</span>';");
  sendChunk("document.getElementById('uploadBtn').disabled=false;");
  sendChunk("}");
  sendChunk("});");
  sendChunk("xhr.addEventListener('error',function(){");
  sendChunk("document.getElementById('progressText').innerHTML='<span style=\"color:#dc3545\">✗ Upload error</span>';");
  sendChunk("document.getElementById('uploadBtn').disabled=false;");
  sendChunk("});");
  sendChunk("xhr.open('POST','/firmware/upload',true);");
  sendChunk("xhr.send(formData);");
  sendChunk("});");
  sendChunk("</script>");
  
  sendChunk("</div>");
  
  // How to get firmware.bin
  sendChunk("<div class='card'><h3>📝 How to Get firmware.bin</h3>");
  sendChunk("<p>If you have the firmware source code:</p>");
  sendChunk("<ol>");
  sendChunk("<li>Connect to the device via USB</li>");
  sendChunk("<li>Compile with PlatformIO: <code>pio run -e lolin_s3_pro</code></li>");
  sendChunk("<li>Find the firmware at: <code>.pio/build/lolin_s3_pro/firmware.bin</code></li>");
  sendChunk("<li>Upload it using the form above</li>");
  sendChunk("</ol>");
  sendChunk("</div>");
  
  // Warnings
  sendChunk("<div class='card' style='background:#fff3cd;border-left:4px solid #ffc107;'>");
  sendChunk("<h3>⚠️ Important Notes</h3>");
  sendChunk("<ul>");
  sendChunk("<li><strong>Backup first:</strong> Copy the SD card contents before updating</li>");
  sendChunk("<li><strong>Stable connection:</strong> Ensure strong WiFi signal during update</li>");
  sendChunk("<li><strong>Don't disconnect:</strong> Do NOT remove power during update (2-3 minutes)</li>");
  sendChunk("<li><strong>Settings preserved:</strong> All configurations on SD card will remain intact</li>");
  sendChunk("<li><strong>Automatic rollback:</strong> If update fails, device will restore previous firmware</li>");
  sendChunk("<li><strong>Test first:</strong> Always test new firmware on a spare device if possible</li>");
  sendChunk("</ul>");
  sendChunk("</div>");
  
  sendFooter();
}

// ========================================================================
// LOG FILES PAGE
// ========================================================================

static void handleLogsPage() {
  sendHeader("Log Files", "logs");
  
  sendChunk("<div class='card'>");
  sendChunk("<h2>📁 Logged Files</h2>");
  
  // Lock SD
  bool locked = sdLock(5000);
  if (!locked) {
    sendChunk("<p style='color:red;'>⚠️ SD card busy, try again</p>");
    sendChunk("</div>");
    sendFooter();
    return;
  }
  
  FsFile dir = _sd->open("/gnss");
  if (!dir || !dir.isDirectory()) {
    sendChunk("<p>No log files found</p>");
    sendChunk("</div>");
    sendFooter();
    sdUnlock(locked);
    return;
  }
  
  // Collect ALL .ubx files (no limit)
  struct LogFile {
    String name;
    uint32_t size;
  };
  
  std::vector<LogFile> logFiles;
  FsFile file;
  while ((file = dir.openNextFile())) {
    char name[64];
    file.getName(name, sizeof(name));
    String filename = String(name);
    
    if (filename.endsWith(".ubx")) {
      LogFile lf;
      lf.name = filename;
      lf.size = file.size();
      logFiles.push_back(lf);
    }
    file.close();
  }
  dir.close();
  sdUnlock(locked);
  
  if (logFiles.empty()) {
    sendChunk("<p>No .ubx log files found</p>");
  } else {
    // Responsive table
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table>");
    sendChunk("<thead><tr>");
    sendChunk("<th>Filename</th>");
    sendChunk("<th>Size</th>");
    sendChunk("<th>Actions</th>");
    sendChunk("</tr></thead>");
    sendChunk("<tbody>");
    
    for (size_t i = 0; i < logFiles.size(); i++) {
      const auto& lf = logFiles[i];
      
      // Format size
      float size = lf.size;
      String unit = " B";
      bool isBytes = true;
      if (size > 1024) { size /= 1024; unit = " KB"; isBytes = false; }
      if (size > 1024) { size /= 1024; unit = " MB"; isBytes = false; }
      if (size > 1024) { size /= 1024; unit = " GB"; isBytes = false; }
      
      sendChunk("<tr>");
      
      // Filename
      sendChunk("<td data-label='Filename'>");
      sendChunk(htmlEscape(lf.name));
      sendChunk("</td>");
      
      // Size
      sendChunk("<td data-label='Size'>");
      char sizeStr[32];
      if (isBytes) {
        snprintf(sizeStr, sizeof(sizeStr), "%.0f%s", size, unit.c_str());
      } else {
        snprintf(sizeStr, sizeof(sizeStr), "%.2f%s", size, unit.c_str());
      }
      sendChunk(sizeStr);
      sendChunk("</td>");
      
      // Actions
      sendChunk("<td data-label='Actions'>");
      
      // Download button
      sendChunk("<button class='btn btn-small' onclick='location.href=\"/download?file=");
      sendChunk(urlEncode(lf.name));
      sendChunk("\"'>⬇ Download</button> ");
      
      // Delete button
      sendChunk("<button class='btn btn-danger btn-small' onclick='if(confirm(\"Delete ");
      sendChunk(htmlEscape(lf.name));
      sendChunk("?\")) location.href=\"/delete?file=");
      sendChunk(urlEncode(lf.name));
      sendChunk("\"'>🗑️ Delete</button>");
      
      sendChunk("</td>");
      sendChunk("</tr>");
    }
    
    sendChunk("</tbody></table>");
    sendChunk("</div>");
    
    // Total summary
    uint64_t totalSize = 0;
    for (const auto& lf : logFiles) {
      totalSize += lf.size;
    }
    float totalMB = totalSize / 1024.0 / 1024.0;
    char totalStr[128];
    snprintf(totalStr, sizeof(totalStr), "<p><strong>Total:</strong> %d files, %.2f MB</p>", 
             (int)logFiles.size(), totalMB);
    sendChunk(totalStr);
    
    // Delete All button
    sendChunk("<div style='margin-top:20px; border-top:2px solid #e74c3c; padding-top:20px;'>");
    sendChunk("<button class='btn btn-danger' onclick='confirmDeleteAll()' style='width:100%; max-width:300px;'>");
    sendChunk("🗑️ Delete All Log Files");
    sendChunk("</button>");
    sendChunk("</div>");
    
    // JavaScript double confirmation
    sendChunk("<script>");
    sendChunk("function confirmDeleteAll() {");
    sendChunk("  var msg = '⚠️ DELETE ALL LOG FILES?\\n\\nThis will permanently delete ");
    char countStr[32];
    snprintf(countStr, sizeof(countStr), "%d", (int)logFiles.size());
    sendChunk(countStr);
    sendChunk(" files (");
    char sizeStr[32];
    snprintf(sizeStr, sizeof(sizeStr), "%.2f", totalMB);
    sendChunk(sizeStr);
    sendChunk(" MB).\\n\\nThis action CANNOT be undone!';");
    sendChunk("  if (confirm(msg)) {");
    sendChunk("    if (confirm('Are you ABSOLUTELY SURE?')) {");
    sendChunk("      location.href='/logs/deleteall';");
    sendChunk("    }");
    sendChunk("  }");
    sendChunk("}");
    sendChunk("</script>");
  }
  
  sendChunk("</div>");
  sendFooter();
}

static void handleDeleteAllLogs() {
  // Block if logging active (consistent with existing delete protection)
  if (loggingActive) {
    _server->send(409, "text/plain", "Cannot delete logs while logging is active");
    return;
  }
  
  bool locked = sdLock(10000);
  if (!locked) {
    _server->send(503, "text/plain", "SD card busy");
    return;
  }
  
  FsFile dir = _sd->open("/gnss");
  if (!dir) {
    sdUnlock(locked);
    _server->send(500, "text/plain", "Cannot open /gnss directory");
    return;
  }
  
  int deletedCount = 0;
  FsFile file;
  while ((file = dir.openNextFile())) {
    char name[64];
    file.getName(name, sizeof(name));
    String filename = String(name);
    file.close();
    
    // Only delete .ubx files
    if (filename.endsWith(".ubx")) {
      String fullPath = String("/gnss/") + filename;
      if (_sd->remove(fullPath.c_str())) {
        deletedCount++;
        Serial.printf("[LOGS] Deleted: %s\n", filename.c_str());
      }
    }
  }
  dir.close();
  sdUnlock(locked);
  
  Serial.printf("[LOGS] Delete All completed: %d files removed\n", deletedCount);
  
  // Redirect to logs page
  _server->sendHeader("Location", "/logs");
  _server->send(303, "text/plain", String("Deleted ") + deletedCount + " files");
}

// ========================================================================
// WIFI CRUD HANDLERS
// ========================================================================

static void handleWifiAdd() {
  if (!_server->hasArg("ssid")) { _server->send(400,"text/plain","SSID missing"); return; }
  WifiCred c;
  c.priority = _server->hasArg("prio") ? _server->arg("prio").toInt() : 10;
  c.ssid     = _server->arg("ssid");
  c.password = _server->arg("pass");
  std::vector<WifiCred> list;
  WifiProfiles::loadFromFlash(list);
  list.push_back(c);
  WifiProfiles::sortByPriority(list);
  WifiProfiles::saveToFlash(list);
  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

static void handleWifiDel() {
  if (!_server->hasArg("idx")) { _server->send(400,"text/plain","idx missing"); return; }
  size_t idx = _server->arg("idx").toInt();
  if (!WifiProfiles::removeAtFlash(idx)) {
    _server->send(400,"text/plain","Delete failed");
    return;
  }
  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

static void handleWifiEdit() {
  if (!_server->hasArg("idx")) { _server->send(400,"text/plain","idx missing"); return; }
  size_t idx = _server->arg("idx").toInt();

  std::vector<WifiCred> list;
  WifiProfiles::loadFromFlash(list);
  if (idx >= list.size()) { _server->send(400,"text/plain","Index out of range"); return; }

  const WifiCred& c = list[idx];

  sendHeader("Edit WiFi Network", "settings");
  sendChunk("<div class='card'><h2>Edit WiFi Network</h2>");
  sendChunk("<form method='POST' action='/wifi/update'>");
  sendChunk("<input type='hidden' name='idx' value='" + String(idx) + "'>");
  sendChunk("<label>Priority:</label><input name='prio' type='number' value='" + String(c.priority) + "'><br>");
  sendChunk("<label>SSID:</label><input name='ssid' value='" + htmlEscape(String(c.ssid)) + "'><br>");
  sendChunk("<label>Password:</label><input name='pass' type='password' value='" + htmlEscape(String(c.password)) + "'><br>");
  sendChunk("<button type='submit'>Save Changes</button> ");
  sendChunk("<a class='btn' href='/settings'>Cancel</a>");
  sendChunk("</form></div>");
  sendFooter();
}


static void handleWifiUpdate() {
  if (!_server->hasArg("idx"))  { _server->send(400,"text/plain","idx missing"); return; }
  if (!_server->hasArg("ssid")) { _server->send(400,"text/plain","SSID missing"); return; }

  size_t idx = _server->arg("idx").toInt();
  String ssid = _server->arg("ssid");
  String pass = _server->arg("pass");
  int prio    = _server->hasArg("prio") ? _server->arg("prio").toInt() : 10;

  std::vector<WifiCred> list;
  WifiProfiles::loadFromFlash(list);
  if (idx >= list.size()) { _server->send(400,"text/plain","Index out of range"); return; }

  list[idx].ssid = ssid;
  list[idx].password = pass;
  list[idx].priority = prio;

  WifiProfiles::sortByPriority(list);
  WifiProfiles::saveToFlash(list);

  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

// ========================================================================
// ANTENNA CRUD HANDLERS
// ========================================================================

static void handleAntennaAdd() {
  if (!_server->hasArg("name") || !_server->hasArg("offset")) {
    _server->send(400, "text/plain", "Parameters missing");
    return;
  }
  
  AntennaRec a;
  a.name = clampSemi(_server->arg("name"));
  a.offset = _server->arg("offset").toFloat();
  
  std::vector<AntennaRec> v;
  loadAntennas(v);
  v.push_back(a);
  
  if (!saveAntennas(v)) {
    _server->send(500, "text/plain", "Save failed");
    return;
  }
  
  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

static void handleAntennaDel() {
  if (!_server->hasArg("idx")) {
    _server->send(400, "text/plain", "idx missing");
    return;
  }
  
  int idx = _server->arg("idx").toInt();
  std::vector<AntennaRec> v;
  loadAntennas(v);
  
  if (idx < 0 || idx >= (int)v.size()) {
    _server->send(400, "text/plain", "Index out of range");
    return;
  }
  
  v.erase(v.begin() + idx);
  
  if (!saveAntennas(v)) {
    _server->send(500, "text/plain", "Save failed");
    return;
  }
  
  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

static void handleAntennaEdit() {
  if (!_server->hasArg("idx")) {
    _server->send(400, "text/plain", "idx missing");
    return;
  }
  
  int idx = _server->arg("idx").toInt();
  std::vector<AntennaRec> v;
  loadAntennas(v);
  
  if (idx < 0 || idx >= (int)v.size()) {
    _server->send(400, "text/plain", "Index out of range");
    return;
  }
  
  const auto& a = v[idx];
  
  sendHeader("Edit Antenna", "settings");
  sendChunk("<div class='card'><h2>Edit Antenna Model</h2>");
  sendChunk("<form method='POST' action='/antennas/update'>");
  sendChunk("<input type='hidden' name='idx' value='" + String(idx) + "'>");
  sendChunk("<label>Name:</label><input name='name' value='" + htmlEscape(a.name) + "' required><br>");
  sendChunk("<label>Offset (m):</label><input name='offset' type='number' step='0.001' value='" + 
            String(a.offset, 3) + "' style='width:150px' required><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/settings'>Cancel</a>");
  sendChunk("</form></div>");
  sendFooter();
}

static void handleAntennaUpdate() {
  if (!_server->hasArg("idx") || !_server->hasArg("name") || !_server->hasArg("offset")) {
    _server->send(400, "text/plain", "Parameters missing");
    return;
  }
  
  int idx = _server->arg("idx").toInt();
  std::vector<AntennaRec> v;
  loadAntennas(v);
  
  if (idx < 0 || idx >= (int)v.size()) {
    _server->send(400, "text/plain", "Index out of range");
    return;
  }
  
  v[idx].name = clampSemi(_server->arg("name"));
  v[idx].offset = _server->arg("offset").toFloat();
  
  if (!saveAntennas(v)) {
    _server->send(500, "text/plain", "Save failed");
    return;
  }
  
  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

// ========================================================================
// NTP HANDLERS
// ========================================================================

static void handleNtpSave() {
  String s = _server->arg("server");
  s.trim();
  if (s.length() < 2) { _server->send(400, "text/plain", "Invalid server"); return; }

  if (!saveNtpServerFile(s)) { _server->send(503, "text/plain", "Write failed"); return; }

  strncpy(g_ntpServer, s.c_str(), sizeof(g_ntpServer) - 1);
  g_ntpServer[sizeof(g_ntpServer) - 1] = 0;

  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

static void handleNtpSync() {
  if (WiFi.status() != WL_CONNECTED) {
    _server->send(200, "text/plain", "WiFi not connected: NTP unavailable");
    return;
  }
  bool ok = syncTimeFromNtp(g_ntpServer);
  _server->send(200, "text/plain", ok ? "OK" : "FAIL");
}

static void handleNtpTzSave() {
  String s = _server->arg("tz");
  s.trim();
  if (s.isEmpty()) { _server->send(400, "text/plain", "Invalid timezone"); return; }

  if (!saveNtpTzFile(s)) { _server->send(503, "text/plain", "Write failed"); return; }
  FlashConfig::markDirty();

  strncpy(g_ntpTz, s.c_str(), sizeof(g_ntpTz) - 1);
  g_ntpTz[sizeof(g_ntpTz) - 1] = 0;
  applyTimezone();

  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

// ========================================================================
// mDNS HANDLERS
// ========================================================================

static void handleMdnsSave() {
  String s = _server->arg("hostname");
  s.trim();
  if (s.endsWith(".local")) s = s.substring(0, s.length() - 6);
  s.toLowerCase();

  if (!isValidMdnsHostLabel(s)) {
    _server->send(400, "text/plain", "Invalid mDNS hostname (use a-z, 0-9, '-')");
    return;
  }

  if (!saveMdnsNameFile(s)) {
    _server->send(503, "text/plain", "Write failed");
    return;
  }

  strncpy(g_mdnsName, s.c_str(), sizeof(g_mdnsName) - 1);
  g_mdnsName[sizeof(g_mdnsName) - 1] = 0;

  // Best effort: apply immediately (if Wi-Fi is up). Ignore failure and still redirect.
  (void)applyMdnsHostname(g_mdnsName);

  _server->sendHeader("Location", "/settings");
  _server->send(303);
}

// ========================================================================
// BLE SETTINGS HANDLER
// ========================================================================

static void handleBleSettings() {
  // Validate parameters
  if (!_server->hasArg("ble_enable") || !_server->hasArg("ble_name") || !_server->hasArg("ble_pin")) {
    _server->send(400, "text/plain", "Missing parameters");
    return;
  }
  
  bool newEnabled = (_server->arg("ble_enable") == "1");
  String newName = _server->arg("ble_name");
  String newPinStr = _server->arg("ble_pin");
  newName.trim();
  newPinStr.trim();
  
  // Validate device name
  if (newName.length() == 0) newName = "RTKino";
  if (newName.length() > 20) {
    _server->send(400, "text/plain", "Name too long (max 20 chars)");
    return;
  }
  
  // Validate characters (alphanumeric + _ -)
  for (size_t i = 0; i < newName.length(); i++) {
    char c = newName[i];
    if (!isalnum(c) && c != '_' && c != '-') {
      _server->send(400, "text/plain", "Invalid characters in name (use a-z A-Z 0-9 _ -)");
      return;
    }
  }
  
  // Validate PIN
  if (newPinStr.length() != 6) {
    _server->send(400, "text/plain", "PIN must be exactly 6 digits");
    return;
  }
  
  for (char c : newPinStr) {
    if (!isdigit(c)) {
      _server->send(400, "text/plain", "PIN must contain only digits (0-9)");
      return;
    }
  }
  
  uint32_t newPin = atoi(newPinStr.c_str());
  if (newPin > 999999) {
    _server->send(400, "text/plain", "Invalid PIN range");
    return;
  }
  
  // Save device name and PIN to flash (persistent)
  if (!saveBleName(newName.c_str())) {
    _server->send(500, "text/plain", "Failed to save BLE name");
    return;
  }
  
  if (!saveBlePin(newPin)) {
    _server->send(500, "text/plain", "Failed to save PIN");
    return;
  }
  
  // Apply new name and PIN
  applyBleName(newName.c_str());
  g_blePasskey = newPin;
  
  // Enable/disable BLE (runtime only - NOT saved to SD)
  if (newEnabled && !g_bleEnabled) {
    BLESerial::begin(g_bleDeviceName, g_blePasskey);
    g_bleEnabled = true;
    
    // Set RX callback for RTCM input (forward to ZED-F9P)
    extern HardwareSerial RTCMSerial;
    BLESerial::setRxCallback([](const uint8_t* data, size_t len) {
      // Process RTCM input: forward to ZED-F9P UART
      if (len > 0) {
        RTCMSerial.write(data, len);
      }
    });
    
    Serial.printf("[BLE] Enabled: %s (PIN: %06u)\n", g_bleDeviceName, g_blePasskey);
    
  } else if (!newEnabled && g_bleEnabled) {
    BLESerial::end();
    g_bleEnabled = false;
    Serial.println("[BLE] Disabled");
  } else if (newEnabled && g_bleEnabled) {
    // Name or PIN changed while BLE active - restart BLE with new settings
    const uint16_t BLE_RESTART_DELAY_MS = 500;  // Allow BLE to shut down cleanly
    BLESerial::end();
    delay(BLE_RESTART_DELAY_MS);
    BLESerial::begin(g_bleDeviceName, g_blePasskey);
    
    // Re-set RX callback
    extern HardwareSerial RTCMSerial;
    BLESerial::setRxCallback([](const uint8_t* data, size_t len) {
      if (len > 0) {
        RTCMSerial.write(data, len);
      }
    });
    
    Serial.printf("[BLE] Restarted: %s (PIN: %06u)\n", g_bleDeviceName, g_blePasskey);
  }
  
  // Redirect to settings
  _server->sendHeader("Location", "/settings");
  _server->send(303, "text/plain", "Bluetooth settings saved");
}

// ========================================================================
// BLE RTCM SETTINGS HANDLER
// ========================================================================

static void handleBleRtcmSettings() {
  if (!_server->hasArg("blertcm_enable") || !_server->hasArg("blertcm_target") || !_server->hasArg("blertcm_pin")) {
    _server->send(400, "text/plain", "Missing parameters");
    return;
  }

  bool newEnabled = (_server->arg("blertcm_enable") == "1");
  String newTarget = _server->arg("blertcm_target");
  String newPinStr = _server->arg("blertcm_pin");
  newTarget.trim();
  newPinStr.trim();

  if (newTarget.length() == 0) newTarget = "rtcm-lora";
  if (newTarget.length() > 20) {
    _server->send(400, "text/plain", "Name too long (max 20 chars)");
    return;
  }
  for (size_t i = 0; i < newTarget.length(); i++) {
    char c = newTarget[i];
    if (!isalnum(c) && c != '_' && c != '-') {
      _server->send(400, "text/plain", "Invalid characters in name");
      return;
    }
  }
  if (newPinStr.length() != 6) {
    _server->send(400, "text/plain", "PIN must be exactly 6 digits");
    return;
  }
  for (char c : newPinStr) {
    if (!isdigit(c)) {
      _server->send(400, "text/plain", "PIN must contain only digits");
      return;
    }
  }
  uint32_t newPin = (uint32_t)atoi(newPinStr.c_str());

  // Save config
  strncpy(g_bleRtcmTargetName, newTarget.c_str(), sizeof(g_bleRtcmTargetName) - 1);
  g_bleRtcmTargetName[sizeof(g_bleRtcmTargetName) - 1] = '\0';
  g_bleRtcmPasskey = newPin;

  FlashConfig::writeFile("/config/ble_rtcm_target.txt", newTarget);
  char pinBuf[8]; snprintf(pinBuf, sizeof(pinBuf), "%06u", newPin);
  FlashConfig::writeFile("/config/ble_rtcm_pin.txt", String(pinBuf));

  // Toggle
  if (newEnabled && !g_bleRtcmEnabled) {
    startBleRtcm(newTarget, newPin);
  } else if (!newEnabled && g_bleRtcmEnabled) {
    stopBleRtcm();
  } else if (newEnabled && g_bleRtcmEnabled) {
    stopBleRtcm();
    delay(200);
    startBleRtcm(newTarget, newPin);
  }

  _server->sendHeader("Location", "/settings");
  _server->send(303, "text/plain", "BLE RTCM settings saved");
}

// ========================================================================
// NTRIP IN CRUD HANDLERS
// ========================================================================

static void handleNtripAdd() {
  if(!_server->hasArg("name")||!_server->hasArg("host")||!_server->hasArg("port")||!_server->hasArg("mount")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  NtripIn n;
  n.name = clampSemi(_server->arg("name"));
  n.host = _server->arg("host");
  n.port = _server->arg("port").toInt();
  n.mount= _server->arg("mount");
  n.user = _server->arg("user");
  n.pass = _server->arg("pwd");
  std::vector<NtripIn> v; int last=-1; loadNtripInList(v,last);
  v.push_back(n); if (last<0) last=0;
  if(!saveNtripInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/rover"); _server->send(303);
}

static void handleNtripDel() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx = _server->arg("idx").toInt();
  std::vector<NtripIn> v; int last=-1; loadNtripInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v.erase(v.begin()+idx);
  if (last >= (int)v.size()) last = (int)v.size()-1;
  if(!saveNtripInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/rover"); _server->send(303);
}

static void handleNtripEdit() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx = _server->arg("idx").toInt();
  std::vector<NtripIn> v; int last=-1; loadNtripInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  const auto& n=v[idx];
  
  sendHeader("Edit NTRIP IN Profile", "rover");
  sendChunk("<div class='card'><h2>Edit NTRIP IN Profile</h2>");
  sendChunk("<form method='POST' action='/ntrip/update'>");
  sendChunk("<input type='hidden' name='idx' value='" + String(idx) + "'>");
  sendChunk("<label>Name:</label><input name='name' value='" + htmlEscape(n.name) + "' required><br>");
  sendChunk("<label>Host:</label><input name='host' value='" + htmlEscape(n.host) + "' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='" + String(n.port) + "' required><br>");
  sendChunk("<label>Mountpoint:</label><input name='mount' value='" + htmlEscape(n.mount) + "' required><br>");
  sendChunk("<label>User:</label><input name='user' value='" + htmlEscape(n.user) + "'><br>");
  sendChunk("<label>Password:</label><input name='pwd' type='password' value='" + htmlEscape(n.pass) + "'><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/rover'>Cancel</a>");
  sendChunk("</form></div>");
  sendFooter();
}

static void handleNtripUpdate() {
  if(!_server->hasArg("idx")||!_server->hasArg("name")||!_server->hasArg("host")||!_server->hasArg("port")||!_server->hasArg("mount")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  int idx = _server->arg("idx").toInt();
  std::vector<NtripIn> v; int last=-1; loadNtripInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v[idx].name = clampSemi(_server->arg("name"));
  v[idx].host = _server->arg("host");
  v[idx].port = _server->arg("port").toInt();
  v[idx].mount= _server->arg("mount");
  v[idx].user = _server->arg("user");
  v[idx].pass = _server->arg("pwd");
  if(!saveNtripInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/rover"); _server->send(303);
}

static void handleNtripSelect() {
  if (!_server->hasArg("idx")) {
    _server->send(400, "text/plain", "idx missing");
    return;
  }

  int idx = _server->arg("idx").toInt();
  std::vector<NtripIn> v; int last = -1;
  loadNtripInList(v, last);
  if (idx < 0 || idx >= (int)v.size()) {
    _server->send(400, "text/plain", "Index out of range");
    return;
  }

  // === LOCK for safe modification ===
  if (! ntripLock(2000)) {
    _server->send(503, "text/plain", "NTRIP busy, retry");
    return;
  }

  // Update global RAM variables FIRST - so system works even if SD save fails later
  const auto& n = v[idx];
  ntrip_host = n.host;
  ntrip_port = n.port;
  mountpoint = n.mount;
  ntrip_user = n.user;
  ntrip_pass = n.pass;
  String profileName = n.name;  // Save name for logging later

  // FIRST: disable flag (other tasks will stop using ntripClient)
  ntripEnabled = false;
  
  // Small pause to let other tasks see the change
  vTaskDelay(pdMS_TO_TICKS(50));

  // Now safe to stop/delete/new
  if (ntripClient) {
    ntripClient->stop();
    delete ntripClient;
    ntripClient = nullptr;
  }

  // Reset RTCM statistics when changing profile
  resetRtcmStats();

  // Create new client using global variables
  // Remove leading slash from mountpoint if present
  if (mountpoint.startsWith("/")) mountpoint.remove(0, 1);

  ntripClient = new NtripClient(ntrip_host.c_str(), ntrip_port, mountpoint.c_str(), ntrip_user.c_str(), ntrip_pass.c_str());
  ntripClient->setGgaMinPeriodMs(5000);
  ntripClient->begin(RTCMSerial);
  
  // AFTER: re-enable flag
  ntripEnabled = true;

  ntripUnlock();
  // === END LOCK ===

  oledSetNtrip(true);
  oledPrintln(String("[NTRIP] Active profile: ") + profileName);

  // Try to save LAST, but don't fail the operation if save fails
  // System is already working from RAM variables set above
  last = idx;
  if (!saveNtripInList(v, last)) {
    // Log the failure but don't abort - client is already running
    oledPrintln("[NTRIP] Warning: Failed to save LAST to SD");
  }

  _server->sendHeader("Location", "/rover");
  _server->send(303);
}

// ========================================================================
// LAN TCP-IN CRUD HANDLERS
// ========================================================================

static void handleLanInAdd() {
  if(!_server->hasArg("name")||!_server->hasArg("host")||!_server->hasArg("port")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  TcpIn t; t.name=clampSemi(_server->arg("name")); t.host=_server->arg("host"); t.port=_server->arg("port").toInt();
  std::vector<TcpIn> v; int last=-1; loadTcpInList(v,last);
  v.push_back(t); if (last<0) last=0;
  if(!saveTcpInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/rover"); _server->send(303);
}

static void handleLanInDel() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<TcpIn> v; int last=-1; loadTcpInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v.erase(v.begin()+idx);
  if (last >= (int)v.size()) last = (int)v.size()-1;
  if(!saveTcpInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/rover"); _server->send(303);
}

static void handleLanInEdit() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<TcpIn> v; int last=-1; loadTcpInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  const auto& t=v[idx];
  
  sendHeader("Edit TCP-IN Profile", "rover");
  sendChunk("<div class='card'><h2>Edit TCP-IN Profile</h2>");
  sendChunk("<form method='POST' action='/lanin/update'>");
  sendChunk("<input type='hidden' name='idx' value='" + String(idx) + "'>");
  sendChunk("<label>Name:</label><input name='name' value='" + htmlEscape(t.name) + "' required><br>");
  sendChunk("<label>Host/IP:</label><input name='host' value='" + htmlEscape(t.host) + "' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='" + String(t.port) + "' required><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/rover'>Cancel</a>");
  sendChunk("</form></div>");
  sendFooter();
}

static void handleLanInUpdate() {
  if(!_server->hasArg("idx")||!_server->hasArg("name")||!_server->hasArg("host")||!_server->hasArg("port")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  int idx=_server->arg("idx").toInt(); std::vector<TcpIn> v; int last=-1; loadTcpInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v[idx].name=clampSemi(_server->arg("name"));
  v[idx].host=_server->arg("host");
  v[idx].port=_server->arg("port").toInt();
  if(!saveTcpInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/rover"); _server->send(303);
}

static void handleLanInSelect() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<TcpIn> v; int last=-1; loadTcpInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  last=idx;
  if(!saveTcpInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  // update global variables for quick-start
  tcpin_host = v[idx].host; tcpin_port = v[idx].port;
  _server->sendHeader("Location","/rover"); _server->send(303);
}

static void handleLanInStart() {
  // Use cached global variables — no SD access needed
  if (tcpin_host.length() == 0 || tcpin_port <= 0) {
    _server->send(400, "text/plain", "No TCP-IN profile configured");
    return;
  }
  bool ok = startTcpIn(tcpin_host, tcpin_port);
  _server->send(200, "text/plain", String("TCP-IN ") + (ok ? "started" : "failed"));
}

static void handleLanInStop() {
  stopTcpIn();
  _server->send(200,"text/plain","TCP-IN stopped");
}

// ========================================================================
// BASE LLH HANDLER
// ========================================================================

static void handleBaseLLH() {
  if (!_server->hasArg("lat") || !_server->hasArg("lon") || !_server->hasArg("alt")) {
    _server->send(400,"text/plain","lat/lon/alt missing"); return;
  }
  double lat = _server->arg("lat").toDouble();
  double lon = _server->arg("lon").toDouble();
  double alt = _server->arg("alt").toDouble();
  uint16_t stid = 1;
  if (_server->hasArg("stid") && _server->arg("stid").length()>0) {
    int v=_server->arg("stid").toInt(); if (v>0 && v<4096) stid=(uint16_t)v;
  }
  uint8_t rtcmType = 0; // Default MSM7
  if (_server->hasArg("rtcm_type") && _server->arg("rtcm_type").length()>0) {
    int v=_server->arg("rtcm_type").toInt(); 
    if (v>=0 && v<=1) rtcmType=(uint8_t)v;
  }
  applyBaseFixedLLH(lat, lon, alt, stid, rtcmType);
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

// ========================================================================
// BASES CRUD HANDLERS
// ========================================================================

static void handleBasesAdd() {
  if(!_server->hasArg("name")||!_server->hasArg("lat")||!_server->hasArg("lon")||!_server->hasArg("alt")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  BaseRec b; b.name=clampSemi(_server->arg("name"));
  b.lat=_server->arg("lat").toDouble(); 
  b.lon=_server->arg("lon").toDouble(); 
  b.altGround=_server->arg("alt").toDouble();
  int st=_server->hasArg("stid")?_server->arg("stid").toInt():1; if (st<=0||st>=4096) st=1; b.stid=(uint16_t)st;
  b.hARP=_server->hasArg("harp")?_server->arg("harp").toFloat():0.0f;
  b.antennaIdx=_server->hasArg("antenna_idx")?_server->arg("antenna_idx").toInt():-1;
  int rt=_server->hasArg("rtcm_type")?_server->arg("rtcm_type").toInt():0; if (rt<0||rt>1) rt=0; b.rtcmType=(uint8_t)rt;
  std::vector<BaseRec> v; loadBases(v); v.push_back(b);
  std::sort(v.begin(), v.end(), [](const BaseRec&a,const BaseRec&b){return a.stid<b.stid;});
  if(!saveBases(v)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

static void handleBasesDel() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt();
  std::vector<BaseRec> v; loadBases(v);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v.erase(v.begin()+idx);
  if(!saveBases(v)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

static void handleBasesEdit() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); 
  std::vector<BaseRec> v; loadBases(v);
  std::vector<AntennaRec> antennas; loadAntennas(antennas);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  const auto& b=v[idx];
  
  sendHeader("Edit Base Station", "base");
  sendChunk("<div class='card'><h2>Edit Base Station</h2>");
  sendChunk("<form method='POST' action='/bases/update'>");
  sendChunk("<input type='hidden' name='idx' value='" + String(idx) + "'>");
  sendChunk("<label>Name:</label><input name='name' value='" + htmlEscape(b.name) + "' required><br>");
  sendChunk("<label>Latitude [deg]:</label><input name='lat' value='" + String(b.lat,8) + "' required><br>");
  sendChunk("<label>Longitude [deg]:</label><input name='lon' value='" + String(b.lon,8) + "' required><br>");
  sendChunk("<label>H ground [m]:</label><input name='alt' value='" + String(b.altGround,3) + "' required> (ellipsoidal)<br>");
  sendChunk("<label>Station ID [1..4095]:</label><input name='stid' type='number' value='" + String(b.stid) + "' style='width:150px'><br>");
  sendChunk("<h4>Antenna Setup</h4>");
  sendChunk("<label>H antenna ARP [m]:</label><input name='harp' type='number' step='0.001' value='" + String(b.hARP,3) + "' style='width:150px'> (ground to ARP)<br>");
  sendChunk("<label>Antenna model:</label><select name='antenna_idx'>");
  sendChunk("<option value='-1'" + String(b.antennaIdx == -1 ? " selected" : "") + ">None / Manual</option>");
  for (size_t i = 0; i < antennas.size(); i++) {
    sendChunk("<option value='" + String(i) + "'" + String(b.antennaIdx == (int)i ? " selected" : "") + ">" + 
              htmlEscape(antennas[i].name) + " (" + String(antennas[i].offset, 3) + "m)</option>");
  }
  sendChunk("</select><br>");
  sendChunk("<label>RTCM Messages:</label><select name='rtcm_type' style='width:350px'>");
  sendChunk("<option value='0'" + String(b.rtcmType == 0 ? " selected" : "") + ">MSM7 - 4 constellations (GPS, GLO, GAL, BDS) @1Hz</option>");
  sendChunk("<option value='1'" + String(b.rtcmType == 1 ? " selected" : "") + ">MSM4 - 3 constellations (GPS, GLO, GAL) @1Hz</option>");
  sendChunk("</select><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/base-cfg'>Cancel</a>");
  sendChunk("</form></div>");
  sendFooter();
}

static void handleBasesUpdate() {
  if(!_server->hasArg("idx")||!_server->hasArg("name")||!_server->hasArg("lat")||!_server->hasArg("lon")||!_server->hasArg("alt")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  int idx=_server->arg("idx").toInt(); std::vector<BaseRec> v; loadBases(v);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  BaseRec b; b.name=clampSemi(_server->arg("name"));
  b.lat=_server->arg("lat").toDouble(); 
  b.lon=_server->arg("lon").toDouble(); 
  b.altGround=_server->arg("alt").toDouble();
  int st=_server->hasArg("stid")?_server->arg("stid").toInt():1; if (st<=0||st>=4096) st=1; b.stid=(uint16_t)st;
  b.hARP=_server->hasArg("harp")?_server->arg("harp").toFloat():0.0f;
  b.antennaIdx=_server->hasArg("antenna_idx")?_server->arg("antenna_idx").toInt():-1;
  int rt=_server->hasArg("rtcm_type")?_server->arg("rtcm_type").toInt():0; if (rt<0||rt>1) rt=0; b.rtcmType=(uint8_t)rt;
  v[idx]=b;
  std::sort(v.begin(), v.end(), [](const BaseRec&a,const BaseRec&b){return a.stid<b.stid;});
  if(!saveBases(v)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

static void handleBasesStart() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); 
  std::vector<BaseRec> v; loadBases(v);
  std::vector<AntennaRec> antennas; loadAntennas(antennas);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  const auto& b=v[idx]; 
  
  // Calculate H_to_send = H_ground + H_ARP + Antenna_offset
  float antennaOffset = 0.0f;
  if (b.antennaIdx >= 0 && b.antennaIdx < (int)antennas.size()) {
    antennaOffset = antennas[b.antennaIdx].offset;
  }
  double h_to_send = b.altGround + b.hARP + antennaOffset;
  
  applyBaseFixedLLH(b.lat, b.lon, h_to_send, b.stid, b.rtcmType);
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

static void handleBasesStartConfirm() {
  if(!_server->hasArg("idx") || !_server->hasArg("harp") || !_server->hasArg("antenna")) {
    _server->send(400, "text/plain", "Missing parameters");
    return;
  }
  
  int idx = _server->arg("idx").toInt();
  float harp = _server->arg("harp").toFloat();
  int antennaIdx = _server->arg("antenna").toInt();
  
  std::vector<BaseRec> v;
  loadBases(v);
  std::vector<AntennaRec> antennas;
  loadAntennas(antennas);
  
  if (idx < 0 || idx >= (int)v.size()) {
    _server->send(400, "text/plain", "Index out of range");
    return;
  }
  
  const auto& b = v[idx];
  
  // Calculate H_to_send = H_ground + H_ARP + Antenna_offset
  float antennaOffset = 0.0f;
  if (antennaIdx >= 0 && antennaIdx < (int)antennas.size()) {
    antennaOffset = antennas[antennaIdx].offset;
  }
  double h_to_send = b.altGround + harp + antennaOffset;
  
  applyBaseFixedLLH(b.lat, b.lon, h_to_send, b.stid, b.rtcmType);
  
  String msg = "Base '" + b.name + "' started successfully!\n";
  msg += "Position sent to ZED-F9P:\n";
  msg += "Lat: " + String(b.lat, 8) + "°\n";
  msg += "Lon: " + String(b.lon, 8) + "°\n";
  msg += "Height: " + String(h_to_send, 3) + " m\n";
  msg += "RTCM Type: " + String(b.rtcmType == 0 ? "MSM7 (4 const)" : "MSM4 (3 const)");
  
  _server->send(200, "text/plain", msg);
}

// ========================================================================
// BASE OUT CRUD HANDLERS
// ========================================================================

static void handleBaseOutAdd() {
  if(!_server->hasArg("name")||!_server->hasArg("host")||!_server->hasArg("port")||!_server->hasArg("mount")||!_server->hasArg("tcp")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  NtripOut n;
  n.name=clampSemi(_server->arg("name"));
  n.host=_server->arg("host");
  n.port=_server->arg("port").toInt();
  n.mount=_server->arg("mount");
  n.pass=_server->arg("pass");
  n.tcpPort=_server->arg("tcp").toInt();
  std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  v.push_back(n); if(last<0) last=0;
  if(!saveNtripOutList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

static void handleBaseOutDel() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v.erase(v.begin()+idx);
  if (last >= (int)v.size()) last = (int)v.size()-1;
  if(!saveNtripOutList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

static void handleBaseOutEdit() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  const auto& n=v[idx];
  
  sendHeader("Edit Output Profile", "base");
  sendChunk("<div class='card'><h2>Edit Output Profile</h2>");
  sendChunk("<form method='POST' action='/baseout/update'>");
  sendChunk("<input type='hidden' name='idx' value='" + String(idx) + "'>");
  sendChunk("<label>Name:</label><input name='name' value='" + htmlEscape(n.name) + "' required><br>");
  sendChunk("<label>Host:</label><input name='host' value='" + htmlEscape(n.host) + "' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='" + String(n.port) + "' required><br>");
  sendChunk("<label>Mountpoint:</label><input name='mount' value='" + htmlEscape(n.mount) + "' required><br>");
  sendChunk("<label>Password:</label><input name='pass' type='password' value='" + htmlEscape(n.pass) + "'><br>");
  sendChunk("<label>TCP Server Port:</label><input name='tcp' type='number' value='" + String(n.tcpPort) + "' required><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/base-cfg'>Cancel</a>");
  sendChunk("</form></div>");
  sendFooter();
}

static void handleBaseOutUpdate() {
  if(!_server->hasArg("idx")||!_server->hasArg("name")||!_server->hasArg("host")||!_server->hasArg("port")||!_server->hasArg("mount")||!_server->hasArg("tcp")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  int idx=_server->arg("idx").toInt(); std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v[idx].name=clampSemi(_server->arg("name"));
  v[idx].host=_server->arg("host");
  v[idx].port=_server->arg("port").toInt();
  v[idx].mount=_server->arg("mount");
  v[idx].pass=_server->arg("pass");
  v[idx].tcpPort=_server->arg("tcp").toInt();
  if(!saveNtripOutList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

static void handleBaseOutSelect() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  last=idx;
  if(!saveNtripOutList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base-cfg"); _server->send(303);
}

static void handleBaseOutStart() {
  std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  int id = _server->hasArg("id") ? _server->arg("id").toInt() : last;
  if(id<0 || id>=(int)v.size()){ _server->send(400,"text/plain","No valid profile selected"); return; }
  const auto& n=v[id];
  bool okCaster = startCasterOut(n.host, (uint16_t)n.port, n.mount, n.pass);
  bool okTcp    = startTcpOut((uint16_t)n.tcpPort);
  // No need to save - LAST was already saved by handleBaseOutSelect, and profile data hasn't changed
  _server->send(200, "text/plain", String("Caster: ")+(okCaster?"OK":"FAIL")+" | TCP: "+(okTcp?"OK":"FAIL"));
}

static void handleBaseOutStop() {
  stopCasterOut(); stopTcpOut();
  _server->send(200, "text/plain", "OUT stopped");
}

// ========================================================================
// TCP OUT CLIENT HANDLERS
// ========================================================================

static void handleTcpClientAdd() {
  TcpOutClient t;
  t.name = _server->arg("name");
  t.host = _server->arg("host");
  t.port = _server->arg("port").toInt();
  
  std::vector<TcpOutClient> v; int last=-1; 
  loadTcpOutClientList(v,last);
  v.push_back(t);
  if(!saveTcpOutClientList(v,last)){
    _server->send(500,"text/plain","Save failed"); 
    return; 
  }
  _server->sendHeader("Location","/base-cfg"); 
  _server->send(303);
}

static void handleTcpClientDel() {
  int idx=_server->arg("idx").toInt(); 
  std::vector<TcpOutClient> v; int last=-1; 
  loadTcpOutClientList(v,last);
  if(idx>=0 && idx<(int)v.size()){ 
    v.erase(v.begin()+idx); 
    if(last==idx) last=-1; else if(last>idx) last--;
  }
  if(!saveTcpOutClientList(v,last)){
    _server->send(500,"text/plain","Save failed"); 
    return; 
  }
  _server->sendHeader("Location","/base-cfg"); 
  _server->send(303);
}

static void handleTcpClientEditForm() {
  int idx=_server->arg("idx").toInt(); 
  std::vector<TcpOutClient> v; int last=-1; 
  loadTcpOutClientList(v,last);
  if(idx<0 || idx>=(int)v.size()){ 
    _server->send(404,"text/plain","Not found"); 
    return; 
  }
  
  const auto& t=v[idx];
  sendHeader("Edit TCP Client Profile","base");
  sendChunk("<div class='card'><h2>Edit TCP Client Profile</h2>");
  sendChunk("<form method='POST' action='/tcpclient/edit?idx="+String(idx)+"'>");
  sendChunk("<label>Name:</label><input name='name' value='"+htmlEscape(t.name)+"' required><br>");
  sendChunk("<label>Host:</label><input name='host' value='"+htmlEscape(t.host)+"' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='"+String(t.port)+"' required><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/base-cfg'>Cancel</a>");
  sendChunk("</form></div>");
  sendFooter();
}

static void handleTcpClientEditSave() {
  int idx=_server->arg("idx").toInt(); 
  std::vector<TcpOutClient> v; int last=-1; 
  loadTcpOutClientList(v,last);
  if(idx<0 || idx>=(int)v.size()){ 
    _server->send(404,"text/plain","Not found"); 
    return; 
  }
  
  v[idx].name = _server->arg("name");
  v[idx].host = _server->arg("host");
  v[idx].port = _server->arg("port").toInt();
  
  if(!saveTcpOutClientList(v,last)){
    _server->send(500,"text/plain","Save failed"); 
    return; 
  }
  _server->sendHeader("Location","/base-cfg"); 
  _server->send(303);
}

static void handleTcpClientSelect() {
  int idx=_server->arg("idx").toInt(); 
  std::vector<TcpOutClient> v; int last=-1; 
  loadTcpOutClientList(v,last);
  if(idx<0 || idx>=(int)v.size()){ 
    _server->send(404,"text/plain","Not found"); 
    return; 
  }
  
  last = idx;
  if(!saveTcpOutClientList(v,last)){
    _server->send(500,"text/plain","Save failed"); 
    return; 
  }
  _server->sendHeader("Location","/base-cfg"); 
  _server->send(303);
}

static void handleTcpClientStart() {
  int id = _server->arg("id").toInt();
  std::vector<TcpOutClient> v; int last=-1; 
  loadTcpOutClientList(v,last);
  
  if(id<0 || id>=(int)v.size()){ 
    _server->send(404,"text/plain","Profile not found"); 
    return; 
  }
  
  const auto& t = v[id];
  bool ok = startTcpOutClient(t.host, (uint16_t)t.port);
  
  // No need to save - LAST was already saved by handleTcpClientSelect, and profile data hasn't changed
  
  if(ok) _server->send(200,"text/plain","TCP Client started");
  else _server->send(500,"text/plain","Failed to start TCP Client");
}

static void handleTcpClientStop() {
  stopTcpOutClient();
  _server->send(200,"text/plain","TCP Client stopped");
}


// ========================================================================
// RATE, STREAM, DOWNLOAD, DELETE HANDLERS
// ========================================================================

static void handleRateSubmit() {
  if (_server->hasArg("rate")) {
    uint16_t rateMs = _server->arg("rate").toInt();
    sendCfgRateToZED(rateMs);
    _server->sendHeader("Location", "/settings");
    _server->send(303);
  } else {
    _server->send(400, "text/plain", "Parameter rate missing");
  }
}

static void handleDownload() {
  if (!_server->hasArg("file")) { _server->send(400,"text/plain","missing file"); return; }
  if (!isValidGnssPath(_server->arg("file"))) { _server->send(403,"text/plain","Invalid path"); return; }
  if (loggingActive) { _server->send(409, "text/plain", "Download not allowed during RAW log active"); return; }

  SdLockGuard guard(5000);
  if (!guard.locked) { _server->send(503,"text/plain","SD busy"); return; }

  String filename = "/gnss/" + _server->arg("file");

  FsFile file = _sd->open(filename.c_str(), FILE_READ);
  if (!file || file.isDirectory()) {
    _server->send(404,"text/plain","File not found");
    return;
  }

  _server->sendHeader("Content-Type", "application/octet-stream");
  _server->sendHeader("Content-Disposition", "attachment; filename=\"" + _server->arg("file") + "\"");
  _server->sendHeader("Connection", "close");
  _server->setContentLength(file.size());
  _server->send(200);

  WiFiClient c = _server->client();
  c.setNoDelay(true);

  static uint8_t buf[8192];
  while (file.available()) {
    size_t n = file.read(buf, sizeof(buf));
    if (!n) break;
    size_t off = 0;
    while (off < n) {
      size_t w = c.write(buf + off, n - off);
      if (w == 0) { file.close(); return; }
      off += w;
    }
  }
  file.close();
}

static void handleDelete() {
  if (!_server->hasArg("file")) { _server->send(400,"text/plain","missing file"); return; }
  if (!isValidGnssPath(_server->arg("file"))) { _server->send(403,"text/plain","Invalid path"); return; }
  if (loggingActive) { _server->send(409, "text/plain", "Delete not allowed during RAW log active"); return; }

  SdLockGuard guard(3000);
  if (!guard.locked) { _server->send(503,"text/plain","SD busy"); return; }

  String filename = "/gnss/" + _server->arg("file");
  _sd->remove(filename.c_str());

  // Redirect back to /logs if coming from logs page, otherwise to home
  String referer = _server->header("Referer");
  String redirect = referer.indexOf("/logs") >= 0 ? "/logs" : "/";
  
  _server->sendHeader("Location", redirect);
  _server->send(303);
}

// ========================================================================
// CONFIG EXPORT/IMPORT HELPERS
// ========================================================================

// Helper: Unescape JSON string (handles \" and \\)
static String unescapeJson(const String& str) {
  String result = "";
  result.reserve(str.length());
  
  for (unsigned int i = 0; i < str.length(); i++) {
    if (str.charAt(i) == '\\' && i + 1 < str.length()) {
      char next = str.charAt(i + 1);
      if (next == '"') {
        result += '"';
        i++; // skip next char
      } else if (next == '\\') {
        result += '\\';
        i++; // skip next char
      } else if (next == '/') {
        result += '/';
        i++; // skip next char
      } else if (next == 'b') {
        result += '\b';
        i++; // skip next char
      } else if (next == 'f') {
        result += '\f';
        i++; // skip next char
      } else if (next == 'n') {
        result += '\n';
        i++; // skip next char
      } else if (next == 'r') {
        result += '\r';
        i++; // skip next char
      } else if (next == 't') {
        result += '\t';
        i++; // skip next char
      } else {
        result += str.charAt(i);
      }
    } else {
      result += str.charAt(i);
    }
  }
  
  return result;
}

// Helper: Find the end quote of a JSON string, handling escape sequences
static int findJsonStringEnd(const String& str, int startPos) {
  int pos = startPos;
  while (pos < (int)str.length()) {
    if (str.charAt(pos) == '\\') {
      // Skip the next character (it's escaped)
      pos += 2;
    } else if (str.charAt(pos) == '"') {
      return pos;
    } else {
      pos++;
    }
  }
  return -1; // Not found
}

// Helper: Read a config file with LAST= header and convert to JSON
static String readConfigFileAsJson(const char* path) {
  String result = "{\"lastIdx\":-1,\"entries\":[";
  String content = FlashConfig::readFile(path);
  if (content.length() == 0) return "{\"lastIdx\":-1,\"entries\":[]}";

  bool firstEntry = true;
  int lastIdx = -1;
  int start = 0;

  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (line.length() == 0) continue;

    if (line.startsWith("#")) {
      int p = line.indexOf("LAST=");
      if (p >= 0) {
        lastIdx = line.substring(p + 5).toInt();
      }
      continue;
    }

    if (!firstEntry) result += ",";
    firstEntry = false;
    result += "\"";
    result += jsonEscape(line);
    result += "\"";
  }

  result += "]}";

  String finalResult = "{\"lastIdx\":";
  finalResult += String(lastIdx);
  finalResult += ",\"entries\":[";
  int bracketPos = result.indexOf('[');
  if (bracketPos >= 0 && bracketPos < (int)result.length() - 1) {
    finalResult += result.substring(bracketPos + 1);
  } else {
    finalResult += "]}";
  }

  return finalResult;
}

// Helper: Read WiFi file (no LAST= header, format: priority;ssid;password)
static String readWifiAsJson(const char* path) {
  String result = "{\"lastIdx\":-1,\"entries\":[";
  String content = FlashConfig::readFile(path);
  if (content.length() == 0) return "{\"lastIdx\":-1,\"entries\":[]}";

  bool firstEntry = true;
  int start = 0;

  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (line.length() == 0 || line.startsWith("#")) continue;

    if (!firstEntry) result += ",";
    firstEntry = false;
    result += "\"";
    result += jsonEscape(line);
    result += "\"";
  }

  result += "]}";
  return result;
}

// Helper: Read NTP file (simple single line)
static String readNtpAsJson(const char* path) {
  String content = FlashConfig::readFile(path);
  content.trim();
  return "\"" + jsonEscape(content) + "\"";
}

// Handler: Sync flash→SD on demand
static void handleConfigSync() {
  if (loggingActive) {
    _server->send(409, "text/plain", "Cannot sync while logging is active");
    return;
  }
  if (!sdOK) {
    _server->send(503, "text/plain", "SD not available");
    return;
  }
  FlashConfig::syncToSD(sd, sdMutex, true);
  _server->send(200, "text/plain", "OK");
}

// Handler: Factory Reset
static void handleFactoryReset() {
  bool wipeSD = _server->hasArg("wipeSD") && _server->arg("wipeSD") == "1";

  if (loggingActive) {
    _server->send(409, "application/json", "{\"error\":\"Cannot reset while logging is active\"}");
    return;
  }

  bool ok = FlashConfig::factoryReset(sd, sdMutex, wipeSD);
  if (ok) {
    _server->send(200, "application/json", "{\"status\":\"ok\"}");
    delay(500);
    ESP.restart();
  } else {
    _server->send(500, "application/json", "{\"error\":\"LittleFS format failed\"}");
  }
}

// Handler: Export all configuration as JSON
static void handleConfigExport() {
  String json = "{";
  
  // WiFi profiles
  json += "\"wifi\":";
  json += readWifiAsJson("/config/wifi.txt");
  json += ",";
  
  // NTRIP IN
  json += "\"ntrip_in\":";
  json += readConfigFileAsJson("/config/ntrip_in_list.txt");
  json += ",";
  
  // NTRIP OUT  
  json += "\"ntrip_out\":";
  json += readConfigFileAsJson("/config/ntrip_out_list.txt");
  // TCP OUT CLIENT
  json += ",\"tcp_out_client\":";
  json += readConfigFileAsJson("/config/tcp_out_client_list.txt");
  json += ",";
  
  // TCP IN
  json += "\"tcp_in\":";
  json += readConfigFileAsJson("/config/tcp_in_lista.txt");
  json += ",";
  
  // Bases
  json += "\"bases\":";
  json += readConfigFileAsJson("/config/bases.txt");
  json += ",";
  
  // NTP
  json += "\"ntp\":";
  json += readNtpAsJson("/config/ntp.txt");
  
  json += "}";
  
  _server->sendHeader("Content-Disposition", "attachment; filename=rtkino_config.json");
  _server->send(200, "application/json", json);
}

// Helper: Import a config section from JSON and write to flash
static bool importConfigSection(const String& json, const char* key, const char* path) {
  String searchKey = "\"" + String(key) + "\"";
  int start = json.indexOf(searchKey);
  if (start < 0) return false;

  int sectionStart = json.indexOf("{", start);
  if (sectionStart < 0) return false;

  const char* lastIdxKey = "\"lastIdx\":";
  int lastIdxStart = json.indexOf(lastIdxKey, sectionStart);
  int lastIdx = -1;
  int entriesPos = json.indexOf("\"entries\":", sectionStart);
  if (lastIdxStart >= 0 && (entriesPos < 0 || lastIdxStart < entriesPos)) {
    int numStart = lastIdxStart + strlen(lastIdxKey);
    int numEnd = json.indexOf(",", numStart);
    if (numEnd < 0) numEnd = json.indexOf("}", numStart);
    lastIdx = json.substring(numStart, numEnd).toInt();
  }

  const char* entriesKey = "\"entries\":[";
  int entriesStart = json.indexOf(entriesKey, start);
  if (entriesStart < 0) return false;
  entriesStart += strlen(entriesKey);

  int entriesEnd = json.indexOf("]", entriesStart);
  if (entriesEnd < 0) return false;

  String entriesStr = json.substring(entriesStart, entriesEnd);

  String content;
  if (lastIdx >= 0) {
    content += String("# LAST=") + lastIdx + "\n";
  }

  int pos = 0;
  while (pos < (int)entriesStr.length()) {
    int quoteStart = entriesStr.indexOf('"', pos);
    if (quoteStart < 0) break;
    int quoteEnd = findJsonStringEnd(entriesStr, quoteStart + 1);
    if (quoteEnd < 0) break;
    String entry = entriesStr.substring(quoteStart + 1, quoteEnd);
    entry = unescapeJson(entry);
    content += entry + "\n";
    pos = quoteEnd + 1;
  }

  return FlashConfig::writeFile(path, content);
}

// Helper: Import WiFi from JSON (no LAST= header)
static bool importWifiFromJson(const String& json) {
  String searchKey = "\"wifi\"";
  int start = json.indexOf(searchKey);
  if (start < 0) return false;

  const char* entriesKey = "\"entries\":[";
  int entriesStart = json.indexOf(entriesKey, start);
  if (entriesStart < 0) return false;
  entriesStart += strlen(entriesKey);

  int entriesEnd = json.indexOf("]", entriesStart);
  if (entriesEnd < 0) return false;

  String entriesStr = json.substring(entriesStart, entriesEnd);

  String content;
  int pos = 0;
  while (pos < (int)entriesStr.length()) {
    int quoteStart = entriesStr.indexOf('"', pos);
    if (quoteStart < 0) break;
    int quoteEnd = findJsonStringEnd(entriesStr, quoteStart + 1);
    if (quoteEnd < 0) break;
    String entry = entriesStr.substring(quoteStart + 1, quoteEnd);
    entry = unescapeJson(entry);
    content += entry + "\n";
    pos = quoteEnd + 1;
  }

  return FlashConfig::writeFile("/config/wifi.txt", content);
}

static bool importNtripInFromJson(const String& json) {
  return importConfigSection(json, "ntrip_in", "/config/ntrip_in_list.txt");
}

static bool importNtripOutFromJson(const String& json) {
  return importConfigSection(json, "ntrip_out", "/config/ntrip_out_list.txt");
}

static bool importTcpOutClientFromJson(const String& json) {
  return importConfigSection(json, "tcp_out_client", "/config/tcp_out_client_list.txt");
}
static bool importTcpInFromJson(const String& json) {
  return importConfigSection(json, "tcp_in", "/config/tcp_in_lista.txt");
}

static bool importBasesFromJson(const String& json) {
  return importConfigSection(json, "bases", "/config/bases.txt");
}

// Helper: Import NTP from JSON (simple string value)
static bool importNtpFromJson(const String& json) {
  const char* searchKey = "\"ntp\":\"";
  int start = json.indexOf(searchKey);
  if (start < 0) return false;
  start += strlen(searchKey);

  int end = findJsonStringEnd(json, start);
  if (end < 0) return false;

  String ntp = json.substring(start, end);
  ntp = unescapeJson(ntp);

  if (ntp.length() == 0) return true;

  return FlashConfig::writeFile("/config/ntp.txt", ntp + "\n");
}

// Handler: Import configuration from JSON
static void handleConfigImport() {
  if (!_server->hasArg("plain")) {
    _server->send(400, "application/json", "{\"error\":\"No data\"}");
    return;
  }
  
  String body = _server->arg("plain");
  
  bool success = true;
  String errors = "";
  
  // Import WiFi
  if (body.indexOf("\"wifi\"") >= 0) {
    if (!importWifiFromJson(body)) {
      success = false;
      errors += "wifi,";
    }
  }
  
  // Import NTRIP IN
  if (body.indexOf("\"ntrip_in\"") >= 0) {
    if (!importNtripInFromJson(body)) {
      success = false;
      errors += "ntrip_in,";
    }
  }
  
  // Import NTRIP OUT
  if (body.indexOf("\"ntrip_out\"") >= 0) {
    if (!importNtripOutFromJson(body)) {
      success = false;
      errors += "ntrip_out,";
    }
  }
  
  // Import TCP OUT CLIENT
  if (body.indexOf("\"tcp_out_client\"") >= 0) {
    if (!importTcpOutClientFromJson(body)) {
      success = false;
      errors += "tcp_out_client,";
    }
  }

  // Import TCP IN
  if (body.indexOf("\"tcp_in\"") >= 0) {
    if (!importTcpInFromJson(body)) {
      success = false;
      errors += "tcp_in,";
    }
  }
  
  // Import Bases
  if (body.indexOf("\"bases\"") >= 0) {
    if (!importBasesFromJson(body)) {
      success = false;
      errors += "bases,";
    }
  }
  
  // Import NTP
  if (body.indexOf("\"ntp\"") >= 0) {
    if (!importNtpFromJson(body)) {
      success = false;
      errors += "ntp,";
    }
  }
  
  if (success) {
    _server->send(200, "application/json", "{\"success\":true}");
  } else {
    _server->send(200, "application/json", "{\"success\":false,\"errors\":\"" + errors + "\"}");
  }
}

// ========================================================================
// AUDIO UPLOAD HANDLERS
// ========================================================================

// Shared state for audio upload
static String g_audioUploadContent;
static bool g_audioUploadError;
static String g_audioUploadErrorMsg;

static void handleAudioUpload() {
  HTTPUpload& upload = _server->upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    g_audioUploadContent = "";
    g_audioUploadError = false;
    g_audioUploadErrorMsg = "";
    Serial.printf("[AUDIO] Upload started: %s\n", upload.filename.c_str());
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    // Accumulate content (max 2KB for safety)
    if (g_audioUploadContent.length() + upload.currentSize <= MAX_MELODY_UPLOAD_SIZE) {
      for (size_t i = 0; i < upload.currentSize; i++) {
        g_audioUploadContent += (char)upload.buf[i];
      }
    } else if (!g_audioUploadError) {
      g_audioUploadError = true;
      g_audioUploadErrorMsg = "File too large (max 2KB)";
      Serial.println("[AUDIO] " + g_audioUploadErrorMsg);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    Serial.printf("[AUDIO] Upload complete: %d bytes\n", g_audioUploadContent.length());
    
    if (g_audioUploadError) {
      // Error already set during upload
      return;
    }
    
    if (g_audioUploadContent.length() == 0) {
      g_audioUploadError = true;
      g_audioUploadErrorMsg = "Empty file";
      return;
    }
    
    // Save to SD
    bool locked = sdLock(3000);
    if (!locked) {
      g_audioUploadError = true;
      g_audioUploadErrorMsg = "SD card busy";
      Serial.println("[AUDIO] SD card lock timeout");
      return;
    }
    
    FsFile f = _sd->open("/gnss/buzzer_melody.json", O_WRITE | O_CREAT | O_TRUNC);
    if (!f) {
      g_audioUploadError = true;
      g_audioUploadErrorMsg = "Failed to open file";
      Serial.println("[AUDIO] Failed to open file for writing");
      sdUnlock(locked);
      return;
    }
    
    f.print(g_audioUploadContent);
    f.close();
    sdUnlock(locked);
    
    Serial.println("[AUDIO] File saved successfully");
    
    // Reload in buzzer if available
    if (g_buzzer) {
      g_buzzer->loadCustomMelody("/gnss/buzzer_melody.json");
    }
  }
}

static void handleAudioUploadComplete() {
  if (g_audioUploadError) {
    sendHeader("Upload Failed", "settings");
    sendChunk("<div class='card'>");
    sendChunk("<h2>Upload Failed</h2>");
    sendChunk("<p style='color:#e74c3c'><strong>Error:</strong> ");
    sendChunk(htmlEscape(g_audioUploadErrorMsg));
    sendChunk("</p>");
    sendChunk("<p><a href='/settings' class='btn'>Back to Settings</a></p>");
    sendChunk("</div>");
    sendFooter();
  } else {
    _server->sendHeader("Location", "/settings");
    _server->send(303);
  }
}

// ========================================================================
// FIRMWARE OTA HANDLERS
// ========================================================================

static void handleFirmwareUpload() {
  HTTPUpload& upload = _server->upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("[OTA] Firmware upload started: %s\n", upload.filename.c_str());
    Serial.printf("[OTA] File size: %u bytes\n", upload.totalSize);
    
    // Begin OTA update
    if (!OTAManager::beginWebUpdate(upload.totalSize)) {
      Serial.println("[OTA] Failed to begin update");
      return;
    }
    
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    // Write firmware chunk
    if (!OTAManager::writeWebUpdate(upload.buf, upload.currentSize)) {
      Serial.println("[OTA] Failed to write firmware chunk");
      return;
    }
    
  } else if (upload.status == UPLOAD_FILE_END) {
    Serial.printf("[OTA] Firmware upload complete: %u bytes\n", upload.totalSize);
    
    // Finalize update
    if (!OTAManager::endWebUpdate()) {
      Serial.println("[OTA] Failed to finalize update");
      return;
    }
    
    Serial.println("[OTA] Update successful! Rebooting...");
  }
}

static void handleFirmwareUploadComplete() {
  // Check if update was successful
  String status = OTAManager::getLastUpdateStatus();
  
  if (status.startsWith("Success")) {
    _server->send(200, "text/plain", "Firmware updated successfully! Rebooting...");
    delay(1000);
    OTAManager::reboot();
  } else {
    _server->send(500, "text/plain", "Firmware update failed: " + status);
  }
}


// ========================================================================
// NEW API HANDLERS - TMODE REFRESH, BASE STOP, ZED RESET
// ========================================================================

static void handleZedTmodeRefresh() {
  readZedTmode();
  
  ZedTmodeState tmode;
  if (getZedTmode(tmode)) {
    String json = "{\"success\":true,\"mode\":" + String(tmode.mode);
    json += ",\"lat\":" + String(tmode.lat, 8);
    json += ",\"lon\":" + String(tmode.lon, 8);
    json += ",\"height\":" + String(tmode.height, 3);
    json += "}";
    _server->send(200, "application/json", json);
  } else {
    _server->send(500, "application/json", "{\"success\":false}");
  }
}

static void handleBaseStop() {
  stopBaseMode();
  _server->send(200, "text/plain", "Base mode stopped. TMODE disabled, returned to Rover mode.");
}

static void handleSwitchToRover() {
  switchToRover();
  _server->send(200, "text/plain", "Switched to Rover mode. NTRIP IN re-enabled.");
}

static void handleZedReset() {
  String type = _server->hasArg("type") ? _server->arg("type") : "hot";
  bool cold = (type == "cold");
  
  bool ok = UbxVal::resetZed(cold);
  
  if (ok) {
    _server->send(200, "text/plain", 
      cold ? "ZED-F9P Cold Reset sent. Fix will take ~30s." 
           : "ZED-F9P Hot Reset sent. Restarting...");
  } else {
    _server->send(500, "text/plain", "Reset command failed");
  }
}

// ========================================================================
// SURVEY POINTS PAGE AND API HANDLERS
// ========================================================================

// ---- Survey API: quality check ----
static void handlePtsQuality() {
  QualityWarning w = SurveyPoints::checkQuality();
  String json = "{";
  json += "\"ok\":" + String(w.anyBad() ? "false" : "true") + ",";
  json += "\"warnings\":[";
  bool first = true;
  auto addW = [&](const String& msg) {
    if (!first) json += ",";
    json += "\"" + msg + "\"";
    first = false;
  };
  if (w.stale)   addW("No fix 3D");
  if (w.hAccBad) addW("Precisione orizzontale elevata");
  if (w.pdopBad) addW("PDOP elevato");
  if (w.svBad)   addW("Pochi satelliti");
  if (w.noRtk)   addW("No RTK fix");
  json += "],";
  json += "\"details\":\"" + w.details + "\"";
  json += "}";
  _server->send(200, "application/json", json);
}

static void handleSurveyPage() {
  sendHeader("Survey", "survey");

  // ---- Inline styles for quality modal ----
  sendChunk("<style>");
  sendChunk(".modal-overlay{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.5);z-index:1000;justify-content:center;align-items:center;}");
  sendChunk(".modal-overlay.active{display:flex;}");
  sendChunk(".modal-box{background:white;border-radius:8px;padding:24px;max-width:420px;width:90%;box-shadow:0 4px 20px rgba(0,0,0,0.3);}");
  sendChunk(".modal-box h3{margin-top:0;color:#e67e22;}");
  sendChunk(".modal-box ul{margin:12px 0;padding-left:20px;color:#c0392b;}");
  sendChunk(".modal-actions{display:flex;gap:10px;margin-top:16px;justify-content:flex-end;}");
  sendChunk(".btn-force{background:#e67e22;color:white;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;}");
  sendChunk(".btn-cancel-modal{background:#95a5a6;color:white;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;}");
  sendChunk(".prog-bar{height:8px;background:#ecf0f1;border-radius:4px;margin:8px 0;}");
  sendChunk(".prog-fill{height:100%;background:#2ecc71;border-radius:4px;transition:width 0.3s;}");
  sendChunk("</style>");

  // ---- Quality warning modal ----
  sendChunk("<div id='quality-modal' class='modal-overlay'>");
  sendChunk("<div class='modal-box'>");
  sendChunk("<h3>&#9888; Warning &mdash; GNSS Quality</h3>");
  sendChunk("<p>The GNSS signal quality does not meet standard criteria:</p>");
  sendChunk("<ul id='quality-warnings-list'></ul>");
  sendChunk("<div class='modal-actions'>");
  sendChunk("<button class='btn-cancel-modal' onclick='closeQualityModal()'>&#x1F6AB; Cancel</button>");
  sendChunk("<button class='btn-force' onclick='forceMeasure()'>&#9888; Measure anyway</button>");
  sendChunk("</div></div></div>");

  // ---- JavaScript ----
  sendChunk("<script>");

  // Robust polling with setTimeout recursion and 120s global timeout
  sendChunk("var _measureTimeout=null;var _measureDeadline=0;");
  sendChunk("function startMeasurePoll(){");
  sendChunk("  _measureDeadline=Date.now()+120000;");
  sendChunk("  (function poll(){");
  sendChunk("    fetch('/api/pts/measure/status').then(function(r){return r.json();}).then(function(d){");
  sendChunk("      var bar=document.getElementById('meas-bar');");
  sendChunk("      var st=document.getElementById('meas-status');");
  sendChunk("      if(bar)bar.style.width=d.pct+'%';");
  sendChunk("      if(st)st.innerHTML=d.pct+'% | '+d.nSamples+' camp. | hAcc:'+d.curHAcc.toFixed(4)+'m | '+d.elapsed.toFixed(1)+'s';");
  sendChunk("      if(d.status==='done'||d.status==='error'){");
  sendChunk("        var res=document.getElementById('meas-result');");
  sendChunk("        if(res){res.style.display='';");
  sendChunk("          res.innerHTML=d.status==='done'?'<b style=color:green>&#10003; Saved: '+d.lastPointId+'</b>':'<span style=color:red>&#9888; Error: '+d.errorMsg+'</span>';}");
  sendChunk("        document.getElementById('btn-misura').disabled=false;");
  sendChunk("        if(d.status==='done')setTimeout(function(){location.reload();},2000);");
  sendChunk("      } else if(Date.now()<_measureDeadline){");
  sendChunk("        _measureTimeout=setTimeout(poll,500);");
  sendChunk("      } else {");
  sendChunk("        var res=document.getElementById('meas-result');");
  sendChunk("        if(res){res.style.display='';res.innerHTML='<span style=color:red>Measure timeout (120s)</span>';}");
  sendChunk("        document.getElementById('btn-misura').disabled=false;");
  sendChunk("      }");
  sendChunk("    }).catch(function(){");
  sendChunk("      if(Date.now()<_measureDeadline)_measureTimeout=setTimeout(poll,1000);");
  sendChunk("    });");
  sendChunk("  })();");
  sendChunk("}");

  // doMeasure: check quality first, then show modal or proceed
  sendChunk("var _pendingForce=false;");
  sendChunk("function doMeasure(){");
  sendChunk("  _pendingForce=false;");
  sendChunk("  fetch('/api/pts/quality').then(function(r){return r.json();}).then(function(q){");
  sendChunk("    if(!q.ok){");
  sendChunk("      var ul=document.getElementById('quality-warnings-list');");
  sendChunk("      ul.innerHTML='';");
  sendChunk("      (q.warnings||[]).forEach(function(w){var li=document.createElement('li');li.textContent=w;ul.appendChild(li);});");
  sendChunk("      if(q.details){var li=document.createElement('li');li.style.fontSize='0.85em';li.style.color='#7f8c8d';li.textContent=q.details;ul.appendChild(li);}");
  sendChunk("      document.getElementById('quality-modal').classList.add('active');");
  sendChunk("    } else { startMeasureRequest(false); }");
  sendChunk("  }).catch(function(){startMeasureRequest(false);});");
  sendChunk("}");

  sendChunk("function closeQualityModal(){document.getElementById('quality-modal').classList.remove('active');}");
  sendChunk("function forceMeasure(){closeQualityModal();startMeasureRequest(true);}");

  // Cascade dropdown support
  sendChunk("var _codesData=null;");
  sendChunk("function loadCodes(){");
  sendChunk("  fetch('/api/codes').then(function(r){return r.json();}).then(function(d){");
  sendChunk("    _codesData=d;");
  sendChunk("    var cat=document.getElementById('pt-cat');");
  sendChunk("    if(!cat)return;");
  sendChunk("    cat.innerHTML='<option value=\"\">-- choose --</option>';");
  sendChunk("    d.categorie.forEach(function(c,i){");
  sendChunk("      var o=document.createElement('option');");
  sendChunk("      o.value=i;o.text=c.label;");
  sendChunk("      cat.appendChild(o);");
  sendChunk("    });");
  sendChunk("  }).catch(function(){});");
  sendChunk("}");
  sendChunk("function updateCodici(catIdx){");
  sendChunk("  var sel=document.getElementById('pt-codice');");
  sendChunk("  sel.innerHTML='<option value=\"\">-- choose --</option>';");
  sendChunk("  if(!_codesData||catIdx==='')return;");
  sendChunk("  var cat=_codesData.categorie[parseInt(catIdx)];");
  sendChunk("  if(!cat)return;");
  sendChunk("  cat.codici.forEach(function(c){");
  sendChunk("    var o=document.createElement('option');");
  sendChunk("    o.value=c.cod;o.text=c.cod+' \u2014 '+c.label;");
  sendChunk("    sel.appendChild(o);");
  sendChunk("  });");
  sendChunk("}");
  sendChunk("loadCodes();");

  sendChunk("function startMeasureRequest(force){");
  sendChunk("  var name=document.getElementById('pt-name').value;");
  sendChunk("  var codice=document.getElementById('pt-codice').value;");
  sendChunk("  var dur=document.getElementById('pt-dur').value||10;");
  sendChunk("  document.getElementById('btn-misura').disabled=true;");
  sendChunk("  var st=document.getElementById('meas-status');if(st)st.innerHTML='Avvio...';");
  sendChunk("  var res=document.getElementById('meas-result');if(res)res.style.display='none';");
  sendChunk("  var bar=document.getElementById('meas-bar');if(bar)bar.style.width='0%';");
  sendChunk("  fetch('/api/pts/measure?name='+encodeURIComponent(name)+'&codice='+encodeURIComponent(codice)+'&duration='+dur+'&force='+(force?1:0))");
  sendChunk("    .then(function(r){return r.json();}).then(function(d){");
  sendChunk("      if(d.error){");
  sendChunk("        var res=document.getElementById('meas-result');");
  sendChunk("        if(res){res.style.display='';res.innerHTML='<span style=color:red>&#9888; '+d.error+'</span>';}");
  sendChunk("        document.getElementById('btn-misura').disabled=false;");
  sendChunk("      } else { startMeasurePoll(); }");
  sendChunk("    }).catch(function(e){");
  sendChunk("      var res=document.getElementById('meas-result');");
  sendChunk("      if(res){res.style.display='';res.innerHTML='<span style=color:red>Network error: '+e+'</span>';}");
  sendChunk("      document.getElementById('btn-misura').disabled=false;");
  sendChunk("    });");
  sendChunk("}");

  sendChunk("function setActive(sid){");
  sendChunk("  fetch('/api/pts/setactive?sid='+sid).then(function(r){return r.json();})");
  sendChunk("    .then(function(d){if(d.ok)location.reload();else alert('Error: '+(d.error||'unknown'));})");
  sendChunk("    .catch(function(e){alert('Network error: '+e);});");
  sendChunk("}");

  sendChunk("function delSurvey(sid){");
  sendChunk("  if(!confirm('Delete survey? All points will be lost.'))return;");
  sendChunk("  fetch('/api/pts/delete?sid='+sid,{method:'POST'}).then(function(r){return r.json();})");
  sendChunk("    .then(function(d){");
  sendChunk("      if(d.ok){");
  sendChunk("        var div=document.querySelector('div[data-sid=\"'+sid+'\"]');");
  sendChunk("        if(div)div.remove();");
  sendChunk("        var banner=document.getElementById('active-banner');");
  sendChunk("        if(banner&&banner.getAttribute('data-sid')===sid)location.reload();");
  sendChunk("      }else{alert('Error: '+(d.error||'unknown'));}");
  sendChunk("    })");
  sendChunk("    .catch(function(e){alert('Network error: '+e);});");
  sendChunk("}");

  sendChunk("function delPoint(sid,pid){");
  sendChunk("  if(!confirm('Delete point '+pid+'?'))return;");
  sendChunk("  fetch('/api/pts/point/delete?sid='+sid+'&pid='+pid,{method:'POST'}).then(function(r){return r.json();})");
  sendChunk("    .then(function(d){");
  sendChunk("      if(d.ok){");
  sendChunk("        var tr=document.querySelector('tr[data-pid=\"'+pid+'\"]');");
  sendChunk("        if(tr)tr.remove();");
  sendChunk("        var cnt=document.getElementById('active-pts-count');");
  sendChunk("        if(cnt){var n=parseInt(cnt.textContent,10);if(!isNaN(n))cnt.textContent=n-1;}");
  sendChunk("        refreshSurveyMap();");
  sendChunk("      }else{alert('Error: '+(d.error||'unknown'));}");
  sendChunk("    })");
  sendChunk("    .catch(function(e){alert('Network error: '+e);});");
  sendChunk("}");

  sendChunk("function doSync(){fetch('/api/pts/sync').then(function(r){return r.text();}).then(function(t){alert(t);}).catch(function(e){alert('Sync error: '+e);});}");
  sendChunk("</script>");

  // ---- Active survey banner ----
  String activeSid = SurveyPoints::getActiveSurveyId();
  sendChunk("<div class='card'><h2>&#128205; Point Survey</h2>");
  if (activeSid.isEmpty()) {
    sendChunk("<div style='background:#fadbd8;color:#922b21;padding:12px;border-radius:4px;margin-bottom:12px;'>");
    sendChunk("&#9888; No active survey. Create a new survey or select one from the list.</div>");
  } else {
    String json  = SurveyPoints::loadSurveyJSON(activeSid);
    String title = activeSid;
    int ti = json.indexOf("\"title\":\"");
    if (ti >= 0) { int ts=ti+9, te=json.indexOf("\"",ts); if(te>ts) title=json.substring(ts,te); }
    int pts = SurveyPoints::getSurveyPointCount(activeSid);
    sendChunk("<div id='active-banner' data-sid='" + activeSid + "' style='background:#2ecc71;color:white;padding:10px;border-radius:4px;margin-bottom:12px'>");
    sendChunk("<b>Active survey: " + title + "</b> &nbsp;|&nbsp; Points: <span id='active-pts-count'>" + String(pts) + "</span> &nbsp;|&nbsp; ID: " + activeSid);
    sendChunk("</div>");
    // Measure form
    sendChunk("<h3>Measure point</h3>");
    sendChunk("<label>Point name</label><input id='pt-name' type='text' placeholder='automatic' style='max-width:200px'>");
    sendChunk("<label>Category</label><select id='pt-cat' onchange='updateCodici(this.value)' style='max-width:220px'><option value=''>-- choose --</option></select>");
    sendChunk("<label>Code</label><select id='pt-codice' style='max-width:220px'><option value=''>-- choose --</option></select>");
    sendChunk("<label>Duration (s)</label><input id='pt-dur' type='number' value='10' min='1' max='120' style='max-width:100px'>");
    sendChunk("<br><button id='btn-misura' onclick='doMeasure()'>&#9654; Measure</button>");
    sendChunk("<div class='prog-bar'><div id='meas-bar' class='prog-fill' style='width:0%'></div></div>");
    sendChunk("<div id='meas-status' style='margin:4px 0;font-family:monospace;font-size:0.9em;color:#555'></div>");
    sendChunk("<div id='meas-result' style='display:none;margin:8px 0;padding:8px;background:#ecf0f1;border-radius:4px'></div>");
  }
  sendChunk("</div>");

  // ---- Survey list ----
  sendChunk("<div class='card'><h2>Elenco Rilievi</h2>");
  sendChunk("<h3>New survey</h3>");
  sendChunk("<form method='POST' action='/api/pts/create' style='display:flex;gap:8px;flex-wrap:wrap'>");
  sendChunk("<input name='title' type='text' placeholder='Survey name' required style='max-width:300px'>");
  sendChunk("<input name='desc' type='text' placeholder='Descrizione (opzionale)' style='max-width:300px'>");
  sendChunk("<button type='submit'>+ Create</button></form><br>");

  std::vector<String> ids;
  SurveyPoints::listSurveyIds(ids);
  if (ids.empty()) {
    sendChunk("<p>No surveys found.</p>");
  } else {
    for (const String& sid : ids) {
      String json  = SurveyPoints::loadSurveyJSON(sid);
      String title = sid;
      String created = "";
      int ti = json.indexOf("\"title\":\"");
      if (ti >= 0) { int ts=ti+9, te=json.indexOf("\"",ts); if(te>ts) title=json.substring(ts,te); }
      int ci = json.indexOf("\"created\":\"");
      if (ci >= 0) { int cs=ci+11, ce=json.indexOf("\"",cs); if(ce>cs) created=json.substring(cs,ce); }
      int pts = SurveyPoints::getSurveyPointCount(sid);
      bool isActive = (sid == activeSid);

      sendChunk("<div data-sid='" + sid + "' style='border:1px solid #ddd;border-radius:4px;padding:10px;margin:6px 0;background:" + String(isActive?"#eafaf1":"white") + "'>");
      sendChunk("<b>" + String(isActive ? "(*) " : "") + title + "</b> &nbsp;");
      sendChunk("<span style='color:#7f8c8d'>Punti: " + String(pts) + " | " + created + " | ID: " + sid + "</span><br>");
      if (!isActive) {
        sendChunk("<button class='btn-small' onclick=\"setActive('" + sid + "')\">Set active</button> ");
      }
      sendChunk("<a class='btn btn-small' href='/api/pts/download?sid=" + sid + "' download='" + sid + ".geojson'>GeoJSON</a> ");
      sendChunk("<a class='btn btn-small' href='/api/pts/download/csv?sid=" + sid + "' download='" + sid + ".csv'>CSV</a> ");
      sendChunk("<button class='btn btn-small btn-danger' onclick=\"delSurvey('" + sid + "')\">Delete</button>");
      sendChunk("</div>");
    }
  }
  sendChunk("<br><button class='btn-secondary' onclick='doSync()'>Sync &rarr; SD</button>");
  sendChunk("</div>");

  // ---- Interactive map of active survey points (with measure tool + osnap) ----
  if (!activeSid.isEmpty()) {
    sendChunk("<div class='card'>");
    sendChunk("<h2>&#128205; Active survey map</h2>");

    // ---- CSS for map + measurement UI ----
    sendChunk("<style>");
    sendChunk("#survey-map{height:420px;background:#fff;border-radius:4px;position:relative;}");
    sendChunk("#survey-map.measure-mode{cursor:crosshair;}");
    // toolbar
    sendChunk(".map-toolbar{display:flex;gap:6px;margin-top:8px;align-items:center;flex-wrap:wrap;}");
    sendChunk(".map-toolbar button{font-size:13px;padding:6px 12px;}");
    sendChunk(".map-toolbar .active{background:#e67e22;color:white;}");
    // measurement panel
    sendChunk("#meas-panel{margin-top:10px;display:none;}");
    sendChunk("#meas-panel table{width:100%;font-size:0.85em;border-collapse:collapse;}");
    sendChunk("#meas-panel th{background:#34495e;color:white;padding:6px 8px;text-align:left;font-size:0.8em;}");
    sendChunk("#meas-panel td{padding:5px 8px;border-bottom:1px solid #ecf0f1;font-family:monospace;font-size:0.85em;}");
    sendChunk("#meas-panel tr:hover{background:#f7f9fa;}");
    // snap ring
    sendChunk(".snap-ring{border:2px solid #e67e22;background:rgba(230,126,34,0.15);}");
    // info bar
    sendChunk(".map-info-bar{display:flex;gap:12px;margin-top:6px;font-size:0.8em;color:#7f8c8d;font-family:monospace;}");
    // cross icon (divIcon SVG)
    sendChunk(".survey-cross-icon{background:none!important;border:none!important;}");
    sendChunk("</style>");

    // ---- Leaflet CSS + JS ----
    sendChunk("<style>");
    sendChunkPROGMEM(LEAFLET_CSS);
    sendChunk("</style>");
    sendChunk("<script>");
    sendChunkPROGMEM(LEAFLET_JS);
    sendChunk("</script>");

    // ---- Map container ----
    sendChunk("<div id='survey-map'></div>");

    // ---- Toolbar ----
    sendChunk("<div class='map-toolbar'>");
    sendChunk("<button onclick='refreshSurveyMap()' class='btn-secondary' title='Reload points'>&#x1F504; Refresh</button>");
    sendChunk("<button id='btn-measure' onclick='toggleMeasure()' class='btn-secondary' title='Measure distance between points (click on 2 points)'>&#128207; Measure</button>");
    sendChunk("<label style='display:flex;align-items:center;gap:4px;font-size:13px;cursor:pointer;'><input type='checkbox' id='chk-osnap' checked onchange='toggleOsnap(this.checked)'> OSnap</label>");
    sendChunk("<button id='btn-clear-meas' onclick='clearMeasurements()' class='btn-secondary' style='display:none' title='Clear all measurements'>&#128465; Clear measurements</button>");
    sendChunk("<span id='map-status' style='margin-left:auto;font-size:0.85em;color:#666'></span>");
    sendChunk("</div>");

    // ---- Info bar (zoom, cursor hint) ----
    sendChunk("<div class='map-info-bar'>");
    sendChunk("<span id='info-zoom'>Zoom: -</span>");
    sendChunk("<span id='info-cursor'></span>");
    sendChunk("<span id='info-snap' style='color:#e67e22;font-weight:bold'></span>");
    sendChunk("</div>");

    // ---- Measurement results panel ----
    sendChunk("<div id='meas-panel'>");
    sendChunk("<h3 style='margin:8px 0 4px;font-size:14px;color:#34495e'>&#128207; Measurements</h3>");
    sendChunk("<table><thead><tr>");
    sendChunk("<th>#</th><th>Da</th><th>A</th><th>Dist 2D</th><th>Dist 3D</th><th>&Delta;H</th><th>Azimut</th><th>Pendenza</th>");
    sendChunk("</tr></thead><tbody id='meas-tbody'></tbody></table>");
    sendChunk("</div>");

    // ======== JAVASCRIPT ========
    sendChunk("<script>");

    // --- State ---
    sendChunk("var _surveyMap=null,_markersLayer=null,_zoomHandler=null;");
    sendChunk("var _measMode=false,_measLayer=null,_snapLayer=null;");
    sendChunk("var _snapRing=null,_snappedPt=null,_measPtA=null;");
    sendChunk("var _measLine=null,_measCount=0;");
    sendChunk("var _pointsIndex=[];"); // [{name,lat,lon,alt,marker}]
    sendChunk("var _snapEnabled=true;"); // OSnap active by default

    // --- Utility: RTK color ---
    sendChunk("function rtkColor(s){var r=String(s).toLowerCase();if(r.indexOf('fixed')>=0)return'#27ae60';if(r.indexOf('float')>=0)return'#f39c12';return'#e74c3c';}");

    // --- Cross icon helpers ---
    sendChunk("function getCrossSize(z){if(z>=20)return 18;if(z>=17)return 14;return 12;}");
    sendChunk("function makeCrossIcon(color,sizePx){");
    sendChunk("  var s=sizePx||14;");
    sendChunk("  var svg='<svg width=\"'+s+'\" height=\"'+s+'\" viewBox=\"0 0 '+s+' '+s+'\" xmlns=\"http://www.w3.org/2000/svg\">'");
    sendChunk("    +'<line x1=\"'+(s/2)+'\" y1=\"0\" x2=\"'+(s/2)+'\" y2=\"'+s+'\" stroke=\"'+color+'\" stroke-width=\"2\"/>'");
    sendChunk("    +'<line x1=\"0\" y1=\"'+(s/2)+'\" x2=\"'+s+'\" y2=\"'+(s/2)+'\" stroke=\"'+color+'\" stroke-width=\"2\"/>'");
    sendChunk("    +'</svg>';");
    sendChunk("  return L.divIcon({className:'survey-cross-icon',html:svg,iconSize:[s,s],iconAnchor:[s/2,s/2],popupAnchor:[0,-s/2]});}");

    // --- Geodetic calculations ---
    // Haversine for 2D distance (metres)
    sendChunk("function haversine(lat1,lon1,lat2,lon2){");
    sendChunk("  var R=6378137,dLat=(lat2-lat1)*Math.PI/180,dLon=(lon2-lon1)*Math.PI/180;");
    sendChunk("  var a=Math.sin(dLat/2)*Math.sin(dLat/2)+Math.cos(lat1*Math.PI/180)*Math.cos(lat2*Math.PI/180)*Math.sin(dLon/2)*Math.sin(dLon/2);");
    sendChunk("  return R*2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));}");
    // Azimuth (degrees)
    sendChunk("function azimuth(lat1,lon1,lat2,lon2){");
    sendChunk("  var dLon=(lon2-lon1)*Math.PI/180;var y=Math.sin(dLon)*Math.cos(lat2*Math.PI/180);");
    sendChunk("  var x=Math.cos(lat1*Math.PI/180)*Math.sin(lat2*Math.PI/180)-Math.sin(lat1*Math.PI/180)*Math.cos(lat2*Math.PI/180)*Math.cos(dLon);");
    sendChunk("  return(Math.atan2(y,x)*180/Math.PI+360)%360;}");
    // Format distance with adaptive units
    sendChunk("function fmtDist(m){if(m>=1000)return(m/1000).toFixed(3)+' km';if(m>=1)return m.toFixed(3)+' m';return(m*100).toFixed(1)+' cm';}");
    // Format angle DMS
    sendChunk("function fmtAz(deg){var d=Math.floor(deg);var mm=(deg-d)*60;var m=Math.floor(mm);var s=((mm-m)*60).toFixed(1);return d+'\\u00b0'+('0'+m).slice(-2)+\"'\"+('0'+s).slice(-4)+'\"';}");

    // --- Snap engine (osnap) ---
    sendChunk("var SNAP_RADIUS_PX=18;"); // snap tolerance in pixels
    sendChunk("function findSnap(latlng){");
    sendChunk("  if(!_surveyMap||_pointsIndex.length===0)return null;");
    sendChunk("  var best=null,bestDist=Infinity;");
    sendChunk("  var pt=_surveyMap.latLngToContainerPoint(latlng);");
    sendChunk("  for(var i=0;i<_pointsIndex.length;i++){");
    sendChunk("    var pp=_surveyMap.latLngToContainerPoint(L.latLng(_pointsIndex[i].lat,_pointsIndex[i].lon));");
    sendChunk("    var dx=pt.x-pp.x,dy=pt.y-pp.y,d=Math.sqrt(dx*dx+dy*dy);");
    sendChunk("    if(d<SNAP_RADIUS_PX&&d<bestDist){bestDist=d;best=_pointsIndex[i];}");
    sendChunk("  }");
    sendChunk("  return best;}");

    // --- Show/hide snap ring ---
    sendChunk("function showSnapRing(pt){");
    sendChunk("  if(!_snapLayer)return;");
    sendChunk("  if(_snapRing){_snapLayer.removeLayer(_snapRing);}");
    sendChunk("  if(pt){");
    sendChunk("    _snapRing=L.circleMarker([pt.lat,pt.lon],{radius:12,className:'snap-ring',fillOpacity:0,weight:2,color:'#e67e22'});");
    sendChunk("    _snapRing.addTo(_snapLayer);");
    sendChunk("    document.getElementById('info-snap').textContent='\\u25CE SNAP: '+pt.name;");
    sendChunk("  }else{");
    sendChunk("    _snapRing=null;");
    sendChunk("    document.getElementById('info-snap').textContent='';");
    sendChunk("  }");
    sendChunk("  _snappedPt=pt;}");

    // --- Measurement live line (rubber band) ---
    sendChunk("var _rubberLine=null;");
    sendChunk("function updateRubber(latlng){");
    sendChunk("  if(!_measPtA||!_measLayer)return;");
    sendChunk("  var target=_snappedPt?L.latLng(_snappedPt.lat,_snappedPt.lon):latlng;");
    sendChunk("  if(_rubberLine){_rubberLine.setLatLngs([L.latLng(_measPtA.lat,_measPtA.lon),target]);}");
    sendChunk("  else{_rubberLine=L.polyline([L.latLng(_measPtA.lat,_measPtA.lon),target],{color:'#e67e22',weight:2,dashArray:'6,4'}).addTo(_measLayer);}");
    // live distance in info-cursor
    sendChunk("  var d2=haversine(_measPtA.lat,_measPtA.lon,target.lat,target.lng);");
    sendChunk("  document.getElementById('info-cursor').textContent='Dist: '+fmtDist(d2);}");

    // --- Toggle measurement mode ---
    sendChunk("function toggleMeasure(){");
    sendChunk("  _measMode=!_measMode;");
    sendChunk("  var btn=document.getElementById('btn-measure');");
    sendChunk("  btn.classList.toggle('active',_measMode);");
    sendChunk("  document.getElementById('survey-map').classList.toggle('measure-mode',_measMode);");
    sendChunk("  if(_measMode){");
    sendChunk("    if(!_measLayer){_measLayer=L.layerGroup().addTo(_surveyMap);}");
    sendChunk("    if(!_snapLayer){_snapLayer=L.layerGroup().addTo(_surveyMap);}");
    sendChunk("    _measPtA=null;_rubberLine=null;");
    sendChunk("    document.getElementById('info-cursor').textContent='Click point A';");
    sendChunk("  }else{");
    sendChunk("    if(_rubberLine&&_measLayer){_measLayer.removeLayer(_rubberLine);_rubberLine=null;}");
    sendChunk("    showSnapRing(null);_measPtA=null;");
    sendChunk("    document.getElementById('info-cursor').textContent='';");
    sendChunk("  }}");

    // --- Map click handler for measurement ---
    sendChunk("function onMapClickMeas(e){");
    sendChunk("  if(!_measMode)return;");
    sendChunk("  if(e.originalEvent&&e.originalEvent._measHandled)return;"); // skip map click when marker already handled it
    // Determine point to use: prefer explicit _pt (from marker click), else snap if enabled, else virtual from click coords
    sendChunk("  var pt=e._pt||null;");
    sendChunk("  if(!pt){");
    sendChunk("    if(_snapEnabled){");
    sendChunk("      pt=_snappedPt;");
    sendChunk("      if(!pt){document.getElementById('info-cursor').textContent='\\u26A0 Move closer to a point to snap!';return;}");
    sendChunk("    }else{");
    sendChunk("      pt={name:'('+e.latlng.lat.toFixed(6)+','+e.latlng.lng.toFixed(6)+')',lat:e.latlng.lat,lon:e.latlng.lng,alt:0};");
    sendChunk("    }");
    sendChunk("  }");
    sendChunk("  if(!_measPtA){");
    // First point selected
    sendChunk("    _measPtA=pt;");
    sendChunk("    document.getElementById('info-cursor').textContent='A: '+pt.name+' \\u2014 now click point B';");
    sendChunk("    L.circleMarker([pt.lat,pt.lon],{radius:8,color:'#e67e22',fillColor:'#e67e22',fillOpacity:0.6,weight:2}).addTo(_measLayer);");
    sendChunk("  }else{");
    // Second point -> compute and display measurement
    sendChunk("    var ptB=pt;");
    sendChunk("    var d2=haversine(_measPtA.lat,_measPtA.lon,ptB.lat,ptB.lon);");
    sendChunk("    var dH=ptB.alt-_measPtA.alt;");
    sendChunk("    var d3=Math.sqrt(d2*d2+dH*dH);");
    sendChunk("    var az=azimuth(_measPtA.lat,_measPtA.lon,ptB.lat,ptB.lon);");
    sendChunk("    var slope=(d2>0.001)?Math.atan2(Math.abs(dH),d2)*180/Math.PI:0;");
    // Draw final measurement line
    sendChunk("    if(_rubberLine){_measLayer.removeLayer(_rubberLine);_rubberLine=null;}");
    sendChunk("    var line=L.polyline([[_measPtA.lat,_measPtA.lon],[ptB.lat,ptB.lon]],{color:'#c0392b',weight:2.5,opacity:0.9});");
    sendChunk("    line.addTo(_measLayer);");
    // Midpoint label
    sendChunk("    var midLat=(_measPtA.lat+ptB.lat)/2,midLon=(_measPtA.lon+ptB.lon)/2;");
    sendChunk("    var lbl=L.divIcon({className:'',html:'<div style=\"background:rgba(192,57,43,0.9);color:white;padding:2px 6px;border-radius:3px;font-size:11px;font-family:monospace;white-space:nowrap;transform:translate(-50%,-50%)\">'+fmtDist(d2)+'</div>',iconAnchor:[0,0]});");
    sendChunk("    L.marker([midLat,midLon],{icon:lbl,interactive:false}).addTo(_measLayer);");
    // B marker
    sendChunk("    L.circleMarker([ptB.lat,ptB.lon],{radius:8,color:'#c0392b',fillColor:'#c0392b',fillOpacity:0.6,weight:2}).addTo(_measLayer);");
    // Popup on line
    sendChunk("    line.bindPopup('<b>'+_measPtA.name+' \\u2192 '+ptB.name+'</b><br>2D: '+fmtDist(d2)+'<br>3D: '+fmtDist(d3)+'<br>\\u0394H: '+(dH>=0?'+':'')+dH.toFixed(3)+' m<br>Az: '+fmtAz(az)+'<br>Pendenza: '+slope.toFixed(2)+'\\u00b0').openPopup();");
    // Add row to table
    sendChunk("    _measCount++;");
    sendChunk("    var tbody=document.getElementById('meas-tbody');");
    sendChunk("    var tr=document.createElement('tr');");
    sendChunk("    tr.innerHTML='<td>'+_measCount+'</td><td>'+_measPtA.name+'</td><td>'+ptB.name+'</td><td>'+fmtDist(d2)+'</td><td>'+fmtDist(d3)+'</td><td>'+(dH>=0?'+':'')+dH.toFixed(3)+' m</td><td>'+fmtAz(az)+'</td><td>'+slope.toFixed(2)+'\\u00b0</td>';");
    sendChunk("    tbody.appendChild(tr);");
    sendChunk("    document.getElementById('meas-panel').style.display='block';");
    sendChunk("    document.getElementById('btn-clear-meas').style.display='';");
    // Reset for next measurement
    sendChunk("    _measPtA=null;");
    sendChunk("    document.getElementById('info-cursor').textContent='Click point A for new measurement';");
    sendChunk("  }}");

    // --- Mouse move handler for snap ---
    sendChunk("function onMapMoveMeas(e){");
    sendChunk("  if(!_measMode)return;");
    sendChunk("  var snap=_snapEnabled?findSnap(e.latlng):null;");
    sendChunk("  showSnapRing(snap);");
    sendChunk("  if(_measPtA)updateRubber(e.latlng);}");

    // --- Clear all measurements ---
    sendChunk("function clearMeasurements(){");
    sendChunk("  if(_measLayer){_measLayer.clearLayers();}");
    sendChunk("  _measPtA=null;_rubberLine=null;_measCount=0;");
    sendChunk("  document.getElementById('meas-tbody').innerHTML='';");
    sendChunk("  document.getElementById('meas-panel').style.display='none';");
    sendChunk("  document.getElementById('btn-clear-meas').style.display='none';");
    sendChunk("  document.getElementById('info-cursor').textContent=_measMode?'Click point A':'';}");

    // --- Toggle OSnap ---
    sendChunk("function toggleOsnap(val){");
    sendChunk("  _snapEnabled=val;");
    sendChunk("  if(!_snapEnabled){showSnapRing(null);_snappedPt=null;}");
    sendChunk("}");

    // --- Update info bar on zoom/move ---
    sendChunk("function updateInfoBar(){");
    sendChunk("  if(!_surveyMap)return;");
    sendChunk("  var z=_surveyMap.getZoom();");
    sendChunk("  document.getElementById('info-zoom').textContent='Zoom: '+z.toFixed(1);}");

    // --- Main map init ---
    sendChunk("function initSurveyMap(geojson){");
    sendChunk("  var features=geojson.features||[];");
    sendChunk("  if(features.length===0){document.getElementById('map-status').textContent='No points';return;}");
    sendChunk("  if(!_surveyMap){");
    sendChunk("    _surveyMap=L.map('survey-map',{zoomControl:true,attributionControl:false,scrollWheelZoom:true,zoomSnap:0.25,zoomDelta:0.5,touchZoom:true,maxZoom:28,minZoom:1}).setView([42.5,12.5],6);");
    sendChunk("    setTimeout(function(){if(_surveyMap){_surveyMap.invalidateSize();}},200);");
    sendChunk("    document.getElementById('survey-map').style.background='#ffffff';");

    // --- Custom scale bar (improved with ticks + dual readout) ---
    sendChunk("    var CustomScale=L.Control.extend({options:{position:'bottomleft'},");
    sendChunk("      onAdd:function(map){this._map=map;this._container=L.DomUtil.create('div','leaflet-control');");
    sendChunk("        this._container.style.cssText='background:rgba(255,255,255,0.92);padding:4px 10px 2px;border-radius:4px;font-size:11px;font-family:monospace;box-shadow:0 1px 5px rgba(0,0,0,0.35);pointer-events:none';");
    sendChunk("        map.on('zoom move',this._update,this);this._update();return this._container;},");
    sendChunk("      onRemove:function(map){map.off('zoom move',this._update,this);},");
    sendChunk("      _update:function(){var map=this._map;var lat=map.getBounds().getCenter().lat;");
    sendChunk("        var mpp=40075016.686*Math.abs(Math.cos(lat*Math.PI/180))/Math.pow(2,map.getZoom()+8);");
    sendChunk("        var raw=mpp*150,val,unit,px;");
    sendChunk("        if(raw>=1000){val=[1,2,5,10,20,50,100,200,500,1000].find(function(v){return v>=raw/1000;})||1000;unit='km';px=(val*1000)/mpp;}");
    sendChunk("        else if(raw>=1){val=[1,2,5,10,20,50,100,200,500].find(function(v){return v>=raw;})||500;unit='m';px=val/mpp;}");
    sendChunk("        else{var cm=raw*100;val=[1,2,5,10,20,50].find(function(v){return v>=cm;})||50;unit='cm';px=(val/100)/mpp;}");
    // Improved scale bar with 4 ticks
    sendChunk("        var h='<svg width=\"'+(px+4)+'\" height=\"14\" style=\"display:block\"><line x1=\"2\" y1=\"10\" x2=\"'+(px+2)+'\" y2=\"10\" stroke=\"#333\" stroke-width=\"2\"/>';");
    sendChunk("        for(var t=0;t<=4;t++){var tx=2+px*t/4;h+='<line x1=\"'+tx+'\" y1=\"6\" x2=\"'+tx+'\" y2=\"14\" stroke=\"#333\" stroke-width=\"1.5\"/>';}");
    sendChunk("        h+='</svg><div style=\"display:flex;justify-content:space-between;width:'+(px+4)+'px;color:#333;font-weight:bold;font-size:10px\"><span>0</span><span>'+val+' '+unit+'</span></div>';");
    sendChunk("        this._container.innerHTML=h;}");
    sendChunk("    });");
    sendChunk("    new CustomScale().addTo(_surveyMap);");

    // --- Zoom level indicator (top-right) ---
    sendChunk("    var ZoomInfo=L.Control.extend({options:{position:'topright'},");
    sendChunk("      onAdd:function(map){this._map=map;this._el=L.DomUtil.create('div','leaflet-control');");
    sendChunk("        this._el.style.cssText='background:rgba(44,62,80,0.85);color:white;padding:3px 8px;border-radius:4px;font:bold 11px monospace;pointer-events:none';");
    sendChunk("        map.on('zoom',this._upd,this);this._upd();return this._el;},");
    sendChunk("      onRemove:function(map){map.off('zoom',this._upd,this);},");
    sendChunk("      _upd:function(){this._el.textContent='Z'+this._map.getZoom().toFixed(1);}");
    sendChunk("    });");
    sendChunk("    new ZoomInfo().addTo(_surveyMap);");

    // --- Event listeners ---
    sendChunk("    _surveyMap.on('click',onMapClickMeas);");
    sendChunk("    _surveyMap.on('mousemove',onMapMoveMeas);");
    sendChunk("    _surveyMap.on('zoom move',updateInfoBar);");

    sendChunk("  }"); // end if(!_surveyMap)

    // --- Reset layers ---
    sendChunk("  if(_zoomHandler){_surveyMap.off('zoomend',_zoomHandler);_zoomHandler=null;}");
    sendChunk("  if(_markersLayer){_markersLayer.clearLayers();}else{_markersLayer=L.layerGroup().addTo(_surveyMap);}");
    sendChunk("  _pointsIndex=[];");

    // --- Add markers ---
    sendChunk("  var latlngs=[],markers=[];");
    sendChunk("  features.forEach(function(f){");
    sendChunk("    var c=f.geometry.coordinates;var lat=c[1],lon=c[0];");
    sendChunk("    var props=f.properties||{};");
    sendChunk("    var name=props.name||f.id||'P';");
    sendChunk("    var codice=props.codice||'';");
    sendChunk("    var tpv=props.TPV||{};var hp=props.HPPOSLLH||{};");
    sendChunk("    var rtkRaw=tpv.rtk||props.rtk||'No RTK';");
    sendChunk("    var altHAE=hp.altHAE||c[2]||0;");
    sendChunk("    var altMSL=hp.altMSL||c[2]||0;");
    sendChunk("    var crossColor=rtkColor(rtkRaw);");
    sendChunk("    var m=L.marker([lat,lon],{icon:makeCrossIcon(crossColor,getCrossSize(_surveyMap.getZoom())),interactive:true});");
    // Enhanced popup with more info
    sendChunk("    var popupContent='<b>'+name+'</b>'+(codice?' <span style=color:#7f8c8d>('+codice+')</span>':'')+'<br>Lat: '+lat.toFixed(8)+'<br>Lon: '+lon.toFixed(8)+'<br>Alt HAE: '+parseFloat(altHAE).toFixed(3)+' m<br>Alt MSL: '+parseFloat(altMSL).toFixed(3)+' m<br>RTK: <b>'+rtkRaw+'</b>';");
    sendChunk("    m.bindPopup(popupContent);");
    sendChunk("    (function(marker,mLat,mLon,mAlt,mName){");
    sendChunk("      marker.on('click',function(e){");
    sendChunk("        if(_measMode){");
    sendChunk("          marker.closePopup();");
    sendChunk("          if(e.originalEvent){e.originalEvent.stopPropagation();e.originalEvent._measHandled=true;}");
    sendChunk("          var usePt=(_snapEnabled&&_snappedPt)?_snappedPt:{name:mName,lat:mLat,lon:mLon,alt:mAlt};");
    sendChunk("          onMapClickMeas({latlng:L.latLng(usePt.lat,usePt.lon),_pt:usePt});");
    sendChunk("        }");
    sendChunk("      });");
    sendChunk("    })(m,lat,lon,parseFloat(altHAE)||0,name);");
    sendChunk("    m.addTo(_markersLayer);markers.push(m);latlngs.push([lat,lon]);");
    // Index for snap
    sendChunk("    _pointsIndex.push({name:name,lat:lat,lon:lon,alt:parseFloat(altHAE)||0,marker:m});");
    sendChunk("  });");

    // --- Zoom handler ---
    sendChunk("  _zoomHandler=function(){");
    sendChunk("    var sz=getCrossSize(_surveyMap.getZoom());");
    sendChunk("    markers.forEach(function(mk){");
    sendChunk("      var h=mk.options.icon.options.html||'';");
    sendChunk("      var colorMatch=h.match(/stroke=\"([^\"]+)\"/);");
    sendChunk("      var col=colorMatch?colorMatch[1]:'#e74c3c';");
    sendChunk("      mk.setIcon(makeCrossIcon(col,sz));");
    sendChunk("    });");
    sendChunk("  };");
    sendChunk("  _surveyMap.on('zoomend',_zoomHandler);");

    // --- Fit bounds ---
    sendChunk("  if(latlngs.length>0){_surveyMap.fitBounds(L.latLngBounds(latlngs),{padding:[30,30]});}");
    sendChunk("  document.getElementById('map-status').textContent=features.length+' points';");
    sendChunk("  updateInfoBar();");
    sendChunk("}"); // end initSurveyMap

    // --- Refresh ---
    sendChunk("function refreshSurveyMap(){");
    sendChunk("  if(_surveyMap){_surveyMap.invalidateSize();}");
    sendChunk("  document.getElementById('map-status').textContent='Loading...';");
    sendChunk("  fetch('/api/pts/download').then(function(r){return r.json();}).then(function(g){initSurveyMap(g);}).catch(function(e){document.getElementById('map-status').textContent='Error: '+e;});");
    sendChunk("}");
    sendChunk("(function(){");
    sendChunk("  var mapDiv=document.getElementById('survey-map');");
    sendChunk("  if(!mapDiv)return;");
    sendChunk("  if('IntersectionObserver' in window){");
    sendChunk("    var obs=new IntersectionObserver(function(entries){");
    sendChunk("      if(entries[0].isIntersecting){obs.disconnect();refreshSurveyMap();}");
    sendChunk("    },{threshold:0.1});");
    sendChunk("    obs.observe(mapDiv);");
    sendChunk("  } else {");
    sendChunk("    refreshSurveyMap();");
    sendChunk("  }");
    sendChunk("})();");

    sendChunk("</script>");
    sendChunk("</div>");
  }

  // ---- Point table for active survey ----
  if (!activeSid.isEmpty() && SurveyPoints::getSurveyPointCount(activeSid) > 0) {
    sendChunk("<div class='card'><h2>Active survey points</h2>");
    sendChunk("<table><thead><tr>");
    sendChunk("<th>ID</th><th>Name</th><th>Code</th><th>Lat</th><th>Lon</th><th>AltHAE</th>");
    sendChunk("<th>PDOP</th><th>HDOP</th><th>VDOP</th><th>&sigma;N(m)</th><th>&sigma;E(m)</th><th>&sigma;U(m)</th><th>RTK</th><th>Samples</th><th>Actions</th>");
    sendChunk("</tr></thead><tbody>");

    String json = SurveyPoints::loadSurveyJSON(activeSid);
    int pos = 0;
    while (true) {
      int fi = json.indexOf("\"type\":\"Feature\"", pos);
      if (fi < 0) break;
      int featureStart = -1;
      for (int i = fi; i >= 1; i--) { if (json[i] == '{') { featureStart = i; break; } }
      if (featureStart < 0) { pos = fi+1; continue; }
      int depth=0, featureEnd=-1;
      for (int i=featureStart; i<(int)json.length(); i++) {
        if (json[i]=='{') depth++;
        else if (json[i]=='}') { if(--depth==0){featureEnd=i;break;} }
      }
      if (featureEnd<0) break;

      auto getStr=[&](const String& key, int from)->String{
        String nd="\""+key+"\":\""; int ki=json.indexOf(nd,from); if(ki<0)return "";
        int vs=ki+nd.length(), ve=json.indexOf("\"",vs); return ve>vs?json.substring(vs,ve):"";
      };
      auto getNum=[&](const String& key, int from)->String{
        String nd="\""+key+"\":"; int ki=json.indexOf(nd,from); if(ki<0)return "0";
        int vs=ki+nd.length(); while(vs<(int)json.length()&&json[vs]==' ')vs++;
        int ve=vs; while(ve<(int)json.length()&&(isdigit(json[ve])||json[ve]=='.'||json[ve]=='-'||json[ve]=='e'||json[ve]=='E'))ve++;
        return json.substring(vs,ve);
      };

      String pid    = getStr("id",    featureStart);
      String name   = getStr("name",  featureStart);
      String codice = getStr("codice",featureStart);
      String rtk    = getStr("rtk",   featureStart);
      int propsPos  = json.indexOf("\"properties\":", featureStart);
      int from      = propsPos>0 ? propsPos : featureStart;

      int geomPos   = json.indexOf("\"coordinates\":", featureStart);
      String lat_s="0", lon_s="0", alt_s="0";
      if (geomPos>0) {
        int bp=json.indexOf("[",geomPos);
        if(bp>0){int eb=json.indexOf("]",bp); String coords=json.substring(bp+1,eb);
          int c1=coords.indexOf(","); int c2=coords.indexOf(",",c1+1);
          lon_s=coords.substring(0,c1); lat_s=coords.substring(c1+1,c2>0?c2:coords.length());
          if(c2>0) alt_s=coords.substring(c2+1); lat_s.trim(); lon_s.trim(); alt_s.trim();}
      }

      sendChunk("<tr data-pid='" + pid + "'><td>" + pid + "</td><td>" + name + "</td><td>" + codice + "</td>");
      sendChunk("<td>" + lat_s + "</td><td>" + lon_s + "</td><td>" + alt_s + "</td>");
      sendChunk("<td>" + getNum("pdop",from) + "</td><td>" + getNum("hdop",from) + "</td><td>" + getNum("vdop",from) + "</td>");
      sendChunk("<td>" + getNum("sigma_N",from) + "</td><td>" + getNum("sigma_E",from) + "</td><td>" + getNum("sigma_U",from) + "</td>");
      sendChunk("<td>" + rtk + "</td><td>" + getNum("n_samples",from) + "</td>");
      sendChunk("<td><button class='btn btn-small btn-danger' onclick=\"delPoint('" + activeSid + "','" + pid + "')\">X</button></td>");
      sendChunk("</tr>");
      pos = featureEnd + 1;
    }
    sendChunk("</tbody></table></div>");
  }

  sendFooter();
}

// ---- Survey API: create ----
static void handlePtsCreate() {
  String title = _server->hasArg("title") ? _server->arg("title") : "";
  String desc  = _server->hasArg("desc")  ? _server->arg("desc")  : "";
  if (title.isEmpty()) { _server->sendHeader("Location","/survey"); _server->send(303); return; }
  String sid = SurveyPoints::createSurvey(title, desc);
  if (!sid.isEmpty()) SurveyPoints::setActiveSurveyId(sid);
  _server->sendHeader("Location", "/survey");
  _server->send(303);
}

// ---- Survey API: list ----
static void handlePtsList() {
  std::vector<String> ids;
  SurveyPoints::listSurveyIds(ids);
  String activeSid = SurveyPoints::getActiveSurveyId();
  String json = "[";
  for (size_t i = 0; i < ids.size(); i++) {
    if (i > 0) json += ",";
    String sv = SurveyPoints::loadSurveyJSON(ids[i]);
    String title = ids[i];
    int ti = sv.indexOf("\"title\":\"");
    if (ti >= 0) { int ts=ti+9, te=sv.indexOf("\"",ts); if(te>ts) title=sv.substring(ts,te); }
    json += "{\"sid\":\"" + ids[i] + "\",\"title\":\"" + title + "\",\"pts\":" + String(SurveyPoints::getSurveyPointCount(ids[i])) + ",\"active\":" + (ids[i]==activeSid?"true":"false") + "}";
  }
  json += "]";
  _server->send(200, "application/json", json);
}

// ---- Survey API: delete ----
static void handlePtsDelete() {
  String sid = _server->hasArg("sid") ? _server->arg("sid") : "";
  if (sid.isEmpty()) { _server->send(400, "application/json", "{\"error\":\"Missing sid\"}"); return; }
  bool ok = SurveyPoints::deleteSurvey(sid);
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"Delete failed\"}");
}

// ---- Survey API: set active ----
static void handlePtsSetActive() {
  String sid = _server->hasArg("sid") ? _server->arg("sid") : "";
  if (sid.isEmpty()) { _server->send(400,"application/json","{\"error\":\"Missing sid\"}"); return; }
  bool ok = SurveyPoints::setActiveSurveyId(sid);
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"Failed\"}");
}

// ---- Survey API: start measure ----
static void handlePtsMeasure() {
  MeasureParams mp;
  mp.name         = _server->hasArg("name")     ? _server->arg("name")     : "";
  mp.codice       = _server->hasArg("codice")    ? _server->arg("codice")   : "";
  mp.desc         = _server->hasArg("desc")      ? _server->arg("desc")     : "";
  mp.durationSec  = _server->hasArg("duration")  ? _server->arg("duration").toFloat() : 10.0f;
  mp.intervalSec  = _server->hasArg("interval")  ? _server->arg("interval").toFloat() : 0.5f;
  mp.forceQuality = _server->hasArg("force")     ? (_server->arg("force") != "0") : false;
  if (mp.durationSec < 1.0f) mp.durationSec = 1.0f;
  String activeSid = SurveyPoints::getActiveSurveyId();
  if (activeSid.isEmpty()) {
    _server->send(400, "application/json", "{\"error\":\"No active survey\"}");
    return;
  }
  if (SurveyPoints::isMeasuring()) {
    _server->send(400, "application/json", "{\"error\":\"Already measuring\"}");
    return;
  }
  bool ok = SurveyPoints::startMeasure(mp);
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"Failed to start measure\"}");
}

// ---- Survey API: measure status ----
static void handlePtsMeasureStatus() {
  MeasureProgress p = SurveyPoints::getMeasureProgress();
  String statusStr;
  switch (p.status) {
    case MS_IDLE:    statusStr = "idle";    break;
    case MS_RUNNING: statusStr = "running"; break;
    case MS_DONE:    statusStr = "done";    break;
    case MS_ERROR:   statusStr = "error";   break;
  }
  String json = "{";
  json += "\"status\":\"" + statusStr + "\",";
  json += "\"pct\":"      + String(p.pct) + ",";
  json += "\"nSamples\":" + String(p.nSamples) + ",";
  json += "\"curHAcc\":"  + String(p.curHAcc, 4) + ",";
  json += "\"elapsed\":"  + String(p.elapsed, 1) + ",";
  json += "\"lastPointId\":\"" + p.lastPointId + "\",";
  json += "\"errorMsg\":\"" + p.errorMsg + "\"";
  json += "}";
  _server->send(200, "application/json", json);
}

// ---- Survey API: get points ----
static void handlePtsPoints() {
  String sid = _server->hasArg("sid") ? _server->arg("sid") : SurveyPoints::getActiveSurveyId();
  if (sid.isEmpty()) { _server->send(400,"application/json","{\"error\":\"No survey\"}"); return; }
  _server->send(200, "application/json", SurveyPoints::loadSurveyJSON(sid));
}

// ---- Survey API: delete point ----
static void handlePtsPointDelete() {
  String sid = _server->hasArg("sid") ? _server->arg("sid") : "";
  String pid = _server->hasArg("pid") ? _server->arg("pid") : "";
  if (sid.isEmpty() || pid.isEmpty()) { _server->send(400,"application/json","{\"error\":\"Missing sid/pid\"}"); return; }
  bool ok = SurveyPoints::deletePoint(sid, pid);
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"Not found\"}");
}

// ---- Survey API: download GeoJSON ----
static void handlePtsDownload() {
  String sid = _server->hasArg("sid") ? _server->arg("sid") : SurveyPoints::getActiveSurveyId();
  if (sid.isEmpty()) { _server->send(404,"text/plain","No survey"); return; }
  String geo = SurveyPoints::getSurveyGeoJSON(sid);
  _server->sendHeader("Content-Disposition", "attachment; filename=\"" + sid + ".geojson\"");
  _server->send(200, "application/geo+json", geo);
}

// ---- Survey API: download CSV ----
static void handlePtsDownloadCSV() {
  String sid = _server->hasArg("sid") ? _server->arg("sid") : SurveyPoints::getActiveSurveyId();
  if (sid.isEmpty()) { _server->send(404,"text/plain","No survey"); return; }
  String csv = SurveyPoints::getSurveyCSV(sid);
  _server->sendHeader("Content-Disposition", "attachment; filename=\"" + sid + ".csv\"");
  _server->send(200, "text/csv", csv);
}

// ---- Survey API: force sync to SD ----
static void handlePtsSync() {
  SurveyPoints::syncToSD();
  _server->send(200, "text/plain", "Survey sync to SD completed");
}

// ========================================================================
// STAKEOUT PAGE AND API HANDLERS
// ========================================================================

// Upload buffer (max 32KB — sufficient for ~300 points in GeoJSON)
#define STAKEOUT_MAX_UPLOAD 32768
static String  g_stakeoutUploadContent;
static bool    g_stakeoutUploadError  = false;
static String  g_stakeoutUploadErrMsg;
static String  g_stakeoutUploadName;   // filename (detected extension)
static String  g_stakeoutHintLat;
static String  g_stakeoutHintLon;
static String  g_stakeoutHintH;
static String  g_stakeoutHintName;

// ----- Stakeout page -----
static void handleStakeoutPage() {
  sendHeader("Stakeout", "stakeout");

  sendChunk("<style>");
  sendChunk(".sk-status{background:#2c3e50;color:#ecf0f1;padding:12px 16px;border-radius:6px;font-family:monospace;margin-bottom:16px;}");
  sendChunk(".sk-status p{margin:4px 0;font-size:15px;}");
  sendChunk(".sk-badge-fix{padding:2px 8px;border-radius:10px;font-size:11px;margin-left:6px;}");
  sendChunk(".fix-fixed{background:#2ecc71;color:#000;}.fix-float{background:#f39c12;color:#fff;}.fix-none{background:#7f8c8d;color:#fff;}");
  sendChunk("</style>");

  sendChunk("<script>");
  // Poll /api/stakeout/status every 2 seconds
  sendChunk("var _skTimer=null;");
  sendChunk("function skStart(){_skTimer=setInterval(skPoll,2000);skPoll();}");
  sendChunk("function skPoll(){");
  sendChunk("  fetch('/api/stakeout/status').then(function(r){return r.json();}).then(function(d){");
  sendChunk("    var el=document.getElementById('sk-status');");
  sendChunk("    if(!el)return;");
  sendChunk("    if(!d.valid){el.innerHTML='<p>No active target or no GNSS fix.</p>';return;}");
  sendChunk("    var fc={'2':'fix-fixed','1':'fix-float','0':'fix-none'};");
  sendChunk("    var fl={'2':'RTK FIX','1':'FLOAT','0':'NO FIX'};");
  sendChunk("    var cs=String(d.roverCarrSoln||0);");
  sendChunk("    el.innerHTML='<p><b>Target:</b> '+d.targetName+' ('+d.targetId+')'");
  sendChunk("      +'<span class=\"sk-badge-fix '+(fc[cs]||'fix-none')+'\">'+(fl[cs]||'NO FIX')+'</span></p>'");
  sendChunk("      +'<p>&#x1F4CF; D2D: <b>'+d.d2d.toFixed(3)+' m</b></p>'");
  sendChunk("      +'<p>&#x2B06; dH: <b>'+(d.dH===null?'N/A':((d.dH>=0?'+':'')+d.dH.toFixed(3)+' m'))+'</b></p>'");
  sendChunk("      +'<p>&#x1F4CF; Quota HAE: <b>'+(d.targetH===null?'N/A':d.targetH.toFixed(3)+' m')+'</b></p>'");
  sendChunk("      +'<p>&#x1F9ED; Az: <b>'+d.az.toFixed(1)+'&deg;</b></p>';");
  sendChunk("  }).catch(function(){});");
  sendChunk("}");
  // setActive: file+point
  sendChunk("function skSetActive(fid,pid){");
  sendChunk("  fetch('/api/stakeout/active',{method:'POST',headers:{'Content-Type':'application/json'},");
  sendChunk("    body:JSON.stringify({fileId:fid,pointId:pid})})");
  sendChunk("  .then(function(r){return r.json();}).then(function(){location.reload();}).catch(function(e){alert('Error: '+e);});");
  sendChunk("}");
  // deleteFile
  sendChunk("function skDelFile(fid){");
  sendChunk("  if(!confirm('Delete this file?'))return;");
  sendChunk("  fetch('/api/stakeout/file/delete',{method:'POST',headers:{'Content-Type':'application/json'},");
  sendChunk("    body:JSON.stringify({fileId:fid})})");
  sendChunk("  .then(function(){location.reload();}).catch(function(e){alert('Error: '+e);});");
  sendChunk("}");
  // importFromSurvey
  sendChunk("function skImportSurvey(sid,title){");
  sendChunk("  fetch('/api/stakeout/import-survey',{method:'POST',headers:{'Content-Type':'application/json'},");
  sendChunk("    body:JSON.stringify({sid:sid,name:title})})");
  sendChunk("  .then(function(r){return r.json();}).then(function(d){");
  sendChunk("    if(d.ok){alert('Imported!');location.reload();}else{alert('Error: '+(d.error||'unknown'));}");
  sendChunk("  }).catch(function(e){alert('Error: '+e);});");
  sendChunk("}");
  sendChunk("window.addEventListener('load',skStart);");
  sendChunk("</script>");

  // --- Live status card ---
  sendChunk("<div class='card'><h2>&#127987; Active navigation</h2>");
  sendChunk("<div id='sk-status' class='sk-status'><p>Loading...</p></div>");
  sendChunk("</div>");

  // --- Upload card ---
  sendChunk("<div class='card'><h2>&#8679; Upload file</h2>");
  sendChunk("<p>Supported formats: <b>CSV</b> (columns: name/id, lat, lon, h) and <b>GeoJSON</b> (FeatureCollection of Point).</p>");
  sendChunk("<form method='POST' action='/api/stakeout/upload' enctype='multipart/form-data' style='display:flex;gap:8px;flex-wrap:wrap;align-items:center'>");
  sendChunk("<input type='text' name='name' placeholder='Dataset name' style='max-width:200px' required>");
  sendChunk("<input type='file' name='file' accept='.csv,.geojson,.json,.txt,.tsv' required>");
  sendChunk("<button type='submit' class='btn btn-success'>Upload</button>");
  sendChunk("</form>");
  sendChunk("<details style='margin-top:10px'><summary style='cursor:pointer;color:#2980b9;font-size:13px'>&#9881; CSV column mapping (optional)</summary>");
  sendChunk("<p style='font-size:12px;color:#666;margin:6px 0'>If automatic detection does not work, specify the exact column names from your CSV.</p>");
  sendChunk("<form method='POST' action='/api/stakeout/upload' enctype='multipart/form-data' style='display:flex;gap:6px;flex-wrap:wrap;align-items:end'>");
  sendChunk("<div style='display:flex;flex-direction:column;gap:2px'><label style='font-size:11px'>Dataset name</label><input type='text' name='name' placeholder='Name' style='width:120px' required></div>");
  sendChunk("<div style='display:flex;flex-direction:column;gap:2px'><label style='font-size:11px'>CSV file</label><input type='file' name='file' accept='.csv,.txt,.tsv' required></div>");
  sendChunk("<div style='display:flex;flex-direction:column;gap:2px'><label style='font-size:11px'>Latitude col.</label><input type='text' name='hint_lat' placeholder='e.g. lat' style='width:90px'></div>");
  sendChunk("<div style='display:flex;flex-direction:column;gap:2px'><label style='font-size:11px'>Longitude col.</label><input type='text' name='hint_lon' placeholder='e.g. lon' style='width:90px'></div>");
  sendChunk("<div style='display:flex;flex-direction:column;gap:2px'><label style='font-size:11px'>Height HAE col.</label><input type='text' name='hint_h' placeholder='e.g. altHAE' style='width:90px'></div>");
  sendChunk("<div style='display:flex;flex-direction:column;gap:2px'><label style='font-size:11px'>Point name col.</label><input type='text' name='hint_name' placeholder='e.g. name' style='width:90px'></div>");
  sendChunk("<button type='submit' class='btn btn-success'>Upload with mapping</button>");
  sendChunk("</form></details></div>");

  // --- Import from existing surveys card ---
  {
    std::vector<String> sids;
    SurveyPoints::listSurveyIds(sids);
    if (!sids.empty()) {
      sendChunk("<div class='card'><h2>&#128229; Import from Surveys</h2>");
      sendChunk("<p>Import points directly from an existing survey without downloading and re-uploading.</p>");
      sendChunk("<table><thead><tr><th>Survey</th><th>Points</th><th></th></tr></thead><tbody>");
      for (const String& sid : sids) {
        int pts = SurveyPoints::getSurveyPointCount(sid);
        // Extract title
        String json = SurveyPoints::loadSurveyJSON(sid);
        String title = sid;
        int ti = json.indexOf("\"title\":\"");
        if (ti >= 0) {
          int ts = ti + 9;
          int te = json.indexOf("\"", ts);
          if (te > ts) title = json.substring(ts, te);
        }
        sendChunk("<tr><td>" + htmlEscape(title) + "</td>");
        sendChunk("<td>" + String(pts) + "</td>");
        sendChunk("<td><button class='btn btn-small btn-success' onclick=\"skImportSurvey('" + sid + "','" + htmlEscape(title) + "')\">&#10145; Import</button></td>");
        sendChunk("</tr>");
      }
      sendChunk("</tbody></table></div>");
    }
  }

  // --- Files and points card ---
  sendChunk("<div class='card'><h2>&#128193; Stakeout files</h2>");

  // Load files list
  std::vector<StakeoutFile> files;
  Stakeout::listFiles(files);
  StakeoutActive act = Stakeout::getActive();

  if (files.empty()) {
    sendChunk("<p>No files loaded.</p>");
  } else {
    for (auto& sf : files) {
      bool isActiveFile = (sf.fileId == act.fileId);
      sendChunk("<div style='border:1px solid #ddd;border-radius:6px;padding:12px;margin-bottom:12px;'>");
      sendChunk("<div style='display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px'>");
      sendChunk("<div><b>" + htmlEscape(sf.name) + "</b>");
      sendChunk(" <span class='badge'>" + String(sf.count) + " points</span>");
      if (isActiveFile) sendChunk(" <span class='badge' style='background:#2ecc71;color:#fff'>&#9654; Active</span>");
      sendChunk("</div>");
      sendChunk("<button class='btn btn-small btn-danger' onclick=\"skDelFile('" + sf.fileId + "')\">&#128465; Delete</button>");
      sendChunk("</div>");

      // Load points for this file and show table
      std::vector<StakeoutPoint> pts;
      Stakeout::loadFilePoints(sf.fileId, pts);
      if (!pts.empty()) {
        sendChunk("<div style='overflow-x:auto;margin-top:8px'>");
        sendChunk("<table><thead><tr><th>Name</th><th>Lat</th><th>Lon</th><th>H (m)</th><th>Action</th></tr></thead><tbody>");
        for (auto& pt : pts) {
          bool isActivePt = isActiveFile && (pt.id == act.pointId);
          String rowStyle = isActivePt ? " style='background:#d5f5e3'" : "";
          sendChunk("<tr" + rowStyle + ">");
          sendChunk("<td>" + htmlEscape(pt.name) + (isActivePt ? " <b>&#10003;</b>" : "") + "</td>");
          char latbuf[20], lonbuf[20], hbuf[12];
          snprintf(latbuf, sizeof(latbuf), "%.8f", pt.lat);
          snprintf(lonbuf, sizeof(lonbuf), "%.8f", pt.lon);
          if (isnan(pt.h)) snprintf(hbuf, sizeof(hbuf), "—");
          else              snprintf(hbuf, sizeof(hbuf), "%.3f", pt.h);
          sendChunk("<td>" + String(latbuf) + "</td><td>" + String(lonbuf) + "</td><td>" + String(hbuf) + "</td>");
          sendChunk("<td><button class='btn btn-small' onclick=\"skSetActive('" + sf.fileId + "','" + pt.id + "')\">&#9654; Navigate</button></td>");
          sendChunk("</tr>");
        }
        sendChunk("</tbody></table></div>");
      }
      sendChunk("</div>");
    }
  }
  sendChunk("</div>");

  sendFooter();
}

// ----- Upload handler (multipart) -----
static void handleStakeoutUpload() {
  HTTPUpload& upload = _server->upload();
  if (upload.status == UPLOAD_FILE_START) {
    g_stakeoutUploadContent = "";
    g_stakeoutUploadContent.reserve(4096);
    g_stakeoutUploadError  = false;
    g_stakeoutUploadErrMsg = "";
    g_stakeoutUploadName   = upload.filename;
    Serial.printf("[Stakeout] Upload started: %s\n", upload.filename.c_str());
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if ((int)g_stakeoutUploadContent.length() + (int)upload.currentSize <= STAKEOUT_MAX_UPLOAD) {
      for (size_t i = 0; i < upload.currentSize; i++) {
        g_stakeoutUploadContent += (char)upload.buf[i];
      }
    } else if (!g_stakeoutUploadError) {
      g_stakeoutUploadError  = true;
      g_stakeoutUploadErrMsg = "File troppo grande (max 32KB)";
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    Serial.printf("[Stakeout] Upload end: %d bytes\n", (int)g_stakeoutUploadContent.length());
  }
}

static void handleStakeoutUploadComplete() {
  if (g_stakeoutUploadError) {
    _server->send(400, "text/plain", "Upload error: " + g_stakeoutUploadErrMsg);
    return;
  }
  if (g_stakeoutUploadContent.isEmpty()) {
    _server->send(400, "text/plain", "File vuoto");
    return;
  }

  // Get the user-supplied dataset name from the form
  String dsName = _server->hasArg("name") ? _server->arg("name") : g_stakeoutUploadName;
  if (dsName.isEmpty()) dsName = g_stakeoutUploadName;
  if (dsName.isEmpty()) dsName = "Stakeout";

  // Read optional CSV column hints
  StakeoutCSVHints hints;
  if (_server->hasArg("hint_lat"))  hints.latCol  = _server->arg("hint_lat");
  if (_server->hasArg("hint_lon"))  hints.lonCol  = _server->arg("hint_lon");
  if (_server->hasArg("hint_h"))    hints.hCol    = _server->arg("hint_h");
  if (_server->hasArg("hint_name")) hints.nameCol = _server->arg("hint_name");

  // Detect format by extension
  String fname = g_stakeoutUploadName;
  fname.toLowerCase();
  const uint8_t* buf = (const uint8_t*)g_stakeoutUploadContent.c_str();
  size_t         len = g_stakeoutUploadContent.length();

  String fileId;
  if (fname.endsWith(".csv") || fname.endsWith(".tsv") || fname.endsWith(".txt")) {
    fileId = Stakeout::importCSV(dsName, buf, len, hints);
  } else {
    // Default to GeoJSON for .geojson / .json and anything else
    fileId = Stakeout::importGeoJSON(dsName, buf, len);
    if (fileId.isEmpty()) {
      // Fallback: try CSV
      fileId = Stakeout::importCSV(dsName, buf, len, hints);
    }
  }
  g_stakeoutUploadContent = "";  // free memory

  if (fileId.isEmpty()) {
    _server->send(400, "text/plain", "Unrecognised format or no valid points");
    return;
  }

  _server->sendHeader("Location", "/stakeout");
  _server->send(303);
}

// ----- API: import from existing survey -----
static void handleStakeoutImportSurvey() {
  String body = _server->arg("plain");
  // Parse {"sid":"...","name":"..."}
  auto extractField = [&](const String& key) -> String {
    String search = "\"" + key + "\":\"";
    int ki = body.indexOf(search);
    if (ki < 0) return "";
    int ks = ki + search.length();
    int ke = body.indexOf("\"", ks);
    return (ke > ks) ? body.substring(ks, ke) : "";
  };
  String sid  = extractField("sid");
  String name = extractField("name");
  if (sid.isEmpty()) {
    _server->send(400, "application/json", "{\"error\":\"Missing sid\"}");
    return;
  }
  String fileId = Stakeout::importFromSurvey(sid, name);
  if (fileId.isEmpty()) {
    _server->send(400, "application/json", "{\"error\":\"Import failed — no points or survey not found\"}");
    return;
  }
  _server->send(200, "application/json", "{\"ok\":true,\"fileId\":\"" + fileId + "\"}");
}

// ----- API: list files -----
static void handleStakeoutFiles() {
  std::vector<StakeoutFile> files;
  Stakeout::listFiles(files);
  StakeoutActive act = Stakeout::getActive();
  String json = "[";
  for (size_t i = 0; i < files.size(); i++) {
    if (i > 0) json += ",";
    json += "{\"fileId\":\"" + files[i].fileId + "\","
         +  "\"name\":\""   + files[i].name   + "\","
         +  "\"count\":"    + String(files[i].count) + ","
         +  "\"active\":"   + (files[i].fileId == act.fileId ? "true" : "false") + "}";
  }
  json += "]";
  _server->send(200, "application/json", json);
}

// ----- API: get file (metadata + points) -----
static void handleStakeoutFile() {
  String fileId = _server->hasArg("id") ? _server->arg("id") : "";
  if (fileId.isEmpty()) { _server->send(400, "application/json", "{\"error\":\"Missing id\"}"); return; }

  std::vector<StakeoutPoint> pts;
  if (!Stakeout::loadFilePoints(fileId, pts)) {
    _server->send(404, "application/json", "{\"error\":\"File not found\"}");
    return;
  }
  String json = "{\"fileId\":\"" + fileId + "\",\"points\":[";
  for (size_t i = 0; i < pts.size(); i++) {
    if (i > 0) json += ",";
    char buf[160];
    if (isnan(pts[i].h)) {
      snprintf(buf, sizeof(buf),
               "{\"id\":\"%s\",\"name\":\"%s\",\"lat\":%.9f,\"lon\":%.9f,\"h\":null}",
               pts[i].id.c_str(), pts[i].name.c_str(),
               pts[i].lat, pts[i].lon);
    } else {
      snprintf(buf, sizeof(buf),
               "{\"id\":\"%s\",\"name\":\"%s\",\"lat\":%.9f,\"lon\":%.9f,\"h\":%.4f}",
               pts[i].id.c_str(), pts[i].name.c_str(),
               pts[i].lat, pts[i].lon, pts[i].h);
    }
    json += String(buf);
  }
  json += "]}";
  _server->send(200, "application/json", json);
}

// ----- API: delete file -----
static void handleStakeoutDeleteFile() {
  String fileId;
  // Support both JSON body and query param
  if (_server->hasArg("plain")) {
    String body = _server->arg("plain");
    int fi = body.indexOf("\"fileId\":\"");
    if (fi >= 0) {
      int fs = fi + 10, fe = body.indexOf("\"", fs);
      if (fe > fs) fileId = body.substring(fs, fe);
    }
  }
  if (fileId.isEmpty()) fileId = _server->hasArg("id") ? _server->arg("id") : "";
  if (fileId.isEmpty()) { _server->send(400, "application/json", "{\"error\":\"Missing fileId\"}"); return; }
  bool ok = Stakeout::deleteFile(fileId);
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"Delete failed\"}");
}

// ----- API: set active -----
static void handleStakeoutSetActive() {
  String body    = _server->arg("plain");
  String fileId, pointId;
  auto extract = [&](const String& key) -> String {
    String search = "\"" + key + "\":\"";
    int ki = body.indexOf(search);
    if (ki < 0) return "";
    int ks = ki + search.length();
    int ke = body.indexOf("\"", ks);
    return (ke > ks) ? body.substring(ks, ke) : "";
  };
  fileId  = extract("fileId");
  pointId = extract("pointId");
  if (fileId.isEmpty() || pointId.isEmpty()) {
    _server->send(400, "application/json", "{\"error\":\"Missing fileId or pointId\"}");
    return;
  }
  bool ok = Stakeout::setActive(fileId, pointId);
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"Failed\"}");
}

// ----- API: get active -----
static void handleStakeoutGetActive() {
  StakeoutActive act = Stakeout::getActive();
  String json = "{\"fileId\":\"" + act.fileId + "\",\"pointId\":\"" + act.pointId + "\","
              + "\"valid\":" + (act.valid() ? "true" : "false") + "}";
  _server->send(200, "application/json", json);
}

// ----- API: get navigation status -----
static void handleStakeoutStatus() {
  StakeoutStatus st = Stakeout::getStatus();
  String json = "{";
  json += "\"valid\":"       + String(st.valid ? "true" : "false") + ",";
  if (st.valid) {
    char dbuf[200];
    snprintf(dbuf, sizeof(dbuf),
             "\"d2d\":%.4f,\"az\":%.2f,"
             "\"targetName\":\"%s\",\"targetId\":\"%s\","
             "\"targetLat\":%.9f,\"targetLon\":%.9f,"
             "\"roverCarrSoln\":%d,\"roverFixQuality\":%d",
             st.d2d, st.az,
             st.targetName.c_str(), st.targetId.c_str(),
             st.targetLat, st.targetLon,
             st.roverCarrSoln, st.roverFixQuality);
    json += String(dbuf);
    // dH and targetH: output null when NaN (no height available)
    if (isnan(st.dH)) {
      json += ",\"dH\":null";
    } else {
      char hbuf[32]; snprintf(hbuf, sizeof(hbuf), ",\"dH\":%.4f", st.dH);
      json += String(hbuf);
    }
    if (isnan(st.targetH)) {
      json += ",\"targetH\":null";
    } else {
      char hbuf[32]; snprintf(hbuf, sizeof(hbuf), ",\"targetH\":%.4f", st.targetH);
      json += String(hbuf);
    }
  } else {
    json += "\"d2d\":0,\"dH\":null,\"az\":0,"
            "\"targetName\":\"\",\"targetId\":\"\","
            "\"targetH\":null,"
            "\"roverCarrSoln\":0,\"roverFixQuality\":0";
  }
  json += "}";
  _server->send(200, "application/json", json);
}

// ========================================================================
// PointCodes API handlers
// ========================================================================

static void handleGetCodes() {
  _server->send(200, "application/json", PointCodes::toJSON());
}

static void handlePostCodes() {
  String body = _server->arg("plain");
  if (body.isEmpty()) {
    _server->send(400, "application/json", "{\"error\":\"empty body\"}");
    return;
  }
  if (!PointCodes::fromJSON(body)) {
    _server->send(400, "application/json", "{\"error\":\"invalid JSON\"}");
    return;
  }
  if (!PointCodes::saveToFlash()) {
    _server->send(500, "application/json", "{\"error\":\"save failed\"}");
    return;
  }
  _server->send(200, "application/json", "{\"ok\":true}");
}

static void handleResetCodes() {
  PointCodes::resetToDefaults();
  _server->send(200, "application/json", "{\"ok\":true}");
}


// ========================================================================
// BEGIN FUNCTION - ROUTE REGISTRATION
// ========================================================================

void WebUI::begin(SdFat& sd, WebServer& server) {
  _sd = &sd; 
  _server = &server;

  // Main pages
  _server->on("/", handleRoot);
  _server->on("/rover", HTTP_GET, handleRoverPage);
  _server->on("/base-cfg", HTTP_GET, handleBasePage);
  _server->on("/settings", HTTP_GET, handleSettingsPage);
  _server->on("/firmware", HTTP_GET, handleFirmwarePage);
  _server->on("/logs", HTTP_GET, handleLogsPage);
  _server->on("/logs/deleteall", HTTP_GET, handleDeleteAllLogs);
  _server->on("/survey", HTTP_GET, handleSurveyPage);
  _server->on("/stakeout", HTTP_GET, handleStakeoutPage);
  
  // CSS and API
  _server->on("/css", HTTP_GET, handleCSS);
  _server->on("/api/status", HTTP_GET, handleApiStatus);
  _server->on("/api/position", HTTP_GET, handleApiPosition);
  _server->on("/api/rtcm", HTTP_GET, handleApiRtcm);
  _server->on("/api/antennas", HTTP_GET, handleApiAntennas);
  _server->on("/api/bases", HTTP_GET, handleApiBasesIdx);
  _server->on("/api/zed/tmode", HTTP_GET, handleApiZedTmode);
  _server->on("/api/zed/tmode/refresh", HTTP_GET, handleZedTmodeRefresh);
  _server->on("/api/zed/reset", HTTP_GET, handleZedReset);
  _server->on("/api/switchToRover", HTTP_GET, handleSwitchToRover);
  _server->on("/api/config/export", HTTP_GET, handleConfigExport);
  _server->on("/api/config/import", HTTP_POST, handleConfigImport);
  _server->on("/api/config/sync",   HTTP_GET, handleConfigSync);
  _server->on("/api/factory-reset", HTTP_GET, handleFactoryReset);
  
  // Survey API (base position averaging — existing)
  _server->on("/api/survey/start", HTTP_GET, handleSurveyStart);
  _server->on("/api/survey/stop", HTTP_GET, handleSurveyStop);
  _server->on("/api/survey/status", HTTP_GET, handleSurveyStatus);
  _server->on("/api/survey/save", HTTP_POST, handleSurveySave);

  // Survey points API (new feature)
  _server->on("/api/pts/list",            HTTP_GET,  handlePtsList);
  _server->on("/api/pts/create",          HTTP_POST, handlePtsCreate);
  _server->on("/api/pts/delete",          HTTP_POST, handlePtsDelete);
  _server->on("/api/pts/setactive",       HTTP_GET,  handlePtsSetActive);
  _server->on("/api/pts/quality",         HTTP_GET,  handlePtsQuality);
  _server->on("/api/pts/measure",         HTTP_GET,  handlePtsMeasure);
  _server->on("/api/pts/measure/status",  HTTP_GET,  handlePtsMeasureStatus);
  _server->on("/api/pts/points",          HTTP_GET,  handlePtsPoints);
  _server->on("/api/pts/point/delete",    HTTP_POST, handlePtsPointDelete);
  _server->on("/api/pts/download",        HTTP_GET,  handlePtsDownload);
  _server->on("/api/pts/download/csv",    HTTP_GET,  handlePtsDownloadCSV);
  _server->on("/api/pts/sync",            HTTP_GET,  handlePtsSync);

  // Stakeout API
  _server->on("/api/stakeout/files",       HTTP_GET,  handleStakeoutFiles);
  _server->on("/api/stakeout/file",        HTTP_GET,  handleStakeoutFile);
  _server->on("/api/stakeout/file/delete", HTTP_POST, handleStakeoutDeleteFile);
  _server->on("/api/stakeout/active",      HTTP_GET,  handleStakeoutGetActive);
  _server->on("/api/stakeout/active",      HTTP_POST, handleStakeoutSetActive);
  _server->on("/api/stakeout/status",      HTTP_GET,  handleStakeoutStatus);
  _server->on("/api/stakeout/upload",      HTTP_POST, handleStakeoutUploadComplete, handleStakeoutUpload);
  _server->on("/api/stakeout/import-survey", HTTP_POST, handleStakeoutImportSurvey);
  
  // PointCodes API
  _server->on("/api/codes",       HTTP_GET,  handleGetCodes);
  _server->on("/api/codes",       HTTP_POST, handlePostCodes);
  _server->on("/api/codes/reset", HTTP_POST, handleResetCodes);
  
  // Compatibility redirects
  _server->on("/wifi", HTTP_GET, [](){ _server->sendHeader("Location", "/settings"); _server->send(303); });
  _server->on("/ntp", HTTP_GET, [](){ _server->sendHeader("Location", "/settings"); _server->send(303); });
  _server->on("/ntrip", HTTP_GET, [](){ _server->sendHeader("Location", "/rover"); _server->send(303); });
  _server->on("/base", HTTP_GET, [](){ _server->sendHeader("Location", "/base-cfg"); _server->send(303); });
  _server->on("/bases", HTTP_GET, [](){ _server->sendHeader("Location", "/base-cfg"); _server->send(303); });
  _server->on("/baseout", HTTP_GET, [](){ _server->sendHeader("Location", "/base-cfg"); _server->send(303); });
  _server->on("/base/out", HTTP_GET, [](){ _server->sendHeader("Location", "/base-cfg"); _server->send(303); });
  // TCP OUT CLIENT endpoints
  _server->on("/tcpclient/add", HTTP_POST, handleTcpClientAdd);
  _server->on("/tcpclient/edit", HTTP_GET, handleTcpClientEditForm);
  _server->on("/tcpclient/edit", HTTP_POST, handleTcpClientEditSave);
  _server->on("/tcpclient/del", HTTP_GET, handleTcpClientDel);
  _server->on("/tcpclient/select", HTTP_GET, handleTcpClientSelect);
  _server->on("/tcpclient/start", HTTP_GET, handleTcpClientStart);
  _server->on("/tcpclient/stop", HTTP_GET, handleTcpClientStop);

  _server->on("/lanin", HTTP_GET, [](){ _server->sendHeader("Location", "/rover"); _server->send(303); });
  _server->on("/rate", HTTP_GET, [](){ _server->sendHeader("Location", "/settings"); _server->send(303); });

  // Wi-Fi CRUD
  _server->on("/wifi/add", HTTP_POST, handleWifiAdd);
  _server->on("/wifi/del", HTTP_GET, handleWifiDel);
  _server->on("/wifi/edit", HTTP_GET, handleWifiEdit);
  _server->on("/wifi/update", HTTP_POST, handleWifiUpdate);

  // Antenna CRUD
  _server->on("/antennas/add", HTTP_POST, handleAntennaAdd);
  _server->on("/antennas/del", HTTP_GET, handleAntennaDel);
  _server->on("/antennas/edit", HTTP_GET, handleAntennaEdit);
  _server->on("/antennas/update", HTTP_POST, handleAntennaUpdate);

  // NTP
  _server->on("/ntp/save",    HTTP_POST, handleNtpSave);
  _server->on("/ntp/sync",    HTTP_GET,  handleNtpSync);
  _server->on("/ntp/tz/save", HTTP_POST, handleNtpTzSave);

  // mDNS
  _server->on("/mdns/save", HTTP_POST, handleMdnsSave);

  // BLE
  _server->on("/settings/ble", HTTP_POST, handleBleSettings);
  _server->on("/settings/blertcm", HTTP_POST, handleBleRtcmSettings);

  // BLE RTCM API (start/stop streaming + status)
  _server->on("/api/blertcm/start", HTTP_GET, []() {
    if (!g_bleRtcmEnabled) {
      // Not enabled yet — try to start the whole connection
      if (String(g_bleRtcmTargetName).length() > 0) {
        startBleRtcm(String(g_bleRtcmTargetName), g_bleRtcmPasskey);
        _server->send(200, "text/plain", "BLE RTCM started");
      } else {
        _server->send(400, "text/plain", "BLE RTCM target not configured");
      }
      return;
    }
    // Already enabled — send START command to rtcm-lora
    if (g_bleRtcm.isConnected()) {
      if (g_bleRtcm.sendStart()) {
        _server->send(200, "text/plain", "RTCM streaming started");
      } else {
        _server->send(500, "text/plain", "Failed to send START command");
      }
    } else {
      _server->send(200, "text/plain", "BLE RTCM enabled, waiting for connection...");
    }
  });

  _server->on("/api/blertcm/stop", HTTP_GET, []() {
    if (!g_bleRtcmEnabled) {
      _server->send(200, "text/plain", "BLE RTCM already inactive");
      return;
    }
    // Send STOP command (connection stays alive)
    if (g_bleRtcm.isConnected()) {
      g_bleRtcm.sendStop();
    }
    _server->send(200, "text/plain", "RTCM streaming stopped");
  });

  _server->on("/api/blertcm/status", HTTP_GET, []() {
    String json = "{\"enabled\":";
    json += g_bleRtcmEnabled ? "true" : "false";
    json += ",\"connected\":";
    json += (g_bleRtcmEnabled && g_bleRtcm.isConnected()) ? "true" : "false";
    json += ",\"streaming\":";
    json += (g_bleRtcmEnabled && g_bleRtcm.isStreaming()) ? "true" : "false";
    json += ",\"scanning\":";
    json += (g_bleRtcmEnabled && g_bleRtcm.isScanning()) ? "true" : "false";
    json += ",\"target\":\"" + htmlEscape(String(g_bleRtcmTargetName)) + "\"";
    json += ",\"rx_bytes\":" + String((unsigned long)g_bleRtcm.getRxBytes());
    json += ",\"rx_chunks\":" + String((unsigned long)g_bleRtcm.getRxChunks());
    json += "}";
    _server->send(200, "application/json", json);
  });

  // NTRIP IN CRUD
  _server->on("/ntrip/add", HTTP_POST, handleNtripAdd);
  _server->on("/ntrip/del", HTTP_GET, handleNtripDel);
  _server->on("/ntrip/edit", HTTP_GET, handleNtripEdit);
  _server->on("/ntrip/update", HTTP_POST, handleNtripUpdate);
  _server->on("/ntrip/select", HTTP_GET, handleNtripSelect);
  _server->on("/ntrip/toggle", HTTP_GET, [](){
    if (_server->hasArg("enable")) {
      bool enable = _server->arg("enable") == "1";
      toggleNtrip(enable);
      _server->send(200, "text/plain", String("NTRIP ") + (enable ? "enabled" : "disabled"));
    } else {
      _server->send(400, "text/plain", "Parameter 'enable' missing");
    }
  });

  // ZED rate
  _server->on("/setrate", HTTP_POST, handleRateSubmit);

  // TCP Stream mode (viewer)
  _server->on("/stream", HTTP_GET, [](){
    if (_server->hasArg("mode")) {
      String m = _server->arg("mode");
      if (m == "raw") setStreamModeRaw();
      else            setStreamModeNmea();
      _server->send(200, "text/plain", String("TCP stream: ") + getStreamModeName());
    } else {
      _server->send(200, "text/plain", String("Actual mode: ") + getStreamModeName());
    }
  });

  // BASE (VALSET)
  _server->on("/base/llh", HTTP_POST, handleBaseLLH);
  _server->on("/base/stop", HTTP_GET, handleBaseStop);

  // BASES CRUD
  _server->on("/bases/add", HTTP_POST, handleBasesAdd);
  _server->on("/bases/del", HTTP_GET, handleBasesDel);
  _server->on("/bases/edit", HTTP_GET, handleBasesEdit);
  _server->on("/bases/update", HTTP_POST, handleBasesUpdate);
  _server->on("/bases/start", HTTP_GET, handleBasesStart);
  _server->on("/bases/start-confirm", HTTP_POST, handleBasesStartConfirm);

  // BASE OUT CRUD
  _server->on("/baseout/add", HTTP_POST, handleBaseOutAdd);
  _server->on("/baseout/del", HTTP_GET, handleBaseOutDel);
  _server->on("/baseout/edit", HTTP_GET, handleBaseOutEdit);
  _server->on("/baseout/update", HTTP_POST, handleBaseOutUpdate);
  _server->on("/baseout/select", HTTP_GET, handleBaseOutSelect);
  _server->on("/baseout/start", HTTP_GET, handleBaseOutStart);
  _server->on("/baseout/stop",  HTTP_GET, handleBaseOutStop);

  // LAN IN (TCP) CRUD
  _server->on("/lanin/add",    HTTP_POST, handleLanInAdd);
  _server->on("/lanin/del",    HTTP_GET,  handleLanInDel);
  _server->on("/lanin/edit",   HTTP_GET,  handleLanInEdit);
  _server->on("/lanin/update", HTTP_POST, handleLanInUpdate);
  _server->on("/lanin/select", HTTP_GET,  handleLanInSelect);
  _server->on("/lanin/start",  HTTP_GET,  handleLanInStart);
  _server->on("/lanin/stop",   HTTP_GET,  handleLanInStop);

  // RAW LOG endpoints
  _server->on("/log/start", HTTP_GET, [](){
    startLogging();
    _server->send(200, "text/plain", "RAW log STARTED");
  });
  _server->on("/log/stop", HTTP_GET, [](){
    stopLogging();
    _server->send(200, "text/plain", "RAW log STOPPED");
  });

  // Reboot endpoint
  _server->on("/reboot", HTTP_GET, [](){
    stopLogging();
    _server->send(200, "text/plain", "Restarting ESP32...");
    delay(250);
    ESP.restart();
    while (true) { delay(100); }
  });
  
  // ===== NEW: System Log API =====
  _server->on("/api/logs", HTTP_GET, [](){
    String files[SYSTEM_LOG_MAX_FILES];
    int count = 0;
    
    String json = "{\"files\":[";
    if (g_systemLog && g_systemLog->listLogFiles(files, count, SYSTEM_LOG_MAX_FILES)) {
      for (int i = 0; i < count; i++) {
        if (i > 0) json += ",";
        json += "\"" + files[i] + "\"";
      }
    }
    json += "]}";
    _server->send(200, "application/json", json);
  });
  
  _server->on("/api/logs/download", HTTP_GET, [](){
    if (!_server->hasArg("file")) {
      _server->send(400, "text/plain", "Missing file parameter");
      return;
    }
    
    String filename = _server->arg("file");
    if (!isValidLogPath(filename)) {
      _server->send(403, "text/plain", "Invalid path");
      return;
    }
    SdLockGuard guard(3000);
    if (!guard.locked) {
      _server->send(503, "text/plain", "SD busy");
      return;
    }
    
    FsFile logFile = _sd->open(filename.c_str(), FILE_READ);
    if (!logFile) {
      _server->send(404, "text/plain", "File not found");
      return;
    }
    
    _server->setContentLength(logFile.size());
    _server->send(200, "text/plain", "");
    
    uint8_t buf[512];
    while (logFile.available()) {
      size_t n = logFile.read(buf, sizeof(buf));
      _server->client().write(buf, n);
    }
    logFile.close();
  });
  
  _server->on("/api/logs/delete", HTTP_GET, [](){
    if (!_server->hasArg("file")) {
      _server->send(400, "text/plain", "Missing file parameter");
      return;
    }
    
    String filename = _server->arg("file");
    if (!isValidLogPath(filename)) {
      _server->send(403, "text/plain", "Invalid path");
      return;
    }
    
    if (g_systemLog && g_systemLog->deleteLogFile(filename)) {
      _server->sendHeader("Location", "/settings");
      _server->send(303, "text/plain", "Deleted");
    } else {
      _server->send(500, "text/plain", "Failed to delete");
    }
  });
  
  // ===== NEW: Audio/Buzzer Settings =====
  _server->on("/audio/save", HTTP_POST, [](){
    bool enabled = _server->hasArg("enabled") && _server->arg("enabled") == "1";
    
    if (g_buzzer) {
      g_buzzer->setEnabled(enabled);
    }
    
    _server->sendHeader("Location", "/settings");
    _server->send(303, "text/plain", "Audio settings saved");
  });
  
  _server->on("/audio/upload", HTTP_POST, handleAudioUploadComplete, handleAudioUpload);

  // ===== Firmware OTA route =====
  _server->on("/firmware/upload", HTTP_POST, handleFirmwareUploadComplete, handleFirmwareUpload);

  // ===== Firmware OTA routes =====
  _server->on("/firmware/upload", HTTP_POST, handleFirmwareUploadComplete, handleFirmwareUpload);

  // Files
  _server->on("/download", HTTP_GET, handleDownload);
  _server->on("/delete",   HTTP_GET, handleDelete);

  _server->begin();
}

// ========================================================================
// LOADER FUNCTIONS (used by main.cpp)
// ========================================================================

bool WebUI::loadNtrip(SdFat& sd, String& host, int& port,
                      String& mountpoint, String& user, String& pass) {
  // 1) Try flash IN list + LAST
  std::vector<NtripIn> v; int last=-1;
  loadNtripInList(v, last);
  if (last>=0 && last<(int)v.size()){
    host=v[last].host; port=v[last].port; mountpoint=v[last].mount; user=v[last].user; pass=v[last].pass;
    return true;
  }
  // 2) Fallback to ntrip.txt in flash
  String content = FlashConfig::readFile("/config/ntrip.txt");
  if (content.length() > 0) {
    int start = 0;
    while (start < (int)content.length()) {
      int endPos = content.indexOf('\n', start);
      if (endPos < 0) endPos = content.length();
      String line = content.substring(start, endPos);
      line.trim();
      start = endPos + 1;
      int sep = line.indexOf('='); if (sep < 0) continue;
      String key = line.substring(0, sep);
      String value = line.substring(sep + 1); value.trim();
      if (key == "ntrip_host") host = value;
      else if (key == "ntrip_port") port = value.toInt();
      else if (key == "mountpoint") mountpoint = value;
      else if (key == "ntrip_user") user = value;
      else if (key == "ntrip_password") pass = value;
    }
    if (host.length()>0 && mountpoint.length()>0 && port>0) return true;
  }
  // 3) Fallback old config.txt
  return loadOldConfigForNtrip(sd, host, port, mountpoint, user, pass);
}

bool WebUI::loadOldConfigForNtrip(SdFat& sd, String& host, int& port,
                                  String& mountpoint, String& user, String& pass) {
  // Try flash config.txt first
  String content = FlashConfig::readFile("/config/config.txt");
  if (content.length() == 0) {
    // Fall back to SD for legacy compatibility
    FsFile config = sd.open("/gnss/config.txt", FILE_READ);
    if (!config) return false;
    while (config.available()) {
      content += (char)config.read();
    }
    config.close();
  }
  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    int sep = line.indexOf('=');
    if (sep < 0) continue;
    String key = line.substring(0, sep);
    String value = line.substring(sep + 1); value.trim();
    if (key == "ntrip_host") host = value;
    else if (key == "ntrip_port") port = value.toInt();
    else if (key == "mountpoint") mountpoint = value;
    else if (key == "ntrip_user") user = value;
    else if (key == "ntrip_password") pass = value;
  }
  return host.length() > 0 && mountpoint.length() > 0 && port > 0;
}

bool WebUI::loadNtripOut(SdFat& sd, String& host, int& port, String& mount, String& pass){
  host=""; mount=""; pass=""; port=2101; return false;
}

bool WebUI::saveNtripOut(SdFat& sd, const String& host, int port, const String& mount, const String& pass){
  (void)sd; (void)host; (void)port; (void)mount; (void)pass; return false;
}

bool WebUI::loadTcpIn(SdFat& sd, String& host, int& port) {
  // Read from flash
  String content = FlashConfig::readFile("/config/tcp_in_lista.txt");
  if (content.length() == 0) return false;

  int lastIdx = -1;
  // First pass: find LAST=
  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (!line.length()) continue;
    if (line.startsWith("#")) {
      int p = line.indexOf("LAST=");
      if (p >= 0) lastIdx = line.substring(p + 5).toInt();
    }
  }
  if (lastIdx < 0) return false;

  // Second pass: get record at lastIdx
  int recIdx = 0;
  start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (!line.length() || line.startsWith("#")) continue;
    int p1 = line.indexOf(';');
    int p2 = line.indexOf(';', p1 + 1);
    if (p1 > 0 && p2 > p1) {
      if (recIdx == lastIdx) {
        host = line.substring(p1 + 1, p2);
        port = line.substring(p2 + 1).toInt();
        return (host.length() > 0 && port > 0);
      }
      recIdx++;
    }
  }
  return false;
}

