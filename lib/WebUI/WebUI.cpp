// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "WebUI.h"
#include "config.h"


// OTA Manager
#include "OTAManager.h"

// BLE Serial (for settings page)
#include "BLESerial.h"

// Flash config (LittleFS-based config storage)
#include "FlashConfig.h"

// Time sync globals (defined in main.cpp)
extern char g_ntpServer[64];
extern char g_ntpTz[64];
extern char g_deviceName[32];
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

// EXTINT marker flag (defined in main.cpp) — PPK event marking via GPIO6
extern volatile bool g_extintMarkerEnabled;

// PointCodes (point code/category library)
#include "PointCodes.h"

// Stakeout (navigate to design points)
#include "Stakeout.h"
#include "TrackRecorder.h"

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

extern bool loadBleName(char* out, size_t maxLen);
extern bool saveBleName(const char* name);
extern bool loadBlePin(uint32_t* out);
extern bool saveBlePin(uint32_t pin);
extern bool applyBleName(const char* newName);

// ===== ESP-NOW RTCM mesh (defined in main.cpp) =====
#include "EspNowRtcm.h"
extern EspNowRtcm g_espNow;
extern bool g_espNowEnabled;
extern bool g_espNowTxEnabled;
extern uint8_t g_espNowChannel;
extern uint16_t g_espNowRelayNodeId;
extern bool startEspNowRx();
extern void stopEspNowRx();
extern bool startEspNowTx();
extern void stopEspNowTx();
extern void applyEspNowConfigFromFlash();

// Stream mode extern
extern "C" {
  void setStreamModeRaw();
  void setStreamModeNmea();
  const char* getStreamModeName();
}

// --- RAW logging controls (defined in main.cpp) ---
extern void startLogging();
extern void stopLogging();
extern void switchToApModeNow();
extern volatile bool g_apMode;
extern char g_apSsid[33];
extern char g_apPass[64];
extern bool saveApConfig(const char* ssid, const char* pass);
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
.header { background: #2c3e50; color: white; padding: 12px 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); display: flex; align-items: center; gap: 14px; }
.header h1 { font-size: 22px; font-weight: 600; flex: 1; min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.hamburger-btn { background: transparent; border: none; color: white; font-size: 26px; line-height: 1; padding: 0 4px; margin: 0; width: auto; min-height: 0; flex: none; cursor: pointer; }
.hamburger-btn:hover { background: transparent; opacity: 0.8; }
.nav-overlay { display: none; position: fixed; inset: 0; background: rgba(0,0,0,0.5); z-index: 1000; }
.nav-overlay.open { display: block; }
.nav-drawer { position: fixed; top: 0; left: 0; bottom: 0; width: 260px; max-width: 80vw; background: #34495e; box-shadow: 2px 0 8px rgba(0,0,0,0.3); z-index: 1001; transform: translateX(-100%); transition: transform 0.25s ease; overflow-y: auto; }
.nav-drawer.open { transform: translateX(0); }
.nav-drawer-header { display: flex; align-items: center; justify-content: space-between; padding: 14px 16px; color: white; font-weight: 600; font-size: 16px; border-bottom: 1px solid rgba(255,255,255,0.15); }
.nav-close-btn { background: transparent; border: none; color: white; font-size: 22px; line-height: 1; padding: 0 4px; margin: 0; width: auto; min-height: 0; cursor: pointer; }
.nav-close-btn:hover { background: transparent; opacity: 0.8; }
.nav-drawer a { display: flex; align-items: center; gap: 10px; color: white; padding: 14px 16px; text-decoration: none; transition: background 0.2s; font-size: 15px; }
.nav-drawer a:hover, .nav-drawer a.active { background: #2c3e50; }
.nav-drawer .nav-icon { font-size: 18px; line-height: 1; width: 20px; text-align: center; }
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
/* Settings hub grid */
.settings-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; margin-top: 8px; }
.settings-card { display: flex !important; align-items: center; text-decoration: none !important; color: #2c3e50 !important; background: #fff; border-radius: 10px; padding: 20px 20px 20px 22px; box-shadow: 0 3px 10px rgba(0,0,0,0.12); border-left: 6px solid #3498db; transition: box-shadow 0.2s, transform 0.15s; cursor: pointer; }
.settings-card:hover { box-shadow: 0 6px 20px rgba(0,0,0,0.18); transform: translateY(-2px); text-decoration: none !important; }
.settings-card-body { flex: 1; min-width: 0; }
.settings-card-title { font-size: 16px; font-weight: 700; color: #2c3e50; display: block; margin-bottom: 5px; }
.settings-card-summary { font-size: 12px; color: #95a5a6; line-height: 1.5; display: block; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.settings-card-arrow { color: #bdc3c7; font-size: 28px; font-weight: 300; margin-left: 12px; flex-shrink: 0; line-height: 1; }
.settings-hub-title { color: #2c3e50; font-size: 20px; margin-bottom: 4px; }
.settings-hub-sub { color: #7f8c8d; font-size: 13px; margin-bottom: 18px; }
.settings-breadcrumb { margin-bottom: 16px; }
@media screen and (max-width: 600px) { .settings-grid { grid-template-columns: 1fr; } }
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
  sendChunk("<link rel='stylesheet' href='/css?v=4'>");
  sendChunk("</head><body>");
  
  // Header
  sendChunk("<div class='header'>");
  sendChunk("<button class='hamburger-btn' onclick='toggleNav()' aria-label='Menu'>&#9776;</button>");
  sendChunk("<h1>");
  sendChunk(g_deviceName[0] ? g_deviceName : "RTKino");
  sendChunk(" &ndash; ");
  sendChunk(title);
  sendChunk("</h1></div>");

  // Navigation drawer (hamburger-triggered, replaces the old top tab bar on all screen sizes)
  sendChunk("<div id='navOverlay' class='nav-overlay' onclick='closeNav()'></div>");
  sendChunk("<div id='navDrawer' class='nav-drawer'>");
  sendChunk("<div class='nav-drawer-header'><span>Menu</span><button class='nav-close-btn' onclick='closeNav()' aria-label='Close'>&times;</button></div>");

  String active = String(activePage);

  String nav = "<a href='/'";
  if (active == "home") nav += " class='active'";
  nav += "><span class='nav-icon'>⌂</span><span class='nav-text'>Home</span></a>";
  sendChunk(nav);

  nav = "<a href='/survey'";
  if (active == "survey") nav += " class='active'";
  nav += "><span class='nav-icon'>&#128205;</span><span class='nav-text'>Survey</span></a>";
  sendChunk(nav);

  nav = "<a href='/stakeout'";
  if (active == "stakeout") nav += " class='active'";
  nav += "><span class='nav-icon'>&#127919;</span><span class='nav-text'>Stakeout</span></a>";
  sendChunk(nav);

  nav = "<a href='/tracking'";
  if (active == "tracking") nav += " class='active'";
  nav += "><span class='nav-icon'>&#128663;</span><span class='nav-text'>Tracking</span></a>";
  sendChunk(nav);

  nav = "<a href='/rover'";
  if (active == "rover") nav += " class='active'";
  nav += "><span class='nav-icon'>&#128225;</span><span class='nav-text'>RTCM IN</span></a>";
  sendChunk(nav);

  nav = "<a href='/base-cfg'";
  if (active == "base") nav += " class='active'";
  nav += "><span class='nav-icon'>&#128225;</span><span class='nav-text'>Base</span></a>";
  sendChunk(nav);

  nav = "<a href='/settings'";
  if (active == "settings") nav += " class='active'";
  nav += "><span class='nav-icon'>⚙</span><span class='nav-text'>Settings</span></a>";
  sendChunk(nav);

  sendChunk("</div>");
  sendChunk("<script>");
  sendChunk("function toggleNav(){document.getElementById('navDrawer').classList.toggle('open');document.getElementById('navOverlay').classList.toggle('open');}");
  sendChunk("function closeNav(){document.getElementById('navDrawer').classList.remove('open');document.getElementById('navOverlay').classList.remove('open');}");
  sendChunk("</script>");
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
static std::vector<NtripIn> s_ntripInCache;
static int s_ntripInLastIdx = -1;
static bool s_ntripInCacheValid = false;

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
static std::vector<TcpIn> s_tcpInCache;
static int s_tcpInLastIdx = -1;
static bool s_tcpInCacheValid = false;

// ========================================================================
// LOAD/SAVE FUNCTIONS FOR DATA FILES
// ========================================================================

static void loadBases(std::vector<BaseRec>& out, int& lastIdx){
  out.clear(); lastIdx = -1;
  String content = FlashConfig::readFile(basesFilePath().c_str());
  if (content.length() == 0) return;

  int start = 0;
  while (start < (int)content.length()) {
    int endPos = content.indexOf('\n', start);
    if (endPos < 0) endPos = content.length();
    String line = content.substring(start, endPos);
    line.trim();
    start = endPos + 1;
    if (line.length() == 0) continue;
    if (line[0] == '#') {
      int p = line.indexOf("LAST=");
      if (p >= 0) lastIdx = line.substring(p + 5).toInt();
      continue;
    }
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

static void loadBases(std::vector<BaseRec>& out) { int dummy; loadBases(out, dummy); }

static bool saveBases(const std::vector<BaseRec>& v, int lastIdx = -1){
  String content = String("# LAST=") + lastIdx + "\n";
  content += "# name;lat;lon;altGround;stid;hARP;antennaIdx;rtcmType\n";
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
// BASE AUTO-START CONFIG
// ========================================================================

struct BaseAutoStart { bool ntrip=false, tcp=false; };
static String baseAutoStartPath(){ return "/config/base_autostart.txt"; }

static BaseAutoStart loadBaseAutoStart(){
  BaseAutoStart s;
  String content = FlashConfig::readFile(baseAutoStartPath().c_str());
  int pos=0;
  while (pos < (int)content.length()) {
    int e=content.indexOf('\n',pos); if(e<0) e=content.length();
    String line=content.substring(pos,e); line.trim(); pos=e+1;
    if(line.startsWith("NTRIP=")) s.ntrip = line.substring(6).toInt() != 0;
    else if(line.startsWith("TCP=")) s.tcp = line.substring(4).toInt() != 0;
  }
  return s;
}

static bool saveBaseAutoStart(const BaseAutoStart& s){
  String content = String("NTRIP=") + (s.ntrip?1:0) + "\n";
  content += String("TCP=") + (s.tcp?1:0) + "\n";
  return FlashConfig::writeFile(baseAutoStartPath().c_str(), content);
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
  if (!s_ntripInCacheValid) {
    s_ntripInCache.clear(); s_ntripInLastIdx = -1;
    String content = FlashConfig::readFile(ntripInListPath().c_str());
    if (content.length() > 0) {
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
          if (p>=0){ s_ntripInLastIdx = line.substring(p+5).toInt(); }
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
        s_ntripInCache.push_back(n);
      }
    }
    s_ntripInCacheValid = true;
  }
  out = s_ntripInCache;
  lastIdx = s_ntripInLastIdx;
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
  bool ok = FlashConfig::writeFile(ntripInListPath().c_str(), content);
  if (ok) { s_ntripInCache = v; s_ntripInLastIdx = lastIdx; s_ntripInCacheValid = true; }
  return ok;
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
  if (!s_tcpInCacheValid) {
    s_tcpInCache.clear(); s_tcpInLastIdx = -1;
    String content = FlashConfig::readFile(tcpInListPath().c_str());
    if (content.length() > 0) {
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
          if (p>=0){ s_tcpInLastIdx = line.substring(p+5).toInt(); }
          continue;
        }
        int p1=line.indexOf(';'); if (p1<0) continue;
        int p2=line.indexOf(';',p1+1); if (p2<0) continue;
        TcpIn t; t.name = line.substring(0,p1);
        t.host = line.substring(p1+1,p2);
        t.port = line.substring(p2+1).toInt();
        s_tcpInCache.push_back(t);
      }
    }
    s_tcpInCacheValid = true;
  }
  out = s_tcpInCache;
  lastIdx = s_tcpInLastIdx;
  return (int)out.size();
}

static bool saveTcpInList(const std::vector<TcpIn>& v, int lastIdx){
  String content = String("# LAST=") + lastIdx + "\n";
  content += "# name;host;port\n";
  for (auto& t: v){
    content += clampSemi(t.name); content += ';'; content += t.host;
    content += ';'; content += t.port; content += '\n';
  }
  bool ok = FlashConfig::writeFile(tcpInListPath().c_str(), content);
  if (ok) { s_tcpInCache = v; s_tcpInLastIdx = lastIdx; s_tcpInCacheValid = true; }
  return ok;
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

static void handleApiRtcmReset() {
  resetRtcmStats();
  _server->send(200, "text/plain", "OK");
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
    
    // Load existing bases (preserve lastIdx)
    int baseLast=-1; std::vector<BaseRec> v;
    loadBases(v, baseLast);
    uint16_t selStid = (baseLast>=0&&baseLast<(int)v.size()) ? v[baseLast].stid : 0;
    v.push_back(b);

    // Sort by STID
    std::sort(v.begin(), v.end(), [](const BaseRec&a, const BaseRec&b){return a.stid<b.stid;});
    if (selStid>0) { baseLast=-1; for(int i=0;i<(int)v.size();i++) if(v[i].stid==selStid){baseLast=i;break;} }

    // Save to file
    if (!saveBases(v, baseLast)) {
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
  ntripStat += "<button onclick='toggleNtrip(1)' style='background-color:#2ecc71;color:white;border:none;padding:7.2px 14.4px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:1.08em;' title='Enable NTRIP' aria-label='Enable NTRIP'>✓</button>";
  ntripStat += "<button onclick='toggleNtrip(0)' style='background-color:#e74c3c;color:white;border:none;padding:7.2px 14.4px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:1.08em;' title='Disable NTRIP' aria-label='Disable NTRIP'>✕</button>";
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
  tcpInStat += "<button onclick='startTcpIn()' style='background-color:#2ecc71;color:white;border:none;padding:7.2px 14.4px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:1.08em;' title='Start TCP IN' aria-label='Start TCP IN'>✓</button>";
  tcpInStat += "<button onclick='stopTcpIn()' style='background-color:#e74c3c;color:white;border:none;padding:7.2px 14.4px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:1.08em;' title='Stop TCP IN' aria-label='Stop TCP IN'>✕</button>";
  tcpInStat += "</span>";
  tcpInStat += "</div>";
  sendChunk(tcpInStat);
  
  // Logging status
  String logStat = "<div class='status-row'><span class='status-led ";
  logStat += loggingActive ? "led-on" : "led-off";
  logStat += "'></span><strong>RAW Log:</strong> ";
  logStat += loggingActive ? "Recording" : "Stopped";
  // Inline control buttons
  logStat += "<span style='float:right;'>";
  logStat += "<button onclick='startLog()' style='background-color:#2ecc71;color:white;border:none;padding:7.2px 14.4px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:1.08em;' title='Start Log' aria-label='Start Log'>✓</button>";
  logStat += "<button onclick='stopLog()' style='background-color:#e74c3c;color:white;border:none;padding:7.2px 14.4px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:1.08em;' title='Stop Log' aria-label='Stop Log'>✕</button>";
  logStat += "<button onclick='location.href=\"/logs\"' class='btn-secondary' style='padding:7.2px 14.4px;border-radius:4px;cursor:pointer;margin-left:4px;font-size:1.08em;' title='View Log Files' aria-label='View Log Files'>📁 Logs</button>";
  logStat += "</span>";
  logStat += "</div>";
  sendChunk(logStat);
  
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
  sendChunk("<button onclick=\"syncNtp()\" style=\"background:none;border:1px solid #95a5a6;border-radius:4px;cursor:pointer;margin-left:8px;padding:7.2px 14.4px;font-size:1.08em;\" title=\"Sync NTP Now\" aria-label=\"Sync NTP\">🔄</button>");
  sendChunk("</div>");
  
  // ZED-F9P TMODE status — always read fresh from ZED when serving Home page
  // (single I2C call, ~5 ms; Home page is user-initiated, not a hot path)
  ZedTmodeState tmode;
  if (UbxVal::getTmodeState(tmode) && tmode.valid) {
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
    // Add "Switch to Rover" button only in BASE mode
    if (tmode.mode == 2) {
      sendChunk("<button onclick=\"switchToRover()\" style=\"margin-left:8px;background-color:#f39c12;color:white;border:none;padding:7.2px 14.4px;border-radius:4px;cursor:pointer;font-size:1.08em;\" title=\"Switch to Rover Mode\">Rover</button>");
    }
    sendChunk("</div>");
  } else {
    sendChunk("<div class='status-row'><strong>ZED-F9P Mode:</strong> ");
    sendChunk("<span style='color:#95a5a6'>Unknown</span>");
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
  sendChunk("<h2>📡 RTCM Stream <button onclick='resetRtcm()' class='btn btn-small btn-secondary' style='float:right;margin-top:-4px;' title='Azzera conteggio messaggi RTCM'>&#x21BA; Reset</button></h2>");
  sendChunk("<div id='rtcm-status'>");
  sendChunk("<p>Loading...</p>");
  sendChunk("</div>");
  sendChunk("</div>");

  // ESP-NOW Mesh Card
  sendChunk("<div class='card'>");
  sendChunk("<h2>📡 ESP-NOW Mesh</h2>");
  sendChunk("<div id='espnow-status'><p>Loading...</p></div>");
  sendChunk("<div style='margin-top:10px' id='espnow-buttons'>");
  sendChunk("<button class='btn btn-success btn-small' onclick='espnowStart(\"tx\")'>&#x25B6; Base TX</button>");
  sendChunk("&nbsp;<button class='btn btn-small' style='background:#2980b9;color:#fff' onclick='espnowStart(\"rx\")'>&#x25B6; Rover RX</button>");
  sendChunk("</div>");
  sendChunk("<div style='margin-top:6px'>");
  sendChunk("<button class='btn btn-small' style='background:#8e44ad;color:#fff' onclick='startApMode()' title='Avvia AP su canale ESP-NOW e disconnetti WiFi STA'>&#x1F4F6; Avvia AP</button>");
  sendChunk("</div>");
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
  sendChunk("function resetRtcm(){fetch('/api/rtcm/reset',{method:'POST'}).then(()=>updateRtcm()).catch(e=>alert('Errore: '+(e.message||e)));}");

  // ESP-NOW status polling
  sendChunk("function roleLabel(r){return r==='tx'?'Base TX':(r==='rx'?'Rover RX':(r==='relay'?'Relay':'---'));}");
  sendChunk("function updateEspNow(){");
  sendChunk("fetch('/api/espnow/status').then(r=>r.json()).then(d=>{");
  sendChunk("let led=d.enabled?'led-on':'led-off';");
  sendChunk("let h='<div class=\"status-row\"><span class=\"status-led '+led+'\"></span><strong>ESP-NOW:</strong>&nbsp;'+(d.enabled?roleLabel(d.role):'INATTIVO')+'</div>';");
  sendChunk("if(d.enabled){");
  sendChunk("h+='<table style=\"width:100%\">';");
  sendChunk("h+='<tr><td>Node ID</td><td>'+d.node_id+'</td><td>Canale</td><td>'+d.channel+'</td></tr>';");
  sendChunk("h+='<tr><td>Network ID</td><td>'+d.network_id+'</td><td>RSSI</td><td>'+d.last_rssi+' dBm</td></tr>';");
  sendChunk("h+='<tr><td>RX pkts</td><td>'+d.rx_pkts+'</td><td>TX pkts</td><td>'+d.tx_pkts+'</td></tr>';");
  sendChunk("h+='<tr><td>Drop dedup</td><td>'+d.drop_dedup+'</td><td>Drop old</td><td>'+d.drop_old+'</td></tr>';");
  sendChunk("h+='<tr><td>Drop CRC</td><td>'+d.drop_crc+'</td><td>RTCM bytes RX</td><td>'+d.rtcm_bytes_rx+'</td></tr>';");
  sendChunk("h+='</table>';");
  sendChunk("if(d.peers&&d.peers.length>0){");
  sendChunk("h+='<h3>Peer attivi</h3><table style=\"width:100%\"><tr><th>Node</th><th>Ruolo</th><th>Fix</th><th>Carr</th><th>hAcc mm</th><th>RSSI</th><th>Age ms</th></tr>';");
  sendChunk("var now=Date.now();");
  sendChunk("d.peers.forEach(p=>{");
  sendChunk("var roles=['Rover','Base','Relay'];");
  sendChunk("h+='<tr><td>'+p.node_id+'</td><td>'+(roles[p.role]||p.role)+'</td><td>'+p.fix+'</td><td>'+p.carr_soln+'</td><td>'+p.h_acc_mm+'</td><td>'+p.rssi+'</td><td>'+p.age_ms+'</td></tr>';");
  sendChunk("});");
  sendChunk("h+='</table>';");
  sendChunk("}");
  sendChunk("}");
  sendChunk("document.getElementById('espnow-status').innerHTML=h;");
  // Update action buttons based on current state
  sendChunk("var bd=document.getElementById('espnow-buttons');");
  sendChunk("if(bd){");
  sendChunk("  if(d.enabled){");
  sendChunk("    bd.innerHTML='<button class=\"btn btn-danger btn-small\" onclick=\"espnowStop()\">&#x25A0; Stop ESP-NOW&nbsp;('+roleLabel(d.role)+')</button>';");
  sendChunk("  } else {");
  sendChunk("    bd.innerHTML='<button class=\"btn btn-success btn-small\" onclick=\"espnowStart(\\\"tx\\\")\">&#x25B6; Base TX</button>&nbsp;<button class=\"btn btn-small\" style=\"background:#2980b9;color:#fff\" onclick=\"espnowStart(\\\"rx\\\")\">&#x25B6; Rover RX</button>';");
  sendChunk("  }");
  sendChunk("}");
  sendChunk("}).catch(()=>{document.getElementById('espnow-status').innerHTML='<p>Error</p>';});");
  sendChunk("}");
  sendChunk("updateEspNow();setInterval(updateEspNow,2000);");
  sendChunk("function espnowStart(role){fetch('/espnow/start'+role,{method:'POST'}).then(()=>updateEspNow());}");
  sendChunk("function espnowStop(){fetch('/espnow/stop',{method:'POST'}).then(()=>updateEspNow());}");  
  sendChunk("function syncNtp(){");
  sendChunk("fetch('/ntp/sync').then(r=>r.text()).then(t=>{location.reload();}).catch(e=>{alert('NTP Sync Error: '+(e.message||e));});");
  sendChunk("}");
  
  sendChunk("function switchToRover(){");
  sendChunk("if(confirm('Are you sure you want to switch to Rover mode? This will stop RTCM output and restart the ZED.')){");
    sendChunk("fetch('/api/switchToRover').then(r=>r.text()).then(t=>{alert(t);setTimeout(function(){location.reload();},3000);}).catch(e=>{alert('Error: '+(e.message||e));});");
  sendChunk("}}");

  // AP mode runtime switch
  sendChunk("function startApMode(){");
  sendChunk("if(!confirm('Passare in modalita\\' AP?\\n\\nRTKino avviera\\' AP: " + htmlEscape(String(g_apSsid)) + " (ch ESP-NOW).\\n\\nConnettiti a quell\\'AP per continuare.')) return;");
  sendChunk("fetch('/api/start-ap',{method:'POST'}).then(r=>r.text()).then(t=>{alert(t);}).catch(()=>{alert('Connettiti a " + htmlEscape(String(g_apSsid)) + " \\u2192 http://192.168.4.1');});");
  sendChunk("}");

  sendChunk("</script>");
  
  sendFooter();
}

// ========================================================================
// ROVER PAGE (/rover) - Hub: status summary + quick actions
// ========================================================================

static void handleRoverPage() {
  sendHeader("RTCM IN", "rover");

  // Load connection data
  std::vector<NtripIn> ntripList; int ntripLast=-1;
  loadNtripInList(ntripList, ntripLast);
  std::vector<TcpIn> tcpList; int tcpLast=-1;
  loadTcpInList(tcpList, tcpLast);

  // JavaScript for quick actions
  sendChunk("<script>");
  sendChunk("function toggleNtrip(e){fetch('/ntrip/toggle?enable='+e).then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function startTcpIn(){fetch('/lanin/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error starting TCP IN: '+(err.message||err));})}");
  sendChunk("function stopTcpIn(){fetch('/lanin/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error stopping TCP IN: '+(err.message||err));})}");
  sendChunk("</script>");

  sendChunk("<div class='card'>");
  sendChunk("<h2>&#x1F4E5; RTCM IN</h2>");

  // --- NTRIP IN ---
  sendChunk("<h3>NTRIP IN</h3>");
  sendChunk("<p><strong>Selected Profile:</strong> ");
  if (ntripLast>=0 && ntripLast<(int)ntripList.size()) {
    const auto& n = ntripList[ntripLast];
    sendChunk("<span class='badge'>" + htmlEscape(n.name) + "</span>");
    sendChunk("<br><small style='color:#555'>" + htmlEscape(n.host) + ":" + String(n.port) +
              " &mdash; Mount: " + htmlEscape(n.mount) +
              (n.user.length() ? " &mdash; User: " + htmlEscape(n.user) : "") + "</small>");
  } else {
    sendChunk("<em>None selected &mdash; go to Manage Connections IN to add/select a profile</em>");
  }
  sendChunk("</p>");

  String ntripLedClass = ntripEnabled ? "led-on" : "led-off";
  String ntripStatusText = ntripEnabled ? "Active" : "Inactive";
  sendChunk("<div class='status-row'><span class='status-led " + ntripLedClass + "'></span>");
  sendChunk("<strong>Status:</strong>&nbsp;" + ntripStatusText + "</div>");

  sendChunk("<div style='margin-top:10px;'>");
  sendChunk("<button onclick='toggleNtrip(1)' class='btn-success' style='min-height:44px;padding:10px 18px;font-size:1em;' aria-label='Enable NTRIP IN'>&#x25BA; Enable NTRIP IN</button> ");
  sendChunk("<button onclick='toggleNtrip(0)' class='btn-danger' style='min-height:44px;padding:10px 18px;font-size:1em;' aria-label='Disable NTRIP IN'>&#x25A0; Disable NTRIP IN</button>");
  sendChunk("</div>");

  sendChunk("<hr style='margin:14px 0;border:none;border-top:1px solid #ecf0f1;'>");

  // --- TCP IN ---
  sendChunk("<h3>TCP IN</h3>");
  sendChunk("<p><strong>Selected Profile:</strong> ");
  if (tcpLast>=0 && tcpLast<(int)tcpList.size()) {
    const auto& t = tcpList[tcpLast];
    sendChunk("<span class='badge'>" + htmlEscape(t.name) + "</span>");
    sendChunk("<br><small style='color:#555'>" + htmlEscape(t.host) + ":" + String(t.port) + "</small>");
  } else {
    sendChunk("<em>None selected &mdash; go to Manage Connections IN to add/select a profile</em>");
  }
  sendChunk("</p>");

  String tcpLedClass = tcpInEnabled ? "led-on" : "led-off";
  String tcpStatusText = tcpInEnabled ? "Active" : "Inactive";
  sendChunk("<div class='status-row'><span class='status-led " + tcpLedClass + "'></span>");
  sendChunk("<strong>Status:</strong>&nbsp;" + tcpStatusText + "</div>");

  sendChunk("<div style='margin-top:10px;'>");
  sendChunk("<button onclick='startTcpIn()' class='btn-success' style='min-height:44px;padding:10px 18px;font-size:1em;' aria-label='Start TCP IN'>&#x25BA; Start TCP IN</button> ");
  sendChunk("<button onclick='stopTcpIn()' class='btn-danger' style='min-height:44px;padding:10px 18px;font-size:1em;' aria-label='Stop TCP IN'>&#x25A0; Stop TCP IN</button>");
  sendChunk("</div>");

  sendChunk("<hr style='margin:14px 0;border:none;border-top:1px solid #ecf0f1;'>");

  // Manage button
  sendChunk("<a href='/rover/connections' style='display:block;text-align:center;min-height:48px;line-height:48px;font-size:1.05em;background:#3498db;color:white;border-radius:4px;text-decoration:none;margin-top:8px;'>&#x2699; Manage Connections IN</a>");

  sendChunk("</div>"); // end card

  sendFooter();
}

// ========================================================================
// ROVER CONNECTIONS PAGE (/rover/connections) - Full IN profile management
// ========================================================================

static void handleRoverConnectionsPage() {
  sendHeader("RTCM IN – Connections", "rover");

  sendChunk("<p style='margin-bottom:14px;'><a class='btn' href='/rover'>&larr; Back to RTCM IN</a></p>");

  // --- NTRIP IN Profiles ---
  std::vector<NtripIn> ntripList; int ntripLast=-1;
  loadNtripInList(ntripList, ntripLast);

  sendChunk("<div class='card'><h2>&#x1F4E5; NTRIP IN Profiles</h2>");

  sendChunk("<p><strong>Selected Profile:</strong> ");
  if (ntripLast>=0 && ntripLast<(int)ntripList.size()) {
    sendChunk("<span class='badge'>" + htmlEscape(ntripList[ntripLast].name) + "</span>");
  } else {
    sendChunk("<em>None</em>");
  }
  sendChunk("</p>");

  sendChunk("<button onclick='toggleNtrip(1)' class='btn-success' aria-label='Enable NTRIP IN'>&#x25BA; Enable NTRIP IN</button> ");
  sendChunk("<button onclick='toggleNtrip(0)' class='btn-danger' aria-label='Disable NTRIP IN'>&#x25A0; Disable NTRIP IN</button>");
  sendChunk("<script>function toggleNtrip(e){fetch('/ntrip/toggle?enable='+e).then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}</script>");

  if (!ntripList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Host</th><th>Port</th><th>Mount</th><th>User</th><th>Actions</th></tr>");
    for (size_t i=0;i<ntripList.size();++i){
      const auto& n=ntripList[i];
      String sel = (ntripLast==(int)i) ? " style='font-weight:bold;'" : "";
      String row = "<tr" + sel + "><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(n.name) + "</td>";
      row += "<td data-label='Host'>" + htmlEscape(n.host) + "</td>";
      row += "<td data-label='Port'>" + String(n.port) + "</td>";
      row += "<td data-label='Mount'>" + htmlEscape(n.mount) + "</td>";
      row += "<td data-label='User'>" + htmlEscape(n.user) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/ntrip/select?idx=" + String(i) + "' class='btn btn-small'>&#x2713; Select</a> ";
      row += "<a href='/ntrip/edit?idx=" + String(i) + "' class='btn btn-small'>&#x270F; Edit</a> ";
      row += "<a href='/ntrip/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(n.name) + "?\")'>&#x274C; Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No NTRIP IN profiles saved.</em></p>");
  }

  sendChunk("<h3>Add NTRIP IN Profile</h3>");
  sendChunk("<form method='POST' action='/ntrip/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Host:</label><input id='aHost' name='host' required><br>");
  sendChunk("<label>Port:</label><input id='aPort' name='port' type='number' value='2101' required><br>");
  sendChunk("<label>User:</label><input id='aUser' name='user'><br>");
  sendChunk("<label>Password:</label><input id='aPwd' name='pwd' type='password'><br>");
  sendChunk("<label>Mountpoint:</label><div style='display:flex;gap:6px;align-items:center;max-width:320px;'>");
  sendChunk("<input id='aMount' name='mount' required style='flex:1;'>");
  sendChunk("<button type='button' id='aBrowseBtn' onclick='browseMount(\"aMount\",\"aHost\",\"aPort\",\"aUser\",\"aPwd\",\"aStResult\",\"aBrowseBtn\")'>Browse</button>");
  sendChunk("</div><div id='aStResult' style='margin-top:8px;overflow-x:auto;'></div><br>");
  sendChunk("<button type='submit'>Add NTRIP IN Profile</button>");
  sendChunk("</form></div>");
  sendChunk("<script>");
  sendChunk("function browseMount(mId,hId,pId,uId,wId,rId,bId){");
  sendChunk("var h=document.getElementById(hId).value.trim(),p=document.getElementById(pId).value.trim();");
  sendChunk("if(!h||!p){alert('Inserisci host e porta prima di sfogliare');return;}");
  sendChunk("var btn=document.getElementById(bId);btn.disabled=true;btn.textContent='Caricamento...';");
  sendChunk("var u=document.getElementById(uId).value.trim(),w=document.getElementById(wId).value.trim();");
  sendChunk("fetch('/ntrip/sourcetable?host='+encodeURIComponent(h)+'&port='+p+'&user='+encodeURIComponent(u)+'&pass='+encodeURIComponent(w))");
  sendChunk(".then(function(r){return r.json();}).then(function(d){");
  sendChunk("btn.disabled=false;btn.textContent='Browse';");
  sendChunk("var div=document.getElementById(rId);");
  sendChunk("if(!d.ok){div.innerHTML='<p style=\"color:red\">'+d.err+'</p>';return;}");
  sendChunk("if(!d.mounts||d.mounts.length===0){div.innerHTML='<p><em>Nessun mountpoint trovato</em></p>';return;}");
  sendChunk("var html='<table style=\"font-size:13px\"><tr><th>ID</th><th>Formato</th><th>Nav</th><th>Paese</th><th>Lat</th><th>Lon</th></tr>';");
  sendChunk("d.mounts.forEach(function(m){");
  sendChunk("html+='<tr style=\"cursor:pointer\" onclick=\"document.getElementById(\\''+mId+'\\').value=\\''+m.id+'\\'\">'+");
  sendChunk("'<td><strong>'+m.id+'</strong></td><td>'+(m.fmt||'')+'</td><td>'+(m.nav||'')+'</td><td>'+(m.ctr||'')+'</td><td>'+(m.lat||'')+'</td><td>'+(m.lon||'')+'</td></tr>';");
  sendChunk("});html+='</table><p style=\"font-size:12px;color:#666\">Clicca una riga per selezionare il mountpoint</p>';");
  sendChunk("div.innerHTML=html;");
  sendChunk("}).catch(function(e){btn.disabled=false;btn.textContent='Browse';alert('Errore: '+e.message);});}");
  sendChunk("</script>");

  // --- TCP IN Profiles ---
  std::vector<TcpIn> tcpList; int tcpLast=-1;
  loadTcpInList(tcpList, tcpLast);

  sendChunk("<div class='card'><h2>&#x1F4E5; TCP IN Profiles</h2>");

  sendChunk("<p><strong>Selected Profile:</strong> ");
  if (tcpLast>=0 && tcpLast<(int)tcpList.size()) {
    sendChunk("<span class='badge'>" + htmlEscape(tcpList[tcpLast].name) + "</span>");
  } else {
    sendChunk("<em>None</em>");
  }
  sendChunk("</p>");

  String tcpStatusHtml = "<p><strong>Status:</strong> ";
  tcpStatusHtml += tcpInEnabled ? "<span class='badge' style='background:#2ecc71;color:white'>Active</span>" : "<span class='badge'>Inactive</span>";
  tcpStatusHtml += "</p>";
  sendChunk(tcpStatusHtml);

  sendChunk("<button onclick='startTcpIn()' class='btn-success' aria-label='Start TCP IN'>&#x25BA; Start TCP IN</button> ");
  sendChunk("<button onclick='stopTcpIn()' class='btn-danger' aria-label='Stop TCP IN'>&#x25A0; Stop TCP IN</button>");
  sendChunk("<script>");
  sendChunk("function startTcpIn(){fetch('/lanin/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function stopTcpIn(){fetch('/lanin/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("</script>");

  if (!tcpList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Host</th><th>Port</th><th>Actions</th></tr>");
    for (size_t i=0;i<tcpList.size();++i){
      const auto& t=tcpList[i];
      String sel = (tcpLast==(int)i) ? " style='font-weight:bold;'" : "";
      String row = "<tr" + sel + "><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(t.name) + "</td>";
      row += "<td data-label='Host'>" + htmlEscape(t.host) + "</td>";
      row += "<td data-label='Port'>" + String(t.port) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/lanin/select?idx=" + String(i) + "' class='btn btn-small'>&#x2713; Select</a> ";
      row += "<a href='/lanin/edit?idx=" + String(i) + "' class='btn btn-small'>&#x270F; Edit</a> ";
      row += "<a href='/lanin/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(t.name) + "?\")'>&#x274C; Delete</a> ";
      row += "<a href='/lanin/start?id=" + String(i) + "' class='btn btn-small btn-success'>&#x25BA; Start</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No TCP IN profiles saved.</em></p>");
  }

  sendChunk("<h3>Add TCP IN Profile</h3>");
  sendChunk("<form method='POST' action='/lanin/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Host/IP:</label><input name='host' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='2103' required><br>");
  sendChunk("<button type='submit'>Add TCP IN Profile</button>");
  sendChunk("</form></div>");

  sendFooter();
}

// ========================================================================
// BASE PAGE (/base-cfg) - Hub: status summary + quick actions
// ========================================================================

static void handleBasePage() {
  sendHeader("Base", "base");

  sendChunk("<script>");
  sendChunk("function stopOut(){fetch('/baseout/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function stopTcpClient(){fetch('/tcpclient/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function switchToRover(){if(confirm('Switch to Rover mode? This will stop RTCM OUT and restart the ZED.')){fetch('/api/switchToRover').then(r=>r.text()).then(t=>{alert(t);setTimeout(()=>location.reload(),3000);}).catch(e=>{alert('Error: '+(e.message||e));})}}");
  sendChunk("function avviaBase(){");
  sendChunk("  var btn=document.getElementById('btn-avvia-base');");
  sendChunk("  btn.disabled=true; btn.textContent='Configuring ZED...';");
  sendChunk("  fetch('/api/startBaseMode').then(function(r){");
  sendChunk("    if(!r.ok) return r.text().then(function(t){throw new Error(t);});");
  sendChunk("    btn.textContent='Starting outputs in 4s...';");
  sendChunk("    return new Promise(function(res){setTimeout(res,4000);});");
  sendChunk("  }).then(function(){ return fetch('/api/startAllOutputs'); })");
  sendChunk("  .then(function(r){ return r.text(); })");
  sendChunk("  .then(function(t){ alert('Base mode started!\\n'+t); location.reload(); })");
  sendChunk("  .catch(function(e){ alert('Error: '+(e.message||e)); btn.disabled=false; btn.textContent='\\u25BA Start Base Mode'; });");
  sendChunk("}");
  sendChunk("</script>");

  ZedTmodeState tmode;
  bool tmodeOk = UbxVal::getTmodeState(tmode) && tmode.valid;
  bool inBaseMode = tmodeOk && tmode.mode == 2;

  if (inBaseMode) {
    // === BASE ATTIVA ===
    sendChunk("<div class='card' style='border-left:4px solid #2ecc71;'>");
    sendChunk("<h2><span class='status-led led-on' style='margin-right:8px;'></span>&#x1F4FB; BASE – Fixed</h2>");
    String coords = "<p style='margin:4px 0 12px 0;color:#555;'>";
    coords += "Lat: " + String(tmode.lat, 8) + "&deg;<br>";
    coords += "Lon: " + String(tmode.lon, 8) + "&deg;<br>";
    coords += "H: " + String(tmode.height, 3) + " m</p>";
    sendChunk(coords);
    sendChunk("<button onclick='switchToRover()' style='width:100%;min-height:52px;font-size:1.1em;background:#e67e22;color:white;border:none;border-radius:6px;cursor:pointer;font-weight:bold;'>&#x21BA; Rover</button>");
    sendChunk("</div>");
  } else {
    // === ROVER / NON IN BASE ===
    int baseLast=-1; std::vector<BaseRec> bases; loadBases(bases, baseLast);
    std::vector<NtripOut> outList; int outLast=-1; loadNtripOutList(outList, outLast);
    std::vector<TcpOutClient> tcpList; int tcpLast=-1; loadTcpOutClientList(tcpList, tcpLast);
    BaseAutoStart as = loadBaseAutoStart();

    sendChunk("<div class='card'>");
    sendChunk("<h2>&#x1F4FB; Start Base Mode</h2>");

    // Selected station
    sendChunk("<p><strong>Station:</strong> ");
    if (baseLast>=0 && baseLast<(int)bases.size()) {
      const auto& b=bases[baseLast];
      sendChunk("<span class='badge'>" + htmlEscape(b.name) + "</span>");
      sendChunk("<br><small style='color:#555'>Lat: "+String(b.lat,8)+" &nbsp; Lon: "+String(b.lon,8)+" &nbsp; H: "+String(b.altGround,3)+" m</small>");
    } else {
      sendChunk("<em style='color:#e74c3c;'>None selected &mdash; go to Base Stations and select one</em>");
    }
    sendChunk("</p>");

    // Auto-start outputs
    sendChunk("<p><strong>Outputs on start:</strong>");
    sendChunk("<ul style='margin:4px 0 8px 16px;font-size:0.92em;'>");
    if (as.ntrip) {
      String n = (outLast>=0&&outLast<(int)outList.size()) ? htmlEscape(outList[outLast].name) : "<em>no profile selected</em>";
      sendChunk("<li>NTRIP OUT: " + n + "</li>");
    }
    if (as.tcp) {
      String t = (tcpLast>=0&&tcpLast<(int)tcpList.size()) ? htmlEscape(tcpList[tcpLast].name) : "<em>no profile selected</em>";
      sendChunk("<li>TCP Client OUT: " + t + "</li>");
    }
    if (!as.ntrip && !as.tcp) sendChunk("<li style='color:#95a5a6;'>No outputs configured &mdash; set them in RTCM Outputs</li>");
    sendChunk("</ul></p>");

    bool canStart = (baseLast>=0 && baseLast<(int)bases.size());
    sendChunk("<button id='btn-avvia-base' onclick='avviaBase()' style='width:100%;min-height:52px;font-size:1.1em;background:");
    sendChunk(canStart ? "#27ae60" : "#95a5a6");
    sendChunk(";color:white;border:none;border-radius:6px;cursor:pointer;font-weight:bold;'");
    if (!canStart) sendChunk(" disabled title='Select a station first'");
    sendChunk(">&#x25BA; Start Base Mode</button>");
    sendChunk("</div>");
  }

  // Manage links (always visible)
  sendChunk("<div class='card'>");
  sendChunk("<h2>&#x2699; Manage</h2>");
  sendChunk("<div style='display:flex;gap:10px;flex-wrap:wrap;'>");
  sendChunk("<a href='/base/stations' style='flex:1;display:block;text-align:center;min-height:48px;line-height:48px;font-size:1em;background:#3498db;color:white;border-radius:4px;text-decoration:none;'>&#x1F5FA; Base Stations</a>");
  sendChunk("<a href='/base/outputs' style='flex:1;display:block;text-align:center;min-height:48px;line-height:48px;font-size:1em;background:#3498db;color:white;border-radius:4px;text-decoration:none;'>&#x1F4E4; RTCM Outputs</a>");
  sendChunk("</div>");
  sendChunk("</div>");

  sendFooter();
}

// ========================================================================
// BASE STATIONS PAGE (/base/stations) - Full base station management
// ========================================================================

static void handleBaseStationsPage() {
  sendHeader("Base – Stations", "base");

  sendChunk("<p style='margin-bottom:14px;'><a class='btn' href='/base-cfg'>&larr; Back to Base</a></p>");

  // --- TMODE Manual Entry ---
  sendChunk("<div class='card'><h2>&#x1F4CD; Start Base from Coordinates</h2>");
  sendChunk("<p>Apply fixed LLH coordinates directly to ZED-F9P (stored in RAM):</p>");
  sendChunk("<ul>");
  sendChunk("<li>TMODE: <strong>FIXED LLH (HP)</strong></li>");
  sendChunk("<li>UART2 TX: <strong>RTCM3 only</strong></li>");
  sendChunk("<li>MSGOUT: 1005/1230 @10s; MSM7 @1s or MSM4 @1s</li>");
  sendChunk("</ul>");
  sendChunk("<form method='POST' action='/base/llh'>");
  sendChunk("<label>Latitude [deg]:</label><input name='lat' required><br>");
  sendChunk("<label>Longitude [deg]:</label><input name='lon' required><br>");
  sendChunk("<label>Altitude ellips. [m]:</label><input name='alt' required><br>");
  sendChunk("<label>Station ID [1..4095]:</label><input name='stid' type='number' value='1' style='width:150px'><br>");
  sendChunk("<label>RTCM Messages:</label><select name='rtcm_type' style='width:350px'>");
  sendChunk("<option value='0'>MSM7 - 4 constellations (GPS, GLO, GAL, BDS) @1Hz</option>");
  sendChunk("<option value='1'>MSM4 - 3 constellations (GPS, GLO, GAL) @1Hz</option>");
  sendChunk("</select><br>");
  sendChunk("<button type='submit' class='btn-success' style='min-height:44px;padding:10px 18px;font-size:1em;'>&#x25BA; Start Base</button>");
  sendChunk("</form></div>");
  
  // Saved Bases
  int baseLast=-1;
  std::vector<BaseRec> bases;
  loadBases(bases, baseLast);

  // Load antennas for display
  std::vector<AntennaRec> antennas;
  loadAntennas(antennas);

  sendChunk("<div class='card'><h2>&#x1F5FA; Saved Base Stations</h2>");

  // Default station indicator
  sendChunk("<p><strong>Default station:</strong> ");
  if (baseLast>=0 && baseLast<(int)bases.size()) {
    sendChunk("<span class='badge'>" + htmlEscape(bases[baseLast].name) + "</span> <small style='color:#555;'>(used by Start Base Mode)</small>");
  } else {
    sendChunk("<em style='color:#e74c3c;'>None &mdash; press Select on a station below</em>");
  }
  sendChunk("</p>");

  if (!bases.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>STID</th><th>Name</th><th>Lat</th><th>Lon</th><th>H ground [m]</th><th>H ARP [m]</th><th>Antenna</th><th>RTCM</th><th>Actions</th></tr>");
    for (size_t i=0;i<bases.size();++i){
      auto& b=bases[i];
      bool isSel = (baseLast == (int)i);
      String row = "<tr" + String(isSel ? " style='font-weight:bold;'" : "") + "><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='STID'>" + String(b.stid) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(b.name) + (isSel ? " <span class='badge' style='background:#27ae60;'>&#x2713;</span>" : "") + "</td>";
      row += "<td data-label='Lat'>" + String(b.lat,8) + "</td>";
      row += "<td data-label='Lon'>" + String(b.lon,8) + "</td>";
      row += "<td data-label='H ground [m]'>" + String(b.altGround,3) + "</td>";
      row += "<td data-label='H ARP [m]'>" + String(b.hARP,3) + "</td>";
      String antennaName = "None";
      if (b.antennaIdx >= 0 && b.antennaIdx < (int)antennas.size()) {
        antennaName = htmlEscape(antennas[b.antennaIdx].name);
      }
      row += "<td data-label='Antenna'>" + antennaName + "</td>";
      String rtcmType = (b.rtcmType == 0) ? "MSM7 4c" : "MSM4 3c";
      row += "<td data-label='RTCM'>" + rtcmType + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/bases/select?idx=" + String(i) + "' class='btn btn-small" + (isSel ? " btn-success" : "") + "'>&#x2713; Select</a> ";
      row += "<a href='/bases/edit?idx=" + String(i) + "' class='btn btn-small'>&#x270F; Edit</a> ";
      row += "<a href='/bases/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(b.name) + "?\")'>&#x274C; Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No saved base stations.</em></p>");
  }

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
  sendChunk("<button type='submit'>Add Base Station</button>");
  sendChunk("</form></div>");

  // Survey Base Position
  std::vector<AntennaRec> surveyAntennas;
  loadAntennas(surveyAntennas);

  sendChunk("<div class='card'><h2>&#x1F4CD; Survey Base Position</h2>");
  sendChunk("<p>Record averaged GNSS positions over time and save as a new base station.</p>");

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
  sendChunk("<button type='submit' class='btn-success' id='startBtn'>&#x1F7E2; Start Survey</button> ");
  sendChunk("<button type='button' class='btn-danger' id='stopBtn' onclick='stopSurvey()' style='display:none'>&#x1F534; Stop Survey</button>");
  sendChunk("</form>");

  sendChunk("<div id='surveyProgress' style='display:none; margin-top:20px'>");
  sendChunk("<h3>Progress</h3>");
  sendChunk("<div style='background:#ecf0f1; border-radius:4px; height:30px; overflow:hidden; margin:10px 0'>");
  sendChunk("<div id='progressBar' style='background:#3498db; height:100%; width:0%; transition:width 0.5s'></div>");
  sendChunk("</div>");
  sendChunk("<p><strong>Time:</strong> <span id='timeProgress'>0/0 s</span> (<span id='progressPct'>0</span>%)</p>");
  sendChunk("<p><strong>Samples:</strong> <span id='sampleCount'>0</span></p>");
  sendChunk("<h3>Current Average:</h3>");
  sendChunk("<table style='width:100%'>");
  sendChunk("<tr><td><b>Lat:</b></td><td><span id='currLat'>-</span>&deg;</td><td><b>&sigma;:</b></td><td><span id='stdLat'>-</span>&deg;</td></tr>");
  sendChunk("<tr><td><b>Lon:</b></td><td><span id='currLon'>-</span>&deg;</td><td><b>&sigma;:</b></td><td><span id='stdLon'>-</span>&deg;</td></tr>");
  sendChunk("<tr><td><b>H (ARP):</b></td><td><span id='currAltARP'>-</span> m</td><td><b>&sigma;:</b></td><td><span id='stdAlt'>-</span> m</td></tr>");
  sendChunk("<tr><td><b>H (Ground):</b></td><td colspan='3'><span id='currAltGround'>-</span> m</td></tr>");
  sendChunk("</table>");
  sendChunk("</div>");

  sendChunk("<div id='surveyResults' style='display:none; margin-top:20px'>");
  sendChunk("<h3>&#x2705; Survey Complete!</h3>");
  sendChunk("<h4>Final Position (ground point):</h4>");
  sendChunk("<table style='width:100%'>");
  sendChunk("<tr><td><b>Latitude:</b></td><td><span id='finalLat'>-</span>&deg;</td></tr>");
  sendChunk("<tr><td><b>Longitude:</b></td><td><span id='finalLon'>-</span>&deg;</td></tr>");
  sendChunk("<tr><td><b>H (ellips):</b></td><td><span id='finalAlt'>-</span> m</td></tr>");
  sendChunk("<tr><td><b>Std Dev:</b></td><td><span id='finalStd'>-</span> m (horizontal)</td></tr>");
  sendChunk("</table>");
  sendChunk("<h4>Save as Base Station:</h4>");
  sendChunk("<form onsubmit='saveSurvey(event)'>");
  sendChunk("<label>Name:</label><input id='saveName' type='text' placeholder='e.g. Benchmark Via Roma' required><br>");
  sendChunk("<label>Station ID:</label><input id='saveStid' type='number' min='1' max='4095' value='1' required><br>");
  sendChunk("<button type='submit' class='btn-success'>&#x1F4BE; Save Base Station</button>");
  sendChunk("</form>");
  sendChunk("</div>");

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

  // Base Start Confirmation Modal
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
  sendChunk("    <h2 style='margin-bottom:16px;'>&#x26A0; Confirm Base Station Start</h2>");
  sendChunk("    <p><strong>Base:</strong> ${base.name}</p>");
  sendChunk("    <h4>Ground coordinates:</h4>");
  sendChunk("    <p>Lat: ${base.lat.toFixed(8)}&deg;<br>");
  sendChunk("       Lon: ${base.lon.toFixed(8)}&deg;<br>");
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
  sendChunk("      <strong>&#x1F4D0; H to send to ZED-F9P:</strong><br>");
  sendChunk("      <span id='calcFormula'></span>");
  sendChunk("    </div>");
  sendChunk("    <div style='margin-top:20px;display:flex;gap:12px;justify-content:flex-end;'>");
  sendChunk("      <button onclick='closeModal()' style='padding:10px 20px;background:#e74c3c;color:white;border:none;border-radius:4px;cursor:pointer;'>&#x274C; Cancel</button>");
  sendChunk("      <button onclick='doStartBase(${idx})' style='padding:10px 20px;background:#2ecc71;color:white;border:none;border-radius:4px;cursor:pointer;'>&#x2705; Start Base</button>");
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
  sendChunk("</div>"); // end Survey card

  sendFooter();
}

// ========================================================================
// BASE OUTPUTS PAGE (/base/outputs) - Full RTCM OUT management
// ========================================================================

static void handleBaseOutputsPage() {
  sendHeader("Base – RTCM Outputs", "base");

  sendChunk("<p style='margin-bottom:14px;'><a class='btn' href='/base-cfg'>&larr; Back to Base</a></p>");

  // JavaScript for controls
  sendChunk("<script>");
  sendChunk("function startOut(){fetch('/baseout/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function stopOut(){fetch('/baseout/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function startTcpClient(){fetch('/tcpclient/start').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("function stopTcpClient(){fetch('/tcpclient/stop').then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}");
  sendChunk("</script>");

  // --- Auto-start con Modalità Base ---
  {
    BaseAutoStart as = loadBaseAutoStart();
    std::vector<NtripOut> outList; int outLast=-1; loadNtripOutList(outList, outLast);
    std::vector<TcpOutClient> tcpList; int tcpLast=-1; loadTcpOutClientList(tcpList, tcpLast);

    sendChunk("<div class='card' style='border-left:4px solid #27ae60;'>");
    sendChunk("<h2>&#x26A1; Auto-start with Base Mode</h2>");
    sendChunk("<p style='font-size:0.9em;color:#555;'>Select which outputs start automatically when <strong>Start Base Mode</strong> is pressed.</p>");
    sendChunk("<form method='POST' action='/api/base-autostart'>");

    // NTRIP OUT
    String ntripLabel = (outLast>=0&&outLast<(int)outList.size()) ? htmlEscape(outList[outLast].name) : "<em>no profile selected</em>";
    sendChunk("<label style='display:flex;align-items:center;gap:8px;margin:8px 0;'>");
    sendChunk("<input type='checkbox' name='ntrip' value='1'" + String(as.ntrip?" checked":"") + ">");
    sendChunk("<span><strong>NTRIP OUT</strong>: " + ntripLabel + "</span></label>");

    // TCP Client OUT
    String tcpLabel = (tcpLast>=0&&tcpLast<(int)tcpList.size()) ? htmlEscape(tcpList[tcpLast].name) : "<em>no profile selected</em>";
    sendChunk("<label style='display:flex;align-items:center;gap:8px;margin:8px 0;'>");
    sendChunk("<input type='checkbox' name='tcp' value='1'" + String(as.tcp?" checked":"") + ">");
    sendChunk("<span><strong>TCP Client OUT</strong>: " + tcpLabel + "</span></label>");

    sendChunk("<button type='submit' class='btn-success' style='margin-top:10px;'>&#x1F4BE; Save Auto-start</button>");
    sendChunk("</form>");
    sendChunk("</div>");
  }

  // --- NTRIP OUT Profiles ---
  std::vector<NtripOut> outList; int outLast=-1;
  loadNtripOutList(outList, outLast);

  sendChunk("<div class='card'><h2>&#x1F4E4; NTRIP OUT Profiles</h2>");
  sendChunk("<p><strong>Selected Profile:</strong> ");
  if (outLast>=0 && outLast<(int)outList.size()) {
    const auto& o = outList[outLast];
    sendChunk("<span class='badge'>" + htmlEscape(o.name) + "</span>");
    sendChunk("<br><small style='color:#555'>" + htmlEscape(o.host) + ":" + String(o.port) +
              " &mdash; Mount: " + htmlEscape(o.mount) + " &mdash; TCP Port: " + String(o.tcpPort) + "</small>");
  } else {
    sendChunk("<em>None</em>");
  }
  sendChunk("</p>");
  sendChunk("<button onclick='startOut()' class='btn-success' aria-label='Start NTRIP OUT'>&#x25BA; Start NTRIP OUT</button> ");
  sendChunk("<button onclick='stopOut()' class='btn-danger' aria-label='Stop NTRIP OUT'>&#x25A0; Stop NTRIP OUT</button>");

  if (!outList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Host</th><th>Port</th><th>Mount</th><th>TCP Port</th><th>Actions</th></tr>");
    for (size_t i=0;i<outList.size();++i){
      const auto& n=outList[i];
      String sel = (outLast==(int)i) ? " style='font-weight:bold;'" : "";
      String row = "<tr" + sel + "><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(n.name) + "</td>";
      row += "<td data-label='Host'>" + htmlEscape(n.host) + "</td>";
      row += "<td data-label='Port'>" + String(n.port) + "</td>";
      row += "<td data-label='Mount'>" + htmlEscape(n.mount) + "</td>";
      row += "<td data-label='TCP Port'>" + String(n.tcpPort) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/baseout/select?idx=" + String(i) + "' class='btn btn-small" + String(outLast==(int)i?" btn-success":"") + "'>&#x2713; Select</a> ";
      row += "<a href='/baseout/edit?idx=" + String(i) + "' class='btn btn-small'>&#x270F; Edit</a> ";
      row += "<a href='/baseout/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(n.name) + "?\")'>&#x274C; Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No NTRIP OUT profiles saved.</em></p>");
  }

  sendChunk("<h3>Add NTRIP OUT Profile</h3>");
  sendChunk("<form method='POST' action='/baseout/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Host:</label><input name='host' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='2101' required><br>");
  sendChunk("<label>Mountpoint:</label><input name='mount' required><br>");
  sendChunk("<label>Password:</label><input name='pass' type='password'><br>");
  sendChunk("<label>TCP Server Port:</label><input name='tcp' type='number' value='2102' required><br>");
  sendChunk("<button type='submit'>Add NTRIP OUT Profile</button>");
  sendChunk("</form></div>");

  // --- TCP Client OUT Profiles ---
  std::vector<TcpOutClient> tcpClientList; int tcpClientLast=-1;
  loadTcpOutClientList(tcpClientList, tcpClientLast);

  sendChunk("<div class='card'><h2>&#x1F4E4; TCP Client OUT Profiles</h2>");
  sendChunk("<p>Connect to external TCP servers and stream RTCM data.</p>");
  sendChunk("<p><strong>Selected Profile:</strong> ");
  if (tcpClientLast>=0 && tcpClientLast<(int)tcpClientList.size()) {
    const auto& tc = tcpClientList[tcpClientLast];
    sendChunk("<span class='badge'>" + htmlEscape(tc.name) + "</span>");
    sendChunk("<br><small style='color:#555'>" + htmlEscape(tc.host) + ":" + String(tc.port) + "</small>");
  } else {
    sendChunk("<em>None</em>");
  }
  sendChunk("</p>");
  sendChunk("<button onclick='startTcpClient()' class='btn-success'>&#x25BA; Start TCP Client OUT</button> ");
  sendChunk("<button onclick='stopTcpClient()' class='btn-danger'>&#x25A0; Stop TCP Client OUT</button>");

  if (!tcpClientList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Host</th><th>Port</th><th>Actions</th></tr>");
    for (size_t i=0;i<tcpClientList.size();++i){
      const auto& t=tcpClientList[i];
      String sel = (tcpClientLast==(int)i) ? " style='font-weight:bold;'" : "";
      String row = "<tr" + sel + "><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(t.name) + "</td>";
      row += "<td data-label='Host'>" + htmlEscape(t.host) + "</td>";
      row += "<td data-label='Port'>" + String(t.port) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/tcpclient/select?idx=" + String(i) + "' class='btn btn-small" + String(tcpClientLast==(int)i?" btn-success":"") + "'>&#x2713; Select</a> ";
      row += "<a href='/tcpclient/edit?idx=" + String(i) + "' class='btn btn-small'>&#x270F; Edit</a> ";
      row += "<a href='/tcpclient/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(t.name) + "?\")'>&#x274C; Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table>");
    sendChunk("</div>");
  } else {
    sendChunk("<p><em>No TCP Client OUT profiles saved.</em></p>");
  }

  sendChunk("<h3>Add TCP Client OUT Profile</h3>");
  sendChunk("<form method='POST' action='/tcpclient/add'>");
  sendChunk("<label>Name:</label><input name='name' required><br>");
  sendChunk("<label>Host:</label><input name='host' required placeholder='192.168.1.100 or rtk.server.com'><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='5000' required><br>");
  sendChunk("<button type='submit'>Add TCP Client OUT Profile</button>");
  sendChunk("</form></div>");

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
// ESP-NOW Mesh Card (Settings page)
// ========================================================================

static void renderEspNowCard() {
  sendChunk("<div class='card'><h2>&#x1F4E1; ESP-NOW Mesh</h2>");

  // Live status section — populated by updateEspNowSettings()
  sendChunk("<div id='espnow-cfg-status' style='margin-bottom:12px;'><em>Caricamento stato...</em></div>");

  // Configuration form
  sendChunk("<h3>Configurazione</h3>");
  sendChunk("<div style='margin-bottom:12px;'>");
  sendChunk("<label>Modalit&#xE0;:</label><br>");
  sendChunk("<select id='espnow-cfg-role' style='width:220px;margin-bottom:8px;'>");
  sendChunk("<option value='rx'>Rover RX (riceve correzioni)</option>");
  sendChunk("<option value='tx'>Base TX (trasmette correzioni)</option>");
  sendChunk("</select><br>");
  sendChunk("<label>Canale WiFi (1&ndash;13):</label><br>");
  char chBuf[8]; snprintf(chBuf, sizeof(chBuf), "%d", ESPNOW_WIFI_CHANNEL);
  sendChunk(String("<input id='espnow-cfg-channel' type='number' min='1' max='13' value='") + chBuf + "' style='width:80px;margin-bottom:8px;'><br>");
  sendChunk("</div>");
  sendChunk("<button class='btn btn-success' onclick='espnowSaveSettings()'>&#x25B6; Salva e Attiva</button>&nbsp;");
  sendChunk("<button class='btn btn-danger' onclick='espnowStopSettings()'>&#x25A0; Stop</button>");

  // Remote commands — hidden until status polling confirms enabled==true
  sendChunk("<div id='espnow-cfg-cmds' style='display:none;margin-top:18px;'>");
  sendChunk("<h3>Invia Comando Remoto</h3>");
  sendChunk("<div style='display:flex;gap:8px;flex-wrap:wrap;align-items:center;'>");
  sendChunk("<select id='espnow-cmd-sel' style='width:220px'>");
  sendChunk("<option value='0x08'>Status Request</option>");
  sendChunk("<option value='0x02'>ZED Reset Hot</option>");
  sendChunk("<option value='0x03'>ZED Reset Cold</option>");
  sendChunk("<option value='0x04'>Log Start</option>");
  sendChunk("<option value='0x05'>Log Stop</option>");
  sendChunk("<option value='0x06'>Base Stop</option>");
  sendChunk("<option value='0x07'>ESP-NOW Stop</option>");
  sendChunk("<option value='0x01'>Reboot</option>");
  // RELAY_START/RELAY_STOP: RTKino emits these toward Crocevia relay nodes; relay mode is not local
  sendChunk("<option value='16'>RELAY_START (0x10) → Crocevia</option>");
  sendChunk("<option value='17'>RELAY_STOP (0x11) → Crocevia</option>");
  sendChunk("</select>");
  sendChunk("<input id='espnow-dst' type='text' placeholder='Node ID hex (vuoto=broadcast)' style='width:220px' />");
  sendChunk("<button class='btn btn-small' onclick='espnowSendCmd()'>&#x1F4E4; Invia</button>");
  sendChunk("</div>");
  sendChunk("<div id='espnow-cmd-result' style='margin-top:8px;font-size:0.9em;'></div>");
  sendChunk("</div>");

  sendChunk("<script>");
  sendChunk("function espnowRoleLabel(r){return r==='tx'?'Base TX':(r==='rx'?'Rover RX':(r==='relay'?'Relay':'---'));}");
  sendChunk("function updateEspNowSettings(){");
  sendChunk("fetch('/api/espnow/status').then(function(r){return r.json();}).then(function(d){");
  sendChunk("var led=d.enabled?'led-on':'led-off';");
  sendChunk("var h='<div class=\"status-row\"><span class=\"status-led '+led+'\"></span><strong>'+(d.enabled?espnowRoleLabel(d.role)+' &mdash; Node ID: '+d.node_id+' &mdash; Ch: '+d.channel:'INATTIVO')+'</strong></div>';");
  sendChunk("if(d.enabled){");
  sendChunk("h+='<table style=\"width:100%;font-size:0.9em;margin-top:6px\">';");
  sendChunk("h+='<tr><td>RSSI</td><td>'+d.last_rssi+' dBm</td><td>Network ID</td><td>'+d.network_id+'</td></tr>';");
  sendChunk("h+='<tr><td>RX pkts</td><td>'+d.rx_pkts+'</td><td>TX pkts</td><td>'+d.tx_pkts+'</td></tr>';");
  sendChunk("h+='<tr><td>Dedup</td><td>'+d.drop_dedup+'</td><td>CRC err</td><td>'+d.drop_crc+'</td></tr>';");
  sendChunk("h+='<tr><td>Drop no mem</td><td>'+d.drop_no_mem+'</td><td>Drop old</td><td>'+d.drop_old+'</td></tr>';");
  sendChunk("if(d.psk_enabled)h+='<tr><td>PSK</td><td>attivo</td><td>Auth err</td><td>'+d.drop_auth+'</td></tr>';");
  // relay_node_id: the remote Crocevia relay node currently serving this rover (0 = none selected)
  sendChunk("if(d.relay_node_id&&d.relay_node_id!=='0x0000')h+='<tr><td colspan=2>Relay: 0x'+d.relay_node_id.replace('0x','').toUpperCase()+'</td><td colspan=2></td></tr>';");
  sendChunk("h+='</table>';");
  sendChunk("if(d.peers&&d.peers.length>0){");
  sendChunk("h+='<div style=\"overflow-x:auto;margin-top:8px\"><table style=\"width:100%;font-size:0.9em\"><tr><th>Node ID</th><th>Ruolo</th><th>RSSI</th><th>Age (s)</th></tr>';");
  sendChunk("var roles=['Rover','Base','Relay'];");
  sendChunk("d.peers.forEach(function(p){h+='<tr><td>'+p.node_id+'</td><td>'+(roles[p.role]||p.role)+'</td><td>'+p.rssi+' dBm</td><td>'+(p.age_ms/1000).toFixed(1)+'</td></tr>';});");
  sendChunk("h+='</table></div>';");
  sendChunk("}");
  sendChunk("}");
  sendChunk("document.getElementById('espnow-cfg-status').innerHTML=h;");
  sendChunk("document.getElementById('espnow-cfg-cmds').style.display=d.enabled?'block':'none';");
  sendChunk("var sel=document.getElementById('espnow-cfg-role');");
  sendChunk("if(d.role==='tx'&&sel.options.length>1)sel.selectedIndex=1;else sel.selectedIndex=0;");
  sendChunk("if(d.channel){document.getElementById('espnow-cfg-channel').value=d.channel;}");
  sendChunk("}).catch(function(){document.getElementById('espnow-cfg-status').innerHTML='<em style=color:red>Errore caricamento stato</em>';});");
  sendChunk("}");
  sendChunk("updateEspNowSettings();setInterval(updateEspNowSettings,3000);");
  sendChunk("function espnowSaveSettings(){");
  sendChunk("var role=document.getElementById('espnow-cfg-role').value;");
  sendChunk("var ch=document.getElementById('espnow-cfg-channel').value;");
  sendChunk("var body='espnow_role='+encodeURIComponent(role)+'&espnow_channel='+encodeURIComponent(ch);");
  sendChunk("fetch('/espnow/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body})");
  sendChunk(".then(function(r){return r.json();}).then(function(d){if(d.ok){updateEspNowSettings();}else{alert('Errore: '+(d.error||'Init failed'));}})");
  sendChunk(".catch(function(){alert('Errore salvataggio ESP-NOW');});");
  sendChunk("}");
  sendChunk("function espnowStopSettings(){fetch('/espnow/stop',{method:'POST'}).then(function(){updateEspNowSettings();});}");
  sendChunk("function espnowSendCmd(){");
  sendChunk("var cmd=document.getElementById('espnow-cmd-sel').value;");
  sendChunk("var dst=document.getElementById('espnow-dst').value.trim()||'0xFFFF';");
  sendChunk("var body='cmd='+encodeURIComponent(cmd)+'&dst='+encodeURIComponent(dst)+'&param=0';");
  sendChunk("fetch('/espnow/command',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body})");
  sendChunk(".then(function(r){return r.json();}).then(function(d){document.getElementById('espnow-cmd-result').innerHTML=d.sent?'<span style=color:green>&#x2713; Inviato cmd_uid='+d.cmd_uid+'</span>':'<span style=color:red>&#x2717; Errore</span>';})");
  sendChunk(".catch(function(){document.getElementById('espnow-cmd-result').innerHTML='<span style=color:red>Errore</span>';});");
  sendChunk("}");
  sendChunk("</script>");

  sendChunk("</div>");

  // ---- ESP-NOW Rete (Network ID + PSK) card ----
  sendChunk("<div class='card'><h2>&#x1F512; ESP-NOW Rete</h2>");
  sendChunk("<div style='margin-bottom:12px;'>");
  sendChunk("<label>Network ID (hex):</label><br>");
  sendChunk("<input id='enNetId' type='text' maxlength='10' placeholder='0x52544B4E' style='width:160px;margin-bottom:8px;'><br>");
  sendChunk("<label>PSK (max 32 char):</label><br>");
  sendChunk("<input id='enPsk' type='password' maxlength='32' placeholder='(vuoto = solo CRC16)' style='width:220px;margin-bottom:8px;'><br>");
  sendChunk("</div>");
  sendChunk("<button class='btn btn-success' onclick='enSaveNetCfg()'>&#x1F4BE; Salva</button>");
  sendChunk("<div id='enNetMsg' style='margin-top:8px;font-size:0.9em;'></div>");
  sendChunk("<script>");
  sendChunk("function enSaveNetCfg(){");
  sendChunk("var netId=document.getElementById('enNetId').value.trim();");
  sendChunk("var psk=document.getElementById('enPsk').value;");
  sendChunk("var body=JSON.stringify({network_id:netId,psk:psk});");
  sendChunk("fetch('/api/espnow/config/set',{method:'POST',headers:{'Content-Type':'application/json'},body:body})");
  sendChunk(".then(function(r){return r.json();}).then(function(d){document.getElementById('enNetMsg').innerHTML=d.ok?'<span style=color:green>&#x2713; Salvato</span>':'<span style=color:red>&#x2717; '+( d.err||'Errore')+'</span>';})");
  sendChunk(".catch(function(){document.getElementById('enNetMsg').innerHTML='<span style=color:red>Errore</span>';});");
  sendChunk("}");
  sendChunk("(function(){fetch('/api/espnow/config/get').then(function(r){return r.json();}).then(function(d){");
  sendChunk("if(d.ok){");
  sendChunk("if(d.network_id)document.getElementById('enNetId').value='0x'+d.network_id.toString(16).toUpperCase();");
  sendChunk("document.getElementById('enPsk').placeholder=d.psk_set?'(PSK impostato)':'(vuoto = solo CRC16)';");
  sendChunk("}}).catch(function(){});})();");
  sendChunk("</script>");
  sendChunk("</div>");
}

// ========================================================================
// SETTINGS PAGE (/settings) - WiFi, NTP, ZED rate, System
// ========================================================================

// ========================================================================
// SETTINGS HUB — /settings
// ========================================================================

static void handleSettingsPage() {
  sendHeader("Settings", "settings");

  sendChunk("<h2 class='settings-hub-title'>Settings</h2>");
  sendChunk("<p class='settings-hub-sub'>Select a category to configure.</p>");
  sendChunk("<div class='settings-grid'>");

  // --- Connectivity card ---
  {
    std::vector<WifiCred> wl;
    WifiProfiles::loadFromFlash(wl);
    String summary = String(wl.size()) + " WiFi network" + (wl.size() != 1 ? "s" : "");
    summary += " &middot; BLE: " + String(g_bleEnabled ? "ON" : "OFF");
    String mdns = String(g_mdnsName);
    if (mdns.length() == 0) mdns = "rtkino";
    summary += " &middot; " + htmlEscape(mdns) + ".local";
    sendChunk("<a href='/settings/connectivity' class='settings-card' style='border-left-color:#3498db'>");
    sendChunk("<div class='settings-card-body'>");
    sendChunk("<span class='settings-card-title'>Connectivity</span>");
    sendChunk("<span class='settings-card-summary'>" + summary + "</span>");
    sendChunk("</div><span class='settings-card-arrow'>&#8250;</span></a>");
  }

  // --- GNSS card ---
  {
    String summary = "Rate: " + (lastRateSet.length() ? lastRateSet : "default");
    summary += " &middot; TCP: " + String(getStreamModeName());
    sendChunk("<a href='/settings/gnss' class='settings-card' style='border-left-color:#27ae60'>");
    sendChunk("<div class='settings-card-body'>");
    sendChunk("<span class='settings-card-title'>GNSS</span>");
    sendChunk("<span class='settings-card-summary'>" + summary + "</span>");
    sendChunk("</div><span class='settings-card-arrow'>&#8250;</span></a>");
  }

  // --- Survey card ---
  {
    std::vector<AntennaRec> antennas;
    loadAntennas(antennas);
    int catCount = PointCodes::getCategoryCount();
    String summary = String(antennas.size()) + " antenna" + (antennas.size() != 1 ? "s" : "");
    summary += " &middot; " + String(catCount) + " code categor" + (catCount != 1 ? "ies" : "y");
    sendChunk("<a href='/settings/survey' class='settings-card' style='border-left-color:#e67e22'>");
    sendChunk("<div class='settings-card-body'>");
    sendChunk("<span class='settings-card-title'>Survey</span>");
    sendChunk("<span class='settings-card-summary'>" + summary + "</span>");
    sendChunk("</div><span class='settings-card-arrow'>&#8250;</span></a>");
  }

  // --- Time card ---
  {
    String srv = String(g_ntpServer);
    if (srv.length() == 0) srv = "not set";
    String summary = "NTP: " + htmlEscape(srv);
    summary += " &middot; " + String(timeSourceName(g_timeSource));
    sendChunk("<a href='/settings/time' class='settings-card' style='border-left-color:#9b59b6'>");
    sendChunk("<div class='settings-card-body'>");
    sendChunk("<span class='settings-card-title'>Time</span>");
    sendChunk("<span class='settings-card-summary'>" + summary + "</span>");
    sendChunk("</div><span class='settings-card-arrow'>&#8250;</span></a>");
  }

  // --- System card ---
  {
    bool buzzerOn = (g_buzzer && g_buzzer->isEnabled());
    String summary = String(buzzerOn ? "Audio: ON" : "Audio: OFF");
    summary += " &middot; " + String(OTAManager::getCurrentPartitionInfo());
    sendChunk("<a href='/settings/system' class='settings-card' style='border-left-color:#e74c3c'>");
    sendChunk("<div class='settings-card-body'>");
    sendChunk("<span class='settings-card-title'>System</span>");
    sendChunk("<span class='settings-card-summary'>" + summary + "</span>");
    sendChunk("</div><span class='settings-card-arrow'>&#8250;</span></a>");
  }

  sendChunk("</div>");
  sendFooter();
}

// ========================================================================
// SETTINGS — CONNECTIVITY  /settings/connectivity
// ========================================================================

static void handleSettingsConnectivity() {
  sendHeader("Settings - Connectivity", "settings");

  sendChunk("<div class='settings-breadcrumb'><a href='/settings' class='btn btn-secondary btn-small'>&#8592; Settings</a></div>");

  // WiFi Networks
  std::vector<WifiCred> wifiList;
  WifiProfiles::loadFromFlash(wifiList);
  WifiProfiles::sortByPriority(wifiList);

  sendChunk("<div class='card'><h2>WiFi Networks</h2>");
  if (!wifiList.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Priority</th><th>SSID</th><th>Actions</th></tr>");
    for (size_t i = 0; i < wifiList.size(); ++i) {
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Priority'>" + String(wifiList[i].priority) + "</td>";
      row += "<td data-label='SSID'>" + htmlEscape(wifiList[i].ssid) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/wifi/edit?idx=" + String(i) + "' class='btn btn-small'>&#9998; Edit</a> ";
      row += "<a href='/wifi/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(wifiList[i].ssid) + "?\")'>&#10006; Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table></div>");
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

  // BLE
  renderBluetoothCard();

  // ESP-NOW
  renderEspNowCard();

  // AP Credentials
  sendChunk("<div class='card'><h2>Access Point (AP)</h2>");
  sendChunk("<p>Credentials for RTKino local AP (used when no WiFi is available or via the &ldquo;Start AP&rdquo; button on the home page).</p>");
  sendChunk("<p><strong>IP:</strong> <code>http://192.168.4.1</code></p>");
  sendChunk("<form method='POST' action='/api/ap/config'>");
  sendChunk("<label>SSID:</label>");
  sendChunk("<input name='ap_ssid' value='" + htmlEscape(String(g_apSsid)) + "' maxlength='32' required style='width:220px' placeholder='rtkino_AP'><br>");
  sendChunk("<label>Password:</label>");
  sendChunk("<input name='ap_pass' type='password' value='" + htmlEscape(String(g_apPass)) + "' maxlength='63' style='width:220px' placeholder='Leave empty for open network'><br>");
  sendChunk("<small style='color:#666'>Min 8 characters for WPA2, or leave empty for open network.</small><br>");
  sendChunk("<button type='submit' class='btn' style='margin-top:12px'>Save AP Credentials</button>");
  sendChunk("</form>");
  sendChunk("<p style='font-size:0.85em;color:#888;margin-top:10px'>Takes effect on next AP start.</p>");
  sendChunk("</div>");

  sendFooter();
}

// ========================================================================
// SETTINGS — GNSS  /settings/gnss
// ========================================================================

static void handleSettingsGnss() {
  sendHeader("Settings - GNSS", "settings");

  sendChunk("<div class='settings-breadcrumb'><a href='/settings' class='btn btn-secondary btn-small'>&#8592; Settings</a></div>");

  // Update Rate
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

  // ZED-F9P Reset
  sendChunk("<div class='card'><h2>ZED-F9P Reset</h2>");
  sendChunk("<button onclick=\"resetZed('hot')\" class='btn'>Hot Reset</button> ");
  sendChunk("<button onclick=\"resetZed('cold')\" class='btn btn-danger'>Cold Reset</button>");
  sendChunk("<p style='font-size:0.85em;color:#666;margin-top:8px'>");
  sendChunk("<b>Hot Reset:</b> Keeps ephemeris, fast restart (~5s)<br>");
  sendChunk("<b>Cold Reset:</b> Clears everything, slow restart (~30s)");
  sendChunk("</p>");
  sendChunk("<script>");
  sendChunk("function resetZed(type){");
  sendChunk("var coldMsg='Cold Reset will clear all ephemeris data.\\nFix will take ~30 seconds.\\n\\nContinue?';");
  sendChunk("var hotMsg='Hot Reset will restart ZED-F9P keeping ephemeris.\\n\\nContinue?';");
  sendChunk("var msg=type==='cold'?coldMsg:hotMsg;");
  sendChunk("if(confirm('WARNING: '+msg)){");
  sendChunk("fetch('/api/zed/reset?type='+type).then(r=>r.text()).then(t=>{alert(t);location.reload();})");
  sendChunk(".catch(err=>{alert('Error resetting ZED: '+(err.message||err));});}}");
  sendChunk("</script></div>");

  // TCP Stream Mode
  sendChunk("<div class='card'><h2>TCP Stream Mode</h2>");
  sendChunk("<p>Data format sent over TCP stream to connected viewer apps.</p>");
  sendChunk("<p><strong>Current mode:</strong> " + String(getStreamModeName()) + "</p>");
  sendChunk("<button onclick='setMode(\"nmea\")' class='btn'>NMEA+UBX/RTCM</button> ");
  sendChunk("<button onclick='setMode(\"raw\")' class='btn'>RAW (UBX)</button>");
  sendChunk("<script>function setMode(m){fetch('/stream?mode='+m).then(r=>r.text()).then(t=>{alert(t);location.reload();}).catch(err=>{alert('Error: '+(err.message||err));})}</script>");
  sendChunk("</div>");

  sendFooter();
}

// ========================================================================
// SETTINGS — SURVEY  /settings/survey
// ========================================================================

static void handleSettingsSurvey() {
  sendHeader("Settings - Survey", "settings");

  sendChunk("<div class='settings-breadcrumb'><a href='/settings' class='btn btn-secondary btn-small'>&#8592; Settings</a></div>");

  // Antenna Models
  std::vector<AntennaRec> antennas;
  loadAntennas(antennas);

  sendChunk("<div class='card'><h2>Antenna Models</h2>");
  sendChunk("<p>Antenna models with ARP to phase center offsets.</p>");
  if (!antennas.empty()) {
    sendChunk("<div class='responsive-table'>");
    sendChunk("<table><tr><th>#</th><th>Name</th><th>Offset (m)</th><th>Actions</th></tr>");
    for (size_t i = 0; i < antennas.size(); ++i) {
      auto& a = antennas[i];
      String row = "<tr><td data-label='#'>" + String(i) + "</td>";
      row += "<td data-label='Name'>" + htmlEscape(a.name) + "</td>";
      row += "<td data-label='Offset (m)'>" + String(a.offset, 3) + "</td>";
      row += "<td data-label='Actions'>";
      row += "<a href='/antennas/edit?idx=" + String(i) + "' class='btn btn-small'>&#9998; Edit</a> ";
      row += "<a href='/antennas/del?idx=" + String(i) + "' class='btn btn-small btn-danger' onclick='return confirm(\"Delete " + escapeForOnclick(a.name) + "?\")'>&#10006; Delete</a>";
      row += "</td></tr>";
      sendChunk(row);
    }
    sendChunk("</table></div>");
  } else {
    sendChunk("<p><em>No antenna models configured.</em></p>");
  }
  sendChunk("<h3>Add Antenna Model</h3>");
  sendChunk("<form method='POST' action='/antennas/add'>");
  sendChunk("<label>Name:</label><input name='name' required placeholder='e.g. u-blox ANN-MB'><br>");
  sendChunk("<label>Offset (m):</label><input name='offset' type='number' step='0.001' value='0.000' style='width:150px' required> (ARP to phase center)<br>");
  sendChunk("<button type='submit'>Add Antenna</button>");
  sendChunk("</form></div>");

  // Point Codes
  sendChunk("<div class='card'><h2>Point Codes</h2>");
  sendChunk("<p style='color:#666;font-size:13px'>Categories and codes used during the survey. Freely editable; use &ldquo;Reset&rdquo; to restore defaults.</p>");
  sendChunk("<textarea id='codes-ta' style='width:100%;height:300px;font-family:monospace;font-size:12px;border:1px solid #ddd;border-radius:4px;padding:8px'>Loading...</textarea>");
  sendChunk("<div style='margin-top:12px'>");
  sendChunk("<button onclick='saveCodes()' class='btn'>Save</button>");
  sendChunk("<button onclick='resetCodes()' class='btn' style='margin-left:8px;background:#e67e22'>Reset default</button>");
  sendChunk("</div></div>");
  sendChunk("<script>");
  sendChunk("function loadCodesEditor(){fetch('/api/codes').then(r=>r.json()).then(d=>{var ta=document.getElementById('codes-ta');if(ta)ta.value=JSON.stringify(d,null,2);}).catch(function(){});}");
  sendChunk("function saveCodes(){var ta=document.getElementById('codes-ta');if(!ta)return;");
  sendChunk("fetch('/api/codes',{method:'POST',headers:{'Content-Type':'application/json'},body:ta.value})");
  sendChunk(".then(r=>r.json()).then(d=>{alert(d.ok?'Saved!':'Error: '+(d.error||'?'));}).catch(e=>{alert('Network error: '+e);});}");
  sendChunk("function resetCodes(){if(!confirm('Restore default codes?'))return;");
  sendChunk("fetch('/api/codes/reset',{method:'POST'}).then(r=>r.json()).then(d=>{if(d.ok){alert('Reset OK');loadCodesEditor();}}).catch(e=>{alert('Error: '+e);});}");
  sendChunk("loadCodesEditor();");
  sendChunk("</script>");

  sendFooter();
}

// ========================================================================
// SETTINGS — TIME  /settings/time
// ========================================================================

static void handleSettingsTime() {
  sendHeader("Settings - Time", "settings");

  sendChunk("<div class='settings-breadcrumb'><a href='/settings' class='btn btn-secondary btn-small'>&#8592; Settings</a></div>");

  // Refresh globals from flash
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
  for (int i = 0; i < (int)(sizeof(TZ_LIST) / sizeof(TZ_LIST[0])); i++) {
    String sel = (curTz == TZ_LIST[i].posix) ? " selected" : "";
    sendChunk("<option value='" + htmlEscape(String(TZ_LIST[i].posix)) + "'" + sel + ">" +
              htmlEscape(String(TZ_LIST[i].label)) + "</option>");
  }
  sendChunk("</select><br>");
  sendChunk("<button type='submit'>Save</button>");
  sendChunk("</form></div>");

  sendFooter();
}

// ========================================================================
// SETTINGS — SYSTEM  /settings/system
// ========================================================================

static void handleDeviceNameSave() {
  if (!_server->hasArg("device_name")) {
    _server->send(400, "text/plain", "Missing device_name");
    return;
  }
  String name = _server->arg("device_name");
  name.trim();
  if (name.length() == 0) name = "RTKino";
  if (name.length() > 20) {
    _server->send(400, "text/plain", "Name too long (max 20 chars)");
    return;
  }
  for (size_t i = 0; i < name.length(); i++) {
    char c = name[i];
    if (!isalnum(c) && c != '-' && c != '_') {
      _server->send(400, "text/plain", "Invalid characters (a-z A-Z 0-9 - _)");
      return;
    }
  }
  extern bool saveDeviceName(const char*);
  extern void applyDeviceName(const char*);
  if (!saveDeviceName(name.c_str())) {
    _server->send(500, "text/plain", "Failed to save device name");
    return;
  }
  applyDeviceName(name.c_str());
  // Restart BLE with new name if active
  extern bool g_bleEnabled;
  extern uint32_t g_blePasskey;
  if (g_bleEnabled) {
    BLESerial::end();
    delay(300);
    BLESerial::begin(g_bleDeviceName, g_blePasskey);
    extern HardwareSerial RTCMSerial;
    BLESerial::setRxCallback([](const uint8_t* data, size_t len) {
      if (len > 0) RTCMSerial.write(data, len);
    });
  }
  _server->sendHeader("Location", "/settings/system");
  _server->send(303);
}

static void handleSettingsSystem() {
  sendHeader("Settings - System", "settings");

  sendChunk("<div class='settings-breadcrumb'><a href='/settings' class='btn btn-secondary btn-small'>&#8592; Settings</a></div>");

  // Device Name
  sendChunk("<div class='card'><h2>Device Name</h2>");
  sendChunk("<p>Identifies this device in the WebUI header, BLE advertising and mDNS (<code>.local</code>).</p>");
  sendChunk("<p><strong>Current:</strong> <code>" + htmlEscape(String(g_deviceName)) + "</code> &nbsp; mDNS: <code>http://" + htmlEscape(String(g_deviceName)) + ".local/</code></p>");
  sendChunk("<form method='POST' action='/api/device/name'>");
  sendChunk("<label>Name (a-z A-Z 0-9 - _, max 20 chars):</label>");
  sendChunk("<input name='device_name' value='" + htmlEscape(String(g_deviceName)) + "' maxlength='20' required style='width:220px'><br>");
  sendChunk("<small style='color:#666'>Examples: RTKino-Base1 &nbsp; Rover-2 &nbsp; Base-Office</small><br>");
  sendChunk("<button type='submit' class='btn' style='margin-top:10px'>Save &amp; Apply</button>");
  sendChunk("</form>");
  sendChunk("<p style='font-size:0.85em;color:#888;margin-top:8px'>BLE restarts automatically. mDNS applies immediately.</p>");
  sendChunk("</div>");

  // ESP32 Reboot
  sendChunk("<div class='card'><h2>ESP32</h2>");
  sendChunk("<button onclick=\"rebootEsp()\" class='btn btn-danger'>Reboot ESP32</button>");
  sendChunk("<p style='font-size:0.85em;color:#666;margin-top:8px'>Connection will be lost during reboot.</p>");
  sendChunk("<script>function rebootEsp(){if(confirm('Reboot ESP32?\\nConnection will be lost.')){location.href='/reboot';}}</script>");
  sendChunk("</div>");

  // Audio Settings
  sendChunk("<div class='card'><h2>Audio Settings</h2>");
  sendChunk("<p><em>Buzzer configuration for RTK events (GPIO 5)</em></p>");
  sendChunk("<form method='POST' action='/audio/save'>");
  bool buzzerEnabled = (g_buzzer && g_buzzer->isEnabled());
  String checkedAttr = buzzerEnabled ? " checked" : "";
  sendChunk("<label><input type='checkbox' name='enabled' value='1'" + checkedAttr + "> Enable Audio Alerts</label><br>");
  sendChunk("<p style='margin-top:10px'><strong>Sound Events:</strong></p>");
  sendChunk("<p>&#8226; RTK Fix Acquired: 2 ascending beeps</p>");
  sendChunk("<p>&#8226; RTK Fix Lost: 3 descending beeps</p>");
  sendChunk("<button type='submit' class='btn'>Save Audio Settings</button>");
  sendChunk("</form>");
  sendChunk("<h4 style='margin-top:20px'>Custom Melody (JSON)</h4>");
  sendChunk("<form action='/audio/upload' method='POST' enctype='multipart/form-data'>");
  sendChunk("<input type='file' name='melody' accept='.json'>");
  sendChunk("<button type='submit' class='btn'>Upload</button>");
  sendChunk("</form>");
  sendChunk("<p style='font-size:0.85em;color:#666'>Format: {\"name\":\"RTK Fixed\",\"tones\":[{\"freq\":880,\"duration\":100}]}</p>");
  sendChunk("</div>");

  // Backup & Restore
  sendChunk("<div class='card'><h2>Backup &amp; Restore</h2>");
  sendChunk("<div style='margin-bottom:20px'>");
  sendChunk("<button onclick=\"window.location='/api/config/export'\" class='btn'>Export Config</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Download all settings as JSON</span>");
  sendChunk("</div>");
  sendChunk("<div style='margin-bottom:20px'>");
  sendChunk("<button onclick='syncToSD()' class='btn'>Sync Flash&#8594;SD</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Copy settings from flash to SD card</span>");
  sendChunk("<p id='syncStatus' style='margin-top:8px'></p>");
  sendChunk("</div>");
  sendChunk("<div>");
  sendChunk("<input type='file' id='importFile' accept='.json' style='margin-bottom:10px'><br>");
  sendChunk("<button onclick='importConfig()' class='btn'>Import Config</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Upload and restore settings</span>");
  sendChunk("<p id='importStatus' style='margin-top:10px'></p>");
  sendChunk("</div></div>");
  sendChunk("<script>");
  sendChunk("function importConfig(){const f=document.getElementById('importFile').files[0];");
  sendChunk("if(!f){alert('Select a file first');return;}");
  sendChunk("const r=new FileReader();r.onload=function(e){");
  sendChunk("fetch('/api/config/import',{method:'POST',body:e.target.result,headers:{'Content-Type':'application/json'}})");
  sendChunk(".then(r=>r.json()).then(d=>{");
  sendChunk("if(d.success){document.getElementById('importStatus').innerHTML='<span style=\"color:green\">Import successful! Reboot recommended.</span>';}");
  sendChunk("else{document.getElementById('importStatus').innerHTML='<span style=\"color:red\">Import failed: '+d.errors+'</span>';}");
  sendChunk("}).catch(e=>{document.getElementById('importStatus').innerHTML='<span style=\"color:red\">Error: '+e+'</span>';});};r.readAsText(f);}");
  sendChunk("function syncToSD(){document.getElementById('syncStatus').innerHTML='<span style=\"color:#888\">Syncing...</span>';");
  sendChunk("fetch('/api/config/sync').then(r=>r.text()).then(t=>{document.getElementById('syncStatus').innerHTML='<span style=\"color:green\">'+t+'</span>';})");
  sendChunk(".catch(e=>{document.getElementById('syncStatus').innerHTML='<span style=\"color:red\">Error: '+e+'</span>';});}");
  sendChunk("</script>");

  // Firmware OTA
  sendChunk("<div class='card'><h2>Firmware Update (OTA)</h2>");
  sendChunk("<p>Update RTKino firmware over-the-air via web browser.</p>");
  sendChunk("<div style='background:#f8f9fa;padding:12px;border-radius:4px;margin:12px 0;'>");
  sendChunk("<strong>Current:</strong> " + String(OTAManager::getCurrentPartitionInfo()));
  sendChunk("<br><strong>Next:</strong> " + String(OTAManager::getNextPartitionInfo()));
  sendChunk("</div>");
  sendChunk("<form id='otaUploadForm' method='POST' action='/firmware/upload' enctype='multipart/form-data'>");
  sendChunk("<input type='file' name='firmware' id='otaFirmwareFile' accept='.bin' required style='margin-bottom:10px;'><br>");
  sendChunk("<button type='submit' id='otaUploadBtn' class='btn btn-success'>Upload &amp; Update</button>");
  sendChunk("</form>");
  sendChunk("<div id='otaProgressContainer' style='display:none;margin-top:16px;'>");
  sendChunk("<div style='background:#e9ecef;border-radius:4px;height:24px;overflow:hidden;'>");
  sendChunk("<div id='otaProgressBar' style='background:#28a745;height:100%;width:0%;transition:width 0.3s;text-align:center;line-height:24px;color:white;font-weight:bold;'></div>");
  sendChunk("</div>");
  sendChunk("<p id='otaProgressText' style='margin-top:8px;'></p></div>");
  sendChunk("<div style='background:#fff3cd;border-left:4px solid #ffc107;padding:12px;margin-top:16px;'>");
  sendChunk("<strong>Important:</strong><ul style='margin:8px 0;padding-left:20px;'>");
  sendChunk("<li>Do NOT disconnect power during update (2-3 minutes)</li>");
  sendChunk("<li>Device will reboot automatically after update</li>");
  sendChunk("<li>All settings on SD card are preserved</li>");
  sendChunk("<li>Failed updates rollback automatically to previous firmware</li>");
  sendChunk("</ul></div>");
  sendChunk("<script>");
  sendChunk("document.getElementById('otaUploadForm').addEventListener('submit',function(e){e.preventDefault();");
  sendChunk("var file=document.getElementById('otaFirmwareFile').files[0];");
  sendChunk("if(!file){alert('Please select a file');return;}");
  sendChunk("if(!file.name.endsWith('.bin')){alert('Please select a .bin file');return;}");
  sendChunk("var sizeMB=(file.size/(1024*1024)).toFixed(2);");
  sendChunk("if(!confirm('Update firmware?\\nFile: '+file.name+' ('+sizeMB+' MB)\\nDevice will reboot after update.')){return;}");
  sendChunk("document.getElementById('otaUploadBtn').disabled=true;");
  sendChunk("document.getElementById('otaProgressContainer').style.display='block';");
  sendChunk("var formData=new FormData();formData.append('firmware',file);");
  sendChunk("var xhr=new XMLHttpRequest();");
  sendChunk("xhr.upload.addEventListener('progress',function(e){if(e.lengthComputable){");
  sendChunk("var percent=Math.round((e.loaded/e.total)*100);");
  sendChunk("document.getElementById('otaProgressBar').style.width=percent+'%';");
  sendChunk("document.getElementById('otaProgressBar').textContent=percent+'%';");
  sendChunk("document.getElementById('otaProgressText').textContent='Uploading: '+percent+'%';}});");
  sendChunk("xhr.addEventListener('load',function(){if(xhr.status===200){");
  sendChunk("document.getElementById('otaProgressText').innerHTML='<span style=\"color:#28a745\">Update successful! Rebooting...</span>';");
  sendChunk("setTimeout(function(){location.href='/';},5000);}else{");
  sendChunk("document.getElementById('otaProgressText').innerHTML='<span style=\"color:#dc3545\">Failed: '+xhr.responseText+'</span>';");
  sendChunk("document.getElementById('otaUploadBtn').disabled=false;}});");
  sendChunk("xhr.addEventListener('error',function(){document.getElementById('otaProgressText').innerHTML='<span style=\"color:#dc3545\">Upload error</span>';");
  sendChunk("document.getElementById('otaUploadBtn').disabled=false;});");
  sendChunk("xhr.open('POST','/firmware/upload',true);xhr.send(formData);});");
  sendChunk("</script></div>");

  // Factory Reset
  sendChunk("<div class='card'><h2>Factory Reset</h2>");
  sendChunk("<p style='color:#888;font-size:13px;margin-bottom:15px'>Erase all configuration, survey data, and stakeout files from internal flash. RTKino will restart as if it were brand new.</p>");
  sendChunk("<div style='margin-bottom:15px'>");
  sendChunk("<button onclick='factoryReset(false)' class='btn btn-danger'>Reset Flash Only</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Keeps SD card data intact</span>");
  sendChunk("</div><div>");
  sendChunk("<button onclick='factoryReset(true)' class='btn btn-danger'>Reset Flash + SD</button>");
  sendChunk("<span style='margin-left:10px;color:#666'>Wipes everything from both flash and SD</span>");
  sendChunk("</div>");
  sendChunk("<p id='resetStatus' style='margin-top:12px'></p></div>");
  sendChunk("<script>");
  sendChunk("function factoryReset(wipeSD){");
  sendChunk("var msg=wipeSD?'WARNING: This will ERASE ALL DATA from flash AND SD card.\\n\\nWiFi, NTRIP, bases, surveys, stakeout - everything will be lost.\\n\\nAre you absolutely sure?'");
  sendChunk(":'WARNING: This will ERASE ALL DATA from internal flash.\\n\\nSD card data will NOT be erased.\\n\\nContinue?';");
  sendChunk("if(!confirm(msg))return;");
  sendChunk("if(wipeSD&&!confirm('LAST CHANCE - Wipe SD card too?\\nThis cannot be undone.'))return;");
  sendChunk("document.getElementById('resetStatus').innerHTML='<span style=\"color:#e67e22\">Factory reset in progress...</span>';");
  sendChunk("fetch('/api/factory-reset?wipeSD='+(wipeSD?'1':'0')).then(r=>r.json()).then(d=>{");
  sendChunk("if(d.status==='ok'){document.getElementById('resetStatus').innerHTML='<span style=\"color:green\">Reset complete. Rebooting...</span>';");
  sendChunk("setTimeout(function(){location.reload();},5000);}");
  sendChunk("else{document.getElementById('resetStatus').innerHTML='<span style=\"color:red\">Error: '+(d.error||'unknown')+'</span>';}");
  sendChunk("}).catch(e=>{document.getElementById('resetStatus').innerHTML='<span style=\"color:red\">'+e+'</span>';});}");
  sendChunk("</script>");

  // System Logs
  sendChunk("<div class='card'><h2>System Logs</h2>");
  sendChunk("<p>View and download system event logs (max 3 files).</p>");
  sendChunk("<div id='logsList'><p>Loading...</p></div></div>");
  sendChunk("<script>");
  sendChunk("function loadLogs(){fetch('/api/logs').then(r=>r.json()).then(d=>{");
  sendChunk("let html='';");
  sendChunk("if(d.files&&d.files.length>0){html='<ul class=\"log-list\">';");
  sendChunk("for(let f of d.files){html+='<li><strong>'+f+'</strong><br>';");
  sendChunk("html+='<a href=\"/api/logs/download?file='+encodeURIComponent(f)+'\" class=\"btn btn-small\">Download</a> ';");
  sendChunk("html+='<a href=\"/api/logs/delete?file='+encodeURIComponent(f)+'\" class=\"btn btn-small btn-danger\" onclick=\"return confirm(\\'Delete log?\\');\">Delete</a>';");
  sendChunk("html+='</li>';}html+='</ul>';}else{html='<p><em>No system logs found.</em></p>';}");
  sendChunk("document.getElementById('logsList').innerHTML=html;");
  sendChunk("}).catch(e=>{document.getElementById('logsList').innerHTML='<p style=\"color:red\">Error loading logs</p>';});}");
  sendChunk("loadLogs();");
  sendChunk("</script>");

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
  _server->sendHeader("Location", "/settings/connectivity");
  _server->send(303);
}

static void handleWifiDel() {
  if (!_server->hasArg("idx")) { _server->send(400,"text/plain","idx missing"); return; }
  size_t idx = _server->arg("idx").toInt();
  if (!WifiProfiles::removeAtFlash(idx)) {
    _server->send(400,"text/plain","Delete failed");
    return;
  }
  _server->sendHeader("Location", "/settings/connectivity");
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
  sendChunk("<a class='btn' href='/settings/connectivity'>Cancel</a>");
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

  _server->sendHeader("Location", "/settings/connectivity");
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
  _server->sendHeader("Location", "/settings/survey");
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
  _server->sendHeader("Location", "/settings/survey");
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
  sendChunk("<a class='btn' href='/settings/survey'>Cancel</a>");
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
  _server->sendHeader("Location", "/settings/survey");
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

  _server->sendHeader("Location", "/settings/time");
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

  _server->sendHeader("Location", "/settings/time");
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

  _server->sendHeader("Location", "/settings/system");
  _server->send(303);
}

// ========================================================================
// BLE SETTINGS HANDLER
// ========================================================================

static void handleBleSettings() {
  // Validate parameters
  if (!_server->hasArg("ble_enable") || !_server->hasArg("ble_pin")) {
    _server->send(400, "text/plain", "Missing parameters");
    return;
  }

  bool newEnabled = (_server->arg("ble_enable") == "1");
  String newPinStr = _server->arg("ble_pin");
  newPinStr.trim();

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
  
  // Save PIN to flash (persistent)
  if (!saveBlePin(newPin)) {
    _server->send(500, "text/plain", "Failed to save PIN");
    return;
  }
  
  // Apply new PIN
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
  _server->sendHeader("Location", "/settings/connectivity");
  _server->send(303, "text/plain", "Bluetooth settings saved");
}

// ========================================================================
// NTRIP IN CRUD HANDLERS
// ========================================================================

// ========================================================================
// NTRIP SOURCETABLE BROWSER
// ========================================================================

static bool isJsonNumber(const String& s) {
  if (s.isEmpty()) return false;
  int i = 0;
  if (s[i] == '-') i++;
  if (i >= (int)s.length() || !isdigit((unsigned char)s[i])) return false;
  bool dot = false;
  for (; i < (int)s.length(); i++) {
    if (s[i] == '.') { if (dot) return false; dot = true; }
    else if (!isdigit((unsigned char)s[i])) return false;
  }
  return true;
}

static String _b64st(const String& in) {
  static const char t[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String o; uint8_t b[3]; int i;
  for (size_t x = 0; x < in.length();) {
    i = 0; memset(b, 0, 3);
    while (i < 3 && x < in.length()) b[i++] = in[x++];
    o += t[(b[0]&0xfc)>>2]; o += t[((b[0]&3)<<4)|((b[1]&0xf0)>>4)];
    o += (i>1) ? t[((b[1]&0xf)<<2)|((b[2]&0xc0)>>6)] : '=';
    o += (i>2) ? t[b[2]&0x3f] : '=';
  }
  return o;
}

static void handleNtripSourcetable() {
  if (WiFi.status() != WL_CONNECTED) {
    _server->send(503, "application/json", F("{\"ok\":false,\"err\":\"WiFi non connesso\"}"));
    return;
  }
  String host = _server->arg("host");
  int port = _server->arg("port").toInt();
  if (host.isEmpty() || port == 0) {
    _server->send(400, "application/json", F("{\"ok\":false,\"err\":\"host e porta richiesti\"}"));
    return;
  }
  String user = _server->arg("user");
  String pass = _server->arg("pass");

  WiFiClient cl;
  cl.setTimeout(1000);
  if (!cl.connect(host.c_str(), port)) {
    _server->send(502, "application/json", F("{\"ok\":false,\"err\":\"Impossibile connettersi al caster\"}"));
    return;
  }
  cl.setNoDelay(true);
  cl.print("GET / HTTP/1.0\r\nUser-Agent: NTRIP RTKino/1.0\r\nAuthorization: Basic ");
  cl.print(_b64st(user + ":" + pass));
  cl.print("\r\n\r\n");

  // Skip HTTP/NTRIP header — wait for blank line, check for 200
  bool is200 = false;
  uint32_t t0 = millis();
  while (cl.connected() && millis() - t0 < 6000) {
    String line = cl.readStringUntil('\n');
    line.trim();
    if (line.startsWith("SOURCETABLE 200") ||
        (line.startsWith("HTTP/") && line.indexOf(" 200 ") > 0)) {
      is200 = true;
    }
    if (line.isEmpty()) break;
  }
  if (!is200) {
    cl.stop();
    _server->send(502, "application/json", F("{\"ok\":false,\"err\":\"Caster ha rifiutato la richiesta (credenziali errate?)\"}"));
    return;
  }

  // Stream-parse STR lines, build JSON
  String json = "{\"ok\":true,\"mounts\":[";
  bool first = true;
  int count = 0;
  t0 = millis();
  while (cl.connected() && millis() - t0 < 10000 && count < 300) {
    String line = cl.readStringUntil('\n');
    if (line.isEmpty() && !cl.available()) { delay(20); continue; }
    line.trim();
    if (line.startsWith("ENDSOURCETABLE")) break;
    if (!line.startsWith("STR;")) continue;

    // STR;id(1);identifier(2);format(3);fmtdetail(4);carrier(5);nav(6);network(7);country(8);lat(9);lon(10);...
    String f[17]; int fc = 0, fp = 0;
    for (int i = 0; i < (int)line.length() && fc < 16; i++) {
      if (line[i] == ';') { f[fc++] = line.substring(fp, i); fp = i + 1; }
    }
    f[fc] = line.substring(fp);

    for (int j = 0; j <= min(fc, 16); j++) f[j].trim();
    if (f[1].isEmpty()) continue;
    if (!first) json += ',';
    first = false;
    json += "{\"id\":\"" + jsonEscape(f[1]) + "\"";
    if (fc >= 3  && f[3].length())  json += ",\"fmt\":\"" + jsonEscape(f[3])  + "\"";
    if (fc >= 6  && f[6].length())  json += ",\"nav\":\"" + jsonEscape(f[6])  + "\"";
    if (fc >= 8  && f[8].length())  json += ",\"ctr\":\"" + jsonEscape(f[8])  + "\"";
    if (fc >= 9  && isJsonNumber(f[9]))  json += ",\"lat\":" + f[9];
    if (fc >= 10 && isJsonNumber(f[10])) json += ",\"lon\":" + f[10];
    json += '}';
    count++;
  }
  cl.stop();
  json += "]}";
  _server->send(200, "application/json", json);
}

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
  int newIdx = (int)v.size();  // index of the profile we're about to add
  v.push_back(n);
  if(!saveNtripInList(v, last)){ _server->send(500,"text/plain","Save failed"); return; }
  // Auto-select the new profile so ntripClient is ready to use immediately
  _server->sendHeader("Location", "/ntrip/select?idx=" + String(newIdx)); _server->send(303);
}

static void handleNtripDel() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx = _server->arg("idx").toInt();
  std::vector<NtripIn> v; int last=-1; loadNtripInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v.erase(v.begin()+idx);
  if (last >= (int)v.size()) last = (int)v.size()-1;
  if(!saveNtripInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/rover/connections"); _server->send(303);
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
  sendChunk("<label>Host:</label><input id='eHost' name='host' value='" + htmlEscape(n.host) + "' required><br>");
  sendChunk("<label>Port:</label><input id='ePort' name='port' type='number' value='" + String(n.port) + "' required><br>");
  sendChunk("<label>User:</label><input id='eUser' name='user' value='" + htmlEscape(n.user) + "'><br>");
  sendChunk("<label>Password:</label><input id='ePwd' name='pwd' type='password' value='" + htmlEscape(n.pass) + "'><br>");
  sendChunk("<label>Mountpoint:</label><div style='display:flex;gap:6px;align-items:center;max-width:320px;'>");
  sendChunk("<input id='eMount' name='mount' value='" + htmlEscape(n.mount) + "' required style='flex:1;'>");
  sendChunk("<button type='button' id='eBrowseBtn' onclick='browseMount(\"eMount\",\"eHost\",\"ePort\",\"eUser\",\"ePwd\",\"eStResult\",\"eBrowseBtn\")'>Browse</button>");
  sendChunk("</div><div id='eStResult' style='margin-top:8px;overflow-x:auto;'></div><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/rover/connections'>Cancel</a>");
  sendChunk("</form></div>");
  sendChunk("<script>");
  sendChunk("function browseMount(mId,hId,pId,uId,wId,rId,bId){");
  sendChunk("var h=document.getElementById(hId).value.trim(),p=document.getElementById(pId).value.trim();");
  sendChunk("if(!h||!p){alert('Inserisci host e porta prima di sfogliare');return;}");
  sendChunk("var btn=document.getElementById(bId);btn.disabled=true;btn.textContent='Caricamento...';");
  sendChunk("var u=document.getElementById(uId).value.trim(),w=document.getElementById(wId).value.trim();");
  sendChunk("fetch('/ntrip/sourcetable?host='+encodeURIComponent(h)+'&port='+p+'&user='+encodeURIComponent(u)+'&pass='+encodeURIComponent(w))");
  sendChunk(".then(function(r){return r.json();}).then(function(d){");
  sendChunk("btn.disabled=false;btn.textContent='Browse';");
  sendChunk("var div=document.getElementById(rId);");
  sendChunk("if(!d.ok){div.innerHTML='<p style=\"color:red\">'+d.err+'</p>';return;}");
  sendChunk("if(!d.mounts||d.mounts.length===0){div.innerHTML='<p><em>Nessun mountpoint trovato</em></p>';return;}");
  sendChunk("var html='<table style=\"font-size:13px\"><tr><th>ID</th><th>Formato</th><th>Nav</th><th>Paese</th><th>Lat</th><th>Lon</th></tr>';");
  sendChunk("d.mounts.forEach(function(m){");
  sendChunk("html+='<tr style=\"cursor:pointer\" onclick=\"document.getElementById(\\''+mId+'\\').value=\\''+m.id+'\\'\">'+");
  sendChunk("'<td><strong>'+m.id+'</strong></td><td>'+(m.fmt||'')+'</td><td>'+(m.nav||'')+'</td><td>'+(m.ctr||'')+'</td><td>'+(m.lat||'')+'</td><td>'+(m.lon||'')+'</td></tr>';");
  sendChunk("});html+='</table><p style=\"font-size:12px;color:#666\">Clicca una riga per selezionare il mountpoint</p>';");
  sendChunk("div.innerHTML=html;");
  sendChunk("}).catch(function(e){btn.disabled=false;btn.textContent='Browse';alert('Errore: '+e.message);});}");
  sendChunk("</script>");
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
  _server->sendHeader("Location","/rover/connections"); _server->send(303);
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

  // FIRST: disable flag so the currently active session, if any, is stopped.
  // Selecting a profile must NOT auto-activate NTRIP.
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

  // Create new client using global variables, but keep it INACTIVE.
  // Remove leading slash from mountpoint if present
  if (mountpoint.startsWith("/")) mountpoint.remove(0, 1);

  ntripClient = new (std::nothrow) NtripClient(ntrip_host.c_str(), ntrip_port, mountpoint.c_str(), ntrip_user.c_str(), ntrip_pass.c_str());
  if (!ntripClient) {
    ntripEnabled = false;
    ntripUnlock();
    _server->send(500, "text/plain", "Out of memory");
    return;
  }
  ntripClient->setGgaMinPeriodMs(5000);

  ntripUnlock();
  // === END LOCK ===

  oledSetNtrip(false);
  oledPrintln(String("[NTRIP] Profile selected: ") + profileName);

  // Try to save LAST, but don't fail the operation if save fails
  // System is already working from RAM variables set above
  last = idx;
  if (!saveNtripInList(v, last)) {
    // Log the failure but don't abort - client is already running
    oledPrintln("[NTRIP] Warning: Failed to save LAST to SD");
  }

  _server->sendHeader("Location", "/rover/connections");
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
  _server->sendHeader("Location","/rover/connections"); _server->send(303);
}

static void handleLanInDel() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<TcpIn> v; int last=-1; loadTcpInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v.erase(v.begin()+idx);
  if (last >= (int)v.size()) last = (int)v.size()-1;
  if(!saveTcpInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/rover/connections"); _server->send(303);
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
  sendChunk("<a class='btn' href='/rover/connections'>Cancel</a>");
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
  _server->sendHeader("Location","/rover/connections"); _server->send(303);
}

static void handleLanInSelect() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<TcpIn> v; int last=-1; loadTcpInList(v,last);
  if(idx<0 || idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  last=idx;
  if(!saveTcpInList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  // update global variables for quick-start
  tcpin_host = v[idx].host; tcpin_port = v[idx].port;
  _server->sendHeader("Location","/rover/connections"); _server->send(303);
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

static void handleBaseStop() {
  stopBaseMode();
  stopCasterOut();
  stopTcpOut();
  _server->send(200, "text/plain", "Base stopped");
}

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
  int baseLast=-1; std::vector<BaseRec> v; loadBases(v, baseLast);
  uint16_t selStid = (baseLast>=0 && baseLast<(int)v.size()) ? v[baseLast].stid : 0;
  v.push_back(b);
  std::sort(v.begin(), v.end(), [](const BaseRec&a,const BaseRec&b){return a.stid<b.stid;});
  if (selStid>0) { baseLast=-1; for(int i=0;i<(int)v.size();i++) if(v[i].stid==selStid){baseLast=i;break;} }
  if(!saveBases(v, baseLast)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base/stations"); _server->send(303);
}

static void handleBasesDel() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt();
  int baseLast=-1; std::vector<BaseRec> v; loadBases(v, baseLast);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v.erase(v.begin()+idx);
  if(baseLast==idx) baseLast=-1; else if(baseLast>idx) baseLast--;
  if(!saveBases(v, baseLast)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base/stations"); _server->send(303);
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
  sendChunk("<a class='btn' href='/base/stations'>Cancel</a>");
  sendChunk("</form></div>");
  sendFooter();
}

static void handleBasesUpdate() {
  if(!_server->hasArg("idx")||!_server->hasArg("name")||!_server->hasArg("lat")||!_server->hasArg("lon")||!_server->hasArg("alt")){
    _server->send(400,"text/plain","Parameters missing"); return;
  }
  int idx=_server->arg("idx").toInt(); int baseLast=-1; std::vector<BaseRec> v; loadBases(v, baseLast);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  uint16_t selStid = (baseLast>=0 && baseLast<(int)v.size()) ? v[baseLast].stid : 0;
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
  if (selStid>0) { baseLast=-1; for(int i=0;i<(int)v.size();i++) if(v[i].stid==selStid){baseLast=i;break;} }
  if(!saveBases(v, baseLast)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base/stations"); _server->send(303);
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

static void handleBasesSelect() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt();
  int baseLast=-1; std::vector<BaseRec> v; loadBases(v, baseLast);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  if(!saveBases(v, idx)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base/stations"); _server->send(303);
}

static void handleApiStartBaseMode() {
  int baseLast=-1; std::vector<BaseRec> v; loadBases(v, baseLast);
  if(baseLast<0||baseLast>=(int)v.size()){
    _server->send(400,"text/plain","No base station selected. Go to Base Stations and select one."); return;
  }
  const auto& b=v[baseLast];
  std::vector<AntennaRec> antennas; loadAntennas(antennas);
  float antennaOffset=0.0f;
  if(b.antennaIdx>=0&&b.antennaIdx<(int)antennas.size()) antennaOffset=antennas[b.antennaIdx].offset;
  double h=b.altGround+b.hARP+antennaOffset;
  applyBaseFixedLLH(b.lat, b.lon, h, b.stid, b.rtcmType);
  _server->send(200,"text/plain","OK");
}

static void handleApiStartAllOutputs() {
  BaseAutoStart as=loadBaseAutoStart();
  String result="";
  if(as.ntrip){
    std::vector<NtripOut> outList; int outLast=-1; loadNtripOutList(outList,outLast);
    if(outLast>=0&&outLast<(int)outList.size()){
      const auto& n=outList[outLast];
      bool okC=startCasterOut(n.host,(uint16_t)n.port,n.mount,n.pass);
      bool okT=startTcpOut((uint16_t)n.tcpPort);
      result+="NTRIP OUT: "+(String)(okC?"OK":"FAIL")+" / TCP: "+(okT?"OK":"FAIL")+". ";
    } else { result+="NTRIP OUT: no profile selected. "; }
  }
  if(as.tcp){
    std::vector<TcpOutClient> tcpList; int tcpLast=-1; loadTcpOutClientList(tcpList,tcpLast);
    if(tcpLast>=0&&tcpLast<(int)tcpList.size()){
      const auto& t=tcpList[tcpLast];
      bool ok=startTcpOutClient(t.host,(uint16_t)t.port);
      result+="TCP Client: "+(String)(ok?"OK":"FAIL")+". ";
    } else { result+="TCP Client: no profile selected. "; }
  }
  _server->send(200,"text/plain",result.length()?result:"No outputs configured for auto-start.");
}

static void handleApiSaveAutoStart() {
  BaseAutoStart s;
  s.ntrip = _server->hasArg("ntrip") && _server->arg("ntrip")=="1";
  s.tcp   = _server->hasArg("tcp")   && _server->arg("tcp")=="1";
  if(!saveBaseAutoStart(s)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base/outputs"); _server->send(303);
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
  _server->sendHeader("Location","/base/outputs"); _server->send(303);
}

static void handleBaseOutDel() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  v.erase(v.begin()+idx);
  if (last >= (int)v.size()) last = (int)v.size()-1;
  if(!saveNtripOutList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base/outputs"); _server->send(303);
}

static void handleBaseOutEdit() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  const auto& n=v[idx];
  
  sendHeader("Edit NTRIP OUT Profile", "base");
  sendChunk("<div class='card'>");
  sendChunk("<form method='POST' action='/baseout/update'>");
  sendChunk("<input type='hidden' name='idx' value='" + String(idx) + "'>");
  sendChunk("<label>Name:</label><input name='name' value='" + htmlEscape(n.name) + "' required><br>");
  sendChunk("<label>Host:</label><input name='host' value='" + htmlEscape(n.host) + "' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='" + String(n.port) + "' required><br>");
  sendChunk("<label>Mountpoint:</label><input name='mount' value='" + htmlEscape(n.mount) + "' required><br>");
  sendChunk("<label>Password:</label><input name='pass' type='password' value='" + htmlEscape(n.pass) + "'><br>");
  sendChunk("<label>TCP Server Port:</label><input name='tcp' type='number' value='" + String(n.tcpPort) + "' required><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/base/outputs'>Cancel</a>");
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
  _server->sendHeader("Location","/base/outputs"); _server->send(303);
}

static void handleBaseOutSelect() {
  if(!_server->hasArg("idx")){ _server->send(400,"text/plain","idx missing"); return; }
  int idx=_server->arg("idx").toInt(); std::vector<NtripOut> v; int last=-1; loadNtripOutList(v,last);
  if(idx<0||idx>=(int)v.size()){ _server->send(400,"text/plain","Index out of range"); return; }
  last=idx;
  if(!saveNtripOutList(v,last)){ _server->send(500,"text/plain","Save failed"); return; }
  _server->sendHeader("Location","/base/outputs"); _server->send(303);
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
  _server->sendHeader("Location","/base/outputs"); 
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
  _server->sendHeader("Location","/base/outputs"); 
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
  sendHeader("Edit TCP Client OUT Profile","base");
  sendChunk("<div class='card'><h2>Edit TCP Client OUT Profile</h2>");
  sendChunk("<form method='POST' action='/tcpclient/edit?idx="+String(idx)+"'>");
  sendChunk("<label>Name:</label><input name='name' value='"+htmlEscape(t.name)+"' required><br>");
  sendChunk("<label>Host:</label><input name='host' value='"+htmlEscape(t.host)+"' required><br>");
  sendChunk("<label>Port:</label><input name='port' type='number' value='"+String(t.port)+"' required><br>");
  sendChunk("<button type='submit'>Save</button> ");
  sendChunk("<a class='btn' href='/base/outputs'>Cancel</a>");
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
  _server->sendHeader("Location","/base/outputs"); 
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
  _server->sendHeader("Location","/base/outputs"); 
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
    _server->sendHeader("Location", "/settings/gnss");
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

static void handleApiGnssFiles() {
  SdLockGuard guard(5000);
  if (!guard.locked) {
    _server->send(503, "application/json", "{\"error\":\"SD busy\"}");
    return;
  }

  FsFile dir = _sd->open("/gnss");
  if (!dir || !dir.isDirectory()) {
    dir.close();
    _server->send(200, "application/json", "{\"files\":[]}");
    return;
  }

  struct UbxFile {
    String name;
    uint32_t size;
  };

  std::vector<UbxFile> files;
  FsFile file;
  while ((file = dir.openNextFile())) {
    char name[64];
    file.getName(name, sizeof(name));
    String filename = String(name);
    if (filename.endsWith(".ubx")) {
      UbxFile uf;
      uf.name = filename;
      uf.size = file.size();
      files.push_back(uf);
    }
    file.close();
  }
  dir.close();

  // Sort descending by name (newest first, filenames are timestamped)
  std::sort(files.begin(), files.end(), [](const UbxFile& a, const UbxFile& b) {
    return a.name > b.name;
  });

  String json = "{\"files\":[";
  for (size_t i = 0; i < files.size(); i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"" + jsonEscape(files[i].name) + "\",\"size\":" + String(files[i].size) + "}";
  }
  json += "]}";

  _server->send(200, "application/json", json);
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
    
    // Update buzzer directly from the already-in-memory content
    // (no need to re-read from SD — avoids an extra lock/read cycle)
    if (g_buzzer) {
      g_buzzer->setCustomMelody(g_audioUploadContent);
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
    sendChunk("<p><a href='/settings/system' class='btn'>Back to Settings</a></p>");
    sendChunk("</div>");
    sendFooter();
  } else {
    _server->sendHeader("Location", "/settings/system");
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
// NEW API HANDLERS - BASE STOP, ZED RESET
// ========================================================================

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

  // ---- Inline styles ----
  sendChunk("<style>");
  sendChunk(".modal-overlay{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.5);z-index:1000;justify-content:center;align-items:center;}");
  sendChunk(".modal-overlay.active{display:flex;}");
  sendChunk(".modal-box{background:white;border-radius:8px;padding:24px;max-width:420px;width:90%;box-shadow:0 4px 20px rgba(0,0,0,0.3);}");
  sendChunk(".modal-box h3{margin-top:0;color:#e67e22;}");
  sendChunk(".modal-box ul{margin:12px 0;padding-left:20px;color:#c0392b;}");
  sendChunk(".modal-actions{display:flex;gap:10px;margin-top:16px;justify-content:flex-end;}");
  sendChunk(".btn-force{background:#e67e22;color:white;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;}");
  sendChunk(".btn-cancel-modal{background:#95a5a6;color:white;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;}");
  sendChunk(".prog-bar{height:14px;background:#ecf0f1;border-radius:7px;margin:8px 0;overflow:hidden;}");
  sendChunk(".prog-fill{height:100%;background:#2ecc71;border-radius:7px;transition:width 0.3s,background 0.5s;}");
  sendChunk(".form-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px;}");
  sendChunk(".form-grid label{display:block;font-size:0.82em;color:#555;margin-bottom:2px;}");
  sendChunk(".form-grid input,.form-grid select{width:100%;box-sizing:border-box;}");
  sendChunk(".form-full{grid-column:1/-1;}");
  sendChunk(".sv-card{border:1px solid #dce1e7;border-radius:6px;padding:10px 12px;margin:6px 0;background:#fff;}");
  sendChunk(".sv-card-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:4px;}");
  sendChunk(".sv-card-title{font-weight:bold;font-size:1em;}");
  sendChunk(".rtk-badge{display:inline-block;padding:2px 7px;border-radius:10px;font-size:0.78em;font-weight:bold;color:#fff;}");
  sendChunk(".rtk-fix{background:#27ae60;}.rtk-float{background:#f39c12;}.rtk-none{background:#e74c3c;}");
  sendChunk(".sv-card-meta{font-size:0.83em;color:#555;margin:2px 0;}");
  sendChunk(".sv-dl-row{display:flex;gap:10px;margin-top:6px;}");
  sendChunk(".sv-dl-row a{flex:1;text-align:center;padding:10px 0;border-radius:5px;font-size:0.95em;font-weight:bold;text-decoration:none;background:#2980b9;color:#fff;}");
  sendChunk(".sv-dl-row a:last-child{background:#16a085;}");
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
  sendChunk("      if(bar){bar.style.width=d.pct+'%';");
  sendChunk("        var h=d.curHAcc;var bg=h<0.02?'#27ae60':h<0.05?'#f39c12':'#e74c3c';");
  sendChunk("        bar.style.background=bg;}");
  sendChunk("      if(st)st.innerHTML=d.pct+'% | '+d.nSamples+' samples | hAcc:'+d.curHAcc.toFixed(4)+'m | '+d.elapsed.toFixed(1)+'s';");
  sendChunk("      if(d.status==='done'||d.status==='error'){");
  sendChunk("        var res=document.getElementById('meas-result');");
  sendChunk("        if(res){res.style.display='';");
  sendChunk("          res.innerHTML=d.status==='done'?'<b style=color:green>&#10003; Saved: '+d.lastPointId+'</b>':'<span style=color:red>&#9888; Error: '+d.errorMsg+'</span>';}");
  sendChunk("        document.getElementById('btn-measure').disabled=false;");
  sendChunk("        if(d.status==='done'){autoIncrementName();addPointCard(d.lastPointId);");
  sendChunk("          setTimeout(function(){");
  sendChunk("            var r2=document.getElementById('meas-result');if(r2)r2.style.display='none';");
  sendChunk("            var b2=document.getElementById('meas-bar');if(b2){b2.style.width='0%';b2.style.background='#2ecc71';}");
  sendChunk("            var s2=document.getElementById('meas-status');if(s2)s2.innerHTML='';");
  sendChunk("          },3000);");
  sendChunk("        }");
  sendChunk("      } else if(Date.now()<_measureDeadline){");
  sendChunk("        _measureTimeout=setTimeout(poll,500);");
  sendChunk("      } else {");
  sendChunk("        var res=document.getElementById('meas-result');");
  sendChunk("        if(res){res.style.display='';res.innerHTML='<span style=color:red>Measure timeout (120s)</span>';}");
  sendChunk("        document.getElementById('btn-measure').disabled=false;");
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

  sendChunk("function autoIncrementName(){");
  sendChunk("  var n=document.getElementById('pt-name');if(!n)return;");
  sendChunk("  var m=n.value.match(/^(.*?)(\\d+)$/);");
  sendChunk("  if(m){var num=parseInt(m[2],10)+1;var pad=m[2].length;");
  sendChunk("    n.value=m[1]+(''+num).padStart(pad,'0');}");
  sendChunk("}");

  sendChunk("function getActiveSid(){var b=document.getElementById('active-banner');return b?b.getAttribute('data-sid'):'';}");

  // Render (or re-render) a card's static view from its data-* attributes
  sendChunk("function buildCardBody(card){");
  sendChunk("  card.innerHTML='';");
  sendChunk("  var pid=card.getAttribute('data-pid'),name=card.getAttribute('data-name')||'',");
  sendChunk("      codice=card.getAttribute('data-codice')||'',rtk=card.getAttribute('data-rtk')||'',");
  sendChunk("      lat=card.getAttribute('data-lat')||'0',lon=card.getAttribute('data-lon')||'0',");
  sendChunk("      alt=card.getAttribute('data-alt')||'0',hacc=card.getAttribute('data-hacc')||'0',");
  sendChunk("      nsamp=card.getAttribute('data-nsamp')||'0';");
  sendChunk("  var rl=rtk.toLowerCase(),isFix=rl.indexOf('fix')>=0,isFloat=rl.indexOf('float')>=0;");
  sendChunk("  var badgeClass=isFix?'rtk-fix':isFloat?'rtk-float':'rtk-none';");
  sendChunk("  var rtkLabel=isFix?'FIX \\u2713':isFloat?'FLOAT ~':'NO RTK';");
  sendChunk("  var hdr=document.createElement('div');hdr.className='sv-card-header';");
  sendChunk("  var title=document.createElement('span');title.className='sv-card-title';");
  sendChunk("  title.textContent=name+(codice?(' \\u2014 '+codice):'');");
  sendChunk("  var badge=document.createElement('span');badge.className='rtk-badge '+badgeClass;badge.textContent=rtkLabel;");
  sendChunk("  hdr.appendChild(title);hdr.appendChild(badge);");
  sendChunk("  var meta1=document.createElement('div');meta1.className='sv-card-meta';");
  sendChunk("  meta1.textContent='\\u{1F30D} '+lat+'\\u00B0  '+lon+'\\u00B0';");
  sendChunk("  var meta2=document.createElement('div');meta2.className='sv-card-meta';");
  sendChunk("  meta2.textContent='H: '+alt+' m | hAcc: '+hacc+' m | '+nsamp+' samples';");
  sendChunk("  var actions=document.createElement('div');actions.style.textAlign='right';actions.style.marginTop='4px';");
  sendChunk("  var editBtn=document.createElement('button');editBtn.className='btn btn-small';editBtn.innerHTML='&#9998;';");
  sendChunk("  editBtn.onclick=function(){startEditPoint(card);};");
  sendChunk("  var delBtn=document.createElement('button');delBtn.className='btn btn-small btn-danger';delBtn.innerHTML='&#128465;';");
  sendChunk("  delBtn.onclick=function(){delPoint(getActiveSid(),pid);};");
  sendChunk("  actions.appendChild(editBtn);actions.appendChild(delBtn);");
  sendChunk("  card.appendChild(hdr);card.appendChild(meta1);card.appendChild(meta2);card.appendChild(actions);");
  sendChunk("}");

  // Swap a card into inline name/code editing mode
  sendChunk("function startEditPoint(card){");
  sendChunk("  var name=card.getAttribute('data-name')||'',codice=card.getAttribute('data-codice')||'';");
  sendChunk("  card.innerHTML='';");
  sendChunk("  var row=document.createElement('div');row.style.display='flex';row.style.gap='6px';row.style.flexWrap='wrap';");
  sendChunk("  var inp=document.createElement('input');inp.type='text';inp.value=name;inp.style.flex='1';inp.style.minWidth='100px';");
  sendChunk("  var sel=document.createElement('select');sel.style.flex='1';sel.style.minWidth='140px';");
  sendChunk("  var optNone=document.createElement('option');optNone.value='';optNone.textContent='-- no code --';sel.appendChild(optNone);");
  sendChunk("  if(_codesData&&_codesData.categorie){_codesData.categorie.forEach(function(cat){");
  sendChunk("    var og=document.createElement('optgroup');og.label=cat.label;");
  sendChunk("    (cat.codici||[]).forEach(function(c){");
  sendChunk("      var o=document.createElement('option');o.value=c.cod;o.textContent=c.cod+' \\u2014 '+c.label;");
  sendChunk("      if(c.cod===codice)o.selected=true;og.appendChild(o);");
  sendChunk("    });sel.appendChild(og);");
  sendChunk("  });}");
  sendChunk("  row.appendChild(inp);row.appendChild(sel);");
  sendChunk("  var actions=document.createElement('div');actions.style.marginTop='8px';actions.style.textAlign='right';");
  sendChunk("  var saveBtn=document.createElement('button');saveBtn.className='btn btn-small btn-success';saveBtn.textContent='Save';");
  sendChunk("  var cancelBtn=document.createElement('button');cancelBtn.className='btn btn-small btn-secondary';cancelBtn.textContent='Cancel';");
  sendChunk("  saveBtn.onclick=function(){saveEditPoint(card,inp.value,sel.value);};");
  sendChunk("  cancelBtn.onclick=function(){buildCardBody(card);};");
  sendChunk("  actions.appendChild(saveBtn);actions.appendChild(cancelBtn);");
  sendChunk("  card.appendChild(row);card.appendChild(actions);");
  sendChunk("}");

  sendChunk("function saveEditPoint(card,name,codice){");
  sendChunk("  var sid=getActiveSid(),pid=card.getAttribute('data-pid');");
  sendChunk("  fetch('/api/pts/point/edit?sid='+sid+'&pid='+pid+'&name='+encodeURIComponent(name)+'&codice='+encodeURIComponent(codice),{method:'POST'})");
  sendChunk("    .then(function(r){return r.json();}).then(function(d){");
  sendChunk("      if(d.ok){card.setAttribute('data-name',name);card.setAttribute('data-codice',codice);buildCardBody(card);}");
  sendChunk("      else alert('Error: '+(d.error||'unknown'));");
  sendChunk("    }).catch(function(e){alert('Network error: '+e);});");
  sendChunk("}");

  // Prepend the just-saved point as a card, without reloading the page
  sendChunk("function addPointCard(pid){");
  sendChunk("  var sid=getActiveSid();if(!sid)return;");
  sendChunk("  fetch('/api/pts/points?sid='+sid).then(function(r){return r.json();}).then(function(geo){");
  sendChunk("    var feat=(geo.features||[]).filter(function(f){return f.id===pid;})[0];");
  sendChunk("    if(!feat)return;");
  sendChunk("    var list=document.getElementById('pts-list');if(!list)return;");
  sendChunk("    var ph=document.getElementById('pts-empty');if(ph)ph.remove();");
  sendChunk("    var p=feat.properties||{};var c=(feat.geometry&&feat.geometry.coordinates)||[0,0,0];");
  sendChunk("    var div=document.createElement('div');div.className='sv-card';");
  sendChunk("    div.setAttribute('data-pid',pid);");
  sendChunk("    div.setAttribute('data-name',p.name||'');");
  sendChunk("    div.setAttribute('data-codice',p.codice||'');");
  sendChunk("    div.setAttribute('data-rtk',(p.TPV&&p.TPV.rtk)||'');");
  sendChunk("    div.setAttribute('data-lon',c[0]);div.setAttribute('data-lat',c[1]);div.setAttribute('data-alt',c[2]||0);");
  sendChunk("    div.setAttribute('data-hacc',(p.HPPOSLLH&&p.HPPOSLLH.hAcc)||0);");
  sendChunk("    div.setAttribute('data-nsamp',(p.sampling&&p.sampling.n_samples)||0);");
  sendChunk("    buildCardBody(div);");
  sendChunk("    list.insertBefore(div,list.firstChild);");
  sendChunk("    var cnt=document.getElementById('active-pts-count');");
  sendChunk("    if(cnt){var n=parseInt(cnt.textContent,10);if(!isNaN(n))cnt.textContent=n+1;}");
  sendChunk("  }).catch(function(){});");
  sendChunk("}");

  sendChunk("function startMeasureRequest(force){");
  sendChunk("  var name=document.getElementById('pt-name').value;");
  sendChunk("  var codice=document.getElementById('pt-codice').value;");
  sendChunk("  var dur=document.getElementById('pt-dur').value||10;");
  sendChunk("  var interval=document.getElementById('pt-rate').value||0.5;");
  sendChunk("  document.getElementById('btn-measure').disabled=true;");
  sendChunk("  var st=document.getElementById('meas-status');if(st)st.innerHTML='Starting...';");
  sendChunk("  var res=document.getElementById('meas-result');if(res)res.style.display='none';");
  sendChunk("  var bar=document.getElementById('meas-bar');if(bar){bar.style.width='0%';bar.style.background='#2ecc71';}");
  sendChunk("  fetch('/api/pts/measure?name='+encodeURIComponent(name)+'&codice='+encodeURIComponent(codice)+'&duration='+dur+'&interval='+interval+'&force='+(force?1:0))");
  sendChunk("    .then(function(r){return r.json();}).then(function(d){");
  sendChunk("      if(d.error){");
  sendChunk("        var res=document.getElementById('meas-result');");
  sendChunk("        if(res){res.style.display='';res.innerHTML='<span style=color:red>&#9888; '+d.error+'</span>';}");
  sendChunk("        document.getElementById('btn-measure').disabled=false;");
  sendChunk("      } else { startMeasurePoll(); }");
  sendChunk("    }).catch(function(e){");
  sendChunk("      var res=document.getElementById('meas-result');");
  sendChunk("      if(res){res.style.display='';res.innerHTML='<span style=color:red>Network error: '+e+'</span>';}");
  sendChunk("      document.getElementById('btn-measure').disabled=false;");
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
  sendChunk("        var el=document.querySelector('[data-pid=\"'+pid+'\"]');");
  sendChunk("        if(el)el.remove();");
  sendChunk("        var cnt=document.getElementById('active-pts-count');");
  sendChunk("        if(cnt){var n=parseInt(cnt.textContent,10);if(!isNaN(n))cnt.textContent=n-1;}");
  sendChunk("      }else{alert('Error: '+(d.error||'unknown'));}");
  sendChunk("    })");
  sendChunk("    .catch(function(e){alert('Network error: '+e);});");
  sendChunk("}");

  sendChunk("function doSync(){fetch('/api/pts/sync').then(function(r){return r.text();}).then(function(t){alert(t);}).catch(function(e){alert('Sync error: '+e);});}");
  sendChunk("function toggleExtint(en){fetch('/api/pts/extint?enable='+(en?1:0)).then(function(r){return r.json();}).then(function(d){var c=document.getElementById('chk-extint');if(c)c.checked=d.extintMarker;}).catch(function(e){console.error('EXTINT toggle error:',e);});}");
  sendChunk("</script>");

  // ---- Active survey banner ----
  String activeSid = SurveyPoints::getActiveSurveyId();
  sendChunk("<div class='card'><h2>&#127919; Survey</h2>");
  if (activeSid.isEmpty()) {
    sendChunk("<div style='background:#fadbd8;color:#922b21;padding:12px;border-radius:4px;margin-bottom:12px;'>");
    sendChunk("&#9888; No active survey. Create a new survey or select one from the list below.</div>");
  } else {
    String json  = SurveyPoints::loadSurveyJSON(activeSid);
    String title = activeSid;
    int ti = json.indexOf("\"title\":\"");
    if (ti >= 0) { int ts=ti+9, te=json.indexOf("\"",ts); if(te>ts) title=json.substring(ts,te); }
    int pts = SurveyPoints::getSurveyPointCount(activeSid);
    String extintChecked = g_extintMarkerEnabled ? " checked" : "";
    sendChunk("<div id='active-banner' data-sid='" + activeSid + "' style='background:#2ecc71;color:white;padding:10px;border-radius:4px;margin-bottom:12px'>");
    sendChunk("<b>&#128205; " + title + "</b> &nbsp;|&nbsp; ");
    sendChunk("<span id='active-pts-count'>" + String(pts) + "</span> pts &nbsp;|&nbsp; ");
    sendChunk("&#128225; EXTINT: <label style='cursor:pointer;display:inline-flex;align-items:center;gap:4px;'>");
    sendChunk("<input type='checkbox' id='chk-extint'" + extintChecked + " onchange='toggleExtint(this.checked)'> PPK marker</label>");
    sendChunk("</div>");
    // ---- Compact 2-column measure form ----
    sendChunk("<h3 style='margin:0 0 8px'>&#128208; New Measurement</h3>");
    sendChunk("<div class='form-grid'>");
    sendChunk("<div><label>Point name</label><input id='pt-name' type='text' placeholder='P001' autocomplete='off'></div>");
    sendChunk("<div><label>Category</label><select id='pt-cat' onchange='updateCodici(this.value)'><option value=''>-- choose --</option></select></div>");
    sendChunk("<div class='form-full'><label>Code</label><select id='pt-codice'><option value=''>-- choose --</option></select></div>");
    sendChunk("<div><label>Duration (s)</label><input id='pt-dur' type='number' value='10' min='1' max='120'></div>");
    sendChunk("<div><label>Interval (s)</label><input id='pt-rate' type='number' value='0.5' min='0.1' max='10' step='0.1'></div>");
    sendChunk("</div>");
    sendChunk("<button id='btn-measure' onclick='doMeasure()' style='width:100%;padding:12px;font-size:1.1em;background:#27ae60;color:white;border:none;border-radius:6px;cursor:pointer;margin-top:4px'>&#128994; MEASURE</button>");
    sendChunk("<div class='prog-bar' style='margin-top:10px'><div id='meas-bar' class='prog-fill' style='width:0%'></div></div>");
    sendChunk("<div id='meas-status' style='margin:4px 0;font-family:monospace;font-size:0.85em;color:#555'></div>");
    sendChunk("<div id='meas-result' style='display:none;margin:8px 0;padding:8px;background:#ecf0f1;border-radius:4px'></div>");
  }
  sendChunk("</div>");

  // ---- Survey list ----
  sendChunk("<div class='card'><h2>&#128193; Survey List</h2>");
  sendChunk("<form method='POST' action='/api/pts/create' style='display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px'>");
  sendChunk("<input name='title' type='text' placeholder='Survey name' required style='flex:1;min-width:160px'>");
  sendChunk("<input name='desc' type='text' placeholder='Description (optional)' style='flex:2;min-width:160px'>");
  sendChunk("<button type='submit' style='white-space:nowrap'>&#10133; Create</button></form>");

  std::vector<String> ids;
  SurveyPoints::listSurveyIds(ids);
  if (ids.empty()) {
    sendChunk("<p style='color:#888'>No surveys found.</p>");
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

      sendChunk("<div data-sid='" + sid + "' style='border:1px solid " + String(isActive?"#27ae60":"#ddd") + ";border-radius:4px;padding:10px;margin:6px 0;background:" + String(isActive?"#eafaf1":"white") + "'>");
      sendChunk("<b>" + String(isActive ? "&#10003; " : "") + title + "</b> &nbsp;");
      sendChunk("<span style='color:#7f8c8d;font-size:0.85em'>&#128197; " + created + " | " + String(pts) + " pt | <code>" + sid + "</code></span><br style='margin-bottom:4px'>");
      if (!isActive) {
        sendChunk("<button class='btn-small' onclick=\"setActive('" + sid + "')\">&#128257; Set active</button> ");
      }
      sendChunk("<button class='btn btn-small btn-danger' onclick=\"delSurvey('" + sid + "')\">&#128465; Delete</button>");
      sendChunk("</div>");
    }
  }
  sendChunk("<button class='btn-secondary' style='margin-top:6px' onclick='doSync()'>Sync &#8594; SD</button>");
  sendChunk("</div>");

  // ---- Card-based point list for active survey ----
  if (!activeSid.isEmpty()) {
    sendChunk("<div class='card'><h2>&#128203; Recorded points</h2>");
    sendChunk("<div id='pts-list'>");

    int ptCount = SurveyPoints::getSurveyPointCount(activeSid);
    if (ptCount == 0) {
      sendChunk("<p id='pts-empty' style='color:#888;font-size:0.9em'>No points recorded yet.</p>");
    } else {
      String json = SurveyPoints::loadSurveyJSON(activeSid);

      struct PtRow { String pid, name, codice, rtk, lat, lon, alt, hacc, nsamp; };
      std::vector<PtRow> rows;

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

        PtRow row;
        row.pid    = getStr("id",    featureStart);
        row.name   = getStr("name",  featureStart);
        row.codice = getStr("codice",featureStart);
        row.rtk    = getStr("rtk",   featureStart);
        int propsPos  = json.indexOf("\"properties\":", featureStart);
        int from      = propsPos>0 ? propsPos : featureStart;

        int geomPos   = json.indexOf("\"coordinates\":", featureStart);
        row.lat="0"; row.lon="0"; row.alt="0";
        if (geomPos>0) {
          int bp=json.indexOf("[",geomPos);
          if(bp>0){int eb=json.indexOf("]",bp); String coords=json.substring(bp+1,eb);
            int c1=coords.indexOf(","); int c2=coords.indexOf(",",c1+1);
            row.lon=coords.substring(0,c1); row.lat=coords.substring(c1+1,c2>0?c2:coords.length());
            if(c2>0) row.alt=coords.substring(c2+1); row.lat.trim(); row.lon.trim(); row.alt.trim();}
        }

        row.hacc  = getNum("hAcc", from);
        row.nsamp = getNum("n_samples", from);

        rows.push_back(row);
        pos = featureEnd + 1;
      }

      // Newest point first (features are appended in chronological order)
      std::reverse(rows.begin(), rows.end());

      const int PAGE_SIZE = 20;
      int total      = (int)rows.size();
      int totalPages = (total + PAGE_SIZE - 1) / PAGE_SIZE;
      int ppage = _server->hasArg("ppage") ? _server->arg("ppage").toInt() : 0;
      if (ppage < 0) ppage = 0;
      if (ppage > totalPages - 1) ppage = totalPages - 1;
      int startIdx = ppage * PAGE_SIZE;
      int endIdx   = std::min(startIdx + PAGE_SIZE, total);

      for (int i = startIdx; i < endIdx; i++) {
        const PtRow& r = rows[i];

        // RTK badge: compute class and label in a single pass
        String rtkLower = r.rtk; rtkLower.toLowerCase();
        bool isFix   = rtkLower.indexOf("fix")   >= 0;
        bool isFloat = rtkLower.indexOf("float") >= 0;
        String badgeClass = isFix ? "rtk-fix" : isFloat ? "rtk-float" : "rtk-none";
        String rtkLabel   = isFix ? "FIX &#10003;" : isFloat ? "FLOAT ~" : "NO RTK";

        String nameEsc   = htmlEscape(r.name);
        String codiceEsc = htmlEscape(r.codice);

        sendChunk("<div class='sv-card' data-pid='" + r.pid + "' data-name='" + nameEsc + "' data-codice='" + codiceEsc +
                  "' data-rtk='" + r.rtk + "' data-lat='" + r.lat + "' data-lon='" + r.lon + "' data-alt='" + r.alt +
                  "' data-hacc='" + r.hacc + "' data-nsamp='" + r.nsamp + "'>");
        sendChunk("<div class='sv-card-header'>");
        sendChunk("<span class='sv-card-title'>" + nameEsc + (codiceEsc.isEmpty() ? "" : " &mdash; " + codiceEsc) + "</span>");
        sendChunk("<span class='rtk-badge " + badgeClass + "'>" + rtkLabel + "</span>");
        sendChunk("</div>");
        sendChunk("<div class='sv-card-meta'>&#127757; " + r.lat + "&#176; &nbsp; " + r.lon + "&#176;</div>");
        sendChunk("<div class='sv-card-meta'>H: " + r.alt + " m &nbsp;|&nbsp; hAcc: " + r.hacc + " m &nbsp;|&nbsp; " + r.nsamp + " samples</div>");
        sendChunk("<div style='text-align:right;margin-top:4px'>");
        sendChunk("<button class='btn btn-small' onclick=\"startEditPoint(this.closest('.sv-card'))\">&#9998;</button> ");
        sendChunk("<button class='btn btn-small btn-danger' onclick=\"delPoint('" + activeSid + "','" + r.pid + "')\">&#128465;</button>");
        sendChunk("</div></div>");
      }

      if (totalPages > 1) {
        sendChunk("<div style='display:flex;justify-content:space-between;align-items:center;margin-top:10px;font-size:0.9em;color:#555'>");
        if (ppage > 0) sendChunk("<a class='btn btn-small' href='/survey?ppage=" + String(ppage-1) + "'>&#8592; Prev</a>");
        else sendChunk("<span></span>");
        sendChunk("<span>Page " + String(ppage+1) + " / " + String(totalPages) + " (" + String(total) + " pts)</span>");
        if (ppage < totalPages-1) sendChunk("<a class='btn btn-small' href='/survey?ppage=" + String(ppage+1) + "'>Next &#8594;</a>");
        else sendChunk("<span></span>");
        sendChunk("</div>");
      }
    }
    sendChunk("</div>"); // end pts-list

    // ---- Download buttons ----
    sendChunk("<div class='sv-dl-row' style='margin-top:10px'>");
    sendChunk("<a href='/api/pts/download?sid=" + activeSid + "' download='" + activeSid + ".geojson'>&#11015; GeoJSON</a>");
    sendChunk("<a href='/api/pts/download/csv?sid=" + activeSid + "' download='" + activeSid + ".csv'>&#11015; CSV</a>");
    sendChunk("</div>");
    sendChunk("</div>");
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

// ---- Survey API: edit point (name/codice only) ----
static void handlePtsPointEdit() {
  String sid    = _server->hasArg("sid")    ? _server->arg("sid")    : "";
  String pid    = _server->hasArg("pid")    ? _server->arg("pid")    : "";
  String name   = _server->hasArg("name")   ? _server->arg("name")   : "";
  String codice = _server->hasArg("codice") ? _server->arg("codice") : "";
  if (sid.isEmpty() || pid.isEmpty()) { _server->send(400,"application/json","{\"error\":\"Missing sid/pid\"}"); return; }
  bool ok = SurveyPoints::editPoint(sid, pid, name, codice);
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

// ---- Survey API: toggle EXTINT marker (PPK event marking) ----
static void handlePtsExtint() {
  if (_server->hasArg("enable")) {
    g_extintMarkerEnabled = (_server->arg("enable") == "1");
  }
  String json = "{\"extintMarker\":";
  json += g_extintMarkerEnabled ? "true" : "false";
  json += "}";
  _server->send(200, "application/json", json);
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
  sendChunk(".sk-canvas-wrap{text-align:center;margin:10px 0;}");
  sendChunk("#skCanvas{display:block;margin:0 auto;background:#1a252f;border-radius:50%;border:2px solid #34495e;}");
  sendChunk(".sk-imu-btn{display:none;margin:8px auto;padding:10px 20px;background:#2980b9;color:#fff;");
  sendChunk("border:none;border-radius:6px;font-size:15px;cursor:pointer;width:100%;max-width:280px;}");
  sendChunk(".sk-info-row{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;margin-top:10px;font-family:monospace;font-size:13px;}");
  sendChunk(".sk-info-item{background:#34495e;color:#ecf0f1;padding:4px 10px;border-radius:4px;}");
  sendChunk("</style>");

  sendChunk("<script>");
  // IMU state variables
  sendChunk("var _devHdg=null,_imuOk=false,_skData=null;");
  // requestIMU: auto on Android, prompt on iOS
  sendChunk("function requestIMU(){");
  sendChunk("  if(typeof DeviceOrientationEvent==='undefined')return;");
  sendChunk("  if(typeof DeviceOrientationEvent.requestPermission==='function'){");
  sendChunk("    var btn=document.getElementById('skImuBtn');if(btn)btn.style.display='block';");
  sendChunk("  } else {");
  sendChunk("    window.addEventListener('deviceorientation',onSkOri,true);");
  sendChunk("    setTimeout(function(){_imuOk=(_devHdg!==null);},2000);");
  sendChunk("  }");
  sendChunk("}");
  // iOS permission button handler
  sendChunk("function askImuPerm(){");
  sendChunk("  DeviceOrientationEvent.requestPermission().then(function(r){");
  sendChunk("    if(r==='granted'){");
  sendChunk("      window.addEventListener('deviceorientation',onSkOri,true);");
  sendChunk("      _imuOk=true;");
  sendChunk("    }");
  sendChunk("    document.getElementById('skImuBtn').style.display='none';");
  sendChunk("  }).catch(function(){});");
  sendChunk("}");
  // orientation event handler
  sendChunk("function onSkOri(e){");
  sendChunk("  if(e.webkitCompassHeading!=null){");
  sendChunk("    _devHdg=e.webkitCompassHeading;");
  sendChunk("  } else if(e.alpha!=null){");
  sendChunk("    _devHdg=(360-e.alpha)%360;");
  sendChunk("  }");
  sendChunk("  _imuOk=true;");
  sendChunk("}");
  // drawCompass: draws compass rose and heading-relative arrow on canvas
  sendChunk("function drawCompass(az,d2d,dH){");
  sendChunk("  var c=document.getElementById('skCanvas');if(!c)return;");
  sendChunk("  var ctx=c.getContext('2d');");
  sendChunk("  var w=c.width,h=c.height,cx=w/2,cy=h/2;");
  sendChunk("  var r=Math.min(cx,cy)-16;");
  sendChunk("  ctx.clearRect(0,0,w,h);");
  // outer ring
  sendChunk("  ctx.beginPath();ctx.arc(cx,cy,r,0,2*Math.PI);");
  sendChunk("  ctx.strokeStyle='#5d6d7e';ctx.lineWidth=3;ctx.stroke();");
  // tick marks
  sendChunk("  for(var t=0;t<360;t+=30){");
  sendChunk("    var rot=(_imuOk&&_devHdg!=null)?(_devHdg*Math.PI/180):0;");
  sendChunk("    var ta=(t*Math.PI/180)-rot;");
  sendChunk("    var tlen=(t%90===0)?12:6;");
  sendChunk("    ctx.beginPath();");
  sendChunk("    ctx.moveTo(cx+(r-tlen)*Math.sin(ta),cy-(r-tlen)*Math.cos(ta));");
  sendChunk("    ctx.lineTo(cx+r*Math.sin(ta),cy-r*Math.cos(ta));");
  sendChunk("    ctx.strokeStyle='#7f8c8d';ctx.lineWidth=(t%90===0)?2:1;ctx.stroke();");
  sendChunk("  }");
  // cardinal labels (rotate with heading so they stay geo-fixed)
  sendChunk("  var cards=[['N',0,'#e74c3c'],['E',90,'#bdc3c7'],['S',180,'#bdc3c7'],['O',270,'#bdc3c7']];");
  sendChunk("  ctx.font='bold 14px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';");
  sendChunk("  var hdgRot=(_imuOk&&_devHdg!=null)?(_devHdg*Math.PI/180):0;");
  sendChunk("  cards.forEach(function(cv){");
  sendChunk("    var ca=(cv[1]*Math.PI/180)-hdgRot;");
  sendChunk("    var lx=cx+(r-14)*Math.sin(ca),ly=cy-(r-14)*Math.cos(ca);");
  sendChunk("    ctx.fillStyle=cv[2];ctx.fillText(cv[0],lx,ly);");
  sendChunk("  });");
  // arrow pointing to target
  sendChunk("  var angle;");
  sendChunk("  if(_imuOk&&_devHdg!=null){");
  sendChunk("    angle=((az-_devHdg+360)%360)*Math.PI/180;");
  sendChunk("  } else {");
  sendChunk("    angle=az*Math.PI/180;");
  sendChunk("  }");
  sendChunk("  var ar=r-34;");
  sendChunk("  ctx.save();ctx.translate(cx,cy);ctx.rotate(angle);");
  sendChunk("  ctx.beginPath();");
  // Arrow triangle: tip at top, base at ~42% radius, waist at ~18% (width ±11px)
  sendChunk("  ctx.moveTo(0,-ar);ctx.lineTo(11,ar*0.42);ctx.lineTo(0,ar*0.18);ctx.lineTo(-11,ar*0.42);");
  sendChunk("  ctx.closePath();ctx.fillStyle='#e74c3c';ctx.fill();");
  sendChunk("  ctx.restore();");
  // center dot
  sendChunk("  ctx.beginPath();ctx.arc(cx,cy,5,0,2*Math.PI);ctx.fillStyle='#ecf0f1';ctx.fill();");
  // distance label
  sendChunk("  ctx.font='bold 15px monospace';ctx.textAlign='center';ctx.textBaseline='middle';");
  sendChunk("  ctx.fillStyle='#ecf0f1';ctx.fillText(d2d.toFixed(3)+' m',cx,cy+r*0.52);");
  // dH label
  sendChunk("  var dhs=(dH===null||dH===undefined)?'dH: N/A':'dH: '+(dH>=0?'+':'')+dH.toFixed(2)+'m';");
  sendChunk("  ctx.font='12px monospace';ctx.fillStyle='#95a5a6';ctx.fillText(dhs,cx,cy+r*0.68);");
  // fallback label when no IMU
  sendChunk("  if(!_imuOk||_devHdg===null){");
  sendChunk("    ctx.font='10px sans-serif';ctx.fillStyle='#7f8c8d';");
  sendChunk("    ctx.fillText('North-up (compass N/A)',cx,cy+r*0.84);");
  sendChunk("  }");
  sendChunk("}");
  // RAF animation loop - smooth redraw with latest IMU heading
  sendChunk("function skFrame(){");
  sendChunk("  if(_skData)drawCompass(_skData.az,_skData.d2d,_skData.dH);");
  sendChunk("  requestAnimationFrame(skFrame);");
  sendChunk("}");
  // update info row labels
  sendChunk("function skUpdateInfo(az,cs){");
  sendChunk("  var fc={'2':'fix-fixed','1':'fix-float','0':'fix-none'};");
  sendChunk("  var fl={'2':'RTK FIX','1':'FLOAT','0':'NO FIX'};");
  sendChunk("  var bi=document.getElementById('sk-bearing');");
  sendChunk("  if(bi)bi.textContent='Bearing: '+az.toFixed(1)+'\\u00b0';");
  sendChunk("  var hi=document.getElementById('sk-heading');");
  sendChunk("  if(hi)hi.textContent='Heading: '+(_devHdg!=null?_devHdg.toFixed(1)+'\\u00b0':'--');");
  sendChunk("  var fi=document.getElementById('sk-fixbadge');");
  sendChunk("  if(fi){fi.textContent=fl[cs]||'NO FIX';fi.className='sk-badge-fix '+(fc[cs]||'fix-none');}");
  sendChunk("}");
  // Poll /api/stakeout/status every 2 seconds
  sendChunk("var _skTimer=null;");
  sendChunk("function skStart(){");
  sendChunk("  requestIMU();");
  sendChunk("  requestAnimationFrame(skFrame);");
  sendChunk("  _skTimer=setInterval(skPoll,2000);skPoll();");
  sendChunk("}");
  sendChunk("function skPoll(){");
  sendChunk("  fetch('/api/stakeout/status').then(function(r){return r.json();}).then(function(d){");
  sendChunk("    var el=document.getElementById('sk-status');");
  sendChunk("    if(!el)return;");
  sendChunk("    if(!d.valid){el.innerHTML='<p>No active target or no GNSS fix.</p>';_skData=null;return;}");
  sendChunk("    var fc={'2':'fix-fixed','1':'fix-float','0':'fix-none'};");
  sendChunk("    var fl={'2':'RTK FIX','1':'FLOAT','0':'NO FIX'};");
  sendChunk("    var cs=String(d.roverCarrSoln||0);");
  sendChunk("    el.innerHTML='<p><b>Target:</b> '+d.targetName+' ('+d.targetId+')'");
  sendChunk("      +'<span class=\"sk-badge-fix '+(fc[cs]||'fix-none')+'\">'+(fl[cs]||'NO FIX')+'</span></p>'");
  sendChunk("      +'<p>&#x1F4CF; D2D: <b>'+d.d2d.toFixed(3)+' m</b></p>'");
  sendChunk("      +'<p>&#x2B06; dH: <b>'+(d.dH===null?'N/A':((d.dH>=0?'+':'')+d.dH.toFixed(3)+' m'))+'</b></p>'");
  sendChunk("      +'<p>&#x1F4CF; Quota HAE: <b>'+(d.targetH===null?'N/A':d.targetH.toFixed(3)+' m')+'</b></p>'");
  sendChunk("      +'<p>&#x1F9ED; Az: <b>'+d.az.toFixed(1)+'&deg;</b></p>';");
  sendChunk("    _skData={az:d.az,d2d:d.d2d,dH:d.dH};");
  sendChunk("    skUpdateInfo(d.az,cs);");
  // Arrival threshold: 5cm D2D; vibration: 200ms on, 100ms off, 200ms on
  sendChunk("    if(d.d2d<0.05&&navigator.vibrate)navigator.vibrate([200,100,200]);");
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
  // Canvas compass
  sendChunk("<div class='sk-canvas-wrap'>");
  sendChunk("<canvas id='skCanvas' width='220' height='220'></canvas>");
  sendChunk("</div>");
  // iOS IMU permission button (hidden by default, shown by JS on iOS)
  sendChunk("<div style='text-align:center'>");
  sendChunk("<button id='skImuBtn' class='sk-imu-btn' onclick='askImuPerm()'>&#x1F9ED; Enable compass</button>");
  sendChunk("</div>");
  // Info row: bearing, heading, fix
  sendChunk("<div class='sk-info-row'>");
  sendChunk("<span class='sk-info-item' id='sk-bearing'>Bearing: --</span>");
  sendChunk("<span class='sk-info-item' id='sk-heading'>Heading: --</span>");
  sendChunk("<span id='sk-fixbadge' class='sk-badge-fix fix-none'>NO FIX</span>");
  sendChunk("</div>");
  // Text status (target, D2D, dH, HAE, Az)
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
// TRACKING PAGE AND API HANDLERS
// ========================================================================

static void handleTrackingPage() {
  sendHeader("Tracking", "tracking");

  sendChunk("<style>");
  sendChunk(".tk-status{background:#2c3e50;color:#ecf0f1;padding:12px 16px;border-radius:6px;font-family:monospace;margin-bottom:16px;}");
  sendChunk(".tk-status p{margin:4px 0;font-size:15px;}");
  sendChunk(".tk-rec-badge{padding:2px 8px;border-radius:10px;font-size:11px;margin-left:6px;}");
  sendChunk(".tk-rec-on{background:#e74c3c;color:#fff;}.tk-rec-off{background:#7f8c8d;color:#fff;}");
  sendChunk(".tk-dl-row{display:flex;gap:10px;margin-top:6px;}");
  sendChunk(".tk-dl-row a{flex:1;text-align:center;padding:8px 0;border-radius:5px;font-size:0.9em;font-weight:bold;text-decoration:none;background:#2980b9;color:#fff;}");
  sendChunk(".tk-dl-row a:last-child{background:#16a085;}");
  sendChunk("</style>");

  // ---- Status card ----
  sendChunk("<div class='card'><h2>&#128663; Tracking</h2>");
  sendChunk("<div id='tk-status' class='tk-status'><p>Loading...</p></div>");
  sendChunk("<input id='tk-name' type='text' placeholder='Track name (optional)' style='width:100%;margin-bottom:8px'>");
  sendChunk("<button id='tk-btn' class='btn-success' style='width:100%;padding:12px' data-rec='0' onclick='tkToggle()'>Start</button>");
  sendChunk("</div>");

  // ---- Settings card ----
  sendChunk("<div class='card'><h2>&#9881; Settings</h2>");
  sendChunk("<div style='display:flex;gap:16px;margin-bottom:10px'>");
  sendChunk("<label><input type='radio' name='tkMode' value='0' onchange='tkModeChanged()'> Time</label>");
  sendChunk("<label><input type='radio' name='tkMode' value='1' onchange='tkModeChanged()'> Distance</label>");
  sendChunk("</div>");
  sendChunk("<label id='tk-thlabel'>Interval (s)</label>");
  sendChunk("<input id='tk-threshold' type='number' step='0.5' min='0.5' style='width:100%;margin-bottom:8px'>");
  sendChunk("<button class='btn' onclick='tkSaveConfig()'>Save settings</button>");
  sendChunk("</div>");

  // ---- Track list ----
  sendChunk("<div class='card'><h2>&#128193; Recorded tracks</h2>");
  sendChunk("<div id='tk-list'>Loading...</div>");
  sendChunk("</div>");

  sendChunk("<script>");
  sendChunk("function escHtml(s){var d=document.createElement('div');d.textContent=s;return d.innerHTML;}");
  sendChunk("function tkModeChanged(){");
  sendChunk("  var mode=document.querySelector('input[name=tkMode]:checked').value;");
  sendChunk("  document.getElementById('tk-thlabel').textContent=(mode=='0')?'Interval (s)':'Distance (m)';");
  sendChunk("}");
  sendChunk("function tkToggle(){");
  sendChunk("  var recording=document.getElementById('tk-btn').getAttribute('data-rec')==='1';");
  sendChunk("  if(recording){");
  sendChunk("    fetch('/api/track/stop',{method:'POST'}).then(function(r){return r.json();}).then(function(d){");
  sendChunk("      if(!d.ok)alert('Error: '+(d.error||'unknown'));");
  sendChunk("      tkPoll();tkRefreshList();");
  sendChunk("    }).catch(function(e){alert('Network error: '+e);});");
  sendChunk("  } else {");
  sendChunk("    var name=encodeURIComponent(document.getElementById('tk-name').value);");
  sendChunk("    fetch('/api/track/start?name='+name,{method:'POST'}).then(function(r){return r.json();}).then(function(d){");
  sendChunk("      if(!d.ok)alert('Error: '+(d.error||'unknown'));");
  sendChunk("      tkPoll();tkRefreshList();");
  sendChunk("    }).catch(function(e){alert('Network error: '+e);});");
  sendChunk("  }");
  sendChunk("}");
  sendChunk("function tkSaveConfig(){");
  sendChunk("  var mode=document.querySelector('input[name=tkMode]:checked').value;");
  sendChunk("  var val=document.getElementById('tk-threshold').value;");
  sendChunk("  fetch('/api/track/config?mode='+mode+'&value='+val,{method:'POST'})");
  sendChunk("    .then(function(r){return r.json();}).then(function(d){");
  sendChunk("      if(!d.ok)alert('Error: '+(d.error||'unknown'));");
  sendChunk("    }).catch(function(e){alert('Network error: '+e);});");
  sendChunk("}");
  sendChunk("function tkPoll(){");
  sendChunk("  fetch('/api/track/status').then(function(r){return r.json();}).then(function(d){");
  sendChunk("    var el=document.getElementById('tk-status');");
  sendChunk("    var badge=d.recording?'<span class=\"tk-rec-badge tk-rec-on\">REC</span>':'<span class=\"tk-rec-badge tk-rec-off\">OFF</span>';");
  sendChunk("    var html='<p>Status: '+badge+'</p>';");
  sendChunk("    if(d.recording){");
  sendChunk("      html+='<p>Points: '+d.pointCount+' | Distance: '+d.distanceM.toFixed(1)+'m | Duration: '+d.durationSec+'s</p>';");
  sendChunk("      if(d.pointsDropped>0)html+='<p style=color:#e67e22>Dropped: '+d.pointsDropped+'</p>';");
  sendChunk("    }");
  sendChunk("    if(d.lastError)html+='<p style=color:#e74c3c>'+escHtml(d.lastError)+'</p>';");
  sendChunk("    el.innerHTML=html;");
  sendChunk("    var btn=document.getElementById('tk-btn');");
  sendChunk("    btn.textContent=d.recording?'Stop':'Start';");
  sendChunk("    btn.className=d.recording?'btn-danger':'btn-success';");
  sendChunk("    btn.setAttribute('data-rec',d.recording?'1':'0');");
  sendChunk("    if(!d.recording){");
  sendChunk("      var r=document.querySelector('input[name=tkMode][value=\"'+d.triggerMode+'\"]');");
  sendChunk("      if(r)r.checked=true;");
  sendChunk("      document.getElementById('tk-threshold').value=(d.triggerMode==0)?d.intervalSec:d.distanceThresholdM;");
  sendChunk("      tkModeChanged();");
  sendChunk("    }");
  sendChunk("  }).catch(function(){});");
  sendChunk("}");
  sendChunk("function tkDelete(id){");
  sendChunk("  if(!confirm('Delete track '+id+'?'))return;");
  sendChunk("  fetch('/api/track/delete?id='+id,{method:'POST'}).then(function(r){return r.json();})");
  sendChunk("    .then(function(d){if(d.ok)tkRefreshList();else alert('Error: '+(d.error||'unknown'));})");
  sendChunk("    .catch(function(e){alert('Network error: '+e);});");
  sendChunk("}");
  sendChunk("function tkRefreshList(){");
  sendChunk("  fetch('/api/track/list').then(function(r){return r.json();}).then(function(d){");
  sendChunk("    var list=document.getElementById('tk-list');");
  sendChunk("    var tracks=d.tracks||[];");
  sendChunk("    if(tracks.length===0){list.innerHTML='<p style=color:#888>No tracks recorded yet.</p>';return;}");
  sendChunk("    var html='';");
  sendChunk("    tracks.forEach(function(t){");
  sendChunk("      var nm=escHtml(t.name);");
  sendChunk("      html+='<div style=\"border:1px solid #ddd;border-radius:4px;padding:10px;margin:6px 0\">';");
  sendChunk("      html+='<b>'+nm+'</b> <span style=\"color:#7f8c8d;font-size:0.85em\">'+t.pointCount+' pts | '+t.distanceM.toFixed(0)+'m'+(t.endEpoch===0?' | recording':'')+'</span><br>';");
  sendChunk("      html+='<div class=\"tk-dl-row\">';");
  sendChunk("      html+='<a href=\"/api/track/download/gpx?id='+t.trackId+'\" download=\"'+t.trackId+'.gpx\">GPX</a>';");
  sendChunk("      html+='<a href=\"/api/track/download/kml?id='+t.trackId+'\" download=\"'+t.trackId+'.kml\">KML</a>';");
  sendChunk("      html+='</div>';");
  sendChunk("      html+='<button class=\"btn btn-small btn-danger\" style=\"margin-top:6px\" onclick=\"tkDelete(\\''+t.trackId+'\\')\">Delete</button>';");
  sendChunk("      html+='</div>';");
  sendChunk("    });");
  sendChunk("    list.innerHTML=html;");
  sendChunk("  }).catch(function(){});");
  sendChunk("}");
  sendChunk("tkPoll();tkRefreshList();");
  sendChunk("setInterval(tkPoll,3000);");
  sendChunk("</script>");

  sendFooter();
}

// ---- Tracking API: status ----
static void handleTrackStatusApi() {
  TrackStatus st = TrackRecorder::getStatus();
  String json = "{";
  json += "\"recording\":"          + String(st.recording ? "true" : "false") + ",";
  json += "\"trackId\":\""          + st.trackId + "\",";
  json += "\"pointCount\":"         + String(st.pointCount) + ",";
  json += "\"pointsDropped\":"      + String(st.pointsDropped) + ",";
  json += "\"distanceM\":"          + String(st.distanceM, 2) + ",";
  json += "\"durationSec\":"        + String(st.durationSec) + ",";
  json += "\"triggerMode\":"        + String((int)st.cfg.triggerMode) + ",";
  json += "\"intervalSec\":"        + String(st.cfg.intervalSec, 2) + ",";
  json += "\"distanceThresholdM\":" + String(st.cfg.distanceM, 2) + ",";
  json += "\"lastError\":\""        + st.lastError + "\"";
  json += "}";
  _server->send(200, "application/json", json);
}

// ---- Tracking API: start ----
static void handleTrackStart() {
  String name = _server->hasArg("name") ? _server->arg("name") : "";
  bool ok = TrackRecorder::start(name);
  if (ok) { _server->send(200, "application/json", "{\"ok\":true}"); return; }
  TrackStatus st = TrackRecorder::getStatus();
  _server->send(200, "application/json", "{\"error\":\"" + st.lastError + "\"}");
}

// ---- Tracking API: stop ----
static void handleTrackStop() {
  bool ok = TrackRecorder::stop();
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"Not recording\"}");
}

// ---- Tracking API: save config (trigger mode + threshold) ----
static void handleTrackConfigSave() {
  int   mode  = _server->hasArg("mode")  ? _server->arg("mode").toInt()   : 0;
  float value = _server->hasArg("value") ? _server->arg("value").toFloat() : 0;
  if (value <= 0) { _server->send(200, "application/json", "{\"error\":\"Invalid value\"}"); return; }
  TrackRecorder::setTriggerMode((TrackTriggerMode)mode);
  if (mode == TRACK_TRIGGER_TIME) TrackRecorder::setIntervalSec(value);
  else                            TrackRecorder::setDistanceM(value);
  _server->send(200, "application/json", "{\"ok\":true}");
}

// ---- Tracking API: list tracks ----
static void handleTrackList() {
  std::vector<TrackInfo> tracks;
  TrackRecorder::listTracks(tracks);
  String json = "{\"tracks\":[";
  for (size_t i = 0; i < tracks.size(); i++) {
    if (i > 0) json += ",";
    json += "{\"trackId\":\"" + tracks[i].trackId + "\",";
    json += "\"name\":\""     + tracks[i].name + "\",";
    json += "\"startEpoch\":" + String(tracks[i].startEpoch) + ",";
    json += "\"endEpoch\":"   + String(tracks[i].endEpoch) + ",";
    json += "\"pointCount\":" + String(tracks[i].pointCount) + ",";
    json += "\"distanceM\":"  + String(tracks[i].distanceM, 2);
    json += "}";
  }
  json += "]}";
  _server->send(200, "application/json", json);
}

// ---- Tracking API: delete track ----
static void handleTrackDelete() {
  String id = _server->hasArg("id") ? _server->arg("id") : "";
  if (id.isEmpty()) { _server->send(400, "application/json", "{\"error\":\"Missing id\"}"); return; }
  bool ok = TrackRecorder::deleteTrack(id);
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"Not found or still recording\"}");
}

// ---- Tracking API: streamed export (never materializes the whole track in RAM) ----
static void trackEmitToClient(const String& chunk, void* ctx) {
  (void)ctx;
  _server->sendContent(chunk);
}

static void handleTrackDownloadGPX() {
  String id = _server->hasArg("id") ? _server->arg("id") : "";
  if (id.isEmpty()) { _server->send(404, "text/plain", "No track"); return; }
  _server->sendHeader("Content-Disposition", "attachment; filename=\"" + id + ".gpx\"");
  _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
  _server->send(200, "application/gpx+xml", "");
  if (!TrackRecorder::exportGPX(id, trackEmitToClient, nullptr)) {
    _server->sendContent("");
  }
}

static void handleTrackDownloadKML() {
  String id = _server->hasArg("id") ? _server->arg("id") : "";
  if (id.isEmpty()) { _server->send(404, "text/plain", "No track"); return; }
  _server->sendHeader("Content-Disposition", "attachment; filename=\"" + id + ".kml\"");
  _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
  _server->send(200, "application/vnd.google-earth.kml+xml", "");
  if (!TrackRecorder::exportKML(id, trackEmitToClient, nullptr)) {
    _server->sendContent("");
  }
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
// ESP-NOW API HANDLERS
// ========================================================================

static void handleEspNowStatus() {
  String json = "{";
  json += "\"enabled\":" + String(g_espNowEnabled ? "true" : "false") + ",";
  if (g_espNowEnabled) {
    json += "\"role\":\"" + String(g_espNowTxEnabled ? "tx" : "rx") + "\",";
  } else {
    json += "\"role\":\"off\",";
  }
  uint8_t ch = g_espNowChannel ? g_espNowChannel : (uint8_t)ESPNOW_WIFI_CHANNEL;
  char chBuf[8]; snprintf(chBuf, sizeof(chBuf), "%d", ch);
  json += "\"channel\":" + String(chBuf) + ",";
  char nodeIdBuf[12]; snprintf(nodeIdBuf, sizeof(nodeIdBuf), "0x%04X", g_espNow.getNodeId());
  json += "\"node_id\":\"" + String(nodeIdBuf) + "\",";
  json += "\"network_id\":\"0x52544B4E\",";
  json += "\"rx_pkts\":" + String((unsigned long)g_espNow.getRxPkts()) + ",";
  json += "\"drop_dedup\":" + String((unsigned long)g_espNow.getDropDedup()) + ",";
  json += "\"drop_old\":" + String((unsigned long)g_espNow.getDropOld()) + ",";
  json += "\"drop_crc\":" + String((unsigned long)g_espNow.getDropCrc()) + ",";
  json += "\"drop_no_mem\":" + String((unsigned long)g_espNow.getDropNoMem()) + ",";
  json += "\"tx_pkts\":" + String((unsigned long)g_espNow.getTxPkts()) + ",";
  json += "\"rtcm_bytes_rx\":" + String((unsigned long)g_espNow.getRtcmBytesRx()) + ",";
  json += "\"last_rssi\":" + String((int)g_espNow.getLastRssi()) + ",";
  char relayBuf[12]; snprintf(relayBuf, sizeof(relayBuf), "0x%04X", g_espNowRelayNodeId);
  json += "\"relay_node_id\":\"" + String(relayBuf) + "\",";
  json += "\"relay_active\":" + String(g_espNowRelayNodeId != 0 ? "true" : "false") + ",";
  json += "\"psk_enabled\":" + String(g_espNow.isPskEnabled() ? "true" : "false") + ",";
  json += "\"drop_auth\":" + String((unsigned long)g_espNow.getDropAuth()) + ",";
  // Relay forwarding fields (RTKino is never a relay node itself; always false/0)
  json += "\"relay_forwarding\":false,";
  json += "\"relay_upstream_id\":0,";
  json += "\"relay_rover_id\":0,";
  json += "\"peers\":[";
  uint32_t nowMs = (uint32_t)millis();
  for (int i = 0; i < g_espNow.peerCount; i++) {
    const auto& p = g_espNow.peers[i];
    if (i > 0) json += ",";
    char pBuf[16]; snprintf(pBuf, sizeof(pBuf), "0x%04X", p.node_id);
    json += "{";
    json += "\"node_id\":\"" + String(pBuf) + "\",";
    json += "\"role\":" + String((int)p.role) + ",";
    json += "\"fix\":" + String((int)p.fix_quality) + ",";
    json += "\"carr_soln\":" + String((int)p.carr_soln) + ",";
    json += "\"lat_e7\":" + String((long)p.lat_e7) + ",";
    json += "\"lon_e7\":" + String((long)p.lon_e7) + ",";
    json += "\"alt_dm\":" + String((int)p.alt_dm) + ",";
    json += "\"h_acc_mm\":" + String((int)p.h_acc_mm) + ",";
    json += "\"rssi\":" + String((int)p.last_rssi) + ",";
    json += "\"pkt_loss_pct\":" + String((int)p.pkt_loss_pct) + ",";
    json += "\"rtcm_age_ms\":" + String((int)p.rtcm_age_ms) + ",";
    uint32_t age = (nowMs >= p.last_seen_ms) ? (nowMs - p.last_seen_ms) : 0;
    json += "\"age_ms\":" + String((unsigned long)age);
    json += "}";
  }
  json += "]}";
  _server->send(200, "application/json", json);
}

static void handleEspNowStartRx() {
  bool ok = startEspNowRx();
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"Init failed\"}");
}

static void handleEspNowStartTx() {
  bool ok = startEspNowTx();
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"Init failed\"}");
}

static void handleEspNowStop() {
  if (g_espNowTxEnabled) stopEspNowTx();
  else                   stopEspNowRx();
  _server->send(200, "application/json", "{\"ok\":true}");
}

static void handleEspNowSave() {
  String role = _server->arg("espnow_role");
  role.trim();
  if (role == "relay") {
    _server->send(400, "application/json", "{\"ok\":false,\"err\":\"Relay mode not supported on RTKino\"}");
    return;
  }
  String chStr = _server->arg("espnow_channel");
  chStr.trim();
  if (!chStr.isEmpty()) {
    int ch = chStr.toInt();
    if (ch >= 1 && ch <= 13) {
      FlashConfig::writeFile("/config/espnow_channel.txt", String(ch));
    }
  }
  bool ok;
  if (role == "tx") {
    ok = startEspNowTx();
  } else {
    ok = startEspNowRx();
  }
  _server->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"Init failed\"}");
}

static void handleEspNowCommand() {
  if (!g_espNowEnabled) {
    _server->send(400, "application/json", "{\"error\":\"ESP-NOW not active\"}");
    return;
  }
  String cmdStr  = _server->arg("cmd");
  String dstStr  = _server->arg("dst");
  String paramStr = _server->arg("param");

  uint8_t cmd   = (uint8_t)strtoul(cmdStr.c_str(), nullptr, 0);
  uint16_t dst  = (uint16_t)strtoul(dstStr.length() > 0 ? dstStr.c_str() : "0xFFFF", nullptr, 0);
  uint8_t param = (uint8_t)strtoul(paramStr.c_str(), nullptr, 0);

  bool sent = g_espNow.sendCommand(dst, cmd, param);
  char uidBuf[16];
  snprintf(uidBuf, sizeof(uidBuf), "0x%08lX", (unsigned long)millis());
  String resp = "{\"sent\":";
  resp += sent ? "true" : "false";
  resp += ",\"cmd_uid\":\"" + String(uidBuf) + "\"}";
  _server->send(200, "application/json", resp);
}

static void handleEspNowConfigGet() {
  String netIdStr = FlashConfig::readFile("/config/espnow_network_id.txt");
  netIdStr.trim();
  uint32_t netId = (uint32_t)ESPNOW_NETWORK_ID;
  if (netIdStr.length() > 0) {
    if (netIdStr.startsWith("0x") || netIdStr.startsWith("0X"))
      netId = (uint32_t)strtoul(netIdStr.c_str() + 2, nullptr, 16);
    else
      netId = (uint32_t)strtoul(netIdStr.c_str(), nullptr, 10);
  }
  String pskStr = FlashConfig::readFile("/config/espnow_psk.txt");
  bool pskSet = (pskStr.length() > 0);
  String json = "{\"ok\":true,\"network_id\":";
  json += String((unsigned long)netId);
  json += ",\"psk_set\":";
  json += pskSet ? "true" : "false";
  json += "}";
  _server->send(200, "application/json", json);
}

static void handleEspNowConfigSet() {
  String body = _server->arg("plain");
  // Parse JSON body: {"network_id":"0x...","psk":"..."}
  String netIdVal, pskVal;

  // Simple extraction without heavy JSON lib
  int ni = body.indexOf("\"network_id\"");
  if (ni >= 0) {
    int q1 = body.indexOf(':', ni);
    if (q1 >= 0) {
      int v1 = body.indexOf('"', q1 + 1);
      int v2 = (v1 >= 0) ? body.indexOf('"', v1 + 1) : -1;
      if (v1 >= 0 && v2 > v1) {
        netIdVal = body.substring(v1 + 1, v2);
      } else {
        // numeric value without quotes
        int vs = q1 + 1;
        while (vs < (int)body.length() && (body[vs] == ' ')) vs++;
        int ve = vs;
        while (ve < (int)body.length() && body[ve] != ',' && body[ve] != '}') ve++;
        netIdVal = body.substring(vs, ve);
        netIdVal.trim();
      }
    }
  }
  int pi = body.indexOf("\"psk\"");
  if (pi >= 0) {
    int q1 = body.indexOf(':', pi);
    if (q1 >= 0) {
      int v1 = body.indexOf('"', q1 + 1);
      int v2 = (v1 >= 0) ? body.indexOf('"', v1 + 1) : -1;
      if (v1 >= 0 && v2 >= v1) {
        pskVal = body.substring(v1 + 1, v2);
      }
    }
  }

  if (netIdVal.length() > 0) {
    netIdVal.trim();
    FlashConfig::writeFile("/config/espnow_network_id.txt", netIdVal);
  }
  FlashConfig::writeFile("/config/espnow_psk.txt", pskVal);

  // Apply immediately if running; startEspNow* will re-read config from flash
  if (g_espNowEnabled) {
    bool wasTx = g_espNowTxEnabled;
    if (wasTx) stopEspNowTx();
    else       stopEspNowRx();
    if (wasTx) startEspNowTx();
    else       startEspNowRx();
  } else {
    // Apply to in-memory object without starting (reads from flash we just wrote)
    applyEspNowConfigFromFlash();
  }

  _server->send(200, "application/json", "{\"ok\":true}");
}

void WebUI::begin(SdFat& sd, WebServer& server) {
  _sd = &sd; 
  _server = &server;

  // Main pages
  _server->on("/", handleRoot);
  _server->on("/rover", HTTP_GET, handleRoverPage);
  _server->on("/rover/connections", HTTP_GET, handleRoverConnectionsPage);
  _server->on("/base-cfg", HTTP_GET, handleBasePage);
  _server->on("/base/stations", HTTP_GET, handleBaseStationsPage);
  _server->on("/base/outputs", HTTP_GET, handleBaseOutputsPage);
  _server->on("/base/stop", HTTP_GET, handleBaseStop);
  _server->on("/settings", HTTP_GET, handleSettingsPage);
  _server->on("/settings/connectivity", HTTP_GET, handleSettingsConnectivity);
  _server->on("/settings/gnss",         HTTP_GET, handleSettingsGnss);
  _server->on("/settings/survey",       HTTP_GET, handleSettingsSurvey);
  _server->on("/settings/time",         HTTP_GET, handleSettingsTime);
  _server->on("/settings/system",       HTTP_GET, handleSettingsSystem);
  _server->on("/api/device/name",       HTTP_POST, handleDeviceNameSave);
  _server->on("/firmware", HTTP_GET, handleFirmwarePage);
  _server->on("/logs", HTTP_GET, handleLogsPage);
  _server->on("/logs/deleteall", HTTP_GET, handleDeleteAllLogs);
  _server->on("/survey", HTTP_GET, handleSurveyPage);
  _server->on("/stakeout", HTTP_GET, handleStakeoutPage);
  _server->on("/tracking", HTTP_GET, handleTrackingPage);
  _server->on("/api/track/status",        HTTP_GET,  handleTrackStatusApi);
  _server->on("/api/track/start",         HTTP_POST, handleTrackStart);
  _server->on("/api/track/stop",          HTTP_POST, handleTrackStop);
  _server->on("/api/track/config",        HTTP_POST, handleTrackConfigSave);
  _server->on("/api/track/list",          HTTP_GET,  handleTrackList);
  _server->on("/api/track/delete",        HTTP_POST, handleTrackDelete);
  _server->on("/api/track/download/gpx",  HTTP_GET,  handleTrackDownloadGPX);
  _server->on("/api/track/download/kml",  HTTP_GET,  handleTrackDownloadKML);

  // CSS and API
  _server->on("/css", HTTP_GET, handleCSS);
  _server->on("/api/status", HTTP_GET, handleApiStatus);
  _server->on("/api/position", HTTP_GET, handleApiPosition);
  _server->on("/api/rtcm", HTTP_GET, handleApiRtcm);
  _server->on("/api/rtcm/reset", HTTP_POST, handleApiRtcmReset);
  _server->on("/api/antennas", HTTP_GET, handleApiAntennas);
  _server->on("/api/bases", HTTP_GET, handleApiBasesIdx);
  _server->on("/api/zed/tmode", HTTP_GET, handleApiZedTmode);
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
  _server->on("/api/pts/point/edit",      HTTP_POST, handlePtsPointEdit);
  _server->on("/api/pts/download",        HTTP_GET,  handlePtsDownload);
  _server->on("/api/pts/download/csv",    HTTP_GET,  handlePtsDownloadCSV);
  _server->on("/api/pts/sync",            HTTP_GET,  handlePtsSync);
  _server->on("/api/pts/extint",          HTTP_GET,  handlePtsExtint);

  // GNSS log files API
  _server->on("/api/gnss/files", HTTP_GET, handleApiGnssFiles);

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
  _server->on("/wifi", HTTP_GET, [](){ _server->sendHeader("Location", "/settings/connectivity"); _server->send(303); });
  _server->on("/ntp", HTTP_GET, [](){ _server->sendHeader("Location", "/settings/time"); _server->send(303); });
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
  _server->on("/rate", HTTP_GET, [](){ _server->sendHeader("Location", "/settings/gnss"); _server->send(303); });

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

  // ESP-NOW Mesh API
  _server->on("/api/espnow/status",     HTTP_GET,  handleEspNowStatus);
  _server->on("/api/espnow/config/get", HTTP_GET,  handleEspNowConfigGet);
  _server->on("/api/espnow/config/set", HTTP_POST, handleEspNowConfigSet);
  _server->on("/espnow/startrx",        HTTP_POST, handleEspNowStartRx);
  _server->on("/espnow/starttx",        HTTP_POST, handleEspNowStartTx);
  _server->on("/espnow/stop",           HTTP_POST, handleEspNowStop);
  _server->on("/espnow/save",           HTTP_POST, handleEspNowSave);
  _server->on("/espnow/command",        HTTP_POST, handleEspNowCommand);

  // NTRIP IN CRUD
  _server->on("/ntrip/sourcetable", HTTP_GET, handleNtripSourcetable);
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
  _server->on("/bases/select", HTTP_GET, handleBasesSelect);
  _server->on("/bases/start", HTTP_GET, handleBasesStart);
  _server->on("/bases/start-confirm", HTTP_POST, handleBasesStartConfirm);
  _server->on("/api/startBaseMode", HTTP_GET, handleApiStartBaseMode);
  _server->on("/api/startAllOutputs", HTTP_GET, handleApiStartAllOutputs);
  _server->on("/api/base-autostart", HTTP_POST, handleApiSaveAutoStart);

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

  // AP credentials save
  _server->on("/api/ap/config", HTTP_POST, [](){
    if (!_server->hasArg("ap_ssid")) { _server->send(400, "text/plain", "Missing ap_ssid"); return; }
    String ssid = _server->arg("ap_ssid");
    String pass = _server->hasArg("ap_pass") ? _server->arg("ap_pass") : "";
    ssid.trim(); pass.trim();
    if (ssid.length() < 1 || ssid.length() > 32) {
      _server->send(400, "text/plain", "SSID non valido (1-32 caratteri)"); return;
    }
    if (pass.length() > 0 && pass.length() < 8) {
      _server->send(400, "text/plain", "Password troppo corta (min 8 caratteri o vuota)"); return;
    }
    if (pass.length() > 63) {
      _server->send(400, "text/plain", "Password troppo lunga (max 63 caratteri)"); return;
    }
    if (saveApConfig(ssid.c_str(), pass.c_str())) {
      _server->sendHeader("Location", "/settings/connectivity");
      _server->send(303);
    } else {
      _server->send(500, "text/plain", "Errore salvataggio credenziali AP");
    }
  });

  // Switch to AP mode at runtime (session-only, no persistence — reboot resets to normal WiFi)
  _server->on("/api/start-ap", HTTP_POST, [](){
    if (g_apMode) {
      _server->send(200, "text/plain", "Gia' in modalita' AP.");
      return;
    }
    _server->send(200, "text/plain",
      String("Passando in modalita' AP (") + g_apSsid + ", ch" + String(ESPNOW_WIFI_CHANNEL) + ").\n"
      "Connettiti a " + g_apSsid + " e vai su http://192.168.4.1\n"
      "Al prossimo riavvio RTKino tornera' alla modalita' WiFi normale.");
    delay(300);
    switchToApModeNow();
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
    
    _server->sendHeader("Location", "/settings/system");
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

