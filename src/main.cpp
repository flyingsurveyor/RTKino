// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
// Project: RTKino
// Author: FlyingSurveyor (+ tweaks)
// IMPORTANT: SD access MUST be guarded (RAII) to avoid system-wide deadlock

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Wire.h>
#include "SdFat.h"
#include "config.h"         // Board-specific hardware configuration
#include "OledLogger.h"
#include "NtripClient.h"
#include "WebUI.h"
#include "TcpStreamer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <vector>
#include "WifiProfiles.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "UbxValset.h"
#include "NtripPusher.h"   
#include "RtcmStreamer.h"  
#include "RTCMInput.h"
#include "TcpClientStreamer.h"
#include "Buzzer.h"
#include "SystemLog.h"
#include "OTAManager.h"
#include "BLESerial.h"
#include "BleRtcmClient.h"
#include "FlashConfig.h"
#include "SurveyPoints.h"
#include "PointCodes.h"
#include "Stakeout.h"
#include "EspNowRtcm.h"
#if ENC_CLK_GPIO > 0 && ENC_DT_GPIO > 0 && ENC_SW_GPIO > 0
#include "RotaryInput.h"
#include "OledMenu.h"
#endif

// All hardware pin definitions are now in config.h
// This allows for easy multi-board support via platformio.ini environments

// Forward declarations
extern NtripClient* ntripClient;
void toggleNtrip(bool enable);
void toggleTcpIn(bool enable);
void surveyAddSample();
bool applyMdnsHostname(const char* hostname);
bool loadBleName(char* out, size_t maxLen);
bool saveBleName(const char* name);
bool loadBlePin(uint32_t* out);
bool saveBlePin(uint32_t pin);
bool applyBleName(const char* newName);
bool startEspNowRx();
void stopEspNowRx();
bool startEspNowTx();
void stopEspNowTx();
void stopBaseMode();

struct GNSSPacket { uint8_t data[PACKET_SIZE]; size_t len; };

HardwareSerial GNSSSerial(1);   // UART1 - RX from ZED TX2 (NMEA/UBX o RTCM)
HardwareSerial RTCMSerial(2);   // UART2 - TX to ZED RX2 (RTCM input dal caster rover)

WebServer server(80);

SdFat sd;

// ===== Time sync / timestamped log filenames =====
// Italian local time (CET/CEST), synced either via NTP (when STA connected) or via UBX-NAV-TIMEUTC (offline AP mode).
enum TimeSyncSource :  uint8_t { TIME_SRC_NONE = 0, TIME_SRC_NTP = 1, TIME_SRC_TIMEUTC = 2 };
volatile uint8_t g_timeSource = TIME_SRC_NONE;
volatile time_t  g_lastSyncEpoch = 0;

// NTP server config (can be edited from WebUI)
char g_ntpServer[64] = "pool.ntp.org";
// NTP timezone POSIX string (can be edited from WebUI, default: Italy CET/CEST)
char g_ntpTz[64] = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// mDNS hostname (can be edited from WebUI) - browse: http://<name>.local/
// NOTE: only the host label is stored here (without .local)
char g_mdnsName[32] = "rtkino";

// ===== BLE configuration =====
// BLE is ALWAYS disabled by default on boot (non-persistent)
bool g_bleEnabled = false;
// BLE device name (persistent, saved to SD)
char g_bleDeviceName[21] = "RTKino";
// BLE pairing PIN (persistent, saved to SD)
uint32_t g_blePasskey = 123456;  // Default PIN

// ===== BLE RTCM input (correction source from rtcm-lora radio) =====
BleRtcmClient g_bleRtcm;
bool g_bleRtcmEnabled = false;
char g_bleRtcmTargetName[21] = "rtcm-lora";
uint32_t g_bleRtcmPasskey = 123456;

// ===== ESP-NOW RTCM mesh =====
EspNowRtcm g_espNow;
bool g_espNowEnabled   = false;  // true = ESP-NOW active
bool g_espNowTxEnabled = false;  // true = base mode (TX); false = rover mode (RX)
static uint32_t g_espNowLastTelem = 0;

// ESP-NOW relay selection (rover side)
uint16_t g_espNowRelayNodeId    = 0;      // best relay node_id (0 = none selected)
static uint32_t g_espNowRelayLastReq   = 0;      // millis() of last CMD_RELAY_START sent
static uint32_t g_espNowRelayLeaseMs   = 10000;  // send CMD_RELAY_START every 10s to renew lease

// AP-mode boot time (for offline TIMEUTC sync gating)
volatile uint32_t g_apStartMillis = 0;
volatile bool g_apMode = false;

// Forward declarations (necessarie:  le definizioni sono più sotto nel file)
static bool syncTimeFromUbxTimeUtc(uint16_t year, uint8_t month, uint8_t day,
                                   uint8_t hour, uint8_t min, uint8_t sec,
                                   uint8_t validFlags);

static bool makeTimestampedLogFilename(char* out, size_t outSize);

// ===== GNSS Position Status (shared definition in gnss_types.h) =====
#include "gnss_types.h"

GNSSPosition g_position = {
    0.0, 0.0, 0.0,       // lat, lon, alt
    0, 0, 0,             // fixQuality, numSats, numSV
    99.9, 99.9, 99.9,    // hdop, pdop, vdop
    0.0, 0, 0, 0,        // age, carrSoln, lastUpdate, lastPvtUpdate
    0.0f, 0.0f, false,   // hAcc, vAcc, hpAccValid
    0.0, 0.0,            // latHP, lonHP
    0.0, 0.0,            // altHAE, altMSLHP
    0.0, 0.0, 0.0,       // ecefX, ecefY, ecefZ
    0.0f, false,         // pAcc, ecefValid
    0.0f, 0.0f, 0.0f, 0.0f,   // gdop, ndop, edop, tdop
    0.0f, 0.0f, 0.0f,    // covNN, covEE, covDD
    0.0f, 0.0f, 0.0f,    // covNE, covND, covED
    false,               // covValid
    0.0, 0.0, 0.0,       // relN, relE, relD
    0.0f, 0.0f, 0.0f,    // relSN, relSE, relSD
    false                // relValid
};
SemaphoreHandle_t positionMutex = nullptr;

// Helper thread-safe per leggere la posizione
bool getPosition(GNSSPosition& out) {
    if (!positionMutex) return false;
    if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    out = g_position;
    xSemaphoreGive(positionMutex);
    return true;
}

// ===== Survey State (for averaging base position) =====
struct SurveyState {
    bool active;
    bool completed;             // true when survey ended naturally (duration reached)
    uint32_t startTime;
    uint32_t durationMs;        // configured duration (min 30000ms)
    uint32_t sampleCount;
    double latSum, lonSum, altSum;
    double latSqSum, lonSqSum, altSqSum;  // for standard deviation
    float instrumentHeight;     // ground to ARP (user input)
    float arpOffset;            // antenna internal offset
    String pendingName;         // name for saving
};

SurveyState g_survey = {false, false, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ""};
SemaphoreHandle_t surveyMutex = nullptr;

// ===== RTCM Statistics — struct defined in gnss_types.h =====

RtcmStats g_rtcmStats = {0};
SemaphoreHandle_t rtcmStatsMutex = nullptr;

// ===== ZED-F9P TMODE State =====
ZedTmodeState g_zedTmode = {0, 0, 0.0, 0.0, 0.0, false, 0};
SemaphoreHandle_t zedTmodeMutex = nullptr;

// Safe access helper to g_zedTmode
bool zedTmodeLock(uint32_t timeoutMs = 100) {
    return (zedTmodeMutex && xSemaphoreTake(zedTmodeMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

void zedTmodeUnlock() {
    if (zedTmodeMutex) xSemaphoreGive(zedTmodeMutex);
}

bool getZedTmode(ZedTmodeState& out) {
    if (!zedTmodeLock(100)) return false;
    out = g_zedTmode;
    zedTmodeUnlock();
    return true;
}

// ===== GLOBAL MUTEX=====
// Global Mutex for all SD/SPI accesses (logger + WebUI + profiles)
SemaphoreHandle_t sdMutex = nullptr;

// SD card availability flag (set in setup, used in loop for periodic sync)
bool sdOK = false;

// Mutex to protect ntripClient from concurrent access (FIX CRASH)
SemaphoreHandle_t ntripMutex = nullptr;

// Mutex to protect g_tcpIn (RTCMInput) from concurrent access
SemaphoreHandle_t tcpInMutex = nullptr;

// Mutex to protect g_pusher (NtripPusher) from concurrent access
SemaphoreHandle_t pusherMutex = nullptr;

// Helper for safe access to ntripClient
bool ntripLock(uint32_t timeoutMs = 100) {
    return (ntripMutex && xSemaphoreTake(ntripMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

void ntripUnlock() {
    if (ntripMutex) xSemaphoreGive(ntripMutex);
}

// Helper for safe access to g_tcpIn
bool tcpInLock(uint32_t timeoutMs = 100) {
    return (tcpInMutex && xSemaphoreTake(tcpInMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

void tcpInUnlock() {
    if (tcpInMutex) xSemaphoreGive(tcpInMutex);
}

// Helper for safe access to g_pusher
bool pusherLock(uint32_t timeoutMs = 100) {
    return (pusherMutex && xSemaphoreTake(pusherMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

void pusherUnlock() {
    if (pusherMutex) xSemaphoreGive(pusherMutex);
}

// Controlled flush of RAW log (prevents empty file after reset)
static const uint32_t RAW_LOG_SYNC_MS = 2000;        // 0 = disable
static const size_t   RAW_LOG_SYNC_BYTES = 256 * 1024; // 0 = disable

// Queue warning threshold (percentage of queue size)
static const uint8_t QUEUE_WARNING_THRESHOLD_PERCENT = 75;

String ntrip_host, mountpoint, ntrip_user, ntrip_pass;

int ntrip_port = 0;

NtripClient* ntripClient = nullptr;   // NTRIP IN (rover)

FsFile rawFile;

uint16_t logFileIndex = 0;

volatile bool loggingActive = false;

volatile bool ntripEnabled = false;  // rover IN - volatile for cross-task visibility

bool wifiAvailable = false; // used to run web/tcp even in AP

bool silentMode = false;

unsigned long lastOledUpdate = 0;

String lastRateSet = "---";

QueueHandle_t sdQueue;

TaskHandle_t uartTaskHandle = NULL;

TaskHandle_t sdTaskHandle = NULL;

TaskHandle_t nmeaTaskHandle = NULL;

static std::vector<WifiCred> wifiList;

// ===== Base output state =====
static volatile bool g_baseCasterOn = false;
static volatile bool g_baseTcpOn    = false;
static String g_outHost, g_outMount, g_outPass;
static uint16_t g_outPort = 2101, g_tcpPort = 2102;

// NB: NtripPusher does not have a default constructor -> we use a pointer
static NtripPusher* g_pusher = nullptr;

// TCP OUT CLIENT state
static String g_tcpClientHost = "";
static uint16_t g_tcpClientPort = 0;
static bool g_tcpClientOn = false;
// ===== TCP streaming mode (viewer) =====
enum class StreamMode : uint8_t { NMEA = 0, RAW = 1 };
static volatile StreamMode g_streamMode = StreamMode::NMEA;
static inline bool isRawMode() { return g_streamMode == StreamMode::RAW; }
static const char* streamModeName() { return isRawMode() ? "RAW (UBX)" : "NMEA+UBX/RTCM"; }
extern "C" {
  void setStreamModeRaw()  { g_streamMode = StreamMode::RAW;  oledPrintln(String("[TCP] Mode:  ") + streamModeName()); }
  void setStreamModeNmea() { g_streamMode = StreamMode::NMEA; oledPrintln(String("[TCP] Mode: ") + streamModeName()); }
  const char* getStreamModeName() { return streamModeName(); }
}

// ---- New: local TCP input state (LAN IN) ----
RTCMInput* g_tcpIn = nullptr;      // client TCP for RTCM IN
bool tcpInEnabled  = false;
String tcpin_host; int tcpin_port = 0;

// ===== NEW: Buzzer and SystemLog =====
Buzzer* g_buzzer = nullptr;
SystemLog* g_systemLog = nullptr;
static uint8_t g_lastCarrSoln = 0;  // Track RTK fix state changes
static uint32_t g_lastHeapCheck = 0;  // For periodic heap monitoring

// EXTINT marker flag — toggled from WebUI; drives GPIO6 HIGH/LOW around sampling
volatile bool g_extintMarkerEnabled = false;

// ---------------- ZED helpers (reading/CFG-RATE legacy via I2C) ----------------
void readCfgRateFromZED() {
  const uint8_t pollRate[] = { 0xB5,0x62,0x06,0x08,0x00,0x00,0x0E,0x30 };
  Wire.beginTransmission(ZED_I2C_ADDR);
  for (uint8_t b :  pollRate) Wire.write(b);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf(">> I2C error polling CFG-RATE: %d\n", err);
    lastRateSet = "I2C err";
    return;
  }
  delay(100);
  Wire.requestFrom(ZED_I2C_ADDR, 16);
  uint8_t buffer[16];
  int i = 0;
  while (Wire.available() && i < 16) buffer[i++] = Wire.read();
  if (i == 16 && buffer[0]==0xB5 && buffer[1]==0x62 && buffer[2]==0x06 && buffer[3]==0x08) {
    uint16_t measRate = buffer[6] | (buffer[7] << 8);
    lastRateSet = measRate ?  String(1000 / measRate) + " Hz" : String("---");
    Serial.println(">> Current CFG-RATE: " + lastRateSet);
  } else {
    lastRateSet = "---";
    Serial.println(">> Error reading CFG-RATE");
  }
}

void sendCfgSaveToZED() {
  uint8_t msg[19] = {0xB5,0x62,0x06,0x09,13,0x00,0,0,0,0, 0xFF,0xFF,0,0, 0,0,0,0, 0x07};
  uint8_t ckA=0, ckB=0;
  for (int i=2;i<17;i++){ ckA+=msg[i]; ckB+=ckA; }
  Wire.beginTransmission(ZED_I2C_ADDR);
  for (int i=0;i<19;i++) Wire.write(msg[i]);
  Wire.write(ckA); Wire.write(ckB);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf(">> I2C error saving config: %d\n", err);
    return;
  }
  Serial.println("Configuration save sent to ZED (RAM, BBR, Flash)");
}

void sendCfgRateToZED(uint16_t measRateMs) {
  uint8_t payload[6] = {
    (uint8_t)(measRateMs & 0xFF), (uint8_t)(measRateMs >> 8), 0x01, 0x00, 0x01, 0x00
  };
  uint8_t msg[14] = {0xB5,0x62,0x06,0x08,6,0x00};
  memcpy(&msg[6], payload, 6);
  uint8_t ckA=0, ckB=0;
  for (int i=2;i<12;i++){ ckA+=msg[i]; ckB+=ckA; }
  msg[12]=ckA; msg[13]=ckB;
  Wire. beginTransmission(ZED_I2C_ADDR);
  for (int i=0;i<14;i++) Wire.write(msg[i]);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf(">> I2C error setting rate: %d\n", err);
    return;
  }
  Serial.printf("CFG-RATE sent via I2C:  %u Hz\n", 1000/measRateMs);
  lastRateSet = String(1000 / measRateMs) + " Hz";
  delay(100);
  sendCfgSaveToZED();
}

// ---------------- RAW logging ----------------
void startLogging() {
  if (!sdMutex) { oledPrintln("SD mutex err"); return; }
  bool locked = (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE);
  if (! locked) { oledPrintln("SD busy"); return; }
  char filename[40];
  // Prefer timestamped filename when system time is synced; otherwise fallback to numeric sequence. 
  bool hasTs = makeTimestampedLogFilename(filename, sizeof(filename));
  if (hasTs) {
    // If a file with same timestamp exists (rare), append a suffix. 
    if (sd.exists(filename)) {
      for (int k = 1; k < 1000; k++) {
        snprintf(filename, sizeof(filename), "/gnss/log_%ld_%03d.ubx", (long)time(nullptr), k);
        if (! sd.exists(filename)) break;
      }
    }
  } else {
    do { snprintf(filename, sizeof(filename), "/gnss/log_%03d.ubx", logFileIndex++); }
    while (sd.exists(filename));
  }
  rawFile = sd.open(filename, FILE_WRITE);
  if (rawFile && rawFile.isOpen()) {
    oledSetLogging(true);
    loggingActive = true;
    oledPrintln(String("[LOG] ") + filename);
  } else {
    oledPrintln("RAW log error");
    if (! hasTs) logFileIndex--;
  }
  xSemaphoreGive(sdMutex);
}

void stopLogging() {
  loggingActive = false;
  delay(20);
  if (! sdMutex) { oledSetLogging(false); return; }
  bool locked = (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE);
  if (!locked) { oledPrintln("SD busy"); return; }
  if (rawFile && rawFile.isOpen()) {
    // Close more safely: explicit sync before close
    rawFile.sync();
    rawFile.close();
  }
  xSemaphoreGive(sdMutex);
  oledSetLogging(false);
  oledPrintln("[LOG] Stopped");
  // Force config sync to SD after logging stops
  if (FlashConfig::isDirty()) {
    FlashConfig::syncToSD(sd, sdMutex, true);
  }
}

// --------- Mini-RTCM sniffer (type+len) ----------
static inline int rtcm_type(const uint8_t* p, size_t n, size_t& outLen) {
  if (n < 6 || p[0] != 0xD3) return 0;
  size_t len = ((p[1] & 0x03) << 8) | p[2];
  if (len + 6 > n) return 0; // incomplete frame (3 hdr + payload + 3 CRC)
  int type = (p[3] << 4) | (p[4] >> 4);
  outLen = len + 6;
  return type;
}

// Parser NMEA GGA
// Format: $GNGGA,hhmmss.ss,llll.lll,N,yyyyy.yyy,E,q,nn,hdop,alt,M,geoid,M,age,refid*cs
static void parseGGA(const char* line) {
    if (!line || !positionMutex) return;
       // Local copy for strtok
    char buf[160];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
     char* tokens[15];
    int tokenCount = 0;
     char* p = strtok(buf, ",");
    while (p && tokenCount < 15) {
        tokens[tokenCount++] = p;
        p = strtok(NULL, ",");
    }
    // GGA has at least 14 fields
    if (tokenCount < 14) return;
    // tokens[0] = $GNGGA o $GPGGA
    // tokens[1] = time (hhmmss.ss)
    // tokens[2] = lat (ddmm.mmmmm)
    // tokens[3] = N/S
    // tokens[4] = lon (dddmm.mmmmm)
    // tokens[5] = E/W
    // tokens[6] = fix quality
    // tokens[7] = num satellites
    // tokens[8] = HDOP
    // tokens[9] = altitude
    // tokens[10] = M
    // tokens[11] = geoid separation
    // tokens[12] = M
    // tokens[13] = age of differential (can be empty)
        // Fix quality
    uint8_t fix = (uint8_t)atoi(tokens[6]);
        // Number of satellites
    uint8_t sats = (uint8_t)atoi(tokens[7]);
        // HDOP
    float hdop = atof(tokens[8]);
    if (hdop <= 0) hdop = 99.9;
        // Altitude
    float alt = atof(tokens[9]);
        // Age of differential (can be empty)
    float age = 0.0;
    if (tokens[13] && strlen(tokens[13]) > 0) {
        age = atof(tokens[13]);
    }
        // Latitude: ddmm.mmmmm -> decimal degrees
    double lat = 0.0;
    if (strlen(tokens[2]) >= 4) {
        double raw = atof(tokens[2]);
        int deg = (int)(raw / 100);
        double min = raw - (deg * 100);
        lat = deg + (min / 60.0);
        if (tokens[3] && strlen(tokens[3]) > 0 && tokens[3][0] == 'S') lat = -lat;
    }
        // Longitude: dddmm.mmmmm -> decimal degrees  
    double lon = 0.0;
    if (strlen(tokens[4]) >= 5) {
        double raw = atof(tokens[4]);
        int deg = (int)(raw / 100);
        double min = raw - (deg * 100);
        lon = deg + (min / 60.0);
        if (tokens[5] && strlen(tokens[5]) > 0 && tokens[5][0] == 'W') lon = -lon;
    }
   
    // Update global structure (thread-safe)
    if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_position.lat = lat;
        g_position.lon = lon;
        g_position.alt = alt;
        g_position.fixQuality = fix;
        g_position.numSats = sats;
        g_position.hdop = hdop;
        g_position.age = age;
        g_position.lastUpdate = millis();
        xSemaphoreGive(positionMutex);
        // Update survey if active
        surveyAddSample();
    }
}

// Update RTCM statistics (called from UBX parser)
static void updateRtcmStats(uint16_t msgType, bool crcOk) {
    if (!rtcmStatsMutex) return;
    if (xSemaphoreTake(rtcmStatsMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
    RtcmStats* s = &g_rtcmStats;
    uint32_t now = millis();
   // Check if this type already exists
    int idx = -1;
    for (int i = 0; i < s->numTypes; i++) {
        if (s->msgs[i].msgType == msgType) {
            idx = i;
            break;
        }
    }
    // If it doesn't exist and there's space, add it
    if (idx < 0 && s->numTypes < RTCM_MAX_TYPES) {
        idx = s->numTypes++;
        s->msgs[idx].msgType = msgType;
        s->msgs[idx].count = 0;
        s->msgs[idx].crcErrors = 0;
    }
    
    // Update statistics
    if (idx >= 0) {
        s->msgs[idx].count++;
        s->msgs[idx].lastSeen = now;
        if (!crcOk) s->msgs[idx].crcErrors++;
    }
    
    s->totalMessages++;
    s->lastUpdate = now;
    
    xSemaphoreGive(rtcmStatsMutex);
}

// Helper to read statistics (thread-safe)
bool getRtcmStats(RtcmStats& out) {
    if (!rtcmStatsMutex) return false;
    if (xSemaphoreTake(rtcmStatsMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    memcpy(&out, &g_rtcmStats, sizeof(RtcmStats));
    xSemaphoreGive(rtcmStatsMutex);
    return true;
}

// Helper to reset RTCM statistics (used when changing NTRIP profile)
void resetRtcmStats() {
    if (!rtcmStatsMutex) return;
    if (xSemaphoreTake(rtcmStatsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&g_rtcmStats, 0, sizeof(RtcmStats));
        xSemaphoreGive(rtcmStatsMutex);
    }
}

// ---------------- FreeRTOS tasks ----------------
void nmeaReaderTask(void* pvParameters) {
  // UART1 = ZED UART2 TX (NMEA/UBX/RTCM base) -> SOURCE for caster/TCP when in base
  uint8_t buffer[1024];
  static char line[160]; static size_t idx = 0; static bool inLine = false;
  // UBX parser state (NAV-TIMEUTC, NAV-PVT, RXM-RTCM), for time and position sync
  static uint8_t ubxState = 0;
  static uint8_t ubxClass = 0;
  static uint8_t ubxId    = 0;
  static uint16_t ubxLen  = 0;
  static uint16_t ubxPos  = 0;
  static uint8_t ubxPayload[100]; // 100 bytes covers NAV-PVT (92), NAV-RELPOSNED v1 (72), NAV-COV (64)
  static uint8_t ckA = 0;
  static uint8_t ckB = 0;
  auto isPrintable = [](char c) {
    return (c >= 32 && c <= 126) || c == ',' || c == '.' || c == '*' || c == '$';
  };

  while (true) {
    int len = 0;
    int ava = GNSSSerial.available();
    if (ava > 0) {
      if (ava > (int)sizeof(buffer)) ava = sizeof(buffer);
      len = GNSSSerial.readBytes(buffer, ava); // short timeout (20 ms)
    }
    if (len > 0) {
      // --- RTCM sniffer: log some key types (throttle 500 ms)
      static uint32_t lastPrint = 0;
      uint32_t now = millis();
      if (now - lastPrint > 500) {
        for (int i = 0; i + 6 <= len; ) {
          size_t fLen = 0;
          int t = rtcm_type(&buffer[i], len - i, fLen);
          if (!t) { i++; continue; }
          if (t==1005 || t==1006 || t==1077 || t==1087 || t==1097 || t==1127 || t==1230) {
            Serial.printf("[RTCM] type=%d len=%u\n", t, (unsigned)fLen);
            lastPrint = now;
            break;
          }
          i += fLen ?  (int)fLen : 1;
        }
      }

      // Viewer:  only if not in RAW mode
      if (!isRawMode()) {
        TcpStreamer::broadcast(buffer, len);
        if (g_tcpClientOn) {
          TcpClientStreamer::broadcast(buffer, len);
        }
      }
      
      // BLE broadcast (NMEA/UBX to connected app)
      if (g_bleEnabled && BLESerial::isConnected()) {
        BLESerial::write(buffer, len);
      }
      
      // RTCM source of the base → caster and TCP
      if (g_baseCasterOn) {
        if (pusherLock(2)) {
          if (g_pusher && g_pusher->isConnected()) {
            g_pusher->write(buffer, len);
          }
          pusherUnlock();
        }
      }
      if (g_baseTcpOn && RtcmStreamer::active()) {
        RtcmStreamer::broadcast(buffer, len);
      }
      // BLE RTCM output (base mode: send RTCM to rtcm-lora via BLE NUS)
      if (g_bleRtcmEnabled && g_bleRtcm.isConnected()) {
        g_bleRtcm.writeRtcm(buffer, len);
      }
      // ESP-NOW TX (base mode): fragment and broadcast RTCM via ESP-NOW mesh
      // Fire and forget — drops if radio busy, never buffers.
      if (g_espNowEnabled && g_espNowTxEnabled) {
        g_espNow.broadcastRtcm(buffer, len);
      }
      // Extract GGA only if we detect NMEA (rover)
      for (int i = 0; i < len; i++) {
        uint8_t b = buffer[i];
        // --- UBX frame parsing (NAV-TIMEUTC) ---
        switch (ubxState) {
          case 0: // sync1
            if (b == 0xB5) ubxState = 1;
            break;

          case 1: // sync2
            if (b == 0x62) { ubxState = 2; ckA = 0; ckB = 0; }
            else ubxState = 0;
            break;

          case 2: // class
            ubxClass = b; ckA += b; ckB += ckA; ubxState = 3;
            break;

          case 3: // id
            ubxId = b; ckA += b; ckB += ckA; ubxState = 4;
            break;

          case 4: // len L
            ubxLen = b; ckA += b; ckB += ckA; ubxState = 5;
            break;

          case 5: // len H
            ubxLen |= ((uint16_t)b << 8);
            ckA += b; ckB += ckA;
            ubxPos = 0;
            if (ubxLen > sizeof(ubxPayload)) { ubxState = 0; break; }
            ubxState = (ubxLen == 0) ? 7 : 6;
            break;

          case 6: // payload
            ubxPayload[ubxPos++] = b;
            ckA += b; ckB += ckA;
            if (ubxPos >= ubxLen) ubxState = 7;
            break;

          case 7: // ckA
            if (b == ckA) ubxState = 8;
            else ubxState = 0;
            break;

          case 8: // ckB
            if (b == ckB) {

              // complete frame
              if (ubxClass == 0x01 && ubxId == 0x21 && ubxLen == 20) {
                // NAV-TIMEUTC (year.. valid are in payload[12.. 19])
                uint16_t year  = (uint16_t)ubxPayload[12] | ((uint16_t)ubxPayload[13] << 8);
                uint8_t  month = ubxPayload[14];
                uint8_t  day   = ubxPayload[15];
                uint8_t  hour  = ubxPayload[16];
                uint8_t  min   = ubxPayload[17];
                uint8_t  sec   = ubxPayload[18];
                uint8_t  valid = ubxPayload[19];

                // Offline strategy: only in AP, after ~60s, and only if we haven't already synced
                if (g_apMode) {
                  uint32_t sinceAp = (uint32_t)(millis() - g_apStartMillis);
                  if (sinceAp > 60000 && g_timeSource == TIME_SRC_NONE) {
                    syncTimeFromUbxTimeUtc(year, month, day, hour, min, sec, valid);
                  }
                }
              }

// RXM-RTCM (0x02 0x32) - incoming RTCM statistics
if (ubxClass == 0x02 && ubxId == 0x32 && ubxLen >= 8) {
  uint8_t flags = ubxPayload[1];

  // In spec “classica” msgType è a offset 6-7, ma per robustezza
  // prendiamo gli ultimi due byte del payload (resta corretto anche se cambia layout).
  uint16_t msgType = (uint16_t)ubxPayload[ubxLen - 2] | ((uint16_t)ubxPayload[ubxLen - 1] << 8);

  bool crcOk = !(flags & 0x01);  // bit 0 = crcFailed
  updateRtcmStats(msgType, crcOk);
}

              // NAV-DOP (0x01 0x04) - accurate DOP values including VDOP
              if (ubxClass == 0x01 && ubxId == 0x04 && ubxLen == 18) {
                uint16_t gdopRaw = (uint16_t)ubxPayload[4]  | ((uint16_t)ubxPayload[5]  << 8);
                uint16_t pdopRaw = (uint16_t)ubxPayload[6]  | ((uint16_t)ubxPayload[7]  << 8);
                uint16_t tdopRaw = (uint16_t)ubxPayload[8]  | ((uint16_t)ubxPayload[9]  << 8);
                uint16_t vdopRaw = (uint16_t)ubxPayload[10] | ((uint16_t)ubxPayload[11] << 8);
                uint16_t hdopRaw = (uint16_t)ubxPayload[12] | ((uint16_t)ubxPayload[13] << 8);
                uint16_t ndopRaw = (uint16_t)ubxPayload[14] | ((uint16_t)ubxPayload[15] << 8);
                uint16_t edopRaw = (uint16_t)ubxPayload[16] | ((uint16_t)ubxPayload[17] << 8);
                
                // Update position (thread-safe)
                if (positionMutex && xSemaphoreTake(positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  g_position.pdop = (float)pdopRaw * 0.01f;
                  g_position.vdop = (float)vdopRaw * 0.01f;
                  g_position.hdop = (float)hdopRaw * 0.01f;  // More accurate than GGA HDOP
                  g_position.gdop = (float)gdopRaw * 0.01f;
                  g_position.tdop = (float)tdopRaw * 0.01f;
                  g_position.ndop = (float)ndopRaw * 0.01f;
                  g_position.edop = (float)edopRaw * 0.01f;
                  xSemaphoreGive(positionMutex);
                }
              }

              // NAV-PVT (0x01 0x07) - position and satellite count
              if (ubxClass == 0x01 && ubxId == 0x07 && ubxLen == 92) {
                uint8_t fixType = ubxPayload[20];           // offset 20: fixType
                uint8_t flags   = ubxPayload[21];           // offset 21: flags
                uint8_t numSV   = ubxPayload[23];           // offset 23: numSV
                uint32_t hAccRaw = (uint32_t)ubxPayload[24] | ((uint32_t)ubxPayload[25] << 8)
                                 | ((uint32_t)ubxPayload[26] << 16) | ((uint32_t)ubxPayload[27] << 24); // offset 24-27: hAcc (mm)
                uint32_t vAccRaw = (uint32_t)ubxPayload[28] | ((uint32_t)ubxPayload[29] << 8)
                                 | ((uint32_t)ubxPayload[30] << 16) | ((uint32_t)ubxPayload[31] << 24); // offset 28-31: vAcc (mm)
                uint16_t pdopRaw = (uint16_t)ubxPayload[76] | ((uint16_t)ubxPayload[77] << 8); // offset 76-77: pDOP
                
                // Estract carrSoln (bits 6-7 flags)
                uint8_t carrSoln = (flags >> 6) & 0x03;     // 0=no RTK, 1=float, 2=fixed
                // Convert pDOP (scale 0.01)
                float pdop = (float)pdopRaw * 0.01f;
                // Convert hAcc/vAcc from mm to metres (fallback if HPPOSLLH not received)
                float hAcc = (float)hAccRaw * 0.001f;
                float vAcc = (float)vAccRaw * 0.001f;
                
                // Update position (thread-safe)
                if (positionMutex && xSemaphoreTake(positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  g_position.numSV = numSV;
                  g_position.pdop = pdop;
                  g_position.carrSoln = carrSoln;
                  // Only update hAcc/vAcc from PVT if HPPOSLLH has not provided them yet
                  if (!g_position.hpAccValid) {
                    g_position.hAcc = hAcc;
                    g_position.vAcc = vAcc;
                  }
                  g_position.lastPvtUpdate = millis();
                  xSemaphoreGive(positionMutex);
                }
              }

              // NAV-HPPOSLLH (0x01 0x14) - high precision position and accuracy
              if (ubxClass == 0x01 && ubxId == 0x14 && ubxLen == 36) {
                // lon/lat/height at offsets 8-27 (all I4, 1e-7 deg / mm)
                // lonHp/latHp/heightHp/hMSLHp at offsets 24-27 (I1, 0.1mm)
                int32_t lonRaw, latRaw, heightRaw, hMSLRaw;
                memcpy(&lonRaw,    &ubxPayload[8],  4);
                memcpy(&latRaw,    &ubxPayload[12], 4);
                memcpy(&heightRaw, &ubxPayload[16], 4);
                memcpy(&hMSLRaw,   &ubxPayload[20], 4);
                int8_t lonHp    = (int8_t)ubxPayload[24];
                int8_t latHp    = (int8_t)ubxPayload[25];
                int8_t heightHp = (int8_t)ubxPayload[26];
                int8_t hMSLHp   = (int8_t)ubxPayload[27];

                // hAcc at offset 28-31, vAcc at offset 32-35 (units: 0.1 mm)
                uint32_t hAccRaw = (uint32_t)ubxPayload[28] | ((uint32_t)ubxPayload[29] << 8)
                                 | ((uint32_t)ubxPayload[30] << 16) | ((uint32_t)ubxPayload[31] << 24);
                uint32_t vAccRaw = (uint32_t)ubxPayload[32] | ((uint32_t)ubxPayload[33] << 8)
                                 | ((uint32_t)ubxPayload[34] << 16) | ((uint32_t)ubxPayload[35] << 24);

                double lon    = lonRaw    * 1e-7 + lonHp    * 1e-9;
                double lat    = latRaw    * 1e-7 + latHp    * 1e-9;
                double altHAE = heightRaw * 0.001 + heightHp * 0.0001;
                double altMSL = hMSLRaw   * 0.001 + hMSLHp   * 0.0001;
                float  hAcc   = (float)hAccRaw * 0.0001f;
                float  vAcc   = (float)vAccRaw * 0.0001f;

                if (positionMutex && xSemaphoreTake(positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  g_position.hAcc       = hAcc;
                  g_position.vAcc       = vAcc;
                  g_position.hpAccValid = true;
                  g_position.latHP      = lat;
                  g_position.lonHP      = lon;
                  g_position.altHAE     = altHAE;
                  g_position.altMSLHP   = altMSL;
                  xSemaphoreGive(positionMutex);
                }
              }

              // NAV-HPPOSECEF (0x01 0x13) - high precision ECEF position
              if (ubxClass == 0x01 && ubxId == 0x13 && ubxLen == 28) {
                int32_t ecefXRaw, ecefYRaw, ecefZRaw;
                memcpy(&ecefXRaw, &ubxPayload[8],  4);
                memcpy(&ecefYRaw, &ubxPayload[12], 4);
                memcpy(&ecefZRaw, &ubxPayload[16], 4);
                int8_t ecefXHp = (int8_t)ubxPayload[20];
                int8_t ecefYHp = (int8_t)ubxPayload[21];
                int8_t ecefZHp = (int8_t)ubxPayload[22];
                uint32_t pAccRaw = (uint32_t)ubxPayload[24] | ((uint32_t)ubxPayload[25] << 8)
                                 | ((uint32_t)ubxPayload[26] << 16) | ((uint32_t)ubxPayload[27] << 24);

                double ecefX = ecefXRaw * 0.01 + ecefXHp * 0.0001;
                double ecefY = ecefYRaw * 0.01 + ecefYHp * 0.0001;
                double ecefZ = ecefZRaw * 0.01 + ecefZHp * 0.0001;
                float  pAcc  = (float)pAccRaw * 0.0001f;

                if (positionMutex && xSemaphoreTake(positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  g_position.ecefX     = ecefX;
                  g_position.ecefY     = ecefY;
                  g_position.ecefZ     = ecefZ;
                  g_position.pAcc      = pAcc;
                  g_position.ecefValid = true;
                  xSemaphoreGive(positionMutex);
                }
              }

              // NAV-COV (0x01 0x36) - position covariance matrix (64-byte payload)
              if (ubxClass == 0x01 && ubxId == 0x36 && ubxLen == 64) {
                uint8_t posCovValid = ubxPayload[5];
                float covNN, covNE, covND, covEE, covED, covDD;
                memcpy(&covNN, &ubxPayload[12], 4);
                memcpy(&covNE, &ubxPayload[16], 4);
                memcpy(&covND, &ubxPayload[20], 4);
                memcpy(&covEE, &ubxPayload[24], 4);
                memcpy(&covED, &ubxPayload[28], 4);
                memcpy(&covDD, &ubxPayload[32], 4);

                if (positionMutex && xSemaphoreTake(positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  g_position.covNN    = covNN;
                  g_position.covEE    = covEE;
                  g_position.covDD    = covDD;
                  g_position.covNE    = covNE;
                  g_position.covND    = covND;
                  g_position.covED    = covED;
                  g_position.covValid = (posCovValid & 0x01) != 0;
                  xSemaphoreGive(positionMutex);
                }
              }

              // NAV-RELPOSNED (0x01 0x3C) - relative position to base (64-byte payload)
              if (ubxClass == 0x01 && ubxId == 0x3C && ubxLen >= 64) {
                int32_t relPosN_cm, relPosE_cm, relPosD_cm;
                memcpy(&relPosN_cm, &ubxPayload[8],  4);
                memcpy(&relPosE_cm, &ubxPayload[12], 4);
                memcpy(&relPosD_cm, &ubxPayload[16], 4);
                int8_t relHPN = (int8_t)ubxPayload[32];
                int8_t relHPE = (int8_t)ubxPayload[33];
                int8_t relHPD = (int8_t)ubxPayload[34];
                uint32_t accN_raw, accE_raw, accD_raw;
                memcpy(&accN_raw, &ubxPayload[36], 4);
                memcpy(&accE_raw, &ubxPayload[40], 4);
                memcpy(&accD_raw, &ubxPayload[44], 4);
                uint32_t flags32;
                memcpy(&flags32, &ubxPayload[60], 4);

                double relN = relPosN_cm * 0.01 + relHPN * 0.0001;
                double relE = relPosE_cm * 0.01 + relHPE * 0.0001;
                double relD = relPosD_cm * 0.01 + relHPD * 0.0001;
                float  relSN = (float)accN_raw * 0.0001f;
                float  relSE = (float)accE_raw * 0.0001f;
                float  relSD = (float)accD_raw * 0.0001f;
                bool relValid = ((flags32 >> 2) & 0x01) != 0;

                if (positionMutex && xSemaphoreTake(positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  g_position.relN     = relN;
                  g_position.relE     = relE;
                  g_position.relD     = relD;
                  g_position.relSN    = relSN;
                  g_position.relSE    = relSE;
                  g_position.relSD    = relSD;
                  g_position.relValid = relValid;
                  xSemaphoreGive(positionMutex);
                }
              }
            }
            ubxState = 0;
            break;
        }

        char c = (char)b;
        if (!inLine) { if (c == '$') { inLine = true; idx = 0; line[idx++] = c; } continue; }
        if (c == '\r') continue;
        if (c == '\n') {
          if (idx > 6 && line[0]=='$' && line[3]=='G' && line[4]=='G' && line[5]=='A') {
            // Null-terminate for safety
            line[idx] = '\0';
            parseGGA(line);
            if (ntripEnabled) {
              if (ntripLock(10)) {  
                if (ntripClient && ntripClient->isActive()) {
                  ntripClient->sendGGALine(line, idx);
                }
                ntripUnlock();
              }
            }
          }

          inLine = false; idx = 0; continue;
        }

        if (! isPrintable(c)) { inLine = false; idx = 0; continue; }
        if (idx < sizeof(line) - 2) line[idx++] = c; else { inLine = false; idx = 0; }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // more responsive
  }
}

void sdWriterTask(void* pvParameters) {
  GNSSPacket packet;
  uint32_t lastSyncMs = 0;
  size_t bytesSinceSync = 0;
  while (true) {
    if (xQueueReceive(sdQueue, &packet, pdMS_TO_TICKS(100))) {
      // Quick preliminary check (without lock)
      if (!loggingActive || !sdMutex) {
        continue;
      }
      // Protect the entire SD/SPI bus from concurrent access (WebUI/config)
      if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        // SD busy for too long: do not crash
        continue;
      }
      // Re-check state INSIDE the lock (it might have changed)
      if (!loggingActive || !rawFile || !rawFile.isOpen()) {
        xSemaphoreGive(sdMutex);
        continue;
      }
      size_t written = rawFile.write(packet. data, packet.len);
      if (written > 0) {
        bytesSinceSync += written;
      }
      // Controlled flush: drastically reduces the risk of 0-byte files after reboot
      if (RAW_LOG_SYNC_MS || RAW_LOG_SYNC_BYTES) {
        const uint32_t now = millis();
        const UBaseType_t waiting = uxQueueMessagesWaiting(sdQueue);
        const bool queueOk = (waiting < (QUEUE_SIZE / 4));
        const bool timeDue = (RAW_LOG_SYNC_MS && (now - lastSyncMs >= RAW_LOG_SYNC_MS));
        const bool bytesDue = (RAW_LOG_SYNC_BYTES && (bytesSinceSync >= RAW_LOG_SYNC_BYTES));
        if (queueOk && (timeDue || bytesDue)) {
          rawFile.sync();
          lastSyncMs = now;
          bytesSinceSync = 0;
        }
      }
      xSemaphoreGive(sdMutex);
    }
  }
}

// RAW reader from UART2 (ZED UART1 TX → ESP RX RAW)
// => ONLY RAW logging and RAW viewer (no caster/TCP here)
void uartReaderTask(void* pvParameters) {
  uint8_t buffer[PACKET_SIZE];
  while (true) {
    int len = uart_read_bytes(UART_NUM_2, buffer, PACKET_SIZE, pdMS_TO_TICKS(10));
    if (len > 0) {
      if (loggingActive) {
        GNSSPacket packet;
        memcpy(packet.data, buffer, len); 
        packet.len = len;
        BaseType_t result = xQueueSend(sdQueue, &packet, pdMS_TO_TICKS(50));
        if (result != pdTRUE) {
          static uint32_t lastWarn = 0;
          if (millis() - lastWarn > 5000) {
            Serial.println("[UART] WARNING: SD Queue full!");
            lastWarn = millis();
          }
        }
      }
      if (isRawMode()) {
        TcpStreamer::broadcast(buffer, len);
        if (g_tcpClientOn) {
          TcpClientStreamer::broadcast(buffer, len);
        }
      }
    }
  }
}

// ---------------- Controls & utils ----------------

// Forward declarations for BLE RTCM (defined further below)
bool startBleRtcm(const String& targetName, uint32_t passkey);
void stopBleRtcm();

// (PATCH) Mutual exclusion: enabling NTRIP disables TCP-IN and vice versa
// === FIX: Thread-safe version with mutex ===
void toggleNtrip(bool enable) {
    // Mutual exclusion: stop TCP-IN first if enabling NTRIP
    // This is done BEFORE taking the NTRIP lock to avoid unlock/relock
    if (enable) {
        toggleTcpIn(false);
        if (g_bleRtcmEnabled) stopBleRtcm();
    }
    if (! ntripLock(500)) {
        oledPrintln("[NTRIP] busy, try again");
        return;
    }

    if (enable && !ntripEnabled) {
        if (ntripClient) {
            ntripClient->begin(RTCMSerial);
            ntripEnabled = true;
            oledSetNtrip(true);
            oledPrintln("[NTRIP] Enabled");
            // Reset RTCM stats on profile change
            resetRtcmStats();
            // Log NTRIP arm/activation
            if (g_systemLog) {
                g_systemLog->logEvent("NTRIP", String("Enabled profile ") + mountpoint + " @ " + ntrip_host + ":" + ntrip_port);
            }
        } else {
            oledPrintln("[NTRIP] Client not configured");
        }
    }
    else if (!enable && ntripEnabled) {
        ntripEnabled = false;
        
        if (ntripClient) {
            ntripClient->stop();
        }
        oledSetNtrip(false);
        oledPrintln("[NTRIP] Disabled");
        // Log NTRIP disconnection
        if (g_systemLog) {
            g_systemLog->logEvent("NTRIP", "Disconnected");
        }
    }
    ntripUnlock();
}

bool startTcpIn(const String& host, int port) {
  if (! tcpInLock(1000)) {
    oledPrintln("[LAN IN] busy, try again");
    return false;
  }
  if (g_tcpIn) {
    g_tcpIn->stop();
    delete g_tcpIn;
    g_tcpIn = nullptr;
  }
  g_tcpIn = new RTCMInput(host. c_str(), (uint16_t)port);
  if (!g_tcpIn) {
    tcpInUnlock();
    oledPrintln("[LAN IN] Allocation error");
    return false;
  }
  g_tcpIn->begin(RTCMSerial);
  tcpInEnabled = true;
  tcpInUnlock();
  oledPrintln(String("[LAN IN] TCP -> ") + host + ":" + String(port));
  return true;
}

void stopTcpIn() {
  if (!tcpInLock(1000)) {
    oledPrintln("[LAN IN] busy");
    return;
  }

  tcpInEnabled = false;
  
  if (g_tcpIn) {
    g_tcpIn->stop();
    delete g_tcpIn;
    g_tcpIn = nullptr;
  }
  tcpInUnlock();
  oledPrintln("[LAN IN] stopped");
}

void toggleTcpIn(bool enable) {
  if (enable) {
    // Mutual exclusion: stop competing sources BEFORE acquiring tcpInMutex
    // (canonical lock order: always stop competitor before taking own mutex)
    toggleNtrip(false);
    if (g_bleRtcmEnabled) stopBleRtcm();
    if (tcpin_host.length() && tcpin_port > 0) {
      startTcpIn(tcpin_host, tcpin_port);
    } else {
      oledPrintln("[LAN IN] Host/port not configured");
    }
  } else {
    // Only if actually active, to avoid unnecessary locks
    if (tcpInEnabled || g_tcpIn) {
      stopTcpIn();
    }
  }
}

// ---- BLE RTCM IN (correction source from rtcm-lora radio) ----

bool startBleRtcm(const String& targetName, uint32_t passkey) {
  // Mutual exclusion: disable other correction sources
  toggleNtrip(false);
  toggleTcpIn(false);

  strncpy(g_bleRtcmTargetName, targetName.c_str(), sizeof(g_bleRtcmTargetName) - 1);
  g_bleRtcmTargetName[sizeof(g_bleRtcmTargetName) - 1] = '\0';
  g_bleRtcmPasskey = passkey;

  g_bleRtcm.setRxCallback([](const uint8_t* data, size_t len) {
    if (len > 0) RTCMSerial.write(data, len);
  });

  if (!g_bleRtcm.begin(g_bleRtcmTargetName, g_bleRtcmPasskey)) {
    oledPrintln("[BLE-RTCM] Start failed");
    return false;
  }

  g_bleRtcmEnabled = true;
  oledPrintln(String("[BLE-RTCM] -> ") + g_bleRtcmTargetName);

  // Save config to flash
  FlashConfig::writeFile("/config/ble_rtcm_target.txt", String(g_bleRtcmTargetName));
  char pinBuf[8]; snprintf(pinBuf, sizeof(pinBuf), "%06u", g_bleRtcmPasskey);
  FlashConfig::writeFile("/config/ble_rtcm_pin.txt", String(pinBuf));
  FlashConfig::writeFile("/config/ble_rtcm_enabled.txt", "1");

  return true;
}

void stopBleRtcm() {
  g_bleRtcm.stop();  // sends STOP command before disconnecting
  g_bleRtcmEnabled = false;
  FlashConfig::writeFile("/config/ble_rtcm_enabled.txt", "0");
  oledPrintln("[BLE-RTCM] Stopped");
}

void toggleBleRtcm(bool enable) {
  if (enable) {
    if (String(g_bleRtcmTargetName).length() > 0) {
      startBleRtcm(String(g_bleRtcmTargetName), g_bleRtcmPasskey);
    } else {
      oledPrintln("[BLE-RTCM] Target not set");
    }
  } else {
    if (g_bleRtcmEnabled) {
      stopBleRtcm();
    }
  }
}

// ---- ESP-NOW RTCM mesh ----

bool startEspNowRx() {
  // Mutual exclusion: only one RTCM source active at a time
  toggleNtrip(false);
  toggleTcpIn(false);
  if (g_bleRtcmEnabled) stopBleRtcm();

  uint8_t ch = (uint8_t)WiFi.channel();
  if (ch == 0) ch = ESPNOW_WIFI_CHANNEL;

  g_espNow.onRtcmReceived = [](const uint8_t* data, size_t len) {
    if (len > 0) RTCMSerial.write(data, len);
  };

  g_espNow.onTelemReceived = [](const EspNowTelemPacket& pkt) {
    // Discover available relays: track the relay with best RSSI.
    // node_role == 2 means relay.
    // We prefer idle relays (relay_for_node_id == 0) or relays already serving us.
    if (pkt.node_role == 2) {
      const uint16_t our_id = g_espNow.getNodeId();
      // Accept this relay if: no relay selected yet, OR this one has better RSSI,
      // OR this is already our relay (keep tracking it)
      const bool is_our_relay = (g_espNowRelayNodeId == pkt.node_id);
      const bool is_available = (pkt.relay_for_node_id == 0 || pkt.relay_for_node_id == our_id);
      EspNowRtcm::PeerInfo* existing = (g_espNowRelayNodeId != 0) ? g_espNow.findOrAddPeer(g_espNowRelayNodeId) : nullptr;
      const bool better_rssi = (g_espNowRelayNodeId == 0) ||
                                (is_available && pkt.last_rssi > (existing ? existing->last_rssi : -120));
      if (is_our_relay || (is_available && better_rssi)) {
        g_espNowRelayNodeId = pkt.node_id;
      }
    }
  };

  g_espNow.onCommandReceived = nullptr;

  if (!g_espNow.begin(ch)) {
    oledPrintln("[ESPNOW] Init failed");
    return false;
  }
  g_espNowEnabled   = true;
  g_espNowTxEnabled = false;
  FlashConfig::writeFile("/config/espnow_enabled.txt", "1");
  FlashConfig::writeFile("/config/espnow_role.txt", "rx");
  oledPrintln("[ESPNOW] RX rover attivo");
  return true;
}

void stopEspNowRx() {
  g_espNowEnabled   = false;
  g_espNowTxEnabled = false;
  g_espNow.stop();
  FlashConfig::writeFile("/config/espnow_enabled.txt", "0");
  g_espNowRelayNodeId  = 0;
  g_espNowRelayLastReq = 0;
  oledPrintln("[ESPNOW] RX stop");
}

bool startEspNowTx() {
  uint8_t ch = (uint8_t)WiFi.channel();
  if (ch == 0) ch = ESPNOW_WIFI_CHANNEL;

  g_espNow.onCommandReceived = [](uint8_t cmd, uint8_t param, uint16_t src) {
    Serial.printf("[ESPNOW] CMD 0x%02X param=%u src=0x%04X\n", cmd, param, src);
    switch (cmd) {
      case CMD_REBOOT:
        oledPrintln("[ESPNOW] CMD: REBOOT");
        delay(500);
        ESP.restart();
        break;
      case CMD_ZED_RESET_HOT:
        oledPrintln("[ESPNOW] CMD: ZED hot reset");
        UbxVal::resetZed(false);
        break;
      case CMD_ZED_RESET_COLD:
        oledPrintln("[ESPNOW] CMD: ZED cold reset");
        UbxVal::resetZed(true);
        break;
      case CMD_LOG_START:
        startLogging();
        break;
      case CMD_LOG_STOP:
        stopLogging();
        break;
      case CMD_BASE_STOP:
        stopBaseMode();
        break;
      case CMD_ESPNOW_STOP:
        stopEspNowTx();
        break;
      case CMD_STATUS_REQ:
        g_espNowLastTelem = 0;  // force immediate telemetry send
        break;
      default:
        Serial.printf("[ESPNOW] Unknown CMD 0x%02X\n", cmd);
        break;
    }
  };

  if (!g_espNow.begin(ch)) {
    oledPrintln("[ESPNOW] TX init failed");
    return false;
  }
  g_espNowEnabled   = true;
  g_espNowTxEnabled = true;
  FlashConfig::writeFile("/config/espnow_enabled.txt", "1");
  FlashConfig::writeFile("/config/espnow_role.txt", "tx");
  oledPrintln("[ESPNOW] TX base attivo");
  return true;
}

void stopEspNowTx() {
  g_espNowEnabled   = false;
  g_espNowTxEnabled = false;
  g_espNow.stop();
  FlashConfig::writeFile("/config/espnow_enabled.txt", "0");
  oledPrintln("[ESPNOW] TX stop");
}

static bool connectAnyWifi(const std::vector<WifiCred>& list, uint32_t perNetTimeoutMs = 8000) {
  if (list.empty()) return false;
  WiFi.mode(WIFI_STA);

  // ---- Phase 1: Scan ----
  oledPrintln("[WiFi] Scanning...");
  int n = WiFi.scanNetworks(false, false, false, 300);  // sync, no hidden, no passive, 300ms/ch

  if (n > 0) {
    Serial.printf("[WiFi] Scan found %d networks\n", n);

    // Find the best match: saved network visible with lowest priority number
    const WifiCred* best = nullptr;
    int bestRSSI = -999;

    for (const auto& saved : list) {
      for (int i = 0; i < n; i++) {
        if (saved.ssid.equalsIgnoreCase(WiFi.SSID(i))) {
          // Found a match! If priority is better (lower number), or same priority but stronger signal
          if (!best || saved.priority < best->priority ||
              (saved.priority == best->priority && WiFi.RSSI(i) > bestRSSI)) {
            best = &saved;
            bestRSSI = WiFi.RSSI(i);
          }
          break;  // this saved network is found, check next saved
        }
      }
    }

    WiFi.scanDelete();  // free scan results memory

    if (best) {
      oledPrintln(String("[WiFi] Found: ") + best->ssid + " (" + String(bestRSSI) + "dBm)");
      Serial.printf("[WiFi] Best match: %s (prio=%d, RSSI=%d)\n",
                    best->ssid.c_str(), best->priority, bestRSSI);
      WiFi.begin(best->ssid.c_str(), best->password.c_str());
      uint32_t t0 = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - t0) < perNetTimeoutMs) delay(200);
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] Connected to %s in %lums\n", best->ssid.c_str(), millis() - t0);
        return true;
      }
      WiFi.disconnect(true, true); delay(200);
      Serial.printf("[WiFi] Failed to connect to %s despite being visible\n", best->ssid.c_str());
    } else {
      oledPrintln("[WiFi] No known network found");
      Serial.println("[WiFi] Scan: no saved networks visible");
    }
  } else {
    Serial.printf("[WiFi] Scan returned %d (failed or empty)\n", n);
    WiFi.scanDelete();
  }

  // ---- Phase 2: Fallback — try first 2 networks sequentially ----
  // (in case scan missed something, e.g. hidden networks)
  oledPrintln("[WiFi] Fallback...");
  int fallbackCount = min((int)list.size(), 2);
  for (int i = 0; i < fallbackCount; i++) {
    const auto& w = list[i];
    oledPrintln(String("[WiFi] Trying: ") + w.ssid);
    WiFi.begin(w.ssid.c_str(), w.password.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < perNetTimeoutMs) delay(200);
    if (WiFi.status() == WL_CONNECTED) return true;
    WiFi.disconnect(true, true); delay(200);
  }

  return false;
}

static bool fileExistsLocked(SdFat& sd, const char* path) {
  FsFile f = sd.open(path, FILE_READ);
  if (!f) return false;
  f.close();
  return true;
}

static void ensureGnssDirLocked(SdFat& sd) { sd.mkdir("/gnss"); }

static void ensureWifiFileExists(SdFat& sd) {
  if (!sdMutex) return;
  bool locked = (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE);
  if (!locked) { oledPrintln("SD busy"); return; }
  if (fileExistsLocked(sd, "/gnss/wifi.txt")) { xSemaphoreGive(sdMutex); return; }
  ensureGnssDirLocked(sd);
  FsFile f = sd.open("/gnss/wifi.txt", O_WRITE | O_CREAT | O_TRUNC);
  if (!f) { xSemaphoreGive(sdMutex); oledPrintln("Impossible create /gnss/wifi.txt"); return; }
  f.println("# RTKino Wi-Fi profiles"); f.println("# priority;ssid;password");
  f.print("1;"); f.print(DEFAULT_WIFI_SSID); f.print(";"); f.println(DEFAULT_WIFI_PASSWORD);
  f.close();
  xSemaphoreGive(sdMutex);
  oledPrintln("Created /gnss/wifi.txt");
}

// --- AP mode helper ---
static void startApMode() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, ESPNOW_WIFI_CHANNEL);
  IPAddress ip = WiFi.softAPIP();
  g_apMode = true;
  g_apStartMillis = millis();
  oledPrintln(ok ? "[WiFi] AP started" : "[WiFi] AP failed");
  oledPrintln(String("SSID: ") + AP_SSID);
  oledPrintln(String("IP: ") + ip.toString());

  // In AP server and viewer must run as in STA
  TcpStreamer::enable(true);
  TcpStreamer::begin();
  WebUI:: begin(sd, server);
  applyMdnsHostname(g_mdnsName);

  // OSD:  Wi-Fi on, AP IP, no NTRIP
  oledSetWifi(true);
  oledSetApMode(true);    // distinguish AP from STA on OLED
  oledSetIP(ip. toString());
  oledSetNtrip(false);
  silentMode = false; oledSetSilent(false);

  // To run server/ntrip/tcp in loop(), consider "wifiAvailable" true
  wifiAvailable = true;
}

void applyTimezone() {
  setenv("TZ", g_ntpTz, 1);
  tzset();
}

static bool loadNtpTz(char* out, size_t outSize) {
  return FlashConfig::readFileToBuffer("/config/tz.txt", out, outSize);
}

static bool loadNtpServer(char* out, size_t outSize) {
  return FlashConfig::readFileToBuffer("/config/ntp.txt", out, outSize);
}

static bool loadMdnsName(char* out, size_t outSize) {
  if (!out || outSize < 2) return false;
  String content = FlashConfig::readFile("/config/mdns.txt");
  content.trim();
  if (content.endsWith(".local")) content = content.substring(0, content.length() - 6);
  content.toLowerCase();
  if (content.length() < 1) return false;
  strncpy(out, content.c_str(), outSize - 1);
  out[outSize - 1] = '\0';
  return true;
}

// Start (or restart) mDNS with the configured hostname.
// Safe to call multiple times.
bool applyMdnsHostname(const char* hostname) {
  if (!hostname || !hostname[0]) return false;

  // Only meaningful when WiFi is up (STA or AP)
  wifi_mode_t mode = WiFi.getMode();
  if (mode != WIFI_STA && mode != WIFI_AP && mode != WIFI_AP_STA) return false;

  MDNS.end();
  delay(10);

  if (!MDNS.begin(hostname)) {
    Serial.printf("[mDNS] Failed to start for '%s'\n", hostname);
    return false;
  }

  MDNS.addService("http", "tcp", 80);
  Serial.printf("[mDNS] Started: http://%s.local/\n", hostname);
  return true;
}

// ===== BLE name management =====
bool loadBleName(char* out, size_t maxLen) {
  String content = FlashConfig::readFile("/config/ble_name.txt");
  content.trim();
  if (content.length() == 0 || content.length() > 20) return false;
  strncpy(out, content.c_str(), maxLen - 1);
  out[maxLen - 1] = '\0';
  return true;
}

bool saveBleName(const char* name) {
  if (!name || strlen(name) == 0 || strlen(name) > 20) return false;
  bool ok = FlashConfig::writeFile("/config/ble_name.txt", String(name));
  if (ok) FlashConfig::markDirty();
  return ok;
}

// Load BLE PIN from flash
bool loadBlePin(uint32_t* out) {
  String content = FlashConfig::readFile("/config/ble_pin.txt");
  content.trim();
  if (content.length() == 0) {
    *out = 123456;
    return false;
  }
  uint32_t pin = (uint32_t)content.toInt();
  if (pin <= 999999) {
    *out = pin;
    return true;
  }
  *out = 123456;
  return false;
}

// Save BLE PIN to flash
bool saveBlePin(uint32_t pin) {
  if (pin > 999999) return false;
  char buf[16];
  snprintf(buf, sizeof(buf), "%06u\n", pin);
  bool ok = FlashConfig::writeFile("/config/ble_pin.txt", String(buf));
  if (ok) FlashConfig::markDirty();
  return ok;
}

bool applyBleName(const char* newName) {
  if (!newName || strlen(newName) == 0 || strlen(newName) > 20) return false;
  
  strncpy(g_bleDeviceName, newName, sizeof(g_bleDeviceName) - 1);
  g_bleDeviceName[sizeof(g_bleDeviceName) - 1] = 0;
  
  Serial.printf("[BLE] Device name updated: %s\n", g_bleDeviceName);
  return true;
}

bool syncTimeFromNtp(const char* server) {
  if (!server || !server[0]) return false;
  // Use local timezone rules via TZ; configTime expects UTC offsets, but TZ handles localtime().
  configTzTime(g_ntpTz, server);
  tzset();
  struct tm tmNow;
  // wait up to ~3 seconds for SNTP to set time
  if (! getLocalTime(&tmNow, 3000)) return false;
  time_t nowEpoch = time(nullptr);
  if (nowEpoch < 1700000000) return false; // sanity (>= ~2023)
  g_timeSource = TIME_SRC_NTP;
  g_lastSyncEpoch = nowEpoch;
  return true;
}

static bool syncTimeFromUbxTimeUtc(uint16_t year, uint8_t month, uint8_t day,
                                  uint8_t hour, uint8_t min, uint8_t sec,
                                  uint8_t validFlags) {
  // UBX-NAV-TIMEUTC:  validUTC bit is 0x04 (per u-blox spec). We require it. 
  if ((validFlags & 0x04) == 0) return false;
  if (year < 2020 || month < 1 || month > 12 || day < 1 || day > 31) return false;
  if (hour > 23 || min > 59 || sec > 60) return false;
  struct tm t {};
  t.tm_year = (int)year - 1900;
  t.tm_mon  = (int)month - 1;
  t.tm_mday = (int)day;
  t.tm_hour = (int)hour;
  t.tm_min  = (int)min;
  t.tm_sec  = (int)sec;

  // TIMEUTC fields are UTC; convert to epoch (UTC).
  // Use timegm if available; otherwise temporarily force TZ=UTC for mktime.
  time_t epochUtc = 0;
#if defined(__USE_BSD) || defined(__USE_GNU)
  epochUtc = timegm(&t);
#else
  // NAV-TIMEUTC fields are UTC — temporarily set TZ to UTC for mktime()
  char* oldTz = getenv("TZ");
  setenv("TZ", "UTC0", 1);
  tzset();
  epochUtc = mktime(&t);
  // Restore configured timezone
  applyTimezone();
#endif
  if (epochUtc < 1700000000) return false;
  struct timeval tv;
  tv.tv_sec = epochUtc;
  tv. tv_usec = 0;
  settimeofday(&tv, nullptr);
  g_timeSource = TIME_SRC_TIMEUTC;
  g_lastSyncEpoch = time(nullptr);
  return true;
}

static bool makeTimestampedLogFilename(char* out, size_t outSize) {
  if (! out || outSize < 16) return false;

  time_t nowEpoch = time(nullptr);
  if (nowEpoch < 1700000000) return false; // not synced

  struct tm lt {};
  localtime_r(&nowEpoch, &lt);

  // log_YYYYMMDD_HHMMSS. ubx
  snprintf(out, outSize, "/gnss/log_%04d%02d%02d_%02d%02d%02d.ubx",
           lt. tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
           lt.tm_hour, lt.tm_min, lt. tm_sec);
  return true;
}

// ======= BASE:  wrapper for VALSET =======
// DF003 FIRST, then reset pipe and MSGOUT
// rtcmType: 0=MSM7 4 costellazioni (default), 1=MSM4 3 costellazioni
void applyBaseValset(uint16_t stid /*=1*/, uint8_t rtcmType /*=0*/) {
  // 1) Station ID FIRST
  UbxVal:: setStationID(stid);
  // 2) Reset pipe:  turn everything off -> turn on only RTCM3
  UbxVal::setFlag(UbxVal::Keys::UART2OUT_RTCM3, false);
  UbxVal::setFlag(UbxVal::Keys::UART2OUT_UBX,   false);
  UbxVal::setFlag(UbxVal::Keys::UART2OUT_NMEA,  false);
  delay(30);
  UbxVal::setFlag(UbxVal::Keys::UART2OUT_RTCM3, true);

  // 3) MSGOUT coherent (1005/1230 at 10 s; MSM7 or MSM4 at 1 s)
  UbxVal::setBaseMsgout(rtcmType);
  String msgType = (rtcmType == 0) ? "MSM7 4const" : "MSM4 3const";
  oledPrintln(String("[BASE] DF003=") + stid + " " + msgType);

  // Read back TMODE state to confirm (matches applyBaseFixedLLH pattern)
  delay(100);  // Give ZED time to apply settings
  getZedTmode(g_zedTmode);
}

void applyBaseFixedLLH(double lat_deg, double lon_deg, double h_m, uint16_t stid /*=1*/, uint8_t rtcmType /*=0*/) {
  // FIRST: disable NTRIP IN (no point receiving corrections in base mode)
  if (ntripEnabled) {
    toggleNtrip(false);
    oledPrintln("[BASE] NTRIP IN disabled");
  }
  // 1) Station ID FIRST
  UbxVal::setStationID(stid);

  // 2) TMODE (LLH+HP)
  UbxVal::setTmodeFixedLLH_HP(lat_deg, lon_deg, h_m, 200 /*=20. 0 mm*/);
  oledPrintln("[BASE] TMODE FIXED LLH (HP)");

  // 3) Reset pipe and re-apply MSGOUT
  UbxVal::setFlag(UbxVal::Keys::UART2OUT_RTCM3, false);
  UbxVal::setFlag(UbxVal::Keys::UART2OUT_UBX,   false);
  UbxVal::setFlag(UbxVal::Keys:: UART2OUT_NMEA,  false);
  delay(30);
  UbxVal::setFlag(UbxVal::Keys::UART2OUT_RTCM3, true);
  UbxVal::setBaseMsgout(rtcmType);
  String msgType = (rtcmType == 0) ? "MSM7" : "MSM4";
  oledPrintln(String("[BASE] DF003=") + stid + " " + msgType + " ready");
  
  // Read back TMODE state to confirm
  delay(100);  // Give ZED time to apply settings
  getZedTmode(g_zedTmode);
}

// Stop base mode and return to rover
void stopBaseMode() {
  // Disable TMODE → return to rover mode
  UbxVal::setU1(0x20030001, 0);  // CFG_TMODE_MODE = 0 (Disabled)
  delay(100);
  getZedTmode(g_zedTmode);  // Refresh state
  oledPrintln("[BASE] TMODE disabled - Rover mode");
}

// Read TMODE state from ZED-F9P
void readZedTmode() {
  ZedTmodeState temp;
  if (UbxVal::getTmodeState(temp, ZED_I2C_ADDR)) {
    if (zedTmodeLock(100)) {
      g_zedTmode = temp;
      zedTmodeUnlock();
      Serial.printf("[TMODE] Read successful: mode=%d, lat=%.8f, lon=%.8f, h=%.3f\n", 
                    temp.mode, temp.lat, temp.lon, temp.height);
    }
  } else {
    Serial.println("[TMODE] Read failed");
  }
}

// Start/stop caster out and tcp out
bool startCasterOut(const String& host, uint16_t port, const String& mount, const String& pass) {
  if (!pusherLock(1000)) {
    oledPrintln("[CASTER OUT] busy, riprova");
    return false;
  }
  g_outHost = host;
  g_outPort = port;
  g_outMount = mount;
  g_outPass = pass;
  if (g_pusher) {
    g_pusher->stop();
    delete g_pusher;
    g_pusher = nullptr;
  }
  g_pusher = new NtripPusher(host, (int)port, mount, pass);
  if (!g_pusher) {
    g_baseCasterOn = false;
    pusherUnlock();
    oledPrintln("[CASTER OUT] Allocation error");
    return false;
  }

  g_baseCasterOn = g_pusher->begin();
  pusherUnlock();
  if (g_baseCasterOn) {
    oledPrintln("[CASTER OUT] Connected");
  } else {
    oledPrintln("[CASTER OUT] Connection failed");
  }
  return g_baseCasterOn;
}

void stopCasterOut() {
  if (! pusherLock(1000)) {
    oledPrintln("[CASTER OUT] busy");
    return;
  }
  g_baseCasterOn = false;
  if (g_pusher) {
    g_pusher->stop();
    delete g_pusher;
    g_pusher = nullptr;
  }
  pusherUnlock();
  oledPrintln("[CASTER OUT] Stopped");
}

bool startTcpOut(uint16_t port){
  g_tcpPort = port;
  RtcmStreamer::begin(port);
  RtcmStreamer::setActive(true);
  g_baseTcpOn = true;
  return true;
}

void stopTcpOut(){
  RtcmStreamer::stop();
  RtcmStreamer::setActive(false);
  g_baseTcpOn = false;
}

bool startTcpOutClient(const String& host, uint16_t port){
  g_tcpClientHost = host;
  g_tcpClientPort = port;
  
  bool ok = TcpClientStreamer::begin(host, port);
  if(ok){
    TcpClientStreamer::enable(true);
    g_tcpClientOn = true;
    oledPrintln("[TCP-CLIENT] Connected to " + host);
  } else {
    g_tcpClientOn = false;
    oledPrintln("[TCP-CLIENT] Connection failed");
  }
  return ok;
}

void stopTcpOutClient(){
  TcpClientStreamer::stop();
  TcpClientStreamer::enable(false);
  g_tcpClientOn = false;
  oledPrintln("[TCP-CLIENT] Stopped");
}
// Switch from Base mode back to Rover mode
// Called from WebUI /api/switchToRover endpoint
// MINIMAL FIX: Only fixes crash, doesn't modify anything else
void switchToRover() {
  Serial.println("[SWITCH] Starting Base->Rover transition");
  
  // 1. Stop all output streams FIRST (before ZED reset)
  if (g_baseCasterOn) {
    stopCasterOut();
  }
  if (g_baseTcpOn) {
    stopTcpOut();
  }
  if (g_tcpClientOn) {
    stopTcpOutClient();
  }
  
  // 2. Disable TcpStreamer temporarily (give broadcasts time to finish)
  bool wasStreamerActive = TcpStreamer::isEnabled();
  TcpStreamer::enable(false);
  delay(200); // Let ongoing operations complete
  
  // 3. Stop NTRIP IN if active
  if (ntripEnabled) {
    toggleNtrip(false);
    delay(100);
  }
  
  // 4. Flush UART buffer (discard pending data)
  while (GNSSSerial.available()) {
    GNSSSerial.read();
  }
  
  // 5. Wait for SD queue and tasks to settle
  delay(300);
  
  // 6. Hot Reset ZED-F9P
  Serial.println("[SWITCH] Resetting ZED...");
  UbxVal::resetZed(false);  // false = hot reset
  
  // 7. Wait for ZED restart (hot reset takes ~1 second)
  delay(1500);
  
  // 8. Clear UART buffer again (discard reset output)
  while (GNSSSerial.available()) {
    GNSSSerial.read();
  }
  
  // 9. Disable TMODE (return to rover mode)
  stopBaseMode();
  
  // 10. Re-enable TcpStreamer if it was active before
  if (wasStreamerActive) {
    TcpStreamer::enable(true);
  }
  
  // 11. Do NOT auto re-enable NTRIP IN here.
  // NTRIP remains manual after a return to rover mode.

  delay(500);        // lascia stabilizzare ZED post-reset
  readZedTmode();    // aggiorna cache con dati freschi e validi

  Serial.println("[SWITCH] Transition complete");
  oledPrintln("[MODE] Switched to Rover");
}

// ========== SURVEY LOGIC (Base Position Averaging) ==========
// Start a survey with given parameters
void startSurvey(uint32_t durationSec, float instrumentHeight, float arpOffset) {
  if (!surveyMutex) return;
  if (xSemaphoreTake(surveyMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  // Reset survey state
  g_survey.active = true;
  g_survey.completed = false;
  g_survey.startTime = millis();
  g_survey.durationMs = max(30000U, durationSec * 1000U);  // minimum 30 seconds
  g_survey.sampleCount = 0;
  g_survey.latSum = 0.0;
  g_survey.lonSum = 0.0;
  g_survey.altSum = 0.0;
  g_survey.latSqSum = 0.0;
  g_survey.lonSqSum = 0.0;
  g_survey.altSqSum = 0.0;
  g_survey.instrumentHeight = instrumentHeight;
  g_survey.arpOffset = arpOffset;
  
  xSemaphoreGive(surveyMutex);
  
  Serial.printf("[SURVEY] Started: duration=%us, height=%.3fm, arp=%.3fm\n", 
                durationSec, instrumentHeight, arpOffset);
}

// Stop/cancel the survey
void stopSurvey() {
  if (!surveyMutex) return;
   if (xSemaphoreTake(surveyMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    g_survey.active = false;
    g_survey.completed = false;  // manual stop, not a natural completion
    xSemaphoreGive(surveyMutex);
   Serial.println("[SURVEY] Stopped");
}

// Add a position sample to the running survey
void surveyAddSample() {
  if (!surveyMutex || !positionMutex) return;
    // Quick check without mutex (optimization)
  if (!g_survey.active) return;
    // Get current position
  GNSSPosition pos;
  if (!getPosition(pos)) return;
    // Only accept valid fixes (quality >= 1)
  if (pos.fixQuality < 1) return;
    // Lock survey state
  if (xSemaphoreTake(surveyMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    // Check if still active and not expired
  if (!g_survey.active) {
    xSemaphoreGive(surveyMutex);
    return;
  }
  
  uint32_t elapsed = millis() - g_survey.startTime;
  if (elapsed >= g_survey.durationMs) {
    // Survey complete - stop it
    g_survey.active = false;
    g_survey.completed = true;   // natural completion → results available
    xSemaphoreGive(surveyMutex);
    Serial.println("[SURVEY] Auto-stopped (duration reached)");
    return;
  }
  
  // Accumulate sample
  g_survey.latSum += pos.lat;
  g_survey.lonSum += pos.lon;
  g_survey.altSum += pos.alt;
  g_survey.latSqSum += pos.lat * pos.lat;
  g_survey.lonSqSum += pos.lon * pos.lon;
  g_survey.altSqSum += pos.alt * pos.alt;
  g_survey.sampleCount++;
    xSemaphoreGive(surveyMutex);
}

// Get current survey results (thread-safe)
struct SurveyResults {
  bool active;
  bool complete;
  uint32_t elapsed;
  uint32_t duration;
  uint32_t sampleCount;
  double latMean, lonMean, altMean;
  double latStdDev, lonStdDev, altStdDev;
  double altGround;  // altitude corrected to ground point
  float instrumentHeight;
  float arpOffset;
};

bool getSurveyResults(SurveyResults& out) {
  if (!surveyMutex) return false;
    if (xSemaphoreTake(surveyMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    out.active = g_survey.active;
  out.duration = g_survey.durationMs / 1000;
  out.sampleCount = g_survey.sampleCount;
  out.instrumentHeight = g_survey.instrumentHeight;
  out.arpOffset = g_survey.arpOffset;
    if (g_survey.active) {
    out.elapsed = (millis() - g_survey.startTime) / 1000;
    out.complete = (millis() - g_survey.startTime) >= g_survey.durationMs;
  } else {
    // Survey is inactive — check if it completed naturally
    out.elapsed = g_survey.completed ? (g_survey.durationMs / 1000) : 0;
    out.complete = g_survey.completed;
  }
  
  if (g_survey.sampleCount > 0) {
    // Calculate mean
    out.latMean = g_survey.latSum / g_survey.sampleCount;
    out.lonMean = g_survey.lonSum / g_survey.sampleCount;
    out.altMean = g_survey.altSum / g_survey.sampleCount;
    
    // Calculate standard deviation
    if (g_survey.sampleCount > 1) {
      double latVariance = (g_survey.latSqSum / g_survey.sampleCount) - (out.latMean * out.latMean);
      double lonVariance = (g_survey.lonSqSum / g_survey.sampleCount) - (out.lonMean * out.lonMean);
      double altVariance = (g_survey.altSqSum / g_survey.sampleCount) - (out.altMean * out.altMean);
      out.latStdDev = latVariance > 0 ? sqrt(latVariance) : 0.0;
      out.lonStdDev = lonVariance > 0 ? sqrt(lonVariance) : 0.0;
      out.altStdDev = altVariance > 0 ? sqrt(altVariance) : 0.0;
    } else {
      out.latStdDev = 0.0;
      out.lonStdDev = 0.0;
      out.altStdDev = 0.0;
    }
    
    // Calculate ground altitude: H_ground = H_gps - instrument_height - arp_offset
    out.altGround = out.altMean - g_survey.instrumentHeight - g_survey.arpOffset;
  } else {
    out.latMean = 0.0;
    out.lonMean = 0.0;
    out.altMean = 0.0;
    out.latStdDev = 0.0;
    out.lonStdDev = 0.0;
    out.altStdDev = 0.0;
    out.altGround = 0.0;
  }
  
  xSemaphoreGive(surveyMutex);
  return true;
}

// ---------------- WiFi Event Handling ----------------
// Force all network services to reconnect (called after WiFi recovery)
void forceReconnectAllServices() {
  Serial.println("[NET] WiFi back online, forcing service reconnection...");
  // NTRIP stays manual: when WiFi comes back we do NOT auto-reconnect to the caster here.
  // This protects WebUI/local services from bad caster settings or unstable WAN links.
  // Force NtripPusher reconnect  
  if (pusherLock(100)) {
    if (g_pusher) {
      g_pusher->forceReconnect();
    }
    pusherUnlock();
  }

  // Force TcpClientStreamer reconnect
  if (g_tcpClientOn) {
    TcpClientStreamer::forceReconnect();
  }

  // Force TCP IN reconnect
  if (tcpInLock(100)) {
    if (g_tcpIn) {
      g_tcpIn->forceReconnect();
    }
    tcpInUnlock();
  }
}

// WiFi event callback
void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] Disconnected!");
      if (g_systemLog) {
        g_systemLog->logEvent("WIFI", "Disconnected");
      }
      break;
      
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi] Reconnected! IP: %s\n", WiFi.localIP().toString().c_str());
      if (g_systemLog) {
        g_systemLog->logEvent("WIFI", String("Reconnected - IP: ") + WiFi.localIP().toString());
      }
      forceReconnectAllServices();
      applyMdnsHostname(g_mdnsName);  // restart mDNS after WiFi reconnect
      break;
      
    default:
      break;
  }
}

// ---------------- setup/loop ----------------

// ---------------------------------------------------------------------------
// Encoder/menu helper: BLE toggle (used by OledMenu callback)
// ---------------------------------------------------------------------------
#if ENC_CLK_GPIO > 0 && ENC_DT_GPIO > 0 && ENC_SW_GPIO > 0
static void menuToggleBLE(bool enable) {
  if (enable && !g_bleEnabled) {
    BLESerial::begin(g_bleDeviceName, g_blePasskey);
    g_bleEnabled = true;
    BLESerial::setRxCallback([](const uint8_t* data, size_t len) {
      if (len > 0) RTCMSerial.write(data, len);
    });
    Serial.printf("[BLE] Enabled via menu: %s\n", g_bleDeviceName);
  } else if (!enable && g_bleEnabled) {
    BLESerial::end();
    g_bleEnabled = false;
    Serial.println("[BLE] Disabled via menu");
  }
}

// ---------------------------------------------------------------------------
// Encoder/menu shared helper: read the N-th profile line from a flash config
// file that uses the LAST= header convention, parsing semicolon-separated
// fields. Returns false if the file is empty or no LAST= line is found.
//
// File format (ntrip_out_list.txt): name;host;port;mount;pass;tcpPort
// File format (tcp_out_client_list.txt): name;host;port
//
// Populates the provided String vector (fields) with all ';'-separated tokens
// of the active (LAST=N) line.
// ---------------------------------------------------------------------------
static bool menuLoadActiveProfile(const char* flashPath, std::vector<String>& fields) {
  fields.clear();
  String content = FlashConfig::readFile(flashPath);
  if (content.length() == 0) return false;

  int lastIdx = -1;
  int lineNum  = 0;
  int start    = 0;
  while (start < (int)content.length()) {
    int end = content.indexOf('\n', start);
    if (end < 0) end = content.length();
    String line = content.substring(start, end);
    line.trim();
    start = end + 1;
    if (!line.length()) continue;
    if (line.startsWith("#")) {
      int p = line.indexOf("LAST=");
      if (p >= 0) lastIdx = line.substring(p + 5).toInt();
      continue;
    }
    if (lastIdx >= 0 && lineNum == lastIdx) {
      // Split on ';' and collect all fields
      int pos = 0;
      while (pos <= (int)line.length()) {
        int sep = line.indexOf(';', pos);
        if (sep < 0) sep = line.length();
        fields.push_back(line.substring(pos, sep));
        pos = sep + 1;
      }
      return !fields.empty();
    }
    lineNum++;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Encoder/menu helper: start NTRIP OUT from active profile
// Profile format: name;host;port;mount;pass;tcpPort
// ---------------------------------------------------------------------------
static void menuToggleNtripOut(bool enable) {
  if (!enable) {
    stopCasterOut();
    stopTcpOut();
    return;
  }
  std::vector<String> f;
  if (!menuLoadActiveProfile("/config/ntrip_out_list.txt", f) || f.size() < 6) {
    oledPrintln("[NTRIP OUT] No profile selected");
    return;
  }
  // fields: [0]=name [1]=host [2]=port [3]=mount [4]=pass [5]=tcpPort
  startCasterOut(f[1], (uint16_t)f[2].toInt(), f[3], f[4]);
  uint16_t tcpPort = (uint16_t)f[5].toInt();
  if (tcpPort > 0) startTcpOut(tcpPort);
}

// ---------------------------------------------------------------------------
// Encoder/menu helper: start TCP OUT Srv from active NTRIP OUT profile (TCP port)
// Profile format: name;host;port;mount;pass;tcpPort
// ---------------------------------------------------------------------------
static void menuToggleTcpOutSrv(bool enable) {
  if (!enable) { stopTcpOut(); return; }
  std::vector<String> f;
  if (!menuLoadActiveProfile("/config/ntrip_out_list.txt", f) || f.size() < 6) {
    oledPrintln("[TCP OUT] No profile selected");
    return;
  }
  // fields: [0]=name [1]=host [2]=port [3]=mount [4]=pass [5]=tcpPort
  uint16_t tcpPort = (uint16_t)f[5].toInt();
  if (tcpPort > 0) startTcpOut(tcpPort);
  else oledPrintln("[TCP OUT] No port in profile");
}

// ---------------------------------------------------------------------------
// Encoder/menu helper: start TCP OUT Client from active profile
// Profile format: name;host;port
// ---------------------------------------------------------------------------
static void menuToggleTcpOutCli(bool enable) {
  if (!enable) { stopTcpOutClient(); return; }
  std::vector<String> f;
  if (!menuLoadActiveProfile("/config/tcp_out_client_list.txt", f) || f.size() < 3) {
    oledPrintln("[TCP CLI] No profile selected");
    return;
  }
  // fields: [0]=name [1]=host [2]=port
  startTcpOutClient(f[1], (uint16_t)f[2].toInt());
}

// ---------------------------------------------------------------------------
// Encoder/menu helper: build info string for info screen
// ---------------------------------------------------------------------------
static String menuGetInfoString() {
  char buf[128];
  uint32_t upSec = millis() / 1000;
  uint32_t h = upSec / 3600;
  uint32_t m = (upSec % 3600) / 60;
  uint32_t s = upSec % 60;
  String ip = WiFi.isConnected() ? WiFi.localIP().toString() : "---";
  snprintf(buf, sizeof(buf),
    "%s\nIP:%s\nUp:%02lu:%02lu:%02lu\nHeap:%lu",
    getBoardName(), ip.c_str(), h, m, s,
    (unsigned long)ESP.getFreeHeap());
  return String(buf);
}

// ---------------------------------------------------------------------------
// SurveyPoints: snapshot provider — called from measurement task
// ---------------------------------------------------------------------------
static void takeGNSSSnapshot(GNSSSnapshot& snap) {
  GNSSPosition pos;
  if (!getPosition(pos)) {
    memset(&snap, 0, sizeof(snap));
    return;
  }
  snap.fixMode   = pos.fixQuality;
  snap.carrSoln  = pos.carrSoln;
  snap.numSV     = pos.numSV;
  snap.lat       = pos.lat;
  snap.lon       = pos.lon;
  snap.altMSL    = pos.alt;
  snap.hAcc      = pos.hAcc;
  snap.vAcc      = pos.vAcc;
  snap.latHP     = pos.latHP;
  snap.lonHP     = pos.lonHP;
  snap.altHAE    = pos.altHAE;
  snap.altMSLHP  = pos.altMSLHP;
  snap.hAccHP    = pos.hAcc;   // HPPOSLLH hAcc is stored in pos.hAcc when hpAccValid
  snap.vAccHP    = pos.vAcc;
  snap.hpValid   = pos.hpAccValid;
  snap.ecefX     = pos.ecefX;
  snap.ecefY     = pos.ecefY;
  snap.ecefZ     = pos.ecefZ;
  snap.pAcc      = pos.pAcc;
  snap.ecefValid = pos.ecefValid;
  snap.gdop      = pos.gdop;
  snap.pdop      = pos.pdop;
  snap.hdop      = pos.hdop;
  snap.vdop      = pos.vdop;
  snap.ndop      = pos.ndop;
  snap.edop      = pos.edop;
  snap.tdop      = pos.tdop;
  snap.covNN     = pos.covNN;
  snap.covEE     = pos.covEE;
  snap.covDD     = pos.covDD;
  snap.covNE     = pos.covNE;
  snap.covND     = pos.covND;
  snap.covED     = pos.covED;
  snap.covValid  = pos.covValid;
  snap.relN      = pos.relN;
  snap.relE      = pos.relE;
  snap.relD      = pos.relD;
  snap.relSN     = pos.relSN;
  snap.relSE     = pos.relSE;
  snap.relSD     = pos.relSD;
  snap.relValid  = pos.relValid;
}

// ---------------------------------------------------------------------------
// Stakeout: position provider — returns current rover lat/lon/h + fix info
// ---------------------------------------------------------------------------
static void takeStakeoutPosition(double& lat, double& lon, double& h,
                                  uint8_t& carrSoln, uint8_t& fixQuality) {
  GNSSPosition pos;
  if (!getPosition(pos)) { lat = 0; lon = 0; h = 0; carrSoln = 0; fixQuality = 0; return; }
  // Use high-precision ellipsoidal height when available, else fall back to MSL alt
  lat         = pos.hpAccValid ? pos.latHP  : pos.lat;
  lon         = pos.hpAccValid ? pos.lonHP  : pos.lon;
  h           = pos.hpAccValid ? pos.altHAE : (double)pos.alt;
  carrSoln    = pos.carrSoln;
  fixQuality  = pos.fixQuality;
}

// Cached survey list for OLED menu
static std::vector<String> g_surveyIds;
static std::vector<String> g_surveyLabels;

static void refreshSurveyList() {
  g_surveyIds.clear();
  g_surveyLabels.clear();
  SurveyPoints::listSurveyIds(g_surveyIds);
  String activeSid = SurveyPoints::getActiveSurveyId();
  for (const String& sid : g_surveyIds) {
    int pts = SurveyPoints::getSurveyPointCount(sid);
    // Load title from JSON
    String json = SurveyPoints::loadSurveyJSON(sid);
    String title = sid;
    int ti = json.indexOf("\"title\":\"");
    if (ti >= 0) {
      int ts = ti + 9;
      int te = json.indexOf("\"", ts);
      if (te > ts) title = json.substring(ts, te);
      if (title.length() > 10) title = title.substring(0, 10);
    }
    String lbl = (sid == activeSid ? "*" : " ");
    lbl += title + "(" + String(pts) + ")";
    g_surveyLabels.push_back(lbl);
  }
}

// ---- Stakeout OLED cache ---------------------------------------------------
static std::vector<String> g_stakeoutFileIds;
static std::vector<String> g_stakeoutFileLabels;
static int                  g_stakeoutSelFileIdx = -1;
static std::vector<String>  g_stakeoutPointIds;
static std::vector<String>  g_stakeoutPointLabels;

static void refreshStakeoutFileList() {
  g_stakeoutFileIds.clear();
  g_stakeoutFileLabels.clear();
  std::vector<StakeoutFile> files;
  Stakeout::listFiles(files);
  StakeoutActive act = Stakeout::getActive();
  for (auto& sf : files) {
    g_stakeoutFileIds.push_back(sf.fileId);
    String lbl = (sf.fileId == act.fileId ? "*" : " ");
    lbl += sf.name;
    if (lbl.length() > 14) lbl = lbl.substring(0, 14);
    lbl += "(" + String(sf.count) + ")";
    g_stakeoutFileLabels.push_back(lbl);
  }
}

static void loadStakeoutFilePoints(int fileIdx) {
  g_stakeoutPointIds.clear();
  g_stakeoutPointLabels.clear();
  g_stakeoutSelFileIdx = fileIdx;
  if (fileIdx < 0 || fileIdx >= (int)g_stakeoutFileIds.size()) return;
  std::vector<StakeoutPoint> pts;
  Stakeout::loadFilePoints(g_stakeoutFileIds[fileIdx], pts);
  StakeoutActive act = Stakeout::getActive();
  for (auto& pt : pts) {
    g_stakeoutPointIds.push_back(pt.id);
    String lbl = (g_stakeoutFileIds[fileIdx] == act.fileId && pt.id == act.pointId ? "*" : " ");
    lbl += pt.name;
    // Append short HAE if available
    if (!isnan(pt.h)) {
      char hb[12]; snprintf(hb, sizeof(hb), " %.1f", pt.h);
      lbl += hb;
    }
    if (lbl.length() > 20) lbl = lbl.substring(0, 20);
    g_stakeoutPointLabels.push_back(lbl);
  }
}

#endif // ENC_CLK_GPIO > 0 && ...

void setup() {
  Serial.begin(115200);
  delay(300);
  // Apply default timezone (CET/CEST) for localtime() and filename formatting;
  // will be overridden after FlashConfig is loaded from flash.
  applyTimezone();

  // ===== MUTEX INITIALIZATION =====
  // SD/SPI non e' thread-safe:  usiamo un mutex globale per evitare accessi concorrenti
  sdMutex = xSemaphoreCreateMutex();
    // Mutex per NtripClient (FIX CRASH)
  ntripMutex = xSemaphoreCreateMutex();
   // Mutex per RTCMInput (TCP-IN)
  tcpInMutex = xSemaphoreCreateMutex();
   // Mutex per NtripPusher (caster OUT)
  pusherMutex = xSemaphoreCreateMutex();
   // Mutex per Position data
  positionMutex = xSemaphoreCreateMutex();
   // Mutex per RTCM statistics
  rtcmStatsMutex = xSemaphoreCreateMutex();
   // Mutex per Survey state
  surveyMutex = xSemaphoreCreateMutex();
   // Mutex per ZED TMODE state
  zedTmodeMutex = xSemaphoreCreateMutex();
  if (!sdMutex || !ntripMutex || !tcpInMutex || !pusherMutex || !positionMutex || !rtcmStatsMutex || !surveyMutex || !zedTmodeMutex) {
    Serial.println("FATAL: Failed to create mutexes!");
    while(1) delay(1000);
  }


  // Initialize I2C bus
  Wire.begin(ZED_I2C_SDA, ZED_I2C_SCL);
  Serial.printf("[I2C] Shared I2C (Wire) initialized on SDA=%d, SCL=%d (ZED + OLED)\n", ZED_I2C_SDA, ZED_I2C_SCL);
  
  oledInit();
  oledSplashLandscape();
  readCfgRateFromZED();
  oledSetLogging(false); oledSetWifi(false); oledSetApMode(false); oledSetNtrip(false); oledSetIP("---");

  // ---- Rotary encoder + OledMenu (compiled only when pins are configured) ----
#if ENC_CLK_GPIO > 0 && ENC_DT_GPIO > 0 && ENC_SW_GPIO > 0
  RotaryInput::init(ENC_CLK_GPIO, ENC_DT_GPIO, ENC_SW_GPIO);
  OledMenu::init();

  // Action callbacks
  OledMenu::onToggleBLE       = menuToggleBLE;
  OledMenu::onToggleNtripIn   = [](bool en) { toggleNtrip(en); };
  OledMenu::onToggleTcpIn     = [](bool en) { toggleTcpIn(en); };
  OledMenu::onToggleNtripOut  = menuToggleNtripOut;
  OledMenu::onToggleTcpOutSrv = menuToggleTcpOutSrv;
  OledMenu::onToggleTcpOutCli = menuToggleTcpOutCli;
  OledMenu::onToggleBuzzer    = [](bool en) { if (g_buzzer) g_buzzer->setEnabled(en); };
  OledMenu::onSetRate         = [](uint16_t ms) { sendCfgRateToZED(ms); };
  OledMenu::onReboot          = []() { ESP.restart(); };
  OledMenu::onSyncToSD        = []() {
    if (!loggingActive && sdOK) {
      FlashConfig::syncToSD(sd, sdMutex, true);
      SurveyPoints::syncToSD();
      Stakeout::syncToSD();
      Serial.println("[Menu] Manual sync to SD triggered from OLED menu");
    }
  };
  OledMenu::onToggleRawLog    = [](bool enable) {
    if (enable) startLogging();
    else        stopLogging();
  };

  // State readers
  OledMenu::getBleState       = []() -> bool        { return g_bleEnabled; };
  OledMenu::getNtripInState   = []() -> bool        { return ntripEnabled; };
  OledMenu::getTcpInState     = []() -> bool        { return tcpInEnabled; };
  OledMenu::getNtripOutState  = []() -> bool        { return (bool)g_baseCasterOn; };
  OledMenu::getTcpOutSrvState = []() -> bool        { return (bool)g_baseTcpOn; };
  OledMenu::getTcpOutCliState = []() -> bool        { return g_tcpClientOn; };
  OledMenu::getBuzzerState    = []() -> bool        { return g_buzzer ? g_buzzer->isEnabled() : false; };
  OledMenu::getRateString     = []() -> const char* { return lastRateSet.c_str(); };
  OledMenu::getInfoString     = []() -> String      { return menuGetInfoString(); };
  OledMenu::getRawLogState    = []() -> bool        { return loggingActive; };

  // Survey action callbacks
  OledMenu::onMeasurePoint = []() {
    MeasureParams mp;
    mp.name         = "";
    mp.durationSec  = 10.0f;
    mp.intervalSec  = 0.5f;
    mp.forceQuality = false;
    SurveyPoints::startMeasure(mp);
  };
  OledMenu::onForceMeasurePoint = []() {
    MeasureParams mp;
    mp.name         = "";
    mp.durationSec  = 10.0f;
    mp.intervalSec  = 0.5f;
    mp.forceQuality = true;
    SurveyPoints::startMeasure(mp);
  };
  OledMenu::onCreateSurvey = []() {
    time_t now = time(nullptr);
    char buf[24]; struct tm t; localtime_r(&now, &t);
    strftime(buf, sizeof(buf), "Survey %d/%m %H:%M", &t);
    String sid = SurveyPoints::createSurvey(String(buf));
    if (!sid.isEmpty()) {
      SurveyPoints::setActiveSurveyId(sid);
      refreshSurveyList();
      Serial.printf("[Menu] New survey created: %s\n", sid.c_str());
    }
  };
  OledMenu::onSetActiveSurvey = [](int idx) {
    if (idx >= 0 && idx < (int)g_surveyIds.size()) {
      SurveyPoints::setActiveSurveyId(g_surveyIds[idx]);
      refreshSurveyList();
    }
  };

  // Survey state readers
  OledMenu::isMeasuring           = []() -> bool   { return SurveyPoints::isMeasuring(); };
  OledMenu::getQualityOK          = []() -> bool   { return !SurveyPoints::checkQuality().anyBad(); };
  OledMenu::getMeasureProgressStr = []() -> String {
    MeasureProgress p = SurveyPoints::getMeasureProgress();
    char buf[64];
    snprintf(buf, sizeof(buf), "%d%%\n%d samples\nhAcc:%.4fm\n%.1fs",
             p.pct, p.nSamples, p.curHAcc, p.elapsed);
    return String(buf);
  };
  OledMenu::getMeasureResult = []() -> String {
    MeasureProgress p = SurveyPoints::getMeasureProgress();
    if (p.status == MS_DONE) {
      return "OK: " + p.lastPointId + "\n*=back";
    } else if (p.status == MS_ERROR) {
      return "ERROR:\n" + p.errorMsg;
    }
    return "---";
  };
  OledMenu::getSurveyCount = []() -> int {
    return (int)g_surveyIds.size();
  };
  OledMenu::getSurveyLabel = [](int idx) -> String {
    if (idx >= 0 && idx < (int)g_surveyLabels.size()) return g_surveyLabels[idx];
    return "";
  };

  // PointCodes callbacks
  OledMenu::getCodeCatCount = []() -> int    { return PointCodes::getCategoryCount(); };
  OledMenu::getCodeCatLabel = [](int i) -> String { return PointCodes::getCategoryLabel(i); };
  OledMenu::getCodeCount    = [](int i) -> int    { return PointCodes::getCodeCount(i); };
  OledMenu::getCodeCod      = [](int ci, int ki) -> String { return PointCodes::getCodeCod(ci, ki); };
  OledMenu::getCodeLabel    = [](int ci, int ki) -> String { return PointCodes::getCodeLabel(ci, ki); };

  OledMenu::onMeasureWithCode = [](const String& cod, const String& label, bool force) {
    MeasureParams mp;
    mp.name         = "";
    mp.codice       = cod;
    mp.desc         = label;
    mp.durationSec  = 10.0f;
    mp.intervalSec  = 0.5f;
    mp.forceQuality = force;
    SurveyPoints::startMeasure(mp);
  };

  // Stakeout action callbacks
  OledMenu::onSelectStakeoutFile = [](int idx) {
    loadStakeoutFilePoints(idx);
  };
  OledMenu::onSetStakeoutActive = [](int pointIdx) {
    if (g_stakeoutSelFileIdx >= 0 &&
        g_stakeoutSelFileIdx < (int)g_stakeoutFileIds.size() &&
        pointIdx >= 0 && pointIdx < (int)g_stakeoutPointIds.size()) {
      Stakeout::setActive(g_stakeoutFileIds[g_stakeoutSelFileIdx],
                          g_stakeoutPointIds[pointIdx]);
      refreshStakeoutFileList();
    }
  };

  // Stakeout state readers
  OledMenu::getStakeoutFileCount  = []() -> int    {
    if (Stakeout::isDirty()) { refreshStakeoutFileList(); Stakeout::clearDirty(); }
    return (int)g_stakeoutFileIds.size();
  };
  OledMenu::getStakeoutFileLabel  = [](int idx) -> String {
    if (idx >= 0 && idx < (int)g_stakeoutFileLabels.size()) return g_stakeoutFileLabels[idx];
    return "";
  };
  OledMenu::getStakeoutPointCount = []() -> int    { return (int)g_stakeoutPointIds.size(); };
  OledMenu::getStakeoutPointLabel = [](int idx) -> String {
    if (idx >= 0 && idx < (int)g_stakeoutPointLabels.size()) return g_stakeoutPointLabels[idx];
    return "";
  };
  OledMenu::getStakeoutNavString = []() -> String {
    StakeoutStatus st = Stakeout::getStatus();
    if (!st.valid) {
      // Still show fix status even when no target active
      const char* fixStr = "NO FIX";
      if (st.roverCarrSoln == 2)      fixStr = "RTK FIX";
      else if (st.roverCarrSoln == 1) fixStr = "FLOAT";
      return String(fixStr) + "\nNo target\nset active";
    }
    const char* fixStr = "NO FIX";
    if (st.roverCarrSoln == 2)      fixStr = "RTK FIX";
    else if (st.roverCarrSoln == 1) fixStr = "FLOAT";
    String name = st.targetName.isEmpty() ? st.targetId : st.targetName;
    if (name.length() > 12) name = name.substring(0, 12);
    char buf[128];
    if (isnan(st.dH)) {
      snprintf(buf, sizeof(buf), "%s\n%s\nD2D:%.3f Az:%.0f\xb0\ndH:---\nH:---",
               fixStr, name.c_str(), st.d2d, st.az);
    } else {
      char hTarget[20];
      if (isnan(st.targetH)) snprintf(hTarget, sizeof(hTarget), "---");
      else                    snprintf(hTarget, sizeof(hTarget), "%.2f", st.targetH);
      snprintf(buf, sizeof(buf), "%s\n%s\nD2D:%.3f Az:%.0f\xb0\ndH:%+.3fm\nH:%s",
               fixStr, name.c_str(), st.d2d, st.az, st.dH, hTarget);
    }
    return String(buf);
  };

  Serial.printf("[ENC] Rotary encoder on CLK=%d DT=%d SW=%d\n",
                ENC_CLK_GPIO, ENC_DT_GPIO, ENC_SW_GPIO);
#endif

  // Initialize SD card (SPI bus with explicit pins)
  Serial.printf("[SD] Initializing SD (CS=%d, SCK=%d, MOSI=%d, MISO=%d)\n", 
                SD_CS, SD_SCK, SD_MOSI, SD_MISO);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOK = sd.begin(SD_CS, SD_SCK_MHZ(25));
  
  if (!sdOK) { 
    oledPrintln("SD not found"); 
    Serial.printf("[SD] Failed! Board: %s\n", getBoardName());
  } else { 
    Serial.printf("[SD] OK! Board: %s, Flash: %s, PSRAM: %s\n", 
                  getBoardName(), getBoardFlash(), getBoardPsram());
  }

  // Initialize FlashConfig (LittleFS) with migration from SD
  FlashConfig::begin(sd, sdMutex, sdOK);
  Serial.printf("[Flash] LittleFS: %u/%u bytes used\n",
                (unsigned)FlashConfig::usedBytes(), (unsigned)FlashConfig::totalBytes());

  // Initialize SurveyPoints (uses LittleFS for storage)
  SurveyPoints::begin(sd, sdMutex, sdOK);
  SurveyPoints::snapshotProvider = takeGNSSSnapshot;

  // Initialize Stakeout (uses LittleFS for storage)
  Stakeout::begin(sd, sdMutex, sdOK);
  Stakeout::positionProvider = takeStakeoutPosition;

  // Initialize PointCodes (uses LittleFS, loads defaults if not present)
  PointCodes::begin();
#if ENC_CLK_GPIO > 0 && ENC_DT_GPIO > 0 && ENC_SW_GPIO > 0
  refreshSurveyList();
  refreshStakeoutFileList();
#endif

  // Sync flash→SD at boot to ensure SD always has an up-to-date copy
  if (sdOK) {
    FlashConfig::syncToSD(sd, sdMutex, true);
    SurveyPoints::syncToSD();
    Stakeout::syncToSD();
    Serial.println("[Flash] Boot sync flash→SD completed");
  }

  // ===== NEW: Initialize Buzzer and SystemLog =====
  // Buzzer init
  g_buzzer = new Buzzer(BUZZER_GPIO, BUZZER_LEDC_CHANNEL);
  if (g_buzzer && g_buzzer->begin()) {
    Serial.printf("[Buzzer] Initialized on GPIO %d\n", BUZZER_GPIO);
  } else {
    Serial.println("[Buzzer] Failed to initialize");
  }

  // EXTINT GPIO init (ZED-F9P external interrupt for PPK event marking)
  pinMode(EXTINT_GPIO, OUTPUT);
  digitalWrite(EXTINT_GPIO, LOW);

  // SystemLog init (requires SD)
  if (sdOK) {
    g_systemLog = new SystemLog();
    if (g_systemLog && g_systemLog->begin(sd, sdMutex)) {
      Serial.println("[SystemLog] Initialized");
      g_systemLog->logEvent("SYSTEM", "RTKino firmware started");
    } else {
      Serial.println("[SystemLog] Failed to initialize");
    }
  }

  // ===== Load BLE device name from flash (persistent) =====
  if (loadBleName(g_bleDeviceName, sizeof(g_bleDeviceName))) {
    Serial.printf("[BLE] Device name loaded: %s\n", g_bleDeviceName);
  } else {
    Serial.println("[BLE] Using default device name: RTKino");
  }

  // ===== Load BLE PIN from flash (persistent) =====
  if (loadBlePin(&g_blePasskey)) {
    Serial.printf("[BLE] PIN loaded: %06u\n", g_blePasskey);
  } else {
    Serial.printf("[BLE] Using default PIN: %06u\n", g_blePasskey);
  }

  // BLE is enabled by default at boot.
  // Note: RTCMSerial is initialized a few lines below; the RX callback is
  // event-driven and cannot fire until a BLE client connects, well after setup().
  BLESerial::begin(g_bleDeviceName, g_blePasskey);
  g_bleEnabled = true;
  BLESerial::setRxCallback([](const uint8_t* data, size_t len) {
    if (len > 0) RTCMSerial.write(data, len);
  });
  Serial.printf("[BLE] Started at boot: %s (PIN: %06u)\n", g_bleDeviceName, g_blePasskey);

  // ===== Load BLE RTCM config (target device name, PIN, enable state) =====
  {
    String tgt = FlashConfig::readFile("/config/ble_rtcm_target.txt");
    tgt.trim();
    if (tgt.length() > 0 && tgt.length() <= 20) {
      strncpy(g_bleRtcmTargetName, tgt.c_str(), sizeof(g_bleRtcmTargetName) - 1);
      g_bleRtcmTargetName[sizeof(g_bleRtcmTargetName) - 1] = '\0';
    }
    String pin = FlashConfig::readFile("/config/ble_rtcm_pin.txt");
    pin.trim();
    if (pin.length() == 6) {
      g_bleRtcmPasskey = (uint32_t)atoi(pin.c_str());
    }
    Serial.printf("[BLE-RTCM] Config: target='%s' pin=%06u\n",
                  g_bleRtcmTargetName, g_bleRtcmPasskey);
  }


  // UARTs
  GNSSSerial.setRxBufferSize(8192);
  GNSSSerial.setTimeout(20);
  GNSSSerial.begin(UART_BAUD, SERIAL_8N1, UBX_UART_RX, -1);    // RX:  ZED TX2 (NMEA/UBX o RTCM)
  RTCMSerial.begin(UART_BAUD, SERIAL_8N1, -1, RTCM_UART_TX);   // TX: ZED RX2 (RTCM in)

  uart_config_t raw_uart_config = { UART_BAUD, UART_DATA_8_BITS, UART_PARITY_DISABLE, UART_STOP_BITS_1, UART_HW_FLOWCTRL_DISABLE };
  uart_param_config(UART_NUM_2, &raw_uart_config);
  uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, RAW_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, 16384, 0, 0, NULL, 0);

  // Tasks
  sdQueue = xQueueCreate(QUEUE_SIZE, sizeof(GNSSPacket));
  xTaskCreatePinnedToCore(uartReaderTask, "UARTReader", 12288, NULL, 2, &uartTaskHandle, 0);
  xTaskCreatePinnedToCore(sdWriterTask,   "SDWriter",   4096,  NULL, 1, &sdTaskHandle, 1);
  xTaskCreatePinnedToCore(nmeaReaderTask, "NMEAReader", 6144,  NULL, 1, &nmeaTaskHandle, 1);

  // Wi-Fi + Web + NTRIP (rover IN)
  WifiProfiles::loadFromFlash(wifiList);
  loadNtpServer(g_ntpServer, sizeof(g_ntpServer));
  loadNtpTz(g_ntpTz, sizeof(g_ntpTz));
  applyTimezone();   // re-apply after loading persisted timezone from flash
  loadMdnsName(g_mdnsName, sizeof(g_mdnsName));
  if (wifiList.empty()) { WifiCred c{1, DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASSWORD}; wifiList.push_back(c); oledPrintln("[WiFi] Uso fallback:  " DEFAULT_WIFI_SSID); }
  WifiProfiles::sortByPriority(wifiList);

  // Register WiFi event handler for connection recovery
  WiFi.onEvent(onWiFiEvent);
  wifiAvailable = connectAnyWifi(wifiList);
  if (wifiAvailable) {
    oledSetWifi(true); oledSetApMode(false); oledSetIP(WiFi.localIP().toString()); silentMode=false; oledSetSilent(false);

    // Sync orario via NTP SOLO dopo connessione STA
    if (WiFi.status() == WL_CONNECTED) {
      g_apMode = false;
      syncTimeFromNtp(g_ntpServer);
      
      // Read initial TMODE state from ZED-F9P
      delay(100);
      readZedTmode();
    }



    // TCP streaming "viewer"

    TcpStreamer::enable(true);

    TcpStreamer::begin();



    // WebUI

    WebUI::begin(sd, server);

    // mDNS (http://<name>.local/)
    applyMdnsHostname(g_mdnsName);

    // Initialize OTA Manager (Web Upload only)
    if (OTAManager::begin()) {
      Serial.println("[OTA] OTA Manager initialized successfully");
    } else {
      Serial.println("[OTA] Failed to initialize OTA Manager");
    }



    // Carico eventuale ultimo profilo TCP-IN per memorizzare host/port (non autostart)

    String _h; int _p = 0;

    if (WebUI::loadTcpIn(sd, _h, _p)) { tcpin_host = _h; tcpin_port = _p; }



    // NTRIP IN (rover) - con lock
    if (! WebUI::loadNtrip(sd, ntrip_host, ntrip_port, mountpoint, ntrip_user, ntrip_pass)) {

      oledPrintln("NTRIP cfg not read");

    } else {
      if (ntripLock(1000)) {
        ntripClient = new NtripClient(ntrip_host.c_str(), ntrip_port, mountpoint.c_str(), ntrip_user.c_str(), ntrip_pass.c_str());
        ntripClient->setGgaMinPeriodMs(5000);
        // Manual policy: load profile, but do NOT auto-start NTRIP at boot.
        ntripEnabled = false;
        ntripUnlock();
        oledSetNtrip(false);
      }
    }

  } else {

    // No network: start AP for offline use
    startApMode();
  }

  // Auto-start BLE RTCM if previously enabled (works with or without WiFi)
  {
    String en = FlashConfig::readFile("/config/ble_rtcm_enabled.txt");
    en.trim();
    if (en == "1" && !ntripEnabled && !tcpInEnabled) {
      Serial.printf("[BLE-RTCM] Auto-start: target='%s'\n", g_bleRtcmTargetName);
      startBleRtcm(String(g_bleRtcmTargetName), g_bleRtcmPasskey);
    }
  }

  // Auto-start ESP-NOW if previously enabled
  {
    String enEspNow = FlashConfig::readFile("/config/espnow_enabled.txt");
    enEspNow.trim();
    String roleEspNow = FlashConfig::readFile("/config/espnow_role.txt");
    roleEspNow.trim();
    if (enEspNow == "1") {
      if (roleEspNow == "tx") {
        startEspNowTx();
      } else {
        startEspNowRx();
      }
    }
  }
}

void loop() {
  if (wifiAvailable) {
    server.handleClient();
    taskYIELD();  // yield to other FreeRTOS tasks after potentially blocking handleClient
    // NTRIP client (rover IN) - protected with mutex
    if (ntripEnabled) {
      if (ntripLock(5)) {
        if (ntripClient) {
          ntripClient->loop();
          if (!ntripClient->isActive()) {
            ntripEnabled = false;
            oledSetNtrip(false);
            if (g_systemLog) {
              g_systemLog->logEvent("NTRIP", "Stopped after connection failures");
            }
          }
        }
        ntripUnlock();
      }
    }
    
    // TCP-IN (rover LAN) - protected with mutex
    if (tcpInEnabled) {
      if (tcpInLock(5)) {
        if (g_tcpIn) {
          g_tcpIn->loop();
        }
        tcpInUnlock();
      }
    }

    // BLE RTCM IN (rover, from rtcm-lora radio)
    if (g_bleRtcmEnabled) {
      g_bleRtcm.loop();
    }

    // Caster OUT - protected with mutex
    if (g_baseCasterOn) {
      if (pusherLock(5)) {
        if (g_pusher) {
          g_pusher->loop();
        }
        pusherUnlock();
      }
    }

    // TCP OUT (does not require mutex, uses thread-safe static methods)
    if (g_baseTcpOn) {
      RtcmStreamer::handle();
    }

    // Viewer TCP
    TcpStreamer::handle();
    if (g_tcpClientOn) {
      TcpClientStreamer::handle();
    }

    // ESP-NOW periodic telemetry (both base and rover)
    if (g_espNowEnabled) {
      uint32_t nowMs = millis();
      if (nowMs - g_espNowLastTelem >= ESPNOW_TELEM_INTERVAL_MS) {
        g_espNowLastTelem = nowMs;
        GNSSPosition pos;
        getPosition(pos);
        uint8_t  role      = g_espNowTxEnabled ? 1 : 0;
        int32_t  lat_e7    = (int32_t)(pos.lat * 1e7);
        int32_t  lon_e7    = (int32_t)(pos.lon * 1e7);
        int16_t  alt_dm    = (int16_t)(pos.alt * 10.0f);
        uint16_t hacc_mm   = (uint16_t)min((float)65535.0f, pos.hAcc * 1000.0f);
        uint16_t uptime_min = (uint16_t)(millis() / 60000UL);
        uint16_t heap_kb   = (uint16_t)(ESP.getFreeHeap() / 1024);
        g_espNow.sendTelemetry(
            role,
            lat_e7, lon_e7, alt_dm,
            pos.fixQuality, pos.carrSoln, pos.numSV, hacc_mm,
            g_espNow.getLastRssi(),
            0,   // pkt_loss_pct: placeholder
            0,   // rtcm_age_ms:  placeholder
            0,   // hop_count:    0 for endpoint nodes
            uptime_min,
            heap_kb
        );
      }

      // Rover relay lease renewal: if a relay is selected, periodically send CMD_RELAY_START
      // to keep the relay forwarding RTCM to us. The relay auto-stops after 15s of silence.
      if (!g_espNowTxEnabled && g_espNowRelayNodeId != 0) {
        if (nowMs - g_espNowRelayLastReq >= g_espNowRelayLeaseMs) {
          g_espNowRelayLastReq = nowMs;
          g_espNow.sendCommand(g_espNowRelayNodeId, CMD_RELAY_START, 0);
          Serial.printf("[ESPNOW] Relay lease renewed -> relay 0x%04X\n", g_espNowRelayNodeId);
        }
      }
    }
  }

  // Queue monitoring for SD logging
  if (loggingActive && sdQueue) {
    UBaseType_t queueUsage = uxQueueMessagesWaiting(sdQueue);
    if (queueUsage > (QUEUE_SIZE * QUEUE_WARNING_THRESHOLD_PERCENT / 100)) {
      static uint32_t lastQueueWarn = 0;
      if (millis() - lastQueueWarn > 10000) {
        Serial.printf("[WARN] SD Queue usage: %d/%d\n", queueUsage, QUEUE_SIZE);
        lastQueueWarn = millis();
      }
    }
  }

  if (millis() - lastOledUpdate > 1000) {
    // Feed GNSS data to OLED monitor
    GNSSPosition oledPos;
    if (getPosition(oledPos)) {
      oledSetPosition(oledPos.lat, oledPos.lon, oledPos.alt);
      oledSetFix(oledPos.fixQuality, oledPos.carrSoln, oledPos.numSV);
      oledSetPDOP(oledPos.pdop);
      oledSetAge(oledPos.age);
      oledSetAccuracy(oledPos.hAcc, oledPos.vAcc);
      // Baseline 3D da RELPOSNED
      if (oledPos.relValid) {
        float bl = (float)sqrt(oledPos.relN * oledPos.relN + oledPos.relE * oledPos.relE + oledPos.relD * oledPos.relD);
        oledSetBaseline(bl);
      } else {
        oledSetBaseline(-1.0f);
      }
    }
    // BLE status
    oledSetBLE(g_bleEnabled, g_bleEnabled && BLESerial::isConnected());
    // ESP-NOW status
    oledSetEspNow(g_espNowEnabled, g_espNowTxEnabled, g_espNow.getLastRssi());
    // Base mode auto-switch
    {
      ZedTmodeState tmode;
      if (getZedTmode(tmode) && tmode.valid) {
        oledSetBaseMode(tmode.mode > 0, tmode.mode);
        if (tmode.mode == 2) {
          oledSetBaseTmode(tmode.lat, tmode.lon, tmode.height, 1);
        }
      } else {
        oledSetBaseMode(false, 0);
      }
    }
#if ENC_CLK_GPIO > 0 && ENC_DT_GPIO > 0 && ENC_SW_GPIO > 0
    if (OledMenu::isActive()) {
      OledMenu::draw(oledGetDisplay());
    } else {
      oledUpdate();
    }
#else
    oledUpdate();
#endif
    lastOledUpdate = millis();
  }

  // ---- Rotary encoder polling (fast, every loop iteration) ----
#if ENC_CLK_GPIO > 0 && ENC_DT_GPIO > 0 && ENC_SW_GPIO > 0
  {
    RotaryInput::update();
    int  dir     = RotaryInput::getDirection();
    bool click   = RotaryInput::isClicked();
    bool dblClick = RotaryInput::isDoubleClick();
    bool longP   = RotaryInput::isLongPress();
    if (dir != 0 || click || dblClick || longP) {
      OledMenu::handleInput(dir, click, dblClick, longP);
      // Redraw immediately for responsive UI
      if (OledMenu::isActive()) {
        OledMenu::draw(oledGetDisplay());
      } else {
        oledUpdate();
      }
      lastOledUpdate = millis();
    }
  }
#endif

  // ===== BLE housekeeping (only if enabled) =====
  if (g_bleEnabled) {
    BLESerial::loop();
  }

  // ===== NEW: Buzzer and SystemLog updates =====
  // Update buzzer (non-blocking tone playback)
  if (g_buzzer) {
    g_buzzer->loop();
  }

  // Update system log (flush pending entries)
  if (g_systemLog) {
    g_systemLog->loop();
  }

  // Detect RTK fix state changes and trigger buzzer
  GNSSPosition pos;
  if (getPosition(pos)) {
    if (pos.carrSoln != g_lastCarrSoln) {
      // State changed
      if (g_lastCarrSoln == 2 && pos.carrSoln != 2) {
        // Lost RTK Fixed
        if (g_buzzer) g_buzzer->playEvent(BuzzerEvent::RTK_LOST);
        if (g_systemLog) g_systemLog->logEvent("FIX", "Lost: RTK Fixed -> " + String(pos.carrSoln == 1 ? "Float" : "Single"));
      } else if (pos.carrSoln == 2 && g_lastCarrSoln != 2) {
        // Acquired RTK Fixed
        if (g_buzzer) g_buzzer->playEvent(BuzzerEvent::RTK_FIXED);
        if (g_systemLog) g_systemLog->logEvent("FIX", String("Acquired: ") + (g_lastCarrSoln == 1 ? "Float" : "Single") + " -> RTK Fixed");
      }
      g_lastCarrSoln = pos.carrSoln;
    }
  }

  // Periodic heap and stack monitoring (every 30 seconds)
  uint32_t now = millis();
  if (now - g_lastHeapCheck > 30000) {
    g_lastHeapCheck = now;
    
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t minFreeHeap = ESP.getMinFreeHeap();
    
    Serial.printf("[MEM] Heap: %u free, %u min ever\n", freeHeap, minFreeHeap);
    
    // Log heap status to SystemLog periodically
    if (g_systemLog) {
      g_systemLog->logEvent("HEAP", String(freeHeap) + " free, " + String(minFreeHeap) + " min");
      
      // Warning if low
      if (freeHeap < HEAP_WARNING_THRESHOLD) {
        Serial.println("[MEM] WARNING: Low heap!");
        g_systemLog->logEvent("HEAP", "WARNING: Low memory!");
      }
    }
    
    // Log stack watermarks
    if (uartTaskHandle) {
      UBaseType_t stackFree = uxTaskGetStackHighWaterMark(uartTaskHandle);
      Serial.printf("[Stack] UARTReader: %d bytes free\n", stackFree);
    }
    if (nmeaTaskHandle) {
      UBaseType_t stackFree = uxTaskGetStackHighWaterMark(nmeaTaskHandle);
      Serial.printf("[Stack] NMEAReader: %d bytes free\n", stackFree);
    }
    if (sdTaskHandle) {
      UBaseType_t stackFree = uxTaskGetStackHighWaterMark(sdTaskHandle);
      Serial.printf("[Stack] SDWriter: %d bytes free\n", stackFree);
    }
  }
  
  // Periodic survey sync to SD (every 5 minutes)
  if (!loggingActive) {
    SurveyPoints::periodicSync();
  }
}
