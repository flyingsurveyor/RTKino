// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "OledLogger.h"
#include "OledMenu.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "landscapeBitmap.h"
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static bool wifiConnected = false;
static bool apMode = false;
static bool ntripConnected = false;
static bool logging = false;
static bool silent = false;
static String ipAddress = "---";
extern String lastRateSet;

// GNSS state
static double   mon_lat = 0.0;
static double   mon_lon = 0.0;
static float    mon_alt = 0.0;
static uint8_t  mon_fixQuality = 0;
static uint8_t  mon_carrSoln = 0;
static uint8_t  mon_numSV = 0;
static float    mon_pdop = 99.9f;
static float    mon_age = 0.0f;
static float    mon_hAcc = 0.0f;
static float    mon_vAcc = 0.0f;
static bool     mon_bleEnabled = false;
static bool     mon_bleConnected = false;
// Base mode state
static bool     mon_baseActive = false;
static uint8_t  mon_tmodeMode = 0;
static double   mon_baseLat = 0.0;
static double   mon_baseLon = 0.0;
static double   mon_baseHeight = 0.0;
static uint16_t mon_baseStid = 0;
static float    mon_baseline = -1.0f;  // -1 = non valido

// ESP-NOW state
static bool    mon_espNowEnabled = false;
static bool    mon_espNowIsTx    = false;
static int8_t  mon_espNowRssi    = 0;



// Base output status callbacks (default nullptr, wired from main.cpp)
bool (*oledGetCasterState)()     = nullptr;
bool (*oledGetCasterConnected)() = nullptr;
bool (*oledGetTcpSrvState)()     = nullptr;
bool (*oledGetTcpSrvClient)()    = nullptr;
bool (*oledGetTcpCliState)()     = nullptr;
bool (*oledGetTcpCliConnected)() = nullptr;
bool (*oledGetTcpInState)()      = nullptr;
bool (*oledGetTrackingActive)()  = nullptr;

void oledInit() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found");
    return;
  }
  display.setRotation(2);  // physical display mounted upside down on the PCB
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void oledSplashLandscape() {
  display.clearDisplay();
  display.drawBitmap(0, 0, landscapeBitmap, LANDSCAPE_WIDTH, LANDSCAPE_HEIGHT, SSD1306_WHITE);
  display.display();
  delay(5000);
}

void oledSetWifi(bool c){ wifiConnected = c; }
void oledSetApMode(bool active){ apMode = active; }
void oledSetNtrip(bool c){ ntripConnected = c; }
void oledSetLogging(bool a){ logging = a; }
void oledSetIP(const String& ip){ ipAddress = ip; }
void oledSetSilent(bool s){ silent = s; }

void oledSetPosition(double lat, double lon, float alt) {
  mon_lat = lat; mon_lon = lon; mon_alt = alt;
}
void oledSetFix(uint8_t fixQuality, uint8_t carrSoln, uint8_t numSV) {
  mon_fixQuality = fixQuality; mon_carrSoln = carrSoln; mon_numSV = numSV;
}
void oledSetPDOP(float pdop) { mon_pdop = pdop; }
void oledSetAge(float age) { mon_age = age; }
void oledSetBLE(bool enabled, bool connected) {
  mon_bleEnabled = enabled; mon_bleConnected = connected;
}
void oledSetAccuracy(float hAcc, float vAcc) {
  mon_hAcc = hAcc; mon_vAcc = vAcc;
}
void oledSetBaseMode(bool active, uint8_t tmodeMode) {
  mon_baseActive = active; mon_tmodeMode = tmodeMode;
}
void oledSetBaseTmode(double lat, double lon, double height, uint16_t stid) {
  mon_baseLat = lat; mon_baseLon = lon; mon_baseHeight = height; mon_baseStid = stid;
}
void oledSetBaseline(float baseline) { mon_baseline = baseline; }

void oledSetEspNow(bool enabled, bool isTx, int8_t rssi) {
    mon_espNowEnabled = enabled;
    mon_espNowIsTx    = isTx;
    mon_espNowRssi    = rssi;
}

// Draw page indicator "N/T" right-aligned at y=55 (bottom-right corner)
static void drawPageIndicator(int currentPage, int totalPages) {
  char buf[6];
  snprintf(buf, sizeof(buf), "%d/%d", currentPage, totalPages);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w - 1, 55);
  display.print(buf);
}

// Draw dashed horizontal separator line
static void drawDashedLine(int y) {
  for (int x = 0; x < SCREEN_WIDTH; x += 4) {
    display.drawPixel(x, y, SSD1306_WHITE);
    display.drawPixel(x + 1, y, SSD1306_WHITE);
  }
}

// Draw a bare status glyph (no box) — only ever called for an ACTIVE status,
// so an inactive slot simply draws nothing rather than an empty outline.
static void drawBadgeGlyph(int x, int y, const char* label) {
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.print(label);
}

// Draw status badges at y=55, one fixed 14px-wide column per status so the
// eye always knows where to look regardless of which ones are lit:
//   A=AP mode (x=0), N=NTRIP IN (x=14), T=TCP IN (x=28), L=RAW log (x=42),
//   B=GNSS base mode (x=56), ESP-NOW role B/R (x=70), RT=track recording (x=84)
// WiFi-STA-connected has no badge (IP lives on Page 3); BLE has no badge here
// (still visible in Impostazioni > Rete).
static void drawStatusBadges() {
  const int badgeY = 55;
  const int pitch = 14;
  int x = 0;
  if (apMode) drawBadgeGlyph(x, badgeY, "A");
  x += pitch;
  if (ntripConnected) drawBadgeGlyph(x, badgeY, "N");
  x += pitch;
  if (oledGetTcpInState && oledGetTcpInState()) drawBadgeGlyph(x, badgeY, "T");
  x += pitch;
  if (logging) drawBadgeGlyph(x, badgeY, "L");
  x += pitch;
  if (mon_baseActive) drawBadgeGlyph(x, badgeY, "B");
  x += pitch;
  if (mon_espNowEnabled) drawBadgeGlyph(x, badgeY, mon_espNowIsTx ? "B" : "R");
  x += pitch;
  if (oledGetTrackingActive && oledGetTrackingActive()) drawBadgeGlyph(x, badgeY, "RT");
}

// Get fix type string from carrSoln (NAV-PVT) with GGA fallback
static const char* fixTypeStr(uint8_t carrSoln, uint8_t fixQuality) {
  if (carrSoln == 2) return "RTK FIX";
  if (carrSoln == 1) return "FLOAT";
  if (fixQuality >= 2) return "DGPS";
  if (fixQuality == 1) return "3D";
  return "NO FIX";
}

void oledUpdate() {
  if (silent) {
    display.clearDisplay();
    display.display();
    return;
  }

  display.clearDisplay();

  // ---- Page 2: accuracy / detail view ----
  if (OledMenu::getState() == OLED_PAGE2) {
    if (mon_baseActive) {
      // ---- Page 2 in BASE mode: output status layout ----
      display.setCursor(0, 0);
      display.print("BASE OUT");
      // STID right-aligned — moved here from Page 1 (fixed mode) to make room
      // for the live output profile there.
      char stidBuf[16];
      snprintf(stidBuf, sizeof(stidBuf), "STID:%u", (unsigned)mon_baseStid);
      int16_t sx1, sy1; uint16_t sw, sh;
      display.getTextBounds(stidBuf, 0, 0, &sx1, &sy1, &sw, &sh);
      display.setCursor(SCREEN_WIDTH - sw - 2, 0);
      display.print(stidBuf);
      drawDashedLine(9);

      // Helper lambda: draw one output row
      // label (left), state from cb_state, connected from cb_conn
      auto drawOutRow = [](const char* label, int y,
                           bool (*cb_state)(), bool (*cb_conn)(),
                           const char* connStr, const char* noConnStr) {
        display.setCursor(0, y);
        display.print(label);
        char rightBuf[12];
        bool stateOn = cb_state ? cb_state() : false;
        if (!stateOn) {
          snprintf(rightBuf, sizeof(rightBuf), "OFF");
        } else {
          bool connected = cb_conn ? cb_conn() : false;
          if (connected)
            snprintf(rightBuf, sizeof(rightBuf), "ON  %s", connStr);
          else
            snprintf(rightBuf, sizeof(rightBuf), "ON  %s", noConnStr);
        }
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(rightBuf, 0, 0, &x1, &y1, &w, &h);
        display.setCursor(SCREEN_WIDTH - w - 2, y);
        display.print(rightBuf);
      };

      drawOutRow("Caster", 11, oledGetCasterState, oledGetCasterConnected, "OK", "---");
      drawOutRow("TCP Srv", 20, oledGetTcpSrvState, oledGetTcpSrvClient, "cli", "---");
      drawOutRow("TCP Cli", 29, oledGetTcpCliState, oledGetTcpCliConnected, "OK", "---");

      drawStatusBadges();
      drawPageIndicator(2, 3);
      display.display();
      return;
    }

    // ---- Page 2 in ROVER mode: accuracy / detail view ----
    // Row 0 (y=0): fix type + rate (always visible)
    display.setCursor(0, 0);
    display.print(fixTypeStr(mon_carrSoln, mon_fixQuality));
    int16_t rx1, ry1; uint16_t rw, rh;
    display.getTextBounds(lastRateSet, 0, 0, &rx1, &ry1, &rw, &rh);
    display.setCursor(SCREEN_WIDTH - rw - 2, 0);
    display.print(lastRateSet);

    // Row 1 (y=9): dashed separator
    drawDashedLine(9);

    // Rows 2-4: accuracy / baseline
    char buf[24];
    snprintf(buf, sizeof(buf), "hAcc: %.4f m", mon_hAcc);
    display.setCursor(0, 11);
    display.print(buf);

    snprintf(buf, sizeof(buf), "vAcc: %.4f m", mon_vAcc);
    display.setCursor(0, 20);
    display.print(buf);

    if (mon_baseline < 0.0f)
      snprintf(buf, sizeof(buf), "Bsln: ---");
    else if (mon_baseline < 1000.0f)
      snprintf(buf, sizeof(buf), "Bsln: %.3f m", mon_baseline);
    else
      snprintf(buf, sizeof(buf), "Bsln: %.1f m", mon_baseline);
    display.setCursor(0, 29);
    display.print(buf);

    // Row 7 (y=54): status letter badges
    drawStatusBadges();
    drawPageIndicator(2, 3);
    display.display();
    return;
  }

  // ---- Page 3: system info ----
  if (OledMenu::getState() == OLED_PAGE3) {
    // Row 0 (y=0): "INFO" + rate right-aligned
    display.setCursor(0, 0);
    display.print("INFO");
    int16_t rx1, ry1; uint16_t rw, rh;
    display.getTextBounds(lastRateSet, 0, 0, &rx1, &ry1, &rw, &rh);
    display.setCursor(SCREEN_WIDTH - rw - 2, 0);
    display.print(lastRateSet);

    // Row 1 (y=9): dashed separator
    drawDashedLine(9);

    // Rows 2-5: system info via getInfoString callback
    if (OledMenu::getInfoString) {
      String info = OledMenu::getInfoString();
      // Split on '\n' and draw up to 4 lines
      int lineY = 11;
      int start = 0;
      int len = info.length();
      for (int i = 0; i <= len && lineY <= 38; i++) {
        if (i == len || info[i] == '\n') {
          String line = info.substring(start, i);
          display.setCursor(0, lineY);
          display.print(line);
          lineY += 9;
          start = i + 1;
        }
      }
    }

    // Row 6 (y=47): dashed separator
    drawDashedLine(47);

    // Row 7 (y=54): status badges + page indicator
    drawStatusBadges();
    drawPageIndicator(3, 3);
    display.display();
    return;
  }

  // ---- Page 1 (normal rover / base layout) ----
  const bool baseLayout = mon_baseActive && mon_tmodeMode > 0;

  // ---- Row 0 (y=0) ----
  if (baseLayout) {
    const char* modeStr = (mon_tmodeMode == 2) ? "BASE FIXED" : "SURVEY-IN";
    display.setCursor(0, 0);
    display.print(modeStr);
  } else {
    display.setCursor(0, 0);
    display.print(fixTypeStr(mon_carrSoln, mon_fixQuality));
  }
  // Rate right-aligned; compute bounds once for reuse
  int16_t rx1, ry1; uint16_t rw, rh;
  display.getTextBounds(lastRateSet, 0, 0, &rx1, &ry1, &rw, &rh);
  display.setCursor(SCREEN_WIDTH - rw - 2, 0);
  display.print(lastRateSet);
  if (!baseLayout) {
    // SV count to the left of rate
    char svBuf[8];
    snprintf(svBuf, sizeof(svBuf), "%2dsv", (int)mon_numSV);
    display.setCursor(SCREEN_WIDTH - rw - 2 - 30, 0);
    display.print(svBuf);
  }

  // ---- Row 1 (y=9) ----
  if (baseLayout) {
    if (mon_tmodeMode == 2) {
      // TMODE3 fixed: the ZED only outputs RTCM+raw here, so replace the old
      // static "STID:xxxx / RTCM" placeholder with the live RTCM output
      // profile (STID moves to Page 2, which already owns base-out status).
      bool ntripOn   = oledGetCasterState && oledGetCasterState();
      bool ntripConn = ntripOn && oledGetCasterConnected && oledGetCasterConnected();
      char leftBuf[16];
      snprintf(leftBuf, sizeof(leftBuf), "NTRIP:%s", !ntripOn ? "OFF" : (ntripConn ? "OK" : "--"));
      display.setCursor(0, 9);
      display.print(leftBuf);

      bool tcpSrvOn = oledGetTcpSrvState && oledGetTcpSrvState();
      bool tcpCliOn = oledGetTcpCliState && oledGetTcpCliState();
      bool tcpOn    = tcpSrvOn || tcpCliOn;
      bool tcpConn  = (tcpSrvOn && oledGetTcpSrvClient && oledGetTcpSrvClient()) ||
                      (tcpCliOn && oledGetTcpCliConnected && oledGetTcpCliConnected());
      char rightBuf[16];
      snprintf(rightBuf, sizeof(rightBuf), "TCP:%s", !tcpOn ? "OFF" : (tcpConn ? "OK" : "--"));
      int16_t x1, y1; uint16_t w, h;
      display.getTextBounds(rightBuf, 0, 0, &x1, &y1, &w, &h);
      display.setCursor(SCREEN_WIDTH - w - 2, 9);
      display.print(rightBuf);
    } else {
      // Survey-in still converging: unchanged for now.
      char stidBuf[16];
      snprintf(stidBuf, sizeof(stidBuf), "STID:%u", (unsigned)mon_baseStid);
      display.setCursor(0, 9);
      display.print(stidBuf);
      int16_t x1, y1; uint16_t w, h;
      display.getTextBounds("RTCM", 0, 0, &x1, &y1, &w, &h);
      display.setCursor(SCREEN_WIDTH - w - 2, 9);
      display.print("RTCM");
    }
  } else {
    // PDOP left
    char pdopBuf[16];
    if (mon_pdop < 10.0f)
      snprintf(pdopBuf, sizeof(pdopBuf), "PDOP:%.2f", mon_pdop);
    else
      snprintf(pdopBuf, sizeof(pdopBuf), "PDOP:%.1f", mon_pdop);
    display.setCursor(0, 9);
    display.print(pdopBuf);
    // Age right-aligned — inverted block + blink when alarm (RTK active, age > 3s)
    char ageBuf[16];
    snprintf(ageBuf, sizeof(ageBuf), "Age:%.1fs", mon_age);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(ageBuf, 0, 0, &x1, &y1, &w, &h);

    const bool ageAlarm = (mon_carrSoln >= 1 && mon_age > 3.0f);
    static bool s_ageBlinkState = false;
    if (ageAlarm) {
      s_ageBlinkState = !s_ageBlinkState;
      if (s_ageBlinkState) {
        display.fillRect(SCREEN_WIDTH - w - 3, 8, w + 3, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
      }
    } else {
      s_ageBlinkState = false;
    }
    display.setCursor(SCREEN_WIDTH - w - 2, 9);
    display.print(ageBuf);
    display.setTextColor(SSD1306_WHITE);
  }

  // ---- Row 2 (y=18): dashed separator ----
  drawDashedLine(18);

  // ---- Rows 3-5: coordinates ----
  auto printCoord = [](Adafruit_SSD1306& disp, double val, char posChar, char negChar, int y) {
    double absVal = val < 0 ? -val : val;
    char ns = val >= 0 ? posChar : negChar;
    char numStr[22];
    dtostrf(absVal, 12, 8, numStr);
    char buf[32];
    snprintf(buf, sizeof(buf), "%s %c", numStr, ns);
    disp.setCursor(0, y);
    disp.print(buf);
  };

  if (baseLayout) {
    printCoord(display, mon_baseLat, 'N', 'S', 20);
    printCoord(display, mon_baseLon, 'E', 'W', 29);
    char hBuf[16];
    dtostrf(mon_baseHeight, 9, 3, hBuf);
    char heightLine[28];
    snprintf(heightLine, sizeof(heightLine), "H:%s m", hBuf);
    display.setCursor(0, 38);
    display.print(heightLine);
  } else {
    printCoord(display, mon_lat, 'N', 'S', 20);
    printCoord(display, mon_lon, 'E', 'W', 29);
    char altBuf[16];
    dtostrf(mon_alt, 9, 3, altBuf);
    char altLine[28];
    snprintf(altLine, sizeof(altLine), "Alt:%s m", altBuf);
    display.setCursor(0, 38);
    display.print(altLine);
  }

  // ---- Row 6 (y=47): dashed separator ----
  drawDashedLine(47);

  // ---- Row 7 (y=54): status letter badges ----
  drawStatusBadges();
  drawPageIndicator(1, 3);

  display.display();
}

void oledPrintln(const String& m){ Serial.println(m); }
void oledPrint(const String& m){ Serial.print(m); }

Adafruit_SSD1306& oledGetDisplay() { return display; }
