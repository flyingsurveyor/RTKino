// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef OLEDLOGGER_H
#define OLEDLOGGER_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

void oledInit();
void oledSetWifi(bool connected);     // STA connected
void oledSetApMode(bool active);      // AP mode active
void oledSetNtrip(bool connected);
void oledSetLogging(bool active);
void oledSetIP(const String& ip);
void oledSetSilent(bool silent);
void oledUpdate();
void oledPrintln(const String& message);
void oledPrint(const String& message);
void oledSplashLandscape();

// ===== New monitor setters =====
void oledSetPosition(double lat, double lon, float alt);
void oledSetFix(uint8_t fixQuality, uint8_t carrSoln, uint8_t numSV);
void oledSetPDOP(float pdop);
void oledSetAge(float age);
void oledSetBLE(bool enabled, bool connected);
// Accuracy (hAcc/vAcc in metres, from NAV-HPPOSLLH; falls back to NAV-PVT)
void oledSetAccuracy(float hAcc, float vAcc);
// Base mode display
void oledSetBaseMode(bool active, uint8_t tmodeMode);  // tmodeMode: 0=disabled, 1=survey, 2=fixed
void oledSetBaseTmode(double lat, double lon, double height, uint16_t stid);
void oledSetBaseline(float baseline);  // baseline 3D da NAV-RELPOSNED (metri)

// ESP-NOW status
void oledSetEspNow(bool enabled, bool isTx, int8_t rssi);

// Base output status callbacks (set from main.cpp)
extern bool (*oledGetCasterState)();      // g_baseCasterOn
extern bool (*oledGetCasterConnected)();  // g_pusher && g_pusher->isConnected()
extern bool (*oledGetTcpSrvState)();      // g_baseTcpOn
extern bool (*oledGetTcpSrvClient)();     // RtcmStreamer::hasClient()
extern bool (*oledGetTcpCliState)();      // g_tcpClientOn
extern bool (*oledGetTcpCliConnected)();  // TcpClientStreamer::isConnected()

// Returns a reference to the underlying SSD1306 display object.
// Used by OledMenu to render the menu on the same display instance.
Adafruit_SSD1306& oledGetDisplay();

#endif