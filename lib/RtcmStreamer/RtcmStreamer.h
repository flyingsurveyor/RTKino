// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class RtcmStreamer {
public:
  static void begin(uint16_t port=2102);     // TCP server for raw RTCM
  static void stop();
  static void handle();
  static void broadcast(const uint8_t* data, size_t len);
  static bool active();
  static void setActive(bool on);
  static bool hasClient();  // returns true if a TCP client is currently connected

private:
  static WiFiServer _srv;
  static WiFiClient _cli;
  static bool _on;
  static SemaphoreHandle_t _mutex;
  static bool _initialized;
  
  static void ensureMutex();
  static bool lock(uint32_t timeoutMs = 10);
  static void unlock();
};
