// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef NTRIP_CLIENT_H
#define NTRIP_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <Stream.h>

class NtripClient {
public:
  NtripClient(const char* host, int port, const char* mount, const char* user, const char* pass);
  void begin(Stream& output);
  void loop();
  void stop();
  bool isActive();

  // >>> NUOVO: invia una riga GGA (la libreria applica il rate-limit)
  void sendGGALine(const char* gga, size_t len);

  // >>> OPZIONALE: cambia periodo minimo (ms). Default 5000 ms.
  void setGgaMinPeriodMs(uint32_t ms);
  
  // Force reconnection after WiFi recovery
  void forceReconnect();

private:
  String _host, _mount, _user, _pass;
  int _port;
  WiFiClient _client;
  Stream* _output = nullptr;
  bool _connected = false;
  bool _enabled   = true;

  // >>> NUOVO: rate limit GGA
  uint32_t _ggaMinPeriodMs = 5000;   // default 5 s (come i tuoi test: 3000/5000)
  uint32_t _lastGgaSentMs  = 0;

  // >>> NUOVO: non-blocking reconnection
  uint32_t _lastReconnectAttemptMs = 0;
  static const uint32_t RECONNECT_DELAY_MS = 1000;
  
  // Static buffer for data reception (instead of stack allocation)
  uint8_t _rxBuffer[512];
  
  // Health check tracking
  uint32_t _lastDataReceivedMs = 0;
  static const uint32_t STALE_DATA_TIMEOUT_MS = 15000;

  String encodeBase64(const String& auth);
  bool connectToCaster();
};

#endif
