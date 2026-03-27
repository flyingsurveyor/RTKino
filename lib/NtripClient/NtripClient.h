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

  // invia una riga GGA (la libreria applica il rate-limit)
  void sendGGALine(const char* gga, size_t len);

  // cambia periodo minimo (ms). Default 5000 ms.
  void setGgaMinPeriodMs(uint32_t ms);

  // Richiesta prudente di reconnect dopo recovery WiFi:
  // vale solo se la sessione aveva gia' collegato almeno una volta.
  void forceReconnect();

private:
  String _host, _mount, _user, _pass;
  int _port;
  WiFiClient _client;
  Stream* _output = nullptr;

  bool _connected = false;
  bool _enabled = false;
  bool _everConnectedThisSession = false;
  bool _allowAutoReconnect = false;
  uint8_t _consecutiveFailures = 0;
  uint8_t _attemptsRemaining = 0;

  // rate limit GGA
  uint32_t _ggaMinPeriodMs = 5000;
  uint32_t _lastGgaSentMs = 0;

  // retry policy: max 2 connect per activation/recovery cycle
  uint32_t _lastReconnectAttemptMs = 0;
  static const uint32_t RETRY_DELAY_MS = 1500;
  static const uint8_t MAX_CONNECT_ATTEMPTS = 2;

  // Static buffer for data reception (instead of stack allocation)
  uint8_t _rxBuffer[512];

  // Health check tracking
  uint32_t _lastDataReceivedMs = 0;
  static const uint32_t STALE_DATA_TIMEOUT_MS = 15000;
  static const uint32_t HEADER_WAIT_MS = 750;

  String encodeBase64(const String& auth);
  bool connectToCaster();
  void armTwoAttempts(bool allowAutoReconnect);
  void latchFailure();
};

#endif
