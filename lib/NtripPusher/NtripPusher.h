// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <Arduino.h>
#include <WiFi.h>

class NtripPusher {
public:
  NtripPusher(const String& host, int port, const String& mount, const String& pass,
              const String& userAgent = "RTKino/1.7");

  bool begin();        // opens socket + handshake
  void stop();
  bool isConnected();
  void loop();         // auto-reconnect logic
  void write(const uint8_t* data, size_t len);
  void forceReconnect();  // Force reconnection after WiFi recovery

  // Invalida la cache IP (chiamare su WiFi disconnect)
  void clearResolvedIp() { _ipResolved = false; }

private:
  String _host, _mount, _pass, _ua;
  int _port;
  WiFiClient _cli;
  
  // Auto-reconnect state
  uint32_t _lastReconnectAttemptMs = 0;
  static const uint32_t RECONNECT_DELAY_MS = 5000;
  bool _autoReconnect = true;
  
  // Health check tracking
  uint32_t _lastSuccessfulWriteMs = 0;
  static const uint32_t STALE_TIMEOUT_MS = 30000;  // 30 seconds without successful write
  
  // Remember successful handshake variant
  enum HandshakeVariant {
    UNKNOWN = 0,
    V2_SLASH = 1,
    V2_NOSLASH = 2,
    V1_SLASH = 3,
    V1_NOSLASH = 4
  };
  HandshakeVariant _successfulVariant = UNKNOWN;

  bool resolveHost();
  bool connectSocket();
  bool handshakeTry(const String& req);
  bool doHandshake();
  String getHandshakeRequest(HandshakeVariant variant);

  // Cache IP per evitare DNS lookup ad ogni riconnessione
  IPAddress _resolvedIp;
  bool _ipResolved = false;

  // Indice variante corrente per il round-robin in loop() (1 variante per ciclo)
  uint8_t _loopVariantIdx = 0;
};
