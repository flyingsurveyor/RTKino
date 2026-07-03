// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "RTCMInput.h"

static bool resolveRtcmHost(const String& host, IPAddress& out) {
  IPAddress tmp;
  if (tmp.fromString(host.c_str())) { out = tmp; return true; }  // già un IP
  if (WiFi.hostByName(host.c_str(), tmp)) { out = tmp; return true; }
  Serial.println("[RTCM-IN] DNS resolve failed");
  return false;
}

bool RTCMInput::configure(const String& host, uint16_t port) {
  _host = host;
  _port = port;
  return true;
}

bool RTCMInput::begin(HardwareSerial& out) {
  _out = &out;
  _active = true;
  _lastReconnectMs = 0;
  return true;
}

void RTCMInput::stop() {
  _active = false;
  if (_client.connected()) _client.stop();
}

bool RTCMInput::isConnected() {
  return _client.connected();
}

void RTCMInput::loop() {
  if (!_active || !_out) return;

  // auto-reconnect every 3s if disconnected
  if (!_client.connected()) {
    if (WiFi.status() != WL_CONNECTED) return;  // Evita DNS bloccante se WiFi è giù
    uint32_t now = millis();
    if (now - _lastReconnectMs >= 3000) {
      _lastReconnectMs = now;
      _client.stop();

      // Risolvi hostname → IP (max 2s); se fallisce salta il tentativo
      if (!_ipResolved && !resolveRtcmHost(_host, _resolvedIp)) return;
      _ipResolved = true;

      _client.setNoDelay(true);
      _client.setTimeout(1000);
      if (!_client.connect(_resolvedIp, _port)) {
        _client.stop();
        _ipResolved = false;  // Forza re-resolve al prossimo tentativo
      }
    }
    return;
  }

  // if connected -> read RTCM data and send to UART
  int ava = _client.available();
  if (ava > 0) {
    int toRead = min(ava, (int)sizeof(_rxBuffer));
    int n = _client.readBytes((char*)_rxBuffer, toRead);
    if (n > 0) {
      _out->write(_rxBuffer, n);
    }
  }
}

void RTCMInput::forceReconnect() {
  Serial.println("[RTCM-IN] Force reconnect requested");
  if (_client.connected()) {
    _client.stop();
  }
  _lastReconnectMs = 0;  // Allow immediate reconnection attempt
}
