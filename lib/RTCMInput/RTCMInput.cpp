// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "RTCMInput.h"

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
    uint32_t now = millis();
    if (now - _lastReconnectMs >= 3000) {
      _lastReconnectMs = now;
      _client.stop();
      _client.setNoDelay(true);
      _client.setTimeout(3000); // 3 second timeout for connect and read
      
      // Attempt connection (note: WiFiClient::connect is blocking but has timeout)
      // This is acceptable since the timeout is controlled
      bool connected = _client.connect(_host.c_str(), _port);
      if (!connected) {
        _client.stop();
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
