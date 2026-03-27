// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "NtripClient.h"
#include "OledLogger.h"

NtripClient::NtripClient(const char* host, int port, const char* mount, const char* user, const char* pass)
: _host(host), _mount(mount), _user(user), _pass(pass), _port(port) {}

void NtripClient::armTwoAttempts(bool allowAutoReconnect) {
  _enabled = true;
  _connected = false;
  _allowAutoReconnect = allowAutoReconnect;
  _attemptsRemaining = MAX_CONNECT_ATTEMPTS;
  _consecutiveFailures = 0;
  _lastReconnectAttemptMs = 0;
  _lastGgaSentMs = 0;
  _lastDataReceivedMs = millis();
  if (_client.connected()) {
    _client.stop();
  }
}

void NtripClient::latchFailure() {
  _enabled = false;
  _connected = false;
  _attemptsRemaining = 0;
  if (_client.connected()) {
    _client.stop();
  }
  oledPrintln("[NTRIP] Inactive");
}

void NtripClient::begin(Stream& output) {
  _output = &output;
  _everConnectedThisSession = false;
  armTwoAttempts(false);
}

void NtripClient::stop() {
  _enabled = false;
  _connected = false;
  _allowAutoReconnect = false;
  _attemptsRemaining = 0;
  _consecutiveFailures = 0;
  _lastGgaSentMs = 0;
  _lastDataReceivedMs = 0;
  if (_client.connected()) {
    _client.stop();
    oledPrintln("[NTRIP] Disconnected manually");
  }
}

bool NtripClient::isActive() {
  return _enabled;
}

void NtripClient::loop() {
  if (!_enabled) return;

  // If STA is not really up, keep session armed only if it had already worked once.
  if (WiFi.status() != WL_CONNECTED) {
    if (_client.connected()) {
      _client.stop();
    }
    _connected = false;
    return;
  }

  // Zombie/stale connection detection: socket open but no data for STALE_DATA_TIMEOUT_MS
  if (_connected && _client.connected() &&
      millis() - _lastDataReceivedMs > STALE_DATA_TIMEOUT_MS) {
    Serial.println("[NTRIP] Stale connection detected");
    _client.stop();
    _connected = false;

    if (_allowAutoReconnect && _everConnectedThisSession) {
      _attemptsRemaining = MAX_CONNECT_ATTEMPTS;
      _consecutiveFailures = 0;
      _lastReconnectAttemptMs = 0;
    } else {
      latchFailure();
      return;
    }
  }

  if (!_connected || !_client.connected()) {
    if (_attemptsRemaining == 0) {
      latchFailure();
      return;
    }

    const uint32_t now = millis();
    if (_lastReconnectAttemptMs != 0 && (now - _lastReconnectAttemptMs) < RETRY_DELAY_MS) {
      return;
    }

    _lastReconnectAttemptMs = now;
    if (_client.connected()) {
      _client.stop();
    }

    const bool ok = connectToCaster();
    if (ok) {
      _connected = true;
      _everConnectedThisSession = true;
      _consecutiveFailures = 0;
      _attemptsRemaining = MAX_CONNECT_ATTEMPTS;
      _lastGgaSentMs = 0;
      _lastDataReceivedMs = millis();
      return;
    }

    _connected = false;
    ++_consecutiveFailures;
    if (_attemptsRemaining > 0) {
      --_attemptsRemaining;
    }

    if (_attemptsRemaining == 0 || _consecutiveFailures >= MAX_CONNECT_ATTEMPTS) {
      latchFailure();
    }
    return;
  }

  int avail = _client.available();
  if (avail > 0) {
    int toRead = min(avail, (int)sizeof(_rxBuffer));
    int len = _client.read(_rxBuffer, toRead);
    if (len > 0) {
      _output->write(_rxBuffer, len);
      _lastDataReceivedMs = millis();
    } else if (len < 0) {
      Serial.println("[NTRIP] Read error");
      _client.stop();
      _connected = false;
      if (_allowAutoReconnect && _everConnectedThisSession) {
        _attemptsRemaining = MAX_CONNECT_ATTEMPTS;
        _consecutiveFailures = 0;
        _lastReconnectAttemptMs = 0;
      } else {
        latchFailure();
      }
    }
  }
}

String NtripClient::encodeBase64(const String& auth) {
  const char* base64Chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String encoded = "";
  int i = 0;
  uint8_t buf[3];

  for (size_t idx = 0; idx < auth.length();) {
    i = 0;
    memset(buf, '\0', 3);
    while (i < 3 && idx < auth.length()) buf[i++] = auth[idx++];

    encoded += base64Chars[(buf[0] & 0xfc) >> 2];
    encoded += base64Chars[((buf[0] & 0x03) << 4) | ((buf[1] & 0xf0) >> 4)];
    encoded += (i > 1) ? base64Chars[((buf[1] & 0x0f) << 2) | ((buf[2] & 0xc0) >> 6)] : '=';
    encoded += (i > 2) ? base64Chars[buf[2] & 0x3f] : '=';
  }

  return encoded;
}

bool NtripClient::connectToCaster() {
  oledPrintln("[NTRIP] Connecting...");
  _client.setTimeout(1000);

  if (!_client.connect(_host.c_str(), _port)) {
    oledPrintln("[NTRIP] Caster error");
    _client.stop();
    return false;
  }
  _client.setNoDelay(true);

  String auth = encodeBase64(_user + ":" + _pass);
  String req = "GET /" + _mount + " HTTP/1.1\r\n";
  req += "Host: " + _host + "\r\n";
  req += "User-Agent: NTRIP LolinS3Pro/1.0\r\n";
  req += "Authorization: Basic " + auth + "\r\n\r\n";

  _client.print(req);

  unsigned long t0 = millis();
  while (!_client.available() && millis() - t0 < HEADER_WAIT_MS) {
    delay(10);
  }

  if (!_client.available()) {
    oledPrintln("[NTRIP] No header");
    _client.stop();
    return false;
  }

  String header = "";
  while (_client.available()) header += (char)_client.read();

  // Accept both ICY 200 and HTTP/1.1 200 responses for wider compatibility
  if (!header.startsWith("ICY 200") && !header.startsWith("HTTP/1.1 200")) {
    oledPrintln("[NTRIP] Invalid response");
    _client.stop();
    return false;
  }

  oledPrintln("[NTRIP] Connected OK");
  return true;
}

void NtripClient::setGgaMinPeriodMs(uint32_t ms) {
  if (ms < 1000) ms = 1000;
  _ggaMinPeriodMs = ms;
}

void NtripClient::sendGGALine(const char* gga, size_t len) {
  if (!_enabled || !_connected || !_client.connected() || !gga || len == 0) return;

  uint32_t now = millis();
  if (_lastGgaSentMs != 0 && (now - _lastGgaSentMs) < _ggaMinPeriodMs) return;

  _client.write((const uint8_t*)gga, len);
  _client.write((const uint8_t*)"\r\n", 2);
  _lastGgaSentMs = now;
}

void NtripClient::forceReconnect() {
  Serial.println("[NTRIP] Force reconnect requested");
  if (!_enabled) return;
  if (!_everConnectedThisSession) {
    // Never connected in this activation: keep manual / latched semantics.
    return;
  }
  if (_client.connected()) {
    _client.stop();
  }
  armTwoAttempts(true);
}
