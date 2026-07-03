// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "NtripPusher.h"

static bool responseIsOK(const String& resp) {
  if (resp.indexOf("ICY 200") >= 0) return true;
  if (resp.indexOf("HTTP/1.1 200") >= 0) return true;
  if (resp.indexOf("\nOK") >= 0 || resp.startsWith("OK")) return true;
  return false;
}

static bool responseIsAuthError(const String& resp) {
  return resp.indexOf("401")>=0 || resp.indexOf("403")>=0
      || resp.indexOf("Bad Password")>=0 || resp.indexOf("ERROR - Bad")>=0;
}

NtripPusher::NtripPusher(const String& host, int port, const String& mount, const String& pass, const String& ua)
: _host(host), _mount(mount), _pass(pass), _ua(ua), _port(port) {}

bool NtripPusher::resolveHost() {
  IPAddress tmp;
  if (tmp.fromString(_host.c_str())) {
    _resolvedIp = tmp;
    _ipResolved = true;
    return true;
  }
  if (WiFi.hostByName(_host.c_str(), tmp)) {
    _resolvedIp = tmp;
    _ipResolved = true;
    return true;
  }
  Serial.println("[NTRIP-PUSH] DNS resolve failed");
  return false;
}

bool NtripPusher::connectSocket() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (_cli.connected()) _cli.stop();
  if (!_ipResolved && !resolveHost()) return false;
  _cli.setTimeout(1000);
  if (!_cli.connect(_resolvedIp, _port)) {
    Serial.println("[NTRIP-PUSH] socket connect FAIL");
    _ipResolved = false;  // Forza re-resolve al prossimo tentativo
    return false;
  }
  _cli.setNoDelay(true);
  return true;
}

bool NtripPusher::handshakeTry(const String& req) {
  _cli.print(req);

  uint32_t t0 = millis();
  String resp;
  while (millis() - t0 < 1000) {
    while (_cli.available()) resp += (char)_cli.read();
    if (responseIsOK(resp)) {
      Serial.println("[NTRIP-PUSH] OK");
      return true;
    }
    if (responseIsAuthError(resp)) {
      Serial.println("[NTRIP-PUSH] auth error:");
      Serial.println(resp);
      return false;
    }
    delay(10);
  }
  Serial.println("[NTRIP-PUSH] no OK within timeout. Resp:");
  Serial.println(resp);
  return false;
}

bool NtripPusher::doHandshake() {
  // If we know which variant works, try it first
  if (_successfulVariant != UNKNOWN) {
    String req = getHandshakeRequest(_successfulVariant);
    if (handshakeTry(req)) {
      return true;
    }
    // If previously successful variant fails, reset and try all
    _successfulVariant = UNKNOWN;
  }

  // Try all variants to find one that works
  HandshakeVariant variants[] = {V2_SLASH, V2_NOSLASH, V1_SLASH, V1_NOSLASH};
  const char* variantNames[] = {"v2 +slash", "v2 -slash", "v1 +slash", "v1 -slash"};
  const int numVariants = sizeof(variants) / sizeof(variants[0]);
  
  for (int i = 0; i < numVariants; i++) {
    // Reconnect socket before each variant (previous attempt may have left stale data)
    if (i > 0) {
      _cli.stop();
      if (!connectSocket()) continue;
    }
    Serial.print("[NTRIP-PUSH] Trying ");
    Serial.println(variantNames[i]);
    
    String req = getHandshakeRequest(variants[i]);
    if (handshakeTry(req)) {
      _successfulVariant = variants[i];
      return true;
    }
  }

  return false;
}

String NtripPusher::getHandshakeRequest(HandshakeVariant variant) {
  String req;
  
  switch (variant) {
    case V2_SLASH:
      req = "SOURCE " + _pass + " /" + _mount + "\r\n";
      req += "User-Agent: " + _ua + "\r\n";
      req += "Ntrip-Version: Ntrip/2.0\r\n\r\n";
      break;
      
    case V2_NOSLASH:
      req = "SOURCE " + _pass + " " + _mount + "\r\n";
      req += "User-Agent: " + _ua + "\r\n";
      req += "Ntrip-Version: Ntrip/2.0\r\n\r\n";
      break;
      
    case V1_SLASH:
      req = "SOURCE " + _pass + " /" + _mount + "\r\n";
      req += "Source-Agent: " + _ua + "\r\n\r\n";
      break;
      
    case V1_NOSLASH:
      req = "SOURCE " + _pass + " " + _mount + "\r\n";
      req += "Source-Agent: " + _ua + "\r\n\r\n";
      break;
      
    default:
      break;
  }
  
  return req;
}

bool NtripPusher::begin() {
  stop();
  if (!connectSocket()) return false;
  bool ok = doHandshake();
  if (!ok) { _cli.stop(); }
  return ok;
}

void NtripPusher::stop() {
  if (_cli.connected()) _cli.stop();
}

bool NtripPusher::isConnected() {
  return _cli && _cli.connected();
}

void NtripPusher::loop() {
  // Auto-reconnect if enabled and disconnected
  if (!_autoReconnect) return;

  // Non tentare nulla se WiFi non è up
  if (WiFi.status() != WL_CONNECTED) return;

  // Health check: if connected but no successful write for 30s, force reconnect
  if (_cli.connected() && _lastSuccessfulWriteMs > 0) {
    if (millis() - _lastSuccessfulWriteMs > STALE_TIMEOUT_MS) {
      Serial.println("[NTRIP-PUSH] Connection stale (no successful writes), forcing reconnect");
      _cli.stop();
      _lastSuccessfulWriteMs = 0;
    }
  }

  if (!_cli.connected()) {
    uint32_t now = millis();
    if (now - _lastReconnectAttemptMs >= RECONNECT_DELAY_MS) {
      _lastReconnectAttemptMs = now;

      if (!connectSocket()) return;

      // Prova UNA SOLA variante per ciclo (max blocco: 1s connect + 1s handshake = 2s)
      // invece di 4 varianti in sequenza (che bloccava fino a 48s)
      static const HandshakeVariant varOrder[] = {V2_SLASH, V2_NOSLASH, V1_SLASH, V1_NOSLASH};
      HandshakeVariant toTry;
      if (_successfulVariant != UNKNOWN) {
        // Usa la variante già nota
        toTry = _successfulVariant;
      } else {
        // Round-robin: una variante diversa ad ogni ciclo
        toTry = varOrder[_loopVariantIdx % 4];
        _loopVariantIdx++;
      }

      Serial.printf("[NTRIP-PUSH] Reconnecting (variant %d)...\n", (int)toTry);
      if (handshakeTry(getHandshakeRequest(toTry))) {
        _successfulVariant = toTry;
        _loopVariantIdx = 0;
        _lastSuccessfulWriteMs = millis();
        Serial.println("[NTRIP-PUSH] Reconnected successfully");
      } else {
        if (_successfulVariant != UNKNOWN) {
          // La variante nota ha fallito: reset e riprova dall'inizio
          _successfulVariant = UNKNOWN;
          _loopVariantIdx = 0;
        }
        _cli.stop();
      }
    }
  }
}

void NtripPusher::write(const uint8_t* data, size_t len) {
  if (!_cli.connected()) return;
  if (!len || !data) {
    Serial.println("[NTRIP-PUSH] Write called with invalid parameters");
    return;
  }
  
  size_t written = _cli.write(data, len);
  if (written == 0) {
    Serial.println("[NTRIP-PUSH] Write failed, forcing disconnect");
    _cli.stop();
    _lastSuccessfulWriteMs = 0;  // Force health check to trigger reconnect
  } else {
    _lastSuccessfulWriteMs = millis();
    if (written < len) {
      Serial.printf("[NTRIP-PUSH] Partial write: %u/%u bytes\n", (unsigned)written, (unsigned)len);
    }
  }
}

void NtripPusher::forceReconnect() {
  Serial.println("[NTRIP-PUSH] Force reconnect requested");
  if (_cli.connected()) {
    _cli.stop();
  }
  _lastSuccessfulWriteMs = 0;
  _lastReconnectAttemptMs = 0;  // Allow immediate reconnection attempt
}
