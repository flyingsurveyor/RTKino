// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "RtcmStreamer.h"

WiFiServer RtcmStreamer::_srv(2102);
WiFiClient RtcmStreamer::_cli;
bool RtcmStreamer::_on = false;
SemaphoreHandle_t RtcmStreamer::_mutex = nullptr;
bool RtcmStreamer::_initialized = false;

void RtcmStreamer::ensureMutex() {
  if (!_mutex) {
    _mutex = xSemaphoreCreateMutex();
  }
}

bool RtcmStreamer::lock(uint32_t timeoutMs) {
  return (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

void RtcmStreamer::unlock() {
  if (_mutex) xSemaphoreGive(_mutex);
}

void RtcmStreamer::begin(uint16_t port) {
  ensureMutex();
  
  if (lock(100)) {
    if (!_initialized) {
      _srv = WiFiServer(port);
      _srv.begin();
      _initialized = true;
    }
    unlock();
  }
}

void RtcmStreamer::stop() {
  if (lock(100)) {
    if (_cli) _cli.stop();
    _on = false;
    _initialized = false;
    unlock();
  }
}

void RtcmStreamer::setActive(bool on) {
  ensureMutex();
  
  if (lock(100)) {
    _on = on;
    if (!on && _cli) _cli.stop();
    unlock();
  }
}

bool RtcmStreamer::active() { 
  return _on; 
}

bool RtcmStreamer::hasClient() {
  if (!lock(5)) return false;
  bool connected = _cli && _cli.connected();
  unlock();
  return connected;
}

void RtcmStreamer::handle() {
  if (!_on || !_initialized) return;
  if (!lock(10)) return;
  
  if (!_cli || !_cli.connected()) {
    WiFiClient newClient = _srv.available();
    if (newClient) {
      if (_cli) {
        _cli.stop();
      }
      _cli = newClient;
      _cli.setNoDelay(true);
      _cli.setTimeout(5000);  // 5 second timeout for socket operations
    }
  }
  
  unlock();
}

void RtcmStreamer::broadcast(const uint8_t* data, size_t len) {
  if (!_on || !_initialized || !data || !len) return;
  if (!lock(5)) return;
  
  if (_cli && _cli.connected()) {
    size_t written = _cli.write(data, len);
    if (written == 0) {
      // Write failed, disconnect client
      Serial.println("[RTCM-OUT] Write failed, disconnecting client");
      _cli.stop();
    } else if (written < len) {
      Serial.printf("[RTCM-OUT] Partial write: %u/%u bytes\n", (unsigned)written, (unsigned)len);
    }
  }
  
  unlock();
}
