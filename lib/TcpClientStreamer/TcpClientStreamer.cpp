// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "TcpClientStreamer.h"

String TcpClientStreamer::_host = "";
uint16_t TcpClientStreamer::_port = 0;
WiFiClient TcpClientStreamer::_client;
volatile bool TcpClientStreamer::_active = false;
volatile bool TcpClientStreamer::_initialized = false;
SemaphoreHandle_t TcpClientStreamer::_mutex = nullptr;
uint32_t TcpClientStreamer::_lastReconnectAttemptMs = 0;
uint32_t TcpClientStreamer::_lastSuccessfulWriteMs = 0;

void TcpClientStreamer::ensureMutex() {
    if (!_mutex) {
        _mutex = xSemaphoreCreateMutex();
    }
}

bool TcpClientStreamer::lock(uint32_t timeoutMs) {
    return (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

void TcpClientStreamer::unlock() {
    if (_mutex) xSemaphoreGive(_mutex);
}

bool TcpClientStreamer::connectToServer() {
    if (_client.connected()) {
        return true;
    }
    
    Serial.printf("[TCP-CLIENT] Connecting to %s:%d...\n", _host.c_str(), _port);
    
    if (!_client.connect(_host.c_str(), _port)) {
        Serial.println("[TCP-CLIENT] Connection failed");
        return false;
    }
    
    _client.setNoDelay(true);
    _client.setTimeout(10000);  // 10 second timeout
    _lastSuccessfulWriteMs = millis();
    
    Serial.println("[TCP-CLIENT] Connected successfully");
    return true;
}

bool TcpClientStreamer::begin(const String& host, uint16_t port) {
    ensureMutex();
    
    if (!lock(1000)) {
        return false;
    }
    
    _host = host;
    _port = port;
    _initialized = true;
    
    bool connected = connectToServer();
    
    unlock();
    return connected;
}

void TcpClientStreamer::handle() {
    if (!_active || !_initialized) return;
    if (!lock(10)) return;
    
    uint32_t now = millis();
    
    // Check if connection is still alive
    if (!_client.connected()) {
        Serial.println("[TCP-CLIENT] Disconnected, attempting reconnect...");
        
        // Throttle reconnection attempts
        if (now - _lastReconnectAttemptMs >= RECONNECT_DELAY_MS) {
            _lastReconnectAttemptMs = now;
            
            if (_client) {
                _client.stop();
            }
            
            if (connectToServer()) {
                Serial.println("[TCP-CLIENT] Reconnected successfully");
            }
        }
    } else {
        // Check for stale connection (no successful writes for a while)
        if (now - _lastSuccessfulWriteMs > STALE_TIMEOUT_MS) {
            Serial.println("[TCP-CLIENT] Connection stale, forcing reconnect...");
            _client.stop();
            _lastReconnectAttemptMs = now - RECONNECT_DELAY_MS; // Allow immediate reconnect
        }
    }
    
    unlock();
}

void TcpClientStreamer::broadcast(const uint8_t* data, size_t len) {
    if (!_active || !_initialized || !data || !len) return;
    if (!lock(5)) return;
    
    if (_client && _client.connected()) {
        size_t written = _client.write(data, len);
        if (written > 0) {
            _lastSuccessfulWriteMs = millis();
        }
        
        if (written == 0) {
            Serial.println("[TCP-CLIENT] Write failed, marking for reconnect");
            _client.stop();
        } else if (written < len) {
            Serial.printf("[TCP-CLIENT] Partial write: %u/%u bytes\n", (unsigned)written, (unsigned)len);
            _lastSuccessfulWriteMs = millis();  // Partial write still counts as activity
        }
    }
    
    unlock();
}

void TcpClientStreamer::enable(bool on) {
    ensureMutex();
    
    if (!lock(100)) return;
    
    _active = on;
    if (!_active && _client) {
        _client.stop();
    }
    
    unlock();
}

bool TcpClientStreamer::isEnabled() {
    return _active;
}

bool TcpClientStreamer::isConnected() {
    bool connected = false;
    if (lock(10)) {
        connected = _client.connected();
        unlock();
    }
    return connected;
}

void TcpClientStreamer::forceReconnect() {
    if (!lock(100)) return;
    
    if (_client) {
        _client.stop();
    }
    _lastReconnectAttemptMs = 0;  // Allow immediate reconnect on next handle()
    
    unlock();
}

void TcpClientStreamer::stop() {
    if (!lock(100)) return;
    
    _active = false;
    _initialized = false;
    if (_client) {
        _client.stop();
    }
    
    unlock();
}
