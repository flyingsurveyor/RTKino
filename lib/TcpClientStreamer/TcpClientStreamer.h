// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef TCPCLIENTSTREAMER_H
#define TCPCLIENTSTREAMER_H

#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class TcpClientStreamer {
public:
    static bool begin(const String& host, uint16_t port);
    static void handle();  // Auto-reconnect logic
    static void broadcast(const uint8_t* data, size_t len);
    static void enable(bool on);
    static bool isEnabled();
    static bool isConnected();
    static void stop();
    static void forceReconnect();  // Force reconnection after WiFi recovery

private:
    static String _host;
    static uint16_t _port;
    static WiFiClient _client;
    static volatile bool _active;
    static volatile bool _initialized;
    static SemaphoreHandle_t _mutex;
    
    // Auto-reconnect state
    static uint32_t _lastReconnectAttemptMs;
    static const uint32_t RECONNECT_DELAY_MS = 5000;
    
    // Health check tracking
    static uint32_t _lastSuccessfulWriteMs;
    static const uint32_t STALE_TIMEOUT_MS = 30000;  // 30 seconds without successful write
    
    static void ensureMutex();
    static bool lock(uint32_t timeoutMs = 10);
    static void unlock();
    static bool connectToServer();
};

#endif
