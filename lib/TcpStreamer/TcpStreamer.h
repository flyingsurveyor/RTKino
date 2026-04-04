// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef TCPSTREAMER_H
#define TCPSTREAMER_H

#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class TcpStreamer {
public:
    static void begin(uint16_t port = 7856);
    static void handle();
    static void broadcast(const uint8_t* data, size_t len);
    static void enable(bool on);
    static bool isEnabled();
    static void stop();
    
    // Statistics API
    static void getStats(uint32_t& total, uint32_t& dropped, float& lossPercent);
    static void resetStats();

private:
    static WiFiServer server;
    static WiFiClient client;
    static volatile bool active;
    static volatile bool initialized;
    static SemaphoreHandle_t mutex;
    
    // Watchdog and statistics
    static uint32_t lastSuccessfulWrite;
    static uint32_t totalBytes;
    static uint32_t droppedBytes;
    static uint32_t lastDropWarning;
    static const uint32_t STALE_TIMEOUT_MS = 30000;
    
    static void ensureMutex();
    static bool lock(uint32_t timeoutMs = 10);
    static void unlock();
    static void forceDisconnect();
};

#endif