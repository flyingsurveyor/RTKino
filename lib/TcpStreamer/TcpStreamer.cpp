// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "TcpStreamer.h"

WiFiServer TcpStreamer::server(7856);
WiFiClient TcpStreamer::client;
volatile bool TcpStreamer:: active = false;
SemaphoreHandle_t TcpStreamer::mutex = nullptr;
volatile bool TcpStreamer::initialized = false;
uint32_t TcpStreamer::lastSuccessfulWrite = 0;
uint32_t TcpStreamer::totalBytes = 0;
uint32_t TcpStreamer::droppedBytes = 0;
uint32_t TcpStreamer::lastDropWarning = 0;

void TcpStreamer::ensureMutex() {
    if (!mutex) {
        mutex = xSemaphoreCreateMutex();
    }
}

bool TcpStreamer::lock(uint32_t timeoutMs) {
    return (mutex && xSemaphoreTake(mutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

void TcpStreamer:: unlock() {
    if (mutex) xSemaphoreGive(mutex);
}

void TcpStreamer::begin(uint16_t port) {
    ensureMutex();

    if (lock(1000)) {
        if (!initialized) {
            server = WiFiServer(port);
            server.begin();
            initialized = true;
            lastSuccessfulWrite = millis();  // Initialize watchdog timestamp
        }
        unlock();
    } else {
        // ✅ CRITICAL: Lock failed, force initialization anyway
        // This prevents broadcast() from failing due to !initialized
        if (!initialized) {
            server = WiFiServer(port);
            server.begin();
            initialized = true;
            lastSuccessfulWrite = millis();
        }
    }
}

void TcpStreamer::handle() {
    if (!active || !initialized) return;
    if (!lock(10)) return;
    
    if (! client || !client.connected()) {
        WiFiClient newClient = server.available();
        if (newClient) {
            if (client) {
                client.stop();
            }
            client = newClient;
            client.setNoDelay(true);
            client.setTimeout(10000);  // 10 second timeout for socket operations
            lastSuccessfulWrite = millis();  // Reset watchdog for new client
        }
    } else {
        // Watchdog: check if client has timed out (no successful write for >STALE_TIMEOUT_MS)
        // Note: millis() overflow is handled correctly by unsigned arithmetic
        if (millis() - lastSuccessfulWrite > STALE_TIMEOUT_MS) {
            Serial.println("[TCP-OUT] Client timeout, disconnecting");
            forceDisconnect();
        }
    }
    
    unlock();
}

void TcpStreamer::broadcast(const uint8_t* data, size_t len) {
    if (!active || !initialized || !data || !len) return;
    if (!lock(10)) return;
    
    if (client && client.connected()) {
        // client.write() is NON-BLOCKING - always attempt write
        // TCP stack handles flow control and buffering internally
        size_t written = client.write(data, len);
        
        if (written == len) {
            // Complete write - success
            totalBytes += len;
            lastSuccessfulWrite = millis();  // Reset watchdog
        } else if (written == 0) {
            // No bytes written - client is completely blocked, disconnect
            Serial.println("[TCP-OUT] Write failed (0 bytes), disconnecting client");
            forceDisconnect();
        } else {
            // Partial write - client is slow but accepting data
            // This is normal behavior when client buffer is partially full
            totalBytes += written;
            droppedBytes += (len - written);
            lastSuccessfulWrite = millis();  // Reset watchdog - client is still responsive
            
            // Throttled warning (max once per 5 seconds)
            uint32_t now = millis();
            if (now - lastDropWarning > 5000) {
                Serial.printf("[TCP-OUT] Partial write: %u/%u bytes\n", (unsigned)written, (unsigned)len);
                lastDropWarning = now;
            }
        }
    }
    
    unlock();
}

void TcpStreamer::enable(bool on) {
    ensureMutex();
    
    // ✅ CRITICAL: Set active BEFORE attempting lock to ensure broadcast() works
    // Even if lock fails, we need active=true for broadcast() to function
    active = on;
    
    if (lock(100)) {
        if (active) {
            lastSuccessfulWrite = millis();  // Initialize watchdog when enabled
        } else if (client) {
            client.stop();
        }
        unlock();
    } else if (active) {
        // Lock failed but active=true, initialize watchdog anyway
        lastSuccessfulWrite = millis();
    }
}

bool TcpStreamer::isEnabled() {
    return active;
}

void TcpStreamer::stop() {
    if (lock(100)) {
        active = false;
        initialized = false;
        if (client) {
            client.stop();
        }
        unlock();
    }
}

void TcpStreamer::forceDisconnect() {
    // Force disconnect without graceful shutdown to avoid delays
    if (client) {
        client.stop();
    }
}

void TcpStreamer::getStats(uint32_t& total, uint32_t& dropped, float& lossPercent) {
    // Use longer timeout (100ms) for stats API - less time-critical than broadcast
    if (!lock(100)) {
        // If we can't get the lock, return zeros
        total = 0;
        dropped = 0;
        lossPercent = 0.0f;
        return;
    }
    
    total = totalBytes;
    dropped = droppedBytes;
    
    if (totalBytes + droppedBytes > 0) {
        lossPercent = (float)droppedBytes / (float)(totalBytes + droppedBytes) * 100.0f;
    } else {
        lossPercent = 0.0f;
    }
    
    unlock();
}

void TcpStreamer::resetStats() {
    // Use longer timeout (100ms) for stats API - less time-critical than broadcast
    if (lock(100)) {
        totalBytes = 0;
        droppedBytes = 0;
        unlock();
    }
}