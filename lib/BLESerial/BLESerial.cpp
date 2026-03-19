// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
// Bluetooth Low Energy Serial wrapper using Nordic UART Service (NUS)

#include "BLESerial.h"

// Nordic UART Service (NUS) UUIDs
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Write from app
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Notify to app

// Global for callbacks
static uint32_t g_blePasskey = 123456;

// Static members
NimBLEServer* BLESerial::pServer = nullptr;
NimBLECharacteristic* BLESerial::pTxChar = nullptr;
NimBLECharacteristic* BLESerial::pRxChar = nullptr;
bool BLESerial::clientConnected = false;
void (*BLESerial::rxCallback)(const uint8_t*, size_t) = nullptr;

// Server callbacks implementation
void BLESerial::ServerCallbacks::onConnect(NimBLEServer* pServer) {
    BLESerial::clientConnected = true;
    Serial.println("[BLE] Client connecting...");
    
    // Update connection parameters for better throughput
    // This requires the connection handle which we can get via pServer
    // For simplicity, we'll rely on default connection parameters
}

void BLESerial::ServerCallbacks::onDisconnect(NimBLEServer* pServer) {
    BLESerial::clientConnected = false;
    Serial.println("[BLE] Client disconnected");
    
    // Brief delay before restarting advertising to give WiFi stack breathing room
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Restart advertising for auto-reconnect
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising restarted");
}

void BLESerial::ServerCallbacks::onAuthenticationComplete(ble_gap_conn_desc* desc) {
    if (desc->sec_state.encrypted) {
        Serial.println("[BLE] ✓ Pairing successful (encrypted)");
    } else {
        Serial.println("[BLE] ✗ Pairing failed");
    }
}

uint32_t BLESerial::ServerCallbacks::onPassKeyRequest() {
    Serial.printf("[BLE] Passkey requested: %06u\n", g_blePasskey);
    return g_blePasskey;
}

// RX characteristic callbacks implementation
void BLESerial::RxCallbacks::onWrite(NimBLECharacteristic* pCharacteristic) {
    // Get received data
    std::string value = pCharacteristic->getValue();
    
    if (value.length() > 0 && BLESerial::rxCallback != nullptr) {
        // Call user callback with received RTCM data
        BLESerial::rxCallback((const uint8_t*)value.data(), value.length());
    }
}

void BLESerial::begin(const char* deviceName, uint32_t passkey) {
    Serial.printf("[BLE] Initializing as '%s'...\n", deviceName);
    
    // Store passkey for callbacks
    g_blePasskey = passkey;
    
    // Initialize NimBLE
    NimBLEDevice::init(deviceName);
    
    // Set transmit power to medium level for coexistence with WiFi (~0 dBm)
    NimBLEDevice::setPower(ESP_PWR_LVL_P3);
    
    // Configure security
    NimBLEDevice::setSecurityAuth(true, false, true);  
    // Parameters: bonding=true, mitm=false, secure_conn=true
    
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);  
    // Display-only mode (ESP32 "displays" PIN, user enters in app)
    
    NimBLEDevice::setSecurityPasskey(passkey);
    
    Serial.printf("[BLE] Security: Passkey = %06u\n", passkey);
    
    // Create BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create Nordic UART Service
    NimBLEService* pService = pServer->createService(SERVICE_UUID);
    
    // Create TX Characteristic (notify to app)
    pTxChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        NIMBLE_PROPERTY::NOTIFY
    );
    
    // Create RX Characteristic (write from app)
    pRxChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRxChar->setCallbacks(new RxCallbacks());
    
    // Start service
    pService->start();
    
    // Configure advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    
    // Start advertising
    pAdvertising->start();
    
    Serial.println("[BLE] Started and advertising");
}

void BLESerial::end() {
    Serial.println("[BLE] Stopping...");
    
    if (pServer) {
        // Disconnect any connected clients (0 = disconnect all)
        if (clientConnected) {
            pServer->disconnect(0);
        }
        
        // Stop advertising
        NimBLEDevice::stopAdvertising();
    }
    
    // Deinitialize NimBLE
    NimBLEDevice::deinit(true);
    
    pServer = nullptr;
    pTxChar = nullptr;
    pRxChar = nullptr;
    clientConnected = false;
    rxCallback = nullptr;
    
    Serial.println("[BLE] Stopped");
}

void BLESerial::loop() {
    // Minimal housekeeping - NimBLE is event-driven
    // Could add periodic tasks here if needed
}

void BLESerial::write(const uint8_t* data, size_t len) {
    if (!clientConnected || !pTxChar || len == 0) {
        return;
    }
    
    // BLE has MTU limitations (typically 512 bytes max per notify)
    // Limit to single chunk per call to avoid blocking the task
    const size_t MAX_CHUNK = 512;
    size_t chunkSize = (len > MAX_CHUNK) ? MAX_CHUNK : len;
    
    // Send chunk via notification
    pTxChar->setValue(data, chunkSize);
    pTxChar->notify();
    
    // Brief yield to FreeRTOS scheduler to give WiFi stack time to run
    vTaskDelay(pdMS_TO_TICKS(1));
}

void BLESerial::setRxCallback(void (*callback)(const uint8_t* data, size_t len)) {
    rxCallback = callback;
}

bool BLESerial::isConnected() {
    return clientConnected;
}

String BLESerial::getClientName() {
    if (!clientConnected || !pServer) {
        return "";
    }
    
    // Get connected client count
    if (pServer->getConnectedCount() > 0) {
        return "Connected";  // NimBLE doesn't easily expose client name
    }
    
    return "";
}
