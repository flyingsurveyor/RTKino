// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
// Bluetooth Low Energy Serial wrapper using Nordic UART Service (NUS)
// Compatible with SW Maps, GNSS Master, Lefebure NTRIP Client

#ifndef BLE_SERIAL_H
#define BLE_SERIAL_H

#include <Arduino.h>
#include <NimBLEDevice.h>

class BLESerial {
public:
    // Initialize BLE with device name (max 20 characters) and optional passkey
    static void begin(const char* deviceName = "RTKino", uint32_t passkey = 123456);
    
    // Stop BLE and clean up resources
    static void end();
    
    // Minimal housekeeping (called from loop)
    static void loop();
    
    // Output (ESP32 → App) - NMEA/UBX data
    static void write(const uint8_t* data, size_t len);
    
    // Input (App → ESP32) - RTCM corrections callback
    static void setRxCallback(void (*callback)(const uint8_t* data, size_t len));
    
    // Connection status
    static bool isConnected();
    
    // Get connected client name (or empty string if not connected)
    static String getClientName();
    
private:
    static NimBLEServer* pServer;
    static NimBLECharacteristic* pTxChar;
    static NimBLECharacteristic* pRxChar;
    static bool clientConnected;
    static void (*rxCallback)(const uint8_t*, size_t);
    
    // Server callbacks
    class ServerCallbacks : public NimBLEServerCallbacks {
        void onConnect(NimBLEServer* pServer) override;
        void onDisconnect(NimBLEServer* pServer) override;
        void onAuthenticationComplete(ble_gap_conn_desc* desc) override;
        uint32_t onPassKeyRequest() override;
    };
    
    // RX characteristic callback
    class RxCallbacks : public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic* pCharacteristic) override;
    };
};

#endif // BLE_SERIAL_H
