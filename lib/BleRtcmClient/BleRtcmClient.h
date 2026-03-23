// SPDX-License-Identifier: AGPL-3.0-or-later
// BLE Central client for rtcm-lora: RTCM receive (rover) + RTCM send (base)
// Coexists with BLESerial (Peripheral) for simultaneous smartphone + radio link.

#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

#define BLE_RTCM_SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_RTCM_TX_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_RTCM_RX_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_NUS_SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_RX_CHAR_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_TX_CHAR_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static constexpr uint8_t BLE_CMD_START = 0x01;
static constexpr uint8_t BLE_CMD_STOP  = 0x02;

class BleRtcmClient {
public:
    void setRxCallback(void (*cb)(const uint8_t* data, size_t len));

    bool begin(const char* targetName = "", uint32_t passkey = 123456);
    void stop();
    void loop();

    // Commands to rtcm-lora
    bool sendCommand(uint8_t cmd);
    bool sendStart() { return sendCommand(BLE_CMD_START); }
    bool sendStop()  { return sendCommand(BLE_CMD_STOP); }

    // Base mode: write RTCM to rtcm-lora via NUS
    size_t writeRtcm(const uint8_t* data, size_t len);

    // Manual scan trigger (from WebUI)
    void triggerScan();

    // State
    bool isConnected() const { return _connected; }
    bool isScanning()  const { return _scanning; }
    bool isEnabled()   const { return _enabled; }
    bool isStreaming()  const { return _streaming; }
    bool isReconnecting() const { return _needsReconnect; }

    // Stats
    uint32_t getRxBytes()  const { return _rxBytes; }
    uint32_t getRxChunks() const { return _rxChunks; }
    uint32_t getTxBytes()  const { return _txBytes; }
    uint32_t getTxChunks() const { return _txChunks; }
    void resetStats() { _rxBytes = 0; _rxChunks = 0; _txBytes = 0; _txChunks = 0; }

    const String& connectedName() const { return _foundName; }

private:
    void (*_rxCallback)(const uint8_t*, size_t) = nullptr;

    NimBLEClient*               _pClient = nullptr;
    NimBLERemoteCharacteristic* _pTxRemote = nullptr;
    NimBLERemoteCharacteristic* _pRxRemote = nullptr;
    NimBLERemoteCharacteristic* _pNusRxRemote = nullptr;

    volatile bool _connected = false;
    volatile bool _scanning  = false;
    bool          _enabled   = false;
    bool          _streaming = false;

    // Auto-reconnect
    bool          _needsReconnect = false;
    uint32_t      _reconnectAfterMs = 0;
    uint8_t       _reconnectCount = 0;

    String        _targetName;
    String        _foundName;
    uint32_t      _passkey = 123456;
    NimBLEAddress _targetAddr;
    bool          _haveTarget = false;

    uint32_t _lastScanMs = 0;
    uint32_t _rxBytes = 0;
    uint32_t _rxChunks = 0;
    uint32_t _txBytes = 0;
    uint32_t _txChunks = 0;

    // Scan less aggressively to coexist with BLESerial (Peripheral)
    static constexpr uint32_t SCAN_INTERVAL_MS  = 8000;   // scan every 8s (was 5)
    static constexpr uint32_t SCAN_DURATION_SEC = 2;       // scan for 2s (was 4)

    void onDeviceFound(NimBLEAdvertisedDevice* dev);
    bool doConnect();

    class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
    public:
        BleRtcmClient* parent = nullptr;
        void onResult(NimBLEAdvertisedDevice* dev) override;
    };

    class ClientCB : public NimBLEClientCallbacks {
    public:
        BleRtcmClient* parent = nullptr;
        void onConnect(NimBLEClient* c) override;
        void onDisconnect(NimBLEClient* c) override;
        uint32_t onPassKeyRequest() override;
        void onAuthenticationComplete(ble_gap_conn_desc* desc) override;
    };

    ScanCB   _scanCB;
    ClientCB _clientCB;

    static BleRtcmClient* _instance;
    static void notifyCB(NimBLERemoteCharacteristic* pChar,
                         uint8_t* pData, size_t length, bool isNotify);
};
