// SPDX-License-Identifier: AGPL-3.0-or-later
// BLE Central client for rtcm-lora: RTCM receive (rover) + RTCM send (base)
// Coexists with BLESerial (Peripheral) for simultaneous smartphone + radio link.

#include "BleRtcmClient.h"
#include "BLESerial.h"  // to check if smartphone just connected

BleRtcmClient* BleRtcmClient::_instance = nullptr;

// ---- ScanCB ----

void BleRtcmClient::ScanCB::onResult(NimBLEAdvertisedDevice* dev) {
    if (!parent || !dev) return;

    String advName = dev->getName().c_str();

    // Match by target name (primary) or NUS UUID (fallback)
    bool nameMatch = (parent->_targetName.length() > 0 && advName.length() > 0 &&
                      advName == parent->_targetName);
    bool nusMatch = dev->isAdvertisingService(NimBLEUUID(BLE_NUS_SERVICE_UUID));

    if (!nameMatch && !nusMatch) return;
    if (!nameMatch && nusMatch && parent->_targetName.length() > 0) return;

    Serial.printf("[BLE-RTCM] Found: '%s' rssi=%d\n", advName.c_str(), dev->getRSSI());
    parent->onDeviceFound(dev);
}

// ---- ClientCB ----

void BleRtcmClient::ClientCB::onConnect(NimBLEClient* c) {
    if (!parent) return;
    parent->_connected = true;
    parent->_reconnectCount = 0;  // reset backoff on successful connect
    Serial.println("[BLE-RTCM] Connected");
}

void BleRtcmClient::ClientCB::onDisconnect(NimBLEClient* c) {
    if (!parent) return;
    bool wasStreaming = parent->_streaming;
    parent->_connected = false;
    parent->_streaming = false;
    parent->_pTxRemote = nullptr;
    parent->_pRxRemote = nullptr;
    parent->_pNusRxRemote = nullptr;
    parent->_haveTarget = false;

    // Schedule auto-reconnect if we were enabled (not user-stopped)
    if (parent->_enabled) {
        parent->_needsReconnect = true;
        // Backoff: 2s, 4s, 8s, max 15s
        uint32_t backoff = 2000 * (1 << min((int)parent->_reconnectCount, 3));
        if (backoff > 15000) backoff = 15000;
        parent->_reconnectAfterMs = millis() + backoff;
        parent->_reconnectCount++;
        Serial.printf("[BLE-RTCM] Disconnected (was%s streaming). Reconnect in %lums\n",
                      wasStreaming ? "" : " not", (unsigned long)backoff);
    } else {
        parent->_needsReconnect = false;
        Serial.println("[BLE-RTCM] Disconnected (stopped)");
    }
}

uint32_t BleRtcmClient::ClientCB::onPassKeyRequest() {
    if (!parent) return 123456;
    Serial.printf("[BLE-RTCM] PIN: %06u\n", parent->_passkey);
    return parent->_passkey;
}

void BleRtcmClient::ClientCB::onAuthenticationComplete(ble_gap_conn_desc* desc) {
    Serial.println(desc->sec_state.encrypted ? "[BLE-RTCM] Paired OK" : "[BLE-RTCM] Pair FAILED");
}

// ---- Notify callback (static) ----

void BleRtcmClient::notifyCB(NimBLERemoteCharacteristic* pChar,
                               uint8_t* pData, size_t length, bool isNotify)
{
    if (!_instance || !pData || length == 0) return;
    _instance->_rxBytes += length;
    _instance->_rxChunks++;
    if (_instance->_rxCallback) {
        _instance->_rxCallback(pData, length);
    }
}

// ---- Public API ----

void BleRtcmClient::setRxCallback(void (*cb)(const uint8_t*, size_t)) {
    _rxCallback = cb;
}

bool BleRtcmClient::begin(const char* targetName, uint32_t passkey) {
    _instance = this;
    _targetName = targetName ? String(targetName) : "";
    _passkey = passkey;
    _enabled = true;
    _connected = false;
    _streaming = false;
    _haveTarget = false;
    _needsReconnect = false;
    _reconnectCount = 0;
    _lastScanMs = 0;
    resetStats();

    _scanCB.parent = this;
    _clientCB.parent = this;

    if (!NimBLEDevice::getInitialized()) {
        NimBLEDevice::init("");
    }

    Serial.printf("[BLE-RTCM] Started, target='%s'\n",
                  _targetName.length() ? _targetName.c_str() : "(any)");
    return true;
}

void BleRtcmClient::stop() {
    _enabled = false;

    if (_streaming && _connected && _pRxRemote) {
        sendStop();
        delay(50);
    }
    _streaming = false;
    _needsReconnect = false;

    // Stop any running scan
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan && pScan->isScanning()) pScan->stop();
    _scanning = false;

    if (_pClient && _connected) _pClient->disconnect();
    if (_pClient) {
        NimBLEDevice::deleteClient(_pClient);
        _pClient = nullptr;
    }

    _connected = false;
    _haveTarget = false;
    _pTxRemote = nullptr;
    _pRxRemote = nullptr;
    _pNusRxRemote = nullptr;
    _foundName = "";

    Serial.println("[BLE-RTCM] Stopped");
}

void BleRtcmClient::loop() {
    if (!_enabled) return;

    // If connected, nothing to do
    if (_connected) return;

    // Auto-reconnect with backoff
    if (_needsReconnect) {
        if ((int32_t)(millis() - _reconnectAfterMs) < 0) return;  // wait for backoff
        _needsReconnect = false;
        _haveTarget = false;
        _lastScanMs = 0;  // force immediate scan
        Serial.println("[BLE-RTCM] Auto-reconnecting...");
    }

    // If we found a target, try connecting
    if (_haveTarget) {
        if (doConnect()) return;
        _haveTarget = false;
    }

    // Don't scan if BLESerial just had a connection event (give Peripheral priority)
    // This prevents Central scanning from disrupting smartphone pairing
    static uint32_t lastPeripheralCheck = 0;
    static bool lastPeripheralState = false;
    bool peripheralNow = BLESerial::isConnected();
    if (peripheralNow != lastPeripheralState) {
        lastPeripheralState = peripheralNow;
        lastPeripheralCheck = millis();
        // Give 3s grace period after peripheral state change
        return;
    }
    if (millis() - lastPeripheralCheck < 3000) return;

    // Periodic scan — short and infrequent to not interfere with Peripheral
    uint32_t now = millis();
    if (now - _lastScanMs < SCAN_INTERVAL_MS) return;
    _lastScanMs = now;

    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (!pScan || pScan->isScanning()) return;

    pScan->setAdvertisedDeviceCallbacks(&_scanCB, false);
    pScan->setActiveScan(true);
    pScan->setInterval(160);   // wider interval = less aggressive
    pScan->setWindow(80);

    _scanning = true;
    pScan->start(SCAN_DURATION_SEC, [](NimBLEScanResults results) {
        if (_instance) _instance->_scanning = false;
    }, false);
}

bool BleRtcmClient::sendCommand(uint8_t cmd) {
    if (!_connected || !_pRxRemote) return false;
    uint8_t buf[1] = { cmd };
    if (!_pRxRemote->writeValue(buf, 1, false)) return false;
    if (cmd == BLE_CMD_START) _streaming = true;
    if (cmd == BLE_CMD_STOP)  _streaming = false;
    Serial.printf("[BLE-RTCM] Cmd 0x%02X sent\n", cmd);
    return true;
}

size_t BleRtcmClient::writeRtcm(const uint8_t* data, size_t len) {
    if (!_connected || !_pNusRxRemote || len == 0) return 0;

    const size_t CHUNK = 240;
    size_t sent = 0;

    while (sent < len) {
        size_t remaining = len - sent;
        size_t chunk = (remaining > CHUNK) ? CHUNK : remaining;
        if (!_pNusRxRemote->writeValue(&data[sent], chunk, false)) {
            Serial.println("[BLE-RTCM] NUS write failed");
            break;
        }
        sent += chunk;
        _txChunks++;
    }
    _txBytes += sent;
    return sent;
}

// ---- Scan trigger (called from WebUI) ----
void BleRtcmClient::triggerScan() {
    if (!_enabled || _connected) return;
    _lastScanMs = 0;  // force next loop() to scan immediately
    _haveTarget = false;
    Serial.println("[BLE-RTCM] Manual scan triggered");
}

// ---- Internal ----

void BleRtcmClient::onDeviceFound(NimBLEAdvertisedDevice* dev) {
    _targetAddr = dev->getAddress();
    _foundName = dev->getName().c_str();
    _haveTarget = true;
    NimBLEDevice::getScan()->stop();
    _scanning = false;
}

bool BleRtcmClient::doConnect() {
    Serial.printf("[BLE-RTCM] Connecting to %s...\n", _targetAddr.toString().c_str());

    if (_pClient) {
        NimBLEDevice::deleteClient(_pClient);
        _pClient = nullptr;
    }

    _pClient = NimBLEDevice::createClient();
    if (!_pClient) return false;

    _pClient->setClientCallbacks(&_clientCB);
    _pClient->setConnectTimeout(5);

    if (!_pClient->connect(_targetAddr)) {
        Serial.println("[BLE-RTCM] Connect failed");
        NimBLEDevice::deleteClient(_pClient);
        _pClient = nullptr;
        return false;
    }

    // Discover RTCM custom service (rover: notifications + commands)
    NimBLERemoteService* pRtcmSvc = _pClient->getService(BLE_RTCM_SERVICE_UUID);
    if (pRtcmSvc) {
        _pTxRemote = pRtcmSvc->getCharacteristic(BLE_RTCM_TX_CHAR_UUID);
        _pRxRemote = pRtcmSvc->getCharacteristic(BLE_RTCM_RX_CHAR_UUID);
        if (_pTxRemote) {
            _pTxRemote->subscribe(true, notifyCB);
            Serial.println("[BLE-RTCM] RTCM notify subscribed");
        }
        if (_pRxRemote) Serial.println("[BLE-RTCM] RTCM cmd channel ready");
    }

    // Discover NUS service (base: write RTCM data to rtcm-lora)
    NimBLERemoteService* pNusSvc = _pClient->getService(BLE_NUS_SERVICE_UUID);
    if (pNusSvc) {
        _pNusRxRemote = pNusSvc->getCharacteristic(BLE_NUS_RX_CHAR_UUID);
        if (_pNusRxRemote) Serial.println("[BLE-RTCM] NUS write channel ready");
    }

    if (!_pTxRemote && !_pNusRxRemote) {
        Serial.println("[BLE-RTCM] No usable services, disconnecting");
        _pClient->disconnect();
        NimBLEDevice::deleteClient(_pClient);
        _pClient = nullptr;
        return false;
    }

    Serial.printf("[BLE-RTCM] OK '%s' [RTCM:%s NUS:%s]\n",
                  _foundName.c_str(),
                  _pTxRemote ? "yes" : "no",
                  _pNusRxRemote ? "yes" : "no");
    return true;
}
