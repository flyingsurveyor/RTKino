// SPDX-License-Identifier: AGPL-3.0-or-later
// BLE Central client for rtcm-lora: RTCM receive (rover) + RTCM send (base)

#include "BleRtcmClient.h"

BleRtcmClient* BleRtcmClient::_instance = nullptr;

// ---- ScanCB ----

void BleRtcmClient::ScanCB::onResult(NimBLEAdvertisedDevice* dev) {
    if (!parent || !dev) return;

    String advName = dev->getName().c_str();

    // Match by target name (primary) or NUS UUID (fallback).
    // RTCM custom UUID is not advertised; discovered after connection.
    bool nameMatch = false;
    if (parent->_targetName.length() > 0 && advName.length() > 0) {
        nameMatch = (advName == parent->_targetName);
    }

    bool nusMatch = dev->isAdvertisingService(NimBLEUUID(BLE_NUS_SERVICE_UUID));

    if (!nameMatch && !nusMatch) return;
    if (!nameMatch && nusMatch && parent->_targetName.length() > 0) return;

    Serial.printf("[BLE-RTCM] Found: '%s' addr=%s rssi=%d\n",
                  advName.c_str(), dev->getAddress().toString().c_str(), dev->getRSSI());

    parent->onDeviceFound(dev);
}

// ---- ClientCB ----

void BleRtcmClient::ClientCB::onConnect(NimBLEClient* c) {
    if (!parent) return;
    parent->_connected = true;
    Serial.println("[BLE-RTCM] Connected");
}

void BleRtcmClient::ClientCB::onDisconnect(NimBLEClient* c) {
    if (!parent) return;
    parent->_connected = false;
    parent->_streaming = false;
    parent->_pTxRemote = nullptr;
    parent->_pRxRemote = nullptr;
    parent->_pNusRxRemote = nullptr;
    parent->_haveTarget = false;
    Serial.println("[BLE-RTCM] Disconnected");
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
    _lastScanMs = 0;
    resetStats();

    _scanCB.parent = this;
    _clientCB.parent = this;

    if (!NimBLEDevice::getInitialized()) {
        NimBLEDevice::init("");
    }
    // Do NOT override security settings here — BLESerial has already
    // configured them for smartphone pairing. The passkey for connecting
    // to rtcm-lora is handled by ClientCB::onPassKeyRequest().

    Serial.printf("[BLE-RTCM] Started, target='%s'\n",
                  _targetName.length() ? _targetName.c_str() : "(any)");
    return true;
}

void BleRtcmClient::stop() {
    if (_streaming && _connected && _pRxRemote) {
        sendStop();
        delay(50);
    }
    _enabled = false;
    _streaming = false;

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
    if (_connected) return;

    if (_haveTarget) {
        if (doConnect()) return;
        _haveTarget = false;
    }

    uint32_t now = millis();
    if (now - _lastScanMs < SCAN_INTERVAL_MS) return;
    _lastScanMs = now;

    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (!pScan || pScan->isScanning()) return;

    pScan->setAdvertisedDeviceCallbacks(&_scanCB, false);
    pScan->setActiveScan(true);
    pScan->setInterval(100);
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

    // Write in chunks that fit BLE MTU (NUS RX is write-no-response)
    const size_t CHUNK = 240;  // safe for MTU 256 (256 - 3 ATT - some margin)
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

    // Discover RTCM custom service (for rover: notifications + commands)
    NimBLERemoteService* pRtcmSvc = _pClient->getService(BLE_RTCM_SERVICE_UUID);
    if (pRtcmSvc) {
        _pTxRemote = pRtcmSvc->getCharacteristic(BLE_RTCM_TX_CHAR_UUID);
        _pRxRemote = pRtcmSvc->getCharacteristic(BLE_RTCM_RX_CHAR_UUID);

        // Subscribe to RTCM notifications (rover mode)
        if (_pTxRemote) {
            _pTxRemote->subscribe(true, notifyCB);
            Serial.println("[BLE-RTCM] Subscribed to RTCM notifications");
        }
        if (_pRxRemote) {
            Serial.println("[BLE-RTCM] RTCM command channel ready");
        }
    } else {
        Serial.println("[BLE-RTCM] RTCM service not found (OK for base-only use)");
    }

    // Discover NUS service (for base: write RTCM data to rtcm-lora)
    NimBLERemoteService* pNusSvc = _pClient->getService(BLE_NUS_SERVICE_UUID);
    if (pNusSvc) {
        _pNusRxRemote = pNusSvc->getCharacteristic(BLE_NUS_RX_CHAR_UUID);
        if (_pNusRxRemote) {
            Serial.println("[BLE-RTCM] NUS write channel ready (base RTCM output)");
        }
    } else {
        Serial.println("[BLE-RTCM] NUS service not found");
    }

    // Need at least one usable service
    if (!_pTxRemote && !_pNusRxRemote) {
        Serial.println("[BLE-RTCM] No usable services found, disconnecting");
        _pClient->disconnect();
        NimBLEDevice::deleteClient(_pClient);
        _pClient = nullptr;
        return false;
    }

    Serial.printf("[BLE-RTCM] Connected to '%s' [RTCM:%s NUS:%s]\n",
                  _foundName.c_str(),
                  _pTxRemote ? "yes" : "no",
                  _pNusRxRemote ? "yes" : "no");
    return true;
}
