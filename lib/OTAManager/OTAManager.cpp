// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "OTAManager.h"

// Inizializzazione variabili statiche
OTAManager::ProgressCallback OTAManager::_progressCallback = nullptr;
OTAManager::ErrorCallback OTAManager::_errorCallback = nullptr;
bool OTAManager::_initialized = false;
String OTAManager::_lastUpdateStatus = "Not started";
size_t OTAManager::_updateProgress = 0;
size_t OTAManager::_updateTotal = 0;

bool OTAManager::begin() {
    if (_initialized) {
        Serial.println("[OTA] Already initialized");
        return true;
    }
    
    Serial.println("[OTA] Initializing OTA Manager (Web Upload only)...");
    
    _initialized = true;
    _lastUpdateStatus = "Ready";
    
    Serial.printf("[OTA] Current partition: %s\n", getCurrentPartitionInfo().c_str());
    Serial.printf("[OTA] Next partition: %s\n", getNextPartitionInfo().c_str());
    
    return true;
}

void OTAManager::setProgressCallback(ProgressCallback cb) {
    _progressCallback = cb;
}

void OTAManager::setErrorCallback(ErrorCallback cb) {
    _errorCallback = cb;
}

bool OTAManager::beginWebUpdate(size_t updateSize) {
    // NOTE:
    // With ESP32 WebServer, upload.totalSize is often 0 at UPLOAD_FILE_START.
    // Support updateSize==0 by using UPDATE_SIZE_UNKNOWN and deferring exact sizing.
    if (!hasSpaceForUpdate(updateSize)) {
        _lastUpdateStatus = "Error: Not enough space";
        if (_errorCallback) _errorCallback("Not enough space for update");
        return false;
    }
    
    Serial.printf("[OTA] Starting web update, size: %u bytes\n", updateSize);
    _updateProgress = 0;
    _updateTotal = updateSize; // may be 0 (unknown)
    _lastUpdateStatus = "Uploading...";
    
    // Inizia l'aggiornamento
    const size_t beginSize = (updateSize == 0) ? UPDATE_SIZE_UNKNOWN : updateSize;
    if (!Update.begin(beginSize, U_FLASH)) {
        _lastUpdateStatus = "Error: Update.begin failed - " + String(Update.errorString());
        if (_errorCallback) _errorCallback(Update.errorString());
        return false;
    }
    
    return true;
}

bool OTAManager::writeWebUpdate(uint8_t* data, size_t len) {
    if (!Update.isRunning()) {
        _lastUpdateStatus = "Error: Update not running";
        if (_errorCallback) _errorCallback("Update not running");
        return false;
    }
    
    size_t written = Update.write(data, len);
    if (written != len) {
        _lastUpdateStatus = "Error: Write failed - " + String(Update.errorString());
        if (_errorCallback) _errorCallback(Update.errorString());
        return false;
    }
    
    _updateProgress += written;
    
    // Callback progresso
    if (_progressCallback) {
        _progressCallback(_updateProgress, _updateTotal);
    }
    
// Log periodico ogni 10% (solo se size nota)
if (_updateTotal > 0) {
    static size_t lastLogProgress = 0;
    if (_updateProgress <= len) lastLogProgress = 0; // reset on new upload session
    size_t progressPercent = (_updateProgress * 100) / _updateTotal;
    if (progressPercent >= lastLogProgress + 10) {
        Serial.printf("[OTA] Progress: %u%%\n", progressPercent);
        lastLogProgress = progressPercent;
    }
}
    
    return true;
}

bool OTAManager::endWebUpdate() {
    if (!Update.isRunning()) {
        _lastUpdateStatus = "Error: Update not running";
        if (_errorCallback) _errorCallback("Update not running");
        return false;
    }
    
    if (Update.end(true)) { // true = set boot partition
        Serial.println("[OTA] Update SUCCESS!");
        _lastUpdateStatus = "Success - Rebooting...";
        
if (_progressCallback) {
    // If total was unknown, report "complete" as progress==total.
    const size_t total = (_updateTotal > 0) ? _updateTotal : _updateProgress;
    _progressCallback(total, total);
}
       
        return true;
    } else {
        _lastUpdateStatus = "Error: Update.end failed - " + String(Update.errorString());
        if (_errorCallback) _errorCallback(Update.errorString());
        return false;
    }
}

String OTAManager::getCurrentPartitionInfo() {
    const esp_partition_t* partition = esp_ota_get_running_partition();
    if (partition) {
        char info[128];
        snprintf(info, sizeof(info), "%s @ 0x%x (size: %u KB)", 
                 partition->label, 
                 partition->address,
                 partition->size / 1024);
        return String(info);
    }
    return "Unknown";
}

String OTAManager::getNextPartitionInfo() {
    const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
    if (partition) {
        char info[128];
        snprintf(info, sizeof(info), "%s @ 0x%x (size: %u KB)", 
                 partition->label, 
                 partition->address,
                 partition->size / 1024);
        return String(info);
    }
    return "Unknown";
}

bool OTAManager::hasSpaceForUpdate(size_t updateSize) {
    const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
    if (!partition) {
        Serial.println("[OTA] ERROR: No OTA partition available!");
        return false;
    }
    
// If updateSize is unknown (0), just ensure we have a valid OTA partition.
bool hasSpace = (updateSize == 0) ? (partition->size > 0) : (partition->size >= updateSize);
Serial.printf("[OTA] Space check: update=%u, available=%u, OK=%s\n", 
              updateSize, partition->size, hasSpace ? "YES" : "NO");
    
    return hasSpace;
}

String OTAManager::getLastUpdateStatus() {
    return _lastUpdateStatus;
}

void OTAManager::reboot() {
    Serial.println("[OTA] Rebooting in 2 seconds...");
    delay(2000);
    ESP.restart();
}
