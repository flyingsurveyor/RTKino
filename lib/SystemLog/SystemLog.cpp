// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "SystemLog.h"
#include <time.h>
#include <stdarg.h>
#include <rom/rtc.h>

SystemLog::SystemLog()
    : m_sd(nullptr)
    , m_sdMutex(nullptr)
    , m_initialized(false)
    , m_logQueue(nullptr)
{
}

SystemLog::~SystemLog() {
    end();
}

bool SystemLog::begin(SdFat& sd, SemaphoreHandle_t sdMutex) {
    if (m_initialized) {
        return true;
    }
    
    m_sd = &sd;
    m_sdMutex = sdMutex;
    
    // Create log queue for async logging
    m_logQueue = xQueueCreate(32, sizeof(SystemLogEntry));
    if (!m_logQueue) {
        Serial.println("[SystemLog] Failed to create queue");
        return false;
    }
    
    // Open new log file
    if (!openNewLogFile()) {
        Serial.println("[SystemLog] Failed to open log file");
        return false;
    }
    
    m_initialized = true;
    
    // Log system boot
    RESET_REASON reason = rtc_get_reset_reason(0);
    const char* resetStr = "UNKNOWN";
    switch (reason) {
        case POWERON_RESET: resetStr = "POWERON"; break;
        case RTC_SW_SYS_RESET: resetStr = "SW_RESET"; break;
        case RTCWDT_SYS_RESET: resetStr = "WATCHDOG"; break;
        case DEEPSLEEP_RESET: resetStr = "DEEPSLEEP"; break;
        case TG0WDT_SYS_RESET: resetStr = "TG0WDT_SYS"; break;
        case TG1WDT_SYS_RESET: resetStr = "TG1WDT_SYS"; break;
        case TG0WDT_CPU_RESET: resetStr = "TG0WDT_CPU"; break;
        case TG1WDT_CPU_RESET: resetStr = "TG1WDT_CPU"; break;
        case RTC_SW_CPU_RESET: resetStr = "SW_CPU"; break;
        case RTCWDT_CPU_RESET: resetStr = "RTCWDT_CPU"; break;
        case RTCWDT_BROWN_OUT_RESET: resetStr = "BROWNOUT"; break;
        case RTCWDT_RTC_RESET: resetStr = "RTCWDT_RTC"; break;
        case SUPER_WDT_RESET: resetStr = "SUPER_WDT"; break;
        default: break;
    }
    
    logEvent("SYSTEM", String("BOOT ") + resetStr);
    
    return true;
}

void SystemLog::end() {
    if (!m_initialized) {
        return;
    }
    
    m_initialized = false;
    
    // Flush any pending logs
    loop();
    
    // Close log file
    closeCurrentLog();
    
    // Clean up queue
    if (m_logQueue) {
        vQueueDelete(m_logQueue);
        m_logQueue = nullptr;
    }
}

void SystemLog::log(const char* format, ...) {
    if (!m_initialized || !m_logQueue) {
        return;
    }
    
    SystemLogEntry entry;
    entry.timestamp = millis();
    
    va_list args;
    va_start(args, format);
    vsnprintf(entry.message, sizeof(entry.message), format, args);
    va_end(args);
    
    // Try to add to queue (don't block if full)
    xQueueSend(m_logQueue, &entry, 0);
}

void SystemLog::logEvent(const char* category, const String& message) {
    if (!m_initialized) {
        return;
    }
    
    String timestamp = makeTimestamp();
    String fullMsg = String("[") + timestamp + "] " + category + ": " + message;
    log("%s", fullMsg.c_str());
}

bool SystemLog::listLogFiles(String files[], int& count, int maxFiles) {
    if (!m_sd || !m_sdMutex) {
        return false;
    }
    
    bool locked = (xSemaphoreTake(m_sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE);
    if (!locked) {
        return false;
    }
    
    count = 0;
    FsFile dir = m_sd->open("/gnss");
    if (!dir) {
        xSemaphoreGive(m_sdMutex);
        return false;
    }
    
    FsFile entry;
    while (count < maxFiles && entry.openNext(&dir, O_RDONLY)) {
        char name[64];
        entry.getName(name, sizeof(name));
        String filename = String(name);
        
        if (filename.startsWith("system_log_") && filename.endsWith(".txt")) {
            files[count++] = String("/gnss/") + filename;
        }
        entry.close();
    }
    dir.close();
    
    xSemaphoreGive(m_sdMutex);
    return true;
}

bool SystemLog::deleteLogFile(const String& filename) {
    if (!m_sd || !m_sdMutex) {
        return false;
    }
    
    bool locked = (xSemaphoreTake(m_sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE);
    if (!locked) {
        return false;
    }
    
    bool result = m_sd->remove(filename.c_str());
    
    xSemaphoreGive(m_sdMutex);
    return result;
}

void SystemLog::loop() {
    if (!m_initialized || !m_logQueue) {
        return;
    }
    
    // Process all pending log entries
    SystemLogEntry entry;
    while (xQueueReceive(m_logQueue, &entry, 0) == pdTRUE) {
        writeLogEntry(entry.message);
    }
}

bool SystemLog::openNewLogFile() {
    if (!m_sd || !m_sdMutex) {
        return false;
    }
    
    bool locked = (xSemaphoreTake(m_sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE);
    if (!locked) {
        return false;
    }
    
    // Ensure /gnss directory exists
    m_sd->mkdir("/gnss");
    
    // Find next available log file number
    int logNum = 1;
    for (int i = 1; i <= SYSTEM_LOG_MAX_FILES; i++) {
        char filename[32];
        snprintf(filename, sizeof(filename), "/gnss/system_log_%02d.txt", i);
        if (!m_sd->exists(filename)) {
            logNum = i;
            break;
        }
    }
    
    // If all slots are used, rotate (delete oldest)
    if (logNum > SYSTEM_LOG_MAX_FILES) {
        rotateLogFiles();
        logNum = SYSTEM_LOG_MAX_FILES;
    }
    
    char filename[32];
    snprintf(filename, sizeof(filename), "/gnss/system_log_%02d.txt", logNum);
    m_currentLogFile = String(filename);
    
    m_logFile = m_sd->open(filename, O_WRITE | O_CREAT | O_APPEND);
    
    xSemaphoreGive(m_sdMutex);
    
    return m_logFile.isOpen();
}

void SystemLog::closeCurrentLog() {
    if (!m_sdMutex) {
        return;
    }
    
    bool locked = (xSemaphoreTake(m_sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE);
    if (!locked) {
        return;
    }
    
    if (m_logFile.isOpen()) {
        m_logFile.sync();
        m_logFile.close();
    }
    
    xSemaphoreGive(m_sdMutex);
}

void SystemLog::rotateLogFiles() {
    if (!m_sd) {
        return;
    }
    
    // Delete the oldest file (01)
    m_sd->remove("/gnss/system_log_01.txt");
    
    // Shift files down
    for (int i = 1; i < SYSTEM_LOG_MAX_FILES; i++) {
        char oldName[32], newName[32];
        snprintf(oldName, sizeof(oldName), "/gnss/system_log_%02d.txt", i + 1);
        snprintf(newName, sizeof(newName), "/gnss/system_log_%02d.txt", i);
        
        if (m_sd->exists(oldName)) {
            m_sd->rename(oldName, newName);
        }
    }
}

void SystemLog::writeLogEntry(const char* message) {
    if (!m_sdMutex || !m_logFile.isOpen()) {
        return;
    }
    
    bool locked = (xSemaphoreTake(m_sdMutex, pdMS_TO_TICKS(100)) == pdTRUE);
    if (!locked) {
        return;
    }
    
    m_logFile.println(message);
    m_logFile.sync();  // Ensure data is written
    
    xSemaphoreGive(m_sdMutex);
}

String SystemLog::makeTimestamp() {
    time_t now = time(nullptr);
    if (now < 1700000000) {
        // Time not synced, use millis
        return String(millis() / 1000) + "s";
    }
    
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    char buffer[32];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buffer);
}
