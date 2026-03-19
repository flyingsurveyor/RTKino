// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef SYSTEM_LOG_H
#define SYSTEM_LOG_H

#include <Arduino.h>
#include "SdFat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Maximum number of log files to keep
#define SYSTEM_LOG_MAX_FILES 3

// Log entry structure for async logging
struct SystemLogEntry {
    char message[256];
    uint32_t timestamp;
};

class SystemLog {
public:
    SystemLog();
    ~SystemLog();
    
    // Initialize the logging system
    bool begin(SdFat& sd, SemaphoreHandle_t sdMutex);
    
    // Stop logging and cleanup
    void end();
    
    // Log a message (async, non-blocking)
    void log(const char* format, ...);
    
    // Log an event (timestamp + message)
    void logEvent(const char* category, const String& message);
    
    // Get the current log filename
    String getCurrentLogFile() const { return m_currentLogFile; }
    
    // List all system log files
    bool listLogFiles(String files[], int& count, int maxFiles = SYSTEM_LOG_MAX_FILES);
    
    // Delete a specific log file
    bool deleteLogFile(const String& filename);
    
    // Update function - call periodically to flush pending logs
    void loop();
    
private:
    bool openNewLogFile();
    void closeCurrentLog();
    void rotateLogFiles();
    void writeLogEntry(const char* message);
    String makeTimestamp();
    
    SdFat* m_sd;
    SemaphoreHandle_t m_sdMutex;
    FsFile m_logFile;
    String m_currentLogFile;
    bool m_initialized;
    
    // Async logging queue
    QueueHandle_t m_logQueue;
};

#endif // SYSTEM_LOG_H
