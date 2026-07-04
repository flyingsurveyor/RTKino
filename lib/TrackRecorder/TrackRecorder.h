// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <Arduino.h>
#include <vector>
#include "SdFat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ---------------------------------------------------------------------------
// TrackRecorder — continuous GNSS path logger (trail/road/pipeline tracking)
// Not survey-grade: records where the rover has been, at a time or distance
// trigger, exported as GPX/KML. Primary storage is LittleFS (like the rest
// of RTKino); SD is only ever the periodic backup copy, never the primary
// write target.
// ---------------------------------------------------------------------------

enum TrackTriggerMode { TRACK_TRIGGER_TIME = 0, TRACK_TRIGGER_DISTANCE = 1 };

struct TrackConfig {
    TrackTriggerMode triggerMode = TRACK_TRIGGER_TIME;
    float intervalSec = 5.0f;    // used when triggerMode == TIME
    float distanceM   = 10.0f;   // used when triggerMode == DISTANCE
};

struct TrackStatus {
    bool        recording     = false;
    String      trackId;
    uint32_t    pointCount    = 0;
    uint32_t    pointsDropped = 0;   // buffer overflow — oldest unflushed points overwritten
    double      distanceM     = 0.0;
    uint32_t    durationSec   = 0;
    TrackConfig cfg;
    String      lastError;
};

struct TrackInfo {
    String   trackId;
    String   name;
    uint32_t startEpoch;
    uint32_t endEpoch;    // 0 while still recording
    uint32_t pointCount;
    double   distanceM;
};

namespace TrackRecorder {

    // ---- Position provider (same shape as Stakeout::PositionFn — the same
    //      function pointer, e.g. takeStakeoutPosition, is assigned to both) --
    typedef void (*PositionFn)(double& lat, double& lon, double& h,
                               uint8_t& carrSoln, uint8_t& fixQuality);
    extern PositionFn positionProvider;

    // ---- SD reference (backup sync only — never the primary write target) --
    extern SdFat*            _sd;
    extern SemaphoreHandle_t _sdMutex;
    extern bool*             _sdOK;

    // ---- Lifecycle ----------------------------------------------------------
    void begin(SdFat& sd, SemaphoreHandle_t sdMutex, bool& sdOK);

    // FreeRTOS task entry point — created explicitly from main.cpp (mirrors sdWriterTask)
    void backgroundTask(void* pvParameters);

    // ---- Control --------------------------------------------------------------
    bool start(const String& name = "");   // false + status.lastError set on failure
    bool stop();
    bool isRecording();

    // ---- Config ---------------------------------------------------------------
    void        setTriggerMode(TrackTriggerMode mode);
    void        setIntervalSec(float sec);
    void        setDistanceM(float metres);
    TrackConfig getConfig();

    // ---- Status -----------------------------------------------------------
    TrackStatus getStatus();

    // ---- Track listing / management ----------------------------------------
    bool listTracks(std::vector<TrackInfo>& out);
    bool deleteTrack(const String& trackId);

    // ---- SD backup sync (mirrors SurveyPoints::periodicSync/syncToSD) ------
    void periodicSync(bool forceNow = false);

    // ---- Export (streaming — never loads the whole track into RAM) --------
    typedef void (*EmitFn)(const String& chunk, void* ctx);
    bool exportGPX(const String& trackId, EmitFn emit, void* ctx);
    bool exportKML(const String& trackId, EmitFn emit, void* ctx);

} // namespace TrackRecorder
