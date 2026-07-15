// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <Arduino.h>
#include <vector>
#include "SdFat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// A single GNSS snapshot (all fields needed for survey point)
struct GNSSSnapshot {
    uint8_t fixMode;     // fix type (3=3D fix)
    uint8_t carrSoln;    // 0=no RTK, 1=float, 2=fixed
    uint8_t numSV;
    double  lat, lon;    // degrees (from GGA)
    float   altMSL;      // MSL altitude (m, from GGA)
    float   hAcc, vAcc;  // accuracy (m)

    // NAV-HPPOSLLH
    double latHP, lonHP;    // high-precision (degrees)
    double altHAE;          // ellipsoidal height (m)
    double altMSLHP;        // MSL height HP (m)
    float  hAccHP, vAccHP;  // HP accuracy (m)
    bool   hpValid;

    // NAV-HPPOSECEF
    double ecefX, ecefY, ecefZ;  // metres
    float  pAcc;
    bool   ecefValid;

    // DOP (all 7)
    float gdop, pdop, hdop, vdop, ndop, edop, tdop;

    // NAV-COV
    float covNN, covEE, covDD, covNE, covND, covED;
    bool  covValid;

    // NAV-RELPOSNED
    double relN, relE, relD;    // metres
    float  relSN, relSE, relSD; // sigma (m)
    bool   relValid;
};

enum MeasureStatus { MS_IDLE, MS_RUNNING, MS_DONE, MS_ERROR };

struct QualityWarning {
    bool   hAccBad = false;
    bool   pdopBad = false;
    bool   svBad   = false;
    bool   stale   = false;  // no 3D fix (fixMode < 3)
    bool   noRtk   = false;  // no RTK fix (carrSoln == 0)
    String details;          // human-readable description of all failing checks
    bool anyBad() const { return hAccBad || pdopBad || svBad || stale || noRtk; }
};

struct MeasureParams {
    String name;
    String codice;
    String desc;
    float  durationSec  = 10.0f;
    float  intervalSec  = 0.5f;
    bool   forceQuality = false; // skip quality gate
};

struct MeasureProgress {
    MeasureStatus status   = MS_IDLE;
    int           pct      = 0;     // 0-100
    int           nSamples = 0;
    float         curHAcc  = 0.0f;
    float         elapsed  = 0.0f;
    String        lastPointId;
    String        errorMsg;
};

namespace SurveyPoints {

    // Quality thresholds
    extern float maxHAcc;  // default 0.05 m
    extern float maxPDOP;  // default 3.0
    extern int   minSV;    // default 8

    // Snapshot provider callback (set from main.cpp)
    typedef void (*SnapshotFn)(GNSSSnapshot&);
    extern SnapshotFn snapshotProvider;

    // SD reference (for sync)
    extern SdFat*            _sd;
    extern SemaphoreHandle_t _sdMutex;
    extern bool*             _sdOK;

    // Lifecycle
    void begin(SdFat& sd, SemaphoreHandle_t sdMutex, bool& sdOK);

    // ---- Survey CRUD ----
    String createSurvey(const String& title, const String& desc = "");
    bool   deleteSurvey(const String& sid);
    bool   listSurveyIds(std::vector<String>& ids);
    String loadSurveyJSON(const String& sid);
    bool   saveSurveyJSON(const String& sid, const String& json);
    String getActiveSurveyId();
    bool   setActiveSurveyId(const String& sid);
    int    getSurveyPointCount(const String& sid);

    // ---- Lightweight point listing (id/name/lat/lon only) ----
    // Used by the OLED map view; avoids re-parsing the full GeoJSON feature
    // (accuracy/DOP/etc.) when only position is needed.
    struct SurveyPointBrief {
        String id;
        String name;
        double lat;
        double lon;
    };
    bool listPoints(const String& sid, std::vector<SurveyPointBrief>& out);

    // ---- Quality gate ----
    QualityWarning checkQuality();

    // ---- Async measurement ----
    bool           startMeasure(const MeasureParams& params);
    MeasureProgress getMeasureProgress();
    bool           isMeasuring();

    // ---- Delete a point ----
    bool deletePoint(const String& sid, const String& pid);

    // ---- Edit a point's name/codice (coordinates untouched) ----
    bool editPoint(const String& sid, const String& pid, const String& name, const String& codice);

    // ---- Look up a single point's coordinates (lat/lon deg, altHAE m) + name, by id ----
    bool getPointCoords(const String& sid, const String& pid,
                         double& lat, double& lon, double& altHAE, String& name);

    // ---- Persist a COGO-derived point (forward intersection, trilateration, ...) ----
    // Tagged "source":"cogo" (vs. GNSS-measured points) so the WebUI can render a
    // distinct badge; srcA/srcB record the point ids it was computed from.
    bool saveCogoPoint(const String& sid, double lat, double lon, double altHAE,
                        const String& name, const String& codice,
                        const String& method, const String& srcA, const String& srcB,
                        String& outPid);

    // ---- Download ----
    String getSurveyGeoJSON(const String& sid);
    String getSurveyCSV(const String& sid);

    // ---- SD sync ----
    void syncToSD();
    void periodicSync(bool forceNow = false);

} // namespace SurveyPoints
