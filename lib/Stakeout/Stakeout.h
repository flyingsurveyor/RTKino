// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <Arduino.h>
#include <vector>
#include "SdFat.h"
#include "freertos/semphr.h"

// ---------------------------------------------------------------------------
// Stakeout — navigate to design points
// Stores datasets in LittleFS /stakeout/, computes D2D/dH/Az vs current GNSS.
// ---------------------------------------------------------------------------

struct StakeoutPoint {
    String id;
    String name;
    double lat;   // WGS84 latitude (degrees)
    double lon;   // WGS84 longitude (degrees)
    double h;     // ellipsoidal height (m); NaN if not provided
};

struct StakeoutFile {
    String fileId;
    String name;
    int    count;
};

struct StakeoutActive {
    String fileId;
    String pointId;
    bool   valid() const { return !fileId.isEmpty() && !pointId.isEmpty(); }
};

struct StakeoutStatus {
    bool    valid;            // true if active target and rover position available
    double  d2d;              // 2D ground distance (m)
    double  dH;               // height delta: target_h - rover_h (m)
    double  az;               // azimuth from North (degrees, 0–360)
    String  targetName;
    String  targetId;
    double  targetLat;
    double  targetLon;
    double  targetH;
    uint8_t roverCarrSoln;    // 0=no RTK, 1=float, 2=fixed
    uint8_t roverFixQuality;
};

// Column-hint struct for CSV import with manual field mapping.
// If a field is non-empty it overrides auto-detection for that column.
struct StakeoutCSVHints {
    String latCol;    // header name for latitude
    String lonCol;    // header name for longitude
    String hCol;      // header name for height/HAE
    String nameCol;   // header name for point name
};

namespace Stakeout {

    // ---- Position provider callback (set from main.cpp) --------------------
    // Called to obtain the current rover position.
    typedef void (*PositionFn)(double& lat, double& lon, double& h,
                               uint8_t& carrSoln, uint8_t& fixQuality);
    extern PositionFn positionProvider;

    // ---- SD reference (for sync) -------------------------------------------
    extern SdFat*            _sd;
    extern SemaphoreHandle_t _sdMutex;
    extern bool*             _sdOK;

    // ---- Lifecycle ---------------------------------------------------------
    void begin(SdFat& sd, SemaphoreHandle_t sdMutex, bool& sdOK);

    // ---- File management ---------------------------------------------------
    bool listFiles(std::vector<StakeoutFile>& out);
    bool loadFilePoints(const String& fileId, std::vector<StakeoutPoint>& out);
    bool deleteFile(const String& fileId);

    // ---- Import ------------------------------------------------------------
    // Parse the supplied raw bytes and store as a normalised JSON file.
    // Returns the new fileId on success, empty string on error.
    String importCSV   (const String& name, const uint8_t* data, size_t len,
                        const StakeoutCSVHints& hints = StakeoutCSVHints());
    String importGeoJSON(const String& name, const uint8_t* data, size_t len);

    // Import directly from an existing SurveyPoints survey (by survey id).
    // Converts the survey's GeoJSON into a stakeout dataset.
    String importFromSurvey(const String& surveyId, const String& datasetName);

    // ---- Active selection --------------------------------------------------
    StakeoutActive getActive();
    bool           setActive(const String& fileId, const String& pointId);
    bool           getActivePoint(StakeoutPoint& out);  // fills out; false if none

    // ---- Computed navigation status ----------------------------------------
    StakeoutStatus getStatus();

    // ---- Dirty flag (cross-task cache invalidation) ------------------------
    // Set automatically by import/delete/setActive.
    // Consumers (OLED cache) check isDirty() and refresh as needed.
    bool isDirty();
    void clearDirty();

    // ---- SD sync -----------------------------------------------------------
    void syncToSD();

} // namespace Stakeout
