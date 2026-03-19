// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
//
// gnss_types.h — Shared data structures for GNSS position, RTCM stats,
// and ZED-F9P state. Used by main.cpp, WebUI.cpp, and other modules.

#pragma once
#include <Arduino.h>

// ===== GNSS Position Status =====
struct GNSSPosition {
    double lat;           // decimal degrees (positive = N, negative = S)
    double lon;           // decimal degrees (positive = E, negative = W)
    float  alt;           // metres above sea level
    uint8_t fixQuality;   // 0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
    uint8_t numSats;      // satellites used (from GGA, for compatibility)
    uint8_t numSV;        // real multi-GNSS satellite count (from NAV-PVT)
    float  hdop;          // Horizontal DOP
    float  pdop;          // Position DOP (from NAV-PVT)
    float  vdop;          // Vertical DOP (from NAV-PVT)
    float  age;           // differential corrections age (seconds)
    uint8_t carrSoln;     // from NAV-PVT: 0=no RTK, 1=float, 2=fixed
    uint32_t lastUpdate;  // millis() of last valid update
    uint32_t lastPvtUpdate; // timestamp of last valid NAV-PVT
    float  hAcc;          // horizontal accuracy estimate (metres)
    float  vAcc;          // vertical accuracy estimate (metres)
    bool   hpAccValid;    // true once NAV-HPPOSLLH has provided hAcc/vAcc

    // NAV-HPPOSLLH — high-precision geodetic coordinates
    double latHP, lonHP;  // high-precision lat/lon (degrees)
    double altHAE;        // ellipsoidal height (m)
    double altMSLHP;      // MSL height HP (m)

    // NAV-HPPOSECEF — high-precision ECEF
    double ecefX, ecefY, ecefZ;  // metres
    float  pAcc;                 // position accuracy (m)
    bool   ecefValid;

    // NAV-DOP — extended DOP values
    float  gdop;          // Geometric DOP
    float  ndop;          // Northing DOP
    float  edop;          // Easting DOP
    float  tdop;          // Time DOP

    // NAV-COV — position covariance matrix (NED, m^2)
    float  covNN, covEE, covDD;
    float  covNE, covND, covED;
    bool   covValid;

    // NAV-RELPOSNED — relative position to base
    double relN, relE, relD;    // metres
    float  relSN, relSE, relSD; // accuracy sigma (m)
    bool   relValid;
};

// ===== RTCM Statistics (from UBX-RXM-RTCM) =====
struct RtcmMsgStats {
    uint16_t msgType;      // Message type (1005, 1077, etc.)
    uint32_t count;        // Messages received
    uint32_t crcErrors;    // CRC error count
    uint32_t lastSeen;     // millis() of last message
};

#define RTCM_MAX_TYPES 16  // Max distinct types to track

struct RtcmStats {
    RtcmMsgStats msgs[RTCM_MAX_TYPES];
    uint8_t numTypes;           // How many distinct types seen
    uint32_t totalBytes;        // Total bytes received (estimate)
    uint32_t totalMessages;     // Total messages
    uint32_t lastUpdate;        // Last update
};
