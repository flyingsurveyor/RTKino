// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <Arduino.h>
#include <Wire.h>

/**
 * UBX-CFG-VALSET helper per u-blox (ZED-F9x) via I2C.
 * - Layer mask: RAM=0x01 (default), BBR=0x02, FLASH=0x04
 * - Tutte le funzioni ritornano true se l'I2C endTransmission() == 0
 *
 * Consiglio: lasciare layer RAM (volatile) per uso "base" temporaneo.
 */
// ZED-F9P TMODE state structure

namespace UbxVal {

  // =================== LOW LEVEL ===================
  bool setU1(uint32_t keyId, uint8_t  value, uint8_t layerMask = 0x01, uint8_t i2cAddr = 0x42);
  bool setI1(uint32_t keyId, int8_t   value, uint8_t layerMask = 0x01, uint8_t i2cAddr = 0x42);
  bool setU2(uint32_t keyId, uint16_t value, uint8_t layerMask = 0x01, uint8_t i2cAddr = 0x42);
  bool setI2(uint32_t keyId, int16_t  value, uint8_t layerMask = 0x01, uint8_t i2cAddr = 0x42);
  bool setU4(uint32_t keyId, uint32_t value, uint8_t layerMask = 0x01, uint8_t i2cAddr = 0x42);
  bool setI4(uint32_t keyId, int32_t  value, uint8_t layerMask = 0x01, uint8_t i2cAddr = 0x42);
  bool setU8(uint32_t keyId, uint64_t value, uint8_t layerMask = 0x01, uint8_t i2cAddr = 0x42);
  bool setI8(uint32_t keyId, int64_t  value, uint8_t layerMask = 0x01, uint8_t i2cAddr = 0x42);
  bool setBytes(uint32_t keyId, const void* data, uint8_t len, uint8_t layerMask=0x01, uint8_t i2cAddr=0x42);

  inline bool setFlag(uint32_t keyId, bool on, uint8_t layerMask=0x01, uint8_t i2cAddr=0x42) {
    return setU1(keyId, on ? 1 : 0, layerMask, i2cAddr);
  }

  // =================== BATCH ===================
  struct Kv {
    enum Type : uint8_t { U1, I1, U2, I2, U4, I4, U8, I8, BYTES } type;
    uint32_t key;
    union {
      uint8_t  u1;  int8_t  i1;
      uint16_t u2;  int16_t i2;
      uint32_t u4;  int32_t i4;
      uint64_t u8;  int64_t i8;
    } v;
    const void* bytes;
    uint8_t blen;
    Kv(uint32_t k, uint8_t  val): type(U1), key(k){ v.u1=val; }
    Kv(uint32_t k, int8_t   val): type(I1), key(k){ v.i1=val; }
    Kv(uint32_t k, uint16_t val): type(U2), key(k){ v.u2=val; }
    Kv(uint32_t k, int16_t  val): type(I2), key(k){ v.i2=val; }
    Kv(uint32_t k, uint32_t val): type(U4), key(k){ v.u4=val; }
    Kv(uint32_t k, int32_t  val): type(I4), key(k){ v.i4=val; }
    Kv(uint32_t k, uint64_t val): type(U8), key(k){ v.u8=val; }
    Kv(uint32_t k, int64_t  val): type(I8), key(k){ v.i8=val; }
    Kv(uint32_t k, const void* p, uint8_t n): type(BYTES), key(k), bytes(p), blen(n) {}
  };

  bool multiSet(const Kv* items, size_t count, uint8_t layerMask=0x01, uint8_t i2cAddr=0x42);

  // =================== KEYS ===================
  namespace Keys {
    // UART2 OUT protocol enable (U1 flags 0/1)
    static constexpr uint32_t UART2OUT_UBX   = 0x10760001;
    static constexpr uint32_t UART2OUT_NMEA  = 0x10760002;
    static constexpr uint32_t UART2OUT_RTCM3 = 0x10760004;

    // RTCM3 MSGOUT on UART2 (U1: 0=off, 1=ogni epoch @1Hz; 10=ogni 10 epoch, ecc.)
    static constexpr uint32_t RTCM1005_UART2 = 0x209102BF;
    static constexpr uint32_t RTCM1230_UART2 = 0x20910305;
    static constexpr uint32_t RTCM1077_UART2 = 0x209102CE;
    static constexpr uint32_t RTCM1087_UART2 = 0x209102D3;
    static constexpr uint32_t RTCM1097_UART2 = 0x2091031A;
    static constexpr uint32_t RTCM1127_UART2 = 0x209102D8;
    
    // MSM4 messages
    static constexpr uint32_t RTCM1074_UART2 = 0x20910360;  // GPS MSM4
    static constexpr uint32_t RTCM1084_UART2 = 0x20910365;  // GLONASS MSM4
    static constexpr uint32_t RTCM1094_UART2 = 0x2091036A;  // Galileo MSM4

    // TMODE "legacy" + HP
    static constexpr uint32_t TMODE_MODE         = 0x20030001; // U1: 0=DIS,1=SVIN,2=FIXED
    static constexpr uint32_t TMODE_POS_TYPE     = 0x20030002; // U1: 1=LLH
    static constexpr uint32_t TMODE_LAT          = 0x40030009; // I4: deg * 1e-7
    static constexpr uint32_t TMODE_LON          = 0x4003000A; // I4: deg * 1e-7
    static constexpr uint32_t TMODE_HEIGHT       = 0x4003000B; // I4: centimetri
    static constexpr uint32_t TMODE_LAT_HP       = 0x2003000C; // I1: deg * 1e-9
    static constexpr uint32_t TMODE_LON_HP       = 0x2003000D; // I1: deg * 1e-9
    static constexpr uint32_t TMODE_HEIGHT_HP    = 0x2003000E; // I1: 0.1 mm
    static constexpr uint32_t TMODE_FIXEDPOSACC  = 0x4003000F; // U4: 0.1 mm

    // Station ID (DF003 OUT)
    static constexpr uint32_t RTCM_DF003_OUT     = 0x30090001; // U2: 1..4095
  }

  // =================== HELPERS ===================

  // Station ID (DF003) – comodo wrapper
  inline bool setStationID(uint16_t stid, uint8_t layerMask=0x01, uint8_t i2cAddr=0x42){
    return setU2(Keys::RTCM_DF003_OUT, stid, layerMask, i2cAddr);
  }

  // Solo RTCM3 su UART2 TX (volatile)
  inline bool enableRtcmOnUart2(uint8_t layerMask=0x01, uint8_t i2cAddr=0x42){
    bool ok=true;
    ok &= setFlag(Keys::UART2OUT_UBX,   false, layerMask, i2cAddr);
    ok &= setFlag(Keys::UART2OUT_NMEA,  false, layerMask, i2cAddr);
    ok &= setFlag(Keys::UART2OUT_RTCM3, true,  layerMask, i2cAddr);
    return ok;
  }

  // MSGOUT tipici per BASE: 1005/1230 @10s; MSM7 @1s (volatile)
  // rtcmType: 0=MSM7 4 costellazioni (GPS, GLO, GAL, BDS), 1=MSM4 3 costellazioni (GPS, GLO, GAL)
  inline bool setBaseMsgout(uint8_t rtcmType=0, uint8_t layerMask=0x01, uint8_t i2cAddr=0x42){
    bool ok=true;
    ok &= setU1(Keys::RTCM1005_UART2, 5, layerMask, i2cAddr);
    ok &= setU1(Keys::RTCM1230_UART2, 5, layerMask, i2cAddr);
    
    if (rtcmType == 0) {
      // MSM7 - 4 costellazioni @1s
      ok &= setU1(Keys::RTCM1077_UART2, 1,  layerMask, i2cAddr);
      ok &= setU1(Keys::RTCM1087_UART2, 1,  layerMask, i2cAddr);
      ok &= setU1(Keys::RTCM1097_UART2, 1,  layerMask, i2cAddr);
      ok &= setU1(Keys::RTCM1127_UART2, 1,  layerMask, i2cAddr);
    } else {
      // MSM4 - 3 costellazioni @1s (GPS, GLO, GAL - no BDS)
      ok &= setU1(Keys::RTCM1074_UART2, 1,  layerMask, i2cAddr);
      ok &= setU1(Keys::RTCM1084_UART2, 1,  layerMask, i2cAddr);
      ok &= setU1(Keys::RTCM1094_UART2, 1,  layerMask, i2cAddr);
      // Disabilito esplicitamente MSM7 se erano attive
      ok &= setU1(Keys::RTCM1077_UART2, 0,  layerMask, i2cAddr);
      ok &= setU1(Keys::RTCM1087_UART2, 0,  layerMask, i2cAddr);
      ok &= setU1(Keys::RTCM1097_UART2, 0,  layerMask, i2cAddr);
      ok &= setU1(Keys::RTCM1127_UART2, 0,  layerMask, i2cAddr);
    }
    return ok;
  }

  // TMODE = FIXED/LLH con campi HP (volatile)
  bool setTmodeFixedLLH_HP(double lat_deg, double lon_deg, double alt_m,
                           uint32_t fixedPosAcc_0p1mm = 200,
                           uint8_t layerMask=0x01, uint8_t i2cAddr=0x42);

  // =================== VALGET (READ CONFIG) ===================
  
  // Read single values from ZED-F9P configuration
  bool getU1(uint32_t keyId, uint8_t& value, uint8_t i2cAddr = 0x42);
  bool getI4(uint32_t keyId, int32_t& value, uint8_t i2cAddr = 0x42);
  bool getI1(uint32_t keyId, int8_t& value, uint8_t i2cAddr = 0x42);

  // =================== ZED-F9P RESET ===================
  
  // Reset ZED-F9P module (hot or cold start)
  bool resetZed(bool coldStart, uint8_t i2cAddr = 0x42);

} // namespace UbxVal

// =================== TMODE STATE STRUCTURE ===================
// Global state for ZED-F9P TMODE configuration
struct ZedTmodeState {
  uint8_t mode;        // 0=Disabled, 1=Survey-in, 2=Fixed
  uint8_t posType;     // 0=ECEF, 1=LLH
  double lat, lon;     // if posType=1 (LLH)
  double height;       // height sent to ZED (phase center)
  bool valid;          // true if successfully read
  uint32_t lastCheck;  // millis() of last read
};


namespace UbxVal {
  // High-level: read full TMODE state
  bool getTmodeState(ZedTmodeState& state, uint8_t i2cAddr = 0x42);
}
