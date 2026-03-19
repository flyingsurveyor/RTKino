// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "UbxValset.h"
#include <math.h>    // llround
#include <string.h>  // memcpy, memset
#include <stdlib.h>  // malloc, free

// =================== UTILS ===================
static inline void addCk(uint8_t b, uint8_t& A, uint8_t& B){ A += b; B += A; }

// invia un unico VALSET con payload già composto
static bool sendValsetPayload(const uint8_t* payload, uint16_t len, uint8_t i2cAddr){
  uint8_t hdr[6] = {0xB5,0x62,0x06,0x8A, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  uint8_t ckA=0, ckB=0;
  // header (class/id/lenL/lenH)
  addCk(hdr[2], ckA, ckB); addCk(hdr[3], ckA, ckB);
  addCk(hdr[4], ckA, ckB); addCk(hdr[5], ckA, ckB);
  // payload
  for(uint16_t i=0;i<len;i++) addCk(payload[i], ckA, ckB);

  Wire.beginTransmission(i2cAddr);
  Wire.write(hdr, sizeof(hdr));
  Wire.write(payload, len);
  Wire.write(ckA); Wire.write(ckB);
  return (Wire.endTransmission() == 0);
}

// header payload comune (version/layers/reserved)
static inline void fillValsetHeader(uint8_t* p, uint8_t layers){
  p[0] = 0x00;   // version
  p[1] = layers; // layers bitmask
  p[2] = 0x00;   // reserved1 L
  p[3] = 0x00;   // reserved1 H
}
static inline void putKeyLE(uint8_t* p, uint32_t key){
  p[0] = (uint8_t)(key & 0xFF);
  p[1] = (uint8_t)((key >> 8) & 0xFF);
  p[2] = (uint8_t)((key >> 16) & 0xFF);
  p[3] = (uint8_t)((key >> 24) & 0xFF);
}

namespace UbxVal {

// =================== SINGLE-KEY SETTERS ===================
bool setU1(uint32_t keyId, uint8_t value, uint8_t layerMask, uint8_t i2cAddr){
  uint8_t payload[9];
  fillValsetHeader(payload, layerMask);
  putKeyLE(&payload[4], keyId);
  payload[8] = value;
  return sendValsetPayload(payload, sizeof(payload), i2cAddr);
}

bool setI1(uint32_t keyId, int8_t value, uint8_t layerMask, uint8_t i2cAddr){
  return setU1(keyId, (uint8_t)value, layerMask, i2cAddr);
}

bool setU2(uint32_t keyId, uint16_t value, uint8_t layerMask, uint8_t i2cAddr){
  uint8_t payload[10];
  fillValsetHeader(payload, layerMask);
  putKeyLE(&payload[4], keyId);
  payload[8] = (uint8_t)(value & 0xFF);
  payload[9] = (uint8_t)(value >> 8);
  return sendValsetPayload(payload, sizeof(payload), i2cAddr);
}

bool setI2(uint32_t keyId, int16_t value, uint8_t layerMask, uint8_t i2cAddr){
  return setU2(keyId, (uint16_t)value, layerMask, i2cAddr);
}

bool setU4(uint32_t keyId, uint32_t value, uint8_t layerMask, uint8_t i2cAddr){
  uint8_t payload[12];
  fillValsetHeader(payload, layerMask);
  putKeyLE(&payload[4], keyId);
  payload[8]  = (uint8_t)( value        & 0xFF);
  payload[9]  = (uint8_t)((value >> 8 ) & 0xFF);
  payload[10] = (uint8_t)((value >> 16) & 0xFF);
  payload[11] = (uint8_t)((value >> 24) & 0xFF);
  return sendValsetPayload(payload, sizeof(payload), i2cAddr);
}

bool setI4(uint32_t keyId, int32_t value, uint8_t layerMask, uint8_t i2cAddr){
  return setU4(keyId, (uint32_t)value, layerMask, i2cAddr);
}

bool setU8(uint32_t keyId, uint64_t value, uint8_t layerMask, uint8_t i2cAddr){
  uint8_t payload[16];
  fillValsetHeader(payload, layerMask);
  putKeyLE(&payload[4], keyId);
  for (int i=0;i<8;i++) payload[8+i] = (uint8_t)(value >> (8*i));
  return sendValsetPayload(payload, sizeof(payload), i2cAddr);
}

bool setI8(uint32_t keyId, int64_t value, uint8_t layerMask, uint8_t i2cAddr){
  return setU8(keyId, (uint64_t)value, layerMask, i2cAddr);
}

bool setBytes(uint32_t keyId, const void* data, uint8_t len, uint8_t layerMask, uint8_t i2cAddr){
  if (!data || len==0) return false;
  const uint16_t total = 8 + len;       // header(4) + key(4) + data
  uint8_t* payload = (uint8_t*)malloc(total);
  if (!payload) return false;
  fillValsetHeader(payload, layerMask);
  putKeyLE(&payload[4], keyId);
  memcpy(&payload[8], data, len);
  bool ok = sendValsetPayload(payload, total, i2cAddr);
  free(payload);
  return ok;
}

// =================== MULTI-SET (batch in un solo VALSET) ===================
bool multiSet(const Kv* items, size_t count, uint8_t layerMask, uint8_t i2cAddr){
  if (!items || count==0) return false;

  uint16_t len = 4; // version+layers+reserved
  for (size_t i=0;i<count;i++){
    len += 4; // key
    switch(items[i].type){
      case Kv::U1: case Kv::I1: len += 1; break;
      case Kv::U2: case Kv::I2: len += 2; break;
      case Kv::U4: case Kv::I4: len += 4; break;
      case Kv::U8: case Kv::I8: len += 8; break;
      case Kv::BYTES:           len += items[i].blen; break;
    }
  }

  uint8_t* payload = (uint8_t*)malloc(len);
  if (!payload) return false;

  fillValsetHeader(payload, layerMask);
  uint16_t ofs = 4;

  for (size_t i=0;i<count;i++){
    putKeyLE(&payload[ofs], items[i].key); ofs += 4;
    switch(items[i].type){
      case Kv::U1: payload[ofs++] = items[i].v.u1; break;
      case Kv::I1: payload[ofs++] = (uint8_t)items[i].v.i1; break;

      case Kv::U2:
      case Kv::I2: {
        uint16_t v = (items[i].type==Kv::U2) ? items[i].v.u2 : (uint16_t)items[i].v.i2;
        payload[ofs++] = (uint8_t)(v & 0xFF);
        payload[ofs++] = (uint8_t)(v >> 8);
      } break;

      case Kv::U4:
      case Kv::I4: {
        uint32_t v = (items[i].type==Kv::U4) ? items[i].v.u4 : (uint32_t)items[i].v.i4;
        payload[ofs++] = (uint8_t)( v        & 0xFF);
        payload[ofs++] = (uint8_t)((v >> 8 ) & 0xFF);
        payload[ofs++] = (uint8_t)((v >> 16) & 0xFF);
        payload[ofs++] = (uint8_t)((v >> 24) & 0xFF);
      } break;

      case Kv::U8:
      case Kv::I8: {
        uint64_t v = (items[i].type==Kv::U8) ? items[i].v.u8 : (uint64_t)items[i].v.i8;
        for(int b=0;b<8;b++) payload[ofs++] = (uint8_t)(v >> (8*b));
      } break;

      case Kv::BYTES:
        memcpy(&payload[ofs], items[i].bytes, items[i].blen);
        ofs += items[i].blen;
        break;
    }
  }

  bool ok = sendValsetPayload(payload, len, i2cAddr);
  free(payload);
  return ok;
}

// =================== TMODE FIXED LLH (HP) ===================
// Implementazione che corrisponde alla dichiarazione in UbxValset.h
bool setTmodeFixedLLH_HP(double lat_deg, double lon_deg, double alt_m,
                         uint32_t fixedPosAcc_0p1mm,
                         uint8_t layerMask, uint8_t i2cAddr)
{
  // ---- LAT/LON: I4 (1e-7 deg) + I1 HP (1e-9 deg) ----
  int64_t lat_e9 = llround(lat_deg * 1e9);
  int64_t lon_e9 = llround(lon_deg * 1e9);

  int32_t lat_L  = (int32_t)(lat_e9 / 100);                 // 1e-7 deg
  int8_t  lat_HP = (int8_t) (lat_e9 - (int64_t)lat_L * 100);// 1e-9 deg

  int32_t lon_L  = (int32_t)(lon_e9 / 100);
  int8_t  lon_HP = (int8_t) (lon_e9 - (int64_t)lon_L * 100);

  // ---- HEIGHT: I4 cm + I1 HP 0.1 mm ----
  int64_t h_0p1mm = llround(alt_m * 10000.0);               // m -> 0.1 mm
  int32_t h_cm    = (int32_t)(h_0p1mm / 100);               // cm
  int8_t  h_HP    = (int8_t) (h_0p1mm - (int64_t)h_cm * 100); // 0.1 mm

  bool ok = true;
  ok &= setU1(Keys::TMODE_MODE,     2, layerMask, i2cAddr); // FIXED
  ok &= setU1(Keys::TMODE_POS_TYPE, 1, layerMask, i2cAddr); // LLH

  ok &= setI4(Keys::TMODE_LAT,       lat_L,  layerMask, i2cAddr);
  ok &= setI1(Keys::TMODE_LAT_HP,    lat_HP, layerMask, i2cAddr);
  ok &= setI4(Keys::TMODE_LON,       lon_L,  layerMask, i2cAddr);
  ok &= setI1(Keys::TMODE_LON_HP,    lon_HP, layerMask, i2cAddr);
  ok &= setI4(Keys::TMODE_HEIGHT,    h_cm,   layerMask, i2cAddr);
  ok &= setI1(Keys::TMODE_HEIGHT_HP, h_HP,   layerMask, i2cAddr);

  ok &= setU4(Keys::TMODE_FIXEDPOSACC, fixedPosAcc_0p1mm, layerMask, i2cAddr);
  return ok;
}

// =================== VALGET (READ CONFIG) ===================

// Helper to send VALGET request and receive response
static uint16_t sendValgetRequest(uint32_t keyId, uint8_t* response, uint16_t maxLen, uint8_t i2cAddr) {
  // UBX-CFG-VALGET message structure:
  // Header: 0xB5 0x62 0x06 0x8B
  // Length: 8 bytes (version + layer + reserved + keyId)
  // Payload: version(1) + layer(1) + position(2) + keyId(4)
  // Checksum: 2 bytes
  
  uint8_t payload[8];
  payload[0] = 0x00;  // version
  payload[1] = 0x00;  // layer: RAM
  payload[2] = 0x00;  // position LSB
  payload[3] = 0x00;  // position MSB
  
  // Key ID (little endian)
  payload[4] = (uint8_t)(keyId & 0xFF);
  payload[5] = (uint8_t)((keyId >> 8) & 0xFF);
  payload[6] = (uint8_t)((keyId >> 16) & 0xFF);
  payload[7] = (uint8_t)((keyId >> 24) & 0xFF);
  
  // Calculate checksum
  uint8_t ckA = 0, ckB = 0;
  addCk(0x06, ckA, ckB); // class
  addCk(0x8B, ckA, ckB); // id
  addCk(0x08, ckA, ckB); // length LSB
  addCk(0x00, ckA, ckB); // length MSB
  for (int i = 0; i < 8; i++) addCk(payload[i], ckA, ckB);
  
  // Send request
  Wire.beginTransmission(i2cAddr);
  Wire.write(0xB5); // sync char 1
  Wire.write(0x62); // sync char 2
  Wire.write(0x06); // class
  Wire.write(0x8B); // id
  Wire.write(0x08); // length LSB
  Wire.write(0x00); // length MSB
  Wire.write(payload, 8);
  Wire.write(ckA);
  Wire.write(ckB);
  
  if (Wire.endTransmission() != 0) return 0;
  
  // Wait for response (give ZED time to process)
  delay(50);
  
  // Read response
  uint16_t avail = Wire.requestFrom(i2cAddr, maxLen);
  if (avail < 12) return 0; // minimum response size
  
  for (uint16_t i = 0; i < avail && i < maxLen && Wire.available(); i++) {
    response[i] = Wire.read();
  }
  
  return avail; // return number of bytes read
}

// Read U1 value
bool getU1(uint32_t keyId, uint8_t& value, uint8_t i2cAddr) {
  uint8_t response[32];
  uint16_t bytesRead = sendValgetRequest(keyId, response, sizeof(response), i2cAddr);
  
  // Parse UBX-CFG-VALGET response
  // Expected: 0xB5 0x62 0x06 0x8B [len] [version] [layer] [reserved] [keyId] [value]
  if (bytesRead < 15) return false; // need at least 15 bytes for U1 value
  if (response[0] != 0xB5 || response[1] != 0x62) return false;
  if (response[2] != 0x06 || response[3] != 0x8B) return false;
  
  // Value starts at offset: sync(2) + class(1) + id(1) + len(2) + version(1) + layer(1) + pos(2) + key(4) = 14
  value = response[14];
  return true;
}

// Read I1 value
bool getI1(uint32_t keyId, int8_t& value, uint8_t i2cAddr) {
  uint8_t u;
  if (!getU1(keyId, u, i2cAddr)) return false;
  value = (int8_t)u;
  return true;
}

// Read I4 value
bool getI4(uint32_t keyId, int32_t& value, uint8_t i2cAddr) {
  uint8_t response[32];
  uint16_t bytesRead = sendValgetRequest(keyId, response, sizeof(response), i2cAddr);
  
  // Parse UBX-CFG-VALGET response
  if (bytesRead < 18) return false; // need at least 18 bytes for I4 value
  if (response[0] != 0xB5 || response[1] != 0x62) return false;
  if (response[2] != 0x06 || response[3] != 0x8B) return false;
  
  // Value is 4 bytes at offset 14 (little endian)
  value = (int32_t)(response[14] | (response[15] << 8) | (response[16] << 16) | (response[17] << 24));
  return true;
}

// Read full TMODE state
bool getTmodeState(ZedTmodeState& state, uint8_t i2cAddr) {
  state.valid = false;
  state.mode = 0;
  state.posType = 0;
  state.lat = 0.0;
  state.lon = 0.0;
  state.height = 0.0;
  
  // Read TMODE mode
  if (!getU1(Keys::TMODE_MODE, state.mode, i2cAddr)) return false;
  
  // If mode is disabled, that's all we need
  if (state.mode == 0) {
    state.valid = true;
    state.lastCheck = millis();
    return true;
  }
  
  // Read position type
  if (!getU1(Keys::TMODE_POS_TYPE, state.posType, i2cAddr)) return false;
  
  // For Fixed mode in LLH coordinates
  if (state.mode == 2 && state.posType == 1) {
    int32_t lat_L, lon_L, h_cm;
    int8_t lat_HP, lon_HP, h_HP;
    
    // Read latitude (I4 + I1 HP)
    if (!getI4(Keys::TMODE_LAT, lat_L, i2cAddr)) return false;
    if (!getI1(Keys::TMODE_LAT_HP, lat_HP, i2cAddr)) return false;
    
    // Read longitude (I4 + I1 HP)
    if (!getI4(Keys::TMODE_LON, lon_L, i2cAddr)) return false;
    if (!getI1(Keys::TMODE_LON_HP, lon_HP, i2cAddr)) return false;
    
    // Read height (I4 + I1 HP)
    if (!getI4(Keys::TMODE_HEIGHT, h_cm, i2cAddr)) return false;
    if (!getI1(Keys::TMODE_HEIGHT_HP, h_HP, i2cAddr)) return false;
    
    // Convert to double with HP precision
    int64_t lat_e9 = (int64_t)lat_L * 100 + lat_HP;
    int64_t lon_e9 = (int64_t)lon_L * 100 + lon_HP;
    int64_t h_0p1mm = (int64_t)h_cm * 100 + h_HP;
    
    state.lat = lat_e9 / 1e9;
    state.lon = lon_e9 / 1e9;
    state.height = h_0p1mm / 10000.0;  // 0.1mm to meters
  }
  
  state.valid = true;
  state.lastCheck = millis();
  return true;
}

// =================== ZED-F9P RESET ===================

bool resetZed(bool coldStart, uint8_t i2cAddr) {
    uint8_t msg[12];
    msg[0] = 0xB5; msg[1] = 0x62;  // UBX header
    msg[2] = 0x06; msg[3] = 0x04;  // CFG-RST
    msg[4] = 0x04; msg[5] = 0x00;  // Length = 4
    
    // Payload
    if (coldStart) {
        msg[6] = 0xFF; msg[7] = 0xFF;  // navBbrMask = 0xFFFF (cold)
    } else {
        msg[6] = 0x00; msg[7] = 0x00;  // navBbrMask = 0x0000 (hot)
    }
    msg[8] = 0x01;  // resetMode = software reset
    msg[9] = 0x00;  // reserved
    
    // Checksum
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 10; i++) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    msg[10] = ck_a;
    msg[11] = ck_b;
    
    Wire.beginTransmission(i2cAddr);
    Wire.write(msg, 12);
    return Wire.endTransmission() == 0;
}

} // namespace UbxVal
