// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef POINTCODES_H
#define POINTCODES_H

#include <Arduino.h>
#include <vector>

struct PointCode     { String cod; String label; };
struct PointCategory { String id;  String label; std::vector<PointCode> codici; };

namespace PointCodes {

  // Call once at boot (after FlashConfig::begin)
  void begin();

  // Persistence
  bool loadFromFlash();     // read /config/codici_punto.json
  bool saveToFlash();       // write + FlashConfig::markDirty()
  void resetToDefaults();   // overwrite with hardcoded list + save

  // Read accessors
  int    getCategoryCount();
  String getCategoryId   (int catIdx);
  String getCategoryLabel(int catIdx);
  int    getCodeCount    (int catIdx);
  String getCodeCod      (int catIdx, int codeIdx);
  String getCodeLabel    (int catIdx, int codeIdx);

  // Flat list (for OLED): iterates all categories sequentially
  // Separator entries (category headers) have cod == ""
  int    getFlatCount();
  String getFlatLabel(int flatIdx);    // display string
  String getFlatCod  (int flatIdx);    // "" for category-header rows
  int    getFlatCatIdx (int flatIdx);
  int    getFlatCodeIdx(int flatIdx);

  // Return full JSON string (for API)
  String toJSON();
  // Parse and replace from JSON string
  bool   fromJSON(const String& json);

} // namespace PointCodes

#endif // POINTCODES_H
