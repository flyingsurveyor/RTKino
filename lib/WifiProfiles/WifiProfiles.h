// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef WIFIPROFILES_H
#define WIFIPROFILES_H

#include <Arduino.h>
#include <SdFat.h>
#include <vector>

struct WifiCred {
  int priority;       // numeri più piccoli = maggiore priorità
  String ssid;
  String password;
};

namespace WifiProfiles {
  bool load(SdFat& sd, std::vector<WifiCred>& out);
  bool save(SdFat& sd, const std::vector<WifiCred>& list);
  void sortByPriority(std::vector<WifiCred>& list);
  bool removeAt(SdFat& sd, size_t idx);

  // Flash-based overloads (no SD access, no mutex needed)
  bool loadFromFlash(std::vector<WifiCred>& out);
  bool saveToFlash(const std::vector<WifiCred>& list);
  bool removeAtFlash(size_t idx);
}

#endif
