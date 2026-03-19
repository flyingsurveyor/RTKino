// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "WifiProfiles.h"
#include "FlashConfig.h"

static const char* WIFI_FILE = "/gnss/wifi.txt";
static const char* WIFI_FLASH_FILE = "/config/wifi.txt";

// ---- Parse helpers shared by SD and flash variants ----
static void parseWifiLine(const String& line, std::vector<WifiCred>& out) {
  int p1 = line.indexOf(';');
  int p2 = (p1 >= 0) ? line.indexOf(';', p1 + 1) : -1;
  if (p1 < 0 || p2 < 0) return;
  WifiCred c;
  c.priority = line.substring(0, p1).toInt();
  c.ssid     = line.substring(p1 + 1, p2);
  c.password = line.substring(p2 + 1);
  c.ssid.trim(); c.password.trim();
  out.push_back(c);
}

void WifiProfiles::sortByPriority(std::vector<WifiCred>& list) {
  std::sort(list.begin(), list.end(), [](const WifiCred& a, const WifiCred& b){
    if (a.priority != b.priority) return a.priority < b.priority;
    return a.ssid < b.ssid;
  });
}

bool WifiProfiles::load(SdFat& sd, std::vector<WifiCred>& out) {
  out.clear();
  FsFile f = sd.open(WIFI_FILE, FILE_READ);
  if (!f) return false;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;
    // formato: priority;ssid;password
    int p1 = line.indexOf(';');
    int p2 = (p1>=0) ? line.indexOf(';', p1+1) : -1;
    if (p1<0 || p2<0) continue;

    WifiCred c;
    c.priority = line.substring(0, p1).toInt();
    c.ssid     = line.substring(p1+1, p2);
    c.password = line.substring(p2+1);
    c.ssid.trim(); c.password.trim();
    out.push_back(c);
  }
  f.close();
  return !out.empty();
}

bool WifiProfiles::save(SdFat& sd, const std::vector<WifiCred>& list) {
  FsFile f = sd.open(WIFI_FILE, O_WRITE | O_CREAT | O_TRUNC);
  if (!f) return false;
  for (auto& c : list) {
    f.print(String(c.priority) + ";" + c.ssid + ";" + c.password + "\n");
  }
  f.close();
  return true;
}

bool WifiProfiles::removeAt(SdFat& sd, size_t idx) {
  // Semplice implementazione: carica, erase idx, salva
  std::vector<WifiCred> list;
  if (!load(sd, list)) return false;
  if (idx >= list.size()) return false;
  list.erase(list.begin()+idx);
  return save(sd, list);
}

// ---- Flash-based overloads ----

bool WifiProfiles::loadFromFlash(std::vector<WifiCred>& out) {
  out.clear();
  String content = FlashConfig::readFile(WIFI_FLASH_FILE);
  if (content.length() == 0) return false;
  int start = 0;
  while (start < (int)content.length()) {
    int end = content.indexOf('\n', start);
    if (end < 0) end = content.length();
    String line = content.substring(start, end);
    line.trim();
    start = end + 1;
    if (line.length() == 0 || line.startsWith("#")) continue;
    parseWifiLine(line, out);
  }
  return !out.empty();
}

bool WifiProfiles::saveToFlash(const std::vector<WifiCred>& list) {
  String content;
  for (auto& c : list) {
    content += String(c.priority) + ";" + c.ssid + ";" + c.password + "\n";
  }
  return FlashConfig::writeFile(WIFI_FLASH_FILE, content);
}

bool WifiProfiles::removeAtFlash(size_t idx) {
  std::vector<WifiCred> list;
  if (!loadFromFlash(list)) return false;
  if (idx >= list.size()) return false;
  list.erase(list.begin() + idx);
  return saveToFlash(list);
}
