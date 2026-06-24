// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include <LittleFS.h>
#include "FlashConfig.h"

// Migration file list: SD path -> LittleFS path
static const struct { const char* sdPath; const char* flashPath; } MIGRATE_FILES[] = {
  { "/gnss/wifi.txt",                 "/config/wifi.txt"                },
  { "/gnss/ntrip_in_list.txt",        "/config/ntrip_in_list.txt"       },
  { "/gnss/ntrip_out_list.txt",       "/config/ntrip_out_list.txt"      },
  { "/gnss/tcp_in_lista.txt",         "/config/tcp_in_lista.txt"        },
  { "/gnss/tcp_out_client_list.txt",  "/config/tcp_out_client_list.txt" },
  { "/gnss/bases.txt",                "/config/bases.txt"               },
  { "/gnss/antennas.txt",             "/config/antennas.txt"            },
  { "/gnss/ntp.txt",                  "/config/ntp.txt"                 },
  { "/gnss/mdns.txt",                 "/config/mdns.txt"                },
  { "/gnss/ble_name.txt",             "/config/ble_name.txt"            },
  { "/gnss/ble_pin.txt",              "/config/ble_pin.txt"             },
  { "/gnss/ap_ssid.txt",             "/config/ap_ssid.txt"             },
  { "/gnss/ap_pass.txt",             "/config/ap_pass.txt"             },
  { "/gnss/config.txt",               "/config/config.txt"              },
  { "/gnss/ntrip.txt",                "/config/ntrip.txt"               },
  { "/gnss/codici_punto.json",        "/config/codici_punto.json"       },
  { "/gnss/tz.txt",                   "/config/tz.txt"                  },
  { "/gnss/device_name.txt",          "/config/device_name.txt"         },
};
static const int MIGRATE_COUNT = sizeof(MIGRATE_FILES) / sizeof(MIGRATE_FILES[0]);

// Sync destination list: LittleFS path -> SD path
static const struct { const char* flashPath; const char* sdPath; } SYNC_FILES[] = {
  { "/config/wifi.txt",                 "/gnss/wifi.txt"                },
  { "/config/ntrip_in_list.txt",        "/gnss/ntrip_in_list.txt"       },
  { "/config/ntrip_out_list.txt",       "/gnss/ntrip_out_list.txt"      },
  { "/config/tcp_in_lista.txt",         "/gnss/tcp_in_lista.txt"        },
  { "/config/tcp_out_client_list.txt",  "/gnss/tcp_out_client_list.txt" },
  { "/config/bases.txt",                "/gnss/bases.txt"               },
  { "/config/antennas.txt",             "/gnss/antennas.txt"            },
  { "/config/ntp.txt",                  "/gnss/ntp.txt"                 },
  { "/config/mdns.txt",                 "/gnss/mdns.txt"                },
  { "/config/ble_name.txt",             "/gnss/ble_name.txt"            },
  { "/config/ble_pin.txt",              "/gnss/ble_pin.txt"             },
  { "/config/ap_ssid.txt",             "/gnss/ap_ssid.txt"             },
  { "/config/ap_pass.txt",             "/gnss/ap_pass.txt"             },
  { "/config/codici_punto.json",        "/gnss/codici_punto.json"       },
  { "/config/tz.txt",                   "/gnss/tz.txt"                  },
  { "/config/device_name.txt",          "/gnss/device_name.txt"         },
};
static const int SYNC_COUNT = sizeof(SYNC_FILES) / sizeof(SYNC_FILES[0]);

static bool s_dirty = false;

// Sync buffer size for file copy operations
static const size_t SYNC_BUFFER_SIZE = 256;

// ---- Internal helpers ----

// Ensure a LittleFS directory exists
static void ensureDirLFS(const char* path) {
  if (!path) return;
  // Find last '/' before filename
  const char* slash = strrchr(path, '/');
  if (!slash || slash == path) return;
  char dir[64];
  size_t len = slash - path;
  if (len >= sizeof(dir)) return;
  memcpy(dir, path, len);
  dir[len] = '\0';
  if (!LittleFS.exists(dir)) {
    LittleFS.mkdir(dir);
  }
}

// Write content to LittleFS file
static bool writeLFS(const char* path, const String& content) {
  ensureDirLFS(path);
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  f.print(content);
  f.close();
  return true;
}

// Read entire LittleFS file into String
static String readLFS(const char* path) {
  if (!LittleFS.exists(path)) return "";
  File f = LittleFS.open(path, "r");
  if (!f) return "";
  String content = f.readString();
  f.close();
  return content;
}

// Copy a file from SD to LittleFS (called during migration)
static bool migrateFileToFlash(SdFat& sd, const char* sdPath, const char* flashPath) {
  if (!sd.exists(sdPath)) return false;
  FsFile f = sd.open(sdPath, O_RDONLY);
  if (!f) return false;
  String content = "";
  while (f.available()) {
    content += (char)f.read();
  }
  f.close();
  return writeLFS(flashPath, content);
}

// ---- Public API ----

bool FlashConfig::begin(SdFat& sd, SemaphoreHandle_t sdMutex, bool sdOk) {
  if (!LittleFS.begin(true)) {
    Serial.println("[Flash] LittleFS mount failed!");
    return false;
  }
  Serial.println("[Flash] LittleFS mounted OK");

  // Check if migration has already been done
  if (LittleFS.exists("/config/.migrated")) {
    Serial.println("[Flash] Already migrated, skipping");
    return true;
  }

  // Ensure /config directory exists
  if (!LittleFS.exists("/config")) {
    LittleFS.mkdir("/config");
  }

  if (sdOk && sdMutex) {
    // Attempt migration from SD (generous 5s timeout — first boot only)
    bool locked = (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE);
    if (locked) {
      Serial.println("[Flash] Migrating config from SD...");
      int copied = 0;
      for (int i = 0; i < MIGRATE_COUNT; i++) {
        if (migrateFileToFlash(sd, MIGRATE_FILES[i].sdPath, MIGRATE_FILES[i].flashPath)) {
          Serial.printf("[Flash] Migrated: %s -> %s\n",
                        MIGRATE_FILES[i].sdPath, MIGRATE_FILES[i].flashPath);
          copied++;
        }
      }
      xSemaphoreGive(sdMutex);
      Serial.printf("[Flash] Migration complete: %d files copied\n", copied);
    } else {
      Serial.println("[Flash] SD busy during migration, using defaults");
    }
  } else {
    Serial.println("[Flash] SD not available, writing hardcoded defaults");
  }

  // Write migration marker
  writeLFS("/config/.migrated", "1\n");
  Serial.println("[Flash] Migration marker written");
  return true;
}

String FlashConfig::readFile(const char* path) {
  return readLFS(path);
}

bool FlashConfig::writeFile(const char* path, const String& content) {
  bool ok = writeLFS(path, content);
  if (ok) s_dirty = true;
  return ok;
}

bool FlashConfig::exists(const char* path) {
  return LittleFS.exists(path);
}

bool FlashConfig::remove(const char* path) {
  bool ok = LittleFS.remove(path);
  if (ok) s_dirty = true;
  return ok;
}

bool FlashConfig::mkdir(const char* path) {
  return LittleFS.mkdir(path);
}

bool FlashConfig::readFileToBuffer(const char* path, char* buf, size_t bufSize) {
  if (!buf || bufSize < 2) return false;
  String content = readLFS(path);
  content.trim();
  if (content.length() == 0) return false;
  strncpy(buf, content.c_str(), bufSize - 1);
  buf[bufSize - 1] = '\0';
  return true;
}

void FlashConfig::markDirty() {
  s_dirty = true;
}

bool FlashConfig::isDirty() {
  return s_dirty;
}

void FlashConfig::syncToSD(SdFat& sd, SemaphoreHandle_t sdMutex, bool force) {
  if (!s_dirty && !force) return;
  if (!sdMutex) return;

  bool locked = (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100)) == pdTRUE);
  if (!locked) {
    Serial.println("[Flash] syncToSD: SD busy, skipping sync");
    return;
  }

  // Ensure /gnss exists on SD
  sd.mkdir("/gnss");

  int synced = 0;
  for (int i = 0; i < SYNC_COUNT; i++) {
    const char* flashPath = SYNC_FILES[i].flashPath;
    const char* sdPath    = SYNC_FILES[i].sdPath;

    if (!LittleFS.exists(flashPath)) continue;

    File fin = LittleFS.open(flashPath, "r");
    if (!fin) continue;

    FsFile fout = sd.open(sdPath, O_WRITE | O_CREAT | O_TRUNC);
    if (!fout) { fin.close(); continue; }

    // Copy in chunks
    uint8_t buf[SYNC_BUFFER_SIZE];
    while (fin.available()) {
      int n = fin.read(buf, sizeof(buf));
      if (n > 0) fout.write(buf, n);
    }
    fin.close();
    fout.close();
    synced++;
  }

  xSemaphoreGive(sdMutex);
  s_dirty = false;
  if (synced > 0) {
    Serial.printf("[Flash] syncToSD: %d files synced\n", synced);
  }
}

size_t FlashConfig::usedBytes() {
  return LittleFS.usedBytes();
}

size_t FlashConfig::totalBytes() {
  return LittleFS.totalBytes();
}

bool FlashConfig::isMigrated() {
  return LittleFS.exists("/config/.migrated");
}

// ---- Recursive SD directory removal helper ----
static void rmDirRecursiveSD(SdFat& sd, const char* path) {
  FsFile dir = sd.open(path, O_RDONLY);
  if (!dir || !dir.isDirectory()) { dir.close(); return; }
  
  FsFile entry;
  while (entry.openNext(&dir, O_RDONLY)) {
    char name[128];
    entry.getName(name, sizeof(name));
    
    String fullPath = String(path) + "/" + name;
    bool isDir = entry.isDirectory();
    entry.close();
    
    if (isDir) {
      rmDirRecursiveSD(sd, fullPath.c_str());
    } else {
      sd.remove(fullPath.c_str());
    }
  }
  dir.close();
  sd.rmdir(path);
}

bool FlashConfig::factoryReset(SdFat& sd, SemaphoreHandle_t sdMutex, bool wipeSD) {
  Serial.println("[Flash] === FACTORY RESET ===");

  // 1. Format entire LittleFS (wipes /config/, /surveys/, /stakeout/, everything)
  LittleFS.end();
  bool ok = LittleFS.format();
  if (ok) {
    LittleFS.begin(true);  // remount so ESP.restart() is clean
  }
  Serial.printf("[Flash] LittleFS format: %s\n", ok ? "OK" : "FAILED");

  // 2. Optionally wipe SD config and data directories
  if (wipeSD && sdMutex) {
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
      Serial.println("[Flash] Wiping SD directories...");
      rmDirRecursiveSD(sd, "/gnss");
      rmDirRecursiveSD(sd, "/surveys");
      rmDirRecursiveSD(sd, "/stakeout");
      Serial.println("[Flash] SD wipe completed");
      xSemaphoreGive(sdMutex);
    } else {
      Serial.println("[Flash] SD busy, could not wipe SD");
    }
  }

  Serial.println("[Flash] Factory reset complete. Reboot required.");
  return ok;
}
