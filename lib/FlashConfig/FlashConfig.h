// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef FLASHCONFIG_H
#define FLASHCONFIG_H

#include <Arduino.h>
#include <SdFat.h>

namespace FlashConfig {
  // Initialize LittleFS. If first boot (flash empty), attempt migration from SD.
  // sdOk=true means SD card is available for migration.
  // sd and sdMutex are needed only for migration and sync.
  bool begin(SdFat& sd, SemaphoreHandle_t sdMutex, bool sdOk);

  // Basic file operations (on LittleFS — NO mutex needed)
  String readFile(const char* path);
  bool writeFile(const char* path, const String& content);
  bool exists(const char* path);
  bool remove(const char* path);
  bool mkdir(const char* path);

  // Read file into a char buffer (for C-style APIs)
  bool readFileToBuffer(const char* path, char* buf, size_t bufSize);

  // Mark that config has changed (for sync)
  void markDirty();
  bool isDirty();

  // Sync changed files from flash to SD card.
  // ONLY call when loggingActive==false and SD is available.
  // Uses sdMutex internally with short timeout.
  void syncToSD(SdFat& sd, SemaphoreHandle_t sdMutex, bool force = false);

  // Stats
  size_t usedBytes();
  size_t totalBytes();

  // Check if migration has been done
  bool isMigrated();
}

#endif
