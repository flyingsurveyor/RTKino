// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <WebServer.h>
#include "SdFat.h"

namespace WebUI {

  // Start web interface
  void begin(SdFat& sd, WebServer& server);

  // ===== Loaders (IN) =====
  bool loadNtrip(SdFat& sd,
                 String& host, int& port,
                 String& mountpoint, String& user, String& pass);

  // Compatibility with legacy config.txt
  bool loadOldConfigForNtrip(SdFat& sd,
                             String& host, int& port,
                             String& mountpoint, String& user, String& pass);

  // ===== OUT (legacy compat) =====
  bool loadNtripOut(SdFat& sd,
                    String& host, int& port,
                    String& mount, String& pass);

  bool saveNtripOut(SdFat& sd,
                    const String& host, int port,
                    const String& mount, const String& pass);

  // ===== NEW: LAN TCP-IN loader (selected profile) =====
  bool loadTcpIn(SdFat& sd, String& host, int& port);
}
