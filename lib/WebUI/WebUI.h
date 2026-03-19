// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <WebServer.h>
#include "SdFat.h"

namespace WebUI {

  // Avvio interfaccia Web
  void begin(SdFat& sd, WebServer& server);

  // ===== Loaders (IN) =====
  bool loadNtrip(SdFat& sd,
                 String& host, int& port,
                 String& mountpoint, String& user, String& pass);

  // Compatibilità con vecchio config.txt
  bool loadOldConfigForNtrip(SdFat& sd,
                             String& host, int& port,
                             String& mountpoint, String& user, String& pass);

  // ===== OUT (compat pre-esistente) =====
  bool loadNtripOut(SdFat& sd,
                    String& host, int& port,
                    String& mount, String& pass);

  bool saveNtripOut(SdFat& sd,
                    const String& host, int port,
                    const String& mount, const String& pass);

  // ===== NEW: LAN TCP-IN loader (profilo selezionato) =====
  bool loadTcpIn(SdFat& sd, String& host, int& port);
}
