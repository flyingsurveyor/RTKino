// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <Arduino.h>
#include <WiFi.h>

class RTCMInput {
public:
  RTCMInput();                                   // dichiarazione
  RTCMInput(const String& host, uint16_t port);  // dichiarazione

  // Configura host/port (solo memorizzazione parametri)
  bool configure(const String& host, uint16_t port);

  // Avvia la connessione e instrada i dati su una UART (RTCM -> ZED RX2)
  bool begin(HardwareSerial& out);

  // Ferma connessione
  void stop();

  // Chiamare spesso dal loop()
  void loop();
  
  // Force reconnection after WiFi recovery
  void forceReconnect();

  // Stato
  bool isActive() const { return _active; }
  bool isConnected(); // non-const (WiFiClient::connected() non è const)

  // Getter parametri
  const String& host() const { return _host; }
  uint16_t port() const { return _port; }

private:
  WiFiClient _client;
  HardwareSerial* _out = nullptr;

  String _host;
  uint16_t _port = 0;

  bool _active = false;
  uint32_t _lastReconnectMs = 0;
  
  // Static buffer for data reception (instead of stack allocation)
  uint8_t _rxBuffer[1024];
};

// ====== definizioni inline dei costruttori ======
inline RTCMInput::RTCMInput() = default;

inline RTCMInput::RTCMInput(const String& host, uint16_t port) {
  this->configure(host, port); // usa this-> per evitare ambiguità
}
