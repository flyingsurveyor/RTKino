// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
// Hardware configuration for RTKino project
// Minimum supported hardware: ESP32-S3 with 16MB Flash and 8MB PSRAM

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// BOARD-SPECIFIC PIN CONFIGURATION
// ============================================================================

#ifdef BOARD_LOLIN_S3_PRO
  // -------------------------------------------------------------------------
  // LOLIN S3 PRO Configuration
  // -------------------------------------------------------------------------
  // Features:
  // - Integrated microSD slot (native SPI)
  // - 16MB Flash, 8MB PSRAM
  // - Native I2C connector
  // -------------------------------------------------------------------------
  
  // UART Pins (ZED-F9P connections)
  #define RAW_UART_RX 42        // UART2: ZED UART1 TX (RAW only)
  #define UBX_UART_RX 39        // UART1: ZED UART2 TX (NMEA/UBX/RTCM base)
  #define RTCM_UART_TX 40       // UART2: ZED UART2 RX (RTCM input per rover)
  
  // I2C Pins (OLED + ZED-F9P on same bus)
  #define ZED_I2C_SDA 9         // I2C SDA (native connector)
  #define ZED_I2C_SCL 10        // I2C SCL (native connector)
  #define OLED_I2C_SDA 9        // OLED on same I2C bus as ZED
  #define OLED_I2C_SCL 10       // OLED on same I2C bus as ZED
  
  // GPIO Pins
  #define BUZZER_GPIO 4         // Buzzer PWM output
  #define EXTINT_GPIO 5         // ZED-F9P EXTINT — PPK event marker (TIM-TM2)
  
  // SD Card (integrated slot on SPI bus - NOT SDMMC)
  // TF_CS = GPIO 46 per board variant
  #define SD_CS   46            // TF_CS - SD card chip select
  #define SD_MOSI 11            // SPI MOSI
  #define SD_MISO 13            // SPI MISO
  #define SD_SCK  12            // SPI SCK
  #define USE_EXTERNAL_SD       // Uses SPI like the Mini, just different CS pin
  
  // Board info
  #define BOARD_NAME "LOLIN S3 Pro"
  #define BOARD_FLASH_SIZE "16MB"
  #define BOARD_PSRAM_SIZE "8MB"

#else
  #error "No board defined! Use -DBOARD_LOLIN_S3_PRO"
#endif

// ============================================================================
// COMMON CONFIGURATION (same for all boards)
// ============================================================================

// UART Settings
#define UART_BAUD 230400

// I2C Settings
#define ZED_I2C_ADDR 0x42

// Buzzer Settings
#define BUZZER_LEDC_CHANNEL 0

// Memory thresholds
#define HEAP_WARNING_THRESHOLD 50000  // Low heap warning threshold (bytes)

// Queue and buffer sizes
#define QUEUE_SIZE 32
#define PACKET_SIZE 2048

// WiFi fallback credentials
#define DEFAULT_WIFI_SSID      "SSID"
#define DEFAULT_WIFI_PASSWORD  "password"

// Access Point offline mode
#define AP_SSID "rtkino_AP"
#define AP_PASS "rtkino_zedf9p"

// ============================================================================
// ESP-NOW MESH CONFIGURATION
// ============================================================================

// Fixed WiFi channel for ESP-NOW mesh (all nodes must use the same channel)
// Channel 6: common default shared with Crocevia base station
// (channels 1, 6, 11 are non-overlapping; channel 6 is the mesh network default)
#define ESPNOW_WIFI_CHANNEL    6

// Network ID: 32-bit token shared by all nodes of the same network.
// Acts as a lightweight network filter (not cryptographic).
// Change this value to isolate different field networks.
#define ESPNOW_NETWORK_ID      0x52544B4EUL   // "RTKN"

// Maximum accepted age of an RTCM fragment in milliseconds.
// Fragments older than this are dropped to prevent stale corrections.
// RTK is very sensitive to correction age; keep this tight.
#define ESPNOW_MAX_AGE_MS      800

// Default TTL (max hops) for RTCM packets
#define ESPNOW_TTL             5

// RX queue depth (Core 0 → Core 1 handoff)
#define ESPNOW_RX_QUEUE_SIZE   8

// Telemetry broadcast interval in milliseconds
#define ESPNOW_TELEM_INTERVAL_MS  5000

// RTCM payload size per ESP-NOW packet (max 250 - header overhead)
#define ESPNOW_PAYLOAD_SIZE    200

// ============================================================================
// ROTARY ENCODER PINS
// Set all three to the actual GPIO numbers for your wiring.
// Leave at 0 to disable encoder support entirely (no menu, no ISR).
// ============================================================================
#define ENC_CLK_GPIO 16    // Rotary CLK pin — user must set manually S1
#define ENC_DT_GPIO  17    // Rotary DT  pin — user must set manually S2
#define ENC_SW_GPIO  18    // Rotary SW  pin — user must set manually KEY

// ============================================================================
// HELPER MACROS FOR CONDITIONAL CODE
// ============================================================================

// Board identification at runtime
inline const char* getBoardName() { return BOARD_NAME; }
inline const char* getBoardFlash() { return BOARD_FLASH_SIZE; }
inline const char* getBoardPsram() { return BOARD_PSRAM_SIZE; }

#endif // CONFIG_H
