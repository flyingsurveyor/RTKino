// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include <vector>

// Tone structure for melodies
struct BuzzerTone {
    uint16_t freq;      // Frequency in Hz (0 = silence)
    uint16_t duration;  // Duration in milliseconds
};

// Predefined sound events
enum class BuzzerEvent {
    RTK_FIXED,      // RTK Fix acquired
    RTK_LOST,       // RTK Fix lost
    CUSTOM          // Custom melody from JSON
};

class Buzzer {
public:
    Buzzer(uint8_t gpio = 5, uint8_t ledcChannel = 0);
    ~Buzzer();
    
    // Initialize the buzzer (must be called in setup)
    bool begin();
    
    // Stop all buzzer activity
    void end();
    
    // Play a predefined event sound
    void playEvent(BuzzerEvent event);
    
    // Play a custom melody (non-blocking)
    void playMelody(const std::vector<BuzzerTone>& melody);
    
    // Stop currently playing sound
    void stop();
    
    // Update function - call in loop() to handle non-blocking playback
    void loop();
    
    // Enable/disable buzzer
    void setEnabled(bool enabled);
    bool isEnabled() const { return m_enabled; }
    
    // Check if buzzer is currently playing
    bool isPlaying() const { return m_playing; }
    
    // Load custom melody from JSON string
    bool loadMelodyFromJson(const String& json, std::vector<BuzzerTone>& melody);
    
    // Load custom melody from JSON file (requires SdFat and path)
    bool loadCustomMelody(const char* path);
    
private:
    void playTone(uint16_t freq, uint16_t duration);
    void startTone(uint16_t freq);
    void stopTone();
    
    uint8_t m_gpio;
    uint8_t m_ledcChannel;
    bool m_initialized;
    bool m_enabled;
    bool m_playing;
    
    // Current melody being played
    std::vector<BuzzerTone> m_currentMelody;
    size_t m_currentToneIndex;
    uint32_t m_toneStartTime;
    
    // Predefined melodies
    static const std::vector<BuzzerTone> MELODY_RTK_FIXED;
    static const std::vector<BuzzerTone> MELODY_RTK_LOST;
};

#endif // BUZZER_H
