// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "Buzzer.h"
#include <ArduinoJson.h>

// JSON document size for melody loading
#define BUZZER_JSON_BUFFER_SIZE 2048

// Predefined melodies
const std::vector<BuzzerTone> Buzzer::MELODY_RTK_FIXED = {
    {880, 100},   // A5 - short beep
    {0, 50},      // pause
    {1318, 150}   // E6 - longer ascending beep
};

const std::vector<BuzzerTone> Buzzer::MELODY_RTK_LOST = {
    {440, 150},   // A4 - low beep
    {0, 50},      // pause
    {392, 150},   // G4 - lower beep
    {0, 50},      // pause
    {349, 150}    // F4 - lowest beep
};

Buzzer::Buzzer(uint8_t gpio, uint8_t ledcChannel)
    : m_gpio(gpio)
    , m_ledcChannel(ledcChannel)
    , m_initialized(false)
    , m_enabled(true)
    , m_playing(false)
    , m_currentToneIndex(0)
    , m_toneStartTime(0)
{
}

Buzzer::~Buzzer() {
    end();
}

bool Buzzer::begin() {
    if (m_initialized) {
        return true;
    }
    
    // Configure LEDC for PWM tone generation
    ledcSetup(m_ledcChannel, 0, 8);  // 0 Hz initially, 8-bit resolution
    ledcAttachPin(m_gpio, m_ledcChannel);
    ledcWrite(m_ledcChannel, 0);  // Start silent
    
    m_initialized = true;
    return true;
}

void Buzzer::end() {
    if (!m_initialized) {
        return;
    }
    
    stop();
    ledcDetachPin(m_gpio);
    m_initialized = false;
}

void Buzzer::playEvent(BuzzerEvent event) {
    if (!m_enabled || !m_initialized) {
        return;
    }
    
    switch (event) {
        case BuzzerEvent::RTK_FIXED:
            playMelody(MELODY_RTK_FIXED);
            break;
        case BuzzerEvent::RTK_LOST:
            playMelody(MELODY_RTK_LOST);
            break;
        default:
            break;
    }
}

void Buzzer::playMelody(const std::vector<BuzzerTone>& melody) {
    if (!m_enabled || !m_initialized || melody.empty()) {
        return;
    }
    
    stop();  // Stop any current playback
    
    m_currentMelody = melody;
    m_currentToneIndex = 0;
    m_toneStartTime = millis();
    m_playing = true;
    
    // Start first tone
    const BuzzerTone& tone = m_currentMelody[0];
    if (tone.freq > 0) {
        startTone(tone.freq);
    }
}

void Buzzer::stop() {
    if (!m_initialized) {
        return;
    }
    
    stopTone();
    m_playing = false;
    m_currentMelody.clear();
    m_currentToneIndex = 0;
}

void Buzzer::loop() {
    if (!m_playing || !m_initialized || m_currentMelody.empty()) {
        return;
    }
    
    uint32_t now = millis();
    const BuzzerTone& currentTone = m_currentMelody[m_currentToneIndex];
    
    // Check if current tone has finished
    if (now - m_toneStartTime >= currentTone.duration) {
        m_currentToneIndex++;
        
        // Check if melody is complete
        if (m_currentToneIndex >= m_currentMelody.size()) {
            stop();
            return;
        }
        
        // Start next tone
        const BuzzerTone& nextTone = m_currentMelody[m_currentToneIndex];
        m_toneStartTime = now;
        
        if (nextTone.freq > 0) {
            startTone(nextTone.freq);
        } else {
            stopTone();  // Silence/pause
        }
    }
}

void Buzzer::setEnabled(bool enabled) {
    m_enabled = enabled;
    if (!enabled && m_playing) {
        stop();
    }
}

bool Buzzer::loadMelodyFromJson(const String& json, std::vector<BuzzerTone>& melody) {
    DynamicJsonDocument doc(BUZZER_JSON_BUFFER_SIZE);
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
        Serial.printf("[Buzzer] JSON parse error: %s\n", error.c_str());
        return false;
    }
    
    if (!doc.containsKey("tones")) {
        Serial.println("[Buzzer] JSON missing 'tones' array");
        return false;
    }
    
    JsonArray tones = doc["tones"].as<JsonArray>();
    if (tones.size() == 0) {
        Serial.println("[Buzzer] Empty tones array");
        return false;
    }
    
    melody.clear();
    for (JsonObject tone : tones) {
        if (!tone.containsKey("freq") || !tone.containsKey("duration")) {
            Serial.println("[Buzzer] Tone missing freq or duration");
            return false;
        }
        
        BuzzerTone t;
        t.freq = tone["freq"].as<uint16_t>();
        t.duration = tone["duration"].as<uint16_t>();
        melody.push_back(t);
    }
    
    return true;
}

bool Buzzer::loadCustomMelody(const char* path) {
    // TODO: Implement actual melody loading from file
    // Note: This method is designed to be called from WebUI
    // which handles SD locking. Do NOT lock SD here to avoid deadlock.
    // When implemented, this should:
    // 1. Read JSON from path
    // 2. Parse using loadMelodyFromJson()
    // 3. Store as a custom melody event
    // 4. Return false on any error
    Serial.printf("[Buzzer] loadCustomMelody called for %s (stub)\n", path);
    return true;  // Stub - actual loading to be implemented
}

void Buzzer::startTone(uint16_t freq) {
    if (!m_initialized || freq == 0) {
        return;
    }
    
    // Reconfigure LEDC for the new frequency
    ledcSetup(m_ledcChannel, freq, 8);
    ledcAttachPin(m_gpio, m_ledcChannel);
    ledcWrite(m_ledcChannel, 128);  // 50% duty cycle
}

void Buzzer::stopTone() {
    if (!m_initialized) {
        return;
    }
    
    ledcWrite(m_ledcChannel, 0);  // 0% duty cycle = silence
}
