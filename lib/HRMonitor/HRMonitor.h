//
// Created by Shane McKelvey on 11/19/25.
//

#ifndef PELTIERDEMOFIRMWARE_HRMONITOR_H
#define PELTIERDEMOFIRMWARE_HRMONITOR_H


#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

// Buffer sizes
#define HR_BUF_MAX 700
#define HR_WINDOW 500

class HRMonitor {

public:
    HRMonitor();

    //We don't want copying, having more than one sensor instance per physical sensor is not good
    HRMonitor(const HRMonitor &other) = delete;
    HRMonitor & operator=(const HRMonitor &other) = delete;

    HRMonitor(HRMonitor &&other) noexcept
        : _ppg(std::move(other._ppg)),
          _active(other._active),
          _enabled(other._enabled),
          _latestHR(other._latestHR),
          _HR_FS(other._HR_FS),
          _HR_STEP(other._HR_STEP),
          _MUX_ADDR(other._MUX_ADDR),
          _MUX_CHANNEL_HR(other._MUX_CHANNEL_HR),
          _hrBuf(other._hrBuf),
          _hrWriteIndex(other._hrWriteIndex),
          _totalSamples(other._totalSamples),
          _samplesSinceLastHR(other._samplesSinceLastHR),
          _hrRawF(other._hrRawF),
          _hp(other._hp),
          _lp(other._lp),
          _env(other._env),
          _peakMask(other._peakMask) {

        other._hrBuf = nullptr;
        other._hrRawF = nullptr;
        other._hp = nullptr;
        other._lp = nullptr;
        other._env = nullptr;
        other._peakMask = nullptr;
    }

    HRMonitor & operator=(HRMonitor &&other) noexcept {
        if (this == &other)
            return *this;
        _ppg = std::move(other._ppg);
        _active = other._active;
        _enabled = other._enabled;
        _latestHR = other._latestHR;
        _hrBuf = other._hrBuf;
        _hrWriteIndex = other._hrWriteIndex;
        _totalSamples = other._totalSamples;
        _samplesSinceLastHR = other._samplesSinceLastHR;
        _hrRawF = other._hrRawF;
        _hp = other._hp;
        _lp = other._lp;
        _env = other._env;
        _peakMask = other._peakMask;

        other._hrBuf = nullptr;
        other._hrRawF = nullptr;
        other._hp = nullptr;
        other._lp = nullptr;
        other._env = nullptr;
        other._peakMask = nullptr;
        return *this;
    }

    ~HRMonitor();

    // Start the sensor and MUX
    void start_sensor();

    // Stop the sensor
    void stop();

    // Main processing loop (call this in loop())
    void process();

    // Get the latest calculated BPM
    float getLatestHR();

    // Check if sensor is currently active/enabled
    bool isActive();
    bool isEnabled();

    // Toggle enable state
    void setEnabled(bool enabled);

    // Print debug data to Serial
    void debugPrint();

private:
    MAX30105 _ppg;
    bool _active;
    bool _enabled;
    float _latestHR;

    // Configuration
    const float _HR_FS = 50.0f;
    const int _HR_STEP = 200;
    const int _MUX_ADDR = 0x70;
    const int _MUX_CHANNEL_HR = 0b00000010;

    // Buffers
    uint32_t* _hrBuf;
    int _hrWriteIndex;
    unsigned long _totalSamples;
    int _samplesSinceLastHR;

    // Signal Processing Arrays
    float* _hrRawF;
    float* _hp;
    float* _lp;
    float* _env;
    int* _peakMask;

    // Internal Methods
    void muxSelect(uint8_t mask);
    void collectSamples();
    float computeHR_dynamic();
};


#endif //PELTIERDEMOFIRMWARE_HRMONITOR_H