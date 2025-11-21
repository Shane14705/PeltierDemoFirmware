#include "HRMonitor.h"

HRMonitor::HRMonitor() {
    _active = false;
    _enabled = false;
    _latestHR = 0.0f;
    _hrWriteIndex = 0;
    _totalSamples = 0;
    _samplesSinceLastHR = 0;

    //Buffers and Arrays are allocated on heap since they wont fit on stack
    _hrBuf = new uint32_t[HR_BUF_MAX];
    _hrRawF = new float[HR_WINDOW];
    _hp = new float[HR_WINDOW];
    _lp = new float[HR_WINDOW];
    _env = new float[HR_WINDOW];
    _peakMask = new int[HR_WINDOW];
}

HRMonitor::~HRMonitor() {
    //Cleanup Buffers
    delete[] _hrBuf;
    delete[] _hrRawF;
    delete[] _hp;
    delete[] _lp;
    delete[] _env;
    delete[] _peakMask;
}

void HRMonitor::muxSelect(uint8_t mask) {
    Wire.beginTransmission(_MUX_ADDR);
    Wire.write(mask);
    Wire.endTransmission();
    delay(5);
}

void HRMonitor::start_sensor() {
    muxSelect(_MUX_CHANNEL_HR);

    if (!_ppg.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("HR Sensor NOT FOUND!");
        return;
    }

    _ppg.setup(
        60,   // LED brightness
        4,    // sample average
        3,    // LED mode (Red+IR+Green)
        200,  // sample rate setting (actual = ~50 Hz)
        411,  // pulse width
        4096  // ADC range
    );

    // LED amplitude settings: RED only
    _ppg.setPulseAmplitudeRed(60);
    _ppg.setPulseAmplitudeIR(0);
    _ppg.setPulseAmplitudeGreen(0);

    _hrWriteIndex = 0;
    _totalSamples = 0;
    _samplesSinceLastHR = 0;
    _latestHR = 0;
    _active = true;
    
    Serial.println("HR Sensor ON");
}

void HRMonitor::stop() {
    if (_active) {
        muxSelect(_MUX_CHANNEL_HR); // Ensure we talk to the right device
        _ppg.shutDown();
        _active = false;
    }
    Serial.println("HR Sensor OFF");
}

void HRMonitor::setEnabled(bool enabled) {
    _enabled = enabled;
    if (_enabled) {
        if (!_active) start_sensor();
    } else {
        stop();
    }
}

bool HRMonitor::isEnabled() { return _enabled; }
bool HRMonitor::isActive() { return _active; }
float HRMonitor::getLatestHR() { return _latestHR; }

void HRMonitor::process() {
    if (!_enabled) {
        if (_active) stop();
        return;
    }

    if (!_active) start_sensor();

    collectSamples();
    computeHR_dynamic();
}

void HRMonitor::collectSamples() {
    _ppg.check();

    if (_ppg.available()) {
        uint32_t g = _ppg.getRed();

        _hrBuf[_hrWriteIndex] = g;
        _hrWriteIndex = (_hrWriteIndex + 1) % HR_BUF_MAX;

        _totalSamples++;
        _samplesSinceLastHR++;
        
        _ppg.nextSample();
    }
}

float HRMonitor::computeHR_dynamic() {
    if (_totalSamples < (unsigned long)HR_WINDOW) return _latestHR;
    if (_samplesSinceLastHR < _HR_STEP) return _latestHR;

    _samplesSinceLastHR = 0;

    // 1) Extract window
    int start = _hrWriteIndex - HR_WINDOW;
    if (start < 0) start += HR_BUF_MAX;

    float meanRaw = 0.0f;
    for (int i = 0; i < HR_WINDOW; i++) {
        _hrRawF[i] = (float)_hrBuf[(start + i) % HR_BUF_MAX];
        meanRaw += _hrRawF[i];
    }
    meanRaw /= (float)HR_WINDOW;

    // 2) Baseline removal
    float baseline = _hrRawF[0];
    for (int i = 0; i < HR_WINDOW; i++) {
        baseline = baseline + 0.01f * (_hrRawF[i] - baseline);
        _hp[i] = _hrRawF[i] - baseline;
    }

    // 3) Low-pass filter
    float alphaLP = 0.28f;
    float lp_prev = _hp[0];
    for (int i = 0; i < HR_WINDOW; i++) {
        lp_prev = lp_prev + alphaLP * (_hp[i] - lp_prev);
        _lp[i] = lp_prev;
    }

    // 4) Envelope (Debug)
    const int ENV_W = 5;
    float sumEnv = 0.0f;
    for (int i = 0; i < HR_WINDOW; i++) {
        sumEnv += fabsf(_lp[i]);
        if (i >= ENV_W) sumEnv -= fabsf(_lp[i - ENV_W]);
        _env[i] = (i < ENV_W) ? sumEnv / (float)(i + 1) : sumEnv / (float)ENV_W;
    }

    // 5) Adaptive threshold
    float meanLP = 0.0f;
    for (int i = 0; i < HR_WINDOW; i++) meanLP += _lp[i];
    meanLP /= (float)HR_WINDOW;

    float varLP = 0.0f;
    for (int i = 0; i < HR_WINDOW; i++) {
        float d = _lp[i] - meanLP;
        varLP += d * d;
    }
    varLP /= (float)HR_WINDOW;
    float stdLP = sqrtf(varLP);

    if (stdLP < 5.0f) return _latestHR; // Signal too weak

    float thresh = meanLP + 1.0f * stdLP;

    // 6) Peak detection
    const int MIN_DIST = (int)(0.30f * _HR_FS);
    int lastPeak = -MIN_DIST;
    const int MAX_PEAKS = 25;
    int peakPos[MAX_PEAKS];
    int peaks = 0;

    memset(_peakMask, 0, sizeof(int) * HR_WINDOW); //Clear Mask

    float prevDiff = 0.0f;
    for (int i = 1; i < HR_WINDOW; i++) {
        float diff = _lp[i] - _lp[i - 1];

        if (prevDiff > 0 && diff <= 0 && _lp[i] > thresh && (i - lastPeak) >= MIN_DIST) {
            if (peaks < MAX_PEAKS) {
                peakPos[peaks] = i;
            }
            peaks++;
            lastPeak = i;
            if (i < HR_WINDOW) _peakMask[i] = 1;
        }
        prevDiff = diff;
    }

    //Not enough peaks, keep previous HR
    if (peaks < 2) return _latestHR;

    // 7) RR Intervals
    float rrBpm[MAX_PEAKS];
    int nRR = 0;

    for (int k = 1; k < peaks && k < MAX_PEAKS; k++) {
        int diff = peakPos[k] - peakPos[k - 1];
        if (diff <= 0) continue;
        float dt = diff / _HR_FS;  // seconds
        float bpm = 60.0f / dt;

        if (bpm > 40.0f && bpm < 160.0f) {
            rrBpm[nRR++] = bpm;
        }
    }

    if (nRR == 0) return _latestHR;

    // 8) Median
    for (int i = 1; i < nRR; i++) {
        float key = rrBpm[i];
        int j = i - 1;
        while (j >= 0 && rrBpm[j] > key) {
            rrBpm[j + 1] = rrBpm[j];
            j--;
        }
        rrBpm[j + 1] = key;
    }

    float median;
    if (nRR % 2 == 1) median = rrBpm[nRR / 2];
    else median = 0.5f * (rrBpm[nRR / 2 - 1] + rrBpm[nRR / 2]);

    // 9) Smooth update
    if (_latestHR <= 0.0f) {
        _latestHR = median;
    } else {
        float diff = fabsf(median - _latestHR);
        if (diff > 20.0f) _latestHR = 0.7f * _latestHR + 0.3f * median;
        else _latestHR = 0.5f * _latestHR + 0.5f * median;
    }

    return _latestHR;
}

void HRMonitor::debugPrint() {
    if (_totalSamples < (unsigned long)HR_WINDOW) {
        Serial.println("Not enough HR samples collected yet.");
        return;
    }

    Serial.println("i,RAW,HP,LP,ENV,PEAK");
    for (int i = 0; i < HR_WINDOW; i++) {
        Serial.print(i); Serial.print(",");
        Serial.print(_hrRawF[i]); Serial.print(",");
        Serial.print(_hp[i]); Serial.print(",");
        Serial.print(_lp[i]); Serial.print(",");
        Serial.print(_env[i]); Serial.print(",");
        Serial.println(_peakMask[i]);
    }
    Serial.println("END");
}