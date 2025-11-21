#include "HeaterController.h"
#include <math.h>

// Define static constants
const float HeaterController::VCC      = 3.3;
const float HeaterController::R_FIXED  = 100000.0;
const float HeaterController::R0       = 100000.0;
const float HeaterController::T0_K     = 298.15;
const float HeaterController::BETA     = 3950.0;
const bool  HeaterController::THERMISTOR_BOTTOM = true;
const float HeaterController::DUTY_MAX = 40.0f;
const float HeaterController::DUTY_MIN = 0.0f;
const uint32_t HeaterController::TS_MS = 20;
const float HeaterController::ALPHA    = 0.30f;

HeaterController::HeaterController(Adafruit_PWMServoDriver pwm_servo_driver, int therm_pin, uint8_t chPWM, uint8_t chLOW, int sleep_pin)
    : m_pwm(pwm_servo_driver), m_therm_pin(therm_pin), m_chPWM(chPWM), m_chLOW(chLOW), m_sleep_pin(sleep_pin) {

    // Default PID gains
    m_Kp = 2.5f;
    m_Ki = 0.4f;
    m_Kd = 0.02f;

    // Initialize state
    m_targetTempC = 25.0f;
    m_e_prev = 0.0f;
    m_I_term = 0.0f;
    m_T_filt = 25.0f;
    m_currentDuty = 0.0f;
    m_last_ms = 0;
}

void HeaterController::begin() {
    pinMode(m_sleep_pin, OUTPUT);
    digitalWrite(m_sleep_pin, HIGH); //Always keep driver awake as requested

    m_pwm.begin();
    m_pwm.setPWMFreq(1000); // 1 kHz PWM
    setDutyPercent(0.0f);

    // Get an initial temperature reading
    m_T_filt = readTemperatureC();
    m_last_ms = millis();
}

void HeaterController::update() {
    uint32_t now = millis();
    if (now - m_last_ms < TS_MS) {
        return; // Not time to update yet
    }
    float dt = (now - m_last_ms) / 1000.0f;
    m_last_ms = now;

    // --- Measure + Filter ---
    float T = readTemperatureC();
    m_T_filt = ALPHA * T + (1.0f - ALPHA) * m_T_filt;

    // --- PID Controller ---
    float e = m_targetTempC - m_T_filt;
    float P = m_Kp * e;
    float D = m_Kd * (e - m_e_prev) / dt;
    float I_new = m_I_term + m_Ki * e * dt;

    float u_unclamped = P + I_new + D;

    // --- Clamp + Anti-windup ---
    float u = u_unclamped;
    if (u > DUTY_MAX) u = DUTY_MAX;
    if (u < DUTY_MIN) u = DUTY_MIN;

    bool at_upper = (u >= DUTY_MAX - 1e-6f);
    bool at_lower = (u <= DUTY_MIN + 1e-6f);

    // Only integrate if not at limits OR if integrating would move away from limit
    if (!((at_upper && e > 0) || (at_lower && e < 0))) {
        m_I_term = I_new;
    }

    // --- Apply Output ---
    setDutyPercent(u);
    m_currentDuty = u; // Store the duty cycle
    m_e_prev = e;
}

void HeaterController::setTargetTemperature(float tempC) {
    // Clamp the target temperature to a safe range
    if (tempC < 0.0f) tempC = 0.0f;
    if (tempC > 120.0f) tempC = 120.0f; // Safety clamp

    m_targetTempC = tempC;

    // Reset integral/derivative history to avoid wind-up/overshoot on step change
    m_I_term = 0.0f;
    m_e_prev = 0.0f; // Will be set based on new error in next update
}

float HeaterController::getCurrentTemperature() const {
    return m_T_filt;
}

float HeaterController::getDutyCycle() const {
    return m_currentDuty;
}

void HeaterController::setGains(float Kp, float Ki, float Kd) {
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
}

float HeaterController::getCurrentTarget() const {
    return m_targetTempC;
}

// ---- Private Helper Methods ----

uint16_t HeaterController::percentToCounts(float pct) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint16_t)(4095.0f * (pct / 100.0f));
}

void HeaterController::setDutyPercent(float pct) {
    m_pwm.setPWM(m_chPWM, 0, 0);
    m_pwm.setPWM(m_chLOW, 0, 0);
    if (pct <= 0.5f) {
        //We don't sleep the driver since other heaters might need it, just stop this one instead
        return;
    }
    else {
        m_pwm.setPWM(m_chPWM, 0, percentToCounts(pct));
    }
}

float HeaterController::readTemperatureC() {
    // Oversample for noise reduction
    const int N = 8;
    uint32_t acc = 0;
    for (int i = 0; i < N; ++i) {
        acc += analogRead(m_therm_pin);
    }
    float adc = acc / (float)N;

    float vout = (adc / 4095.0f) * VCC;

    float rth;
    if (THERMISTOR_BOTTOM) {
        if (VCC - vout < 1e-6f) vout = VCC - 1e-6f; // Avoid division by zero
        rth = R_FIXED * vout / (VCC - vout);
    } else {
        if (vout < 1e-6f) vout = 1e-6f; // Avoid division by zero
        rth = R_FIXED * (VCC - vout) / vout;
    }

    float invT = (1.0f / T0_K) + (1.0f / BETA) * log(rth / R0);
    float tempK = 1.0f / invT;
    return tempK - 273.15f;
}
