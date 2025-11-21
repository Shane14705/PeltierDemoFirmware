#ifndef HEATER_CONTROLLER_H
#define HEATER_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class HeaterController {
public:
    /**
     * @brief Constructor for the HeaterController.
     * @param pwm_servo_driver The PCA9685 PWM driver.
     * @param therm_pin The analog pin connected to the thermistor for this heater.
     * @param chPWM The PWM channel for this heater
     * @param chLOW Not sure what this is used for, but I saw it in the schematic
     * @param sleep_pin The digital pin connected to the DRV8212 nSLEEP.
     */
    HeaterController(Adafruit_PWMServoDriver pwm_servo_driver, int therm_pin, uint8_t chPWM, uint8_t chLOW, int sleep_pin = 10);

    /**
     * @brief Initializes the hardware (I2C, PWM driver, pins).
     * @param sda_pin The I2C SDA pin.
     * @param scl_pin The I2C SCL pin.
     */
    void begin();

    /**
     * @brief Main update loop. This should be called repeatedly in the main loop().
     * It handles temperature reading, filtering, and PID calculation at the correct interval.
     */
    void update();

    /**
     * @brief Sets the target temperature for the PID controller.
     * @param tempC The desired temperature in degrees Celsius.
     */
    void setTargetTemperature(float tempC);

    /**
     * @brief Gets the current filtered (smoothed) temperature.
     * @return The current temperature in degrees Celsius.
     */
    float getCurrentTemperature() const;

    /**
     * @brief Gets the current output duty cycle of the PID controller.
     * @return The current duty cycle as a percentage (0-100).
     */
    float getDutyCycle() const;

    /**
     * @brief Sets the PID gains.
     * @param Kp Proportional gain.
     * @param Ki Integral gain.
     * @param Kd Derivative gain.
     */
    void setGains(float Kp, float Ki, float Kd);

    float getCurrentTarget() const;

private:
    // ---- Hardware ----
    Adafruit_PWMServoDriver m_pwm;
    int m_therm_pin;
    uint8_t m_chPWM;
    uint8_t m_chLOW;
    int m_sleep_pin;

    // ---- Thermistor Constants ----
    static const float VCC;
    static const float R_FIXED;
    static const float R0;
    static const float T0_K;
    static const float BETA;
    static const bool  THERMISTOR_BOTTOM;

    // ---- Control Constants ----
    static const float DUTY_MAX;
    static const float DUTY_MIN;
    static const uint32_t TS_MS; // Loop interval
    static const float ALPHA; // Temp smoothing
    static const uint8_t CH_EN = 0; // PWM channel

    // ---- PID Gains ----
    float m_Kp;
    float m_Ki;
    float m_Kd;

    // ---- State Variables ----
    float m_targetTempC;
    float m_e_prev;
    float m_I_term;
    float m_T_filt;       // Filtered temperature
    float m_currentDuty;  // Current PWM duty cycle
    uint32_t m_last_ms;   // Last loop execution time

    // ---- Private Helper Methods ----
    uint16_t percentToCounts(float pct);
    void setDutyPercent(float pct);
    float readTemperatureC();
};

#endif // HEATER_CONTROLLER_H
