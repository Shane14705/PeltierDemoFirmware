//
// Created by Shane McKelvey on 11/19/25.
//

#ifndef PELTIERDEMOFIRMWARE_HAPTICMOTOR_H
#define PELTIERDEMOFIRMWARE_HAPTICMOTOR_H


#include <Arduino.h>

class HapticMotor {
public:
    // Constructor: defines the two pins used to drive the motor
    HapticMotor(int pin1, int pin2, uint32_t update_delay=20);

    void start();

    void stop();

    void update();

    // Check if currently enabled
    bool isEnabled();

    void pulse(int ms);

private:
    int _pin1;
    int _pin2;
    uint32_t _last_ms;
    uint32_t _update_delay;
    uint32_t _pulse_len;
    bool _enabled;
};


#endif //PELTIERDEMOFIRMWARE_HAPTICMOTOR_H