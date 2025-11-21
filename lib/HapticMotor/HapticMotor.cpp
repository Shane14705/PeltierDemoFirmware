//
// Created by Shane McKelvey on 11/19/25.
//

#include "HapticMotor.h"

HapticMotor::HapticMotor(int pin1, int pin2, uint32_t update_delay) : _pin1(pin1), _pin2(pin2), _update_delay(update_delay) {
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    _enabled = false;
}

void HapticMotor::start() {
    _enabled = true;
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, HIGH);
}

void HapticMotor::stop() {
    _enabled = false;
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
}



void HapticMotor::update() {
    if (_enabled) {
        if ((millis() - _last_ms) > _pulse_len) {
            this->stop();
        }
    }
}

//Pulse length isn't precise but should be good enough
void HapticMotor::pulse(int ms) {
    _pulse_len = ms;
    _last_ms = millis();
    this->start();

}
