#include "motor.h"

Motor::Motor(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2) {
    _pinEnable = pinEnable;
    _pinIN1 = pinIN1;
    _pinIN2 = pinIN2;
    _pwmValue = 255; 

    pinMode(_pinEnable, OUTPUT);
    pinMode(_pinIN1, OUTPUT);
    pinMode(_pinIN2, OUTPUT);
}

void Motor::setSpeed(unsigned short pwmValue) {
    _pwmValue = pwmValue;
    analogWrite(_pinEnable, _pwmValue);
}

void Motor::forward() {
    digitalWrite(_pinIN1, HIGH);
    digitalWrite(_pinIN2, LOW);

    analogWrite(_pinEnable, _pwmValue);
}

void Motor::backward() {
    digitalWrite(_pinIN1, LOW);
    digitalWrite(_pinIN2, HIGH);

    analogWrite(_pinEnable, _pwmValue);
}

void Motor::brake() {
    analogWrite(_pinEnable, 255);

    digitalWrite(_pinIN1, HIGH);
    digitalWrite(_pinIN2, HIGH);
}

void Motor::stop() {
    digitalWrite(_pinIN1, LOW);
    digitalWrite(_pinIN2, LOW);

    _pwmValue = 0;
    analogWrite(_pinEnable, _pwmValue);
   
}