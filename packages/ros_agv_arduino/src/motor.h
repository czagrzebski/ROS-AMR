#ifndef motor_h
#define motor_h

#include <Arduino.h>

class Motor {
    public:
        Motor(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2);
        void forward();
        void backward();
        void stop();
        void brake();
        void setSpeed(int pwmValue);
    private:
        byte _pinEnable;
        byte _pinIN1;
        byte _pinIN2;
        double _pwmValue;
};


#endif