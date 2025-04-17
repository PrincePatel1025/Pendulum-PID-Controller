#ifndef _ENCODER_h_
#define _ENCODER_h_
#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>

class Encoder
{
private:
    // float Kp = 0, Ki = 0, Kd = 0;
    AMS_5600 ams5600;

public:
    Encoder();
    Encoder(int encoderPinA, int encoderPinB);
    ~Encoder();

    void setup();
    void setup(int encoderPinA, int encoderPinB);

    void setGain(float Kp, float Ki, float Kd);
};

#endif