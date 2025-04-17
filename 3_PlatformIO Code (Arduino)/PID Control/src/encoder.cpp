#include <Arduino.h>
#include "encoder.h"
#include <AS5600.h>
#include <Wire.h>

Encoder::Encoder() {}
Encoder::Encoder(int encoderPinA, int encoderPinB)
{
}
Encoder::~Encoder() {}

void Encoder::setup()
{
    Wire.begin();
}
void Encoder::setup(int encoderPinA, int encoderPinB)
{
    setup();
}