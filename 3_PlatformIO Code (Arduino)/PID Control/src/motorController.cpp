#include "motorController.h"
#include <Arduino.h>

MotorController::MotorController() {}
MotorController::MotorController(int R_EN, int L_EN, int RPWM, int LPWM)
{
    MotorController::R_EN = R_EN;
    MotorController::L_EN = L_EN;
    MotorController::RPWM = RPWM;
    MotorController::LPWM = LPWM;
}
MotorController::~MotorController() {}

void MotorController::setup(int R_EN, int L_EN, int RPWM, int LPWM)
{
    MotorController::R_EN = R_EN;
    MotorController::L_EN = L_EN;
    MotorController::RPWM = RPWM;
    MotorController::LPWM = LPWM;

    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);

    disable();
}

void MotorController::enable()
{
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
}
void MotorController::disable()
{
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
}

void MotorController::controlPWM(int PWM)
{
    if (PWM > 0)
    {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, PWM);
    }

    else if (PWM < 0)
    {
        analogWrite(LPWM, -1 * PWM);
        analogWrite(RPWM, 0);
    }

    else
    {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
    }
}
void MotorController::controlPWM(bool clockwise, unsigned int PWM)
{
    if (clockwise)
    {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, PWM);
    }
    else if (!clockwise)
    {
        analogWrite(LPWM, PWM);
        analogWrite(RPWM, 0);
    }
    else
    {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
    }
}

void MotorController::controlDutyCycle(int dutyCycle)
{
    int PWM = (255 * dutyCycle) / 100;

    if (dutyCycle > 0)
    {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, PWM);
    }
    else if (dutyCycle < 0)
    {
        analogWrite(LPWM, PWM);
        analogWrite(RPWM, 0);
    }
    else
    {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
    }
}
void MotorController::controlDutyCycle(bool clockwise, int dutyCycle)
{
    int PWM = (255 * dutyCycle) / 100;

    if (clockwise)
    {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, PWM);
    }
    else if (!clockwise)
    {
        analogWrite(LPWM, PWM);
        analogWrite(RPWM, 0);
    }
    else
    {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
    }
}