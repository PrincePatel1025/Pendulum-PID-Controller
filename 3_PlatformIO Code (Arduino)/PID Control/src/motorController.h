#ifndef _MotorController_h_
#define _MotorController_h_

#include <Arduino.h>

class MotorController
{
private:
    int R_EN = 0;
    int L_EN = 0;
    int RPWM = 0; // 5
    int LPWM = 0; // 6

public:
    MotorController();
    MotorController(int R_EN, int L_EN, int RPWM, int LPWM);
    ~MotorController();

    //! ----------------  SETUP  ---------------------------------------------------------------
    void setup(int R_EN, int L_EN, int RPWM, int LPWM);

    //! ----------------  On / Off  ------------------------------------------------------------
    void enable();
    void disable();

    //! ----------------  PWM control  ---------------------------------------------------------
    void controlPWM(int PWM);                          //*  PWM = -255 to 255
    void controlPWM(bool clockwise, unsigned int PWM); //*  PWM = 0 to 255

    //! ----------------  dutyCycle Control  ---------------------------------------------------
    void controlDutyCycle(int dutyCycle);                 //*  dutyCycle = -100 to 100
    void controlDutyCycle(bool clockwise, int dutyCycle); //*  dutyCycle = 0 to 100
};

#endif