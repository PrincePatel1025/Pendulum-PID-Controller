#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include "motorController.h"

AMS_5600 ams5600;
MotorController motor;
// int convertRawAngleToDegrees(word newAngle);
float func(int angle);

int startDeg = 0;
float angle = 0;
float pastAngle = 0;
int error = 0;
unsigned long startTime = 0;
unsigned long previousTime = 0;
float outputGain = 0;
float integral = 0;
float derevative = 0;
float pastDerevative = 0;
float pastDerevative2 = 0;
float dGain = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  motor.setup(8, 7, 6, 5);
  motor.enable();

  startDeg = ams5600.getRawAngle();
  startTime = millis();
}
// float Kp = 5;
// float Ki = 0.003;
// float Kd = 250;

float Kp = 3.5;
float Ki = 0.004;
float Kd = 50;

void loop()
{
  angle = func(ams5600.getRawAngle());
  startTime = millis();

  integral += ((startTime - previousTime) * (pastAngle + angle)) / 2;

  derevative = (angle - pastAngle) / (startTime - previousTime);
  dGain = (derevative + pastDerevative + pastDerevative2) / 3;

  outputGain = (Kp * angle) + (Ki * integral) + (Kd * dGain);

  if (angle > -65 && angle < 65)
  {
    if (outputGain > -256 && outputGain < 256)
    {
      motor.controlPWM(outputGain);

      // Serial.print(angle);
      // Serial.print("\t\t");

      // Serial.print(Kp * angle);
      // Serial.print("\t\t");

      // Serial.print(Ki * integral);
      // Serial.print("\t\t");

      // Serial.print(Kd * derevative);
      // Serial.print("\t\t");

      // Serial.println(outputGain);
      Serial.print("angle: ");
      Serial.print(angle);

      // Serial.print("\tpast angle: ");
      // Serial.print(pastAngle);

      // Serial.print("\tprevious time: ");
      // Serial.print(previousTime);

      // Serial.print("\tstart time: ");
      // Serial.print(startTime);

      Serial.print("\tD gain: ");
      Serial.println(Kd * derevative);
    }

    else
    {
      if (outputGain > 0)
      {
        outputGain = 256;
      }
      else
      {
        outputGain = -256;
      }

      motor.controlPWM(outputGain);

      // Serial.print(angle);
      // Serial.print("\t\t");

      // Serial.print(Kp * angle);
      // Serial.print("\t\t");

      // Serial.print(Ki * integral);
      // Serial.print("\t\t");

      // Serial.print(Kd * derevative);
      // Serial.print("\t\t");

      // Serial.println(outputGain);
    }
  }

  else
  {
    motor.disable();
    Serial.println("--------------ERROR--------------");
    delay(100000);
  }

  previousTime = startTime;
  pastAngle = angle;
  pastDerevative2 = pastDerevative;
  pastDerevative = derevative;
}

// int convertRawAngleToDegrees(word newAngle)
// {
//   /* Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree */
//   float retVal = newAngle * 0.087890625;
//   return retVal;
// }

float func(int angle)
{
  int a = 0;
  float b = 0;

  if (startDeg > 2047)
  {
    a = startDeg - 2047;
    if (angle >= a && angle <= 4095)
    {
      b = angle - startDeg;
    }

    else if (angle < a)
    {
      b = 4095 - startDeg + angle;
    }
  }
  else if (startDeg < 2048)
  {
    a = startDeg + 2048;

    if (angle >= 0 && angle <= a)
    {
      b = angle - startDeg;
    }

    else if (angle > a)
    {
      b = -startDeg - 4095 + angle;
    }
  }

  return (b * 0.087890625);
}