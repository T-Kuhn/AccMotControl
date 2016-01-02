/*
  AccMotControl.h - A DC motor (geared) position controller using
  only data from a MPU-6050 accelerometer as position feedback 
  Created by Tobias Kuhn. Sapporo, December 28, 2015.
  Released into the public domain.
*/

#ifndef AccMotControl_h
#define AccMotControl_h
#include "Arduino.h"
#include "MPU6050.h"
#include "PID.h"

// - - - - - - - - - - - - - - - - - - -
// - - - - AccMotControl CLASS - - - - -
// - - - - - - - - - - - - - - - - - - -
class AccMotControl
{
    public:
        AccMotControl(int in1Pin, int in2Pin, int pwmPin);
        MPU6050 mpu;
        PID pid;
        void begin();
        void updatePID();
    private:
        int _motorDriverIN1pin;
        int _motorDriverIN2pin;
        int _motorDriverPWMpin; 
        int _setRotDir(int val);
        int _cntr;
};

#endif














