/*
  AccMotControl.cpp - A DC motor (geared) position controller using
  only data from a MPU-6050 accelerometer as position feedback 
  Created by Tobias Kuhn. Sapporo, December 28, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include <Wire.h>
#include <math.h>
#include "MPU6050.h"
#include "AccMotControl.h"

// - - - - - - - - - - - - - - - - - - -
// - - - AccMotControl CONSTRUCTOR - - -
// - - - - - - - - - - - - - - - - - - -
AccMotControl::AccMotControl(int in1Pin, int in2Pin, int pwmPin)
{
    _motorDriverPWMpin = pwmPin;
    _motorDriverIN1pin = in1Pin;
    _motorDriverIN2pin = in2Pin;
}

// - - - - - - - - - - - - - - - - - - -
// - - - AccMotControl BEGIN - - - - - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::begin()
{
    pinMode(_motorDriverPWMpin, OUTPUT);
    pinMode(_motorDriverIN1pin, OUTPUT);
    pinMode(_motorDriverIN2pin, OUTPUT);
    mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
    pid.begin(20.0f, 0.1f, 10.0f);
    pid.setSetPoint(4.5f);
}

// - - - - - - - - - - - - - - - - - - -
// - - - AccMotControl UPDATE  - - - - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::update()
{
    _cntr++;
    if(_cntr % 100 == 0){
        pid.setSetPoint(0.0f);
    }
    if(_cntr % 199 == 0){
        pid.setSetPoint(4.5f);
        _cntr = 1;
    }
    Vector tmp;
    tmp = mpu.readNormalizeAccel();
    Serial.print(" | AcY = "); Serial.println(tmp.YAxis);
    analogWrite(_motorDriverPWMpin, _setRotDir(pid.update(tmp.YAxis)));
}

// - - - - - - - - - - - - - - - - - - -
// - AccMotControl SET ROT DIRECTION - -
// - - - - - - - - - - - - - - - - - - -
int AccMotControl::_setRotDir(int val)
{
    if(val > 0){
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, LOW);
    }else{
        digitalWrite(_motorDriverIN1pin, LOW);
        digitalWrite(_motorDriverIN2pin, HIGH);
    }
    return abs(val); 
}



