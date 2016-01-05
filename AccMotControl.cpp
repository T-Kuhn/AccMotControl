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
    mpu.setDLPFMode(MPU6050_DLPF_5);   
    pid.begin(250.0f, 0.00f, 150.0f);
    pid.setSetPoint(4.71f);
}

// - - - - - - - - - - - - - - - - - - -
// - - - AccMotControl UPDATE  - - - - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::updatePID()
{
    _cntr++;
    if(_cntr % 1000 == 0){
        pid.setSetPoint(0.78f);
    }
    if(_cntr % 2000 == 0){
        pid.setSetPoint(2.36f);
    }
    if(_cntr % 2999 == 0 || _cntr >= 3000){
        pid.setSetPoint(5.50f); 
        _cntr = 1;
    }
    Serial.println(_getRotAngle());
    analogWrite(_motorDriverPWMpin, _setRotDir(pid.update(_getRotAngle())));
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

// - - - - - - - - - - - - - - - - - - -
//  AccMotControl GET ROTATIONAL ANGLE -
// - - - - - - - - - - - - - - - - - - -
float AccMotControl::_getRotAngle()
{
    Vector _tmp;
    float _XAxis;
    float _YAxis;
    float _aTan;
    _tmp = mpu.readRawAccel();
    _XAxis = _tmp.XAxis;
    _YAxis = _tmp.YAxis;
    if(_YAxis == 0.0f){
        _YAxis = 0.000000001f;
    }
    _aTan = (float)atan(_XAxis / _YAxis);
    if(_YAxis >= 0.0f){
        return _aTan + PI; 
    }else if(_XAxis < 0.0f){
        return _aTan;
    }else if(_XAxis >= 0.0f){
        return _aTan + 2*PI;
    }
    return 0.0f;
}



