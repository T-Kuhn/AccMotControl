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
AccMotControl::AccMotControl(int in1Pin, int in2Pin, int pwmPin, int mpuAdd)
{
    _motorDriverPWMpin = pwmPin;
    _motorDriverIN1pin = in1Pin;
    _motorDriverIN2pin = in2Pin;
    _mpuAdd = mpuAdd;
}

// - - - - - - - - - - - - - - - - - - -
// - - - AccMotControl BEGIN - - - - - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::begin(float pos1, float pos2, float pos3)
{
    _pos1 = pos1;
    _pos2 = pos2;
    _pos3 = pos3;
    pinMode(_motorDriverPWMpin, OUTPUT);
    pinMode(_motorDriverIN1pin, OUTPUT);
    pinMode(_motorDriverIN2pin, OUTPUT);
    mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, _mpuAdd);
    mpu.setDLPFMode(MPU6050_DLPF_4);   
    _pathFollowing = false;
    pid.begin(330.0f, 0.4f, 400.0f);
    pid.setSetPoint(4.71f);
    setMode(1);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - AccMotControl UPDATE  - - - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::update()
{
    if(_pathFollowing){
        _followPath();
    }
    _updatePID();
}

// - - - - - - - - - - - - - - - - - - -
// - - AccMotControl UPDATE PID  - - - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::_updatePID()
{
    //_setPointSetter();
    _moveFunctionCaller();
    //Serial.println(_getRotAngle());
    analogWrite(_motorDriverPWMpin, _setRotDir(pid.update(_getRotAngle(), _pathFollowing)));
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

// - - - - - - - - - - - - - - - - - - -
// - - - - AccMotControl MOVE  - - - - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::move(float goalPos, unsigned int moveTime, unsigned int moveSlopeTime)
{
    _pathFollowing = true;
    _goalPos = goalPos;
    _moveTime = moveTime;
    _moveSlopeTime = moveSlopeTime;
    _startPos = _getRotAngle();
    _startTime = millis();
    if(_mode == 1){
        _calculatePathVars1();
    }else{
        _calculatePathVars2();
    }
}

// - - - - - - - - - - - - - - - - - - - - - 
// - AccMotControl CALCULATE PATHVARS  1 - -
// - - - - - - - - - - - - - - - - - - - - -
void AccMotControl::_calculatePathVars1()
{
    _distance = _goalPos - _startPos; // can be minus
    if(2 * _moveSlopeTime < _moveTime){
        _moveStraightTime = _moveTime - 2 * _moveSlopeTime;  
    }else{
        _moveStraightTime = 0;
    }
    _maxSpeed = _distance / (_moveSlopeTime + _moveStraightTime);
    _slope = _maxSpeed / _moveSlopeTime;
    
    _startSlopeEndPos = _maxSpeed / 2 * _moveSlopeTime;
    _straightMoveEndPos = _startSlopeEndPos + _maxSpeed * _moveStraightTime;
}

// - - - - - - - - - - - - - - - - - - - - -
// - AccMotControl CALCULATE PATHVARS  2 - -
// - - - - - - - - - - - - - - - - - - - - -
void AccMotControl::_calculatePathVars2()
{
    _distance = _goalPos - _startPos; // can be minus
    if(2 * _moveSlopeTime < _moveTime){
        _moveStraightTime = _moveTime - 2 * _moveSlopeTime;  
    }else{
        _moveStraightTime = 0;
    }
    _maxSpeed = _distance / (_moveSlopeTime + _moveStraightTime);
    _slope = _maxSpeed / _moveSlopeTime;
    
    _startSlopeEndPos = _maxSpeed / 2 * _moveSlopeTime;
    _straightMoveEndPos = _startSlopeEndPos + _maxSpeed * _moveStraightTime;
}

// - - - - - - - - - - - - - - - - - - -
// - - - AccMotControl FOLLOW PATH - - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::_followPath()
{
    _currentTime = millis();
    _passedTime = _currentTime - _startTime;
    if(_moveSlopeTime >= _passedTime){
        _followStartSlope();
        return;
    }else if(_moveSlopeTime + _moveStraightTime >= _passedTime){
        _followStraightLine();
        return;
    }else if(2 * _moveSlopeTime + _moveStraightTime >= _passedTime){
        _followEndSlope();
        return;
    }else{
        pid.setSetPoint(_goalPos);
        _pathFollowing = false; // in the END...
    }
}

// - - - - - - - - - - - - - - - - - - -
// - AccMotControl FOLLOW START SLOPE  -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::_followStartSlope()
{
    if(_mode == 1){
        pid.setSetPoint(_startPos + _passedTime * _passedTime * _slope / 2);
    }else{
        pid.setSetPoint(_startPos + ( sin(-PI/2 + (float)_passedTime / _moveSlopeTime * PI) 
                + 1 ) / 4 * _slope * _passedTime * _passedTime);
    }
}

// - - - - - - - - - - - - - - - - - - -
//  AccMotControl FOLLOW STRAIGHT LINE -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::_followStraightLine()
{
    pid.setSetPoint(_startPos + _startSlopeEndPos + (_passedTime - _moveSlopeTime) * _maxSpeed);    
}

// - - - - - - - - - - - - - - - - - - -
// - AccMotControl FOLLOW END SLOPE  - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::_followEndSlope()
{
    unsigned int passedTimeSinceStartEndSlope = _passedTime - _moveSlopeTime - _moveStraightTime;
    if(_mode == 1){
        pid.setSetPoint(_startPos + _straightMoveEndPos
                + (_maxSpeed - _slope / 2 * passedTimeSinceStartEndSlope) 
                * passedTimeSinceStartEndSlope); 
    }else{
        pid.setSetPoint(_startPos + _straightMoveEndPos
                + (_maxSpeed - (sin( -PI/2 + (float)passedTimeSinceStartEndSlope / _moveSlopeTime * PI) 
                + 1 ) / 2 * _slope / 2 * passedTimeSinceStartEndSlope)
                * passedTimeSinceStartEndSlope);
    }
}

// - - - - - - - - - - - - - - - - - - -
// - AccMotControl FOLLOW END SLOPE  - -
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::setMode(int nmbr)
{
    // nmbr ->  1:  linear speedgain
    //          2:  S-curveed speedgain
    _mode = nmbr;
}

// - - - - - - - - - - - - - - - - - - -
// - - - SET POINT SETTER [DEBUG]  - - -  
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::_setPointSetter()
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
}

// - - - - - - - - - - - - - - - - - - -
// - - MOVE FUNCTION CALLER [DEBUG]  - -  
// - - - - - - - - - - - - - - - - - - -
void AccMotControl::_moveFunctionCaller()
{
    _cntr++;
    if(_cntr % 500 == 0){
        move(_pos1, 500, 200);
    }
    if(_cntr % 1000 == 0){
        move(_pos2, 500, 200);
    }
    if(_cntr % 1499 == 0 || _cntr >= 1500){
        move(_pos3, 500, 200); 
        _cntr = 1;
    }
}

