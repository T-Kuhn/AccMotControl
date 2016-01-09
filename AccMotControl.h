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
        AccMotControl(int in1Pin, int in2Pin, int pwmPin, int mpuAdd);
        MPU6050 mpu;
        PID pid;
        void begin(float pos1, float pos2, float pos3);
        void update();
        void setMode(int nmbr);
        void move(float goalPos, unsigned int moveTime, unsigned int moveSlopeTime);
    private:
        void _updatePID();
        int _setRotDir(int val);
        float _getRotAngle();
        void _calculatePathVars1();
        void _calculatePathVars2();
        void _followPath();
        void _followStartSlope();
        void _followStraightLine();
        void _followEndSlope();
        void _setPointSetter();
        void _moveFunctionCaller();
        float _pos1;
        float _pos2;
        float _pos3;
        int _mpuAdd;
        int _motorDriverIN1pin;
        int _motorDriverIN2pin;
        int _motorDriverPWMpin; 
        int _cntr;
        int _mode;
        bool _pathFollowing;
        float _goalPos;
        float _startPos;
        unsigned long _startTime;
        unsigned long _currentTime;
        unsigned long _passedTime;
        unsigned int _moveTime;
        unsigned int _moveSlopeTime;
        unsigned int _moveStraightTime;
        float _distance;
        float _maxSpeed;
        float _slope;
        float _startSlopeEndPos;
        float _straightMoveEndPos;
};

#endif














