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
        void update();
        void move(float goalPos, unsigned int moveTime, unsigned int moveSlopeTime);
    private:
        void _updatePID();
        int _setRotDir(int val);
        float _getRotAngle();
        void _calculatePathVars();
        void _followPath();
        void _followStartSlope();
        void _followStraightLine();
        void _followEndSlope();
        void _setPointSetter();
        void _moveFunctionCaller();
        int _motorDriverIN1pin;
        int _motorDriverIN2pin;
        int _motorDriverPWMpin; 
        int _cntr;
        bool _pathFollowing;
        float _goalPos;
        float _startPos;
        unsigned long _startTime;
        unsigned long _currentTime;
        unsigned int _passedTime;
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














