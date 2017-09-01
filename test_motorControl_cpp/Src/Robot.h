/*
 * Robot.h
 *
 *  Created on: 1 sept. 2017
 *      Author: Cécile
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "WheelSide.h"

#ifndef PI
   #define PI 3.14159265358979323846
#endif

class Robot {
public:

	Robot(WheelSide &leftSide, WheelSide &rightSide);
	virtual ~Robot();
	void regulateMotion(float posDesired, float orientationPos);

	void run();
	void stop();
	float limitSpeed(float command, float speedMeasured);
	float posToPulse(float pos);
	float getLinearPosition();
	float getAngularPosition();
	float getLinearSpeed() ;
	float getAngularSpeed();

	bool getStatusMotion();

	float speedLinearDesiredAf_;
	float speedAngularDesiredAf_;
	float getRightPulse();
	float getLeftPulse();
	float speedLinearDesired_;
	float speedAngularDesired_;

	float rightCommand_;
	float leftCommand_;

	float getRightSpeed();
	float getLeftSpeed();
	float speedRightMeas_;
	float speedLeftMeas_;
		float leftPulse_;
	float rightPulse_;

	private:
	float accLimit_;
	float speedLimit_;

	float distanceBetweenWheels_;
	float pulsesPerRev_;
	float diameterWheel_;

	bool posReached_;

	PID linearPositionPID_;
	PID angularPositionPID_;
	PID linearSpeedPID_;
	PID angularSpeedPID_;

	WheelSide &leftSide_;
	WheelSide &rightSide_;

	float prevLeftPulses_;
	float prevRightPulses_;


};

#endif /* ROBOT_H_ */
