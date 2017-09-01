/*
 * Robot.cpp
 *
 *  Created on: 1 sept. 2017
 *      Author: Cécile
 */

#include "Robot.h"
#include "Math.h"

Robot::Robot(WheelSide &leftSide, WheelSide &rightSide) : leftSide_(leftSide), rightSide_(rightSide)
{
	// TODO Auto-generated constructor stub
	accLimit_ = 30;
	speedLimit_ = 1000;

	distanceBetweenWheels_ = 270;
	pulsesPerRev_ = 663;
	diameterWheel_ = 95;

	rightCommand_ = 0;
	leftCommand_ = 0;

	posReached_ = 0;

	speedLinearDesiredAf_ = 0;
	speedAngularDesiredAf_ = 0;
	speedLinearDesired_ = 0;
	speedAngularDesired_ = 0;
	prevLeftPulses_ = 0;
	prevRightPulses_ = 0;
	speedRightMeas_ = 0;
	speedLeftMeas_ = 0;
	leftPulse_ = 0;
	rightPulse_ = 0;

	linearPositionPID_.setParam(4,0.0005,6);
	angularPositionPID_.setParam(3,0,0);
	linearSpeedPID_.setParam(0.7,0,0);
	angularSpeedPID_.setParam(0.3,0,0);
}

void Robot::regulateMotion(float posDesired, float orientationDes)
{
    orientationDes = orientationDes * pulsesPerRev_ * distanceBetweenWheels_ /360 / diameterWheel_;


   leftPulse_ = leftSide_.getPulse();
   rightPulse_ = rightSide_.getPulse();
   float leftSpeed = leftSide_.getSpeedMeas();
   float rightSpeed = rightSide_.getSpeedMeas();
   speedRightMeas_ = rightSpeed;
   speedLeftMeas_ = leftSpeed;

   float linearPosition = (rightPulse_ + leftPulse_)/2;
   float angularPosition = rightPulse_ - leftPulse_;

   /* float errorDistance = posDesired - getLinearPosition();
    float errorAngle = orientationDes - getAngularPosition();*/
   float errorDistance = posDesired - linearPosition;
   float errorAngle = orientationDes - angularPosition;

    if(errorDistance < 20) //orientation correction, position is reached
    {
        errorDistance = 0;
        linearPositionPID_.resetPID();

        if(fabs(errorAngle) < 10)
        {
            errorAngle = 0;
            angularPositionPID_.resetPID();
            posReached_ = 1;
        }
    }

    float speedLinearDesired = linearPositionPID_.computePID(errorDistance);
    float speedAngularDesired = angularPositionPID_.computePID(errorAngle);
    speedLinearDesired_ = speedLinearDesired;

    if(linearPositionPID_.isComputed() && angularPositionPID_.isComputed())
    {
    	speedLinearDesired_ = speedLinearDesired;
        speedAngularDesired_ = speedAngularDesired;

        linearPositionPID_.isComputed_ = false;
        angularPositionPID_.isComputed_ = false;

      /*  speedLinearDesiredAf_ = limitSpeed(speedLinearDesired_, getLinearSpeed());
        speedAngularDesiredAf_ = limitSpeed(speedAngularDesired_, getAngularSpeed());*/
	    float  linearSpeed = (rightSpeed + leftSpeed)/2;
	    float   angularSpeed = rightSpeed - leftSpeed;
		speedLinearDesiredAf_ = limitSpeed(speedLinearDesired_, linearSpeed);
        speedAngularDesiredAf_ = limitSpeed(speedAngularDesired_, angularSpeed);

    /*   float errorLinearSpeed = speedLinearDesiredAf_ - getLinearSpeed();
        float errorAngularSpeed = speedAngularDesiredAf_ - getAngularSpeed();*/
        float errorLinearSpeed = speedLinearDesiredAf_ - linearSpeed;
		float errorAngularSpeed = speedAngularDesiredAf_ - angularSpeed;

        float commandLinear = linearSpeedPID_.computePID(errorLinearSpeed);
        float commandAngular = angularSpeedPID_.computePID(errorAngularSpeed);

        if(linearSpeedPID_.isComputed() && angularSpeedPID_.isComputed())
        {
            linearSpeedPID_.isComputed_ = false;
            angularSpeedPID_.isComputed_ = false;

            rightCommand_ = commandLinear + commandAngular;
            leftCommand_ = commandLinear - commandAngular;

            rightSide_.run(rightCommand_);
            leftSide_.run(leftCommand_);
        }
    }
}

void Robot::run()
{
    rightSide_.run(rightCommand_);
    leftSide_.run(leftCommand_);
}

float Robot::limitSpeed(float command, float speedMeas)
{ //  printf("%.2f\t",speedMeas);
    float acc = command - speedMeas;
    if (acc > accLimit_)
    {
      //printf("acc > accL\t");
      command = speedMeas + accLimit_;
    }
    if (acc < -accLimit_)
    {
      //printf("acc < accL\t");
      command = speedMeas - accLimit_;
    }
    if (command > speedLimit_)
    {
      //printf("cmd > spL\t");
      command = speedLimit_;
    }
    if (command < -speedLimit_)
    {
      //printf("cmd < spL\t");
      command = -speedLimit_;
    }
    return command;
}

void Robot::stop()
{
    rightSide_.stop();
    leftSide_.stop();
}

float Robot::getLinearPosition()    //p_robot
{
    return ((getLeftPulse() + getRightPulse())/2);
}

float Robot::getAngularPosition()   //angle_robot
{
    return (getRightPulse() - getLeftPulse());
}

float Robot::getLinearSpeed()       //v_robot
{
    return ((getLeftSpeed() + getRightSpeed())/2);
}

float Robot::getAngularSpeed()      //w_robot
{
    return getRightSpeed() - getLeftSpeed();
}


float Robot::getRightSpeed()
{
    return rightSide_.getSpeedMeas();
    //return speedRightMeas_;
}
float Robot::getLeftSpeed()
{
   // return speedLeftMeas_;
    return leftSide_.getSpeedMeas();
}

float Robot::getRightPulse()
{
    return rightSide_.getPulse();
}
float Robot::getLeftPulse()
{
    return leftSide_.getPulse();
}

bool Robot::getStatusMotion()
{
    return posReached_;
}

float Robot::posToPulse(float pos)
{
    return (pos / (float) PI / diameterWheel_ * pulsesPerRev_);
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

