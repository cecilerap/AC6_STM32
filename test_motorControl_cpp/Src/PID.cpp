/*
 * PID.cpp
 *
 *  Created on: 25 août 2017
 *      Author: Cécile
 */

#include "stm32f7xx_hal.h"
#include "PID.h"
#include <stdlib.h>

//constructor
PID::PID()
{
	Kp_ = 0;
	Ki_ = 0;
	Kd_ = 0;
    accumulatedErrorIntegral_ = 0;
    prevError_ = 0.0;
    isComputed_ = false;
}

PID::PID(float Kp, float Ki, float Kd)
{
    setParam(Kp, Ki, Kd);

    accumulatedErrorIntegral_ = 0;
    prevError_ = 0;

    isComputed_ = false;
}


//methods
//___________________________set PID param
void PID::setParam(float Kp, float Ki, float Kd)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}


float PID::computePID(float error)
{
    isComputed_ = false;
    uint32_t timer = HAL_GetTick();

    //if(timer_.read() > PID_TIME_SAMPLING)
    {
        isComputed_ = true;
        timer = 0;
        accumulatedErrorIntegral_ += error;
        float dError = error - prevError_;
        float PID_output = Kp_ * error + Ki_ * accumulatedErrorIntegral_ + Kd_ * dError;

        //Remember the input for the derivative calculation next time.
        prevError_  = error;

        return PID_output;
    }
    return 0.0;
}

bool PID::isComputed()
{
    return isComputed_;
}

void PID::resetPID()
{
    accumulatedErrorIntegral_ = 0;
    prevError_ = 0;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

