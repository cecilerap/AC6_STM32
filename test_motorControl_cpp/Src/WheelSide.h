/*
 * WheelSide.h
 *
 *  Created on: 25 août 2017
 *      Author: Cécile
 */

#ifndef WHEELSIDE_H_
#define WHEELSIDE_H_

#include "stm32f7xx_hal.h"
#include "Wheel.h"

class WheelSide {
public:

	WheelSide(Wheel &frontWheel, Wheel &backWheel);
	virtual ~WheelSide();
    //methods
    float getPulse();
    void run(float speed);
    void stop();
    float getSpeedMeas();
    void countPulse();

    private:
    Wheel &frontWheel_;
    Wheel &backWheel_;
};

#endif /* WHEELSIDE_H_ */
