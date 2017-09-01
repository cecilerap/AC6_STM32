/*
 * WheelSide.cpp
 *
 *  Created on: 25 août 2017
 *      Author: Cécile
 */

#include "WheelSide.h"

/*WheelSide::WheelSide(GPIO_TypeDef* portSpeedFront, uint16_t pinSpeedFront, GPIO_TypeDef* portDirFront, uint16_t pinDirFront,
		GPIO_TypeDef* portAFront, uint16_t pinAFront, GPIO_TypeDef* portBFront, uint16_t pinBFront, TIM_HandleTypeDef *timerFront, __IO uint32_t channelFront,
		GPIO_TypeDef* portSpeedBack, uint16_t pinSpeedBack, GPIO_TypeDef* portDirBack, uint16_t pinDirBack,
		GPIO_TypeDef* portABack, uint16_t pinABack, GPIO_TypeDef* portBBack, uint16_t pinBBack, TIM_HandleTypeDef *timerBack, __IO uint32_t channelBack):
		frontWheel_(portSpeedFront, pinSpeedFront, portDirFront, pinDirFront,
				portAFront, pinAFront, portBFront, pinBFront, timerFront, channelFront),
		backWheel_(portSpeedBack, pinSpeedBack, portDirBack, pinDirBack,
				portABack, pinABack, portBBack, pinBBack, timerBack, channelBack)
{
	// TODO Auto-generated constructor stub

}*/

WheelSide::WheelSide(Wheel &frontWheel, Wheel &backWheel): frontWheel_(frontWheel), backWheel_(backWheel)
{


}

void WheelSide::countPulse()
{
	frontWheel_.countPulse();
	backWheel_.countPulse();
}

float WheelSide::getPulse()
{
    return ((float) frontWheel_.getPulse() + (float) backWheel_.getPulse())/2.0;
}

void WheelSide::run(float speed)
{
    frontWheel_.run(speed);
    backWheel_.run(speed);
}
void WheelSide::stop()
{
    frontWheel_.stop();
    backWheel_.stop();
}
float WheelSide::getSpeedMeas()
{
    return (frontWheel_.getSpeedMeas() + backWheel_.getSpeedMeas())/2;
}

WheelSide::~WheelSide() {
	// TODO Auto-generated destructor stub
}

