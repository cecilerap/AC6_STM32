/*
 * Wheel.cpp
 *
 *  Created on: 25 août 2017
 *      Author: Cécile
 */

#include "Wheel.h"
#include "PID.h"
#include "Math.h"

#ifndef PI
   #define PI 3.14159265358979323846
#endif
/*
Wheel::Wheel(GPIO_TypeDef* portSpeed, uint16_t pinSpeed, GPIO_TypeDef* portDir, uint16_t pinDir,
		GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB)
{
	// TODO Auto-generated constructor stub

	portSpeed_ = portSpeed;
	pinSpeed_ = pinSpeed;
	portDir_ = portDir;
	pinDir_ = pinDir;
	portA_ = portA;
	pinA_ = pinA;
	portB_ = portB;
	pinB_ = pinB;

    pulses_ = 0;
    prevPulses_ = 0;
}*/
Wheel::Wheel(GPIO_TypeDef* portSpeed, uint16_t pinSpeed, GPIO_TypeDef* portDir, uint16_t pinDir, GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB):
		portSpeed_(portSpeed), portDir_(portDir), pinDir_(pinDir), portA_(portA), pinA_(pinA), portB_(portB), pinB_(pinB)
{
	// TODO Auto-generated constructor stub

	portSpeed_ = portSpeed;
	pinSpeed_ = pinSpeed;
	portDir_ = portDir;
	pinDir_ = pinDir;
	portA_ = portA;
	pinA_ = pinA;
	portB_ = portB;
	pinB_ = pinB;

    pulses_ = 0;
    prevPulses_ = 0;
}
//methods
void Wheel::run(float speed)
{
    if(fabs(speed)>100)
    {
       speed = sign(speed) * 100.0f;
    }

    //HAL_GPIO_WritePin(portSpeed_, pinSpeed_, fabs(speed/100));

    if(speed>0)
    {
    	HAL_GPIO_WritePin(portDir_, pinDir_, GPIO_PIN_SET);
    }
    else
    {
    	HAL_GPIO_WritePin(portDir_, pinDir_, GPIO_PIN_RESET);
    }
   // directionPin_ = speed>0;
}

void Wheel::stop()
{
    HAL_GPIO_WritePin(portSpeed_, pinSpeed_, GPIO_PIN_RESET);
}

float Wheel::getSpeedMeas()
{
    float speedMeas = ((float) pulses_ - (float)prevPulses_)/PID_TIME_SAMPLING;
    prevPulses_ = pulses_;
    return speedMeas;
}

//encoder
void Wheel::countPulse()
{
    //int chanB  = encPinB_.read();
    uint8_t chanB = HAL_GPIO_ReadPin(portB_, pinB_);
    if(chanB==0)
    {
        pulses_++;
    }
    else
    {
        pulses_--;
    }
}

int Wheel::getPulse()
{
    return pulses_;
}

float Wheel::sign(float val)
{
    return val > 0 ? 1.0 : -1.0 ;
}
Wheel::~Wheel() {
	// TODO Auto-generated destructor stub
}

