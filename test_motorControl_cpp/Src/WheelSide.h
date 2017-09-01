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
	/*WheelSide(GPIO_TypeDef* portSpeedFront, uint16_t pinSpeedFront, GPIO_TypeDef* portDirFront, uint16_t pinDirFront,
			GPIO_TypeDef* portAFront, uint16_t pinAFront, GPIO_TypeDef* portBFront, uint16_t pinBFront, TIM_HandleTypeDef *timerFront, __IO uint32_t channelFront,
			GPIO_TypeDef* portSpeedBack, uint16_t pinSpeedBack, GPIO_TypeDef* portDirBack, uint16_t pinDirBack,
			GPIO_TypeDef* portABack, uint16_t pinABack, GPIO_TypeDef* portBBack, uint16_t pinBBack, TIM_HandleTypeDef *timerBack, __IO uint32_t channelBack);*/

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
