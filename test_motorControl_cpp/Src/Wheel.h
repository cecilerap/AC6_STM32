/*
 * Wheel.h
 *
 *  Created on: 25 août 2017
 *      Author: Cécile
 */

#ifndef WHEEL_H_
#define WHEEL_H_

#include "stm32f7xx_hal.h"
#include "PID.h"

class Wheel {
public:
    //constructor
	Wheel(GPIO_TypeDef* portSpeed, uint16_t pinSpeed, GPIO_TypeDef* portDir, uint16_t pinDir,
			GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB, TIM_HandleTypeDef *timer, __IO uint32_t channel);

	virtual ~Wheel();

    //methods
    void countPulse();
    int getPulse();
    void run(float speed);
    float getSpeedMeas();
    void stop();
    float sign(float val);

private:
    int pulses_;
    int prevPulses_;

    GPIO_TypeDef* portSpeed_;
    uint16_t pinSpeed_;
    GPIO_TypeDef* portDir_;
    uint16_t pinDir_;
    GPIO_TypeDef* portA_;
	uint16_t pinA_;
	GPIO_TypeDef* portB_;
	uint16_t pinB_;

	__IO uint32_t channel_;
	TIM_HandleTypeDef * timer_;
};

#endif /* WHEEL_H_ */
