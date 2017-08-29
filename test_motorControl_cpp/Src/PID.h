/*
 * PID.h
 *
 *  Created on: 25 août 2017
 *      Author: Cécile
 */

#ifndef PID_H_
#define PID_H_

#define PID_TIME_SAMPLING 0.01

class PID {
public:
    PID(float Kp, float Ki, float Kd);
	virtual ~PID();//constructor
    PID();

    //methods
    float computePID(float error);
    bool isComputed();
    void resetPID();
    float getSpeedError();

    void setParam(float Kp, float Ki, float Kd);
    bool isComputed_;

    private :
    //attributs
    float Kp_;
    float Ki_;
    float Kd_;

    float accumulatedErrorIntegral_;
    float prevError_;
};

#endif /* PID_H_ */
