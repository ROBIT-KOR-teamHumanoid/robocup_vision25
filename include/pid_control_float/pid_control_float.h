/*
 * pid_control_float.h
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#ifndef PID_CONTROL_FLOAT_H_
#define PID_CONTROL_FLOAT_H_

typedef struct _PID{
	double nowValue;		//!<
        double pastValue;

        double nowError;
        double pastError;
        double target;

        double errorSum;
        double errorSumLimit;
        double errorDiff;

        double nowOutput;
        double pastOutput;
        double outputLimit;

        double underOfPoint;

        double kP;
        double kI;
        double kD;
}PID;

void PID_Control_init(PID* temp,double TP, double TI, double TD ,double ELIMIT, double OLIMIT);
void PID_Control_Float(PID* dst, double target, double input);



#endif /* PID_CONTROL_FLOAT_H_ */
