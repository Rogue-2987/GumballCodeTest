/*
 * rrPID.h
 *
 *  Created on: Feb 7, 2017
 *      Author: Corey
 */

#ifndef RRPID_H_
#define RRPID_H_

#include "WPILib.h"
#include <string.h>

class rrPID {
public:
	float kp = 0.03;
	float ki = 0.001;
	float target = 0;
	float actual = 0;
	float errorSignal = 0;
	float integrationTotal = 0;
	float intLim = 0.15;
	float outputLim = 0.5;
	float outputCommand = 0;
	int IDnum = 1;


	rrPID();
	virtual ~rrPID();
	void init(int newIDnum, float newKp, float newKi);
	void setKpKi(float newKp, float newKi);
	void setLimits(float newIntLim, float newOutputLim);
	float controlOutput(float curTarget, float curActual);
	float motorOutput();
	float debug();
};

#endif /* RRPID_H_ */
